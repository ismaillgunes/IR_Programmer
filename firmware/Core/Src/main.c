/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : IR + SWD Programmer - ana program
  *
  *  IR_Programmer_PPM karti (STM32L073VBT) uzerinde calisir. fw_image.h icindeki
  *  firmware'i hedef STM32L0'a iki yoldan biriyle yazar:
  *
  *    - IR  : SW3 / BUTTON_2 (PA12) -> hedefin sistem bootloader'i (AN3155),
  *            LPUART1 (PD8 = TX, PD9 = RX)
  *    - SWD : SW1 / BUTTON_1 (PC3)  -> bit-bang SWD (SWD/swd.c), J1 konnektoru
  *            (PA2 = SWCLK, PA1 = SWDIO, PA0 = NRST)
  *    - Log    : USART1 (PB6 = TX, PB7 = RX) <- J3, PC'ye 115200 8N1 log
  *    - Buzzer : PD12 (Q3) - kartta LCD yok, geri bildirim bip ile
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fw_image.h"      /* hedef firmware: fw_data[], fw_size (IR ve SWD ortak) */
#include "swd.h"
#include "sys_conf.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- STM32 sistem bootloader (AN3155) sabitleri --- */
#define BL_INIT             0x7FU
#define BL_ACK              0x79U
#define BL_NACK             0x1FU
#define TARGET_FLASH_BASE   0x08000000U   /* hedef firmware'in yazilacagi adres */
#define BL_CHUNK            256U          /* Write Memory blok boyutu (maks 256) */
/* IR baud. Bootloader ilk 0x7F'te buna kilitlenir; degistirirsen hedefi
   power-cycle et. Link kararsizsa 57600 / 38400 dene (CubeProgrammer 115200). */
#define IR_BAUDRATE         115200U
#define TARGET_PAGE_SIZE    128U          /* STM32L0 flash sayfa boyutu */
#define LOG_BAUDRATE        115200U

/* IR akisinda hedefin BOOT0 hatti yok: hedef elle power-cycle ile
 * bootloader'a alinir, programlama sonunda 'Go' komutuyla uygulamaya atlanir.
 * SWD akisi ise hedefi J1'deki NRST ile kendisi resetler ve durdurur.          */

/* SWD bit-bang yarim periyot gecikmesi (0 = en hizli). Uzun/parazitli kabloda
   ERR_PARITY / ERR_ACK / ERR_VERIFY gorulurse 2..6'ya cikar. */
#define SWD_CLK_DELAY       0U

/* Hata kodlari (Ui_Error bu sayida kisa bip calar)
 *   IR : -1 baglanti, -2 Get ID, -3 erase, -4 yazma
 *   SWD: -5 attach (IDCODE/halt), -6 flash unlock, -7 erase/yazma, -8 verify */
#define ERR_SWD_ATTACH      (-5)
#define ERR_SWD_UNLOCK      (-6)
#define ERR_SWD_PROGRAM     (-7)
#define ERR_SWD_VERIFY      (-8)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile int g_prog_result = -100;   /* 0 = basari; debug'ta izlemek icin */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void     Log(const char *fmt, ...);
static void     Ui_Begin(void);
static void     Ui_Ready(void);
static void     Ui_Success(void);
static void     Ui_Error(int code);
static int      Button_PressEdge(GPIO_TypeDef *port, uint16_t pin, int *prev);
static int      bl_set_baud(uint32_t baud);
static int      bl_connect(void);
static int      bl_get_id(uint16_t *pid);
static int      bl_erase_range(uint16_t first_page, uint16_t count);
static int      bl_write(uint32_t addr, const uint8_t *data, uint16_t len);
static int      bl_go(uint32_t addr);
int             Program_Target(void);
int             Program_Target_SWD(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================
 *  Buzzer (PD12 -> Q3) - aktif buzzer, ms suresince oter
 * ============================================================ */
void Buzzer(uint16_t ms)
{
  HAL_GPIO_WritePin(Buzzer_Pin_GPIO_Port, Buzzer_Pin_Pin, GPIO_PIN_SET);
  HAL_Delay(ms);
  HAL_GPIO_WritePin(Buzzer_Pin_GPIO_Port, Buzzer_Pin_Pin, GPIO_PIN_RESET);
}

/* ============================================================
 *  Mesgul bipi - programlama surerken arka planda periyodik kisa bip.
 *  SysTick kesmesinden (1 ms) Buzzer_Tick() ile surulur; ana akisi
 *  (IR UART / SWD bit-bang) hic bekletmez.
 * ============================================================ */
#define BUSY_BEEP_PERIOD_MS   1000U   /* bip araligi */
#define BUSY_BEEP_ON_MS       40U     /* bip suresi  */

static volatile uint8_t  s_busy_beep = 0;
static volatile uint16_t s_busy_ms   = 0;

static void BusyBeep_Start(void)
{
  s_busy_ms   = 0;
  s_busy_beep = 1;
}

static void BusyBeep_Stop(void)
{
  s_busy_beep = 0;
  HAL_GPIO_WritePin(Buzzer_Pin_GPIO_Port, Buzzer_Pin_Pin, GPIO_PIN_RESET);
}

/* SysTick_Handler'dan her 1 ms'de cagrilir */
void Buzzer_Tick(void)
{
  if (!s_busy_beep) return;
  uint16_t t = s_busy_ms;
  if (t == 0U)
    HAL_GPIO_WritePin(Buzzer_Pin_GPIO_Port, Buzzer_Pin_Pin, GPIO_PIN_SET);
  else if (t == BUSY_BEEP_ON_MS)
    HAL_GPIO_WritePin(Buzzer_Pin_GPIO_Port, Buzzer_Pin_Pin, GPIO_PIN_RESET);
  s_busy_ms = (uint16_t)((t + 1U >= BUSY_BEEP_PERIOD_MS) ? 0U : (t + 1U));
}

/* ============================================================
 *  Geri bildirim yardimcilari (kartta ekran yok -> sadece buzzer)
 * ============================================================ */
static void Ui_Begin(void)            /* programlama basliyor: tek kisa bip */
{
  Buzzer(60);
}

static void Ui_Ready(void)            /* hazir: menu tusuna basilmasini bekliyor */
{
  Buzzer(60);
}

static void Ui_Success(void)          /* basarili: iki kisa bip */
{
  Buzzer(80); HAL_Delay(120); Buzzer(80);
}

static void Ui_Error(int code)        /* hata: uzun bip + |code| adet kisa bip */
{
  int c = (code < 0) ? -code : code;
  Buzzer(600);
  HAL_Delay(400);
  for (int i = 0; i < c && i < 9; i++) {
    Buzzer(100); HAL_Delay(200);
  }
}

/* ============================================================
 *  Butonlar (aktif HIGH) - basma kenari yakalama (debounce'lu)
 * ============================================================ */
/* serbest->basili gecisini dondurur. Programlama sirasinda CAGRILMADIGI icin
   o sirada basilan tuslar dogal olarak yok sayilir; programlama bittikten sonra
   ancak YENI bir basma (birak + tekrar bas) yeni cevrimi tetikler.
   *prev boot'ta 1 verilir -> acilista basili tus yanlislikla tetiklemez. */
static int Button_PressEdge(GPIO_TypeDef *port, uint16_t pin, int *prev)
{
  int now = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET);
  int edge = (now && !*prev);
  *prev = now;
  if (edge) HAL_Delay(30);    /* debounce */
  return edge;
}

/* printf tarzi log (USART1 / J3): Log("deger=%d\r\n", x); */
static void Log(const char *fmt, ...)
{
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(buf)) n = (int)sizeof(buf);   /* tasma korumasi */
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 1000);
}

/* ============================================================
 *  Dusuk seviye IR/UART yardimcilari (LPUART1)
 * ============================================================ */
static int bl_recv(uint8_t *d, uint16_t n, uint32_t to)
{
  return (HAL_UART_Receive(&hlpuart1, d, n, to) == HAL_OK) ? 0 : -1;
}
static void bl_send(const uint8_t *d, uint16_t n)
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)d, n, 2000);
}
static int bl_wait_ack(uint32_t to)
{
  uint8_t b;
  if (bl_recv(&b, 1, to)) return -1;        /* timeout       */
  return (b == BL_ACK) ? 0 : -2;            /* NACK / hatali */
}
/* RX'te bekleyen artik/echo baytlarini ve hata bayraklarini temizle */
static void bl_flush(void)
{
  uint8_t junk;
  while (HAL_UART_Receive(&hlpuart1, &junk, 1, 5) == HAL_OK) { /* bosalt */ }
  __HAL_UART_CLEAR_OREFLAG(&hlpuart1);
  __HAL_UART_CLEAR_PEFLAG(&hlpuart1);
  __HAL_UART_CLEAR_FEFLAG(&hlpuart1);
  __HAL_UART_CLEAR_NEFLAG(&hlpuart1);
}

/* Komut baytini ve tumleyenini gonderir, ACK bekler */
static int bl_cmd(uint8_t cmd)
{
  uint8_t b[2] = { cmd, (uint8_t)(~cmd) };
  bl_flush();
  bl_send(b, 2);
  return bl_wait_ack(1000);
}

/* LPUART1 baud'unu yeniden ayarla */
static int bl_set_baud(uint32_t baud)
{
  hlpuart1.Init.BaudRate = baud;
  return (HAL_UART_Init(&hlpuart1) == HAL_OK) ? 0 : -1;
}

/* Tek bir 0x7F denemesi. 0 = ACK/NACK geldi (baglandi). */
static int bl_connect_once(void)
{
  uint8_t init = BL_INIT, b;
  HAL_StatusTypeDef s;

  __HAL_UART_CLEAR_OREFLAG(&hlpuart1);
  __HAL_UART_CLEAR_PEFLAG(&hlpuart1);
  __HAL_UART_CLEAR_FEFLAG(&hlpuart1);
  __HAL_UART_CLEAR_NEFLAG(&hlpuart1);

  bl_send(&init, 1);
  s = HAL_UART_Receive(&hlpuart1, &b, 1, 500);

  if (s == HAL_OK) {
    const char *tag = (b == BL_ACK)  ? " (ACK)"
                    : (b == BL_NACK) ? " (NACK - zaten init)"
                    : (b == BL_INIT) ? " (!! kendi echo'muz)"
                    : "";
    Log("        -> 0x%02X%s\r\n", b, tag);
    if (b == BL_ACK || b == BL_NACK) return 0;
  } else if (s == HAL_TIMEOUT) {
    Log("        -> cevap yok\r\n");
  } else {
    Log("        -> UART hata 0x%lX\r\n", (unsigned long)hlpuart1.ErrorCode);
  }
  return -1;
}

/* 0x7F autobaud. Hedef TAZE boot modunda olmali (elle power-cycle);
 * ilk 0x7F bootloader'i IR_BAUDRATE'e kilitler. */
static int bl_connect(void)
{
  bl_set_baud(IR_BAUDRATE);
  Log("    baud %lu...\r\n", (unsigned long)IR_BAUDRATE);
  for (int i = 0; i < 5; i++) {
    if (bl_connect_once() == 0) {
      Log("    >>> BAGLANDI @ %lu baud\r\n", (unsigned long)IR_BAUDRATE);
      return 0;
    }
    HAL_Delay(50);
  }
  return -1;
}

/* 0x02: Get ID -> hedef PID (urun kimligi). Marjinal link icin tekrarli. */
static int bl_get_id(uint16_t *pid)
{
  for (int attempt = 0; attempt < 5; attempt++) {
    uint8_t n, id[4];
    if (bl_cmd(0x02))                  { Log("    GetID d%d: cmd ACK yok\r\n", attempt + 1); continue; }
    if (bl_recv(&n, 1, 1000))          { Log("    GetID d%d: n yok\r\n",      attempt + 1); continue; }
    if ((int)n + 1 > (int)sizeof(id))  { Log("    GetID d%d: n=%u tuhaf\r\n",  attempt + 1, n); continue; }
    if (bl_recv(id, n + 1, 1000))      { Log("    GetID d%d: id yok\r\n",     attempt + 1); continue; }
    if (bl_wait_ack(1000))             { Log("    GetID d%d: son ACK yok\r\n", attempt + 1); continue; }
    *pid = (uint16_t)((id[0] << 8) | id[1]);
    return 0;
  }
  return -1;
}

/* 0x44: Extended Erase -> SAYFA erase. first_page'den baslayarak count sayfa siler.
 * CubeProgrammer'in yaptigi yontem (global 0xFFFF mass erase yerine). Tekrarli. */
static int bl_erase_range(uint16_t first_page, uint16_t count)
{
  if (count == 0) return 0;
  for (int attempt = 0; attempt < 4; attempt++) {
    uint8_t hdr[2], pb[2], cs, b;
    uint16_t nm1 = (uint16_t)(count - 1);  /* sayfa sayisi - 1 (MSB first) */

    if (bl_cmd(0x44)) { Log("    Erase[%u] d%d: cmd ACK yok\r\n", first_page, attempt + 1); continue; }

    hdr[0] = (uint8_t)(nm1 >> 8);
    hdr[1] = (uint8_t)(nm1 & 0xFF);
    cs = (uint8_t)(hdr[0] ^ hdr[1]);
    bl_send(hdr, 2);
    for (uint16_t i = 0; i < count; i++) {
      uint16_t pg = (uint16_t)(first_page + i);
      pb[0] = (uint8_t)(pg >> 8);
      pb[1] = (uint8_t)(pg & 0xFF);
      cs ^= (uint8_t)(pb[0] ^ pb[1]);
      bl_send(pb, 2);
    }
    bl_send(&cs, 1);

    if (bl_recv(&b, 1, 30000)) { Log("    Erase[%u] d%d: cevap yok\r\n", first_page, attempt + 1); continue; }
    if (b == BL_ACK) return 0;
    Log("    Erase[%u] d%d: cevap 0x%02X\r\n", first_page, attempt + 1, b);
  }
  return -1;
}

/* 0x31: Write Memory -> addr adresine len (1..256) bayt yazar. Tekrarli. */
static int bl_write(uint32_t addr, const uint8_t *data, uint16_t len)
{
  if (len == 0 || len > 256) return -10;

  for (int attempt = 0; attempt < 4; attempt++) {
    uint8_t a[5];
    uint8_t n, cs;

    if (bl_cmd(0x31)) continue;                 /* komut ACK */
    a[0] = (uint8_t)(addr >> 24);
    a[1] = (uint8_t)(addr >> 16);
    a[2] = (uint8_t)(addr >> 8);
    a[3] = (uint8_t)(addr);
    a[4] = a[0] ^ a[1] ^ a[2] ^ a[3];
    bl_send(a, 5);
    if (bl_wait_ack(1000)) continue;            /* adres ACK */

    n  = (uint8_t)(len - 1);
    cs = n;
    for (uint16_t i = 0; i < len; i++) cs ^= data[i];
    bl_send(&n, 1);
    bl_send(data, len);
    bl_send(&cs, 1);
    if (bl_wait_ack(5000) == 0) return 0;       /* veri ACK */

    Log("    Write @0x%08lX d%d: ACK yok\r\n", (unsigned long)addr, attempt + 1);
  }
  return -1;
}

/* 0x21: Go -> verilen adresten uygulamayi baslatir */
static int bl_go(uint32_t addr)
{
  uint8_t a[5];
  if (bl_cmd(0x21)) return -1;
  a[0] = (uint8_t)(addr >> 24);
  a[1] = (uint8_t)(addr >> 16);
  a[2] = (uint8_t)(addr >> 8);
  a[3] = (uint8_t)(addr);
  a[4] = a[0] ^ a[1] ^ a[2] ^ a[3];
  bl_send(a, 5);
  return bl_wait_ack(1000);
}

/* ============================================================
 *  Ust seviye akis: hedefi programla
 *  Donus: 0 basari, negatif deger hata adimi
 * ============================================================ */
int Program_Target(void)
{
  uint16_t pid = 0;

  Log("[1-2] Bootloader baglantisi...\r\n");
  if (bl_connect())    { Log("    HATA: hicbir baud'da ACK yok\r\n"); return -1; }

  Log("[3] Get ID...\r\n");
  if (bl_get_id(&pid)) { Log("    HATA: ID okunamadi\r\n"); return -2; }
  Log("    Hedef PID = 0x%04X\r\n", pid);

  /* CubeProgrammer gibi SAYFA erase (global 0xFFFF mass erase bu bootloader'da
     NACK aliyor). Marjinal IR icin 32'lik gruplar halinde, her grup tekrarli. */
  {
    uint16_t total_pages = (uint16_t)((fw_size + TARGET_PAGE_SIZE - 1) / TARGET_PAGE_SIZE);
    const uint16_t BATCH = 32;
    Log("[4] Sayfa erase: %u sayfa...\r\n", total_pages);
    for (uint16_t p = 0; p < total_pages; p += BATCH) {
      uint16_t cnt = (uint16_t)((total_pages - p > BATCH) ? BATCH : (total_pages - p));
      if (bl_erase_range(p, cnt)) { Log("    HATA: erase @ sayfa %u\r\n", p); return -3; }
      Log("    silindi: %u/%u\r\n", (unsigned)(p + cnt), total_pages);
    }
  }
  Log("    Erase OK\r\n");

  Log("[5] Yaziliyor: %lu bayt @ 0x%08lX\r\n",
      (unsigned long)fw_size, (unsigned long)TARGET_FLASH_BASE);
  for (uint32_t off = 0, idx = 0; off < fw_size; off += BL_CHUNK, idx++) {
    uint16_t chunk = (fw_size - off > BL_CHUNK) ? BL_CHUNK
                                                : (uint16_t)(fw_size - off);
    if (bl_write(TARGET_FLASH_BASE + off, &fw_data[off], chunk)) {
      Log("    HATA: yazma basarisiz @ ofset %lu\r\n", (unsigned long)off);
      return -4;
    }
    if ((idx % 16) == 0) {  /* her ~4KB'de bir ilerleme logu */
      uint8_t pct = (uint8_t)(((uint64_t)(off + chunk) * 100U) / fw_size);
      Log("    %lu / %lu bayt (%%%u)\r\n",
          (unsigned long)(off + chunk), (unsigned long)fw_size, pct);
    }
  }
  Log("    Yazma tamam (%lu bayt)\r\n", (unsigned long)fw_size);

  /* Reset hatti yok -> CubeProgrammer gibi Go komutuyla uygulamaya atla */
  Log("[6] Go: uygulamaya atlaniyor @ 0x%08lX\r\n", (unsigned long)TARGET_FLASH_BASE);
  if (bl_go(TARGET_FLASH_BASE)) Log("    Uyari: Go ACK yok\r\n");

  return 0;
}

/* ============================================================
 *  SWD ile programlama (SWD/swd.c surucusu, J1 konnektoru)
 * ============================================================ */
static const char *swd_status_str(swd_status_t s)
{
  switch (s)
  {
    case SWD_OK:         return "OK";
    case SWD_ERR_ACK:    return "ERR_ACK (baglanti yok / WAIT / FAULT)";
    case SWD_ERR_PARITY: return "ERR_PARITY";
    case SWD_ERR_IDCODE: return "ERR_IDCODE (farkli cip?)";
    case SWD_ERR_FLASH:  return "ERR_FLASH (unlock/erase/program)";
    case SWD_ERR_VERIFY: return "ERR_VERIFY (geri-okuma uyusmadi)";
    default:             return "ERR_?";
  }
}

/* swd_flash_program_ex ilerleme bildirimi: %10 adimlarla log */
static void swd_progress(uint32_t done, uint32_t total)
{
  static uint32_t last = 0xFFFFFFFFu;
  uint32_t pct = total ? (done * 100u / total) : 100u;
  if (done == 0u) last = 0xFFFFFFFFu;   /* ilk cagri = erase bitti, yazma basliyor */
  if (pct / 10u != last)
  {
    last = pct / 10u;
    Log("    %lu / %lu bayt (%%%lu)\r\n",
        (unsigned long)done, (unsigned long)total, (unsigned long)pct);
  }
}

/* Donus: 0 basari, negatif ERR_SWD_* */
int Program_Target_SWD(void)
{
  uint32_t idcode = 0, t0;
  swd_status_t st;

  /* Pinleri her cevrimde yeniden kur (onceki cevrim sonunda hi-Z birakildi) */
  swd_set_delay(SWD_CLK_DELAY);
  swd_gpio_init();

  Log("[1] SWD: hedef reset + attach...\r\n");
  swd_target_reset();
  st = swd_attach(&idcode);
  Log("    IDCODE = 0x%08lX (beklenen 0x%08lX) -> %s\r\n",
      (unsigned long)idcode, (unsigned long)SWD_EXPECTED_IDCODE, swd_status_str(st));
  if (st != SWD_OK) { swd_run_target(); return ERR_SWD_ATTACH; }

  Log("[2] Flash unlock...\r\n");
  if (swd_flash_unlock() != SWD_OK)
  {
    Log("    HATA: unlock\r\n");
    swd_run_target();
    return ERR_SWD_UNLOCK;
  }

  Log("[3] Erase + yazma: %lu bayt @ 0x%08lX\r\n",
      (unsigned long)fw_size, (unsigned long)TARGET_FLASH_BASE);
  t0 = HAL_GetTick();
  st = swd_flash_program_ex(TARGET_FLASH_BASE, fw_data, fw_size, swd_progress);
  if (st != SWD_OK)
  {
    Log("    HATA: %s\r\n", swd_status_str(st));
    swd_flash_lock();
    swd_run_target();
    return ERR_SWD_PROGRAM;
  }
  Log("    Yazma tamam (%lu ms)\r\n", (unsigned long)(HAL_GetTick() - t0));

  Log("[4] Dogrulama...\r\n");
  t0 = HAL_GetTick();
  st = swd_flash_verify(TARGET_FLASH_BASE, fw_data, fw_size);
  swd_flash_lock();
  if (st != SWD_OK)
  {
    Log("    HATA: %s\r\n", swd_status_str(st));
    swd_run_target();
    return ERR_SWD_VERIFY;
  }
  Log("    Verify OK (%lu ms)\r\n", (unsigned long)(HAL_GetTick() - t0));

  Log("[5] Hedef reset'leniyor, uygulama calisiyor\r\n");
  swd_run_target();   /* NRST darbesi + SWCLK/SWDIO hi-Z */
  return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Log("\r\n========================================\r\n");
  Log(" STM32L073 IR + SWD Programmer (IR_Programmer_PPM kart)\r\n");
  Log(" Yazilim: v%d   Derleme: %s %s\r\n", SOFTWARE_VERSION, __DATE__, __TIME__);
  Log(" Imaj: %lu bayt\r\n", (unsigned long)fw_size);
  Log("========================================\r\n");
  Log(" Hazir. SW1 = SWD guncelleme, SW3 = IR guncelleme.\r\n");

  Ui_Ready();   /* baslangic bipi - menu tusu bekleniyor */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* SW1 -> SWD, SW3 -> IR guncellemesi.
       - Programlama sirasinda tuslar yoklanmaz -> o sirada basislar yok sayilir.
       - Sonuc bip ile bildirilir; YENI bir basis (birak+bas) yeni cevrim
         baslatir -> hatadan sonra tekrar atar, basaridan sonra sonraki
         hedefe atar. */
    static int prev_ir = 1, prev_swd = 1;
    int start_ir  = Button_PressEdge(BtnIR_GPIO_Port,  BtnIR_Pin,  &prev_ir);
    int start_swd = Button_PressEdge(BtnSWD_GPIO_Port, BtnSWD_Pin, &prev_swd);

    if (start_ir || start_swd)
    {
      const char *mode = start_ir ? "IR" : "SWD";
      Ui_Begin();                       /* tek kisa bip */
      Log("\r\n--- %s: programlama baslatildi ---\r\n", mode);

      HAL_Delay(300);                   /* baslangic bipi ile mesgul bipi karismasin */
      BusyBeep_Start();                 /* programlama boyunca ~1 sn'de bir kisa bip */
      g_prog_result = start_ir ? Program_Target() : Program_Target_SWD();
      BusyBeep_Stop();
      HAL_Delay(300);                   /* sonuc biplerinden once kisa sessizlik */

      if (g_prog_result == 0)
      {
        Ui_Success();
        Log("Sonuc (%s): 0 (BASARILI)\r\n", mode);
      }
      else
      {
        Ui_Error(g_prog_result);
        Log("Sonuc (%s): %d (HATA) - tekrar denemek icin tusa bas\r\n", mode, g_prog_result);
      }

      /* Programlama sirasinda basilip birakilan tus sonradan tetiklemesin */
      prev_ir = prev_swd = 1;
    }
    HAL_Delay(20);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_9B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_EVEN;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W25Q_CS_GPIO_Port, W25Q_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W25Q_RESET_GPIO_Port, W25Q_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_Pin_GPIO_Port, Buzzer_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BtnSWD_Pin */
  GPIO_InitStruct.Pin = BtnSWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BtnSWD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SWD_NRST_Pin SWD_SWDIO_Pin SWD_SWCLK_Pin */
  GPIO_InitStruct.Pin = SWD_NRST_Pin|SWD_SWDIO_Pin|SWD_SWCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : W25Q_CS_Pin */
  GPIO_InitStruct.Pin = W25Q_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W25Q_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W25Q_RESET_Pin */
  GPIO_InitStruct.Pin = W25Q_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W25Q_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BtnIR_Pin */
  GPIO_InitStruct.Pin = BtnIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BtnIR_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
