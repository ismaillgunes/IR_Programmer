/**
  ******************************************************************************
  * @file    swd.c
  * @brief   Bit-bang SWD (ADIv5)
  *          ASAMA 1: fiziksel katman + IDCODE
  *          ASAMA 2: DP power-up + AHB-AP memory read/write + core halt
  ******************************************************************************
  */
#include "swd.h"

/* ===== Pin maskeleri / hizli erisim makrolari ===== */
#define SWCLK_MASK   (1u << SWCLK_PIN_NUM)
#define SWDIO_MASK   (1u << SWDIO_PIN_NUM)
#define NRST_MASK    (1u << NRST_PIN_NUM)

#define SWCLK_HI()   (SWD_GPIO_PORT->BSRR = SWCLK_MASK)
#define SWCLK_LO()   (SWD_GPIO_PORT->BSRR = (uint32_t)SWCLK_MASK << 16)
#define SWDIO_HI()   (SWD_GPIO_PORT->BSRR = SWDIO_MASK)
#define SWDIO_LO()   (SWD_GPIO_PORT->BSRR = (uint32_t)SWDIO_MASK << 16)
#define SWDIO_READ() ((SWD_GPIO_PORT->IDR & SWDIO_MASK) ? 1u : 0u)
#define NRST_HI()    (SWD_GPIO_PORT->BSRR = NRST_MASK)
#define NRST_LO()    (SWD_GPIO_PORT->BSRR = (uint32_t)NRST_MASK << 16)

/* ===== ADIv5 register adresleri (A[3:2] degeri = offset>>2) ===== */
#define DP_IDCODE     0u   /* read  (offset 0x0) */
#define DP_ABORT      0u   /* write (offset 0x0) */
#define DP_CTRLSTAT   1u   /* offset 0x4 */
#define DP_SELECT     2u   /* write (offset 0x8) */
#define DP_RDBUFF     3u   /* read  (offset 0xC) */

#define AP_CSW        0u   /* offset 0x00 */
#define AP_TAR        1u   /* offset 0x04 */
#define AP_DRW        3u   /* offset 0x0C */

/* CTRL/STAT power-up bitleri */
#define CSYSPWRUPACK  (1u << 31)
#define CSYSPWRUPREQ  (1u << 30)
#define CDBGPWRUPACK  (1u << 29)
#define CDBGPWRUPREQ  (1u << 28)
#define PWRUP_REQ     (CSYSPWRUPREQ | CDBGPWRUPREQ)
#define PWRUP_ACK     (CSYSPWRUPACK | CDBGPWRUPACK)

/* MEM-AP CSW: 32-bit erisim + tek adim auto-increment */
#define CSW_VALUE     0x23000052u

/* Cortex-M debug registerlari */
#define ADDR_DHCSR    0xE000EDF0u
#define DHCSR_DBGKEY  0xA05F0000u
#define DHCSR_C_DEBUGEN (1u << 0)
#define DHCSR_C_HALT    (1u << 1)
#define DHCSR_S_HALT    (1u << 17)

/* Bit-bang yarim-periyot gecikmesi. 0 = en hizli. ERR_PARITY/ERR_ACK/ERR_VERIFY
   gorursen (uzun/parazitli kablo) 2-6'ya cikar. */
static uint32_t s_delay = 0u;

void swd_set_delay(uint32_t loops) { s_delay = loops; }

static inline void swd_delay(void)
{
    uint32_t n = s_delay;
    while (n--) { __NOP(); }
}

/* SWDIO yon kontrolu (MODER: 00=input, 01=output PP) */
static inline void swdio_out(void)
{
    SWD_GPIO_PORT->MODER = (SWD_GPIO_PORT->MODER & ~(3u << (SWDIO_PIN_NUM * 2)))
                         | (1u << (SWDIO_PIN_NUM * 2));
}
static inline void swdio_in(void)
{
    SWD_GPIO_PORT->MODER &= ~(3u << (SWDIO_PIN_NUM * 2));
}

/* ===== Tek bit yaz/oku (clock idle HIGH; yukselen kenarda ornekleme) ===== */
static inline void wbit(uint32_t b)
{
    if (b & 1u) SWDIO_HI(); else SWDIO_LO();
    SWCLK_LO(); swd_delay();
    SWCLK_HI(); swd_delay();
}
static inline uint32_t rbit(void)
{
    uint32_t b;
    SWCLK_LO(); swd_delay();
    b = SWDIO_READ();
    SWCLK_HI(); swd_delay();
    return b;
}
/* tek bos clock (turnaround): bus serbest */
static inline void tclk(void)
{
    SWCLK_LO(); swd_delay();
    SWCLK_HI(); swd_delay();
}

static inline uint32_t parity32(uint32_t v)
{
    v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
    return v & 1u;
}

static void swd_send(uint32_t val, int n)
{
    swdio_out();
    for (int i = 0; i < n; i++) { wbit(val & 1u); val >>= 1; }
}

/* ===== Line reset + JTAG->SWD gecis dizisi ===== */
static void swd_line_reset(void)
{
    swdio_out();
    for (int i = 0; i < 56; i++) { wbit(1u); }   /* >=50 clock, SWDIO high */
}

static void swd_jtag_to_swd(void)
{
    swd_line_reset();
    swd_send(0xE79Eu, 16);   /* JTAG-to-SWD select (LSB first) */
    swd_line_reset();
    swd_send(0x0000u, 8);    /* idle bitleri */
}

/* Bir AP/flash mesgul oldugunda hedef WAIT ACK dondurur; bu durumda transfer
   tekrar denenmeli. Yeterince yuksek tutuyoruz (her deneme tam bir transfer). */
#define SWD_WAIT_RETRIES  100

/* ===== Genel SWD transfer (WAIT ACK'te otomatik tekrar) =====
   APnDP: 0=DP, 1=AP ; RnW: 1=read 0=write ; addr: A[3:2] (2 bit) */
static swd_status_t swd_transfer(uint8_t APnDP, uint8_t RnW, uint8_t addr, uint32_t *data)
{
    uint8_t req = 0x81u;                          /* Start=1, Park=1 */
    req |= (uint8_t)((APnDP & 1u) << 1);
    req |= (uint8_t)((RnW   & 1u) << 2);
    req |= (uint8_t)((addr  & 1u) << 3);          /* A2 */
    req |= (uint8_t)(((addr >> 1) & 1u) << 4);    /* A3 */
    {
        uint32_t p = (APnDP & 1u) ^ (RnW & 1u) ^ (addr & 1u) ^ ((addr >> 1) & 1u);
        req |= (uint8_t)((p & 1u) << 5);
    }

    for (int retry = 0; retry <= SWD_WAIT_RETRIES; retry++)
    {
        /* istek (host suruyor) */
        swd_send(req, 8);

        /* turnaround -> input, ACK oku */
        swdio_in();
        tclk();
        uint32_t ack = 0;
        ack |= rbit() << 0;
        ack |= rbit() << 1;
        ack |= rbit() << 2;

        if (ack == SWD_ACK_WAIT)
        {
            tclk();                 /* turnaround geri (veri fazi yok) */
            swdio_out();
            continue;               /* WAIT: transferi tekrar dene */
        }
        if (ack != SWD_ACK_OK)
        {
            tclk();
            swdio_out();
            return SWD_ERR_ACK;     /* FAULT veya yanit yok */
        }

        if (RnW)
        {
            /* ---- READ ---- */
            uint32_t d = 0;
            for (int i = 0; i < 32; i++) { d |= (rbit() << i); }
            uint32_t par = rbit();
            tclk();                 /* turnaround -> output */
            swdio_out();
            if (parity32(d) != (par & 1u)) { return SWD_ERR_PARITY; }
            *data = d;
            return SWD_OK;
        }
        else
        {
            /* ---- WRITE ---- */
            tclk();                 /* turnaround -> output (host) */
            swdio_out();
            uint32_t d = *data;
            for (int i = 0; i < 32; i++) { wbit(d & 1u); d >>= 1; }
            wbit(parity32(*data));
            /* islemi tamamlamak icin birkac idle clock */
            SWDIO_LO();
            for (int i = 0; i < 8; i++) { wbit(0u); }
            return SWD_OK;
        }
    }

    return SWD_ERR_ACK;             /* cok fazla WAIT */
}

/* ===== DP/AP erisim yardimcilari ===== */
static swd_status_t dp_read(uint8_t a, uint32_t *d)  { return swd_transfer(0u, 1u, a, d); }
static swd_status_t dp_write(uint8_t a, uint32_t d)  { return swd_transfer(0u, 0u, a, &d); }
static swd_status_t ap_write(uint8_t a, uint32_t d)  { return swd_transfer(1u, 0u, a, &d); }

/* AP okumalari "posted"tir: once AP'ye istek atilir (eski deger gelir),
   gercek deger DP RDBUFF'tan okunur. */
static swd_status_t ap_read(uint8_t a, uint32_t *d)
{
    uint32_t dummy;
    swd_status_t st = swd_transfer(1u, 1u, a, &dummy);
    if (st != SWD_OK) { return st; }
    return dp_read(DP_RDBUFF, d);
}

/* ===== Public API ===== */
void swd_gpio_init(void)
{
    SWD_GPIO_CLK_EN();

    /* SWCLK: output push-pull */
    SWD_GPIO_PORT->MODER = (SWD_GPIO_PORT->MODER & ~(3u << (SWCLK_PIN_NUM * 2)))
                         | (1u << (SWCLK_PIN_NUM * 2));
    /* SWDIO: output (yon dinamik degisir) */
    SWD_GPIO_PORT->MODER = (SWD_GPIO_PORT->MODER & ~(3u << (SWDIO_PIN_NUM * 2)))
                         | (1u << (SWDIO_PIN_NUM * 2));
    /* NRST: output + open-drain + weak pull-up */
    SWD_GPIO_PORT->MODER = (SWD_GPIO_PORT->MODER & ~(3u << (NRST_PIN_NUM * 2)))
                         | (1u << (NRST_PIN_NUM * 2));
    SWD_GPIO_PORT->OTYPER |= (1u << NRST_PIN_NUM);
    SWD_GPIO_PORT->PUPDR  = (SWD_GPIO_PORT->PUPDR & ~(3u << (NRST_PIN_NUM * 2)))
                          | (1u << (NRST_PIN_NUM * 2));

    /* SWCLK/SWDIO hizli surulsun */
    SWD_GPIO_PORT->OSPEEDR |= (3u << (SWCLK_PIN_NUM * 2)) | (3u << (SWDIO_PIN_NUM * 2));

    /* idle seviyeler */
    SWCLK_HI();
    SWDIO_HI();
    NRST_HI();
}

void swd_target_reset(void)
{
    /* Kisa reset darbesi yeterli; hedefi hemen halt edecegiz, firmware'in
       boot etmesini BEKLEMIYORUZ (erken attach = hizli + guvenilir). */
    NRST_LO();
    for (volatile uint32_t i = 0; i < 20000u; i++) { __NOP(); }   /* ~10 ms reset low */
    NRST_HI();
    for (volatile uint32_t i = 0; i < 20000u; i++) { __NOP(); }   /* ~10 ms stabilizasyon */
}

swd_status_t swd_connect(uint32_t *idcode)
{
    uint32_t id = 0;
    swd_status_t st;

    swd_jtag_to_swd();

    st = dp_read(DP_IDCODE, &id);
    if (st != SWD_OK) { return st; }

    *idcode = id;
    if (id != SWD_EXPECTED_IDCODE) { return SWD_ERR_IDCODE; }
    return SWD_OK;
}

swd_status_t swd_dp_powerup(void)
{
    swd_status_t st;
    uint32_t stat = 0;
    int tries = 200;

    /* sticky hata bayraklarini temizle */
    st = dp_write(DP_ABORT, 0x1Eu);          /* ORUNERR|WDERR|STKERR|STKCMP clear */
    if (st != SWD_OK) { return st; }

    /* AP0 / bank0 sec */
    st = dp_write(DP_SELECT, 0x00000000u);
    if (st != SWD_OK) { return st; }

    /* debug + sistem alanini ac */
    st = dp_write(DP_CTRLSTAT, PWRUP_REQ);
    if (st != SWD_OK) { return st; }

    do {
        st = dp_read(DP_CTRLSTAT, &stat);
        if (st != SWD_OK) { return st; }
    } while (((stat & PWRUP_ACK) != PWRUP_ACK) && (--tries > 0));

    if ((stat & PWRUP_ACK) != PWRUP_ACK) { return SWD_ERR_ACK; }
    return SWD_OK;
}

swd_status_t swd_mem_ap_init(void)
{
    swd_status_t st;
    st = dp_write(DP_SELECT, 0x00000000u);   /* AP0, bank0 */
    if (st != SWD_OK) { return st; }
    return ap_write(AP_CSW, CSW_VALUE);
}

swd_status_t swd_mem_read32(uint32_t addr, uint32_t *data)
{
    swd_status_t st = ap_write(AP_TAR, addr);
    if (st != SWD_OK) { return st; }
    return ap_read(AP_DRW, data);
}

swd_status_t swd_mem_write32(uint32_t addr, uint32_t data)
{
    swd_status_t st = ap_write(AP_TAR, addr);
    if (st != SWD_OK) { return st; }
    return ap_write(AP_DRW, data);
}

/* Ardisik 32-bit blok okuma (AHB-AP auto-increment + posted-read pipeline).
   Word basina ~1 transfer; TAR yalnizca 1KB sinirlarinda yeniden ayarlanir. */
swd_status_t swd_mem_read_block(uint32_t addr, uint32_t *buf, uint32_t words)
{
    swd_status_t st;
    while (words)
    {
        /* TAR auto-increment 1KB sinirinda sarabilir -> chunk'i sinira gore boldur */
        uint32_t to_boundary = (0x400u - (addr & 0x3FFu)) / 4u;
        uint32_t chunk = (words < to_boundary) ? words : to_boundary;

        st = ap_write(AP_TAR, addr);
        if (st != SWD_OK) { return st; }

        uint32_t dummy;
        st = swd_transfer(1u, 1u, AP_DRW, &dummy);   /* pipeline baslat (sonuc atilir) */
        if (st != SWD_OK) { return st; }

        for (uint32_t i = 0; (i + 1u) < chunk; i++)
        {
            st = swd_transfer(1u, 1u, AP_DRW, &buf[i]);  /* word i */
            if (st != SWD_OK) { return st; }
        }
        st = dp_read(DP_RDBUFF, &buf[chunk - 1u]);   /* son word RDBUFF'tan */
        if (st != SWD_OK) { return st; }

        addr  += chunk * 4u;
        buf   += chunk;
        words -= chunk;
    }
    return SWD_OK;
}

swd_status_t swd_halt(void)
{
    swd_status_t st;
    uint32_t v = 0;
    int tries = 200;

    st = swd_mem_write32(ADDR_DHCSR, DHCSR_DBGKEY | DHCSR_C_DEBUGEN | DHCSR_C_HALT);
    if (st != SWD_OK) { return st; }

    do {
        st = swd_mem_read32(ADDR_DHCSR, &v);
        if (st != SWD_OK) { return st; }
    } while (!(v & DHCSR_S_HALT) && (--tries > 0));

    if (!(v & DHCSR_S_HALT)) { return SWD_ERR_ACK; }
    return SWD_OK;
}

swd_status_t swd_attach(uint32_t *idcode)
{
    swd_status_t st;
    st = swd_connect(idcode);   if (st != SWD_OK) { return st; }
    st = swd_dp_powerup();      if (st != SWD_OK) { return st; }
    st = swd_mem_ap_init();     if (st != SWD_OK) { return st; }
    st = swd_halt();            if (st != SWD_OK) { return st; }
    return SWD_OK;
}

/* ===================== ASAMA 3: STM32L0 FLASH (PECR) ===================== */

#define FLASH_REG_BASE   0x40022000u
#define FLASH_PECR       (FLASH_REG_BASE + 0x04u)
#define FLASH_PEKEYR     (FLASH_REG_BASE + 0x0Cu)
#define FLASH_PRGKEYR    (FLASH_REG_BASE + 0x10u)
#define FLASH_SR         (FLASH_REG_BASE + 0x18u)

#define PEKEY1   0x89ABCDEFu
#define PEKEY2   0x02030405u
#define PRGKEY1  0x8C9DAEBFu
#define PRGKEY2  0x13141516u

#define PECR_PELOCK   (1u << 0)
#define PECR_PRGLOCK  (1u << 1)
#define PECR_PROG     (1u << 3)
#define PECR_ERASE    (1u << 9)
#define PECR_FPRG     (1u << 10)   /* half-page (fast) programlama */

/* STM32L0: yarim sayfa = 64 bayt = 16 word */
#define HALF_PAGE_BYTES  64u
#define HALF_PAGE_WORDS  16u

#define SR_BSY        (1u << 0)
#define SR_WRPERR     (1u << 8)
#define SR_PGAERR     (1u << 9)
#define SR_SIZERR     (1u << 10)
#define SR_NOTZEROERR (1u << 16)
#define SR_FWWERR     (1u << 17)
#define SR_ERRMASK    (SR_WRPERR | SR_PGAERR | SR_SIZERR | SR_NOTZEROERR | SR_FWWERR)

/* FLASH_SR'da BSY temizlenene kadar bekle; hata bayraklarini kontrol/temizle. */
static swd_status_t flash_wait(void)
{
    uint32_t sr = 0;
    int tries = 200000;

    do {
        if (swd_mem_read32(FLASH_SR, &sr) != SWD_OK) { return SWD_ERR_ACK; }
    } while ((sr & SR_BSY) && (--tries > 0));

    if (sr & SR_BSY) { return SWD_ERR_FLASH; }

    if (sr & SR_ERRMASK)
    {
        swd_mem_write32(FLASH_SR, sr & SR_ERRMASK);  /* rc_w1: yazarak temizle */
        return SWD_ERR_FLASH;
    }
    return SWD_OK;
}

swd_status_t swd_flash_unlock(void)
{
    uint32_t pecr = 0;

    /* PECR (PELOCK) kilidini ac */
    if (swd_mem_write32(FLASH_PEKEYR, PEKEY1) != SWD_OK) { return SWD_ERR_ACK; }
    if (swd_mem_write32(FLASH_PEKEYR, PEKEY2) != SWD_OK) { return SWD_ERR_ACK; }
    /* program/erase (PRGLOCK) kilidini ac */
    if (swd_mem_write32(FLASH_PRGKEYR, PRGKEY1) != SWD_OK) { return SWD_ERR_ACK; }
    if (swd_mem_write32(FLASH_PRGKEYR, PRGKEY2) != SWD_OK) { return SWD_ERR_ACK; }

    if (swd_mem_read32(FLASH_PECR, &pecr) != SWD_OK) { return SWD_ERR_ACK; }
    if (pecr & (PECR_PELOCK | PECR_PRGLOCK)) { return SWD_ERR_FLASH; }
    return SWD_OK;
}

swd_status_t swd_flash_lock(void)
{
    uint32_t pecr = 0;
    if (swd_mem_read32(FLASH_PECR, &pecr) != SWD_OK) { return SWD_ERR_ACK; }
    pecr |= PECR_PELOCK;
    return swd_mem_write32(FLASH_PECR, pecr);
}

swd_status_t swd_flash_erase_page(uint32_t addr)
{
    uint32_t pecr = 0;
    swd_status_t st;

    st = flash_wait();
    if (st != SWD_OK) { return st; }

    if (swd_mem_read32(FLASH_PECR, &pecr) != SWD_OK) { return SWD_ERR_ACK; }
    pecr |= (PECR_ERASE | PECR_PROG);
    if (swd_mem_write32(FLASH_PECR, pecr) != SWD_OK) { return SWD_ERR_ACK; }

    /* Sayfa icindeki bir adrese yazmak silmeyi tetikler */
    if (swd_mem_write32(addr, 0x00000000u) != SWD_OK) { return SWD_ERR_ACK; }

    st = flash_wait();

    /* ERASE|PROG bitlerini temizle */
    if (swd_mem_read32(FLASH_PECR, &pecr) == SWD_OK)
    {
        pecr &= ~(PECR_ERASE | PECR_PROG);
        swd_mem_write32(FLASH_PECR, pecr);
    }
    return st;
}

/* byte tamponundan little-endian word oku (sinirdan tasarsa 0x00 ile doldur) */
static uint32_t rd_word_le(const uint8_t *p, uint32_t i, uint32_t len)
{
    uint32_t w = 0;
    for (uint32_t b = 0; b < 4u; b++)
    {
        uint32_t idx = i + b;
        uint8_t  v = (idx < len) ? p[idx] : 0x00u;
        w |= ((uint32_t)v) << (8u * b);
    }
    return w;
}

/* Tek bir yarim sayfayi (16 word) tek HV cevriminde programla.
   addr 64 bayt hizali olmali; eksik baytlar 0x00 ile doldurulur. */
static swd_status_t flash_program_halfpage(uint32_t addr, const uint8_t *data,
                                           uint32_t off, uint32_t len)
{
    uint32_t pecr = 0;
    swd_status_t st;

    st = flash_wait();
    if (st != SWD_OK) { return st; }

    /* FPRG | PROG ac */
    if (swd_mem_read32(FLASH_PECR, &pecr) != SWD_OK) { return SWD_ERR_ACK; }
    pecr |= (PECR_FPRG | PECR_PROG);
    if (swd_mem_write32(FLASH_PECR, pecr) != SWD_OK) { return SWD_ERR_ACK; }

    /* TAR = yarim sayfa basi; ardindan 16 word'u auto-increment ile ardarda yaz.
       (Aradaki PECR yazimi TAR'i bozar, bu yuzden TAR'i burada set ediyoruz.) */
    if (ap_write(AP_TAR, addr) != SWD_OK) { return SWD_ERR_ACK; }
    for (uint32_t w = 0; w < HALF_PAGE_WORDS; w++)
    {
        uint32_t word = rd_word_le(data, off + w * 4u, len);
        if (ap_write(AP_DRW, word) != SWD_OK) { return SWD_ERR_ACK; }
    }

    st = flash_wait();   /* yarim sayfa programlama burada gerceklesir */

    /* FPRG | PROG temizle */
    if (swd_mem_read32(FLASH_PECR, &pecr) == SWD_OK)
    {
        pecr &= ~(PECR_FPRG | PECR_PROG);
        swd_mem_write32(FLASH_PECR, pecr);
    }
    return st;
}

swd_status_t swd_flash_program_ex(uint32_t base, const uint8_t *data, uint32_t len,
                                  swd_progress_cb cb)
{
    swd_status_t st;
    uint32_t start = base & ~(TARGET_FLASH_PAGE_SIZE - 1u);
    uint32_t end   = base + len;

    /* 1) ilgili sayfalari sil */
    for (uint32_t a = start; a < end; a += TARGET_FLASH_PAGE_SIZE)
    {
        st = swd_flash_erase_page(a);
        if (st != SWD_OK) { return st; }
    }

    /* 2) yarim sayfa (64 bayt) blokları halinde hizli programla */
    for (uint32_t off = 0; off < len; off += HALF_PAGE_BYTES)
    {
        st = flash_program_halfpage(base + off, data, off, len);
        if (st != SWD_OK) { return st; }

        if (cb && (off % 2048u) == 0u) { cb(off, len); }
    }
    if (cb) { cb(len, len); }
    return SWD_OK;
}

swd_status_t swd_flash_verify(uint32_t base, const uint8_t *data, uint32_t len)
{
    uint32_t buf[64];          /* 256 baytlik blok */
    uint32_t off = 0;

    while (off < len)
    {
        uint32_t remain = len - off;
        uint32_t words  = (remain + 3u) / 4u;
        if (words > 64u) { words = 64u; }

        swd_status_t st = swd_mem_read_block(base + off, buf, words);
        if (st != SWD_OK) { return st; }

        for (uint32_t i = 0; i < words; i++)
        {
            uint32_t expected = rd_word_le(data, off + i * 4u, len);
            if (buf[i] != expected) { return SWD_ERR_VERIFY; }
        }
        off += words * 4u;
    }
    return SWD_OK;
}

void swd_run_target(void)
{
    /* Donanim reset'i ile yeniden baslat */
    NRST_LO();
    for (volatile uint32_t i = 0; i < 200000u; i++) { __NOP(); }
    NRST_HI();

    /* SWCLK/SWDIO'yu hi-Z birak (input) ki hedef serbestce calissin */
    SWD_GPIO_PORT->MODER &= ~(3u << (SWCLK_PIN_NUM * 2));
    SWD_GPIO_PORT->MODER &= ~(3u << (SWDIO_PIN_NUM * 2));
}
