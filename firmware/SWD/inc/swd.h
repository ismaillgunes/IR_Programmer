/**
  ******************************************************************************
  * @file    swd.h
  * @brief   Bit-bang SWD (ADIv5) surucusu - STM32L0 standalone programlayici
  *
  *  Programlayici tarafi pin atamasi (IR_Programmer_PPM karti, J1 konnektoru):
  *      SWCLK -> PA2   (J1 pin 6, SWD_SWCLK)
  *      SWDIO -> PA1   (J1 pin 2, SWD_SWDIO)
  *      NRST  -> PA0   (J1 pin 1, SWD_RESET - open-drain, hedefin pull-up'i ile)
  *  Uc pin ayni portta olmali (surucu tek port register'i kullanir).
  *
  *  ASAMA 1: fiziksel SWD katmani + JTAG->SWD gecisi + IDCODE okuma.   [TAMAM]
  *  ASAMA 2: DP power-up + AHB-AP memory read/write + core halt.       [TAMAM]
  *  ASAMA 3: STM32L0 (PECR) flash programlama, imaj yazma/verify.       [TAMAM]
  ******************************************************************************
  */
#ifndef __SWD_H
#define __SWD_H

#include "main.h"
#include <stdint.h>

/* ===== Programlayici SWD pin konfigurasyonu ===== */
#define SWD_GPIO_PORT       GPIOA
#define SWD_GPIO_CLK_EN()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define SWCLK_PIN_NUM       2u      /* PA2 */
#define SWDIO_PIN_NUM       1u      /* PA1 */
#define NRST_PIN_NUM        0u      /* PA0 */

/* Cortex-M0+ SW-DP icin beklenen IDCODE (STM32L0 ailesi) */
#define SWD_EXPECTED_IDCODE 0x0BC11477u

/* SWD ACK kodlari */
#define SWD_ACK_OK          0x1u
#define SWD_ACK_WAIT        0x2u

/* Donus durumlari */
typedef enum
{
    SWD_OK = 0,
    SWD_ERR_ACK,        /* hedef OK disinda ACK dondurdu (WAIT/FAULT/baglanti yok) */
    SWD_ERR_PARITY,     /* okunan veride parity hatasi */
    SWD_ERR_IDCODE,     /* IDCODE beklenenden farkli */
    SWD_ERR_FLASH,      /* flash unlock/erase/program/verify hatasi */
    SWD_ERR_VERIFY      /* yazilan veri geri okunanla uyusmuyor */
} swd_status_t;

/* STM32L0 flash geometrisi */
#define TARGET_FLASH_PAGE_SIZE  128u    /* silme birimi: 128 bayt */

/* SWD bit-bang hizini ayarlayan gecikme (artarsa yavaslar). Bring-up'ta artirip
   azaltarak guvenilirligi test edebilirsin. */
void         swd_set_delay(uint32_t loops);

/* GPIO pinlerini SWD icin hazirla (SWCLK/SWDIO output PP, NRST open-drain). */
void         swd_gpio_init(void);

/* Hedefi NRST ile resetle (low->high). */
void         swd_target_reset(void);

/* Line reset + JTAG->SWD gecisi + DPIDR (IDCODE) oku.
   Basari halinde *idcode doldurulur. */
swd_status_t swd_connect(uint32_t *idcode);

/* ASAMA 2: debug altyapisi */
swd_status_t swd_dp_powerup(void);   /* CDBG/CSYS power-up + sticky hata temizleme */
swd_status_t swd_mem_ap_init(void);  /* AHB-AP sec + CSW (32-bit, auto-inc) ayarla */
swd_status_t swd_halt(void);         /* Cortex-M cekirdegini durdur (DHCSR) */

/* AHB-AP uzerinden 32-bit hedef hafiza erisimi */
swd_status_t swd_mem_read32(uint32_t addr, uint32_t *data);
swd_status_t swd_mem_write32(uint32_t addr, uint32_t data);

/* Ardisik 32-bit blok okuma (auto-increment + posted-read; verify icin hizli) */
swd_status_t swd_mem_read_block(uint32_t addr, uint32_t *buf, uint32_t words);

/* Hepsini sirayla yap: connect + powerup + mem-ap init + halt */
swd_status_t swd_attach(uint32_t *idcode);

/* ASAMA 3: STM32L0 flash (PECR) - hedef HALT edilmis olmali */
swd_status_t swd_flash_unlock(void);
swd_status_t swd_flash_lock(void);
swd_status_t swd_flash_erase_page(uint32_t addr);       /* 128 bayt sayfa sil */
swd_status_t swd_flash_verify(uint32_t base, const uint8_t *data, uint32_t len);

/* Yuksek seviye: [base, base+len) araligini kapsayan sayfalari sil + programla.
   data little-endian word olarak yazilir; uzunluk 4'un kati degilse 0x00 ile doldurulur.
   cb: ilerleme bildirimi (done/total bayt), NULL olabilir. */
typedef void (*swd_progress_cb)(uint32_t done, uint32_t total);
swd_status_t swd_flash_program_ex(uint32_t base, const uint8_t *data, uint32_t len,
                                  swd_progress_cb cb);

/* Hedefi donanim reset'i ile yeniden baslat ve SWD pinlerini birak (hi-Z),
   boylece hedef yeni yazilan firmware'i calistirir. */
void swd_run_target(void);

#endif /* __SWD_H */
