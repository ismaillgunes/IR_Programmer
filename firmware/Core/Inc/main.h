/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   IR Programmer (IR_Programmer_PPM karti: STM32L073VBT + IR + buzzer)
  ******************************************************************************
  * @attention
  *
  * Bu yazilim, IR uzerinden hedef STM32'nin sistem bootloader'ina (AN3155)
  * baglanip fw_image.h icindeki firmware'i yazar.
  *   - IR linki : LPUART1 (PD8 = IR_TX -> D1 katot, PD9 = IR_RX <- Q2 kollektor)
  *   - SWD      : J1 (PA2 = SWCLK, PA1 = SWDIO, PA0 = NRST) - SWD/inc/swd.h
  *   - Log      : USART1  (PB6 = TX -> J3 pin2, PB7 = RX <- J3 pin1)
  *   - Geri bildirim: buzzer (PD12, Q3 uzerinden) - kartta LCD yok
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Buzzer(uint16_t ms);
void Buzzer_Tick(void);   /* SysTick'ten 1 ms'de bir cagrilir (mesgul bipi) */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BtnSWD_Pin GPIO_PIN_3
#define BtnSWD_GPIO_Port GPIOC
#define SWD_NRST_Pin GPIO_PIN_0
#define SWD_NRST_GPIO_Port GPIOA
#define SWD_SWDIO_Pin GPIO_PIN_1
#define SWD_SWDIO_GPIO_Port GPIOA
#define SWD_SWCLK_Pin GPIO_PIN_2
#define SWD_SWCLK_GPIO_Port GPIOA
#define W25Q_CS_Pin GPIO_PIN_4
#define W25Q_CS_GPIO_Port GPIOA
#define W25Q_RESET_Pin GPIO_PIN_4
#define W25Q_RESET_GPIO_Port GPIOC
#define Buzzer_Pin_Pin GPIO_PIN_12
#define Buzzer_Pin_GPIO_Port GPIOD
#define BtnIR_Pin GPIO_PIN_12
#define BtnIR_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
extern UART_HandleTypeDef hlpuart1; /* IR linki  (LPUART1 / PD8-PD9) */
extern UART_HandleTypeDef huart1;   /* Log       (USART1  / PB6-PB7) */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
