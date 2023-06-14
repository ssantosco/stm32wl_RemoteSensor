/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32wlxx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define ADCV_Pin GPIO_PIN_11
#define ADCV_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB
#define ADCA_Pin GPIO_PIN_10
#define ADCA_GPIO_Port GPIOA
#define FE_CTRL3_Pin GPIO_PIN_3
#define FE_CTRL3_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define FE_CTRL2_Pin GPIO_PIN_5
#define FE_CTRL2_GPIO_Port GPIOC
#define FE_CTRL1_Pin GPIO_PIN_4
#define FE_CTRL1_GPIO_Port GPIOC
#define B3_Pin GPIO_PIN_6
#define B3_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_1
#define B2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOB
#define T_VCP_RX_Pin GPIO_PIN_3
#define T_VCP_RX_GPIO_Port GPIOA
#define T_VCP_RXA2_Pin GPIO_PIN_2
#define T_VCP_RXA2_GPIO_Port GPIOA
#define SIREN_Pin GPIO_PIN_4
#define SIREN_GPIO_Port GPIOA
#define LIGHT_Pin GPIO_PIN_5
#define LIGHT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define SIREN_Pin GPIO_PIN_4
#define SIREN_GPIO_Port GPIOA
#define LIGHT_Pin GPIO_PIN_5
#define LIGHT_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    platform.h
  * @author  MCD Application Team
  * @brief   Header for General HW instances configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

#define USE_BSP_DRIVER
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32wlxx.h"
#include "main.h"
//#include "stm32wlxx_ll_gpio.h"
#if defined(USE_BSP_DRIVER)
/* code generated by STM32CubeMX does not support BSP.  */
/* In order to use BSP, users can add the BSP files in the IDE project space */
/* and define USE_BSP_DRIVER in the preprocessor definitions  */
#include "stm32wlxx_nucleo.h"
#include "stm32wlxx_nucleo_radio.h"
#endif /* defined(USE_BSP_DRIVER) */

/* USER CODE BEGIN include */
#include "radio_driver.h"
/* USER CODE END include */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wlxx_hal_conf.h
  * @author  MCD Application Team
  * @brief   HAL configuration file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32WLxx_HAL_CONF_H
#define STM32WLxx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver
  */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
/*#define HAL_COMP_MODULE_ENABLED   */
/*#define HAL_CRC_MODULE_ENABLED   */
/*#define HAL_CRYP_MODULE_ENABLED   */
/*#define HAL_DAC_MODULE_ENABLED   */
/*#define HAL_GTZC_MODULE_ENABLED   */
/*#define HAL_HSEM_MODULE_ENABLED   */
/*#define HAL_I2C_MODULE_ENABLED   */
/*#define HAL_I2S_MODULE_ENABLED   */
/*#define HAL_IPCC_MODULE_ENABLED   */
/*#define HAL_IRDA_MODULE_ENABLED   */
/*#define HAL_IWDG_MODULE_ENABLED   */
/*#define HAL_LPTIM_MODULE_ENABLED   */
/*#define HAL_PKA_MODULE_ENABLED   */
/*#define HAL_RNG_MODULE_ENABLED   */
/*#define HAL_RTC_MODULE_ENABLED   */
/*#define HAL_SMARTCARD_MODULE_ENABLED   */
/*#define HAL_SMBUS_MODULE_ENABLED   */
/*#define HAL_SPI_MODULE_ENABLED   */
#define HAL_SUBGHZ_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
/*#define HAL_USART_MODULE_ENABLED   */
/*#define HAL_WWDG_MODULE_ENABLED   */
#define HAL_EXTI_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED

/* ########################## Register Callbacks selection ############################## */
/**
  * @brief This is the list of modules where register callback can be used
  */
#define USE_HAL_ADC_REGISTER_CALLBACKS         0u
#define USE_HAL_COMP_REGISTER_CALLBACKS        0u
#define USE_HAL_CRYP_REGISTER_CALLBACKS        0u
#define USE_HAL_DAC_REGISTER_CALLBACKS         0u
#define USE_HAL_I2C_REGISTER_CALLBACKS         0u
#define USE_HAL_I2S_REGISTER_CALLBACKS         0u
#define USE_HAL_IRDA_REGISTER_CALLBACKS        0u
#define USE_HAL_LPTIM_REGISTER_CALLBACKS       0u
#define USE_HAL_PKA_REGISTER_CALLBACKS         0u
#define USE_HAL_RNG_REGISTER_CALLBACKS         0u
#define USE_HAL_RTC_REGISTER_CALLBACKS         0u
#define USE_HAL_SMARTCARD_REGISTER_CALLBACKS   0u
#define USE_HAL_SMBUS_REGISTER_CALLBACKS       0u
#define USE_HAL_SPI_REGISTER_CALLBACKS         0u
#define USE_HAL_SUBGHZ_REGISTER_CALLBACKS      0u
#define USE_HAL_TIM_REGISTER_CALLBACKS         0u
#define USE_HAL_UART_REGISTER_CALLBACKS        0u
#define USE_HAL_USART_REGISTER_CALLBACKS       0u
#define USE_HAL_WWDG_REGISTER_CALLBACKS        0u

/* ########################## Oscillator Values adaptation ####################*/
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).
  */

#if !defined (HSE_VALUE)
#define HSE_VALUE                           32000000UL  /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined (HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT                 100UL       /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief Internal Multiple Speed oscillator (MSI) default value.
  *        This value is the default MSI range value after Reset.
  */
#if !defined  (MSI_VALUE)
#define MSI_VALUE                           4000000UL   /*!< Value of the Internal oscillator in Hz*/
#endif /* MSI_VALUE */

/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL).
  */
#if !defined  (HSI_VALUE)
#define HSI_VALUE                           16000000UL  /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined (LSI_VALUE)
#define LSI_VALUE                           32000UL     /*!< LSI Typical Value in Hz*/
#endif /* LSI_VALUE */                                  /*!< Value of the Internal Low Speed oscillator in Hz
                                                        The real value may vary depending on the variations
                                                        in voltage and temperature. */

/**
  * @brief External Low Speed oscillator (LSE) value.
  *        This value is used by the UART, RTC HAL module to compute the system frequency
  */
#if !defined (LSE_VALUE)
#define LSE_VALUE                           32768UL     /*!< Value of the External oscillator in Hz*/
#endif /* LSE_VALUE */

/**
  * @brief Internal Multiple Speed oscillator (HSI48) default value.
  *        This value is the default HSI48 range value after Reset.
  */
#if !defined (HSI48_VALUE)
#define HSI48_VALUE                         48000000UL  /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI48_VALUE */

#if !defined (LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT                 5000UL      /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */

/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */

/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */
#define  VDD_VALUE                          3300U                             /*!< Value of VDD in mv */
#define  TICK_INT_PRIORITY                  0U
#define  USE_RTOS                           0U
#define  PREFETCH_ENABLE                    0U
#define  INSTRUCTION_CACHE_ENABLE           1U
#define  DATA_CACHE_ENABLE                  1U

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1 */

/* ################## SPI peripheral configuration ########################## */

/* CRC FEATURE: Use to activate CRC feature inside HAL SPI Driver
 * Activated: CRC code is present inside driver
 * Deactivated: CRC code cleaned from driver
 */

#define USE_SPI_CRC                         1U

/* ################## CRYP peripheral configuration ########################## */

#define USE_HAL_CRYP_SUSPEND_RESUME         1U

/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file
  */
#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32wlxx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32wlxx_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_COMP_MODULE_ENABLED
  #include "stm32wlxx_hal_comp.h"
#endif /* HAL_COMP_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32wlxx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32wlxx_hal_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_CRYP_MODULE_ENABLED
  #include "stm32wlxx_hal_cryp.h"
#endif /* HAL_CRYP_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32wlxx_hal_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32wlxx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32wlxx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32wlxx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_GTZC_MODULE_ENABLED
  #include "stm32wlxx_hal_gtzc.h"
#endif /* HAL_GTZC_MODULE_ENABLED */

#ifdef HAL_HSEM_MODULE_ENABLED
  #include "stm32wlxx_hal_hsem.h"
#endif /* HAL_HSEM_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
  #include "stm32wlxx_hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
  #include "stm32wlxx_hal_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_IPCC_MODULE_ENABLED
  #include "stm32wlxx_hal_ipcc.h"
#endif /* HAL_IPCC_MODULE_ENABLED */

#ifdef HAL_IRDA_MODULE_ENABLED
  #include "stm32wlxx_hal_irda.h"
#endif /* HAL_IRDA_MODULE_ENABLED */

#ifdef HAL_IWDG_MODULE_ENABLED
  #include "stm32wlxx_hal_iwdg.h"
#endif /* HAL_IWDG_MODULE_ENABLED */

#ifdef HAL_LPTIM_MODULE_ENABLED
  #include "stm32wlxx_hal_lptim.h"
#endif /* HAL_LPTIM_MODULE_ENABLED */

#ifdef HAL_PKA_MODULE_ENABLED
  #include "stm32wlxx_hal_pka.h"
#endif /* HAL_PKA_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32wlxx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32wlxx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
  #include "stm32wlxx_hal_rng.h"
#endif /* HAL_RNG_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
  #include "stm32wlxx_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
  #include "stm32wlxx_hal_smartcard.h"
#endif /* HAL_SMARTCARD_MODULE_ENABLED */

#ifdef HAL_SMBUS_MODULE_ENABLED
  #include "stm32wlxx_hal_smbus.h"
#endif /* HAL_SMBUS_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
  #include "stm32wlxx_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_SUBGHZ_MODULE_ENABLED
  #include "stm32wlxx_hal_subghz.h"
#endif /* HAL_SUBGHZ_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32wlxx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32wlxx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
  #include "stm32wlxx_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_WWDG_MODULE_ENABLED
  #include "stm32wlxx_hal_wwdg.h"
#endif /* HAL_WWDG_MODULE_ENABLED */

/* Exported macro ------------------------------------------------------------*/
#ifdef USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param expr If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* STM32WLxx_HAL_CONF_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wlxx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32WLxx_IT_H
#define __STM32WLxx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

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
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
void TIM16_IRQHandler(void);
void SUBGHZ_Radio_IRQHandler(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32WLxx_IT_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz.h
  * @brief   This file contains all the function prototypes for
  *          the subghz.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SUBGHZ_H__
#define __SUBGHZ_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SUBGHZ_HandleTypeDef hsubghz;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SUBGHZ_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SUBGHZ_H__ */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim16;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM16_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

