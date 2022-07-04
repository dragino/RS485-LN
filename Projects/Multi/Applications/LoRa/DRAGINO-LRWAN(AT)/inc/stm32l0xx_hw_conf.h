/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: contains hardaware configuration Macros and Constants

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
 /******************************************************************************
  * @file    stm32l0xx_hw_conf.h
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   contains hardaware configuration Macros and Constants
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONF_L0_H__
#define __HW_CONF_L0_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* LORA I/O definition */
	 
#define RADIO_RESET_PORT                          GPIOB
#define RADIO_RESET_PIN                           GPIO_PIN_0

#define RADIO_NSS_PORT                            GPIOA
#define RADIO_NSS_PIN                             GPIO_PIN_15

#define RADIO_SCLK_PORT                           GPIOA
#define RADIO_SCLK_PIN                            GPIO_PIN_5

#define RADIO_MISO_PORT                           GPIOA
#define RADIO_MISO_PIN                            GPIO_PIN_6
	 
#define RADIO_MOSI_PORT                           GPIOA
#define RADIO_MOSI_PIN                            GPIO_PIN_7

#define RADIO_DIO_0_PORT                          GPIOC
#define RADIO_DIO_0_PIN                           GPIO_PIN_13

#define RADIO_DIO_1_PORT                          GPIOB
#define RADIO_DIO_1_PIN                           GPIO_PIN_10

#define RADIO_DIO_2_PORT                          GPIOB
#define RADIO_DIO_2_PIN                           GPIO_PIN_11

#define RADIO_DIO_3_PORT                          GPIOB
#define RADIO_DIO_3_PIN                           GPIO_PIN_8

#define RADIO_DIO_4_PORT                          GPIOB
#define RADIO_DIO_4_PIN                           GPIO_PIN_9

#define RADIO_DIO_5_PORT                          GPIOB
#define RADIO_DIO_5_PIN                           GPIO_PIN_1

#define RADIO_ANT_SWITCH_PORT                     GPIOA
#define RADIO_ANT_SWITCH_PIN                      GPIO_PIN_8

#define BAT_LEVEL_PORT                            GPIOA
#define BAT_LEVEL_PIN                             GPIO_PIN_4
/*  SPI MACRO redefinition */

#define SPI_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()

#define SPI1_AF                          GPIO_AF0_SPI1  

/* ADC MACRO redefinition */

#ifdef USE_STM32L0XX_NUCLEO
#define ADC_READ_CHANNEL                 ADC_CHANNEL_4
#define ADCCLK_ENABLE()                 __HAL_RCC_ADC1_CLK_ENABLE() ;
#define ADCCLK_DISABLE()                __HAL_RCC_ADC1_CLK_DISABLE() ;
#endif

/* --------------------------- RTC HW definition -------------------------------- */

#define RTC_OUTPUT       DBG_RTC_OUTPUT

#define RTC_Alarm_IRQn              RTC_IRQn

/* --------------------------- USART HW definition -------------------------------*/
#define USARTX                           LPUART1
#define USARTX_CLK_ENABLE()              __LPUART1_CLK_ENABLE()
#define USARTX_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTX_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE() 
#define DMAX_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

#define USARTX_FORCE_RESET()             __LPUART1_FORCE_RESET()
#define USARTX_RELEASE_RESET()           __LPUART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTX_TX_PIN                  GPIO_PIN_2
#define USARTX_TX_GPIO_PORT            GPIOA  
#define USARTX_TX_AF                   GPIO_AF6_LPUART1
#define USARTX_RX_PIN                  GPIO_PIN_3
#define USARTX_RX_GPIO_PORT            GPIOA 
#define USARTX_RX_AF                   GPIO_AF6_LPUART1

/* Definition for USART1 Pins */
#define USARTX1_TX_PIN                  GPIO_PIN_9
#define USARTX1_TX_GPIO_PORT            GPIOA  
#define USARTX1_TX_AF                   GPIO_AF4_USART1
#define USARTX1_RX_PIN                  GPIO_PIN_10
#define USARTX1_RX_GPIO_PORT            GPIOA 
#define USARTX1_RX_AF                   GPIO_AF4_USART1

/* Definition for USARTx's NVIC */
#define USARTX_IRQn                      RNG_LPUART1_IRQn
#define USARTX_IRQHandler                RNG_LPUART1_IRQHandler
/* Definition for USARTx's DMA */
#define USARTX_TX_DMA_CHANNEL             DMA1_Channel7

/* Definition for USARTx's DMA Request */
#define USARTX_TX_DMA_REQUEST             DMA_REQUEST_5

/* Definition for USARTx's NVIC */
#define USARTX_DMA_TX_IRQn                DMA1_Channel4_5_6_7_IRQn
#define USARTX_DMA_TX_IRQHandler          DMA1_Channel4_5_6_7_IRQHandler

#define USARTX_Priority 1
#define USARTX_DMA_Priority 1

/* ---------------------------  RS485 TX_RX_pin definition -------------------------------*/
#define RS485_PIN               GPIO_PIN_5 
#define RS485_GPIO_PORT         GPIOB

/* ---------------------------  GPIO EXTI definition -------------------------------*/
#define RS485_EXTI_CLK_ENABLE()	  __HAL_RCC_GPIOB_CLK_ENABLE()
#define RS485_Interrupt_PIN       GPIO_PIN_12 
#define RS485_Interrupt_PORT      GPIOB
#define RS485_ALARM_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define RS485_ALARM_PIN    			  GPIO_PIN_14 
#define RS485_ALARM_PORT          GPIOB

/* --------------------------- LED HW definition -------------------------------*/
#define LED_CLK_ENABLE()                __HAL_RCC_GPIOA_CLK_ENABLE()
#define LED_RGB_PORT                    GPIOA
#define LED_BLUE_PIN                    GPIO_PIN_8
#define LED_RED_PIN                     GPIO_PIN_12
#define LED_GREEN_PIN                   GPIO_PIN_11

#ifdef __cplusplus
}
#endif

#endif /* __HW_CONF_L0_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
