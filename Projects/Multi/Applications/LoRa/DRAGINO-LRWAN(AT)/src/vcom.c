 /******************************************************************************
  * @file    vcom.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    10-July-2018
  * @brief   manages virtual com port
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
  
#include "hw.h"
#include "vcom.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Uart Handle */
static UART_HandleTypeDef UartHandle;
UART_HandleTypeDef UartHandle1;

uint8_t charRx;
uint8_t aRxBuffer[1];
uint8_t response[1];
uint8_t receivr_return_issss;
uint8_t receive_returndatas[256];
uint8_t flags_command_temp;
uint8_t rxdatatemplen;
uint8_t rxdata[256];
uint16_t maxlen=0;
uint8_t rxdata_maxlen[550];
uint8_t rxdata_rxmode[255];
uint8_t maxlen_rxmode=0;
bool cfgdev_flags;
bool receive_return_flag;

extern bool rxdata_time_flag;
extern uint32_t baudr;
extern uint8_t pari,stopbit,databit;
extern uint8_t command_mode;

//uint8_t uartprintf_flag=0;

static void (*TxCpltCallback) (void);

static void (*RxCpltCallback) (uint8_t *rxChar);
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
void vcom_Init(  void (*TxCb)(void) )
{

  /*Record Tx complete for DMA*/
  TxCpltCallback=TxCb;
  /*## Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 921600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTX;
  
  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;

  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}

void vcom_Trace(  uint8_t *p_data, uint16_t size )
{
  HAL_UART_Transmit_DMA(&UartHandle,p_data, size);
}

void ULT_uart_IRQHandler(void)
{
	HAL_UART_IRQHandler(&UartHandle1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* buffer transmission complete*/
   if (NULL != TxCpltCallback)
   {
     TxCpltCallback(); 
   }
}

void vcom_ReceiveInit(  void (*RxCb)(uint8_t *rxChar) )
{
  UART_WakeUpTypeDef WakeUpSelection;
  
  /*record call back*/
  RxCpltCallback=RxCb;

  /*Set wakeUp event on start bit*/
  WakeUpSelection.WakeUpEvent=UART_WAKEUP_ON_STARTBIT;  
//  
  HAL_UARTEx_StopModeWakeUpSourceConfig(&UartHandle, WakeUpSelection );
  
  /*Enable wakeup from stop mode*/
  HAL_UARTEx_EnableStopMode(&UartHandle);
  
  /*Start LPUART receive on IT*/
  HAL_UART_Receive_IT(&UartHandle, &charRx,1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance==LPUART1)
	{
    if ((NULL != RxCpltCallback) && (HAL_UART_ERROR_NONE ==UartHandle->ErrorCode))
    {
      RxCpltCallback(&charRx);
    }
    HAL_UART_Receive_IT(UartHandle, &charRx,1);
  }
	else if(UartHandle->Instance==USART1)
	{
		while(HAL_UART_Receive_IT(&UartHandle1, (uint8_t *)aRxBuffer,1)!=HAL_OK)
		{
			UartHandle1.RxState = HAL_UART_STATE_READY;
			__HAL_UNLOCK(&UartHandle1);
		}		
		response[0]=aRxBuffer[0];
//  	PPRINTF("%02x ",response[0]);	
		if(receive_return_flag==1)
		{
			receive_returndatas[receivr_return_issss]=response[0];
			receivr_return_issss++;
			if(receivr_return_issss>=255)
			{
				receive_return_flag=0;
			}
		}		

		if(command_mode==1)
		{
			rxdata_time_flag=1;
			if(maxlen_rxmode<=254)
			{
			 rxdata_rxmode[maxlen_rxmode++]=response[0];
			}						
		}	
		else
		{
			if(cfgdev_flags==1)
			{
				rxdata_maxlen[maxlen++]=response[0];
				if(maxlen>=550)
				{
					cfgdev_flags=0;
				}
			}
			else if(flags_command_temp==1)
			{			
				rxdata[rxdatatemplen++]=response[0];
				if(rxdatatemplen==255)
				{
					flags_command_temp=0;
				}
			}
		}
	}
}

void vcom_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(UartHandle.hdmatx);
}

void vcom_IRQHandler(void)
{
  HAL_UART_IRQHandler(&UartHandle);
}

void vcom_DeInit(void)
{
  HAL_UART_DeInit(&UartHandle);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	if(huart->Instance==LPUART1)
	{
		static DMA_HandleTypeDef hdma_tx;
			
		/*##-1- Enable peripherals and GPIO Clocks #################################*/
		/* Enable GPIO TX/RX clock */
		USARTX_TX_GPIO_CLK_ENABLE();
		USARTX_RX_GPIO_CLK_ENABLE();

		/* Enable USARTX clock */
		USARTX_CLK_ENABLE();
		 /* select USARTX clock source*/
		RCC_PeriphCLKInitTypeDef  PeriphClkInit={0};
		PeriphClkInit.PeriphClockSelection=RCC_PERIPHCLK_LPUART1;
		PeriphClkInit.Lpuart1ClockSelection=RCC_LPUART1CLKSOURCE_HSI;
		HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

		/* Enable DMA clock */
		DMAX_CLK_ENABLE();
		
		/*##-2- Configure peripheral GPIO ##########################################*/  
		/* UART  pin configuration  */
		vcom_IoInit();

		/*##-3- Configure the DMA ##################################################*/
		/* Configure the DMA handler for Transmission process */
		hdma_tx.Instance                 = USARTX_TX_DMA_CHANNEL;
		hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode                = DMA_NORMAL;
		hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
	#ifndef STM32L152xE
		hdma_tx.Init.Request             = USARTX_TX_DMA_REQUEST;
	#endif
		HAL_DMA_Init(&hdma_tx);

		/* Associate the initialized DMA handle to the UART handle */
		__HAL_LINKDMA(huart, hdmatx, hdma_tx);
			
		/*##-4- Configure the NVIC for DMA #########################################*/
		/* NVIC configuration for DMA transfer complete interrupt*/
		HAL_NVIC_SetPriority(USARTX_DMA_TX_IRQn, USARTX_Priority, 1);
		HAL_NVIC_EnableIRQ(USARTX_DMA_TX_IRQn);
			
		/* NVIC for USART, to catch the TX complete */
		HAL_NVIC_SetPriority(USARTX_IRQn, USARTX_DMA_Priority, 1);
		HAL_NVIC_EnableIRQ(USARTX_IRQn);
	}
	else if(huart->Instance==USART1)
	{
		GPIO_InitTypeDef  GPIO_InitStruct={0};
		__HAL_RCC_GPIOA_CLK_ENABLE();	
		__HAL_RCC_USART1_CLK_ENABLE();
		
		GPIO_InitStruct.Pin       = USARTX1_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_NOPULL;
		GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = USARTX1_TX_AF ;

		HAL_GPIO_Init(USARTX1_TX_GPIO_PORT, &GPIO_InitStruct);

		/* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = USARTX1_RX_PIN;
		GPIO_InitStruct.Alternate = USARTX1_RX_AF;

		HAL_GPIO_Init(USARTX1_RX_GPIO_PORT, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(USART1_IRQn, 2, 1);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  vcom_IoDeInit( );
  /*##-1- Reset peripherals ##################################################*/
  USARTX_FORCE_RESET();
  USARTX_RELEASE_RESET();
   
  /*##-3- Disable the DMA #####################################################*/
  /* De-Initialize the DMA channel associated to reception process */
  if(huart->hdmarx != 0)
  {
    HAL_DMA_DeInit(huart->hdmarx);
  }
  /* De-Initialize the DMA channel associated to transmission process */
  if(huart->hdmatx != 0)
  {
    HAL_DMA_DeInit(huart->hdmatx);
  }  
  
  /*##-4- Disable the NVIC for DMA ###########################################*/
  HAL_NVIC_DisableIRQ(USARTX_DMA_TX_IRQn);
}

void vcom_IoInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct={0};
    /* Enable GPIO TX/RX clock */
  USARTX_TX_GPIO_CLK_ENABLE();
  USARTX_RX_GPIO_CLK_ENABLE();
    /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTX_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = USARTX_TX_AF;

  HAL_GPIO_Init(USARTX_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTX_RX_PIN;
  GPIO_InitStruct.Alternate = USARTX_RX_AF;

  HAL_GPIO_Init(USARTX_RX_GPIO_PORT, &GPIO_InitStruct);
}

void vcom_IoDeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure={0};
  
  USARTX_TX_GPIO_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  
  GPIO_InitStructure.Pin =  USARTX_TX_PIN ;
  HAL_GPIO_Init(  USARTX_TX_GPIO_PORT, &GPIO_InitStructure );
  
  GPIO_InitStructure.Pin =  USARTX_RX_PIN ;
  HAL_GPIO_Init(  USARTX_RX_GPIO_PORT, &GPIO_InitStructure ); 
}

void uarttors485_init(void)
{
	if(baudr<500)
	{	
		RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
		
		RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
		{
			Error_Handler();
		}
	}
	
	UartHandle1.Instance        = USART1;
	if(baudr==0)
	{
		UartHandle1.Init.BaudRate   = 9600;		
	}	
	else
	{
		UartHandle1.Init.BaudRate   = baudr;
	}
	
	if(databit==7)
	{
		if(pari==0)
		{
			UartHandle1.Init.WordLength = UART_WORDLENGTH_7B;				
	  }
	  else
	  {
			UartHandle1.Init.WordLength = UART_WORDLENGTH_8B;		
	  }		
	}
	else
	{
		if(pari==0)
		{
			UartHandle1.Init.WordLength = UART_WORDLENGTH_8B;				
	  }
	  else
	  {
			UartHandle1.Init.WordLength = UART_WORDLENGTH_9B;		
	  }
	}
	
  if(pari==1)
	{
		UartHandle1.Init.Parity     = UART_PARITY_ODD;
	}
	else if(pari==2)
	{
		UartHandle1.Init.Parity     = UART_PARITY_EVEN;
	}
	else
	{
		UartHandle1.Init.Parity     = UART_PARITY_NONE;
	}
  
	if(stopbit==1)
	{
		UartHandle1.Init.StopBits   = UART_STOPBITS_1_5;
	}
	else if(stopbit==2)
	{
		UartHandle1.Init.StopBits   = UART_STOPBITS_2;
	}
	else 
	{	
		UartHandle1.Init.StopBits   = UART_STOPBITS_1;
	}
	
  UartHandle1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle1.Init.Mode       = UART_MODE_TX_RX;
	
	if(HAL_UART_Init(&UartHandle1) != HAL_OK)
  {
    Error_Handler();
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/