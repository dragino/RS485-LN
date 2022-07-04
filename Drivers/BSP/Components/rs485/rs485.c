 /******************************************************************************
  * @file    RS485.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-April-2018
  * @brief   manages the sensors on the application
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
  
  /* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "rs485.h"
#include "timeServer.h"
#include "gpio_exti.h"
#include "iwdg.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

extern UART_HandleTypeDef UartHandle1;
extern uint8_t aRxBuffer[1];
extern bool receive_return_flag;
extern uint8_t receivr_return_issss;
extern uint8_t receive_returndatas[256];
extern uint8_t flags_command_temp;
extern uint8_t rxdatatemplen;
extern uint32_t cmddl;
extern uint8_t senddataBuff[30];
extern uint8_t sendBufferSize;
extern uint8_t rxdatatemplen;
extern uint8_t rxdata[256];
extern uint8_t commanddata[14];
extern uint8_t commandlen;
extern uint8_t testBuff[256];
extern uint16_t testBuffsize;
extern uint8_t rxdata_rxmode[255];
extern uint8_t maxlen_rxmode;

void En_485_Ioinit(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin = RS485_PIN ;
  HAL_GPIO_Init(RS485_GPIO_PORT, &GPIO_InitStruct);  
}

void CRC16_MODBUS(uint8_t *ptr,uint16_t len,uint8_t crcdata[])
{
    uint8_t i;
    uint16_t crc = 0xFFFF;
    if (len == 0) {
        len = 1;
    }
    while (len--) {
        crc ^= *ptr;
        for (i = 0; i<8; i++)
        {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
        ptr++;
    }
		
		crcdata[0]= crc & 0xff;		
    crcdata[1]=(crc>>8) & 0xff;
}

void send_command(uint8_t commanddata[],uint8_t commandlen,uint8_t crc)
{
	uint8_t result=0;
	
	for(uint8_t i=0;i<3;i++)
  {
		result=send_command_temp(commanddata,commandlen,crc);
		if(result!=0)
		{
		  break;
		}	
	}
}

uint8_t send_command_temp(uint8_t commanddata[],uint8_t commandlen,uint8_t crc)
{
	uint8_t data_status;
  uint32_t currentTime = TimerGetCurrentTime();	
	
	En_485_Ioinit();
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET); //set tx
	HAL_Delay(50);	
	HAL_UART_Transmit_IT(&UartHandle1, commanddata, commandlen);	
	while(UartHandle1.gState != HAL_UART_STATE_READY)
	{
		if(TimerGetElapsedTime(currentTime) >= 1000)
		{		
			break;
		}
	}

	rxdatatemplen=0;			
	receivr_return_issss=0;
	flags_command_temp=1;			
	receive_return_flag=1;
	
	HAL_UART_Receive_IT(&UartHandle1, (uint8_t *)aRxBuffer,1);  
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET); 	 //set rx	
	
  if(cmddl!=0)
	{
		for(uint8_t i=0;i<=cmddl/100;i++)    //waiting response
		{
			 HAL_Delay(100);
			 if(i%100==0)
			 {
				IWDG_Refresh();				 
			 }				 
		}
	}
	
	flags_command_temp=0;		
	receive_return_flag=0;	
	
	if(crc==1)
	{
		data_status=check_returndata();
	}
	else
	{
		data_status=3;
	}
	
	return data_status;
}

uint8_t check_returndata(void)
{
	uint8_t crcdata[2]={0x00,0x00};
   
	if((receivr_return_issss!=0)&&(receivr_return_issss<=255))
	{
		CRC16_MODBUS(receive_returndatas,receivr_return_issss-2,crcdata);
		if((receive_returndatas[receivr_return_issss-2]==crcdata[0])&&(receive_returndatas[receivr_return_issss-1]==crcdata[1]))
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else 
	{
		return 2;
	}
}

void Printf_information(void)
{
	PPRINTF("\r\n");
  PPRINTF("CMD      = ");		
	if(sendBufferSize==0)
	{
		PPRINTF("NULL\r\n");
	}
	else
	{
		for(uint8_t o=0;o<sendBufferSize;o++)
		{
			PPRINTF("%02x ",senddataBuff[o]);
			HAL_Delay(6);
		}
		PPRINTF("\r\n");
	}	

	PPRINTF("RETURN   = ");		
	if(rxdatatemplen==0)
	{	
		PPRINTF("NULL\r\n");		
	}
	else
	{
		for(uint8_t p=0;p<rxdatatemplen;p++)
		{
			PPRINTF("%02x ",rxdata[p]);
			HAL_Delay(6);	
		}	
		PPRINTF("\r\n");
	}
}

void printf_tx_cmd(uint8_t mode)
{
	PPRINTF("\r\n");
  PPRINTF("CMD      = ");	
	if(mode==1)
	{
		if(testBuffsize==0)
		{
			PPRINTF("NULL\r\n");
		}
		else
		{
			for(uint8_t o=0;o<testBuffsize;o++)
			{
				PPRINTF("%02x ",testBuff[o]);
				HAL_Delay(6);
			}
			PPRINTF("\r\n");
		}			
	}	
	else if(mode==2)
	{
		if(maxlen_rxmode==0)
		{
			PPRINTF("NULL\r\n");
		}
		else
		{
			for(uint8_t o=0;o<maxlen_rxmode;o++)
			{
				PPRINTF("%02x ",rxdata_rxmode[o]);
				HAL_Delay(6);
			}
			PPRINTF("\r\n");
		}				
	}
  else
  {		
		if(commandlen==0)
		{
			PPRINTF("NULL\r\n");
		}
		else
		{
			for(uint8_t o=0;o<commandlen;o++)
			{
				PPRINTF("%02x ",commanddata[o]);
				HAL_Delay(6);
			}
			PPRINTF("\r\n");
		}	
	}	
}
