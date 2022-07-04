/*******************************************************************************
 * @file    at.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   at command API
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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "at.h"
#include "utilities.h"
#include "lora.h"
#include "radio.h"
#include "vcom.h"
#include "tiny_sscanf.h"
#include "version.h"
#include "hw_msp.h"
#include "flash_eraseprogram.h"
#include "command.h"
#include "gpio_exti.h"
#include "timeServer.h"
#include "rs485.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/**
 * @brief Max size of the data that can be received
 */
extern uint8_t testBuff[256];
extern uint16_t testBuffsize;
extern bool test_uplink_status;
extern bool uplink_data_status;
extern bool fdr_flag;
extern bool sync_value;
extern bool group_mode;
extern uint8_t sendtx_flag;
extern uint8_t group_mode_id;
extern uint8_t txp_value;
extern uint8_t preamble_value;
extern uint8_t tx_spreading_value;
extern uint8_t rx_spreading_value;
extern uint8_t bandwidth_value;
extern uint8_t codingrate_value;
extern uint32_t tx_signal_freqence;
extern uint32_t rx_signal_freqence;
extern uint32_t APP_TX_DUTYCYCLE;
extern uint8_t intmode1;
extern uint32_t baudr;
extern uint8_t pari,stopbit,databit;
extern uint8_t group_id[8];
extern uint32_t uplinkcount;
extern uint32_t downlinkcount;
extern bool sending_flag;
extern uint8_t commanddata[14];
extern uint8_t commandlen;
extern uint8_t crc_flags;
extern uint32_t cmddl;
extern uint8_t rxcrc_check;
extern bool redled_flash_flag;
extern UART_HandleTypeDef UartHandle1;
extern uint8_t aRxBuffer[1];
extern uint16_t maxlen;
extern uint8_t rxdata_maxlen[550];
extern bool cfgdev_flags;
extern uint8_t command_mode;
extern uint16_t rxtime;

extern uint32_t APP_TX_DUTYCYCLE;
extern TimerEvent_t TxTimer;

static uint8_t cmp_char(const char *param,int len);

/* Exported functions ------------------------------------------------------- */

ATEerror_t at_return_ok(const char *param)
{
  return AT_OK;
}

ATEerror_t at_return_error(const char *param)
{
  return AT_ERROR;
}

ATEerror_t at_reset(const char *param)
{
  NVIC_SystemReset();
  return AT_OK;
}

ATEerror_t at_FDR(const char *param)
{
	EEPROM_erase_one_address(DATA_EEPROM_BASE);
	EEPROM_erase_lora_config();
	AT_PRINTF("OK\n\r");
	HAL_Delay(50);
	NVIC_SystemReset();
  return AT_OK;
}

ATEerror_t at_TransmitPower_get(const char *param)
{
	PPRINTF("%d\r\n",txp_value);

  return AT_OK;
}

ATEerror_t at_TransmitPower_set(const char *param)
{
	uint8_t temp=0;
	
	if (tiny_sscanf(param, "%d", &temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(temp<=20)
	{
		txp_value=temp;
	}
	else
	{
		return AT_PARAM_ERROR;		
	}
	
  return AT_OK;
}

ATEerror_t at_syncword_set(const char *param)
{
	uint8_t temp=0;
	
	if (tiny_sscanf(param, "%d", &temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(temp<=1)
	{
		sync_value=temp;
	}
	else
	{
		return AT_PARAM_ERROR;		
	}

  return AT_OK;
}

ATEerror_t at_syncword_get(const char *param)
{
	PPRINTF("%d\r\n",sync_value);
		
	return AT_OK;		
}

ATEerror_t at_preamble_set(const char *param)
{
	uint8_t temp=0;
	
	if (tiny_sscanf(param, "%d", &temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	preamble_value=temp;
	
	return AT_OK;		
}

ATEerror_t at_preamble_get(const char *param)
{
	PPRINTF("%d\r\n",preamble_value);
		
	return AT_OK;		
}

ATEerror_t at_txCHS_set(const char *param)
{
	uint32_t fre;
	if (tiny_sscanf(param, "%lu", &fre) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if((100000000<fre&&fre<999999999)||fre==0)
	{
	  tx_signal_freqence=fre;
	}
	else 
	{
		return AT_PARAM_ERROR;
	}
	
	return AT_OK;
}

ATEerror_t at_txCHS_get(const char *param)
{ 
	PPRINTF("%u\r\n",tx_signal_freqence);
	return AT_OK;
}

ATEerror_t at_txspreading_set(const char *param)
{
	uint8_t temp=0;
	
	if (tiny_sscanf(param, "%d", &temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if((temp>=7)&&(temp<=12))
	{
		tx_spreading_value=temp;
	}
	else
	{
		return AT_PARAM_ERROR;		
	}
	
	return AT_OK;	
}
	
ATEerror_t at_txspreading_get(const char *param)
{
	PPRINTF("%d\r\n",tx_spreading_value);	
		
	return AT_OK;		
}

ATEerror_t at_rxCHS_set(const char *param)
{
	uint32_t fre;
	if (tiny_sscanf(param, "%lu", &fre) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if((100000000<fre&&fre<999999999)||fre==0)
	{
	  rx_signal_freqence=fre;
		PPRINTF("Attention:Take effect after ATZ\r\n");	
	}
	else 
	{
		return AT_PARAM_ERROR;
	}
	
	return AT_OK;
}

ATEerror_t at_rxCHS_get(const char *param)
{ 
	PPRINTF("%u\r\n",rx_signal_freqence);
	return AT_OK;
}

ATEerror_t at_rxspreading_set(const char *param)
{
	uint8_t temp=0;
	
	if (tiny_sscanf(param, "%d", &temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if((temp>=7)&&(temp<=12))
	{
		rx_spreading_value=temp;
  	PPRINTF("Attention:Take effect after ATZ\r\n");	
	}
	else
	{
		return AT_PARAM_ERROR;		
	}
	
	return AT_OK;	
}
	
ATEerror_t at_rxspreading_get(const char *param)
{
	PPRINTF("%d\r\n",rx_spreading_value);	
		
	return AT_OK;		
}
	
ATEerror_t at_bandwidth_set(const char *param)
{
	uint8_t temp=0;
	
	if (tiny_sscanf(param, "%d", &temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(temp<=2)
	{
		bandwidth_value=temp;
  	PPRINTF("Attention:Take effect after ATZ\r\n");	
	}
	else
	{
		return AT_PARAM_ERROR;		
	}
	
	return AT_OK;	
}

ATEerror_t at_bandwidth_get(const char *param)
{
	PPRINTF("%d\r\n",bandwidth_value);
	
	return AT_OK;			
}

ATEerror_t at_codingrate_set(const char *param)
{
	uint8_t temp=0;
	
	if (tiny_sscanf(param, "%d", &temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if((temp>=1)&&(temp<=4))
	{
		codingrate_value=temp;
	}
	else
	{
		return AT_PARAM_ERROR;		
	}
	
	return AT_OK;		
}

ATEerror_t at_codingrate_get(const char *param)
{
	PPRINTF("%d\r\n",codingrate_value);		
		
	return AT_OK;		
}

ATEerror_t at_UplinkCounter_get(const char *param)
{
  PPRINTF("%u\r\n",uplinkcount);

  return AT_OK;
}

ATEerror_t at_UplinkCounter_set(const char *param)
{
	uint8_t temp=0;
	
	if (tiny_sscanf(param, "%d", &temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	uplinkcount=temp;	

  return AT_OK;
}

ATEerror_t at_DownlinkCounter_get(const char *param)
{
  PPRINTF("%u\r\n",downlinkcount);

  return AT_OK;
}

ATEerror_t at_DownlinkCounter_set(const char *param)
{
	uint8_t temp=0;
	
	if (tiny_sscanf(param, "%d", &temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	downlinkcount=temp;	

  return AT_OK;
}

ATEerror_t at_version_get(const char *param)
{
  AT_PRINTF(AT_VERSION_STRING);
	
  return AT_OK;
}

ATEerror_t at_CFG_run(const char *param)
{
	if(sending_flag==1)
	{
		return AT_BUSY_ERROR;
	}
	
	PPRINTF("\n\rStop Tx events,Please wait for all configurations to print\r\n");	
	PPRINTF("\r\n");
	
	TimerStop(&TxTimer);	
	
	printf_all_config();

	if(fdr_flag==0)
	{	
		PPRINTF("\n\rStart Tx events\r\n");
		TimerStart(&TxTimer);	
	}
	
	return AT_OK;	
}

ATEerror_t at_TDC_set(const char *param)
{ 
	uint32_t txtimeout;
	
	if (tiny_sscanf(param, "%lu", &txtimeout) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(txtimeout<6000)
	{
		PRINTF("TDC setting must be more than 6S\n\r");
		APP_TX_DUTYCYCLE=6000;
		return AT_PARAM_ERROR;
	}
	
	APP_TX_DUTYCYCLE=txtimeout;
	
	return AT_OK;
}

ATEerror_t at_TDC_get(const char *param)
{ 
	PPRINTF("%u\r\n",APP_TX_DUTYCYCLE);
	return AT_OK;
}

ATEerror_t at_INTMOD_set(const char *param)
{ 
	uint8_t interrputmode;
	if (tiny_sscanf(param, "%d", &interrputmode) != 1)
  {
    return AT_PARAM_ERROR;
  }
	if (interrputmode<=3)
  {
    intmode1=interrputmode;		
	}
	else
	{
    return AT_PARAM_ERROR;
	}

	GPIO_EXTI_IoInit(intmode1);
	
	return AT_OK;
}

ATEerror_t at_INTMOD_get(const char *param)
{ 
	AT_PRINTF("%d\r\n",intmode1);
	return AT_OK;
}

ATEerror_t at_baudrate_set(const char *param)
{
	uint32_t baudrate;
	if (tiny_sscanf(param, "%d", &baudrate) != 1)
  {
    return AT_PARAM_ERROR;
  }	
	
	baudr=baudrate;
	uarttors485_init();
	
	return AT_OK;
}

ATEerror_t at_baudrate_get(const char *param)
{
	PPRINTF("%u\r\n",baudr);
  return AT_OK;	
}	

ATEerror_t at_databit_set(const char *param)
{
	uint8_t databit_temp;

	if(tiny_sscanf(param, "%d", &databit_temp) != 1)
	{
		  return AT_PARAM_ERROR;
	}

	if((databit_temp>=7)&&(databit_temp<=8))
	{
			databit=databit_temp;
			uarttors485_init();	
	}
	else 
	{
		PPRINTF("Data bit must be 7,8\r\n");
		return AT_PARAM_ERROR;		
	}
	
  return AT_OK;		
}

ATEerror_t at_databit_get(const char *param)
{
 	PPRINTF("%d\r\n",databit); 
  return AT_OK;		
}

ATEerror_t at_parity_set(const char *param)
{
	uint8_t parity;
	if (tiny_sscanf(param, "%d", &parity) != 1)
  {
    return AT_PARAM_ERROR;
  }	
	
	if(parity<=2)
	{
		pari=parity;
		uarttors485_init();
	}
	
	else
	{
    return AT_PARAM_ERROR;
	}
	
	return AT_OK;
}

ATEerror_t at_parity_get(const char *param)
{
	PPRINTF("%d\r\n",pari);
  return AT_OK;	
}	

ATEerror_t at_stopbit_set(const char *param)
{
	uint8_t stopbit_a;

	if(tiny_sscanf(param, "%d", &stopbit_a) != 1)
	{
		  return AT_PARAM_ERROR;
	}

	if(stopbit_a<=2)
	{
		stopbit=stopbit_a;
		uarttors485_init();		
	}
	else 
	{
		PPRINTF("Stop bit must be 0,1 or 2\r\n");
		return AT_PARAM_ERROR;		
	}
	
  return AT_OK;		
}

ATEerror_t at_stopbit_get(const char *param)
{
 	PPRINTF("%d\r\n",stopbit); 
  return AT_OK;		
}

ATEerror_t at_crccheckmod_set(const char *param)
{
	uint8_t crc_check_temp;
	if (tiny_sscanf(param, "%d", &crc_check_temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if (crc_check_temp<=1)
  {
    rxcrc_check=crc_check_temp;		
	}
	else
	{
    return AT_PARAM_ERROR;
	}
	
	return AT_OK;		
}

ATEerror_t at_crccheckmod_get(const char *param)
{
	PPRINTF("%d\r\n",rxcrc_check);
	return AT_OK;		
}

ATEerror_t at_schedule_set(const char *param)
{
  char *p[14];
  char temp[2]="";
  char len[1];
	uint8_t k=0;
	uint8_t l=0;
	uint8_t crc_flag_temp;
  uint8_t crcdata[2]={0x00,0x00};

	if(cmp_char(param,strlen(param))==0)
	{
		PPRINTF("Please enter hexadecimal characters\r\n");
	  return AT_PARAM_ERROR;	
	}
	else
	{
		crc_flags=0;
		for(uint8_t l=0;l<14;l++)
		{
			commanddata[l]=0x00;
		}
		
		for(int i=0;i<strlen(param);i++)
		{
		 if((param[i]==' ')||(param[i]==','))
		 {
			 p[k++]=temp;
			 sscanf(p[k-1],"%hhx",&commanddata[k-1]);				 
			 l=0;
		 }
		 else
		 {
			 temp[l++]=param[i];		 
		 }
		}	 
			 
		commandlen=cmp_char(param,strlen(param));
			 
		len[0]=param[strlen(param)-1];
		crc_flag_temp=len[0]-'0';
			 
		if(crc_flag_temp==0)
		{
		 crc_flags=crc_flag_temp;
		}		 
		else if(crc_flag_temp==1)
		{
			CRC16_MODBUS(commanddata,commandlen,crcdata);
		  commanddata[commandlen]=crcdata[0];
			commanddata[commandlen+1]=crcdata[1];	
			commandlen=commandlen+2;			 
			crc_flags=crc_flag_temp;
		}
   }
	
	 return AT_OK;		
}

ATEerror_t at_schedule_get(const char *param)
{
	if(commandlen==0)
	{
		PPRINTF("0,0\r\n");		
	}
	else
	{
		if(crc_flags==0)
		{
			for(uint8_t i=0;i<commandlen;i++)
			{
				PPRINTF("%02x ",commanddata[i]);
			}
			PPRINTF(",0\r\n");
		}
		else
		{
			for(uint8_t i=0;i<commandlen-2;i++)
			{
				PPRINTF("%02x ",commanddata[i]);
			}
			PPRINTF(",1\r\n");			
		}
	}
	return AT_OK;		
}

ATEerror_t at_commanddl_set(const char *param)
{
	uint32_t detimeout;
	
	if (tiny_sscanf(param, "%lu", &detimeout) != 1)
  {
    return AT_PARAM_ERROR;
  }	
	
	cmddl=detimeout;
	
	return AT_OK;		
}

ATEerror_t at_commanddl_get(const char *param)
{
	PPRINTF("%u\r\n",cmddl);
	return AT_OK;		
}

ATEerror_t at_cfgdev(const char *param)
{
  char *p[30];
  char temp[2]="";
  char len[1];
	uint8_t k=0;
	uint8_t l=0;
	uint8_t crc_flag_temp;
  uint8_t crcdata[2]={0x00,0x00};

	if(sending_flag==1)
	{
		return AT_BUSY_ERROR;
	}
	
	if(cmp_char(param,strlen(param))==0)
	{
		PPRINTF("Please enter hexadecimal characters\r\n");
	  return AT_PARAM_ERROR;	
	}
	else
	{	
		for(int i=0;i<strlen(param);i++)
		{
		 if((param[i]==' ')||(param[i]==','))
		 {
			 p[k++]=temp;
			 sscanf(p[k-1],"%hhx",&testBuff[k-1]);				 
			 l=0;
		 }
		 else
		 {
			 temp[l++]=param[i];		 
		 }
		}	 
			 
		testBuffsize=cmp_char(param,strlen(param));
			 
		len[0]=param[strlen(param)-1];
		crc_flag_temp=len[0]-'0';
			 
		if(crc_flag_temp==1)
		{
		  CRC16_MODBUS(testBuff,testBuffsize,crcdata);
			testBuff[testBuffsize]=crcdata[0];
			testBuff[testBuffsize+1]=crcdata[1];	
			testBuffsize=testBuffsize+2;			 
		}
		
		if(command_mode==0)
		{
			redled_flash_flag=1;
			sendtx_flag=1;
			uplink_data_status=1;
		}
   }
	
	 return AT_OK;	
}

ATEerror_t at_modbus_run(const char *param)
{
	char *p[30];
	char temp[2]="";
  char len[1];
	uint8_t k=0;
	uint8_t l=0;
	uint8_t aaaa=0;
  uint8_t command[30];
	uint8_t crc_flag;
	uint8_t datalen;
  uint8_t crcdata[2]={0x00,0x00};
  uint32_t currentTime = TimerGetCurrentTime();	
	
	if(cmp_char(param,strlen(param))==0)
	{
		return AT_PARAM_ERROR;	
	}
		
	else
	{
	   for(int i=0;i<strlen(param);i++)
	   {
			 if((param[i]==' ')||(param[i]==','))
			 {
				 if(aaaa==0)
				 {
				  p[k++]=temp;
					sscanf(p[k-1],"%hhx",&command[k-1]);
					for(uint8_t oo=0;oo<2;oo++)
					{
						temp[oo]=0x00;
					}		
				 }				
				 if(param[i]==',')
				 {
					if(aaaa==1)
					{
						crc_flag=param[i-1]-'0';
					}
					aaaa++;
				 }
					
				 for(uint8_t oo=0;oo<2;oo++)
				 {
					temp[oo]=0x00;
				 }						 
				 l=0;
			 }
			 else
			 {
				 temp[l++]=param[i];		 
			 }
		 }	 
		 
		 datalen=cmp_char(param,strlen(param));
		 len[0]=param[strlen(param)-1];
		 crc_flag=len[0]-'0';
		 
		 PPRINTF("AT+MODBUS=");		 
		 
	   if(crc_flag==1)
	   {			 
		   CRC16_MODBUS(command,datalen,crcdata);
			 command[datalen]=crcdata[0];
			 command[datalen+1]=crcdata[1];	
		   datalen=datalen+2;
		 }
		 
		 for(int i=0;i<datalen;i++)
		 {
			PPRINTF("%02x ",command[i]);
		 }
		 
		 PPRINTF(",%d\r\n",crc_flag);	
		 
  	 PPRINTF("RETURN DATA:");				 
		 En_485_Ioinit();
		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);//stm tx
		 HAL_Delay(50);	
	   HAL_UART_Transmit_IT(&UartHandle1, command, datalen);	
		 while(UartHandle1.gState != HAL_UART_STATE_READY)
	   {
		   if(TimerGetElapsedTime(currentTime) >= 1000)
		   {		
			   break;
		   }
	   }
			
		HAL_UART_Receive_IT(&UartHandle1, (uint8_t *)aRxBuffer,1);  
		cfgdev_flags=1;		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//stm rx 
		HAL_Delay(cmddl);	
		cfgdev_flags=0;			
		PPRINTF("\r\n");	
		
		for(uint16_t j=0;j<maxlen;j++)
		{
			PPRINTF("%02x ",rxdata_maxlen[j]);
			HAL_Delay(8);
			rxdata_maxlen[j]=0x00;
		}
		maxlen=0;
		PPRINTF("\r\n");			 
  }
	return AT_OK;		
}

ATEerror_t at_groupmode_set(const char *param)
{
	uint8_t temp=0,temp_id=0;

	if (tiny_sscanf(param, "%d,%d", &temp, &temp_id) != 2)
  {	
		if (tiny_sscanf(param, "%d", &temp) != 1)
		{
			return AT_PARAM_ERROR;
		}
	}
	
	if((temp<=1)&&(temp_id<=8))
	{
		group_mode=temp;
		group_mode_id=temp_id;
	}
	else
	{
		PPRINTF("Value1 must be less than or equal to 1, and Value2 must be less than or equal to 8\r\n");
		return AT_PARAM_ERROR;		
	}
	
	return AT_OK;		
}

ATEerror_t at_groupmode_get(const char *param)
{
	AT_PRINTF("%d,%d\r\n",group_mode,group_mode_id);	
	
	return AT_OK;			
}

ATEerror_t at_groupid_set(const char *param)
{
	if(strlen(param)!=8)
	{
		PPRINTF("Group ID length must be 8 characters\r\n");
		return AT_PARAM_ERROR;
	}
	else
	{
		for(uint8_t j=0;j<8;j++)
		{
			group_id[j]=0x00;
		}
		for(uint8_t i=0;i<strlen(param);i++)
		{
			group_id[i]=param[i];
		}
	}
	
	return AT_OK;	
}

ATEerror_t at_groupid_get(const char *param)
{
  for(uint8_t i=0;i<8;i++)
	{
		if(group_id[i]!='\0')
		{
			PPRINTF("%c",group_id[i]);
		}
	}	
	PPRINTF("\r\n");
	
	return AT_OK;		
}

ATEerror_t at_mode_set(const char *param)
{
	uint8_t temp=0;
  uint16_t temp_time=0;
	
	if (tiny_sscanf(param, "%d,%d", &temp,&temp_time) != 2)
  {	
		if (tiny_sscanf(param, "%d", &temp) != 1)
		{
			return AT_PARAM_ERROR;
		}
	}
	
	if(temp<=1)
	{
		command_mode=temp;
		if(command_mode==1)
		{
			En_485_Ioinit();
			HAL_UART_Receive_IT(&UartHandle1, (uint8_t *)aRxBuffer,1);  
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);//stm rx  
		}
		rxtime=temp_time;
	}
	
	return AT_OK;	
}

ATEerror_t at_mode_get(const char *param)
{
	if(command_mode==0)
	{
		PPRINTF("%d\r\n",command_mode);
	}
	else
	{
		PPRINTF("%d,%d\r\n",command_mode,rxtime);
	}
	return AT_OK;	
}

ATEerror_t at_Send(const char *param)
{
  const char *buf= param;
  unsigned char bufSize= strlen(param);
  unsigned size=0;
  char hex[3];

	if(sending_flag==1)
	{
		return AT_BUSY_ERROR;
	}
	
  hex[2] = 0;
  while ((size < 255) && (bufSize > 1))
  {
    hex[0] = buf[size*2];
    hex[1] = buf[size*2+1];
    if (tiny_sscanf(hex, "%hhx", &testBuff[size]) != 1)
    {
      return AT_PARAM_ERROR;
    }
    size++;
    bufSize -= 2;
  }
	
  if (bufSize != 0)
  {
    return AT_PARAM_ERROR;
  }
	
	testBuffsize=size;
	redled_flash_flag=1;
	uplink_data_status=1;
  test_uplink_status=1;
	
  return AT_OK;
}

static uint8_t cmp_char(const char *param,int len)
{
	uint8_t j=0;
	for(uint8_t i=0;i<len;i++)
	{
		if(((param[i]>='0')&&(param[i]<='9'))||((param[i]>='a')&&(param[i]<='f'))||((param[i]>='A')&&(param[i]<='F'))||(param[i]==' ')||(param[i]==','))
		{
       if(param[i]==' ')
			 {
				j++;
			 }
		}
		else 
		{
			return 0;
		}
	}
	
	return j+1;
}
