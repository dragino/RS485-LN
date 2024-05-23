/******************************************************************************
  * @file    lora.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   lora API to drive the lora state Machine
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
#include "timeServer.h"
#include "lora.h"
#include "tiny_sscanf.h"
#include "flash_eraseprogram.h"
#include "version.h"
#include "stdlib.h"
#include "at.h"

bool fdr_flag=0;
bool sync_value;
bool group_mode;
uint8_t group_mode_id;
uint8_t command_mode;
uint16_t rxtime;
uint8_t txp_value;
uint8_t preamble_value;
uint8_t tx_spreading_value;
uint8_t rx_spreading_value;
uint8_t bandwidth_value;
uint8_t codingrate_value;
uint32_t tx_signal_freqence;
uint32_t rx_signal_freqence;
uint32_t APP_TX_DUTYCYCLE;
uint8_t intmode1;
uint32_t baudr;
uint8_t pari,stopbit,databit;
uint8_t group_id[8];
uint8_t commanddata[14];
uint8_t commandlen;
uint8_t crc_flags;
uint32_t cmddl;
uint8_t rxcrc_check;
uint32_t Automatic_join_network[1]={0x11};

static uint8_t config_count=0;
static uint32_t s_config[32]; //store config

/**
 *  lora Init
 */
void Config_Init (void)
{
	PPRINTF("\n\rDRAGINO RS485-LN PINGPONG\n\r");
	PPRINTF("Image Version: "AT_VERSION_STRING"\n\r");
	
	if(*(__IO uint32_t *)DATA_EEPROM_BASE==0x00)// !=0,Automatic join network
  {
		fdr_config();
		EEPROM_program(DATA_EEPROM_BASE,Automatic_join_network,1);					
    PPRINTF("Please set the parameters or reset Device to apply change\n\r");	
		fdr_flag=1;		
	}
	else if(*(__IO uint32_t *)DATA_EEPROM_BASE==0x12)
	{
		fdr_config();			
		EEPROM_program(DATA_EEPROM_BASE,Automatic_join_network,1);	
		NVIC_SystemReset();				
	}		
  else
  {
		EEPROM_Read_Config();
	}		
}

void fdr_config(void)
{
  group_id[0]='1';
  group_id[1]='2';
  group_id[2]='3';
  group_id[3]='4';
  group_id[4]='5';
  group_id[5]='6';
  group_id[6]='7';
  group_id[7]='8';	
	
	cmddl=400;
	rxtime=500;
	rxcrc_check=1;
  baudr=9600;	
	databit=8;
	intmode1=2;
	sync_value=1;
	preamble_value=8;
	txp_value=14;
	codingrate_value=1;
	APP_TX_DUTYCYCLE=600000;
	tx_spreading_value=12;
	rx_spreading_value=12;
	tx_signal_freqence=869000000;
	rx_signal_freqence=869000000;
	
	EEPROM_Store_Config();
	EEPROM_Read_Config();	
}

void EEPROM_Store_Config(void)
{
	s_config[config_count++]=APP_TX_DUTYCYCLE;

	s_config[config_count++]=tx_signal_freqence;

	s_config[config_count++]=rx_signal_freqence;
	
	s_config[config_count++]=(group_id[0]<<24) | (group_id[1]<<16) | (group_id[2]<<8) | group_id[3];
	
	s_config[config_count++]=(group_id[4]<<24) | (group_id[5]<<16) | (group_id[6]<<8) | group_id[7];
	
	s_config[config_count++]=(sync_value<<24) | (group_mode<<16) | (txp_value<<8) | preamble_value;
	
	s_config[config_count++]=(tx_spreading_value<<24) | (bandwidth_value<<16) | (rx_spreading_value<<8) | group_mode_id;

	s_config[config_count++]=cmddl;
	
	s_config[config_count++]=(codingrate_value<<16) | (intmode1<<8) | rxcrc_check;
	
	s_config[config_count++]=(crc_flags<<24) | (commandlen<<16) | (commanddata[0]<<8) | commanddata[1];
	
	s_config[config_count++]=(commanddata[2]<<24) | (commanddata[3]<<16) | (commanddata[4]<<8) | commanddata[5];
	
	s_config[config_count++]=(commanddata[6]<<24) | (commanddata[7]<<16) | (commanddata[8]<<8) | commanddata[9];
	
	s_config[config_count++]=(commanddata[10]<<24) | (commanddata[11]<<16) | (commanddata[12]<<8) | commanddata[13];
	
	s_config[config_count++]=baudr;
	
	s_config[config_count++]= (databit<<24)|(stopbit<<16)|(pari<<8)| command_mode;

	s_config[config_count++]= rxtime;
	
  EEPROM_program(EEPROM_USER_START_ADDR_CONFIG,s_config,config_count);//store config
	
	config_count=0;
}

void EEPROM_Read_Config(void)
{
	uint32_t star_address=0,r_config[16];
	
	star_address=EEPROM_USER_START_ADDR_CONFIG;
	for(int i=0;i<16;i++)
	{
	  r_config[i]=FLASH_read(star_address);
		star_address+=4;
	}
   	
	APP_TX_DUTYCYCLE=r_config[0];
	
	tx_signal_freqence=r_config[1];
	
	rx_signal_freqence=r_config[2];
	
	group_id[0]=(r_config[3]>>24)&0xFF;
	
	group_id[1]=(r_config[3]>>16)&0xFF;
	
	group_id[2]=(r_config[3]>>8)&0xFF;
	
	group_id[3]=r_config[3]&0xFF;	
	
	group_id[4]=(r_config[4]>>24)&0xFF;
	
	group_id[5]=(r_config[4]>>16)&0xFF;
	
	group_id[6]=(r_config[4]>>8)&0xFF;
	
	group_id[7]=r_config[4]&0xFF;		
	
	sync_value=(r_config[5]>>24)&0xFF;
	
	group_mode=(r_config[5]>>16)&0xFF;
	
	txp_value=(r_config[5]>>8)&0xFF;
	
	preamble_value=r_config[5]&0xFF;	
	
	tx_spreading_value=(r_config[6]>>24)&0xFF;
	
	bandwidth_value=(r_config[6]>>16)&0xFF;

	rx_spreading_value=(r_config[6]>>8)&0xFF;
	
	group_mode_id=r_config[6]&0xFF;

  cmddl=r_config[7];
	
	codingrate_value=(r_config[8]>>16)&0xFF;	
	
	intmode1=(r_config[8]>>8)&0xFF;	
	
	rxcrc_check=r_config[8]&0xFF;	
	
	crc_flags=(r_config[9]>>24)&0xFF;
	
	commandlen=(r_config[9]>>16)&0xFF;

  commanddata[0]=(r_config[9]>>8)&0xFF;
	
	commanddata[1]=r_config[9]&0xFF;
	
  commanddata[2]=(r_config[10]>>24)&0xFF;
	
  commanddata[3]=(r_config[10]>>16)&0xFF;
	
  commanddata[4]=(r_config[10]>>8)&0xFF;	
	
  commanddata[5]=r_config[10]&0xFF;
	
  commanddata[6]=(r_config[11]>>24)&0xFF;
	
  commanddata[7]=(r_config[11]>>16)&0xFF;
	
  commanddata[8]=(r_config[11]>>8)&0xFF;
	
  commanddata[9]=r_config[11]&0xFF;
	
  commanddata[10]=(r_config[12]>>24)&0xFF;
	
  commanddata[11]=(r_config[12]>>16)&0xFF;
	
  commanddata[12]=(r_config[12]>>8)&0xFF;
	
  commanddata[13]=r_config[12]&0xFF;

	baudr=r_config[13];
	
  databit=(r_config[14]>>24)&0xFF;
	
	stopbit=(r_config[14]>>16)&0xFF;
	
	pari=(r_config[14]>>8)&0xFF;	
	
	command_mode=r_config[14]&0xFF;	
	
	rxtime=r_config[15]&0xFFFF;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

