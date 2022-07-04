/*******************************************************************************
 * @file    at.h
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   Header for driver at.c module
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
#ifndef __AT_H__
#define __AT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/*
 * AT Command Id errors. Note that they are in sync with ATError_description static array
 * in command.c
 */
typedef enum eATEerror
{
  AT_OK = 0,
  AT_ERROR,
  AT_PARAM_ERROR,
  AT_PARAM_NOT_Range,	
  AT_PARAM_FDR,		
  AT_BUSY_ERROR,
  AT_TEST_PARAM_OVERFLOW,
  AT_NO_NET_JOINED,
  AT_RX_ERROR,
  AT_MAX,
} ATEerror_t;

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* AT printf */
#define AT_PRINTF     PPRINTF

/* AT Command strings. Commands start with AT */
#define AT_RESET      "Z"
#define AT_FDR        "+FDR"
#define AT_FCU        "+FCU"
#define AT_FCD        "+FCD"
#define AT_TXP        "+TXP"
#define AT_SYNC       "+SYNC"
#define AT_PMB        "+PMB"
#define AT_TXCHS      "+TXCHS"
#define AT_TXSF       "+TXSF"
#define AT_RXCHS      "+RXCHS"
#define AT_RXSF       "+RXSF"
#define AT_BW         "+BW"
#define AT_CR         "+CR"
#define AT_GROUPMOD   "+GROUPMOD"
#define AT_GROUPID    "+GROUPID"
#define AT_TDC        "+TDC"
#define AT_INTMOD     "+INTMOD"
#define AT_BAUDR      "+BAUDR"
#define AT_DATABIT    "+DATABIT"
#define AT_PARITY     "+PARITY"
#define AT_STOPBIT    "+STOPBIT"
#define AT_SCHEDULE   "+SCHEDULE"
#define AT_CMDDL      "+CMDDL"
#define AT_CRCCHECK   "+CRCCHECK"
#define AT_CFGDEV     "+CFGDEV"
#define AT_RS485      "+RS485"
#define AT_MOD        "+MOD"
#define AT_SEND       "+SEND"
#define AT_VER        "+VER"
#define AT_CFG        "+CFG"

ATEerror_t at_return_ok(const char *param);
ATEerror_t at_return_error(const char *param);
ATEerror_t at_reset(const char *param);
ATEerror_t at_FDR(const char *param);
ATEerror_t at_UplinkCounter_get(const char *param);
ATEerror_t at_UplinkCounter_set(const char *param);
ATEerror_t at_DownlinkCounter_get(const char *param);
ATEerror_t at_DownlinkCounter_set(const char *param);
ATEerror_t at_TransmitPower_get(const char *param);
ATEerror_t at_TransmitPower_set(const char *param);
ATEerror_t at_syncword_get(const char *param);
ATEerror_t at_syncword_set(const char *param);
ATEerror_t at_preamble_get(const char *param);
ATEerror_t at_preamble_set(const char *param);
ATEerror_t at_txCHS_set(const char *param);
ATEerror_t at_txCHS_get(const char *param);
ATEerror_t at_txspreading_get(const char *param);
ATEerror_t at_txspreading_set(const char *param);
ATEerror_t at_rxCHS_set(const char *param);
ATEerror_t at_rxCHS_get(const char *param);
ATEerror_t at_rxspreading_get(const char *param);
ATEerror_t at_rxspreading_set(const char *param);
ATEerror_t at_bandwidth_get(const char *param);
ATEerror_t at_bandwidth_set(const char *param);
ATEerror_t at_codingrate_get(const char *param);
ATEerror_t at_codingrate_set(const char *param);
ATEerror_t at_groupmode_set(const char *param);
ATEerror_t at_groupmode_get(const char *param);
ATEerror_t at_groupid_set(const char *param);
ATEerror_t at_groupid_get(const char *param);
ATEerror_t at_TDC_set(const char *param);
ATEerror_t at_TDC_get(const char *param);
ATEerror_t at_INTMOD_set(const char *param);
ATEerror_t at_INTMOD_get(const char *param);
ATEerror_t at_baudrate_set(const char *param);
ATEerror_t at_baudrate_get(const char *param);
ATEerror_t at_databit_get(const char *param);
ATEerror_t at_databit_set(const char *param);
ATEerror_t at_parity_get(const char *param);
ATEerror_t at_parity_set(const char *param);
ATEerror_t at_stopbit_get(const char *param);
ATEerror_t at_stopbit_set(const char *param);
ATEerror_t at_schedule_set(const char *param);
ATEerror_t at_schedule_get(const char *param);
ATEerror_t at_commanddl_set(const char *param);
ATEerror_t at_commanddl_get(const char *param);
ATEerror_t at_crccheckmod_set(const char *param);
ATEerror_t at_crccheckmod_get(const char *param);
ATEerror_t at_cfgdev(const char *param);
ATEerror_t at_modbus_run(const char *param);
ATEerror_t at_mode_set(const char *param);
ATEerror_t at_mode_get(const char *param);
ATEerror_t at_Send(const char *param);
ATEerror_t at_version_get(const char *param);
ATEerror_t at_CFG_run(const char *param);

#ifdef __cplusplus
}
#endif

#endif /* __AT_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
