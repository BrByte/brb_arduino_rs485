/*
 * core_pdu.h
 *
 *  Created on: 2019-02-18
 *      Author: Luiz Fernando Souza Softov <softov@brbyte.com>
 *      Author: Guilherme Amorim de Oliveira Alves <guilherme@brbyte.com>
 *
 * Copyright (c) 2018 BrByte Software (Oliveira Alves & Amorim LTDA)
 * Todos os direitos reservados. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CORE_PDU_H_
#define CORE_PDU_H_

/**********************************************************************************************************************/

// #define RESERVED    0 /* RX0 */
// #define RESERVED    1 /* TX0 */
#define PDU_ZEROCROSS_POWER_PIN   2 /* INT4 - PWM */
#define PDU_ZEROCROSS_AUX_PIN   3 /* INT5 - PWM */
#define RESERVED 4 /* PWM */
#define RESERVED 5 /* PWM */
#define RESERVED 6 /* PWM */
#define BUZZER_PIN      7 /* PWM */
#define DHT_SENSOR_PIN  8 /* PWM */
#define RESERVED        9 /* PWM */

// #define RESERVED    10 /* PCINT 4 */
// #define RESERVED    11 /* PCINT 5 */
// #define RESERVED    12 /* PCINT 6 */
// #define RESERVED    13 /* PCINT 7 */
#define RS485_DI_PIN 14 /* PCINT10 - TX3 */
#define RS485_RO_PIN 15 /* PCINT9 - RX3 */
// #define RESERVED    16 /* TX2 */
// #define RESERVED    17 /* RX2 */
// #define RESERVED    18 /* INT3 - TX1 */
// #define RESERVED    19 /* INT2 - RX1 */
// #define RESERVED    20 /* INT0 - SCL */
// #define RESERVED    21 /* INT1 - SDA */
#define RS485_REDE_PIN 22 /* TOGGLE PIN (RE + DE) */
// #define RESERVED    23 /* */
// #define RESERVED    24 /* */
// #define RESERVED    25 /* */
#define BTN_PIN_SELECT 26 /* */
#define BTN_PIN_NEXT 27   /* */
#define BTN_PIN_PREV 28   /* */
// #define RESERVED    29 /* */
// #define RESERVED    30 /* */
// #define RESERVED    31 /* */
// #define RESERVED    32 /* */
// #define RESERVED    33 /* */
// #define RESERVED    34 /* */
// #define RESERVED    35 /* */
// #define RESERVED    36 /* */
// #define RESERVED    37 /* */
// #define RESERVED    38 /* */
// #define RESERVED    39 /* */
// #define RESERVED    40 /* */
// #define RESERVED    41 /* */
// #define RESERVED    42
// #define RESERVED    43 /*  */
// #define RESERVED    44 /*  */
// #define RESERVED    45 /* PWM */
#define TFT_LED 46  /* PWM */
#define TFT_CS 47   /*  */
#define TFT_DC 48   /*  */
#define TFT_RST 49  /* */
#define TFT_MISO 50 /* PCINT3 - MISO */
#define TFT_MOSI 51 /* PCINT2 - MOSI */
#define TFT_CLK 52  /* PCINT1 - SCK */
// #define RESERVED     53 /* PCINT0 - SS */

#define SENSOR_AC_POWER_PIN A1
#define SENSOR_AC_AUX_PIN A2

#ifdef PDU_POWER_REVERSE
#define PDU_POWER_ON LOW
#define PDU_POWER_OFF HIGH
#else
#define PDU_POWER_ON HIGH
#define PDU_POWER_OFF LOW
#endif

#define PDU_TIMER_FAIL_ALARM_MS 5000

#define PDU_TIMER_ZERO_WAIT_MS 2000

#define PDU_POWER_MIN_VALUE 5
#define PDU_POWER_MIN_HZ 10

#define PDU_AUX_MIN_VALUE 5
#define PDU_AUX_MIN_HZ 10

/**********************************************************************************************************************/
/* ENUMS */
/**********************************************************/
typedef enum
{
	PDU_FAILURE_NONE,
	PDU_FAILURE_POWER_DOWN,

} BrbPDUFailureCode;

typedef enum
{
	PDU_STATE_NONE,
	PDU_STATE_RUNNING,
	PDU_STATE_FAILURE

} BrbPDUStateCode;
/**********************************************************************************************************************/
/* STRUCTS */
/**********************************************************/
typedef struct _BrbPDUBase
{
	BrbBase *brb_base;
	BrbToneBase *tone_base;

	long delay;

	BrbZeroCross zero_power;
	BrbZeroCross zero_aux;

	BrbSensorVoltage sensor_power;
	BrbSensorVoltage sensor_aux;

	struct
	{
		long cur;
		long last;
		long delay;

	} ms;
	
	struct
	{
		int ms_delta;
		int ms_last;

	} zerocross;

	struct
	{
		BrbPDUStateCode code;		
		BrbPDUFailureCode fail;

		long time;
		long delta;
		
		int retry;

	} state;

	struct
	{
		long hourmeter_ms;
		long hourmeter_sec;

	} info;

	/* data is persistent */
	struct
	{
		long hourmeter_total;
		long hourmeter_time;
		long hourmeter_reset;

		long reserved1;
		long reserved2;
		long reserved3;

	} data;

	struct
	{
		unsigned int foo : 1;
	} flags;

} BrbPDUBase;
/**********************************************************************************************************************/
int BrbPDUBase_Init(BrbPDUBase *pdu_base);
int BrbPDUBase_Loop(BrbPDUBase *pdu_base);
int BrbPDUBase_Save(BrbPDUBase *pdu_base);
int BrbPDUBase_HourmeterReset(BrbPDUBase *pdu_base);

int BrbPDUBase_Start(BrbPDUBase *pdu_base);
int BrbPDUBase_Stop(BrbPDUBase *pdu_base);

int BrbPDUBase_FailureConfirm(BrbPDUBase *pdu_base);

const char *BrbPDUBase_GetState(BrbPDUBase *pdu_base);
const char *BrbPDUBase_GetStateAction(BrbPDUBase *pdu_base);
const char *BrbPDUBase_GetStateButton(BrbPDUBase *pdu_base);
const char *BrbPDUBase_GetFailure(BrbPDUBase *pdu_base);
/**********************************************************************************************************************/
#endif /* CORE_PDU_H_ */