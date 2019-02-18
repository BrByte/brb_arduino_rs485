/*
 * core_gerador.h
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

#ifndef CORE_GERADOR_H_
#define CORE_GERADOR_H_

#include "BrbBase.h"
#include "BrbToneBase.h"
/**********************************************************************************************************************/
/* DEFINES */
/**********************************************************/
// #define RESERVED    0 /* RX0 */
// #define RESERVED    1 /* TX0 */
#define GERADOR_ZEROCROSS_PIN 2 /* INT4 - PWM */
// #define RESERVED    3 /* INT5 - PWM */
#define GERADOR_PARTIDA_PIN 4 /* PWM */
#define GERADOR_PARADA_PIN 5  /* PWM */
#define GERADOR_EXTRA_PIN 6   /* PWM */
#define BUZZER_PIN 7          /* PWM */
#define DHT_SENSOR_PIN 8      /* PWM */
#define GERADOR_SERVO_PIN 9   /* PWM */
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

#define SENSOR_VOLTAGE_DC_PIN A1
#define SENSOR_VOLTAGE_AC_PIN A2

/* Minimal power to be considered online */
#define GERADOR_TIMER_MIN_POWER 5.0

#define GERADOR_TIMER_FAIL_ALARM_MS 5000

#define GERADOR_TIMER_ZERO_WAIT_MS 2000

#define GERADOR_TIMER_START_WAIT_MS 15000

#define GERADOR_TIMER_START_DELAY_MS 5000
#define GERADOR_TIMER_START_CHECK_MS 15000
#define GERADOR_TIMER_START_RETRY_MAX 3

#define GERADOR_TIMER_STOP_DELAY_MS 10000
#define GERADOR_TIMER_STOP_CHECK_MS 30000
#define GERADOR_TIMER_STOP_RETRY_MAX 3

#define GERADOR_TIMER_CHECK_MS 5000

#define GERADOR_SERVO_BB_POS_OPEN 180
#define GERADOR_SERVO_BB_POS_CLOSE 120

#define GERADOR_HOURMETER_MAX 20

#define GERADOR_POWER_REVERSE 1

#ifdef GERADOR_POWER_REVERSE
#define GERADOR_POWER_ON LOW
#define GERADOR_POWER_OFF HIGH
#else
#define GERADOR_POWER_ON HIGH
#define GERADOR_POWER_OFF LOW
#endif
/**********************************************************************************************************************/
/* ENUMS */
/**********************************************************/
typedef enum
{
	GERADOR_FAILURE_NONE,
	GERADOR_FAILURE_RUNNING_WITHOUT_START,
	GERADOR_FAILURE_DOWN_WITHOUT_STOP,

	GERADOR_FAILURE_START_RETRY_LIMIT,

	GERADOR_FAILURE_STOP_RETRY_LIMIT,

} BrbGeradorFailureCode;

typedef enum
{
	GERADOR_STATE_NONE,
	GERADOR_STATE_START_INIT,
	GERADOR_STATE_START_DELAY,
	GERADOR_STATE_START_CHECK,

	GERADOR_STATE_RUNNING,
	GERADOR_STATE_FAILURE,

	GERADOR_STATE_STOP_INIT,
	GERADOR_STATE_STOP_DELAY,
	GERADOR_STATE_STOP_CHECK,

} BrbGeradorStateCode;
/**********************************************************************************************************************/
/* STRUCTS */
/**********************************************************/
typedef struct _BrbGeradorBase
{
	BrbBase *brb_base;
	BrbToneBase *tone_base;

	long delay;

	// BrbServo servo_bb;

	int pin_servo;
	int pin_partida;
	int pin_parada;

	int pin_extra;
	int pin_zerocross;

	int pin_sensor_ac;
	int pin_sensor_dc;

	struct
	{
		long cur;
		long last;
		long delay;

	} ms;

	struct
	{
		BrbGeradorStateCode code;		
		BrbGeradorFailureCode fail;

		long time;
		long delta;
		
		int retry;

	} state;

	struct
	{

		double battery;
		double gas;

		double power_ac;

		double zero_value;
		int zero_counter;
		int zero_delta;
		int zero_last;

		double load;

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

} BrbGeradorBase;
/**********************************************************************************************************************/
int BrbGeradorBase_Init(BrbGeradorBase *gerador_base);
int BrbGeradorBase_Loop(BrbGeradorBase *gerador_base);
int BrbGeradorBase_Save(BrbGeradorBase *gerador_base);
int BrbGeradorBase_HourmeterReset(BrbGeradorBase *gerador_base);

int BrbGeradorBase_Start(BrbGeradorBase *gerador_base);
int BrbGeradorBase_Stop(BrbGeradorBase *gerador_base);

int BrbGeradorBase_FailureConfirm(BrbGeradorBase *gerador_base);

const char *BrbGeradorBase_GetState(BrbGeradorBase *gerador_base);
const char *BrbGeradorBase_GetStateAction(BrbGeradorBase *gerador_base);
const char *BrbGeradorBase_GetStateButton(BrbGeradorBase *gerador_base);
const char *BrbGeradorBase_GetFailure(BrbGeradorBase *gerador_base);
/**********************************************************************************************************************/
#endif /* CORE_GERADOR_H_ */