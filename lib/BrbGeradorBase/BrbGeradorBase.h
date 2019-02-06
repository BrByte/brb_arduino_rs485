/*
 * BrbGeradorBase.h
 *
 *  Created on: 2019-01-27
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

#ifndef BRB_GERADOR_BASE_H_
#define BRB_GERADOR_BASE_H_
/**********************************************************************************************************************/
#include <BrbBase.h>
#include <BrbToneBase.h>
/****************************************************************************************************/
#define BRB_GERADOR_PARTIDA_MS 5000

#define BRB_GERADOR_SERVO_BB_POS_OPEN 180
#define BRB_GERADOR_SERVO_BB_POS_CLOSE 120

#define GERADOR_POWER_REVERSE 1

#ifdef GERADOR_POWER_REVERSE
#define GERADOR_POWER_ON LOW
#define GERADOR_POWER_OFF HIGH
#else
#define GERADOR_POWER_ON HIGH
#define GERADOR_POWER_OFF LOW
#endif

typedef enum 
{
	GERADOR_STATE_NONE,
	GERADOR_STATE_STARTING,
	GERADOR_STATE_RUNNING,
	GERADOR_STATE_STOPPING,

} BrbGeradorStateCode;
/**********************************************************************************************************************/
typedef struct _BrbGeradorBase
{
	BrbBase *brb_base;
	BrbToneBase *tone_base;
	BrbMicroScript *script;

	BrbGeradorStateCode state;
	
	long delay;

	// BrbServo servo_bb;

	int pin_servo;
	int pin_partida;
	int pin_parada;

	int pin_sensor_ac;
	int pin_sensor_dc;

	struct {
		
		double battery;
		double gas;
		double power;
		double load;

		long hourmeter_ms;
		long hourmeter_sec;

	} info;

	/* data is persistent */
	struct {

		long hourmeter_time;
		long hourmeter_reset;

		long reserved1;
		long reserved2;
		long reserved3;

	} data;

	struct {
		unsigned int foo:1;
	} flags;

} BrbGeradorBase;
/**********************************************************************************************************************/
int BrbGeradorBase_Init(BrbGeradorBase *gerador_base);
int BrbGeradorBase_Loop(BrbGeradorBase *gerador_base);
int BrbGeradorBase_Save(BrbGeradorBase *gerador_base);
int BrbGeradorBase_HourmeterReset(BrbGeradorBase *gerador_base);

int BrbGeradorBase_Start(BrbGeradorBase *gerador_base);
int BrbGeradorBase_Stop(BrbGeradorBase *gerador_base);
/**********************************************************************************************************************/
#endif /* BRB_GERADOR_BASE_H_ */