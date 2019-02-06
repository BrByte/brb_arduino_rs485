/*
 * BrbGeradorBase.cpp
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

#include "BrbGeradorBase.h"

/**********************************************************************************************************************/
int BrbGeradorBase_Init(BrbGeradorBase *gerador_base)
{
	/* Sanitize */
	if (!gerador_base)
		return -1;

	/* Read EEPROM */
	BrbBase_EEPROMRead(gerador_base->brb_base, (uint8_t *)&gerador_base->data, sizeof(gerador_base->data), BRB_PIN_DATA_OFFSET + 100 + (sizeof(BrbBasePinData) * TOTAL_PINS));

	pinMode(gerador_base->pin_partida, OUTPUT);
	digitalWrite(gerador_base->pin_partida, GERADOR_POWER_OFF);

	pinMode(gerador_base->pin_parada, OUTPUT);
	digitalWrite(gerador_base->pin_parada, GERADOR_POWER_OFF);

	// gerador_base->data.hourmeter_time = random(100, 1234);

	BrbServoSetPosByPin(gerador_base->brb_base, gerador_base->pin_servo, BRB_GERADOR_SERVO_BB_POS_OPEN);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Loop(BrbGeradorBase *gerador_base)
{
	gerador_base->info.battery = ((analogRead(gerador_base->pin_sensor_dc) * 24.5) / 1024.0);
	// gerador_base->info.battery = random(100, 125) / 10.0;
	// gerador_base->info.battery = (analogRead(A1) * 10) / 10;

	gerador_base->info.load = random(250, 300) / 10.0;
	// gerador_base->info.power = analogRead(gerador_base->pin_sensor_ac);
	gerador_base->info.power = analogRead(gerador_base->pin_sensor_ac);
	gerador_base->info.gas = random(750, 1000) / 10.0;

	if (gerador_base->info.power > 0)
	{
		gerador_base->info.hourmeter_ms = gerador_base->info.hourmeter_ms + gerador_base->brb_base->ms.delay;

		/* 5 seconds delay */
		if (gerador_base->info.hourmeter_ms > 5000)
		{
			gerador_base->data.hourmeter_time++;
			gerador_base->info.hourmeter_ms = (gerador_base->info.hourmeter_ms - 5000);

			gerador_base->info.hourmeter_sec = gerador_base->info.hourmeter_sec + 5;

			/* 60 seconds delay */
			if (gerador_base->info.hourmeter_sec > 60)
			{
				gerador_base->data.hourmeter_time++;
				gerador_base->info.hourmeter_sec = (gerador_base->info.hourmeter_sec - 60);

				BrbGeradorBase_Save(gerador_base);
			}
		}
	}

	if (gerador_base->script && (gerador_base->state != GERADOR_STATE_NONE) && !gerador_base->script->flags.active)
	{
		/* Disable script */
		gerador_base->script->flags.active = 0;

		if (gerador_base->info.power > 0)
		{
			gerador_base->state = GERADOR_STATE_RUNNING;

			BrbToneBase_PlayAlarm(gerador_base->tone_base);
		}
		else
		{
			gerador_base->state = GERADOR_STATE_NONE;

			BrbToneBase_PlayAlarm3(gerador_base->tone_base);
		}
	}

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Start(BrbGeradorBase *gerador_base)
{
	BrbBase *brb_base = gerador_base->brb_base;
	BrbMicroScript *script;
	BrbMicroScriptOPIf *op_if;

	if (gerador_base->pin_partida <= MIN_DIG_PIN || !gerador_base->brb_base || (gerador_base->state != GERADOR_STATE_NONE))
		return -1;

	/* Put pins down */
	digitalWrite(gerador_base->pin_partida, GERADOR_POWER_OFF);
	digitalWrite(gerador_base->pin_parada, GERADOR_POWER_OFF);

	/* Disable script, this do fine to stop */
	if (gerador_base->script)
		gerador_base->script->flags.active = 0;

	script = BrbMicroScriptGrabFree(gerador_base->brb_base);
	gerador_base->script = script;
	script->flags.persist = 0;
	script->flags.active = 1;

	script->loop.max = 3;
	script->loop.cnt = 0;

	BrbMicroScriptOPAddDelay(brb_base, script, 1000);
	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_parada, OUTPUT, GERADOR_POWER_OFF);
	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, GERADOR_POWER_OFF);
	BrbMicroScriptOPAddServoPos(brb_base, script, gerador_base->pin_servo, BRB_GERADOR_SERVO_BB_POS_CLOSE);
	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, GERADOR_POWER_ON);
	BrbMicroScriptOPAddDelay(brb_base, script, BRB_GERADOR_PARTIDA_MS);
	BrbMicroScriptOPAddServoPos(brb_base, script, gerador_base->pin_servo, BRB_GERADOR_SERVO_BB_POS_OPEN);
	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, GERADOR_POWER_OFF);
	BrbMicroScriptOPAddDelay(brb_base, script, BRB_GERADOR_PARTIDA_MS / 2);

	/* Compare Analog pin 0 */
	BrbMicroScriptOPAddCmp(brb_base, script, gerador_base->pin_sensor_ac, 10);

	op_if = BrbMicroScriptOPAddIf(brb_base, script, SCRIPT_OPCODE_JMP_NOT_GREATER, 0, 0);

	if (op_if)
	{
		BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_parada, OUTPUT, GERADOR_POWER_OFF);
		BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, GERADOR_POWER_OFF);

		op_if->else_offset = script->code.size;

		BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_parada, OUTPUT, GERADOR_POWER_OFF);
		BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, GERADOR_POWER_OFF);

		/* Jump to start and do the script again */
		op_if->end_offset = 0;
	}

	gerador_base->state = GERADOR_STATE_STARTING;

	BrbToneBase_PlayAlarm(gerador_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Stop(BrbGeradorBase *gerador_base)
{
	BrbBase *brb_base = gerador_base->brb_base;
	BrbMicroScript *script;
	BrbMicroScriptOPIf *op_if;

	if (!gerador_base->brb_base)
		return -1;

	/* Put pins down */
	digitalWrite(gerador_base->pin_partida, GERADOR_POWER_OFF);
	digitalWrite(gerador_base->pin_parada, GERADOR_POWER_OFF);

	// /* No need to stop */
	// if (gerador_base->state == GERADOR_STATE_NONE)
	// {
	// 	return -1;
	// }

	/* Disable script, this do fine to stop */
	if (gerador_base->script)
		gerador_base->script->flags.active = 0;

	script = BrbMicroScriptGrabFree(gerador_base->brb_base);
	gerador_base->script = script;
	script->flags.persist = 0;
	script->flags.active = 1;

	script->loop.max = 3;
	script->loop.cnt = 0;

	BrbMicroScriptOPAddDelay(brb_base, script, 1000);
	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, GERADOR_POWER_OFF);
	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_parada, OUTPUT, GERADOR_POWER_OFF);
	BrbMicroScriptOPAddDelay(brb_base, script, 1000);
	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_parada, OUTPUT, GERADOR_POWER_ON);
	BrbMicroScriptOPAddDelay(brb_base, script, 3000);
	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_parada, OUTPUT, GERADOR_POWER_OFF);

	/* Compare Analog pin AC sensor */
	BrbMicroScriptOPAddCmp(brb_base, script, gerador_base->pin_sensor_ac, 10);

	op_if = BrbMicroScriptOPAddIf(brb_base, script, SCRIPT_OPCODE_JMP_GREATER, 0, 0);

	if (op_if)
	{
		BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, GERADOR_POWER_OFF);
		BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_parada, OUTPUT, GERADOR_POWER_OFF);

		op_if->else_offset = script->code.size;

		BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, GERADOR_POWER_OFF);
		BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_parada, OUTPUT, GERADOR_POWER_OFF);

		/* Jump to start and do the script again */
		op_if->end_offset = 0;
	}

	gerador_base->state = GERADOR_STATE_STOPPING;

	BrbToneBase_PlayAlarm2(gerador_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_HourmeterReset(BrbGeradorBase *gerador_base)
{
	if (!gerador_base)
		return -1;

	gerador_base->data.hourmeter_time = 0;
	gerador_base->data.hourmeter_reset++;

	BrbGeradorBase_Save(gerador_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Save(BrbGeradorBase *gerador_base)
{
	if (!gerador_base || !gerador_base->brb_base)
		return -1;

	/* Read EEPROM */
	BrbBase_EEPROMWrite(gerador_base->brb_base, (uint8_t *)&gerador_base->data, sizeof(gerador_base->data), BRB_PIN_DATA_OFFSET + 100 + (sizeof(BrbBasePinData) * TOTAL_PINS));

	return 0;
}
/**********************************************************************************************************************/