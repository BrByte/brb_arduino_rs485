/*
 * gerador_main.cpp
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

#include "../main.h"

static int BrbGeradorBase_PowerStart(BrbGeradorBase *gerador_base);
static int BrbGeradorBase_PowerStop(BrbGeradorBase *gerador_base);
static int BrbGeradorBase_PowerOff(BrbGeradorBase *gerador_base);
static int BrbGeradorBase_PowerSetState(BrbGeradorBase *gerador_base, BrbGeradorStateCode code, BrbGeradorFailureCode fail);

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

	pinMode(gerador_base->pin_extra, OUTPUT);
	digitalWrite(gerador_base->pin_extra, GERADOR_POWER_OFF);

	if (gerador_base->pin_sensor_ac > 0)
		pinMode(gerador_base->pin_sensor_ac, INPUT_PULLUP);

	if (gerador_base->pin_sensor_dc > 0)
		pinMode(gerador_base->pin_sensor_dc, INPUT_PULLUP);

	BrbServoSetPosByPin(gerador_base->brb_base, gerador_base->pin_servo, GERADOR_SERVO_BB_POS_OPEN);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Loop(BrbGeradorBase *gerador_base)
{
	gerador_base->ms.delay = gerador_base->brb_base->ms.cur - gerador_base->ms.last;

	/* Loop Delay */
	if (gerador_base->ms.delay < 50)
		return -1;

	gerador_base->ms.last = gerador_base->ms.cur;
	gerador_base->ms.cur = gerador_base->brb_base->ms.cur;

	gerador_base->state.delta = (gerador_base->ms.cur - gerador_base->state.time);

	gerador_base->info.gas = random(750, 1000) / 10.0;
	gerador_base->info.load = random(250, 300) / 10.0;

	gerador_base->info.zero_delta = (gerador_base->ms.cur - gerador_base->info.zero_last);

	/* We are waiting delay */
	if ((gerador_base->info.zero_last <= 0) || (gerador_base->info.zero_delta >= GERADOR_TIMER_ZERO_WAIT_MS))
	{
		gerador_base->info.zero_last = gerador_base->ms.cur;

		noInterrupts();
		gerador_base->info.zero_value = (gerador_base->info.zero_counter / (gerador_base->info.zero_delta / 1000.0)) / 2.0;
		gerador_base->info.zero_counter = 0;
		interrupts();
	}

	// LOG_NOTICE(gerador_base->brb_base->log_base, "POWER [%02.01f] [%02.01f]\n", gerador_base->info.power_ac, gerador_base->info.zero_value);

#define RDC1 30000.0
#define RDC2 7500.0
#define RDCR (RDC2 / (RDC2 + RDC1))

	gerador_base->info.power_ac = ((analogRead(gerador_base->pin_sensor_ac) * 5.0) / 1024.0) / 0.013;
	gerador_base->info.battery = ((analogRead(gerador_base->pin_sensor_dc) * 5.0) / 1024.0) / RDCR;

	switch (gerador_base->state.code)
	{
	case GERADOR_STATE_NONE:
	{
		/* This can't happen here, do something */
		if (gerador_base->info.power_ac > GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value > 30)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_RUNNING, GERADOR_FAILURE_RUNNING_WITHOUT_START);

			BrbToneBase_PlayArrive(gerador_base->tone_base);

			break;
		}

		break;
	}
	case GERADOR_STATE_START_INIT:
	{
		/* Power off pins */
		BrbGeradorBase_PowerOff(gerador_base);

		/* This can't happen here, do something */
		if (gerador_base->info.power_ac > GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value > 30)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_RUNNING, GERADOR_FAILURE_RUNNING_WITHOUT_START);

			BrbToneBase_PlayArrive(gerador_base->tone_base);

			break;
		}

		/* We are waiting delay */
		if ((gerador_base->state.time > 0) && (gerador_base->state.delta < GERADOR_TIMER_START_WAIT_MS))
			break;

		/* Reset info */
		gerador_base->state.retry = 0;

		BrbGeradorBase_PowerStart(gerador_base);

		break;
	}
	case GERADOR_STATE_START_DELAY:
	{
		if (gerador_base->info.power_ac > GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value > 30)
		{
			/* Power off pins */
			BrbGeradorBase_PowerOff(gerador_base);

			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_RUNNING, GERADOR_FAILURE_NONE);

			break;
		}

		/* We are waiting delay */
		if ((gerador_base->state.time > 0) && (gerador_base->state.delta < GERADOR_TIMER_START_DELAY_MS))
			break;

		/* Power off pins */
		BrbGeradorBase_PowerOff(gerador_base);

		BrbServoSetPosByPin(gerador_base->brb_base, gerador_base->pin_servo, GERADOR_SERVO_BB_POS_OPEN);

		BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_START_CHECK, GERADOR_FAILURE_NONE);

		break;
	}
	case GERADOR_STATE_START_CHECK:
	{
		/* Power off pins */
		BrbGeradorBase_PowerOff(gerador_base);

		if (gerador_base->info.power_ac > GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value > 30)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_RUNNING, GERADOR_FAILURE_NONE);

			BrbToneBase_PlayArrive(gerador_base->tone_base);

			break;
		}

		if (gerador_base->state.retry >= GERADOR_TIMER_START_RETRY_MAX)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_FAILURE, GERADOR_FAILURE_START_RETRY_LIMIT);

			break;
		}

		/* We are waiting delay */
		if ((gerador_base->state.time > 0) && (gerador_base->state.delta < GERADOR_TIMER_START_CHECK_MS))
			break;

		BrbGeradorBase_PowerStart(gerador_base);

		break;
	}
	case GERADOR_STATE_STOP_INIT:
	{
		/* Power off pins */
		BrbGeradorBase_PowerOff(gerador_base);

		/* Reset info */
		gerador_base->state.retry = 0;

		BrbGeradorBase_PowerStop(gerador_base);

		break;
	}
	case GERADOR_STATE_STOP_DELAY:
	{
		if (gerador_base->info.power_ac <= GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value < 30)
		{
			/* Power off pins */
			BrbGeradorBase_PowerOff(gerador_base);

			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_NONE, GERADOR_FAILURE_NONE);

			BrbToneBase_PlayLeave(gerador_base->tone_base);

			break;
		}

		/* We are waiting delay */
		if ((gerador_base->state.time > 0) && (gerador_base->state.delta < GERADOR_TIMER_STOP_DELAY_MS))
			break;

		/* Power off pins */
		BrbGeradorBase_PowerOff(gerador_base);

		BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_STOP_CHECK, GERADOR_FAILURE_NONE);

		break;
	}
	case GERADOR_STATE_STOP_CHECK:
	{
		/* Power off pins */
		BrbGeradorBase_PowerOff(gerador_base);

		if (gerador_base->info.power_ac <= GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value < 30)
		{
			BrbToneBase_PlayAlarm(gerador_base->tone_base);

			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_NONE, GERADOR_FAILURE_NONE);

			BrbToneBase_PlayLeave(gerador_base->tone_base);

			break;
		}

		if (gerador_base->state.retry >= GERADOR_TIMER_START_RETRY_MAX)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_FAILURE, GERADOR_FAILURE_STOP_RETRY_LIMIT);

			break;
		}

		/* We are waiting delay */
		if ((gerador_base->state.time > 0) && (gerador_base->state.delta < GERADOR_TIMER_STOP_CHECK_MS))
			break;

		BrbGeradorBase_PowerStop(gerador_base);

		break;
	}
	case GERADOR_STATE_RUNNING:
	{
		/* Check Power */
		if (gerador_base->info.power_ac < GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value < 30)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_FAILURE, GERADOR_FAILURE_DOWN_WITHOUT_STOP);

			BrbToneBase_PlayAlarm3(gerador_base->tone_base);

			break;
		}

		break;
	}
	case GERADOR_STATE_FAILURE:
	{
		/* We are waiting delay */
		if ((gerador_base->state.time > 0) && (gerador_base->state.delta < GERADOR_TIMER_FAIL_ALARM_MS))
			break;

		/* Power off pins */
		BrbGeradorBase_PowerOff(gerador_base);

		BrbToneBase_PlayAlarm3(gerador_base->tone_base);

		switch (gerador_base->state.fail)
		{
		case GERADOR_FAILURE_RUNNING_WITHOUT_START:
		{
			if (gerador_base->info.power_ac < GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value < 30)
			{
				BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_NONE, GERADOR_FAILURE_NONE);
				break;
			}

			break;
		}
		case GERADOR_FAILURE_DOWN_WITHOUT_STOP:
		{
			if (gerador_base->info.power_ac >= GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value > 30)
			{
				BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_NONE, GERADOR_FAILURE_NONE);
				break;
			}

			break;
		}
		case GERADOR_FAILURE_START_RETRY_LIMIT:
		{
			if (gerador_base->info.power_ac >= GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value > 30)
			{
				BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_NONE, GERADOR_FAILURE_NONE);
				break;
			}

			break;
		}
		case GERADOR_FAILURE_STOP_RETRY_LIMIT:
		{
			if (gerador_base->info.power_ac < GERADOR_TIMER_MIN_POWER && gerador_base->info.zero_value < 30)
			{
				BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_NONE, GERADOR_FAILURE_NONE);
				break;
			}

			break;
		}
		default:
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_NONE, GERADOR_FAILURE_NONE);
			break;
		}
		}

		/* Update time */
		gerador_base->state.time = gerador_base->ms.cur;

		break;
	}
	}

	if (gerador_base->info.power_ac > GERADOR_TIMER_MIN_POWER)
	{
		gerador_base->info.hourmeter_ms = gerador_base->info.hourmeter_ms + gerador_base->ms.delay;

		/* 5 seconds delay */
		if (gerador_base->info.hourmeter_ms > 5000)
		{
			gerador_base->info.hourmeter_ms = (gerador_base->info.hourmeter_ms - 5000);

			gerador_base->info.hourmeter_sec = gerador_base->info.hourmeter_sec + 5;

			/* 60 seconds delay */
			if (gerador_base->info.hourmeter_sec > 60)
			{
				gerador_base->data.hourmeter_total++;
				gerador_base->data.hourmeter_time++;
				gerador_base->info.hourmeter_sec = (gerador_base->info.hourmeter_sec - 60);

				BrbGeradorBase_Save(gerador_base);
			}
		}

		// BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_RUNNING);
	}

	return 0;
}
/**********************************************************************************************************************/
static int BrbGeradorBase_PowerStart(BrbGeradorBase *gerador_base)
{
	// BrbBase *brb_base = gerador_base->brb_base;

	BrbToneBase_PlayAlarm2(gerador_base->tone_base);

	BrbServoSetPosByPin(gerador_base->brb_base, gerador_base->pin_servo, GERADOR_SERVO_BB_POS_CLOSE);

	BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_START_DELAY, GERADOR_FAILURE_NONE);

	/* Set pin data */
	digitalWrite(gerador_base->pin_partida, GERADOR_POWER_ON);

	gerador_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbGeradorBase_PowerStop(BrbGeradorBase *gerador_base)
{
	// BrbBase *brb_base = gerador_base->brb_base;

	BrbToneBase_PlayAlarm3(gerador_base->tone_base);

	BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_STOP_DELAY, GERADOR_FAILURE_NONE);

	/* Set pin data */
	digitalWrite(gerador_base->pin_parada, GERADOR_POWER_ON);

	gerador_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbGeradorBase_PowerOff(BrbGeradorBase *gerador_base)
{
	// BrbBase *brb_base = gerador_base->brb_base;

	/* Set pin data */
	digitalWrite(gerador_base->pin_partida, GERADOR_POWER_OFF);
	digitalWrite(gerador_base->pin_parada, GERADOR_POWER_OFF);

	return 0;
}
/**********************************************************************************************************************/
static int BrbGeradorBase_PowerSetState(BrbGeradorBase *gerador_base, BrbGeradorStateCode code, BrbGeradorFailureCode fail)
{
	// BrbBase *brb_base = gerador_base->brb_base;

	gerador_base->state.delta = 0;
	gerador_base->state.code = code;
	gerador_base->state.fail = fail;
	gerador_base->state.time = gerador_base->ms.cur;

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Start(BrbGeradorBase *gerador_base)
{
	// BrbBase *brb_base = gerador_base->brb_base;

	BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_START_INIT, GERADOR_FAILURE_NONE);

	BrbToneBase_PlayAction(gerador_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Stop(BrbGeradorBase *gerador_base)
{
	// BrbBase *brb_base = gerador_base->brb_base;

	BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_STOP_INIT, GERADOR_FAILURE_NONE);

	BrbToneBase_PlayAction(gerador_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_FailureConfirm(BrbGeradorBase *gerador_base)
{
	// BrbBase *brb_base = gerador_base->brb_base;

	BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_NONE, GERADOR_FAILURE_NONE);

	BrbToneBase_PlayAction(gerador_base->tone_base);

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
const char *BrbGeradorBase_GetState(BrbGeradorBase *gerador_base)
{
	const char *ret_ptr = PSTR("Parado");

	switch (gerador_base->state.code)
	{
	case GERADOR_STATE_START_INIT:
	{
		ret_ptr = PSTR("Iniciando");
		break;
	}
	case GERADOR_STATE_START_DELAY:
	{
		ret_ptr = PSTR("Ligando");
		break;
	}
	case GERADOR_STATE_START_CHECK:
	{
		ret_ptr = PSTR("Verificando");
		break;
	}
	case GERADOR_STATE_RUNNING:
	{
		ret_ptr = PSTR("Funcionando");
		break;
	}
	case GERADOR_STATE_FAILURE:
	{
		ret_ptr = PSTR("Falha");
		break;
	}
	case GERADOR_STATE_STOP_INIT:
	{
		ret_ptr = PSTR("Finalizando");
		break;
	}
	case GERADOR_STATE_STOP_DELAY:
	{
		ret_ptr = PSTR("Desligando");
		break;
	}
	case GERADOR_STATE_STOP_CHECK:
	{
		ret_ptr = PSTR("Encerrando");
		break;
	}
	case GERADOR_STATE_NONE:
	default:
	{
		/**/
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/
const char *BrbGeradorBase_GetStateAction(BrbGeradorBase *gerador_base)
{
	const char *ret_ptr = PSTR("None");

	switch (gerador_base->state.code)
	{
	case GERADOR_STATE_START_INIT:
	case GERADOR_STATE_START_DELAY:
	case GERADOR_STATE_START_CHECK:
	case GERADOR_STATE_RUNNING:
	{
		ret_ptr = PSTR("Desligar Sistema?");
		break;
	}
	case GERADOR_STATE_FAILURE:
	{
		ret_ptr = PSTR("Falha no Sistema!");
		break;
	}
	case GERADOR_STATE_STOP_INIT:
	case GERADOR_STATE_STOP_DELAY:
	case GERADOR_STATE_STOP_CHECK:
	case GERADOR_STATE_NONE:
	{
		ret_ptr = PSTR("Iniciar Sistema?");
		break;
	}
	default:
	{
		/**/
		break;
	}
	}

	return ret_ptr;
}

/**********************************************************************************************************************/
const char *BrbGeradorBase_GetStateButton(BrbGeradorBase *gerador_base)
{
	const char *ret_ptr = PSTR("None");

	switch (gerador_base->state.code)
	{
	case GERADOR_STATE_START_INIT:
	case GERADOR_STATE_START_DELAY:
	case GERADOR_STATE_START_CHECK:
	case GERADOR_STATE_RUNNING:
	{
		ret_ptr = PSTR("DESLIGAR");
		break;
	}
	case GERADOR_STATE_FAILURE:
	{
		ret_ptr = PSTR("IGNORAR");
		break;
	}
	case GERADOR_STATE_STOP_INIT:
	case GERADOR_STATE_STOP_DELAY:
	case GERADOR_STATE_STOP_CHECK:
	case GERADOR_STATE_NONE:
	default:
	{
		ret_ptr = PSTR("LIGAR");
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/
const char *BrbGeradorBase_GetFailure(BrbGeradorBase *gerador_base)
{
	const char *ret_ptr = PSTR("- - - - -");

	switch (gerador_base->state.fail)
	{
	case GERADOR_FAILURE_RUNNING_WITHOUT_START:
	{
		ret_ptr = PSTR("Energia Detectada");
		break;
	}
	case GERADOR_FAILURE_DOWN_WITHOUT_STOP:
	{
		ret_ptr = PSTR("Sem Energia");
		break;
	}
	case GERADOR_FAILURE_START_RETRY_LIMIT:
	{
		ret_ptr = PSTR("Limite Partida");
		break;
	}
	case GERADOR_FAILURE_STOP_RETRY_LIMIT:
	{
		ret_ptr = PSTR("Limite Parada");
		break;
	}
	default:
	{
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/