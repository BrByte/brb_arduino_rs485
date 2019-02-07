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

static int BrbGeradorBase_PowerStart(BrbGeradorBase *gerador_base);
static int BrbGeradorBase_PowerStop(BrbGeradorBase *gerador_base);
static int BrbGeradorBase_PowerDown(BrbGeradorBase *gerador_base);
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

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Loop(BrbGeradorBase *gerador_base)
{
	gerador_base->ms.last = gerador_base->ms.cur;
	gerador_base->ms.cur = gerador_base->brb_base->ms.cur;
	gerador_base->ms.delay = gerador_base->ms.cur - gerador_base->ms.last;
	gerador_base->state.delta = (gerador_base->state.time - gerador_base->ms.cur);

	gerador_base->info.gas = random(750, 1000) / 10.0;
	gerador_base->info.load = random(250, 300) / 10.0;

	gerador_base->info.battery = ((analogRead(gerador_base->pin_sensor_dc) * 25) / 1024.0);
	gerador_base->info.power = analogRead(gerador_base->pin_sensor_ac);

	switch (gerador_base->state.code)
	{
	case GERADOR_STATE_NONE:
	{
		/* This can't happen here, do something */
		if (gerador_base->info.power > GERADOR_TIMER_MIN_POWER)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_RUNNING, GERADOR_FAILURE_RUNNING_WITHOUT_START);

			break;
		}

		break;
	}
	case GERADOR_STATE_START_INIT:
	{
		/* Aways power down before start */
		BrbGeradorBase_PowerDown(gerador_base);

		/* This can't happen here, do something */
		if (gerador_base->info.power > GERADOR_TIMER_MIN_POWER)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_RUNNING, GERADOR_FAILURE_RUNNING_WITHOUT_START);

			break;
		}

		/* Reset info */
		gerador_base->state.retry = 0;

		BrbGeradorBase_PowerStart(gerador_base);
		gerador_base->state.retry++;

		BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_START_DELAY, 0);

		break;
	}
	case GERADOR_STATE_START_DELAY:
	{
		if ((gerador_base->state.time <= 0) || (gerador_base->state.delta >= GERADOR_TIMER_START_MS))
		{
			BrbServoSetPosByPin(gerador_base->brb_base, gerador_base->pin_servo, GERADOR_SERVO_BB_POS_OPEN);

			/* Aways power down before start */
			BrbGeradorBase_PowerDown(gerador_base);

			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_START_CHECK, 0);
		}

		break;
	}
	case GERADOR_STATE_START_CHECK:
	{
		/* Aways power down before start */
		BrbGeradorBase_PowerDown(gerador_base);

		if (gerador_base->info.power > GERADOR_TIMER_MIN_POWER)
		{
			BrbToneBase_PlayAlarm(gerador_base->tone_base);

			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_RUNNING, 0);

			break;
		}

		if (gerador_base->state.retry > GERADOR_TIMER_START_RETRY_MAX)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_FAILURE, GERADOR_FAILURE_START_RETRY_LIMIT);

			break;
		}

		BrbGeradorBase_PowerStart(gerador_base);
		gerador_base->state.retry++;

		BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_START_DELAY, 0);

		break;
	}
	case GERADOR_STATE_STOP_INIT:
	{
		/* Aways power down before start */
		BrbGeradorBase_PowerDown(gerador_base);

		/* Reset info */
		gerador_base->state.retry = 0;

		BrbGeradorBase_PowerStop(gerador_base);
		gerador_base->state.retry++;

		BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_STOP_DELAY, 0);

		break;
	}
	case GERADOR_STATE_STOP_DELAY:
	{
		if ((gerador_base->state.time <= 0) || (gerador_base->state.delta >= GERADOR_TIMER_STOP_MS))
		{
			/* Aways power down before start */
			BrbGeradorBase_PowerDown(gerador_base);

			BrbServoSetPosByPin(gerador_base->brb_base, gerador_base->pin_servo, GERADOR_SERVO_BB_POS_CLOSE);

			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_STOP_CHECK, 0);
		}

		break;
	}
	case GERADOR_STATE_STOP_CHECK:
	{
		/* Aways power down before start */
		BrbGeradorBase_PowerDown(gerador_base);

		if (gerador_base->info.power <= GERADOR_TIMER_MIN_POWER)
		{
			BrbToneBase_PlayAlarm(gerador_base->tone_base);

			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_NONE, 0);

			break;
		}

		if (gerador_base->state.retry > GERADOR_TIMER_START_RETRY_MAX)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_FAILURE, GERADOR_FAILURE_STOP_RETRY_LIMIT);

			break;
		}

		BrbGeradorBase_PowerStop(gerador_base);
		gerador_base->state.retry++;

		BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_START_DELAY, 0);

		break;
	}
	case GERADOR_STATE_RUNNING:
	{
		/* Check Power */
		if (gerador_base->info.power < GERADOR_TIMER_MIN_POWER)
		{
			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_FAILURE, GERADOR_FAILURE_DOWN_WITHOUT_STOP);

			BrbToneBase_PlayAlarm3(gerador_base->tone_base);
		}

		break;
	}
	case GERADOR_STATE_FAILURE:
	{
		/* Aways power down before start */
		BrbGeradorBase_PowerDown(gerador_base);

		if ((gerador_base->state.time <= 0) || (gerador_base->state.delta >= GERADOR_TIMER_FAIL_ALARM_MS))
		{

			BrbToneBase_PlayAlarm3(gerador_base->tone_base);

			BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_FAILURE, gerador_base->state.fail);
		}

		break;
	}
	}

	if (gerador_base->info.power > GERADOR_TIMER_MIN_POWER)
	{
		gerador_base->info.hourmeter_ms = gerador_base->info.hourmeter_ms + gerador_base->ms.delay;

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

		// BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_RUNNING);
	}

	return 0;
}
/**********************************************************************************************************************/
static int BrbGeradorBase_PowerStart(BrbGeradorBase *gerador_base)
{
	BrbBase *brb_base = gerador_base->brb_base;

	BrbServoSetPosByPin(gerador_base->brb_base, gerador_base->pin_servo, GERADOR_SERVO_BB_POS_CLOSE);

	/* Set pin data */
	digitalWrite(gerador_base->pin_partida, GERADOR_POWER_ON);

	return 0;
}
/**********************************************************************************************************************/
static int BrbGeradorBase_PowerStop(BrbGeradorBase *gerador_base)
{
	BrbBase *brb_base = gerador_base->brb_base;

	BrbServoSetPosByPin(gerador_base->brb_base, gerador_base->pin_servo, GERADOR_SERVO_BB_POS_OPEN);

	/* Set pin data */
	digitalWrite(gerador_base->pin_parada, GERADOR_POWER_ON);

	return 0;
}
/**********************************************************************************************************************/
static int BrbGeradorBase_PowerDown(BrbGeradorBase *gerador_base)
{
	BrbBase *brb_base = gerador_base->brb_base;

	/* Set pin data */
	digitalWrite(gerador_base->pin_partida, GERADOR_POWER_OFF);
	digitalWrite(gerador_base->pin_parada, GERADOR_POWER_OFF);

	return 0;
}
/**********************************************************************************************************************/
static int BrbGeradorBase_PowerSetState(BrbGeradorBase *gerador_base, BrbGeradorStateCode code, BrbGeradorFailureCode fail)
{
	BrbBase *brb_base = gerador_base->brb_base;

	gerador_base->state.code = code;
	gerador_base->state.fail = fail;
	gerador_base->state.time = gerador_base->ms.cur;

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Start(BrbGeradorBase *gerador_base)
{
	BrbBase *brb_base = gerador_base->brb_base;

	BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_START_INIT, 0);

	BrbToneBase_PlayAlarm(gerador_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Stop(BrbGeradorBase *gerador_base)
{
	BrbBase *brb_base = gerador_base->brb_base;

	BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_STOP_INIT, 0);

	BrbToneBase_PlayAlarm2(gerador_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_FailureConfirm(BrbGeradorBase *gerador_base)
{
	BrbBase *brb_base = gerador_base->brb_base;

	BrbGeradorBase_PowerSetState(gerador_base, GERADOR_STATE_NONE, 0);

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
const __FlashStringHelper *BrbGeradorBase_GetState(BrbGeradorBase *gerador_base)
{
	const __FlashStringHelper *ret_ptr = F("None");

	switch (gerador_base->state.code)
	{
	case GERADOR_STATE_START_INIT:
	{
		ret_ptr = F("Start Init");
		break;
	}
	case GERADOR_STATE_START_DELAY:
	{
		ret_ptr = F("Start Delay");
		break;
	}
	case GERADOR_STATE_START_CHECK:
	{
		ret_ptr = F("Start Check");
		break;
	}
	case GERADOR_STATE_RUNNING:
	{
		ret_ptr = F("Running");
		break;
	}
	case GERADOR_STATE_FAILURE:
	{
		ret_ptr = F("Failure");
		break;
	}
	case GERADOR_STATE_STOP_INIT:
	{
		ret_ptr = F("Stop Init");
		break;
	}
	case GERADOR_STATE_STOP_DELAY:
	{
		ret_ptr = F("Stop Delay");
		break;
	}
	case GERADOR_STATE_STOP_CHECK:
	{
		ret_ptr = F("Stop Check");
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
const __FlashStringHelper *BrbGeradorBase_GetStateAction(BrbGeradorBase *gerador_base)
{
	const __FlashStringHelper *ret_ptr = F("None");

	switch (gerador_base->state.code)
	{
	case GERADOR_STATE_START_INIT:
	case GERADOR_STATE_START_DELAY:
	case GERADOR_STATE_START_CHECK:
	case GERADOR_STATE_RUNNING:
	{
		ret_ptr = F("Stop Power?");
		break;
	}
	case GERADOR_STATE_FAILURE:
	{
		ret_ptr = F("Failure!!!");
		break;
	}
	case GERADOR_STATE_STOP_INIT:
	case GERADOR_STATE_STOP_DELAY:
	case GERADOR_STATE_STOP_CHECK:
	case GERADOR_STATE_NONE:
	{
		ret_ptr = F("Start Power?");
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
const __FlashStringHelper *BrbGeradorBase_GetFailure(BrbGeradorBase *gerador_base)
{
	const __FlashStringHelper *ret_ptr = F("- - - - -");

	switch (gerador_base->state.code)
	{
	case GERADOR_FAILURE_RUNNING_WITHOUT_START:
	{
		ret_ptr = F("Running without start");
		break;
	}
	case GERADOR_FAILURE_DOWN_WITHOUT_STOP:
	{
		ret_ptr = F("Down without stop");
		break;
	}
	case GERADOR_FAILURE_START_RETRY_LIMIT:
	{
		ret_ptr = F("Start retry limit");
		break;
	}
	case GERADOR_FAILURE_STOP_RETRY_LIMIT:
	{
		ret_ptr = F("Stop retry limit");
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