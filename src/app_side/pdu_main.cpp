/*
 * pdu_main.cpp
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

#ifdef PDU_SYSTEM_COMPILE

static int BrbPDUBase_PowerStart(BrbPDUBase *pdu_base);
static int BrbPDUBase_PowerStop(BrbPDUBase *pdu_base);
static int BrbPDUBase_PowerOff(BrbPDUBase *pdu_base);
static int BrbPDUBase_PowerSetState(BrbPDUBase *pdu_base, BrbPDUStateCode code, BrbPDUFailureCode fail);

/**********************************************************************************************************************/
int BrbPDUBase_Init(BrbPDUBase *pdu_base)
{
	/* Sanitize */
	if (!pdu_base)
		return -1;

	/* Read EEPROM */
	// BrbBase_EEPROMRead(pdu_base->brb_base, (uint8_t *)&pdu_base->data, sizeof(pdu_base->data), BRB_PIN_DATA_OFFSET + 100 + (sizeof(BrbBasePinData) * TOTAL_PINS));

	// pinMode(pdu_base->pin_partida, OUTPUT);
	// digitalWrite(pdu_base->pin_partida, PDU_POWER_OFF);

	// pinMode(pdu_base->pin_parada, OUTPUT);
	// digitalWrite(pdu_base->pin_parada, PDU_POWER_OFF);

	if (pdu_base->sensor_power.pin > 0)
		pinMode(pdu_base->sensor_power.pin, INPUT_PULLUP);

	if (pdu_base->sensor_aux.pin > 0)
		pinMode(pdu_base->sensor_aux.pin, INPUT_PULLUP);

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_Loop(BrbPDUBase *pdu_base)
{
	pdu_base->ms.delay = pdu_base->brb_base->ms.cur - pdu_base->ms.last;

	/* Loop Delay */
	if (pdu_base->ms.delay < 50)
		return -1;

	pdu_base->ms.last = pdu_base->ms.cur;
	pdu_base->ms.cur = pdu_base->brb_base->ms.cur;

	pdu_base->state.delta = (pdu_base->ms.cur - pdu_base->state.time);

	pdu_base->zerocross.ms_delta = (pdu_base->ms.cur - pdu_base->zerocross.ms_last);

	/* We are waiting delay */
	if ((pdu_base->zerocross.ms_last <= 0) || (pdu_base->zerocross.ms_delta >= PDU_TIMER_ZERO_WAIT_MS))
	{
		pdu_base->zerocross.ms_last = pdu_base->ms.cur;

		noInterrupts();
		pdu_base->zerocross.value_power = (pdu_base->zerocross.counter_power / (pdu_base->zerocross.ms_delta / 1000.0)) / 2.0;
		pdu_base->zerocross.counter_power = 0;

		pdu_base->zerocross.value_aux = (pdu_base->zerocross.counter_aux / (pdu_base->zerocross.ms_delta / 1000.0)) / 2.0;
		pdu_base->zerocross.counter_aux = 0;
		interrupts();
	}

	pdu_base->power.value = ((analogRead(pdu_base->power.pin) * 5.0) / 1024.0) / 0.013;
	pdu_base->aux.value = ((analogRead(pdu_base->aux.pin) * 5.0) / 1024.0) / 0.013;

	switch (pdu_base->state.code)
	{
	case PDU_STATE_NONE:
	{
		/* This can't happen here, do something */
		if (pdu_base->power.value > PDU_POWER_MIN_VALUE && pdu_base->zerocross.value_power > PDU_POWER_MIN_HZ)
		{
			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_RUNNING, PDU_FAILURE_NONE);

			BrbToneBase_PlayArrive(pdu_base->tone_base);

			break;
		}

		break;
	}
	case PDU_STATE_RUNNING:
	{
		/* Check Power */
		if (pdu_base->power.value < PDU_POWER_MIN_VALUE && pdu_base->zerocross.value_power < PDU_POWER_MIN_HZ)
		{
			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_FAILURE, PDU_FAILURE_POWER_DOWN);

			BrbToneBase_PlayAlarm3(pdu_base->tone_base);

			break;
		}

		break;
	}
	case PDU_STATE_FAILURE:
	{
		/* We are waiting delay */
		if ((pdu_base->state.time > 0) && (pdu_base->state.delta < PDU_TIMER_FAIL_ALARM_MS))
			break;

		/* Power off pins */
		BrbPDUBase_PowerOff(pdu_base);

		BrbToneBase_PlayAlarm3(pdu_base->tone_base);

		switch (pdu_base->state.fail)
		{
		case PDU_FAILURE_POWER_DOWN:
		{
			if (pdu_base->power.value >= PDU_POWER_MIN_VALUE && pdu_base->zerocross.value_power > PDU_POWER_MIN_HZ)
			{
				BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);
				break;
			}

			break;
		}
		default:
		{
			BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);
			break;
		}
		}

		/* Update time */
		pdu_base->state.time = pdu_base->ms.cur;

		break;
	}
	}

	if (pdu_base->power.value > PDU_POWER_MIN_VALUE)
	{
		pdu_base->info.hourmeter_ms = pdu_base->info.hourmeter_ms + pdu_base->ms.delay;

		/* 5 seconds delay */
		if (pdu_base->info.hourmeter_ms > 5000)
		{
			pdu_base->info.hourmeter_ms = (pdu_base->info.hourmeter_ms - 5000);

			pdu_base->info.hourmeter_sec = pdu_base->info.hourmeter_sec + 5;

			/* 60 seconds delay */
			if (pdu_base->info.hourmeter_sec > 60)
			{
				pdu_base->data.hourmeter_total++;
				pdu_base->data.hourmeter_time++;
				pdu_base->info.hourmeter_sec = (pdu_base->info.hourmeter_sec - 60);

				BrbPDUBase_Save(pdu_base);
			}
		}

		// BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_RUNNING);
	}

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_PowerStart(BrbPDUBase *pdu_base)
{
	BrbBase *brb_base = pdu_base->brb_base;

	BrbToneBase_PlayAlarm2(pdu_base->tone_base);

	// BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_START_DELAY, PDU_FAILURE_NONE);

	/* Set pin data */
	// digitalWrite(pdu_base->pin_partida, PDU_POWER_ON);

	pdu_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_PowerStop(BrbPDUBase *pdu_base)
{
	BrbBase *brb_base = pdu_base->brb_base;

	BrbToneBase_PlayAlarm3(pdu_base->tone_base);

	// BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_STOP_DELAY, PDU_FAILURE_NONE);

	/* Set pin data */
	// digitalWrite(pdu_base->pin_parada, PDU_POWER_ON);

	pdu_base->state.retry++;

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_PowerOff(BrbPDUBase *pdu_base)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	/* Set pin data */
	// digitalWrite(pdu_base->pin_partida, PDU_POWER_OFF);
	// digitalWrite(pdu_base->pin_parada, PDU_POWER_OFF);

	return 0;
}
/**********************************************************************************************************************/
static int BrbPDUBase_PowerSetState(BrbPDUBase *pdu_base, BrbPDUStateCode code, BrbPDUFailureCode fail)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	pdu_base->state.delta = 0;
	pdu_base->state.code = code;
	pdu_base->state.fail = fail;
	pdu_base->state.time = pdu_base->ms.cur;

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_Start(BrbPDUBase *pdu_base)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	// BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_START_INIT, PDU_FAILURE_NONE);

	BrbToneBase_PlayAction(pdu_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_Stop(BrbPDUBase *pdu_base)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	// BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_STOP_INIT, PDU_FAILURE_NONE);

	BrbToneBase_PlayAction(pdu_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_FailureConfirm(BrbPDUBase *pdu_base)
{
	// BrbBase *brb_base = pdu_base->brb_base;

	BrbPDUBase_PowerSetState(pdu_base, PDU_STATE_NONE, PDU_FAILURE_NONE);

	BrbToneBase_PlayAction(pdu_base->tone_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_HourmeterReset(BrbPDUBase *pdu_base)
{
	if (!pdu_base)
		return -1;

	pdu_base->data.hourmeter_time = 0;
	pdu_base->data.hourmeter_reset++;

	BrbPDUBase_Save(pdu_base);

	return 0;
}
/**********************************************************************************************************************/
int BrbPDUBase_Save(BrbPDUBase *pdu_base)
{
	if (!pdu_base || !pdu_base->brb_base)
		return -1;

	/* Read EEPROM */
	BrbBase_EEPROMWrite(pdu_base->brb_base, (uint8_t *)&pdu_base->data, sizeof(pdu_base->data), BRB_PIN_DATA_OFFSET + 100 + (sizeof(BrbBasePinData) * TOTAL_PINS));

	return 0;
}
/**********************************************************************************************************************/
const char *BrbPDUBase_GetState(BrbPDUBase *pdu_base)
{
	const char *ret_ptr = PSTR("Parado");

	switch (pdu_base->state.code)
	{
	case PDU_STATE_RUNNING:
	{
		ret_ptr = PSTR("Funcionando");
		break;
	}
	case PDU_STATE_FAILURE:
	{
		ret_ptr = PSTR("Falha");
		break;
	}
	case PDU_STATE_NONE:
	default:
	{
		/**/
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/
const char *BrbPDUBase_GetStateAction(BrbPDUBase *pdu_base)
{
	const char *ret_ptr = PSTR("None");

	switch (pdu_base->state.code)
	{
	case PDU_STATE_RUNNING:
	{
		ret_ptr = PSTR("Desligar Sistema?");
		break;
	}
	case PDU_STATE_FAILURE:
	{
		ret_ptr = PSTR("Falha no Sistema!");
		break;
	}
	case PDU_STATE_NONE:
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
const char *BrbPDUBase_GetStateButton(BrbPDUBase *pdu_base)
{
	const char *ret_ptr = PSTR("None");

	switch (pdu_base->state.code)
	{
	case PDU_STATE_RUNNING:
	{
		ret_ptr = PSTR("DESLIGAR");
		break;
	}
	case PDU_STATE_FAILURE:
	{
		ret_ptr = PSTR("IGNORAR");
		break;
	}
	case PDU_STATE_NONE:
	default:
	{
		ret_ptr = PSTR("LIGAR");
		break;
	}
	}

	return ret_ptr;
}
/**********************************************************************************************************************/
const char *BrbPDUBase_GetFailure(BrbPDUBase *pdu_base)
{
	const char *ret_ptr = PSTR("- - - - -");

	switch (pdu_base->state.fail)
	{
	case PDU_FAILURE_POWER_DOWN:
	{
		ret_ptr = PSTR("Sem Energia");
		break;
	}
	default:
	{
		break;
	}
	}

	return ret_ptr;
}
#endif
/**********************************************************************************************************************/