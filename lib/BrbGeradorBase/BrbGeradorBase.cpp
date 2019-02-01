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
	digitalWrite(gerador_base->pin_partida, LOW);
	// delay(1000);
	// digitalWrite(gerador_base->pin_partida, HIGH);

	pinMode(gerador_base->pin_parada, OUTPUT);
	digitalWrite(gerador_base->pin_parada, LOW);

	// delay(1000);
	// digitalWrite(gerador_base->pin_parada, HIGH);

	// gerador_base->data.horimeter = random(100, 1234);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Loop(BrbGeradorBase *gerador_base)
{

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Partida(BrbGeradorBase *gerador_base)
{
	BrbBase *brb_base = gerador_base->brb_base;
	BrbMicroScript *script;
	BrbMicroScriptOPIf *op_if;

	if (gerador_base->pin_partida <= MIN_DIG_PIN || !gerador_base->brb_base)
		return -1;

	/* Put pins down */
	digitalWrite(gerador_base->pin_partida, LOW);
	digitalWrite(gerador_base->pin_parada, LOW);

	script = BrbMicroScriptGrabFree(gerador_base->brb_base);
	script->flags.persist = 1;
	script->flags.active = 1;

	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, LOW);
	BrbMicroScriptOPAddDelay(brb_base, script, BRB_GERADOR_PARTIDA_MS);

	script = BrbMicroScriptGrabFree(brb_base);
	script->flags.persist = 1;
	script->flags.active = 1;

	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, LOW);
	BrbMicroScriptOPAddServoPos(brb_base, script, gerador_base->pin_servo, 180);
	BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_partida, OUTPUT, HIGH);
	BrbMicroScriptOPAddDelay(brb_base, script, BRB_GERADOR_PARTIDA_MS);
	BrbMicroScriptOPAddServoPos(brb_base, script, gerador_base->pin_servo, 120);
	BrbMicroScriptOPAddDelay(brb_base, script, 1000);

	/* Compare Analog pin 0 */
    BrbMicroScriptOPAddCmp(brb_base, script, 0, 0);

	op_if   = BrbMicroScriptOPAddIf(brb_base, script, SCRIPT_OPCODE_JMP_LESSER_OR_EQUAL, 0, 0);

	if (op_if)
	{
	    BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_parada, OUTPUT, LOW);

	    op_if->else_offset      = script->code.size;

	    BrbMicroScriptOPAddSetDig(brb_base, script, gerador_base->pin_parada, OUTPUT, HIGH);

	    op_if->end_offset       = script->code.size;
	}

	BrbMicroScriptOPAddDelay(brb_base, script, 2000);
	BrbMicroScriptOPAddServoPos(brb_base, script, gerador_base->pin_servo, 115);

	return 0;
}
/**********************************************************************************************************************/
int BrbGeradorBase_Save(BrbGeradorBase *gerador_base)
{
	/* Read EEPROM */
	BrbBase_EEPROMWrite(gerador_base->brb_base, (uint8_t *)&gerador_base->data, sizeof(gerador_base->data), BRB_PIN_DATA_OFFSET + 100 + (sizeof(BrbBasePinData) * TOTAL_PINS));

	return 0;
}
/**********************************************************************************************************************/