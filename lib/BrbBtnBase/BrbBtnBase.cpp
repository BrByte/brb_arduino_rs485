/*
 * BrbBtnBase.cpp
 *
 *  Created on: 2019-01-31
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

#include "BrbBtnBase.h"

static int BrbBtnBase_DataInit(BrbBtnBase *btn_base, BrbBtnData *btn_data);
static int BrbBtnBase_DataCheck(BrbBtnBase *btn_base, BrbBtnData *btn_data);

/**********************************************************************************************************************/
int BrbBtnBase_Init(BrbBtnBase *btn_base)
{
    /* Sanitize */
    if (!btn_base)
        return -1;

    /* Initialize all buttons */
    for (int i = 0; i < BRB_BTN_LAST_ITEM; i++)
    {
        BrbBtnBase_DataInit(btn_base, (BrbBtnData *)&btn_base->buttons[i]);

        continue;
    }

    return 0;
}
/**********************************************************************************************************************/
void BrbBtnBase_Loop(BrbBtnBase *btn_base)
{
    /* Process each button */
    for (int i = 0; i < BRB_BTN_LAST_ITEM; i++)
    {
        BrbBtnBase_DataCheck(btn_base, (BrbBtnData *)&btn_base->buttons[i]);

        continue;
    }
}
/**********************************************************************************************************************/
static int BrbBtnBase_DataInit(BrbBtnBase *btn_base, BrbBtnData *btn_data)
{
    if (btn_data->pin <= 0)
        return -1;

    btn_data->hit = 0;
    btn_data->state = HIGH;

    pinMode(btn_data->pin, INPUT_PULLUP);
    digitalWrite(btn_data->pin, HIGH);

    return 0;
}
/**********************************************************************************************************************/
static int BrbBtnBase_DataCheck(BrbBtnBase *btn_base, BrbBtnData *btn_data)
{
    int old_state;

    if (btn_data->pin <= 0)
        return -1;

    old_state = btn_data->state;
    btn_data->state = digitalRead(btn_data->pin);

    /* BTN CHANGEG STATE */
    if (old_state != btn_data->state)
    {
        LOG_NOTICE(btn_base->brb_base->log_base, "BTN [%d] - hit [%d] - State [%d] -> [%d]\r\n", btn_data->pin, btn_data->hit, old_state, btn_data->state);

        if (btn_data->state == HIGH)
        {
            btn_data->hit++;
            delay(10);
        }
    }

    return 0;
}
/**********************************************************************************************************************/