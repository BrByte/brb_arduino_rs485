/*
 * main.cpp
 *
 *  Created on: 2019-02-09
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

#include "main.h"

/* Global control structures */
BrbLogBase *glob_log_base;
BrbBase glob_brb_base;
BrbRS485Session glob_rs485_sess;

BrbBtnBase glob_btn_base;
BrbDisplayBase glob_display_base;
BrbGeradorBase glob_gerador_base;
BrbToneBase glob_tone_base;

/**********************************************************************************************************************/
/* RUN ONE TIME ON START */
/**********************************************************************************************************************/
void BrbSetup(void)
{
    /* Clean up base */
    memset(&glob_brb_base, 0, sizeof(BrbBase));

    glob_log_base = BrbLogBase_New();
    BrbLogBase_Init(glob_log_base, &Serial);

    glob_brb_base.log_base = glob_log_base;

    BrbBaseInit(&glob_brb_base);

    return;
}
/**********************************************************************************************************************/
void BrbGeradorBase_ZeroCross()
{
    glob_gerador_base.info.zero_counter++;
}
/**********************************************************************************************************************/
void setup()
{
    randomSeed(((analogRead(0) + analogRead(1)) / 2));

    /* Initialize Brb internal data */
    BrbSetup();

    /* Setup Display before anything, because it can display some info, eg logs */
    BrbAppDisplay_Setup(&glob_brb_base);

    /**************************************/
    /* Clean up base */
    memset(&glob_tone_base, 0, sizeof(BrbToneBase));
    BrbToneBase *tone_base = (BrbToneBase *)&glob_tone_base;

    tone_base->pin = BUZZER_PIN;
    tone_base->brb_base = &glob_brb_base;
    BrbToneBase_Init(&glob_tone_base);

    /**************************************/
    /* Clean up base */
    memset(&glob_gerador_base, 0, sizeof(BrbGeradorBase));
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    gerador_base->brb_base = (BrbBase *)&glob_brb_base;
    gerador_base->tone_base = (BrbToneBase *)&glob_tone_base;
    gerador_base->pin_partida = GERADOR_PARTIDA_PIN;
    gerador_base->pin_parada = GERADOR_PARADA_PIN;
    gerador_base->pin_servo = GERADOR_SERVO_PIN;
    gerador_base->pin_extra = GERADOR_EXTRA_PIN;
    gerador_base->pin_zerocross = GERADOR_ZEROCROSS_PIN;

    gerador_base->pin_sensor_ac = SENSOR_VOLTAGE_AC_PIN;
    gerador_base->pin_sensor_dc = SENSOR_VOLTAGE_DC_PIN;

    BrbGeradorBase_Init(gerador_base);

    attachInterrupt(digitalPinToInterrupt(gerador_base->pin_zerocross), BrbGeradorBase_ZeroCross, RISING);
    /**************************************/
    /* Clean up base */
    memset(&glob_btn_base, 0, sizeof(BrbBtnBase));

    BrbBtnBase *btn_base = (BrbBtnBase *)&glob_btn_base;

    btn_base->brb_base = &glob_brb_base;
    btn_base->buttons[BRB_BTN_SELECT].pin = BTN_PIN_SELECT;
    btn_base->buttons[BRB_BTN_NEXT].pin = BTN_PIN_NEXT;
    btn_base->buttons[BRB_BTN_PREV].pin = BTN_PIN_PREV;

    BrbBtnBase_Init(btn_base);
    /**************************************/

    /* Setup RS485 Serial */
    BrbAppRS485_Setup(&glob_brb_base);

    LOG_NOTICE(glob_log_base, "BrbBox Panel Control - START [%u] - 0.1.2\r\n", micros());
    LOG_NOTICE(glob_log_base, "BRB [%p], RS485 [%p]\r\n", &glob_brb_base, &glob_rs485_sess);
    LOG_NOTICE(glob_log_base, "RS485 - ADDR 0x%02x UUID [%02x-%02x-%02x-%02x]\r\n",
               glob_rs485_sess.address, glob_rs485_sess.uuid[0], glob_rs485_sess.uuid[1], glob_rs485_sess.uuid[2], glob_rs485_sess.uuid[3]);
    LOG_HEAP(glob_log_base);

    return;
}
/**********************************************************************************************************************/
/* RUN FOREVER */
/**********************************************************************************************************************/
void loop()
{
    /* Dispatch */
    BrbBaseLoop(&glob_brb_base);
    
    /* Check for Buttons */
    BrbBtnBase_Loop((BrbBtnBase *)&glob_btn_base);

    if (glob_btn_base.buttons[BRB_BTN_SELECT].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_SELECT].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, DISPLAY_ACTION_SELECT);
    }
    else if (glob_btn_base.buttons[BRB_BTN_NEXT].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_NEXT].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, DISPLAY_ACTION_NEXT);
    }
    else if (glob_btn_base.buttons[BRB_BTN_PREV].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_PREV].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, DISPLAY_ACTION_PREV);
    }
    
    /* Do loop RS485 */
    BrbRS485Session_Loop(&glob_rs485_sess);
    BrbGeradorBase_Loop(&glob_gerador_base);
    BrbToneBase_Loop(&glob_tone_base);

    // LOG_INFO(glob_log_base, "last: %ld - cur %ld - delay %ld\n", glob_brb_base.us.last, glob_brb_base.us.cur, glob_brb_base.us.delay);
}
/**********************************************************************************************************************/