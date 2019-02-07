/*
 * main.cpp
 *
 *  Created on: 2018-10-01
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

/* Import needed libraries */
#include "Arduino.h"
#include <SoftwareSerial.h>
#include <BrbLogBase.h>
#include <BrbBase.h>
#include <BrbRS485Session.h>

#include <BrbBtnBase.h>
#include <BrbDisplayBase.h>
#include <BrbGeradorBase.h>
#include <BrbToneBase.h>

#include "Adafruit_Sensor.h"
#include "DHT.h"

#include <avr/wdt.h>

// #define RESERVED    0 /* RX0 */
// #define RESERVED    1 /* TX0 */
// #define RESERVED    2 /* INT4 - PWM */
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
/**********************************************************************************************************************/
BrbGenericCBH BrbDisplayBase_Timer;

BrbGenericCBH BrbDisplayBase_ShowInfo;
BrbGenericCBH BrbDisplayBase_ShowControl;
BrbGenericCBH BrbDisplayBase_ShowConsume;
BrbGenericCBH BrbDisplayBase_ShowTemp;

BrbGenericCBH BrbDisplayBase_ActionInfo;
BrbGenericCBH BrbDisplayBase_ActionControl;
BrbGenericCBH BrbDisplayBase_ActionConsume;

/**********************************************************************************************************************/
#ifdef BRB_RS485_HARDWARE_SERIAL
#define SerialRS485 Serial3
#else
/* receive pin, transmit pin */
SoftwareSerial SerialRS485(RS485_RO_PIN, RS485_DI_PIN);
#endif

/**********************************************************************************************************************/

/* Global control structures */
BrbLogBase *glob_log_base;
BrbBase glob_brb_base;
BrbRS485Session glob_rs485_sess;

BrbBtnBase glob_btn_base;
BrbDisplayBase glob_display_base;
BrbGeradorBase glob_gerador_base;
BrbToneBase glob_tone_base;

// ILI9341_due tft(TFT_CS, TFT_DC, TFT_RST);
// DHT dht_sensor(DHT_SENSOR_PIN, DHT21);
DHT dht_sensor(DHT_SENSOR_PIN, DHT11);

/**********************************************************************************************************************/

BrbRS485SessionActionCBH BrbRS485SessionActionHandShakeCB;

BrbRS485SessionActionCBH BrbRS485SessionActionGetAnalogCB;
BrbRS485SessionActionCBH BrbRS485SessionActionSetAnalogCB;
BrbRS485SessionActionCBH BrbRS485SessionActionSetAnalogBMPCB;

BrbRS485SessionActionCBH BrbRS485SessionActionGetDigitalCB;
BrbRS485SessionActionCBH BrbRS485SessionActionSetDigitalCB;
BrbRS485SessionActionCBH BrbRS485SessionActionSetDigitalBMPCB;
BrbRS485SessionActionCBH BrbRS485SessionActionSetScriptCB;

/**********************************************************************************************************************/
int BrbRS485SessionActionHandShakeCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    // BrbRS485Session *rs485_sess             = (BrbRS485Session *)base_ptr;
    // BrbRS485PacketHandShake *pkt_recv_hs    = (BrbRS485PacketHandShake *)buffer_ptr;

    // LOG_INFO(rs485_sess->log_base, "GET HANDSHAKE [%02x][%02x][%02x][%02x]\n",
    // 		pkt_recv_hs->uuid[0], pkt_recv_hs->uuid[1], pkt_recv_hs->uuid[2], pkt_recv_hs->uuid[3]);

    return RS485_PKT_RETURN_QUIET;
}
/**********************************************************************************************************************/
int BrbRS485SessionActionGetAnalogCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485Session *rs485_sess = (BrbRS485Session *)base_ptr;
    BrbRS485PacketData *pkt_recv = (BrbRS485PacketData *)buffer_ptr;
    BrbRS485PacketData *pkt_reply = (BrbRS485PacketData *)&rs485_sess->pkt.out.data;
    BrbRS485PacketPinData *pin_data = (BrbRS485PacketPinData *)(pkt_reply + 1);

    int op_status;
    int pin_begin;
    int pin_max;
    int i;

    /* Adjust pin query */
    if (pkt_recv->val < MIN_ANA_PIN || pkt_recv->val >= MAX_ANA_PIN)
    {
        pin_begin = MIN_ANA_PIN;
        pin_max = MAX_ANA_PIN;
    }
    else
    {
        pin_begin = pkt_recv->val;
        pin_max = pkt_recv->val + 1;
    }

    // LOG_DEBUG(glob_log_base, "GET ANALOG - [%u] - [%d][%d]\n", pkt_recv->val, pin_begin, pin_max);

    /* Inverse the order two reply */
    pkt_reply->hdr.dst = pkt_recv->hdr.src;
    pkt_reply->hdr.src = glob_rs485_sess.address;
    pkt_reply->hdr.id = pkt_recv->hdr.id;
    pkt_reply->hdr.type = RS485_PKT_TYPE_CMD_GET_A;
    pkt_reply->hdr.len = sizeof(BrbRS485PacketData) + (sizeof(BrbRS485PacketPinData) * (pin_max - pin_begin));

    for (i = pin_begin; i < pin_max; i++)
    {
        pin_data->pin = i;
        pin_data->type = 0;
        pin_data->mode = BrbBase_PinGetMode(glob_analog_pins[i]);
        pin_data->value = analogRead(glob_analog_pins[i]);

        // LOG_DEBUG(glob_log_base, "GET ANALOG [%u] - [%d]\n", i, pin_data->value);

        pin_data++;
    }

    /* Send CRC */
    op_status = BrbRS485Session_SendPacket(rs485_sess, (byte *)pkt_reply, pkt_reply->hdr.len);

    if (op_status <= 0)
        return RS485_PKT_RETURN_ACK_FAIL;

    return RS485_PKT_RETURN_QUIET;
}
/**********************************************************************************************************************/
int BrbRS485SessionActionSetAnalogCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485PacketSetPin *pkt_recv_set = (BrbRS485PacketSetPin *)buffer_ptr;

    LOG_INFO(glob_log_base, "SET ANALOG - [%u] [%u] [%u]\n", pkt_recv_set->pin, pkt_recv_set->mode, pkt_recv_set->value);

    /* Check pinCode we can set */
    if (pkt_recv_set->pin < MIN_ANA_PIN || pkt_recv_set->pin >= MAX_ANA_PIN)
        return RS485_PKT_RETURN_ACK_FAIL;

    if (pkt_recv_set->mode == OUTPUT)
    {
        pinMode(glob_analog_pins[pkt_recv_set->pin], OUTPUT);

        analogWrite(glob_analog_pins[pkt_recv_set->pin], pkt_recv_set->value);
    }
    else if (pkt_recv_set->mode == INPUT)
        pinMode(glob_analog_pins[pkt_recv_set->pin], INPUT);
    else if (pkt_recv_set->mode == INPUT_PULLUP)
        pinMode(glob_analog_pins[pkt_recv_set->pin], INPUT_PULLUP);

    glob_brb_base.pin_data[MAX_DIG_PIN + pkt_recv_set->pin].value = pkt_recv_set->value;
    glob_brb_base.pin_data[MAX_DIG_PIN + pkt_recv_set->pin].mode = pkt_recv_set->mode;
    glob_brb_base.pin_data[MAX_DIG_PIN + pkt_recv_set->pin].persist = 0;

    BrbBase_PinSave(&glob_brb_base);

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
int BrbRS485SessionActionSetAnalogBMPCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485PacketSetPinBmpAna *pkt_recv_set = (BrbRS485PacketSetPinBmpAna *)buffer_ptr;

    int i;

    for (i = MIN_ANA_PIN; i < MAX_ANA_PIN && i < 32; i++)
    {
        if (pkt_recv_set->map[i].mode == 3)
            continue;

        LOG_INFO(glob_log_base, "SET ANALOG [%u]/[%u] - [%u] [%u]\n",
                 i, glob_analog_pins[i], pkt_recv_set->map[i].mode, pkt_recv_set->map[i].value);

        if (pkt_recv_set->map[i].mode == OUTPUT)
        {
            pinMode(glob_analog_pins[i], OUTPUT);

            analogWrite(glob_analog_pins[i], pkt_recv_set->map[i].value);
        }
        else if (pkt_recv_set->map[i].mode == INPUT)
            pinMode(glob_analog_pins[i], INPUT);
        else if (pkt_recv_set->map[i].mode == INPUT_PULLUP)
            pinMode(glob_analog_pins[i], INPUT_PULLUP);

        glob_brb_base.pin_data[MAX_DIG_PIN + i].value = pkt_recv_set->map[i].value;
        glob_brb_base.pin_data[MAX_DIG_PIN + i].mode = pkt_recv_set->map[i].mode;
        // glob_brb_base.pin_data[MAX_DIG_PIN + i].persist = pkt_recv_set->map[i].persist;
        glob_brb_base.pin_data[MAX_DIG_PIN + i].persist = 0;
    }

    BrbBase_PinSave(&glob_brb_base);

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
int BrbRS485SessionActionGetDigitalCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485Session *rs485_sess = (BrbRS485Session *)base_ptr;
    BrbRS485PacketData *pkt_recv = (BrbRS485PacketData *)buffer_ptr;
    BrbRS485PacketData *pkt_reply = (BrbRS485PacketData *)&rs485_sess->pkt.out.data;
    BrbRS485PacketPinData *pin_data = (BrbRS485PacketPinData *)(pkt_reply + 1);
    int op_status;
    int pin_begin;
    int pin_max;
    int i;

    /* Adjust pin query */
    if (pkt_recv->val < 0 || pkt_recv->val >= MAX_DIG_PIN)
    {
        pin_begin = 0;
        pin_max = MAX_DIG_PIN;
    }
    else
    {
        pin_begin = pkt_recv->val;
        pin_max = pkt_recv->val + 1;
    }

    LOG_INFO(glob_log_base, "GET DIGITAL - [%u] - [%d][%d]\n", pkt_recv->val, pin_begin, pin_max);

    /* Inverse the order two reply */
    pkt_reply->hdr.dst = pkt_recv->hdr.src;
    pkt_reply->hdr.src = glob_rs485_sess.address;
    pkt_reply->hdr.id = pkt_recv->hdr.id;
    pkt_reply->hdr.type = RS485_PKT_TYPE_CMD_GET_D;
    pkt_reply->hdr.len = sizeof(BrbRS485PacketData) + (sizeof(BrbRS485PacketPinData) * (pin_max - pin_begin));

    for (i = pin_begin; i < pin_max; i++)
    {
        pin_data->pin = i;
        pin_data->type = 1;
        pin_data->value = digitalRead(i);
        pin_data->mode = BrbBase_PinGetMode(i);
        // pin_data->value = 0;
        // pin_data->mode = 0;
        pin_data++;
    }

    /* Send CRC */
    op_status = BrbRS485Session_SendPacket(rs485_sess, (byte *)pkt_reply, pkt_reply->hdr.len);

    if (op_status <= 0)
        return RS485_PKT_RETURN_ACK_FAIL;

    return RS485_PKT_RETURN_QUIET;
}
/**********************************************************************************************************************/
int BrbRS485SessionActionSetDigitalCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485PacketSetPin *pkt_recv_set = (BrbRS485PacketSetPin *)buffer_ptr;

    LOG_WARN(glob_log_base, "SET DIGITAL - [%u] [%u] [%u]\n", pkt_recv_set->pin, pkt_recv_set->mode, pkt_recv_set->value);

    /* Check pinCode we can set */
    if (pkt_recv_set->pin < MIN_DIG_PIN || pkt_recv_set->pin >= MAX_DIG_PIN)
        return RS485_PKT_RETURN_ACK_FAIL;

    /* Check output, set value */
    if (pkt_recv_set->mode == OUTPUT)
        pinMode(pkt_recv_set->pin, OUTPUT);
    if (pkt_recv_set->mode == INPUT_PULLUP)
        pinMode(pkt_recv_set->pin, INPUT_PULLUP);
    else if (pkt_recv_set->mode == INPUT)
        pinMode(pkt_recv_set->pin, INPUT);

    digitalWrite(pkt_recv_set->pin, (pkt_recv_set->value == HIGH) ? HIGH : LOW);

    glob_brb_base.pin_data[pkt_recv_set->pin].value = (pkt_recv_set->value == HIGH) ? HIGH : LOW;
    glob_brb_base.pin_data[pkt_recv_set->pin].mode = pkt_recv_set->mode;
    // glob_brb_base.pin_data[pkt_recv_set->pin].persist = pkt_recv_set->persist;
    glob_brb_base.pin_data[pkt_recv_set->pin].persist = 0;

    BrbBase_PinSave(&glob_brb_base);

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
int BrbRS485SessionActionSetDigitalBMPCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485PacketSetPinBmpDig *pkt_recv_set = (BrbRS485PacketSetPinBmpDig *)buffer_ptr;

    int i;
    for (i = MIN_DIG_PIN; i < MAX_DIG_PIN && i < 64; i++)
    {
        if (pkt_recv_set->map[i].mode == 3)
            continue;

        LOG_DEBUG(glob_log_base, "SET DIGITAL [%u] - [%u] [%u]\n", i, pkt_recv_set->map[i].value, pkt_recv_set->map[i].mode);

        if (pkt_recv_set->map[i].mode == OUTPUT)
            pinMode(i, OUTPUT);
        else if (pkt_recv_set->map[i].mode == INPUT)
            pinMode(i, INPUT);
        else if (pkt_recv_set->map[i].mode == INPUT_PULLUP)
            pinMode(i, INPUT_PULLUP);

        digitalWrite(i, (pkt_recv_set->map[i].value == HIGH) ? HIGH : LOW);

        glob_brb_base.pin_data[i].value = (pkt_recv_set->map[i].value == HIGH) ? HIGH : LOW;
        glob_brb_base.pin_data[i].mode = pkt_recv_set->map[i].mode;
        // glob_brb_base.pin_data[i].persist = pkt_recv_set->map[i].persist;
        glob_brb_base.pin_data[i].persist = 0;
    }

    BrbBase_PinSave(&glob_brb_base);

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
int BrbRS485SessionActionSetScriptCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485PacketSetScript *pkt_recv_set = (BrbRS485PacketSetScript *)buffer_ptr;
    BrbBase *brb_base = (BrbBase *)cb_data_ptr;
    BrbMicroScript *script;

    LOG_WARN(glob_log_base, "SCRIPT SET [%p] [%p] [%d]\r\n", &glob_brb_base, brb_base, pkt_recv_set->script_id);

    script = BrbMicroScriptGrabFree(brb_base);

    // LOG_WARN(glob_log_base, "SCRIPT SET [%p] [%p] [%o]\r\n", &glob_brb_base, brb_base, script);

    if (!script)
    {
        LOG_WARN(glob_log_base, "NO SCRIPT [%u]\r\n", pkt_recv_set->script_id);

        return RS485_PKT_RETURN_ACK_FAIL;
    }

    memcpy(&script->code, &pkt_recv_set->code, sizeof(BrbMicroCode));

    script->flags.persist = pkt_recv_set->persist;
    script->flags.active = 1;

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
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
void BrbSetupDisplay(void)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)&glob_display_base;

    display_base->brb_base = &glob_brb_base;
    // display_base->screen_cur = DISPLAY_SCREEN_INFO;
    display_base->screen_cur = DISPLAY_SCREEN_CONTROL;
    // display_base->screen_cur = DISPLAY_SCREEN_TEMP;
    // display_base->screen_cur = DISPLAY_SCREEN_CONSUME;

    // display_base->tft = (ILI9341_due *)&tft;
    // display_base->tft = new ILI9341_due(display_base->pin_cs, display_base->pin_dc, display_base->pin_rst);

    display_base->pin_led = TFT_LED;
    display_base->pin_dc = TFT_DC;
    display_base->pin_rst = TFT_RST;
    display_base->pin_cs = TFT_CS;
    display_base->pin_miso = TFT_MISO;
    display_base->pin_mosi = TFT_MOSI;
    display_base->pin_clk = TFT_CLK;

    BrbDisplayBase_SetScreenShowCB(display_base, DISPLAY_SCREEN_INFO, BrbDisplayBase_ShowInfo, NULL);
    BrbDisplayBase_SetScreenShowCB(display_base, DISPLAY_SCREEN_CONTROL, BrbDisplayBase_ShowControl, NULL);
    BrbDisplayBase_SetScreenShowCB(display_base, DISPLAY_SCREEN_CONSUME, BrbDisplayBase_ShowConsume, NULL);
    BrbDisplayBase_SetScreenShowCB(display_base, DISPLAY_SCREEN_TEMP, BrbDisplayBase_ShowTemp, NULL);

    BrbDisplayBase_SetScreenActionCB(display_base, DISPLAY_SCREEN_INFO, BrbDisplayBase_ActionInfo, NULL);
    BrbDisplayBase_SetScreenActionCB(display_base, DISPLAY_SCREEN_CONTROL, BrbDisplayBase_ActionControl, NULL);
    BrbDisplayBase_SetScreenActionCB(display_base, DISPLAY_SCREEN_CONSUME, BrbDisplayBase_ActionConsume, NULL);

    BrbDisplayBase_Init(display_base);
    BrbDisplayBase_ScreenAction(display_base, -1);

    BrbTimerAdd(&glob_brb_base, 5000, 0, BrbDisplayBase_Timer, display_base);
}
/**********************************************************************************************************************/
void BrbSetup485(void)
{
    /* Initialize session data */
    /* Clean up base */
    memset(&glob_rs485_sess, 0, sizeof(BrbRS485Session));
    BrbRS485Session *rs485_sess = (BrbRS485Session *)&glob_rs485_sess;

    rs485_sess->brb_base = &glob_brb_base;
    rs485_sess->pinRO = RS485_RO_PIN;
    rs485_sess->pinDI = RS485_DI_PIN;
    rs485_sess->pinREDE = RS485_REDE_PIN;
    rs485_sess->log_base = glob_log_base;

    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_HANDSHAKE, BrbRS485SessionActionHandShakeCB, &glob_brb_base);
    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_GET_A, BrbRS485SessionActionGetAnalogCB, &glob_brb_base);
    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_SET_A, BrbRS485SessionActionSetAnalogCB, &glob_brb_base);
    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_SET_A_BMP, BrbRS485SessionActionSetAnalogBMPCB, &glob_brb_base);

    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_GET_D, BrbRS485SessionActionGetDigitalCB, &glob_brb_base);
    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_SET_D, BrbRS485SessionActionSetDigitalCB, &glob_brb_base);
    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_SET_D_BMP, BrbRS485SessionActionSetDigitalBMPCB, &glob_brb_base);

    BrbRS485Session_SetEventCB(rs485_sess, RS485_PKT_TYPE_CMD_SET_SCRIPT, BrbRS485SessionActionSetScriptCB, &glob_brb_base);

    BrbRS485Session_Init(rs485_sess, &SerialRS485);

    return;
}
/**********************************************************************************************************************/
void setup()
{
    randomSeed(((analogRead(0) + analogRead(1)) / 2));

    /* Initialize Brb internal data */
    BrbSetup();

    /* Setup Display before anything, because it can display some info, eg logs */
    BrbSetupDisplay();

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

    gerador_base->pin_sensor_ac = SENSOR_VOLTAGE_AC_PIN;
    gerador_base->pin_sensor_dc = SENSOR_VOLTAGE_DC_PIN;

    BrbGeradorBase_Init(gerador_base);
    // BrbGeradorBase_Partida(gerador_base);

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
    BrbSetup485();

    LOG_NOTICE(glob_log_base, "Brb Slave Module - START [%u] - 0.1.2\r\n", micros());
    LOG_NOTICE(glob_log_base, "BrbRS485 - ADDR 0x%02x - %p\r\n", glob_rs485_sess.address, &glob_brb_base);
    LOG_HEAP(glob_log_base);

    return;
}
/**********************************************************************************************************************/
/* RUN FOREVER */
/**********************************************************************************************************************/
void loop()
{
    /* Check for Buttons */
    BrbBtnBase_Loop((BrbBtnBase *)&glob_btn_base);

    if (glob_btn_base.buttons[BRB_BTN_SELECT].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_SELECT].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, BRB_BTN_SELECT);
    }
    else if (glob_btn_base.buttons[BRB_BTN_NEXT].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_NEXT].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, BRB_BTN_NEXT);
    }
    else if (glob_btn_base.buttons[BRB_BTN_PREV].hit > 0)
    {
        glob_btn_base.buttons[BRB_BTN_PREV].hit = BrbDisplayBase_ScreenAction((BrbDisplayBase *)&glob_display_base, BRB_BTN_PREV);
    }

    // LOG_NOTICE(glob_log_base, "BTNS [%d] [%d] [%d]\r\n", glob_btn_base.buttons[BRB_BTN_PREV].state, glob_btn_base.buttons[BRB_BTN_NEXT].state, glob_btn_base.buttons[BRB_BTN_PREV].state);

    /* We have a new message? */
    BrbRS485Session_ReadMessage(&glob_rs485_sess);

    BrbBaseLoop(&glob_brb_base);

    /* Do Loop */
    BrbGeradorBase_Loop(&glob_gerador_base);
    BrbToneBase_Loop(&glob_tone_base);

    // LOG_INFO(glob_log_base, "last: %ld - cur %ld - delay %ld\n", glob_brb_base.us.last, glob_brb_base.us.cur, glob_brb_base.us.delay);
}
/**********************************************************************************************************************/
/* DISPLAY */
/**********************************************************************************************************************/
int BrbDisplayBase_Timer(void *base_ptr, void *cb_data_ptr)
{
    // BrbTimer *timer = (BrbTimer *)base_ptr;
    BrbDisplayBase *display_base = (BrbDisplayBase *)cb_data_ptr;
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    BrbDisplayBase_ScreenAction(display_base, -1);

    // display_base->screen_cur++;

    int delay = 5000;

    switch (gerador_base->state.code)
    {
    case GERADOR_STATE_START_INIT:
    case GERADOR_STATE_START_DELAY:
    case GERADOR_STATE_START_CHECK:
    case GERADOR_STATE_STOP_INIT:
    case GERADOR_STATE_STOP_DELAY:
    case GERADOR_STATE_STOP_CHECK:
    {
        delay = 2000;
        break;
    }
    case GERADOR_STATE_FAILURE:
    {
        delay = 3000;
        break;
    }
    case GERADOR_STATE_RUNNING:
    case GERADOR_STATE_NONE:
    default:
    {
        delay = 5000;
        break;
    }
    }

    BrbTimerAdd(&glob_brb_base, delay, 0, BrbDisplayBase_Timer, display_base);

    return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_ShowInfo(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    char buf[10] = {0};
    byte len = 5;

    int pos_x;
    int pos_y;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, F("General Info"), 10, 10);
    }

    pos_x = 10;
    pos_y = 50;

    if (display_base->screen_cur != display_base->screen_last)
    {
        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x, pos_y);
        display_base->tft->println("Battery");
    }

    display_base->tft->setTextColor(ILI9341_DARKGREEN, ILI9341_WHITE);
    display_base->tft->setFont(BRB_DISPLAY_FONT_VALUE);
    display_base->tft->setTextScale(1);
    dtostrf(gerador_base->info.battery, len, 1, buf);
    display_base->tft->printAlignedPivotedOffseted(buf, gTextAlignTopLeft, gTextPivotTopLeft, pos_x, pos_y + 20, 0, 60);

    display_base->tft->cursorToXY(display_base->tft->getCursorX() + 10, display_base->tft->getCursorY());
    display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
    display_base->tft->setTextScale(1);
    display_base->tft->print("VDC");

    pos_x = pos_x + 160;

    if (display_base->screen_cur != display_base->screen_last)
    {
        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x, pos_y);
        display_base->tft->println("Power");
    }

    display_base->tft->setTextColor(ILI9341_DARKRED, ILI9341_WHITE);
    display_base->tft->setFont(BRB_DISPLAY_FONT_VALUE);
    display_base->tft->setTextScale(1);
    dtostrf(gerador_base->info.power, len, 1, buf);
    display_base->tft->printAlignedPivotedOffseted(buf, gTextAlignTopLeft, gTextPivotTopLeft, pos_x, pos_y + 20, 0, 60);

    display_base->tft->cursorToXY(display_base->tft->getCursorX() + 10, display_base->tft->getCursorY());
    display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
    display_base->tft->setTextScale(1);
    display_base->tft->print("VAC");

    pos_x = 10;
    pos_y = pos_y + 65;

    BrbDisplayBase_DrawArc(display_base, gerador_base->info.gas, 0, 100, pos_x, pos_y, 65, F("Tanque"), ARC_SCHEME_RED2GREEN);

    pos_x = pos_x + 160;

    BrbDisplayBase_DrawArc(display_base, gerador_base->info.load, 0, 30, pos_x, pos_y, 65, F("Amp"), ARC_SCHEME_GREEN2RED);

    return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_ShowTemp(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    // BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    char buf[10] = {0};
    byte len = 5;

    /* This goes to the loop LIB */
    float dht_t = dht_sensor.readTemperature();
    float dht_h = dht_sensor.readHumidity();

    // Compute heat index in Fahrenheit (the default)
    // float dht_hif = dht.computeHeatIndex(f, h);

    // Compute heat index in Celsius (isFahreheit = false)
    float dht_hic = dht_sensor.computeHeatIndex(dht_t, dht_h, false);

    int pos_x;
    int pos_y;

    int sz_w = 224;
    int sz_h = 40;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, F("Temperature Info"), 10, 10);

        // display_base->tft->fillRect(0, 50, sz_w, 15, ILI9341_LIGHTSALMON);
        // display_base->tft->fillRect(sz_w, 50, sz_w, 15, ILI9341_LIMEGREEN);

        // display_base->tft->fillRect(0, 50, 320 - sz_w, 15, ILI9341_LIGHTSALMON);
        // display_base->tft->fillRect(sz_w, 50, 320 - sz_w, 15, ILI9341_LIMEGREEN);
    }

    pos_x = 10;
    pos_y = 50;

    BrbDisplayBase_DrawArcSeg(display_base, isnan(dht_t) ? 0 : dht_t, 0, 130, pos_x, pos_y, 100, F("Celsius"), ARC_SCHEME_GREEN2RED, 0, 3, 5);

    pos_y = 50;
    pos_x = sz_w;

    if (display_base->screen_cur != display_base->screen_last)
    {
        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x, pos_y);
        display_base->tft->println("HUMIDITY");
    }

    // if (dht_h > 999)
    // 	len = 5;

    // display_base->tft->fillRect(pos_x, pos_y + 20, 80, sz_h, ILI9341_WHEAT);
    // display_base->tft->cursorToXY(pos_x, pos_y + 20);
    display_base->tft->setTextColor(ILI9341_DARKBLUE, ILI9341_WHITE);
    display_base->tft->setFont(BRB_DISPLAY_FONT_VALUE);
    display_base->tft->setTextScale(1);

    dtostrf(dht_h, len, 1, buf);
    display_base->tft->printAlignedPivotedOffseted(buf, gTextAlignTopLeft, gTextPivotTopLeft, pos_x, pos_y + 20, 0, 25);

    display_base->tft->cursorToXY(display_base->tft->getCursorX() + 5, display_base->tft->getCursorY());
    display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
    display_base->tft->setTextScale(1);
    display_base->tft->print("%");

    pos_y = pos_y + sz_h + 30;
    pos_x = sz_w;

    if (display_base->screen_cur != display_base->screen_last)
    {
        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x, pos_y);
        display_base->tft->println("HEAT INDEX");
    }

    display_base->tft->setTextColor(ILI9341_DARKBLUE, ILI9341_WHITE);
    display_base->tft->setFont(BRB_DISPLAY_FONT_VALUE);
    display_base->tft->setTextScale(1);
    dtostrf(dht_hic, len, 1, buf);
    display_base->tft->printAlignedPivotedOffseted(buf, gTextAlignTopLeft, gTextPivotTopLeft, pos_x, pos_y + 20, 0, 25);

    return 0;
}
/****************************************************************************************************/
int BrbDisplayBase_ShowControl(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;
    BrbServo *servo_bb;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, F("Controle de Partida"), 10, 10);
    }

    display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
    display_base->tft->setFont(BRB_DISPLAY_FONT_VALUE);
    display_base->tft->setTextScale(1);

    int btn_h = 65;
    int btn_w = 300;
    int btn_y = 120;
    int btn_x = 10;

    servo_bb = BrbServoGrabByPin(&glob_brb_base, gerador_base->pin_servo);

    display_base->tft->fillRect(10, 50, 300, 40, ILI9341_WHITESMOKE);

    const __FlashStringHelper *text_ptr;

    text_ptr = BrbGeradorBase_GetState(gerador_base);

    display_base->tft->cursorToXY(10, 50);
    display_base->tft->print("STATUS: ");
    display_base->tft->cursorToXY(display_base->tft->getCursorX() + 10, 50);
    display_base->tft->print(text_ptr);

    switch (gerador_base->state.code)
    {
    case GERADOR_STATE_START_INIT:
    case GERADOR_STATE_START_DELAY:
    case GERADOR_STATE_START_CHECK:
    case GERADOR_STATE_RUNNING:
    {
        BrbDisplayBase_DrawBtn(display_base, btn_x, btn_y, btn_w, btn_h, F("STOP"), ILI9341_DARKRED, ILI9341_WHITE);
        break;
    }
    case GERADOR_STATE_FAILURE:
    {

        text_ptr = BrbGeradorBase_GetState(gerador_base);

        display_base->tft->cursorToXY(10, 90);
        display_base->tft->print("FAIL: ");
        display_base->tft->cursorToXY(display_base->tft->getCursorX() + 10, 90);
        display_base->tft->print(text_ptr);

        BrbDisplayBase_DrawBtn(display_base, btn_x, btn_y, btn_w, btn_h, F("IGNORE"), ILI9341_DARKRED, ILI9341_WHITE);
        break;
    }
    case GERADOR_STATE_STOP_INIT:
    case GERADOR_STATE_STOP_DELAY:
    case GERADOR_STATE_STOP_CHECK:
    case GERADOR_STATE_NONE:
    {
        BrbDisplayBase_DrawBtn(display_base, btn_x, btn_y, btn_w, btn_h, F("START"), ILI9341_DARKRED, ILI9341_WHITE);
        break;
    }
    default:
    {
        /**/
    }
    }

    display_base->tft->fillRect(10, 200, 140, 35, ILI9341_WHITESMOKE);

    if (servo_bb)
    {
        display_base->tft->cursorToXY(10, 200);
        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->print("BB: ");
        display_base->tft->cursorToXY(display_base->tft->getCursorX() + 10, 200);
        display_base->tft->print(servo_bb->pos_cur);
    }

    display_base->tft->fillRect(170, 200, 140, 35, ILI9341_WHITE);

    display_base->tft->cursorToXY(170, 200);
    display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
    display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
    display_base->tft->setTextScale(1);
    display_base->tft->print("RETRY: ");
    display_base->tft->cursorToXY(display_base->tft->getCursorX() + 10, 200);
    display_base->tft->print(gerador_base->state.retry, DEC);

    display_base->tft->cursorToXY(170, 215);
    display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
    display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
    display_base->tft->setTextScale(1);
    display_base->tft->print("DELTA: ");
    display_base->tft->cursorToXY(display_base->tft->getCursorX() + 10, 215);
    display_base->tft->print(gerador_base->state.delta, DEC);

    return 0;
}
/****************************************************************************************************/
int BrbDisplayBase_ShowConsume(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    char buf[10] = {0};
    byte len = 5;

    int pos_x;
    int pos_y;

    int sz_w = 224;
    // int sz_h = 40;

    if (display_base->screen_cur != display_base->screen_last)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, F("Consume"), 10, 10);

        // display_base->tft->fillRect(0, 50, sz_w, 15, ILI9341_LIGHTSALMON);
        // display_base->tft->fillRect(sz_w, 50, sz_w, 15, ILI9341_LIMEGREEN);

        // display_base->tft->fillRect(0, 50, 320 - sz_w, 15, ILI9341_LIGHTSALMON);
        // display_base->tft->fillRect(sz_w, 50, 320 - sz_w, 15, ILI9341_LIMEGREEN);
    }

    pos_x = 10;
    pos_y = 50;

    double value_dec = ((double)(gerador_base->data.hourmeter_time) / 60.0);

    BrbDisplayBase_DrawArcSeg(display_base, value_dec, 0, 1300, pos_x, pos_y, 100, F("Hours"), ARC_SCHEME_GREEN2RED, 0, 3, 5);

    if (display_base->screen_cur != display_base->screen_last)
    {
        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
        display_base->tft->setTextScale(2);
        display_base->tft->cursorToXY(pos_x, pos_y);
        display_base->tft->printAlignedPivotedOffseted(F("REMAIN"), gTextAlignTopLeft, gTextPivotTopLeft, pos_x, pos_y + 140, 0, 25);
    }

    display_base->tft->setTextColor(ILI9341_DARKGRAY, ILI9341_WHITE);
    display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
    display_base->tft->setTextScale(2);

    dtostrf((1300 - value_dec), len, 1, buf);
    display_base->tft->printAlignedPivotedOffseted(buf, gTextAlignTopLeft, gTextPivotTopLeft, pos_x + 120, pos_y + 140, 0, 25);

    pos_y = 50;
    pos_x = sz_w;

    if (display_base->screen_cur != display_base->screen_last)
    {
        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x, pos_y);
        display_base->tft->println("Consume");
    }

    display_base->tft->setTextColor(ILI9341_DARKBLUE, ILI9341_WHITE);
    display_base->tft->setFont(BRB_DISPLAY_FONT_VALUE);
    display_base->tft->setTextScale(1);
    dtostrf(value_dec * 3.14, len, 1, buf);
    display_base->tft->printAlignedPivotedOffseted(buf, gTextAlignTopLeft, gTextPivotTopLeft, pos_x, pos_y + 20, 0, 25);

    display_base->tft->cursorToXY(display_base->tft->getCursorX() + 5, display_base->tft->getCursorY());
    display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
    display_base->tft->setTextScale(1);
    display_base->tft->print("Lts");

    pos_y = pos_y + 50;
    pos_x = sz_w;

    if (display_base->screen_cur != display_base->screen_last)
    {
        display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
        display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);
        display_base->tft->setTextScale(1);
        display_base->tft->cursorToXY(pos_x, pos_y);
        display_base->tft->println("RESET");
    }

    display_base->tft->setTextColor(ILI9341_DARKBLUE, ILI9341_WHITE);
    display_base->tft->setFont(BRB_DISPLAY_FONT_VALUE);
    display_base->tft->setTextScale(1);
    display_base->tft->print(1);
    display_base->tft->cursorToXY(pos_x, pos_y + 20);
    display_base->tft->print(gerador_base->data.hourmeter_reset);

    return 0;
}
/****************************************************************************************************/
int BrbDisplayBase_ActionInfo(void *brb_base_ptr, void *display_base_ptr)
{
    // BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    // BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    return 0;
}
/****************************************************************************************************/
int BrbDisplayBase_ActionControl(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    if (!display_base->flags.on_action)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, F("Power Control"), 10, 10);

        display_base->tft->setFont(BRB_DISPLAY_FONT_VALUE);
        display_base->tft->setTextScale(1);

        const __FlashStringHelper *text_ptr;

        text_ptr = BrbGeradorBase_GetStateAction(gerador_base);

        display_base->tft->printAtPivoted(text_ptr, 160, 70, gTextPivotTopCenter);

        display_base->flags.on_action = 1;
    }
    else if (display_base->action_code == BRB_BTN_SELECT)
    {
        if (display_base->user_int == 1)
        {
            switch (gerador_base->state.code)
            {
            case GERADOR_STATE_NONE:
            case GERADOR_STATE_STOP_INIT:
            case GERADOR_STATE_STOP_DELAY:
            case GERADOR_STATE_STOP_CHECK:
            {
                BrbGeradorBase_Start(gerador_base);
                break;
            }
            case GERADOR_STATE_START_INIT:
            case GERADOR_STATE_START_DELAY:
            case GERADOR_STATE_START_CHECK:
            case GERADOR_STATE_RUNNING:
            {
                BrbGeradorBase_Stop(gerador_base);
                break;
            }
            case GERADOR_STATE_FAILURE:
            {
                BrbGeradorBase_FailureConfirm(gerador_base);
                break;
            }
            default:
            {
                break;
            }
            }
        }

        display_base->flags.on_action = 0;
        display_base->user_int = 0;
        display_base->screen_last = -1;

        BrbDisplayBase_ScreenAction(display_base, -1);

        return 0;
    }
    else if ((display_base->action_code == BRB_BTN_NEXT) || (display_base->action_code == BRB_BTN_PREV))
    {
        display_base->user_int = !display_base->user_int;
    }

    display_base->tft->setFont(BRB_DISPLAY_FONT_VALUE);
    display_base->tft->setTextScale(1);

    BrbDisplayBase_DrawBtn(display_base, 20, 120, 120, 75, F("SIM"), display_base->user_int ? ILI9341_DARKRED : ILI9341_LIGHTGREY, display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);
    BrbDisplayBase_DrawBtn(display_base, 170, 120, 120, 75, F("NAO"), !display_base->user_int ? ILI9341_DARKRED : ILI9341_LIGHTGREY, !display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);

    return 0;
}
/****************************************************************************************************/
int BrbDisplayBase_ActionConsume(void *brb_base_ptr, void *display_base_ptr)
{
    BrbDisplayBase *display_base = (BrbDisplayBase *)display_base_ptr;
    BrbGeradorBase *gerador_base = (BrbGeradorBase *)&glob_gerador_base;

    if (!display_base->flags.on_action)
    {
        BrbDisplayBase_SetBg(display_base);
        BrbDisplayBase_SetTitle(display_base, F("Consume"), 10, 10);

        display_base->tft->setFont(BRB_DISPLAY_FONT_TITLE);
        display_base->tft->setTextScale(2);
        display_base->tft->printAtPivoted(F("Resetar Horimetro?"), 160, 80, gTextPivotMiddleCenter); // Units display

        display_base->flags.on_action = 1;
    }
    else if (display_base->action_code == BRB_BTN_SELECT)
    {
        if (display_base->user_int == 1)
        {
            BrbGeradorBase_HourmeterReset(gerador_base);
        }

        display_base->flags.on_action = 0;
        display_base->user_int = 0;
        display_base->screen_last = -1;

        BrbDisplayBase_ScreenAction(display_base, -1);

        return 0;
    }
    else if ((display_base->action_code == BRB_BTN_NEXT) || (display_base->action_code == BRB_BTN_PREV))
    {
        display_base->user_int = !display_base->user_int;
    }

    BrbDisplayBase_DrawBtn(display_base, 20, 120, 120, 75, F("SIM"), display_base->user_int ? ILI9341_DARKRED : ILI9341_LIGHTGREY, display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);
    BrbDisplayBase_DrawBtn(display_base, 170, 120, 120, 75, F("NAO"), !display_base->user_int ? ILI9341_DARKRED : ILI9341_LIGHTGREY, !display_base->user_int ? ILI9341_WHITE : ILI9341_BLACK);

    return 0;
}
/**********************************************************************************************************************/