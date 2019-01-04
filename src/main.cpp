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
// #include <EEPROM.h>
#include <BrbLogBase.h>
#include <BrbBase.h>
#include <BrbRS485Session.h>
#include <avr/wdt.h>

/* Global control structures */
BrbRS485Session glob_rs485_sess_data;
BrbRS485Session *glob_rs485_sess;
BrbLogBase *glob_log_base;
BrbBase glob_brb_base;

#define DI_PIN 50   /* TX PIN */
#define RO_PIN 51  /* RX PIN */
#define REDE_PIN 52 /* TOGGLE PIN (RE + DE) */

// #define DI_PIN 2   /* TX PIN */
// #define RO_PIN 3   /* RX PIN */
// #define REDE_PIN 4 /* TOGGLE PIN (RE + DE) */

/**********************************************************************************************************************/

/* receive pin, transmit pin */
SoftwareSerial SerialRS485(RO_PIN, DI_PIN);

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
    BrbRS485Session *rs485_sess         = (BrbRS485Session *)base_ptr;
    BrbRS485PacketData *pkt_recv        = (BrbRS485PacketData *)buffer_ptr;    
    BrbRS485PacketData *pkt_reply       = (BrbRS485PacketData *)&rs485_sess->pkt.out.data;
    BrbRS485PacketPinData *pin_data     = (BrbRS485PacketPinData *)(pkt_reply + 1);

    int op_status;
    int pin_begin;
    int pin_max;
    int i;

    /* Adjust pin query */
    if (pkt_recv->val < MIN_ANA_PIN || pkt_recv->val >= MAX_ANA_PIN)
    {
        pin_begin               = MIN_ANA_PIN;
        pin_max                 = MAX_ANA_PIN;
    }
    else
    {
        pin_begin               = pkt_recv->val;
        pin_max                 = pkt_recv->val + 1;
    }
    
    LOG_DEBUG(glob_log_base, "GET ANALOG - [%u] - [%d][%d]\n", pkt_recv->val, pin_begin, pin_max);

    /* Inverse the order two reply */
    pkt_reply->hdr.dst          = pkt_recv->hdr.src;
    pkt_reply->hdr.src          = glob_rs485_sess->address;
    pkt_reply->hdr.id           = pkt_recv->hdr.id;
    pkt_reply->hdr.type         = RS485_PKT_TYPE_CMD_GET_A;
    pkt_reply->hdr.len          = sizeof(BrbRS485PacketData) + (sizeof(BrbRS485PacketPinData) * (pin_max - pin_begin));

    for (i = pin_begin; i < pin_max; i++) 
    {
        pin_data->pin           = i;
        pin_data->type          = 0;
        pin_data->mode          = BrbBase_PinGetMode(glob_analog_pins[i]);
        pin_data->value         = analogRead(glob_analog_pins[i]);

        LOG_DEBUG(glob_log_base, "GET ANALOG [%u] - [%d]\n", i, pin_data->value);

        pin_data++;
    }

    /* Send CRC */
    op_status                   = BrbRS485Session_SendPacket(rs485_sess, (byte *)pkt_reply, pkt_reply->hdr.len);

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
    }
    
    BrbBase_PinSave(&glob_brb_base);

    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
int BrbRS485SessionActionGetDigitalCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485Session *rs485_sess         = (BrbRS485Session *)base_ptr;
    BrbRS485PacketData *pkt_recv        = (BrbRS485PacketData *)buffer_ptr;    
    BrbRS485PacketData *pkt_reply       = (BrbRS485PacketData *)&rs485_sess->pkt.out.data;
    BrbRS485PacketPinData *pin_data     = (BrbRS485PacketPinData *)(pkt_reply + 1);
    int op_status;
    int pin_begin;
    int pin_max;
    int i;

    /* Adjust pin query */
    if (pkt_recv->val < 0 || pkt_recv->val >= MAX_DIG_PIN)
    {
        pin_begin               = 0;
        pin_max                 = MAX_DIG_PIN;
    }
    else
    {
        pin_begin               = pkt_recv->val;
        pin_max                 = pkt_recv->val + 1;
    }
    
    LOG_INFO(glob_log_base, "GET DIGITAL - [%u] - [%d][%d]\n", pkt_recv->val, pin_begin, pin_max);

    /* Inverse the order two reply */
    pkt_reply->hdr.dst          = pkt_recv->hdr.src;
    pkt_reply->hdr.src          = glob_rs485_sess->address;
    pkt_reply->hdr.id           = pkt_recv->hdr.id;
    pkt_reply->hdr.type         = RS485_PKT_TYPE_CMD_GET_D;
    pkt_reply->hdr.len          = sizeof(BrbRS485PacketData) + (sizeof(BrbRS485PacketPinData) * (pin_max - pin_begin));

    for (i = pin_begin; i < pin_max; i++) 
    {
        pin_data->pin           = i;
        pin_data->type          = 1;
        pin_data->value         = digitalRead(i);
        pin_data->mode          = BrbBase_PinGetMode(i);
        pin_data++;
    }

    /* Send CRC */
    op_status                   = BrbRS485Session_SendPacket(rs485_sess, (byte *)pkt_reply, pkt_reply->hdr.len);

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
    }

    BrbBase_PinSave(&glob_brb_base);
    
    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
int BrbRS485SessionActionSetScriptCB(void *base_ptr, int action_code, const void *buffer_ptr, unsigned int buffer_sz, void *cb_data_ptr)
{
    BrbRS485PacketSetScript *pkt_recv_set   = (BrbRS485PacketSetScript *)buffer_ptr;    
    BrbBase *brb_base                       = (BrbBase *)cb_data_ptr;
    BrbMicroScript *script;

    // LOG_WARN(glob_log_base, "SCRIPT SET [%p] [%p] [%d]\r\n", &glob_brb_base, brb_base, pkt_recv_set->script_id);

    // brb_base->log_base->serial->flush();

    script      = BrbMicroScriptGrabFree(brb_base);
    // script    = BrbMicroScriptGrabByID(&glob_brb_base, pkt_recv_set->script_id);
    // script    = BrbMicroScriptSetByID((BrbBase *)&glob_brb_base, (BrbMicroScript *)&pkt_recv_set->script);

    // LOG_WARN(glob_log_base, "SCRIPT SET [%p] [%p] [%o]\r\n", &glob_brb_base, brb_base, script);
    
    // brb_base->log_base->serial->flush();

    if (!script)
    {
        LOG_WARN(glob_log_base, "NO SCRIPT [%u]\r\n",  pkt_recv_set->script_id);

        return RS485_PKT_RETURN_ACK_FAIL;
    }
        
    // BrbMicroScriptOPAddDelay(brb_base, script, 500);
    // BrbMicroScriptOPAddSetDig(brb_base, script, 9, OUTPUT, LOW);
    // BrbMicroScriptOPAddSetDig(brb_base, script, 10, OUTPUT, LOW);
    // BrbMicroScriptOPAddDelay(brb_base, script, 1000);
    // BrbMicroScriptOPAddSetDig(brb_base, script, 9, OUTPUT, HIGH);
    // BrbMicroScriptOPAddSetDig(brb_base, script, 10, OUTPUT, HIGH);
    // BrbMicroScriptOPAddDelay(brb_base, script, 250);
    // BrbMicroScriptOPAddSetDig(brb_base, script, 9, OUTPUT, LOW);
    // BrbMicroScriptOPAddSetDig(brb_base, script, 10, OUTPUT, LOW);
    // BrbMicroScriptOPAddDelay(brb_base, script, 500);
    // BrbMicroScriptOPAddSetDig(brb_base, script, 9, OUTPUT, HIGH);
    // BrbMicroScriptOPAddSetDig(brb_base, script, 10, OUTPUT, HIGH);

    memcpy(&script->code, &pkt_recv_set->code, sizeof(BrbMicroCode));
    
    script->flags.persist   = pkt_recv_set->persist;
    script->flags.active    = 1;
    // script->flags.finished  = 0;

    // BrbLogBase_HexDump(brb_base->log_base, (char *)script, sizeof(BrbMicroCode));

    // LOG_WARN(glob_log_base, "[%p] SCRIPT OK [%u] - [%lu]\r\n", script, script->script_id, script->delay_until_ms);

    // delayMicroseconds(5);

    // script            = (BrbMicroScript *)&pkt_recv_set->script;

    // LOG_WARN(glob_log_base, "[%p] SCRIPT OK [%u] - [%lu]\r\n", script, script->script_id, script->delay_until_ms);
    
    return RS485_PKT_RETURN_ACK_SUCCESS;
}
/**********************************************************************************************************************/
/* RUN ONE TIME ON START */
/**********************************************************************************************************************/
void BrbSetup(void)
{
    /* Clean up base */
    memset(&glob_brb_base, 0, sizeof(BrbBase));

    glob_log_base           = BrbLogBase_New();
    BrbLogBase_Init(glob_log_base, &Serial);

    glob_brb_base.log_base  = glob_log_base;

    BrbBaseInit(&glob_brb_base);

    return;
}
/**********************************************************************************************************************/
void BrbSetup485(void)
{
    /* Initialize session data */
    // glob_rs485_sess = BrbRS485Session_New(&glob_brb_base);
    glob_rs485_sess = &glob_rs485_sess_data;
    glob_rs485_sess->brb_base = &glob_brb_base;
    glob_rs485_sess->pinRO = RO_PIN;
    glob_rs485_sess->pinDI = DI_PIN;
    glob_rs485_sess->pinREDE = REDE_PIN;
    glob_rs485_sess->log_base = glob_log_base;

    BrbRS485Session_SetEventCB(glob_rs485_sess, RS485_PKT_TYPE_HANDSHAKE, BrbRS485SessionActionHandShakeCB, &glob_brb_base);
    BrbRS485Session_SetEventCB(glob_rs485_sess, RS485_PKT_TYPE_CMD_GET_A, BrbRS485SessionActionGetAnalogCB, &glob_brb_base);
    BrbRS485Session_SetEventCB(glob_rs485_sess, RS485_PKT_TYPE_CMD_SET_A, BrbRS485SessionActionSetAnalogCB, &glob_brb_base);
    BrbRS485Session_SetEventCB(glob_rs485_sess, RS485_PKT_TYPE_CMD_SET_A_BMP, BrbRS485SessionActionSetAnalogBMPCB, &glob_brb_base);

    BrbRS485Session_SetEventCB(glob_rs485_sess, RS485_PKT_TYPE_CMD_GET_D, BrbRS485SessionActionGetDigitalCB, &glob_brb_base);
    BrbRS485Session_SetEventCB(glob_rs485_sess, RS485_PKT_TYPE_CMD_SET_D, BrbRS485SessionActionSetDigitalCB, &glob_brb_base);
    BrbRS485Session_SetEventCB(glob_rs485_sess, RS485_PKT_TYPE_CMD_SET_D_BMP, BrbRS485SessionActionSetDigitalBMPCB, &glob_brb_base);

    BrbRS485Session_SetEventCB(glob_rs485_sess, RS485_PKT_TYPE_CMD_SET_SCRIPT, BrbRS485SessionActionSetScriptCB, &glob_brb_base);

    // BrbRS485Session_Init(glob_rs485_sess, &Serial1);
    BrbRS485Session_Init(glob_rs485_sess, &SerialRS485);

    return;
}
/**********************************************************************************************************************/
void setup()
{
    randomSeed(((analogRead(0) + analogRead(1)) / 2));

    /* Initialize Brb internal data */
    BrbSetup();

    BrbSetup485();

    LOG_NOTICE(glob_log_base, "Brb Slave Module - START [%u] - 0.1.2\r\n", micros());
    LOG_NOTICE(glob_log_base, "BrbRS485 - ADDR 0x%02x - %p\r\n", glob_rs485_sess->address, &glob_brb_base);
    LOG_HEAP(glob_log_base);

    return;
}
/**********************************************************************************************************************/
/* RUN FOREVER */
/**********************************************************************************************************************/
void loop()
{
    /* We have a new message? */
    BrbRS485Session_ReadMessage(glob_rs485_sess);

    BrbBaseLoop(&glob_brb_base);
    //LOG_INFO(glob_log_base, "last: %ld - cur %ld - delay %ld\n", glob_brb_base.us.last, glob_brb_base.us.cur, glob_brb_base.us.delay);
}
/**********************************************************************************************************************/