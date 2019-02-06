/*
 * BrbBase.cpp
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

#include "BrbBase.h"
#include <EEPROM.h>

/**********************************************************************************************************************/
void BrbBaseInit(BrbBase *brb_base)
{
    LOG_WARN(brb_base->log_base, "Initialize BrbBase\r\n");    

    /* Read pins from EEPROM */
    BrbBase_PinLoad(brb_base);

    /* Check pins */
    BrbBase_PinCheck(brb_base);

    /* Initialize Scripts */
    BrbMicroScriptInit(brb_base);

    return;
}
/**********************************************************************************************************************/
void BrbBaseLoop(BrbBase *brb_base)
{
    brb_base->stats.loop_cnt++;

    /* Update timer counts */
    brb_base->ms.last = brb_base->ms.cur;
    brb_base->ms.cur = millis();
    brb_base->ms.delay = brb_base->ms.cur - brb_base->ms.last;

    brb_base->us.last = brb_base->us.cur;
    brb_base->us.cur = micros();
    brb_base->us.delay = brb_base->us.cur - brb_base->us.last;

    /* Dispatch timers and scripts */
    BrbTimerDispatch(brb_base);
    BrbMicroScriptRunAll(brb_base);

    return;
}
/**********************************************************************************************************************/
void BrbBase_PinLoad(BrbBase *brb_base)
{
    /* Read EEPROM */
    BrbBase_EEPROMRead(brb_base, (uint8_t *)&brb_base->pin_data, sizeof(brb_base->pin_data), BRB_PIN_DATA_OFFSET);

    return;
}
/**********************************************************************************************************************/
void BrbBase_PinCheck(BrbBase *brb_base)
{
    /* Reset pin state */
    BrbBasePinData *pin_data;
    int i;

    LOG_DEBUG(brb_base->log_base, "PINS A %u %u %u %u %u %u %u %u\r\n", A0, A1, A2, A3, A4, A5, A6, A7);

    for (i = MIN_DIG_PIN; i < TOTAL_PINS; i++)
    {
        /* Grab pin data */
        pin_data        = (BrbBasePinData *)&brb_base->pin_data[i];

        /* Not a persist one */
        if (!pin_data->persist || (pin_data->mask != BRB_PIN_DATA_MASK))
            continue;

        BrbBase_PinSet(brb_base, i, pin_data->mode, pin_data->value);
    }

    return;
}
/**********************************************************************************************************************/
void BrbBase_PinSet(BrbBase *brb_base, int pin_num, int pin_mode, int pin_value)
{
    byte pin_code;
    //  || pin_num > sizeof(glob_analog_pins)
    if (pin_num < MAX_DIG_PIN)
        pin_code        = pin_num;
    else
        pin_code        = glob_analog_pins[MAX_DIG_PIN - pin_num];
    
    // LOG_NOTICE(brb_base->log_base, "SET PIN [%u/%u] - mode:%u v:%u\r\n", pin_num, pin_code, pin_mode, pin_value);

    /* Check output, set value */
    if (pin_mode == OUTPUT)
    {
        pinMode(pin_code, OUTPUT);

        if (pin_code < MAX_DIG_PIN)
            digitalWrite(pin_code, (pin_value == HIGH) ? HIGH : LOW);
        else
            analogWrite(pin_code, pin_value);
    }
    else if (pin_mode == INPUT_PULLUP)
        pinMode(pin_code, INPUT_PULLUP);
    else if (pin_mode == INPUT)
        pinMode(pin_code, INPUT);

    return;
}
/**********************************************************************************************************************/
void BrbBase_PinSave(BrbBase *brb_base)
{
    /* Read EEPROM */
    BrbBase_EEPROMWrite(brb_base, (uint8_t *)&brb_base->pin_data, sizeof(brb_base->pin_data), BRB_PIN_DATA_OFFSET);

    return;
}
/**********************************************************************************************************************/
uint8_t BrbBase_PinGetMode(uint8_t pin)
{
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);

    // I don't see an option for mega to return this, but whatever...
    // if (NOT_A_PIN == port)
    //     return 0xFF;

    // // Is there a bit we can check?
    // if (0 == bit)
    //     return 0xFF;

    // // Is there only a single bit set?
    // if ((bit & bit) - 1)
    //     return 0xFF;

    volatile uint8_t *reg = portModeRegister(port);

    if (*reg & bit)
        return OUTPUT;

    volatile uint8_t *out = portOutputRegister(port);

    if (*out & bit)
        return INPUT_PULLUP;
    else
        return INPUT;
}
/**********************************************************************************************************************/
int BrbBase_EEPROMRead(BrbBase *brb_base, uint8_t *data_ptr, uint8_t data_sz, uint8_t eeprom_offset)
{
    int byte_read;

    if (!brb_base || !data_ptr)
        return -1;

    byte_read = EEPROM.read(eeprom_offset);

    if (byte_read != BRB_PIN_DATA_MAGIC)
    {
        /* Reset info */
        memset(data_ptr, 0, data_sz);

        return 0;
    }

    for (int i = 0; i < data_sz; i++)
    {
        data_ptr[i] = EEPROM.read(eeprom_offset + i + 1);
    }

    return 0;
}
/**********************************************************************************************************************/
int BrbBase_EEPROMWrite(BrbBase *brb_base, uint8_t *data_ptr, uint8_t data_sz, uint8_t eeprom_offset)
{
    if (!brb_base || !data_ptr)
        return -1;

    EEPROM.write(eeprom_offset, BRB_PIN_DATA_MAGIC);

    for (int i = 0; i < data_sz; i++)
    {
        EEPROM.write(eeprom_offset + i + 1, data_ptr[i]);
    }

    return 0;
}
/**********************************************************************************************************************/