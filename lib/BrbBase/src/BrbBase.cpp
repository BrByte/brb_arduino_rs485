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
    LOG_DEBUG(brb_base->log_base, "BRB - INITIALIZE\r\n");    

    // /* Read pins from EEPROM */
    // BrbBase_PinLoad(brb_base);

    /* Check pins */
    // BrbBase_PinCheck(brb_base);

    /* Load data */
    BrbBase_DataLoad(brb_base);

    /* Initialize Script Base */
    BrbMicroScriptBase_Init(&brb_base->script_base);

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

    brb_base->ms.lifetime_delay = brb_base->ms.lifetime_delay + brb_base->ms.delay;

    /* 5 seconds delay */
    if (brb_base->ms.lifetime_delay > 5000)
    {
        brb_base->ms.lifetime_delay = (brb_base->ms.lifetime_delay - 5000);

        brb_base->ms.lifetime_sec = brb_base->ms.lifetime_sec + 5;
        brb_base->data.lifetime_sec = brb_base->data.lifetime_sec + 5;

        /* Save every 60 seconds */
        if (brb_base->ms.lifetime_sec > 60)
        {
            brb_base->ms.lifetime_sec = (brb_base->ms.lifetime_sec - 60);

            BrbBase_DataSave(brb_base);
        }
    }

    /* Dispatch Timers */
    BrbTimerDispatch(brb_base);
    
    // /* Check Configuration - to be done */
    // BrbConfigBase_Loop(&brb_base->config_base);

    /* Run all scripts */
    BrbMicroScriptBase_RunAll(&brb_base->script_base);

    return;
}
/**********************************************************************************************************************/
void BrbBase_DataLoad(BrbBase *brb_base)
{
    LOG_DEBUG(brb_base->log_base, "BRB - DATA LOAD\r\n");

    /* Read EEPROM */
    BrbBase_EEPROMRead(brb_base, (uint8_t *)&brb_base->data, sizeof(brb_base->data), 0);

    return;
}
/**********************************************************************************************************************/
void BrbBase_DataSave(BrbBase *brb_base)
{
    LOG_DEBUG(brb_base->log_base, "BRB - DATA SAVE\r\n");

    /* Save to EEPROM */
    BrbBase_EEPROMWrite(brb_base, (uint8_t *)&brb_base->data, sizeof(brb_base->data), 0);

    return;
}
/**********************************************************************************************************************/
void BrbBase_PinLoad(BrbBase *brb_base)
{
    LOG_DEBUG(brb_base->log_base, "BRB - PIN LOAD\r\n");  

    /* Read EEPROM */
    // BrbBase_EEPROMRead(brb_base, (uint8_t *)&brb_base->pin_data, sizeof(brb_base->pin_data), BRB_EEPROM_OFFSET);

    return;
}
/**********************************************************************************************************************/
void BrbBase_PinCheck(BrbBase *brb_base)
{
    // LOG_DEBUG(brb_base->log_base, "BRB - PIN CHECK\r\n");

    // /* Reset pin state */
    // BrbBasePinData *pin_data;
    // int i;

    // LOG_DEBUG(brb_base->log_base, "PINS A %u %u %u %u %u %u %u %u\r\n", A0, A1, A2, A3, A4, A5, A6, A7);

    // for (i = MIN_DIG_PIN; i < TOTAL_PINS; i++)
    // {
    //     /* Grab pin data */
    //     pin_data        = (BrbBasePinData *)&brb_base->pin_data[i];

    //     /* Not a persist one */
    //     if (!pin_data->persist || (pin_data->mask != BRB_EEPROM_MASK))
    //         continue;

    //     BrbBase_PinSet(brb_base, i, pin_data->mode, pin_data->value);
    // }

    return;
}
/**********************************************************************************************************************/
// void BrbBase_PinSet(BrbBase *brb_base, int pin_num, int pin_mode, int pin_value)
// {
//     LOG_DEBUG(brb_base->log_base, "BRB - PIN SET\r\n");

//     byte pin_code;
//     //  || pin_num > sizeof(glob_analog_pins)
//     if (pin_num < MAX_DIG_PIN)
//         pin_code        = pin_num;
//     else
//         pin_code        = glob_analog_pins[MAX_DIG_PIN - pin_num];
    
//     // LOG_NOTICE(brb_base->log_base, "SET PIN [%u/%u] - mode:%u v:%u\r\n", pin_num, pin_code, pin_mode, pin_value);

//     /* Check output, set value */
//     if (pin_mode == OUTPUT)
//     {
//         pinMode(pin_code, OUTPUT);

//         if (pin_code < MAX_DIG_PIN)
//             digitalWrite(pin_code, (pin_value == HIGH) ? HIGH : LOW);
//         else
//             analogWrite(pin_code, pin_value);
//     }
//     else if (pin_mode == INPUT_PULLUP)
//         pinMode(pin_code, INPUT_PULLUP);
//     else if (pin_mode == INPUT)
//         pinMode(pin_code, INPUT);

//     return;
// }
/**********************************************************************************************************************/
void BrbBase_PinSave(BrbBase *brb_base)
{
    // LOG_DEBUG(brb_base->log_base, "BRB - PIN SAVE\r\n");

    // /* Read EEPROM */
    // BrbBase_EEPROMWrite(brb_base, (uint8_t *)&brb_base->pin_data, sizeof(brb_base->pin_data), BRB_EEPROM_OFFSET);

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
uint8_t BrbBase_PinGetAnalogPin(uint8_t pin)
{        
#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
	if (pin < 18) pin += 18; // allow for channel or pin numbers
#endif
	pin = analogPinToChannel(pin);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	if (pin < 54) pin += 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
	if (pin < 18) pin += 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
	if (pin < 24) pin += 24; // allow for channel or pin numbers
#else
	if (pin < 14) pin += 14; // allow for channel or pin numbers
#endif

    return pin;
}
/**********************************************************************************************************************/
int BrbBase_EEPROMRead(BrbBase *brb_base, uint8_t *data_ptr, uint8_t data_sz, uint8_t eeprom_offset)
{
    int byte_read;

    if (!brb_base || !data_ptr)
        return -1;

    LOG_DEBUG(brb_base->log_base, "BRB - EEPROM read %d %d\r\n", eeprom_offset, data_sz);

    byte_read = EEPROM.read(eeprom_offset++);

    if (byte_read != BRB_EEPROM_MAGIC)
    {
        /* Reset info */
        memset(data_ptr, 0, data_sz);

        return 0;
    }

    for (int i = 0; i < data_sz; i++)
    {
        data_ptr[i] = EEPROM.read(eeprom_offset++);
    }

    byte_read = EEPROM.read(eeprom_offset++);

    if (byte_read != BRB_EEPROM_MAGIC)
    {
        /* Reset info */
        memset(data_ptr, 0, data_sz);

        return 0;
    }

    return 0;
}
/**********************************************************************************************************************/
int BrbBase_EEPROMWrite(BrbBase *brb_base, uint8_t *data_ptr, uint8_t data_sz, uint8_t eeprom_offset)
{
    if (!brb_base || !data_ptr)
        return -1;

    LOG_DEBUG(brb_base->log_base, "BRB - EEPROM write %d %d\r\n", eeprom_offset, data_sz);

    EEPROM.write(eeprom_offset++, BRB_EEPROM_MAGIC);

    for (int i = 0; i < data_sz; i++)
    {
        EEPROM.write(eeprom_offset++, data_ptr[i]);
    }

    EEPROM.write(eeprom_offset++, BRB_EEPROM_MAGIC);

    return 0;
}
/**********************************************************************************************************************/
int BrbBase_FreeRAM(void)
{
#if defined(ESP8266) || defined(ESP32)
	return ESP.getFreeHeap();
#elif defined(AVR)
	// taken from
	// https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
	extern uintptr_t __heap_start;
	extern void *__brkval;
	intptr_t v;
	return (uintptr_t)&v -
		   (__brkval == 0 ? (uintptr_t)&__heap_start : (uintptr_t)__brkval);
#else
	return 0;
#endif
}
/**********************************************************************************************************************/