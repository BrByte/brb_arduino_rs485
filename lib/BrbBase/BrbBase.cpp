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

    // if ((brb_base->buzzer.pin > 1) && brb_base->buzzer.pin < TOTAL_PINS)
    // {
    //     // brb_base->buzzer.tones[0].note = NOTE_As2;
    //     // brb_base->buzzer.tones[0].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[1].note = NOTE_REST;
    //     // brb_base->buzzer.tones[1].duration = TONE_DUR_SIXTEENTH;
    //     // brb_base->buzzer.tones[2].note = NOTE_Gs3;
    //     // brb_base->buzzer.tones[2].duration = TONE_DUR_SIXTEENTH;
    //     // brb_base->buzzer.tones[3].note = NOTE_REST;
    //     // brb_base->buzzer.tones[3].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[4].note = NOTE_As3;
    //     // brb_base->buzzer.tones[4].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[5].note = NOTE_REST;
    //     // brb_base->buzzer.tones[5].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[6].note = NOTE_C3;
    //     // brb_base->buzzer.tones[6].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[7].note = NOTE_Cs3;
    //     // brb_base->buzzer.tones[7].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[8].note = NOTE_D3;
    //     // brb_base->buzzer.tones[8].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[9].note = NOTE_Ds3;
    //     // brb_base->buzzer.tones[9].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[10].note = NOTE_REST;
    //     // brb_base->buzzer.tones[10].duration = TONE_DUR_SIXTEENTH;
    //     // brb_base->buzzer.tones[11].note = NOTE_As3;
    //     // brb_base->buzzer.tones[11].duration = TONE_DUR_SIXTEENTH;
    //     // brb_base->buzzer.tones[12].note = NOTE_REST;
    //     // brb_base->buzzer.tones[12].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[13].note = NOTE_Cs4;
    //     // brb_base->buzzer.tones[13].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[14].note = NOTE_REST;
    //     // brb_base->buzzer.tones[14].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[15].note = NOTE_G2;
    //     // brb_base->buzzer.tones[15].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[16].note = NOTE_Gs2;
    //     // brb_base->buzzer.tones[16].duration = TONE_DUR_EIGHTH;
    //     // brb_base->buzzer.tones[17].note = NOTE_A2;
    //     // brb_base->buzzer.tones[17].duration = TONE_DUR_EIGHTH;

    //     // brb_base->buzzer.size = 18;
        
    //     brb_base->buzzer.tones[0].note = NOTE_A4;
    //     brb_base->buzzer.tones[0].duration = TONE_DUR_TICK(4);
    //     brb_base->buzzer.tones[1].note = NOTE_A4; 
    //     brb_base->buzzer.tones[1].duration = TONE_DUR_TICK(4);
    //     brb_base->buzzer.tones[2].note = NOTE_A4; 
    //     brb_base->buzzer.tones[2].duration = TONE_DUR_TICK(4);
    //     brb_base->buzzer.tones[3].note = NOTE_F4; 
    //     brb_base->buzzer.tones[3].duration = TONE_DUR_TICK(5);
    //     brb_base->buzzer.tones[4].note = NOTE_C5; 
    //     brb_base->buzzer.tones[4].duration = TONE_DUR_TICK(16);
    //     brb_base->buzzer.tones[5].note = NOTE_A4; 
    //     brb_base->buzzer.tones[5].duration = TONE_DUR_TICK(4);
    //     brb_base->buzzer.tones[6].note = NOTE_F4; 
    //     brb_base->buzzer.tones[6].duration = TONE_DUR_TICK(5);
    //     brb_base->buzzer.tones[7].note = NOTE_C5; 
    //     brb_base->buzzer.tones[7].duration = TONE_DUR_TICK(16);
    //     brb_base->buzzer.tones[8].note = NOTE_A4; 
    //     brb_base->buzzer.tones[8].duration = TONE_DUR_TICK(2);
    //     brb_base->buzzer.tones[9].note = NOTE_E5; 
    //     brb_base->buzzer.tones[9].duration = TONE_DUR_TICK(4);
    //     brb_base->buzzer.tones[10].note = NOTE_E5; 
    //     brb_base->buzzer.tones[10].duration = TONE_DUR_TICK(4);
    //     brb_base->buzzer.tones[11].note = NOTE_E5; 
    //     brb_base->buzzer.tones[11].duration = TONE_DUR_TICK(4);
    //     brb_base->buzzer.tones[12].note = NOTE_F5; 
    //     brb_base->buzzer.tones[12].duration = TONE_DUR_TICK(5);
    //     brb_base->buzzer.tones[13].note = NOTE_C5; 
    //     brb_base->buzzer.tones[13].duration = TONE_DUR_TICK(16);
    //     brb_base->buzzer.tones[14].note = NOTE_A4; 
    //     brb_base->buzzer.tones[14].duration = TONE_DUR_TICK(4);
    //     brb_base->buzzer.tones[15].note = NOTE_F4; 
    //     brb_base->buzzer.tones[15].duration = TONE_DUR_TICK(5);
    //     brb_base->buzzer.tones[16].note = NOTE_C5;
    //     brb_base->buzzer.tones[16].duration = TONE_DUR_TICK(16);
    //     brb_base->buzzer.tones[17].note = NOTE_A4;
    //     brb_base->buzzer.tones[17].duration = TONE_DUR_TICK(2);
    //     brb_base->buzzer.tones[18].note = NOTE_REST;
    //     brb_base->buzzer.tones[18].duration = TONE_DUR_TICK(4);

    //     brb_base->buzzer.size = 19;

    //     pinMode(brb_base->buzzer.pin, OUTPUT);

    //     brb_base->buzzer.note = brb_base->buzzer.tones[brb_base->buzzer.index].note;
    //     brb_base->buzzer.duration = brb_base->buzzer.tones[brb_base->buzzer.index].duration;

    //     if (brb_base->buzzer.note != NOTE_REST)
    //     {
    //         tone(brb_base->buzzer.pin, brb_base->buzzer.note);
    //     }
    // }

    /* Check pins */
    // BrbBase_PinCheck(brb_base);

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

    brb_base->buzzer.duration = (brb_base->buzzer.duration - brb_base->ms.delay);

    /* Current note has expired */
    // if (brb_base->buzzer.duration <= 0 && brb_base->buzzer.pin > 1)
    // {
    //     noTone(brb_base->buzzer.pin);

    //     // Find the next note to be played
    //     if (brb_base->buzzer.index >= brb_base->buzzer.size)
    //     {
    //         brb_base->buzzer.index = 0;
    //     }
    //     else
    //     {
    //         brb_base->buzzer.index++;
    //     }

    //     brb_base->buzzer.note = brb_base->buzzer.tones[brb_base->buzzer.index].note;
    //     brb_base->buzzer.duration = brb_base->buzzer.tones[brb_base->buzzer.index].duration;

    //     /* Start playing the next note */
    //     if (brb_base->buzzer.note != NOTE_REST)
    //     {
    //         tone(brb_base->buzzer.pin, brb_base->buzzer.note, brb_base->buzzer.duration);
    //     }

    //     brb_base->buzzer.duration = brb_base->buzzer.duration * 1.2;
    // }

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
        if (!pin_data->flags.persist)
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
    BrbBase_EEPROMWrite(brb_base, (uint8_t *)&brb_base->pin_data, sizeof(brb_base->pin_data), 100);

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