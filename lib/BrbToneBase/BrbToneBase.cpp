/*
 * BrbToneBase.cpp
 *
 *  Created on: 2018-02-01
 *      Author: Luiz Fernando Souza Softov <softov@brbyte.com>
 *      Author: Guilherme Amorim de Oliveira Alves <guilherme@brbyte.com>
 *
 * Copyright (c) 2019 BrByte Software (Oliveira Alves & Amorim LTDA)
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

#include "BrbToneBase.h"

int BrbToneBase_PlayStarWars(BrbToneBase *tone_base);

/**********************************************************************************************************************/
int BrbToneBase_Init(BrbToneBase *tone_base)
{
    /* Sanitize */
    if (!tone_base)
        return -1;

    /* Current note has expired */
    if (tone_base->duration > 0 || tone_base->pin < 2 || tone_base->pin > NUM_DIGITAL_PINS)
        return -1;

    /* Enable pin */
    pinMode(tone_base->pin, OUTPUT);

    BrbToneBase_PlayStart(tone_base);

    /* Do the first loop, if it has items */
    // BrbToneBase_Loop(tone_base);

    return 0;
}
/**********************************************************************************************************************/
void BrbToneBase_Loop(BrbToneBase *tone_base)
{
    /* Tone is disabled */
    if (!tone_base->flags.enabled || tone_base->pin < 2 || tone_base->pin > NUM_DIGITAL_PINS)
        return;

    if (tone_base->index >= 0)
    {
	    tone_base->ms_delta = (millis() - tone_base->ms_last);
		tone_base->ms_last = millis();

        /* Update last duration */
        tone_base->duration = (tone_base->duration - tone_base->ms_delta);
        // tone_base->duration = (tone_base->duration - tone_base->brb_base->ms.delay);

        /* Current note has expired */
        if (tone_base->duration > 0)
            return;

        noTone(tone_base->pin);
    }

    /* Find the next index for note to be played */
    if (tone_base->index >= tone_base->size)
    {
        tone_base->index = 0;

        /* We have finished, increase loop count */
        tone_base->loop_cnt++;

        if ((tone_base->loop_max > 0 && tone_base->loop_cnt >= tone_base->loop_max) || tone_base->note == NOTE_FINISH)
        {
            tone_base->flags.enabled = 0;
            tone_base->loop_cnt = 0;

            return;
        }
    }
    else
    {
        tone_base->index++;
    }

    tone_base->note = tone_base->notes[tone_base->index].note;
    tone_base->duration = tone_base->notes[tone_base->index].duration;

    /* Start playing the next note */
    if (tone_base->note != NOTE_REST)
    {
        tone(tone_base->pin, tone_base->note, tone_base->duration);
    }

    tone_base->duration = tone_base->duration * 1.2;

    return;
}
/**********************************************************************************************************************/
int BrbToneBase_PlayStart(BrbToneBase *tone_base)
{
    /* Sanitize */
    if (!tone_base)
        return -1;

    tone_base->notes[0].note = NOTE_C4;
    tone_base->notes[0].duration = TONE_DUR_TICK(8);
    tone_base->notes[1].note = NOTE_REST;
    tone_base->notes[1].duration = TONE_DUR_TICK(6);
    tone_base->notes[2].note = NOTE_D4;
    tone_base->notes[2].duration = TONE_DUR_TICK(8);
    tone_base->notes[3].note = NOTE_REST;
    tone_base->notes[3].duration = TONE_DUR_TICK(6);
    tone_base->notes[4].note = NOTE_C4;
    tone_base->notes[4].duration = TONE_DUR_TICK(8);
    tone_base->notes[5].note = NOTE_REST;
    tone_base->notes[5].duration = TONE_DUR_TICK(2);

    tone_base->size = 6;
    tone_base->loop_max = 1;
    tone_base->index = -1;
    tone_base->flags.enabled = 1;

    return 0;
}
/**********************************************************************************************************************/
int BrbToneBase_PlayAction(BrbToneBase *tone_base)
{
    /* Sanitize */
    if (!tone_base)
        return -1;

    tone_base->notes[0].note = NOTE_F5;
    tone_base->notes[0].duration = TONE_DUR_TICK(6);
    tone_base->notes[1].note = NOTE_C4;
    tone_base->notes[1].duration = TONE_DUR_TICK(8);
    tone_base->notes[2].note = NOTE_F5;
    tone_base->notes[2].duration = TONE_DUR_TICK(6);

    tone_base->size = 2;
    tone_base->loop_max = 1;
    tone_base->index = -1;
    tone_base->flags.enabled = 1;

    return 0;
}
/**********************************************************************************************************************/
int BrbToneBase_PlayArrive(BrbToneBase *tone_base)
{
    /* Sanitize */
    if (!tone_base)
        return -1;

    tone_base->notes[0].note = NOTE_Cs6;
    tone_base->notes[0].duration = TONE_DUR_TICK(6);
    tone_base->notes[1].note = NOTE_REST;
    tone_base->notes[1].duration = TONE_DUR_TICK(6);
    tone_base->notes[2].note = NOTE_Ds6;
    tone_base->notes[2].duration = TONE_DUR_TICK(6);
    tone_base->notes[3].note = NOTE_REST;
    tone_base->notes[3].duration = TONE_DUR_TICK(6);
    tone_base->notes[4].note = NOTE_Cs6;
    tone_base->notes[4].duration = TONE_DUR_TICK(6);

    tone_base->size = 4;
    tone_base->loop_max = 1;
    tone_base->index = -1;
    tone_base->flags.enabled = 1;

    return 0;
}
/**********************************************************************************************************************/
int BrbToneBase_PlayLeave(BrbToneBase *tone_base)
{
    /* Sanitize */
    if (!tone_base)
        return -1;

    tone_base->notes[0].note = NOTE_A5;
    tone_base->notes[0].duration = TONE_DUR_TICK(6);
    tone_base->notes[1].note = NOTE_REST;
    tone_base->notes[1].duration = TONE_DUR_TICK(6);
    tone_base->notes[2].note = NOTE_D7;
    tone_base->notes[2].duration = TONE_DUR_TICK(6);
    tone_base->notes[3].note = NOTE_REST;
    tone_base->notes[3].duration = TONE_DUR_TICK(6);
    tone_base->notes[4].note = NOTE_A5;
    tone_base->notes[4].duration = TONE_DUR_TICK(6);

    tone_base->size = 4;
    tone_base->loop_max = 1;
    tone_base->index = -1;
    tone_base->flags.enabled = 1;

    return 0;
}
/**********************************************************************************************************************/
int BrbToneBase_PlayAlarm(BrbToneBase *tone_base)
{
    /* Sanitize */
    if (!tone_base)
        return -1;

    tone_base->notes[0].note = NOTE_A4;
    tone_base->notes[0].duration = TONE_DUR_TICK(4);
    tone_base->notes[1].note = NOTE_A4;
    tone_base->notes[1].duration = TONE_DUR_TICK(4);
    tone_base->notes[2].note = NOTE_A4;
    tone_base->notes[2].duration = TONE_DUR_TICK(4);
    tone_base->notes[3].note = NOTE_F4;
    tone_base->notes[3].duration = TONE_DUR_TICK(5);
    tone_base->notes[4].note = NOTE_C5;
    tone_base->notes[4].duration = TONE_DUR_TICK(16);
    tone_base->notes[5].note = NOTE_A4;
    tone_base->notes[5].duration = TONE_DUR_TICK(4);

    tone_base->size = 6;
    tone_base->loop_max = 1;
    tone_base->index = -1;
    tone_base->flags.enabled = 1;

    return 0;
}
/**********************************************************************************************************************/
int BrbToneBase_PlayAlarm2(BrbToneBase *tone_base)
{
    /* Sanitize */
    if (!tone_base)
        return -1;

    tone_base->notes[0].note = NOTE_Fs7;
    tone_base->notes[0].duration = TONE_DUR_TICK(8);
    tone_base->notes[1].note = NOTE_REST;
    tone_base->notes[1].duration = TONE_DUR_TICK(6);
    tone_base->notes[2].note = NOTE_Fs7;
    tone_base->notes[2].duration = TONE_DUR_TICK(4);
    tone_base->notes[3].note = NOTE_REST;
    tone_base->notes[3].duration = TONE_DUR_TICK(6);
    tone_base->notes[4].note = NOTE_Fs7;
    tone_base->notes[4].duration = TONE_DUR_TICK(4);
    tone_base->notes[5].note = NOTE_REST;
    tone_base->notes[5].duration = TONE_DUR_TICK(2);

    tone_base->size = 6;
    tone_base->loop_max = 1;
    tone_base->index = -1;
    tone_base->flags.enabled = 1;

    return 0;
}
/**********************************************************************************************************************/
int BrbToneBase_PlayAlarm3(BrbToneBase *tone_base)
{
    /* Sanitize */
    if (!tone_base)
        return -1;

    tone_base->notes[0].note = NOTE_E6;
    tone_base->notes[0].duration = TONE_DUR_TICK(6);
    tone_base->notes[1].note = NOTE_REST;
    tone_base->notes[1].duration = TONE_DUR_TICK(8);
    tone_base->notes[2].note = NOTE_E6;
    tone_base->notes[2].duration = TONE_DUR_TICK(6);
    tone_base->notes[3].note = NOTE_REST;
    tone_base->notes[3].duration = TONE_DUR_TICK(4);

    tone_base->size = 4;
    tone_base->loop_max = 1;
    tone_base->index = -1;
    tone_base->flags.enabled = 1;

    return 0;
}
/**********************************************************************************************************************/
int BrbToneBase_PlayStarWars(BrbToneBase *tone_base)
{
    /* Sanitize */
    if (!tone_base)
        return -1;

    tone_base->notes[0].note = NOTE_A4;
    tone_base->notes[0].duration = TONE_DUR_TICK(4);
    tone_base->notes[1].note = NOTE_A4;
    tone_base->notes[1].duration = TONE_DUR_TICK(4);
    tone_base->notes[2].note = NOTE_A4;
    tone_base->notes[2].duration = TONE_DUR_TICK(4);
    tone_base->notes[3].note = NOTE_F4;
    tone_base->notes[3].duration = TONE_DUR_TICK(5);
    tone_base->notes[4].note = NOTE_C5;
    tone_base->notes[4].duration = TONE_DUR_TICK(16);
    tone_base->notes[5].note = NOTE_A4;
    tone_base->notes[5].duration = TONE_DUR_TICK(4);
    tone_base->notes[6].note = NOTE_F4;
    tone_base->notes[6].duration = TONE_DUR_TICK(5);
    tone_base->notes[7].note = NOTE_C5;
    tone_base->notes[7].duration = TONE_DUR_TICK(16);
    tone_base->notes[8].note = NOTE_A4;
    tone_base->notes[8].duration = TONE_DUR_TICK(2);
    tone_base->notes[9].note = NOTE_E5;
    tone_base->notes[9].duration = TONE_DUR_TICK(4);
    tone_base->notes[10].note = NOTE_E5;
    tone_base->notes[10].duration = TONE_DUR_TICK(4);
    tone_base->notes[11].note = NOTE_E5;
    tone_base->notes[11].duration = TONE_DUR_TICK(4);
    tone_base->notes[12].note = NOTE_F5;
    tone_base->notes[12].duration = TONE_DUR_TICK(5);
    tone_base->notes[13].note = NOTE_C5;
    tone_base->notes[13].duration = TONE_DUR_TICK(16);
    tone_base->notes[14].note = NOTE_A4;
    tone_base->notes[14].duration = TONE_DUR_TICK(4);
    tone_base->notes[15].note = NOTE_F4;
    tone_base->notes[15].duration = TONE_DUR_TICK(5);
    tone_base->notes[16].note = NOTE_C5;
    tone_base->notes[16].duration = TONE_DUR_TICK(16);
    tone_base->notes[17].note = NOTE_A4;
    tone_base->notes[17].duration = TONE_DUR_TICK(2);
    tone_base->notes[18].note = NOTE_REST;
    tone_base->notes[18].duration = TONE_DUR_TICK(4);

    tone_base->size = 19;
    tone_base->loop_max = 1;
    tone_base->index = -1;
    tone_base->flags.enabled = 1;

    return 0;
}
/**********************************************************************************************************************/