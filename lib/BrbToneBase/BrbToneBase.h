/*
 * BrbToneBase.h
 *
 *  Created on: 2018-02-01
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

#ifndef BRB_TONE_H_
#define BRB_TONE_H_

#include "BrbBase.h"

/**********************************************************************************************************************/
// Integer note frequencies for tone()
// Lowest frequency on piano: 27.5 Hz
// Highest frequency on piano: 4186.01 Hz
// Middle C is C4
// http://www.liutaiomottola.com/formulae/freqtab.htm
 
// The Arduino tone() function may not play
// frequencies below NOTE_B2 (123 Hz)
 
#define NOTE_C0   16
#define NOTE_Cs0  17
#define NOTE_D0   18
#define NOTE_Ds0  19
#define NOTE_E0   21
#define NOTE_F0   22
#define NOTE_Fs0  23
#define NOTE_G0   25
#define NOTE_Gs0  26
#define NOTE_A0   28
#define NOTE_As0  29
#define NOTE_B0   31
 
#define NOTE_C1   33
#define NOTE_Cs1  35
#define NOTE_D1   37
#define NOTE_Ds1  39
#define NOTE_E1   41
#define NOTE_F1   44
#define NOTE_Fs1  46
#define NOTE_G1   49
#define NOTE_Gs1  52
#define NOTE_A1   55
#define NOTE_As1  58
#define NOTE_B1   62
 
#define NOTE_C2   65
#define NOTE_Cs2  70
#define NOTE_D2   73
#define NOTE_Ds2  78
#define NOTE_E2   82
#define NOTE_F2   87
#define NOTE_Fs2  93
#define NOTE_G2   98
#define NOTE_Gs2  104
#define NOTE_A2   110
#define NOTE_As2  117
#define NOTE_B2   123
 
#define NOTE_C3   131
#define NOTE_Cs3  139
#define NOTE_D3   147
#define NOTE_Ds3  156
#define NOTE_E3   165
#define NOTE_F3   175
#define NOTE_Fs3  185
#define NOTE_G3   196
#define NOTE_Gs3  208
#define NOTE_A3   220
#define NOTE_As3  233
#define NOTE_B3   247
 
#define NOTE_C4   261
#define NOTE_Cs4  277
#define NOTE_D4   294
#define NOTE_Ds4  311
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_Fs4  370
#define NOTE_G4   392
#define NOTE_Gs4  415
#define NOTE_A4   440
#define NOTE_As4  466
#define NOTE_B4   494
 
#define NOTE_C5   523
#define NOTE_Cs5  554
#define NOTE_D5   587
#define NOTE_Ds5  622
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_Fs5  740
#define NOTE_G5   784
#define NOTE_Gs5  831
#define NOTE_A5   880
#define NOTE_As5  932
#define NOTE_B5   988
 
#define NOTE_C6   1047
#define NOTE_Cs6  1109
#define NOTE_D6   1175
#define NOTE_Ds6  1245
#define NOTE_E6   1319
#define NOTE_F6   1397
#define NOTE_Fs6  1480
#define NOTE_G6   1568
#define NOTE_Gs6  1661
#define NOTE_A6   1760
#define NOTE_As6  1865
#define NOTE_B6   1976
 
#define NOTE_C7   2093
#define NOTE_Cs7  2217
#define NOTE_D7   2349
#define NOTE_Ds7  2489
#define NOTE_E7   2637
#define NOTE_F7   2794
#define NOTE_Fs7  2960
#define NOTE_G7   3136
#define NOTE_Gs7  3322
#define NOTE_A7   3520
#define NOTE_As7  3729
#define NOTE_B7   3951
 
#define NOTE_C8   4186
#define NOTE_Cs8  4435
#define NOTE_D8   4699
#define NOTE_Ds8  4978
#define NOTE_E8   5274
#define NOTE_F8   5588
#define NOTE_Fs8  5920
#define NOTE_G8   6272
#define NOTE_Gs8  6645
#define NOTE_A8   7040
#define NOTE_As8  7459
#define NOTE_B8   7902
 
#define NOTE_C9   8372
 
#define NOTESIZE ((9*12)+1)

/* The program must define TEMPO before including this file. TEMPO is specified in beats per minute (BPM) */
#define TONE_TEMPO      (120)
#define NOTE_REST       0
#define NOTE_FINISH    -1
/**********************************************************************************************************************/
// Convert TEMPO (BPM) to one millisecond ticks.
#define TONE_TICKS   ((60.0 * 4.0 / TONE_TEMPO) * 1000)
// #define TONE_TICKS   (1000 * (60 * 4 / TONE_TEMPO))
 
#define TONE_DUR_SIXTEENTH         (int)(TONE_TICKS / 4)
#define TONE_DUR_EIGHTH            (int)(TONE_TICKS / 2)

#define TONE_DUR_QUARTER           (int)(TONE_TICKS)
#define TONE_DUR_DOTTED_QUARTER    (int)(TONE_TICKS + (TONE_TICKS / 2))
#define TONE_DUR_HALF              (int)(TONE_TICKS * 2)
#define TONE_DUR_DOTTED_HALF       (int)(TONE_TICKS * 3)
#define TONE_DUR_WHOLE             (int)(TONE_TICKS * 4)

#define TONE_DUR_TICK(t)           (int)(TONE_TICKS / t + 0.1)
/**********************************************************************************************************************/
typedef struct _BrbToneNote
{
    int note;
    int duration;

} BrbToneNote;

typedef struct _BrbToneBase
{
    BrbBase *brb_base;
    BrbToneNote notes[32];

    int index;
    int size;
    int duration;
    int note;

    int loop_max;
    int loop_cnt;

    int pin;

    struct {
        unsigned int enabled:1;
    } flags;

} BrbToneBase;
/**********************************************************************************************************************/
int BrbToneBase_Init(BrbToneBase *tone_base);
void BrbToneBase_Loop(BrbToneBase *tone_base);

int BrbToneBase_PlayAction(BrbToneBase *tone_base);

int BrbToneBase_PlayArrive(BrbToneBase *tone_base);
int BrbToneBase_PlayLeave(BrbToneBase *tone_base);

int BrbToneBase_PlayStart(BrbToneBase *tone_base);

int BrbToneBase_PlayAlarm(BrbToneBase *tone_base);
int BrbToneBase_PlayAlarm2(BrbToneBase *tone_base);
int BrbToneBase_PlayAlarm3(BrbToneBase *tone_base);
/**********************************************************************************************************************/
#endif /* BRB_TONE_H_ */