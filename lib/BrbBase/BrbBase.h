/*
 * BrbBase.h
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

#ifndef BRB_BASE_H_
#define BRB_BASE_H_

/**********************************************************************************************************************/
#include "Arduino.h"
#include "Boards.h"
#include "BrbLogBase.h"

static const uint8_t glob_analog_pins[] = {
    A0,A1,A2,A3,A4,A5,A6,A7,
#ifdef A8
    A9,
#endif
#ifdef A10
    A10,
#endif
#ifdef A11
    A12,
#endif
#ifdef A12
    A12,
#endif
#ifdef A13
    A13,
#endif
#ifdef A14
    A14,
#endif
#ifdef A15
    A15,
#endif
#ifdef A16
    A16,
#endif
#ifdef A17
    A17,
#endif
#ifdef A18
    A18,
#endif
#ifdef A19
    A19,
#endif
#ifdef A21
    A21,
#endif
#ifdef A22
    A22,
#endif
#ifdef A23
    A23,
#endif
#ifdef A24
    A24,
#endif
};

/**********************************************************************************************************************/
/* DEFINES */
/**********************************************************/
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//Code in here will only be compiled if an Arduino Mega is used.
#endif
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Code in here will only be compiled if an Arduino Uno (or older) is used.
#endif
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
//Code in here will only be compiled if an Arduino Leonardo is used.
#endif

#define MAX_TIMER 16
#define MAX_SCRIPT 8

#define MIN_ANA_PIN 0
#define MAX_ANA_PIN NUM_ANALOG_INPUTS

#define MIN_DIG_PIN 5

// #define MAX_DIG_PIN NUM_DIGITAL_PINS
#define MAX_DIG_PIN PIN_A0

#define BRB_COMPARE_NUM(a, b) (a > b) - (a < b)
/**********************************************************************************************************************/
/* ENUMS */
/**********************************************************/
typedef enum
{
    SCRIPT_OPCODE_NOT_INIT,
    SCRIPT_OPCODE_DELAY,
    SCRIPT_OPCODE_CMP,
    SCRIPT_OPCODE_JMP_EQUAL,
    SCRIPT_OPCODE_JMP_NOT_EQUAL,
    SCRIPT_OPCODE_JMP_GREATER,
    SCRIPT_OPCODE_JMP_GREATER_OR_EQUAL,
    SCRIPT_OPCODE_JMP_LESSER,
    SCRIPT_OPCODE_JMP_LESSER_OR_EQUAL,
    SCRIPT_OPCODE_SET_DIGITAL,
    SCRIPT_OPCODE_LASTITEM
} BrbMicroScriptOPCodeNum;
/**********************************************************************************************************************/
/* STRUCTS */
/**********************************************************/
typedef int BrbGenericCBH(void *, void *);
/**********************************************************/
typedef struct _BrbTimer
{
    int timer_id;
    BrbGenericCBH *cb_func;
    void *cb_data;

    struct
    {
      long when;
      int delay;
    } ms;

    struct
    {
        unsigned int persist:1;
        unsigned int active:1;
    } flags;

} BrbTimer;
/**********************************************************/
typedef struct _BrbMicroScriptOP
{
    uint8_t opcode:4;
    uint8_t param_sz:4;
} BrbMicroScriptOP;

typedef struct _BrbMicroScriptOPDelay
{
    uint16_t time;
} BrbMicroScriptOPDelay;

typedef struct _BrbMicroScriptOPSetDig
{
    uint8_t pin;
    
    uint8_t mode:2; 	/* 0 INPUT, 1 OUTPUT, 2 INPUT_PULLUP 3 IGNORE */
	uint8_t value:1;
    uint8_t pad0:5;
    
} BrbMicroScriptOPSetDig;

typedef struct _BrbMicroScriptOPCmp
{
    uint8_t pin;
    uint8_t reserved;
	uint16_t value;
    
} BrbMicroScriptOPCmp;

typedef struct _BrbMicroScriptOPIf
{
    uint8_t else_offset;
	uint8_t end_offset;
    
} BrbMicroScriptOPIf;

typedef struct _BrbMicroScriptOPRunTime
{
    uint8_t opcode:4;
    uint8_t param_sz:4;
    BrbGenericCBH *cb_func;
} BrbMicroScriptOPRunTime;

typedef struct _BrbMicroCode
{
    uint8_t size;
    uint8_t offt;

    uint8_t max_offt;
    uint8_t jmp_offt;

    /* grow only 4 in 4 */
    BrbMicroScriptOP arr[54];

} BrbMicroCode;

typedef struct _BrbMicroScript
{
    uint8_t script_id;

    long delay_until_ms;
    uint16_t cmp1;
    uint16_t cmp2;

    BrbMicroCode code;

    struct
    {
    	uint16_t persist:1;
    	uint16_t active:1;
    	uint16_t delaying:1;
    	uint16_t finished:1;
    	uint16_t pad:12;
    } flags;

} BrbMicroScript;
/**********************************************************/
typedef struct _BrbBasePinData
{
    uint8_t value;
	uint8_t mode;

} BrbBasePinData;
/**********************************************************/
typedef struct _BrbBase
{
	BrbLogBase *log_base;
    BrbBasePinData pin_data[TOTAL_PINS];

    struct
    {
        long last;
        long cur;
        int delay;
    } ms;

    struct
    {
        long last;
        long cur;
        int delay;
    } us;

    struct
    {
        int count;
        BrbTimer arr[MAX_TIMER];
    } timer;

    struct
    {
        int count;
        BrbMicroScript arr[MAX_SCRIPT];
    } script; 
} BrbBase;
/**********************************************************************************************************************/
/* PUBLIC FUNCTIONS */
/**********************************************************/
void BrbBaseInit(BrbBase *brb_base);
void BrbBaseLoop(BrbBase *brb_base);

int BrbMicroScriptRunAll(BrbBase *brb_base);
int BrbMicroScriptRunOnce(BrbBase *brb_base, BrbMicroScript *script);

BrbMicroScriptOPDelay *BrbMicroScriptOPAddDelay(BrbBase *brb_base, BrbMicroScript *script, uint16_t time);
BrbMicroScriptOPCmp *BrbMicroScriptOPAddCmp(BrbBase *brb_base, BrbMicroScript *script, uint8_t pin, uint16_t value);
BrbMicroScriptOPIf *BrbMicroScriptOPAddIf(BrbBase *brb_base, BrbMicroScript *script, uint8_t if_op, uint8_t else_offset, uint8_t end_offset);
int BrbMicroScriptOPAddSetDig(BrbBase *brb_base, BrbMicroScript *script, uint8_t pin, uint8_t mode, uint8_t value);

BrbMicroScript *BrbMicroScriptGrabByID(BrbBase *brb_base, int script_id);
BrbMicroScript *BrbMicroScriptGrabFree(BrbBase *brb_base);
BrbMicroScript *BrbMicroScriptSetByID(BrbBase *brb_base, BrbMicroScript *script_new);


BrbTimer *BrbBaseTimerGrabByID(BrbBase *brb_base, int timer_id);
int BrbBaseTimerAdd(BrbBase *brb_base, long delay_ms, int persist, BrbGenericCBH *cb_func, void *cb_data);
void BrbBaseTimerDispatch(BrbBase *brb_base);

void BrbBase_PinLoad(BrbBase *brb_base);
void BrbBase_PinCheck(BrbBase *brb_base);
void BrbBase_PinSet(BrbBase *brb_base, int pin_num, int pin_mode, int pin_value);
void BrbBase_PinSave(BrbBase *brb_base);
uint8_t BrbBase_PinGetMode(uint8_t pin);

int BrbBase_EEPROMRead(BrbBase *brb_base, uint8_t *data_ptr, uint8_t data_sz, uint8_t eeprom_offset);
int BrbBase_EEPROMWrite(BrbBase *brb_base, uint8_t *data_ptr, uint8_t data_sz, uint8_t eeprom_offset);
/**********************************************************************************************************************/
/* SCRIPT FUNCTIONS */
/**********************************************************/

BrbGenericCBH BrbMicroScriptCmpFunc;
BrbGenericCBH BrbMicroScriptJmpEqualFunc;
BrbGenericCBH BrbMicroScriptJmpNotEqualFunc;
BrbGenericCBH BrbMicroScriptJmpGreaterFunc;
BrbGenericCBH BrbMicroScriptJmpNotGreaterFunc;
BrbGenericCBH BrbMicroScriptJmpLesserFunc;
BrbGenericCBH BrbMicroScriptJmpNotLesserFunc;
BrbGenericCBH BrbMicroScriptSetDigitalFunc;

static const BrbMicroScriptOPRunTime glob_script_runtime_arr[] = 
{
    {SCRIPT_OPCODE_NOT_INIT,                0, NULL}, 
    {SCRIPT_OPCODE_DELAY,                   4, NULL},

    {SCRIPT_OPCODE_CMP,                     1, BrbMicroScriptCmpFunc},

    {SCRIPT_OPCODE_JMP_EQUAL,               1, BrbMicroScriptJmpEqualFunc},
    {SCRIPT_OPCODE_JMP_NOT_EQUAL,           1, BrbMicroScriptJmpNotEqualFunc},
    {SCRIPT_OPCODE_JMP_GREATER,             1, BrbMicroScriptJmpGreaterFunc},
    {SCRIPT_OPCODE_JMP_GREATER_OR_EQUAL,    1, BrbMicroScriptJmpNotGreaterFunc},
    {SCRIPT_OPCODE_JMP_LESSER,              1, BrbMicroScriptJmpLesserFunc},
    {SCRIPT_OPCODE_JMP_LESSER_OR_EQUAL,     1, BrbMicroScriptJmpNotLesserFunc},

    {SCRIPT_OPCODE_SET_DIGITAL,             1, BrbMicroScriptSetDigitalFunc},

    {SCRIPT_OPCODE_LASTITEM,    0, NULL}            /* NULL TERMINATOR */
};
/**********************************************************************************************************************/

#endif /* BRB_BASE_H_ */