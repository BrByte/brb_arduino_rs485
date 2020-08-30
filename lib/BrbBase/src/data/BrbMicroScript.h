/*
 * BrbMicroScript.h
 *
 *  Created on: 2019-02-11
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

#ifndef BRB_MICRO_SCRIPT_H_
#define BRB_MICRO_SCRIPT_H_

#include <Arduino.h>

#ifndef BRB_MICRO_SCRIPT_MAX
#define BRB_MICRO_SCRIPT_MAX 12
#endif

#ifndef BRB_MICRO_SCRIPT_OP_SZ
#define BRB_MICRO_SCRIPT_OP_SZ 64
#endif

/**********************************************************************************************************************/
/* DEFINES */
/**********************************************************/

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
    SCRIPT_OPCODE_JMP_NOT_GREATER,
    SCRIPT_OPCODE_JMP_LESSER,
    SCRIPT_OPCODE_JMP_NOT_LESSER,
    SCRIPT_OPCODE_SET_DIGITAL,

    SCRIPT_OPCODE_LASTITEM
} BrbMicroScriptOPCodeNum;
/**********************************************************************************************************************/
/* STRUCTS */
/**********************************************************/
typedef int BrbMicroScriptCBH(void *, void *);
/**********************************************************/
typedef struct _BrbMicroScriptOP
{
    uint8_t opcode : 4;
    uint8_t param_sz : 4;
} BrbMicroScriptOP;

typedef struct _BrbMicroScriptOPDelay
{
    uint16_t time;
} BrbMicroScriptOPDelay;

typedef struct _BrbMicroScriptOPSetDig
{
    uint8_t pin;

    uint8_t mode : 2; /* 0 INPUT, 1 OUTPUT, 2 INPUT_PULLUP 3 IGNORE */
    uint8_t value : 1;
    uint8_t pad0 : 5;

} BrbMicroScriptOPSetDig;

// typedef struct _BrbMicroScriptOPStepperAtDt
// {
//     uint8_t pin;
//     uint8_t index:3;
//     uint8_t action:1;
//     uint8_t pad0:4;

// } BrbMicroScriptOPStepperAtDt;


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
    uint8_t opcode : 4;
    uint8_t param_sz : 4;
    BrbMicroScriptCBH *cb_func;
} BrbMicroScriptOPRunTime;

typedef struct _BrbMicroCode
{
    uint8_t size;
    uint8_t offt;

    uint8_t max_offt;
    uint8_t jmp_offt;

    /* grow only 4 in 4 */
    BrbMicroScriptOP arr[BRB_MICRO_SCRIPT_OP_SZ];

} BrbMicroCode;

typedef struct _BrbMicroScript
{
    uint8_t script_id;

    long delay_until_ms;
    int cmp1;
    int cmp2;

    BrbMicroCode code;

    struct
    {
        uint16_t persist : 1;
        uint16_t active : 1;
        uint16_t delaying : 1;
        uint16_t finished : 1;
        uint16_t pad : 12;
    } flags;

} BrbMicroScript;

typedef struct _BrbMicroScriptBase
{
    BrbLogBase *log_base;
    struct _BrbBase *brb_base;

    BrbMicroScript arr[BRB_MICRO_SCRIPT_MAX];
    int count;

    struct
    {
        long last;
        long cur;
        int delay;
    } ms;

} BrbMicroScriptBase;
/**********************************************************************************************************************/
/* PUBLIC FUNCTIONS */
/**********************************************************/
void BrbMicroScriptBase_Init(BrbMicroScriptBase *script_base);
int BrbMicroScriptBase_RunAll(BrbMicroScriptBase *script_base);
int BrbMicroScriptBase_RunOnce(BrbMicroScriptBase *script_base, BrbMicroScript *script);

BrbMicroScriptOPDelay *BrbMicroScriptOPAddDelay(BrbMicroScriptBase *script_base, BrbMicroScript *script, uint16_t time);
BrbMicroScriptOPCmp *BrbMicroScriptOPAddCmp(BrbMicroScriptBase *script_base, BrbMicroScript *script, uint8_t pin, uint16_t value);
BrbMicroScriptOPIf *BrbMicroScriptOPAddIf(BrbMicroScriptBase *script_base, BrbMicroScript *script, uint8_t if_op, uint8_t else_offset, uint8_t end_offset);
BrbMicroScriptOPSetDig *BrbMicroScriptOPAddSetDig(BrbMicroScriptBase *script_base, BrbMicroScript *script, uint8_t pin, uint8_t mode, uint8_t value);

BrbMicroScript *BrbMicroScriptGrabByID(BrbMicroScriptBase *script_base, int script_id);
BrbMicroScript *BrbMicroScriptGrabFree(BrbMicroScriptBase *script_base);
BrbMicroScript *BrbMicroScriptSetByID(BrbMicroScriptBase *script_base, BrbMicroScript *script_new);
/**********************************************************************************************************************/
/* SCRIPT FUNCTIONS */
/**********************************************************/

BrbMicroScriptCBH BrbMicroScriptCmpFunc;
BrbMicroScriptCBH BrbMicroScriptJmpEqualFunc;
BrbMicroScriptCBH BrbMicroScriptJmpNotEqualFunc;
BrbMicroScriptCBH BrbMicroScriptJmpGreaterFunc;
BrbMicroScriptCBH BrbMicroScriptJmpNotGreaterFunc;
BrbMicroScriptCBH BrbMicroScriptJmpLesserFunc;
BrbMicroScriptCBH BrbMicroScriptJmpNotLesserFunc;
BrbMicroScriptCBH BrbMicroScriptSetDigitalFunc;

static const BrbMicroScriptOPRunTime glob_script_runtime_arr[] =
    {
        {SCRIPT_OPCODE_NOT_INIT, 0, NULL},
        {SCRIPT_OPCODE_DELAY, 4, NULL},

        {SCRIPT_OPCODE_CMP, 1, BrbMicroScriptCmpFunc},

        {SCRIPT_OPCODE_JMP_EQUAL, 1, BrbMicroScriptJmpEqualFunc},
        {SCRIPT_OPCODE_JMP_NOT_EQUAL, 1, BrbMicroScriptJmpNotEqualFunc},
        {SCRIPT_OPCODE_JMP_GREATER, 1, BrbMicroScriptJmpGreaterFunc},
        {SCRIPT_OPCODE_JMP_NOT_GREATER, 1, BrbMicroScriptJmpNotGreaterFunc},
        {SCRIPT_OPCODE_JMP_LESSER, 1, BrbMicroScriptJmpLesserFunc},
        {SCRIPT_OPCODE_JMP_NOT_LESSER, 1, BrbMicroScriptJmpNotLesserFunc},

        {SCRIPT_OPCODE_SET_DIGITAL, 1, BrbMicroScriptSetDigitalFunc},

        {SCRIPT_OPCODE_LASTITEM, 0, NULL} /* NULL TERMINATOR */
};
/**********************************************************************************************************************/
#endif /* BRB_MICRO_SCRIPT_H_ */