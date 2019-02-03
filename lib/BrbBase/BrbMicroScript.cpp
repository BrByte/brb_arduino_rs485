/*
 * BrbMicroScript.cpp
 *
 *  Created on: 2019-01-04
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

/**********************************************************************************************************************/
void BrbMicroScriptInit(BrbBase *brb_base)
{
    LOG_WARN(brb_base->log_base, "Initialize BrbMicroScript\r\n");
    LOG_WARN(brb_base->log_base, "Sizeof BrbMicroScript [%d]\r\n", sizeof(BrbMicroScript));
    LOG_WARN(brb_base->log_base, "Sizeof BrbMicroScriptOP [%d]\r\n", sizeof(BrbMicroScriptOP));
    LOG_WARN(brb_base->log_base, "Sizeof BrbMicroScriptOPDelay [%d]\r\n", sizeof(BrbMicroScriptOPDelay));
    LOG_WARN(brb_base->log_base, "Sizeof BrbMicroScriptOPSetDig [%d]\r\n", sizeof(BrbMicroScriptOPSetDig));
    LOG_WARN(brb_base->log_base, "Sizeof BrbMicroScriptOPCmp [%d]\r\n", sizeof(BrbMicroScriptOPCmp));
    LOG_WARN(brb_base->log_base, "Sizeof BrbMicroScriptOPIf [%d]\r\n", sizeof(BrbMicroScriptOPIf));
    // LOG_WARN(brb_base->log_base, "BrbMicroScriptGrabByID [%p]\r\n", BrbMicroScriptGrabByID(brb_base, 1));
    
    // BrbMicroScript *script;
    // BrbMicroScriptOPIf *op_if;
    // BrbMicroScriptOPIf *op_if_gerando;

    // script      = BrbMicroScriptGrabFree(brb_base);    
    // script->flags.persist   = 1;
    // script->flags.active    = 1;

    // BrbMicroScriptOPAddSetDig(brb_base, script, 5, OUTPUT, LOW);
    // BrbMicroScriptOPAddDelay(brb_base, script, 2000);
    // BrbMicroScriptOPAddSetDig(brb_base, script, 5, OUTPUT, HIGH);
    // BrbMicroScriptOPAddDelay(brb_base, script, 2000);
    // BrbMicroScriptOPAddServoPos(brb_base, script, 32, 180);

    // // BrbMicroScriptOPAddDelay(brb_base, script, 2000);
    // // BrbMicroScriptOPAddServoPos(brb_base, script, 32, 115);

    // // BrbMicroScriptOPAddSetDig(brb_base, script, 2, OUTPUT, LOW);
    // // BrbMicroScriptOPAddSetDig(brb_base, script, 3, OUTPUT, LOW);
    // // BrbMicroScriptOPAddDelay(brb_base, script, 1000);
    // // BrbMicroScriptOPAddSetDig(brb_base, script, 2, OUTPUT, HIGH);
    // // BrbMicroScriptOPAddSetDig(brb_base, script, 3, OUTPUT, HIGH);
    // // BrbMicroScriptOPAddDelay(brb_base, script, 1000);

    // // BrbMicroScriptOPAddDelay(brb_base, script, 500);
    
    // // op_if   = BrbMicroScriptOPAddIf(brb_base, script, SCRIPT_OPCODE_JMP_LESSER, 0, 0);

    // // if (op_if)
    // // {
    // //     BrbMicroScriptOPAddSetDig(brb_base, script, 2, OUTPUT, LOW);

    // //     op_if->else_offset      = script->code.size;
        
    // //     BrbMicroScriptOPAddSetDig(brb_base, script, 2, OUTPUT, HIGH);

    // //     op_if->end_offset       = script->code.size;
    // // }
    
    // int ger_partida_pin = 2;
    // int ger_break_pin = 3;

    // int ger_carga_pin = 8;
    // int ene_carga_pin = 9;

    // BrbMicroScriptOPAddCmp(brb_base, script, 0, 0);
    // BrbMicroScriptOPAddServoPos(brb_base, script, 32, 180);
    // // BrbMicroScriptOPAddSetDig(brb_base, script, ger_partida_pin, OUTPUT, HIGH);
    // // BrbMicroScriptOPAddSetDig(brb_base, script, ger_break_pin, OUTPUT, HIGH);    
    // // BrbMicroScriptOPAddSetDig(brb_base, script, ger_carga_pin, OUTPUT, HIGH);
    // // BrbMicroScriptOPAddSetDig(brb_base, script, ene_carga_pin, OUTPUT, HIGH);

    // op_if   = BrbMicroScriptOPAddIf(brb_base, script, SCRIPT_OPCODE_JMP_EQUAL, 0, 0);

    // if (op_if)
    // {
    //     BrbMicroScriptOPAddServoPos(brb_base, script, 32, 115);
    //     BrbMicroScriptOPAddSetDig(brb_base, script, ger_partida_pin, OUTPUT, LOW);
    //     BrbMicroScriptOPAddDelay(brb_base, script, 5000);
    //     BrbMicroScriptOPAddSetDig(brb_base, script, ger_partida_pin, OUTPUT, HIGH);
    //     BrbMicroScriptOPAddServoPos(brb_base, script, 32, 180);
        
    //     /* CHECK HERE */
    //     BrbMicroScriptOPAddSetDig(brb_base, script, ger_carga_pin, OUTPUT, HIGH);

    //     op_if->else_offset      = script->code.size;

    //     BrbMicroScriptOPAddSetDig(brb_base, script, ger_partida_pin, OUTPUT, HIGH);
    //     BrbMicroScriptOPAddSetDig(brb_base, script, ger_carga_pin, OUTPUT, LOW);

    //     op_if->end_offset       = script->code.size;
    // }
    
    // BrbMicroScriptOPAddCmp(brb_base, script, 1, 0);

    // op_if   = BrbMicroScriptOPAddIf(brb_base, script, SCRIPT_OPCODE_JMP_EQUAL, 0, 0);

    // if (op_if)
    // {
    //     BrbMicroScriptOPAddSetDig(brb_base, script, ger_break_pin, OUTPUT, LOW);
    //     BrbMicroScriptOPAddDelay(brb_base, script, 6000);
    //     BrbMicroScriptOPAddSetDig(brb_base, script, ger_break_pin, OUTPUT, HIGH);

    //     /* CHECK HERE */
    //     BrbMicroScriptOPAddSetDig(brb_base, script, ene_carga_pin, OUTPUT, HIGH);

    //     op_if->else_offset      = script->code.size;

    //     BrbMicroScriptOPAddSetDig(brb_base, script, ger_break_pin, OUTPUT, HIGH);
    //     BrbMicroScriptOPAddSetDig(brb_base, script, ene_carga_pin, OUTPUT, LOW);

    //     op_if->end_offset       = script->code.size;
    // }

    // BrbMicroScriptOPAddDelay(brb_base, script, 1000);

    return;
}
/**********************************************************************************************************************/
BrbMicroScriptOPDelay *BrbMicroScriptOPAddDelay(BrbBase *brb_base, BrbMicroScript *script, uint16_t time)
{
    BrbMicroScriptOP *op_hdr;
    BrbMicroScriptOPDelay *op_delay;
    
    if (!script || (script->code.size + 1 + sizeof(BrbMicroScriptOPDelay)) > sizeof(script->code.arr))
    {
        LOG_WARN(brb_base->log_base, "No delay avail [%d] [%d]\r\n",  (script->code.size + 1 + sizeof(BrbMicroScriptOPDelay)), sizeof(script->code.arr));

        return NULL;
    }

    op_hdr              = (BrbMicroScriptOP *)&script->code.arr[script->code.size++];
    op_hdr->opcode      = SCRIPT_OPCODE_DELAY;
    op_hdr->param_sz    = sizeof(BrbMicroScriptOPDelay);

    op_delay            = (BrbMicroScriptOPDelay *)&script->code.arr[script->code.size];
    script->code.size   += op_hdr->param_sz;    
    op_delay->time      = time;

    return op_delay;
};
/**********************************************************************************************************************/
BrbMicroScriptOPCmp *BrbMicroScriptOPAddCmp(BrbBase *brb_base, BrbMicroScript *script, uint8_t pin, uint16_t value)
{
    BrbMicroScriptOP *op_hdr;
    BrbMicroScriptOPCmp *op_cmp;
    
    if (!script || (script->code.size + 1 + sizeof(BrbMicroScriptOPCmp)) > sizeof(script->code.arr))
    {
        LOG_WARN(brb_base->log_base, "No CMP sz avail [%d] [%d]\r\n",  (script->code.size + 1 + sizeof(BrbMicroScriptOPCmp)), sizeof(script->code.arr));

        return NULL;
    }

    op_hdr              = (BrbMicroScriptOP *)&script->code.arr[script->code.size++];
    op_hdr->opcode      = SCRIPT_OPCODE_CMP;
    op_hdr->param_sz    = sizeof(BrbMicroScriptOPCmp);

    op_cmp              = (BrbMicroScriptOPCmp *)&script->code.arr[script->code.size];
    script->code.size   += op_hdr->param_sz;

    op_cmp->pin         = pin;
    op_cmp->value       = value;

    return op_cmp;
};
/**********************************************************************************************************************/
BrbMicroScriptOPIf *BrbMicroScriptOPAddIf(BrbBase *brb_base, BrbMicroScript *script, uint8_t if_op, uint8_t else_offset, uint8_t end_offset)
{
    BrbMicroScriptOP *op_hdr;
    BrbMicroScriptOPIf *op_if;
    
    if (!script || (script->code.size + 1 + sizeof(BrbMicroScriptOPIf)) > sizeof(script->code.arr))
    {
        LOG_WARN(brb_base->log_base, "No IF sz avail [%d] [%d]\r\n",  (script->code.size + 1 + sizeof(BrbMicroScriptOPIf)), sizeof(script->code.arr));

        return NULL;
    }

    if (if_op < SCRIPT_OPCODE_JMP_EQUAL || if_op > SCRIPT_OPCODE_JMP_LESSER_OR_EQUAL)
    {
        if_op           = SCRIPT_OPCODE_JMP_EQUAL;
    }

    op_hdr              = (BrbMicroScriptOP *)&script->code.arr[script->code.size++];
    op_hdr->opcode      = if_op;
    op_hdr->param_sz    = sizeof(BrbMicroScriptOPIf);

    op_if               = (BrbMicroScriptOPIf *)&script->code.arr[script->code.size];
    script->code.size   += op_hdr->param_sz;

    op_if->else_offset  = else_offset;
    op_if->end_offset   = end_offset;

    return op_if;
};
/**********************************************************************************************************************/
BrbMicroScriptOPSetDig *BrbMicroScriptOPAddSetDig(BrbBase *brb_base, BrbMicroScript *script, uint8_t pin, uint8_t mode, uint8_t value)
{
    BrbMicroScriptOP *op_hdr;
    BrbMicroScriptOPSetDig *op_set_dig;
    
    if (!script || (script->code.size + 1 + sizeof(BrbMicroScriptOPSetDig)) > sizeof(script->code.arr))
    {
        LOG_WARN(brb_base->log_base, "No SD sz avail [%d] [%d]\r\n",  (script->code.size + 1 + sizeof(BrbMicroScriptOPSetDig)), sizeof(script->code.arr));

        return NULL;
    }

    op_hdr              = (BrbMicroScriptOP *)&script->code.arr[script->code.size++];
    op_hdr->opcode      = SCRIPT_OPCODE_SET_DIGITAL;
    op_hdr->param_sz    = sizeof(BrbMicroScriptOPSetDig);

    op_set_dig          = (BrbMicroScriptOPSetDig *)&script->code.arr[script->code.size];
    script->code.size   += op_hdr->param_sz;

    op_set_dig->pin     = pin;
    op_set_dig->mode    = mode;
    op_set_dig->value   = value;

    return op_set_dig;
};
/**********************************************************************************************************************/
// int BrbMicroScriptOPAddStepperAtDt(BrbBase *brb_base, BrbMicroScript *script, uint8_t pin, uint8_t index, uint8_t atdt)
// {
//     BrbMicroScriptOP *op_hdr;
//     BrbMicroScriptOPStepperAtDt *op_atdt;
    
//     if (!script || (script->code.size + 1 + sizeof(BrbMicroScriptOPStepperAtDt)) > sizeof(script->code.arr))
//         return -1;

//     op_hdr              = (BrbMicroScriptOP *)&script->code.arr[script->code.size++];
//     op_hdr->opcode      = SCRIPT_OPCODE_SERVO_ATDT;
//     op_hdr->param_sz    = sizeof(BrbMicroScriptOPStepperAtDt);

//     op_atdt             = (BrbMicroScriptOPStepperAtDt *)&script->code.arr[script->code.size];
//     script->code.size   += op_hdr->param_sz;

//     op_atdt->pin        = pin;
//     op_atdt->index      = index;
//     op_atdt->action     = atdt;

//     return 0;
// };
/**********************************************************************************************************************/
BrbMicroScriptOPServoPos *BrbMicroScriptOPAddServoPos(BrbBase *brb_base, BrbMicroScript *script, uint8_t pin, uint8_t pos)
{
    BrbMicroScriptOP *op_hdr;
    BrbMicroScriptOPServoPos *op_stepper_pos;
    
    if (!script || (script->code.size + 1 + sizeof(BrbMicroScriptOPServoPos)) > sizeof(script->code.arr))
    {
        LOG_WARN(brb_base->log_base, "No size avail [%d] [%d]\r\n",  (script->code.size + 1 + sizeof(BrbMicroScriptOPServoPos)), sizeof(script->code.arr));

        return NULL;
    }

    op_hdr              = (BrbMicroScriptOP *)&script->code.arr[script->code.size++];
    op_hdr->opcode      = SCRIPT_OPCODE_SERVO_POS;
    op_hdr->param_sz    = sizeof(BrbMicroScriptOPServoPos);

    op_stepper_pos      = (BrbMicroScriptOPServoPos *)&script->code.arr[script->code.size];
    script->code.size   += op_hdr->param_sz;

    op_stepper_pos->pin  = pin;
    op_stepper_pos->pos  = pos;

    return op_stepper_pos;
};
/**********************************************************************************************************************/
int BrbMicroScriptCmpFunc(void *base_ptr, void *cb_data)
{
    // BrbBase *brb_base            = (BrbBase*)base_ptr;
    BrbMicroScript *script       = (BrbMicroScript*)cb_data;
    BrbMicroScriptOPCmp *op_cmp  = (BrbMicroScriptOPCmp *)&script->code.arr[script->code.offt + 1];

    // int op_status;
    // int pin_begin;
    // int pin_max;
    int pin_value               = analogRead(op_cmp->pin);

    script->cmp1                = BRB_COMPARE_NUM(pin_value, op_cmp->value);

    // LOG_DEBUG(brb_base->log_base, "CMP [%d] - [%d] [%d]\r\n", script->cmp1, pin_value, op_cmp->value);

    return 0;
}
/**********************************************************************************************************************/
int BrbMicroScriptSetDigitalFunc(void *base_ptr, void *cb_data)
{
    BrbBase *brb_base                   = (BrbBase*)base_ptr;
    BrbMicroScript *script              = (BrbMicroScript*)cb_data;
    // BrbMicroScriptOP *op_hdr            = (BrbMicroScriptOP *)&script->code.arr[script->code.offt];
    BrbMicroScriptOPSetDig *op_set_dig  = (BrbMicroScriptOPSetDig *)&script->code.arr[script->code.offt + 1];
    
    /* Copy param */
    // memcpy(&param, &script->code.arr[script->code.offt + 1], sizeof(uint8_t));

    BrbBase_PinSet(brb_base, op_set_dig->pin, op_set_dig->mode, op_set_dig->value);

    return 0;
}
/**********************************************************************************************************************/
int BrbMicroScriptServoPosFunc(void *base_ptr, void *cb_data)
{
    BrbBase *brb_base                           = (BrbBase*)base_ptr;
    BrbMicroScript *script                      = (BrbMicroScript*)cb_data;
    // BrbMicroScriptOP *op_hdr                 = (BrbMicroScriptOP *)&script->code.arr[script->code.offt];
    BrbMicroScriptOPServoPos *op_servo_pos  = (BrbMicroScriptOPServoPos *)&script->code.arr[script->code.offt + 1];
    
    /* Copy param */
    // memcpy(&param, &script->code.arr[script->code.offt + 1], sizeof(uint8_t));

    BrbServoSetPosByPin(brb_base, op_servo_pos->pin, op_servo_pos->pos);

    return 0;
}
/**********************************************************************************************************************/
int BrbMicroScriptJmpEqualFunc(void *base_ptr, void *cb_data)
{
    BrbBase *brb_base               = (BrbBase*)base_ptr;
    BrbMicroScript *script          = (BrbMicroScript*)cb_data;
    BrbMicroScriptOPIf *op_if       = (BrbMicroScriptOPIf *)&script->code.arr[script->code.offt + 1];
    
    // LOG_DEBUG(brb_base->log_base, "EQ [%d] - el [%u] en [%u]\r\n", script->cmp1, op_if->else_offset, op_if->end_offset);

    /* check param [1 great] [0 equal] [-1 less] */
    if (script->cmp1 == 0)
    {
        if (op_if->else_offset > 0)
        {
            script->code.max_offt    = op_if->else_offset;
            script->code.jmp_offt    = op_if->end_offset;
        }   
    
        return 0;
    }
        
    if (op_if->else_offset > 0)
        script->code.offt           = op_if->else_offset;
    else
        script->code.offt           = op_if->end_offset;
    
    return 1;
}
/**********************************************************************************************************************/
int BrbMicroScriptJmpNotEqualFunc(void *base_ptr, void *cb_data)
{
    BrbBase *brb_base               = (BrbBase*)base_ptr;
    BrbMicroScript *script          = (BrbMicroScript*)cb_data;
    BrbMicroScriptOPIf *op_if       = (BrbMicroScriptOPIf *)&script->code.arr[script->code.offt + 1];
    
    LOG_DEBUG(brb_base->log_base, "NEQ [%d] - el [%u] en [%u]\r\n", script->cmp1, op_if->else_offset, op_if->end_offset);
    
    /* check param [1 great] [0 equal] [-1 less] */
    if (script->cmp1 != 0)
    {
        if (op_if->else_offset > 0)
        {
            script->code.max_offt       = op_if->else_offset;
            script->code.jmp_offt       = op_if->end_offset;
        }   
    
        return 0;
    }
        
    if (op_if->else_offset > 0)
        script->code.offt           = op_if->else_offset;
    else
        script->code.offt           = op_if->end_offset;
    
    return 1;
}
/**********************************************************************************************************************/
int BrbMicroScriptJmpGreaterFunc(void *base_ptr, void *cb_data)
{
    BrbBase *brb_base               = (BrbBase*)base_ptr;
    BrbMicroScript *script          = (BrbMicroScript*)cb_data;
    BrbMicroScriptOPIf *op_if       = (BrbMicroScriptOPIf *)&script->code.arr[script->code.offt + 1];
    
    LOG_DEBUG(brb_base->log_base, "GE [%d] - el [%u] en [%u]\r\n", script->cmp1, op_if->else_offset, op_if->end_offset);
    
    /* check param [1 great] [0 equal] [-1 less] */
    if (script->cmp1 > 0)
    {
        if (op_if->else_offset > 0)
        {
            script->code.max_offt       = op_if->else_offset;
            script->code.jmp_offt       = op_if->end_offset;
        }   
    
        return 0;
    }
        
    if (op_if->else_offset > 0)
        script->code.offt           = op_if->else_offset;
    else
        script->code.offt           = op_if->end_offset;
    
    return 1;
}
/**********************************************************************************************************************/
int BrbMicroScriptJmpNotGreaterFunc(void *base_ptr, void *cb_data)
{
    BrbBase *brb_base               = (BrbBase*)base_ptr;
    BrbMicroScript *script          = (BrbMicroScript*)cb_data;
    BrbMicroScriptOPIf *op_if       = (BrbMicroScriptOPIf *)&script->code.arr[script->code.offt + 1];
    
    LOG_DEBUG(brb_base->log_base, "NGE [%d] - el [%u] en [%u]\r\n", script->cmp1, op_if->else_offset, op_if->end_offset);
    
    /* check param [1 great] [0 equal] [-1 less] */
    if (script->cmp1 <= 0)
    {
        if (op_if->else_offset > 0)
        {
            script->code.max_offt       = op_if->else_offset;
            script->code.jmp_offt       = op_if->end_offset;
        }   
    
        return 0;
    }
        
    if (op_if->else_offset > 0)
        script->code.offt           = op_if->else_offset;
    else
        script->code.offt           = op_if->end_offset;
    
    return 1;
}
/**********************************************************************************************************************/
int BrbMicroScriptJmpLesserFunc(void *base_ptr, void *cb_data)
{
    BrbBase *brb_base               = (BrbBase*)base_ptr;
    BrbMicroScript *script          = (BrbMicroScript*)cb_data;
    BrbMicroScriptOPIf *op_if       = (BrbMicroScriptOPIf *)&script->code.arr[script->code.offt + 1];
    
    LOG_DEBUG(brb_base->log_base, "LE [%d] - el [%u] en [%u]\r\n", script->cmp1, op_if->else_offset, op_if->end_offset);
    
    /* check param [1 great] [0 equal] [-1 less] */
    if (script->cmp1 < 0)
    {
        if (op_if->else_offset > 0)
        {
            script->code.max_offt       = op_if->else_offset;
            script->code.jmp_offt       = op_if->end_offset;
        }   
    
        return 0;
    }
        
    if (op_if->else_offset > 0)
        script->code.offt           = op_if->else_offset;
    else
        script->code.offt           = op_if->end_offset;
    
    return 1;
}
/**********************************************************************************************************************/
int BrbMicroScriptJmpNotLesserFunc(void *base_ptr, void *cb_data)
{
    BrbBase *brb_base               = (BrbBase*)base_ptr;
    BrbMicroScript *script          = (BrbMicroScript*)cb_data;
    BrbMicroScriptOPIf *op_if       = (BrbMicroScriptOPIf *)&script->code.arr[script->code.offt + 1];
    
    LOG_DEBUG(brb_base->log_base, "NLE [%d] - el [%u] en [%u]\r\n", script->cmp1, op_if->else_offset, op_if->end_offset);
    
    /* check param [1 great] [0 equal] [-1 less] */
    if (script->cmp1 >= 0)
    {
        if (op_if->else_offset > 0)
        {
            script->code.max_offt       = op_if->else_offset;
            script->code.jmp_offt       = op_if->end_offset;
        }   
    
        return 0;
    }
        
    if (op_if->else_offset > 0)
        script->code.offt           = op_if->else_offset;
    else
        script->code.offt           = op_if->end_offset;
    
    return 1;
}
/**********************************************************************************************************************/
int BrbMicroScriptRunOnce(BrbBase *brb_base, BrbMicroScript *script)
{
    BrbMicroScriptOPRunTime *opcode_run;
    BrbMicroScriptOP *opcode;
    BrbGenericCBH *cb_func;
    BrbMicroScriptOPDelay *op_delay;
    uint8_t ret_code;
    // uint16_t param;
    
    // LOG_NOTICE(brb_base->log_base, "RUN script [%u] - Persist [%u] Delay [%u:%ld/%ld] - code [%u:%u]\r\n", 
    // script->script_id, script->flags.persist, script->flags.delaying, script->delay_until_ms, brb_base->ms.cur, script->code.offt, script->code.size);

    // brb_base->log_base->serial->flush();

    /* We are delaying, check */
    if ((script->flags.delaying) && (script->delay_until_ms >= brb_base->ms.cur))
        return 0;
    /* Finished delaying, reset flag */
    else
    {
        script->flags.delaying = 0;
    }

    /* Run the binary opcodes */
    while (script->code.offt < script->code.size)
    {
        // LOG_NOTICE(brb_base->log_base, "RUN script [%u] - code [%u:%u]\r\n", script->script_id, script->code.offt, script->code.size);

        // brb_base->log_base->serial->flush();

        /* Grab current opcode */
        opcode      = (BrbMicroScriptOP *)&script->code.arr[script->code.offt]; 
   
        // LOG_NOTICE(brb_base->log_base, "RUN script [%u] - code [%u:%u] - op:%u sz :%u\r\n", 
        //     script->script_id, script->code.offt, script->code.size, opcode->opcode, opcode->param_sz);
        
        // brb_base->log_base->serial->flush();

        /* Sanity check */
        if ((opcode->opcode < SCRIPT_OPCODE_NOT_INIT) || (opcode->opcode >= SCRIPT_OPCODE_LASTITEM))
        {
            /* SHOUT ERROR */
            continue;
        }

        /* Prepare to run CB_FUNC */
        opcode_run  = (BrbMicroScriptOPRunTime *)&glob_script_runtime_arr[opcode->opcode];
        cb_func     = opcode_run->cb_func;

        /* Some opcodes need special code path */
        switch (opcode->opcode)
        {
            /* Leave execution if we find a delay */
            case SCRIPT_OPCODE_DELAY:
                
                // BrbMicroScriptOP *op_hdr;
                op_delay                = (BrbMicroScriptOPDelay *)&script->code.arr[script->code.offt + 1];

                /* Copy param */
                // memcpy(&param, &script->code.arr[script->code.offt + 1], sizeof(uint16_t));

                // LOG_NOTICE(brb_base->log_base, "RUN DELAY [%u] - tm [%u] sz [%u]\r\n", script->script_id, op_delay->time, opcode->param_sz);

                /* Set when to delay */
                script->delay_until_ms  = brb_base->ms.cur + op_delay->time;

                /* Set flags we are delaying and point to next PARAM */
                script->flags.delaying  = 1;
                script->code.offt       += 1 + opcode->param_sz;
                    
                return script->code.offt;

            /* Default action */
            default:
            
                break;
        }

        /* Invoke opcode */
        if (cb_func)
        {
            ret_code    = cb_func(brb_base, script);

            // LOG_NOTICE(brb_base->log_base, "OP CODE [%u] - ret [%u]\r\n", opcode->opcode, ret_code);

            /* Act based on ret code */
            if (ret_code != 0)
                continue;
        }

        /* Walk to next opcode */
        //next_opcode:       
        script->code.offt += 1 + opcode->param_sz;

        if (script->code.max_offt > 0 && script->code.offt >= script->code.max_offt)
        {
            script->code.offt       = script->code.jmp_offt;

            script->code.max_offt   = 0;
            script->code.jmp_offt   = 0;
        }

        // LOG_NOTICE(brb_base->log_base, "LOOP offt [%u] - size [%u]\r\n", script->code.offt, script->code.size);

        continue;
    }

    // LOG_NOTICE(brb_base->log_base, "DONE persist [%u]\r\n", script->flags.persist);

    /* Finished running, if persist then restart OFFSET, else mark as finished */
    if (script->flags.persist)
        script->code.offt = 0;
    else
        script->flags.finished = 1;

    return script->code.offt;
}
/**********************************************************************************************************************/
int BrbMicroScriptRunAll(BrbBase *brb_base)
{
    BrbMicroScript *script;
    int i;

    /* Select a free timer slot from array */
    for (i = 0; i < MAX_SCRIPT; i++)
    {
        /* Not active, ignore */
        if (!brb_base->script.arr[i].flags.active)
            continue;
  
        /* Grab script and run once */
        script = (BrbMicroScript *)&brb_base->script.arr[i];
        BrbMicroScriptRunOnce(brb_base, script);

        /* Finished running script, delete it */
        if (script->flags.finished)
        {
            memset(script, 0, sizeof(BrbMicroScript));
        }

        continue;
    }

    return 1;
}
/**********************************************************************************************************************/
BrbMicroScript *BrbMicroScriptGrabByID(BrbBase *brb_base, int script_id)
{
    BrbMicroScript *script;

    LOG_WARN(brb_base->log_base, "[%p] - GRAB SCRIPT [%d]\r\n", brb_base, script_id);

    /* Sanity check */
    if (script_id > MAX_TIMER)
        return NULL;

    script = (BrbMicroScript *)&brb_base->script.arr[script_id];

    return script;
}
/**********************************************************************************************************************/
BrbMicroScript *BrbMicroScriptGrabFree(BrbBase *brb_base)
{
    BrbMicroScript *script;
    int i;

    script = NULL;

    /* Select a free timer slot from array */
    for (i = 0; i < MAX_SCRIPT; i++)
    {
        if (brb_base->script.arr[i].flags.active)
            continue;
            
        /* Fill up script data */
        script              = (BrbMicroScript *)&brb_base->script.arr[i];
        script->script_id   = i;
        break;
    }

    /* No more timers */
    if (!script)
    {
        LOG_WARN(brb_base->log_base, "Failed Grab Script\r\n");
        return NULL;
    }

    return script;
}
/**********************************************************************************************************************/
BrbMicroScript *BrbMicroScriptSetByID(BrbBase *brb_base, BrbMicroScript *script_new)
{
    BrbMicroScript *script;
    
    // LOG_NOTICE(brb_base->log_base, "[%p] - SET SCRIPT [%d] - [%d]\r\n", brb_base, script_new->script_id, MAX_TIMER);

    /* Sanity check */
    if (script_new->script_id > MAX_TIMER)
        return NULL;

    script      = &brb_base->script.arr[script_new->script_id];

    // LOG_NOTICE(brb_base->log_base, "[%p] - Set Script [%d] - [%d] [%d] - sz [%d]\r\n", brb_base, 
    //     script->script_id, script->flags.active, script->flags.finished, sizeof(BrbMicroScript));

    // LOG_WARN(brb_base->log_base, "[%p] - Set Script [%d] - [%d] [%d] - sz [%d]\r\n", brb_base, 
    //     script_new->script_id, script_new->flags.active, script_new->flags.finished, sizeof(BrbMicroScript));

    memcpy(script, script_new, sizeof(BrbMicroScript));

    return script;
}
/**********************************************************************************************************************/