/*
 * BrbBtnBase.h
 *
 *  Created on: 2019-01-31
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

#ifndef BRB_BTN_BASE_H_
#define BRB_BTN_BASE_H_
/**********************************************************************************************************************/
#include <BrbBase.h>
/****************************************************************************************************/
typedef enum
{
    BRB_BTN_SELECT,
    BRB_BTN_NEXT,
    BRB_BTN_PREV,
    BRB_BTN_LAST_ITEM
} BrbBtnCode;
/**********************************************************************************************************************/
typedef struct _BrbBtnData
{
    int state;

    int hit;
    int pin;

} BrbBtnData;

typedef struct _BrbBtnBase
{
    BrbBase *brb_base;

    BrbBtnData buttons[BRB_BTN_LAST_ITEM];

} BrbBtnBase;
/**********************************************************************************************************************/
int BrbBtnBase_Init(BrbBtnBase *btn_base);
void BrbBtnBase_Loop(BrbBtnBase *btn_base);
/**********************************************************************************************************************/
#endif /* BRB_BTN_BASE_H_ */