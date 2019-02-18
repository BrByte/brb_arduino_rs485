/*
 * main.h
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

#ifndef MAIN_H_
#define MAIN_H_
/**********************************************************************************************************************/
/* Import needed libraries */
#include "Arduino.h"
#include "BrbBase.h"

#include "BrbBtnBase.h"
#include "BrbToneBase.h"

#include <BrbDisplayBase.h>

#include <BrbRS485Session.h>

#include "Adafruit_Sensor.h"
#include "DHT.h"

#include <avr/wdt.h>

/* simple define to organize code in one */
// #define PDU_SYSTEM_COMPILE 1

/**********************************************************************************************************************/
/* DEFINES */
/**********************************************************/
#ifdef PDU_SYSTEM_COMPILE
#include "include/core_pdu.h"
#else
#include "include/core_gerador.h"
#endif

/**********************************************************************************************************************/
/* Display */
/**********************************************************/
int BrbAppDisplay_Setup(BrbBase *brb_base);
/**********************************************************************************************************************/
/* RS485 */
/**********************************************************/
int BrbAppRS485_Setup(BrbBase *brb_base);
/**********************************************************************************************************************/
/* Global control structures */
extern BrbLogBase *glob_log_base;
extern BrbBase glob_brb_base;
extern BrbRS485Session glob_rs485_sess;

extern BrbBtnBase glob_btn_base;
extern BrbDisplayBase glob_display_base;
extern BrbToneBase glob_tone_base;

#ifdef PDU_SYSTEM_COMPILE
extern BrbPDUBase glob_pdu_base;
#else
extern BrbGeradorBase glob_gerador_base;
#endif

/**********************************************************************************************************************/
#endif /* MAIN_H_ */
