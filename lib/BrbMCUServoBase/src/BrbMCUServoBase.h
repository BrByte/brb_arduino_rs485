/*
 * BrbMCUServoBase.h
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

#ifndef BRB_MCU_SERVO_BASE_H_
#define BRB_MCU_SERVO_BASE_H_

/**********************************************************************************************************************/
#include "Arduino.h"
#include "Servo.h"
// #include "BrbBase.h"
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

#define MAX_SERVO 4
/**********************************************************************************************************************/
/* ENUMS */
/**********************************************************/

/**********************************************************************************************************************/
/* STRUCTS */
/**********************************************************/
typedef struct _BrbMCUServo
{
    uint8_t pin;
	uint8_t servo_id;
	uint8_t pos_cur;

    Servo *servo;
    
    struct
    {
    	uint16_t active:1;
    	uint16_t attached:1;
    } flags;
    
} BrbMCUServo;
/**********************************************************/
typedef struct _BrbMCUServoBase
{
    int count;
    BrbMCUServo arr[MAX_SERVO];

} BrbMCUServoBase;
/**********************************************************************************************************************/
/* PUBLIC FUNCTIONS */
/**********************************************************/
/* BrbMCUServo */
/**********************************************************/
int BrbMCUServoBase_Init(BrbMCUServoBase *servo_base);
BrbMCUServo *BrbMCUServoGrabByID(BrbMCUServoBase *servo_base, int servo_id);
BrbMCUServo *BrbMCUServoGrabFree(BrbMCUServoBase *servo_base);
BrbMCUServo *BrbMCUServoGrabByPin(BrbMCUServoBase *servo_base, int pin);
int BrbMCUServoAttach(BrbMCUServoBase *servo_base, BrbMCUServo *servo, int pin);
BrbMCUServo *BrbMCUServoSetPosByPin(BrbMCUServoBase *servo_base, int pin, int pos_set);
BrbMCUServo *BrbMCUServoSetPos(BrbMCUServoBase *servo_base, BrbMCUServo *servo, int pos_set);
/**********************************************************************************************************************/

#endif /* BRB_BASE_H_ */