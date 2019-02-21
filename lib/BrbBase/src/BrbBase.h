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
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

#include "Servo.h"

#include "Boards.h"

#include "log/BrbLogBase.h"
#include "data/BrbDLinkedList.h"
#include "data/BrbMicroScript.h"

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
#define MAX_SERVO 4

#define MIN_ANA_PIN 0
#define MAX_ANA_PIN NUM_ANALOG_INPUTS

#define MIN_DIG_PIN 2

// #define MAX_DIG_PIN NUM_DIGITAL_PINS
#define MAX_DIG_PIN PIN_A0

#define BRB_EEPROM_MAGIC 157
#define BRB_EEPROM_MASK 137
#define BRB_EEPROM_OFFSET 32

#define BRB_COMPARE_NUM(a, b) (a > b) - (a < b)
/**********************************************************************************************************************/
/* ENUMS */
/**********************************************************/

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
typedef struct _BrbBasePinData
{
    uint8_t value;
	uint8_t mode;

    uint8_t mask;
    uint8_t persist:1;
    uint8_t pad:7;

} BrbBasePinData;
/**********************************************************/
typedef struct _BrbServo
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
    
} BrbServo;
/**********************************************************/
typedef struct _BrbBase
{
	BrbLogBase *log_base;
    BrbBasePinData pin_data[TOTAL_PINS];
    BrbMicroScriptBase script_base;

    struct
    {
        int loop_cnt;
    } stats;

    struct
    {
        long last;
        long cur;
        int delay;

        long lifetime_delay;
        long lifetime_sec;
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
        BrbServo arr[MAX_SERVO];
    } servo;

    struct
    {
        long long lifetime_sec;
        int upcount;
    } data;

} BrbBase;
/**********************************************************************************************************************/
/* PUBLIC FUNCTIONS */
/**********************************************************/
void BrbBaseInit(BrbBase *brb_base);
void BrbBaseLoop(BrbBase *brb_base);

void BrbBase_DataLoad(BrbBase *brb_base);
void BrbBase_DataSave(BrbBase *brb_base);

/**********************************************************/
/* BrbTimer */
/**********************************************************/
BrbTimer *BrbTimerGrabByID(BrbBase *brb_base, int timer_id);
int BrbTimerAdd(BrbBase *brb_base, long delay_ms, int persist, BrbGenericCBH *cb_func, void *cb_data);
void BrbTimerDispatch(BrbBase *brb_base);

/**********************************************************/
/* BrbBase */
/**********************************************************/
void BrbBase_PinLoad(BrbBase *brb_base);
void BrbBase_PinCheck(BrbBase *brb_base);
void BrbBase_PinSet(BrbBase *brb_base, int pin_num, int pin_mode, int pin_value);
void BrbBase_PinSave(BrbBase *brb_base);
uint8_t BrbBase_PinGetMode(uint8_t pin);

int BrbBase_EEPROMRead(BrbBase *brb_base, uint8_t *data_ptr, uint8_t data_sz, uint8_t eeprom_offset);
int BrbBase_EEPROMWrite(BrbBase *brb_base, uint8_t *data_ptr, uint8_t data_sz, uint8_t eeprom_offset);

/**********************************************************/
/* BrbServo */
/**********************************************************/
BrbServo *BrbServoGrabByID(BrbBase *brb_base, int servo_id);
BrbServo *BrbServoGrabFree(BrbBase *brb_base);
BrbServo *BrbServoGrabByPin(BrbBase *brb_base, int pin);
int BrbServoAttach(BrbBase *brb_base, BrbServo *servo, int pin);
BrbServo *BrbServoSetPosByPin(BrbBase *brb_base, int pin, int pos_set);
BrbServo *BrbServoSetPos(BrbBase *brb_base, BrbServo *servo, int pos_set);
/**********************************************************************************************************************/
typedef struct _BrbSensorVoltage
{
	double value;
	int counter;
	int pin;

} BrbSensorVoltage;

typedef struct _BrbZeroCross
{
	double value;
	int counter;
	int pin;

} BrbZeroCross;
/**********************************************************************************************************************/

#endif /* BRB_BASE_H_ */