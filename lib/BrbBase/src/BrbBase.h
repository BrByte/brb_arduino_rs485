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
// #include "boards.h"
#include "log/BrbLogBase.h"
#include "data/BrbDLinkedList.h"
#include "data/BrbMicroScript.h"
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

#define MIN_DIG_PIN 2

// #define MAX_DIG_PIN NUM_DIGITAL_PINS
#define MAX_DIG_PIN PIN_A0

#define BRB_EEPROM_MAGIC 157
#define BRB_EEPROM_MASK 137
#define BRB_EEPROM_OFFSET 32

#define BRB_COMPARE_NUM(a, b) (a > b) - (a < b)

#define TOTAL_PINS 70 // 54 digital + 16 analog
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
/**********************************************************/
typedef struct _BrbBase
{
	BrbLogBase *log_base;
    BrbBasePinData pin_data[TOTAL_PINS];
    BrbMicroScriptBase script_base;

    struct
    {
        uint32_t loop_cnt;
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

int BrbBase_FreeRAM(void);
/**********************************************************/
/* BrbTimer */
/**********************************************************/
BrbTimer *BrbTimerGrabByID(BrbBase *brb_base, int timer_id);
int BrbTimerAdd(BrbBase *brb_base, long delay_ms, int persist, BrbGenericCBH *cb_func, void *cb_data);
void BrbTimerDispatch(BrbBase *brb_base);

/**********************************************************/
/* BrbBase */
/**********************************************************/
uint8_t BrbBase_PinGetMode(uint8_t pin);
uint8_t BrbBase_PinGetAnalogPin(uint8_t pin);

int BrbBase_EEPROMRead(BrbBase *brb_base, uint8_t *data_ptr, uint8_t data_sz, uint8_t eeprom_offset);
int BrbBase_EEPROMWrite(BrbBase *brb_base, uint8_t *data_ptr, uint8_t data_sz, uint8_t eeprom_offset);
/**********************************************************************************************************************/
#endif /* BRB_BASE_H_ */