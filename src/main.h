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
#include <BrbLogBase.h>
#include <BrbBase.h>
#include <BrbRS485Session.h>

#include <BrbBtnBase.h>
#include <BrbDisplayBase.h>
#include <BrbGeradorBase.h>
#include <BrbToneBase.h>

#include "Adafruit_Sensor.h"
#include "DHT.h"

#include <avr/wdt.h>

/**********************************************************************************************************************/
/* DEFINES */
/**********************************************************/
// #define RESERVED    0 /* RX0 */
// #define RESERVED    1 /* TX0 */
#define GERADOR_ZEROCROSS_PIN 2 /* INT4 - PWM */
// #define RESERVED    3 /* INT5 - PWM */
#define GERADOR_PARTIDA_PIN 4 /* PWM */
#define GERADOR_PARADA_PIN 5  /* PWM */
#define GERADOR_EXTRA_PIN 6   /* PWM */
#define BUZZER_PIN 7          /* PWM */
#define DHT_SENSOR_PIN 8      /* PWM */
#define GERADOR_SERVO_PIN 9   /* PWM */
// #define RESERVED    10 /* PCINT 4 */
// #define RESERVED    11 /* PCINT 5 */
// #define RESERVED    12 /* PCINT 6 */
// #define RESERVED    13 /* PCINT 7 */
#define RS485_DI_PIN 14 /* PCINT10 - TX3 */
#define RS485_RO_PIN 15 /* PCINT9 - RX3 */
// #define RESERVED    16 /* TX2 */
// #define RESERVED    17 /* RX2 */
// #define RESERVED    18 /* INT3 - TX1 */
// #define RESERVED    19 /* INT2 - RX1 */
// #define RESERVED    20 /* INT0 - SCL */
// #define RESERVED    21 /* INT1 - SDA */
#define RS485_REDE_PIN 22 /* TOGGLE PIN (RE + DE) */
// #define RESERVED    23 /* */
// #define RESERVED    24 /* */
// #define RESERVED    25 /* */
#define BTN_PIN_SELECT 26 /* */
#define BTN_PIN_NEXT 27   /* */
#define BTN_PIN_PREV 28   /* */
// #define RESERVED    29 /* */
// #define RESERVED    30 /* */
// #define RESERVED    31 /* */
// #define RESERVED    32 /* */
// #define RESERVED    33 /* */
// #define RESERVED    34 /* */
// #define RESERVED    35 /* */
// #define RESERVED    36 /* */
// #define RESERVED    37 /* */
// #define RESERVED    38 /* */
// #define RESERVED    39 /* */
// #define RESERVED    40 /* */
// #define RESERVED    41 /* */
// #define RESERVED    42
// #define RESERVED    43 /*  */
// #define RESERVED    44 /*  */
// #define RESERVED    45 /* PWM */
#define TFT_LED 46  /* PWM */
#define TFT_CS 47   /*  */
#define TFT_DC 48   /*  */
#define TFT_RST 49  /* */
#define TFT_MISO 50 /* PCINT3 - MISO */
#define TFT_MOSI 51 /* PCINT2 - MOSI */
#define TFT_CLK 52  /* PCINT1 - SCK */
// #define RESERVED     53 /* PCINT0 - SS */

#define SENSOR_VOLTAGE_DC_PIN A1
#define SENSOR_VOLTAGE_AC_PIN A2

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
extern BrbGeradorBase glob_gerador_base;
extern BrbToneBase glob_tone_base;
/**********************************************************************************************************************/
#endif /* MAIN_H_ */
