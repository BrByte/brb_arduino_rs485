/*
 * BrbDisplayBase.h
 *
 *  Created on: 2019-01-27
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

#ifndef BRB_DYSPLAY_BASE_H_
#define BRB_DYSPLAY_BASE_H_
/**********************************************************************************************************************/
#include <BrbBase.h>
#include <BrbBtnBase.h>

#include <SPI.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_ILI9341.h>
#include <ILI9341_due_config.h>
#include <ILI9341_due.h>

// #include <SystemFont5x7.h>
#include <fonts/Arial14.h>
#include <fonts/Tahoma24.h>
#include <fonts/Ubuntu.h>
/****************************************************************************************************/
#define BRB_DISPLAY_FONT_DEFAULT Arial_14
// #define BRB_DISPLAY_FONT_DEFAULT SystemFont5x7

#define BRB_DISPLAY_FONT_TITLE Arial_14
#define BRB_DISPLAY_FONT_SUB Arial_14
#define BRB_DISPLAY_FONT_VALUE Tahoma24

#define BRB_DISPLAY_ROTATION_DEFAULT iliRotation270

#define ARC_SCHEME_RED2RED 0
#define ARC_SCHEME_GREEN2GREEN 1
#define ARC_SCHEME_BLUE2BLUE 2
#define ARC_SCHEME_BLUE2RED 3
#define ARC_SCHEME_GREEN2RED 4
#define ARC_SCHEME_RED2GREEN 5

typedef enum
{
	DISPLAY_SCREEN_INFO,
	DISPLAY_SCREEN_CONTROL,
	DISPLAY_SCREEN_HOUR_METER,
	DISPLAY_SCREEN_LASTITEM
} BrbDisplayScreen;

typedef struct _BrbDisplayScreenEvents
{
	BrbGenericCBH *cb_func;
	void *cb_data;
} BrbDisplayScreenEvents;

/**********************************************************************************************************************/
typedef struct _BrbDisplayBase
{
    BrbBase *brb_base;

    // Adafruit_ILI9341 *tft;
    ILI9341_due *tft;

	int screen_last;
	int screen_cur;
	int screen_int;

	int action_code;
	
	int user_int;

	// uint8_t pin_dc;
	// uint8_t pin_rst;
	// uint8_t pin_cs;
	// uint8_t pin_miso;
	// uint8_t pin_mosi;
	// uint8_t pin_clk;

	BrbDisplayScreenEvents cb_show[DISPLAY_SCREEN_LASTITEM];
	BrbDisplayScreenEvents cb_action[BRB_BTN_LAST_ITEM];

	struct {
		unsigned int on_action:1;
		// unsigned int on_select:1;
	} flags;

} BrbDisplayBase;
/**********************************************************************************************************************/
int BrbDisplayBase_Init(BrbDisplayBase *display_base);

// int BrbDisplayBase_DrawArc(BrbDisplayBase *display_base, int value, int vmin, int vmax, int x, int y, int r, __FlashStringHelper *units, byte scheme);
int BrbDisplayBase_DrawArc(BrbDisplayBase *display_base, double value, int vmin, int vmax, int x, int y, int r, const __FlashStringHelper *units, byte scheme);

int BrbDisplayBase_SetTitle(BrbDisplayBase *display_base, const __FlashStringHelper *title_str, int x, int y);
int BrbDisplayBase_SetBg(BrbDisplayBase *display_base);

int BrbDisplayBase_ScreenAction(BrbDisplayBase *display_base, int action_code);

int BrbDisplayBase_SetScreenShowCB(BrbDisplayBase *display_base, int screen_code, BrbGenericCBH *cb_func, void *cb_data);
int BrbDisplayBase_SetScreenActionCB(BrbDisplayBase *display_base, int screen_code, BrbGenericCBH *cb_func, void *cb_data);
/**********************************************************************************************************************/

/**********************************************************************************************************************/
#endif /* BRB_DYSPLAY_BASE_H_ */