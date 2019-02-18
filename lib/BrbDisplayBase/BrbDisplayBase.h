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

#ifndef BRB_DISPLAY_BASE_H_
#define BRB_DISPLAY_BASE_H_
/**********************************************************************************************************************/
#include <BrbBase.h>

#include <SPI.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_ILI9341.h>
#include <ILI9341_due_config.h>
#include <ILI9341_due.h>

// #include <SystemFont5x7.h>
#include <fonts/Arial14.h>
#include <fonts/OpenSans36.h>
#include <fonts/Ubuntu36.h>
/****************************************************************************************************/
#define DISPLAY_FONT_DEFAULT Arial_14
// // #define DISPLAY_FONT_DEFAULT SystemFont5x7

#define DISPLAY_FONT_TITLE Arial_14
#define DISPLAY_FONT_VALUE Ubuntu36
#define DISPLAY_FONT_SUB Arial_14

#define DISPLAY_FONT_ARC_VALUE Ubuntu36
#define DISPLAY_FONT_ARC_SUB Arial_14

#define DISPLAY_FONT_BOX_TITLE Arial_14
#define DISPLAY_FONT_BOX_VALUE Ubuntu36
#define DISPLAY_FONT_BOX_SUB Arial_14

/****************************************************************************************************/

#define DISPLAY_COLOR_BORDER ILI9341_DIMGRAY
#define DISPLAY_COLOR_BG ILI9341_WHITE

#define DISPLAY_COLOR_TITLE_TEXT ILI9341_WHITE
#define DISPLAY_COLOR_TITLE_BG ILI9341_BLUE

#define DISPLAY_COLOR_TEXT_DEFAULT ILI9341_BLACK

#define DISPLAY_ROTATION_DEFAULT iliRotation270

#define DISPLAY_ARC_RED2RED 0
#define DISPLAY_ARC_GREEN2GREEN 1
#define DISPLAY_ARC_BLUE2BLUE 2
#define DISPLAY_ARC_BLUE2RED 3
#define DISPLAY_ARC_GREEN2RED 4
#define DISPLAY_ARC_RED2GREEN 5

#define ARC_SEG 3
#define ARC_INC 5

// #define DISPLAY_COLOR_BASE_R 31
// #define DISPLAY_COLOR_BASE_G 63
// #define DISPLAY_COLOR_BASE_B 3

/****************************************************************************************************/
typedef enum
{
    DISPLAY_ACTION_SELECT,
    DISPLAY_ACTION_NEXT,
    DISPLAY_ACTION_PREV,
    DISPLAY_ACTION_LAST_ITEM
} BrbDisplayActionCode;

typedef struct _BrbDisplayScreenPrototype
{
	int code;
	const char *title_ptr;
	BrbGenericCBH *cb_func;
} BrbDisplayScreenPrototype;

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

	uint8_t pin_led;
	uint8_t pin_dc;
	uint8_t pin_rst;
	uint8_t pin_cs;
	uint8_t pin_miso;
	uint8_t pin_mosi;
	uint8_t pin_clk;

	BrbDisplayScreenPrototype *screen_arr_ptr;
	int screen_arr_cnt;

	// BrbDisplayScreenEvents cb_show[DISPLAY_SCREEN_LASTITEM];
	// BrbDisplayScreenEvents cb_screen[DISPLAY_SCREEN_LASTITEM];

	struct {
		unsigned int on_action:1;
		unsigned int on_select:1;
	} flags;

} BrbDisplayBase;
/**********************************************************************************************************************/
int BrbDisplayBase_Init(BrbDisplayBase *display_base);

int BrbDisplayBase_SetTitle(BrbDisplayBase *display_base, const char *title_str, int x, int y);
int BrbDisplayBase_SetBg(BrbDisplayBase *display_base);

int BrbDisplayBase_ScreenAction(BrbDisplayBase *display_base, int action_code);

int BrbDisplayBase_SetScreenShowCB(BrbDisplayBase *display_base, int screen_code, BrbGenericCBH *cb_func, void *cb_data);
int BrbDisplayBase_SetScreenActionCB(BrbDisplayBase *display_base, int screen_code, BrbGenericCBH *cb_func, void *cb_data);

int BrbDisplayBase_DrawBtn(BrbDisplayBase *display_base, int btn_x, int btn_y, int btn_w, int btn_h, const char *text_ptr, int btn_color, int txt_color);
int BrbDisplayBase_DrawBarGraph(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, int16_t pos_h, double value, double min, double max);
// int BrbDisplayBase_DrawArc(BrbDisplayBase *display_base, int value, int vmin, int vmax, int x, int y, int r, char *units, byte scheme);
int BrbDisplayBase_DrawArc(BrbDisplayBase *display_base, double value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme);
int BrbDisplayBase_DrawArcSeg(BrbDisplayBase *display_base, double value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme, int tick, int seg, int inc);

int BrbDisplayBase_BoxTitle(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr);
int BrbDisplayBase_BoxSub(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr, double value, int dots, const char *unit_ptr);
int BrbDisplayBase_BoxMax(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr, int value, int max);
int BrbDisplayBase_BoxValue(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, double value, int dots);
int BrbDisplayBase_BoxUnit(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *unit_ptr);

/**********************************************************************************************************************/

/**********************************************************************************************************************/
#endif /* BRB_DISPLAY_BASE_H_ */