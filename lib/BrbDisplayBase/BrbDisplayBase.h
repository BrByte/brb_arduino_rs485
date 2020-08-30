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

#include <brbyte_white.h>

// #include <SystemFont5x7.h>
#include <fonts/Arial14.h>
// #include <fonts/OpenSans36.h>
// #include <fonts/Ubuntu36.h>
// #include <fonts/TrebuchetMS34.h>
#include <fonts/Open_Sans_32pt.h>

#include <fonts/IconWorks_32pt.h>
#include <fonts/IconWorks_B_32pt.h>

// #include <fonts/Heydings_Controls_32pt.h>
// #include <fonts/Heydings_Icons_32pt.h>
/**********************************************************************************************************************/
/* DEFINES */
/**********************************************************/
/* FONTS */
/**********************************************************/
// #define DISPLAY_FONT_DEFAULT Arial_14
// // // #define DISPLAY_FONT_DEFAULT SystemFont5x7

#define DISPLAY_FONT_SCREEN_TITLE Arial_14

#define DISPLAY_FONT_DEFAULT Arial_14
#define DISPLAY_FONT_TITLE Open_Sans_32pt

#define DISPLAY_FONT_ARC_VALUE Open_Sans_32pt
#define DISPLAY_FONT_ARC_TICK Arial_14
#define DISPLAY_FONT_ARC_SUB Arial_14

#define DISPLAY_FONT_BOX_TITLE Arial_14
#define DISPLAY_FONT_BOX_VALUE Open_Sans_32pt
#define DISPLAY_FONT_BOX_SCALE 1
#define DISPLAY_FONT_BOX_SUB Arial_14


#define DISPLAY_FONT_ICON IconWorks_B_32pt
#define DISPLAY_FONT_ICON2 IconWorks_B_32pt

#define DISPLAY_FONT_ICON_INFO PSTR("m")
#define DISPLAY_FONT_ICON_ALERT PSTR("\xF9")

#define DISPLAY_FONT_ICON_POWER PSTR("\xCF")
#define DISPLAY_FONT_ICON_AUX PSTR("4")

#define DISPLAY_FONT_ICON_A2P PSTR("/")
#define DISPLAY_FONT_ICON_P2A PSTR("0")

#define DISPLAY_FONT_ICON_LAMP PSTR("r")

#define DISPLAY_FONT_ICON_ACTIVE PSTR("\xD9")
#define DISPLAY_FONT_ICON_USER PSTR("P")

#define DISPLAY_FONT_ICON_C_OFF PSTR("X")

#define DISPLAY_FONT_ICON_C_R PSTR("w")
#define DISPLAY_FONT_ICON_C_L PSTR("W")

// #define DISPLAY_FONT_ICON Heydings_Icons_32pt
// #define DISPLAY_FONT_ICON2 Heydings_Controls_32pt

// #define DISPLAY_FONT_ICON_INFO PSTR("i")
// #define DISPLAY_FONT_ICON_ALERT PSTR("!")
// #define DISPLAY_FONT_ICON_POWER PSTR("r")
// #define DISPLAY_FONT_ICON_AUX PSTR("4")

// #define DISPLAY_FONT_ICON_A2P PSTR("N")
// #define DISPLAY_FONT_ICON_P2A PSTR("@")

// #define DISPLAY_FONT_ICON_LAMP PSTR("l")

// #define DISPLAY_FONT_ICON_HANDON PSTR("8")
// #define DISPLAY_FONT_ICON_USER PSTR("A")

// #define DISPLAY_FONT_ICON_C_OFF PSTR("O")

// #define DISPLAY_FONT_ICON_C_R PSTR("<")
// #define DISPLAY_FONT_ICON_C_L PSTR(">")



/**********************************************************/
/* SIZE */
/**********************************************************/
#define DISPLAY_SZ_DISPLAY_W 320
#define DISPLAY_SZ_DISPLAY_H 240

#define DISPLAY_SZ_BORDER 0
#define DISPLAY_SZ_MARGIN DISPLAY_SZ_BORDER + 5

#define DISPLAY_SZ_BOX_H 12

#define DISPLAY_SZ_TITLE_M 8
#define DISPLAY_SZ_TITLE_H 25
#define DISPLAY_SZ_TITLE_W DISPLAY_SZ_DISPLAY_W - (DISPLAY_SZ_MARGIN + DISPLAY_SZ_TITLE_M)
/**********************************************************/
/* COLOR */
/**********************************************************/
#define DISPLAY_COLOR_BORDER ILI9341_DIMGRAY
#define DISPLAY_COLOR_BG ILI9341_WHITE

#define DISPLAY_COLOR_TITLE_TEXT ILI9341_WHITE
#define DISPLAY_COLOR_TITLE_BG ILI9341_MIDNIGHTBLUE

#define DISPLAY_COLOR_TEXT_DEFAULT ILI9341_BLACK

#define DISPLAY_COLOR_BOX_TITLE ILI9341_BLACK
#define DISPLAY_COLOR_BOX_VALUE ILI9341_ORANGERED

#define DISPLAY_ROTATION_DEFAULT iliRotation270

#define DISPLAY_ARC_RED2RED 0
#define DISPLAY_ARC_GREEN2GREEN 1
#define DISPLAY_ARC_BLUE2BLUE 2
#define DISPLAY_ARC_BLUE2RED 3
#define DISPLAY_ARC_GREEN2RED 4
#define DISPLAY_ARC_RED2GREEN 5

#define ARC_SEG 3
#define ARC_INC 5
/**********************************************************************************************************************/
/* ENUMS */
/**********************************************************/
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

	struct
	{
		uint16_t bg_color;

		uint16_t text_color;
		uint16_t text_scale;
	} box;

	// BrbDisplayScreenEvents cb_show[DISPLAY_SCREEN_LASTITEM];
	// BrbDisplayScreenEvents cb_screen[DISPLAY_SCREEN_LASTITEM];

	struct
	{
		unsigned int on_action : 1;
		unsigned int on_select : 1;
	} flags;

} BrbDisplayBase;
/**********************************************************************************************************************/
int BrbDisplayBase_Init(BrbDisplayBase *display_base);

int BrbDisplayBase_SetTitle(BrbDisplayBase *display_base, const char *title_str);
int BrbDisplayBase_SetBg(BrbDisplayBase *display_base);

int BrbDisplayBase_ScreenAction(BrbDisplayBase *display_base, int action_code);

int BrbDisplayBase_SetScreenShowCB(BrbDisplayBase *display_base, int screen_code, BrbGenericCBH *cb_func, void *cb_data);
int BrbDisplayBase_SetScreenActionCB(BrbDisplayBase *display_base, int screen_code, BrbGenericCBH *cb_func, void *cb_data);

/**********************************************************************************************************************/
unsigned int BrbDisplayBase_Rainbow(byte value);

/**********************************************************************************************************************/
int BrbDisplayBase_DrawBtn(BrbDisplayBase *display_base, int btn_x, int btn_y, int btn_w, int btn_h, const char *text_ptr, int btn_color, int txt_color);
int BrbDisplayBase_DrawBarGraph(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, int16_t pos_h, double value, double min, double max);
// int BrbDisplayBase_DrawArc(BrbDisplayBase *display_base, int value, int vmin, int vmax, int x, int y, int r, char *units, byte scheme);
int BrbDisplayBase_DrawArc(BrbDisplayBase *display_base, double value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme);
int BrbDisplayBase_DrawArcSeg(BrbDisplayBase *display_base, double value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme, int tick, int seg, int inc);

int BrbDisplayBase_BoxTitle(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr);
int BrbDisplayBase_BoxSub(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr, double value, int dots, const char *unit_ptr);

int BrbDisplayBase_BoxUptime(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr, long seconds);
int BrbDisplayBase_BoxMax(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr, int value, int max);
int BrbDisplayBase_BoxValue(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, double value, int dots);
int BrbDisplayBase_BoxUnit(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *unit_ptr);

int BrbDisplayBase_BoxFmt(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr, const char *format, ...);
/**********************************************************************************************************************/

/**********************************************************************************************************************/
#endif /* BRB_DISPLAY_BASE_H_ */