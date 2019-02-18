/*
 * BrbDisplayBase.cpp
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

#include "BrbDisplayBase.h"

static unsigned int color_lt(unsigned int color, float pct);
static unsigned int rainbow(byte value);

/**********************************************************************************************************************/
int BrbDisplayBase_Init(BrbDisplayBase *display_base)
{
	/* Sanitize */
	if (!display_base)
		return -1;

	// display_base->tft = (ILI9341_due *)&tft;
	if (!display_base->tft)
	{
		display_base->tft = new ILI9341_due(display_base->pin_cs, display_base->pin_dc, display_base->pin_rst);
	}

	display_base->tft->begin();
	// display_base->tft->reset();
	display_base->tft->setRotation(DISPLAY_ROTATION_DEFAULT);
	display_base->tft->setFont(DISPLAY_FONT_DEFAULT);

	display_base->screen_last = -1;
	display_base->action_code = -1;
	display_base->flags.on_action = 0;

	if (display_base->pin_led > MIN_DIG_PIN)
	{
		pinMode(display_base->pin_led, OUTPUT);
		digitalWrite(display_base->pin_led, HIGH);
	}

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_Loop(BrbDisplayBase *display_base)
{

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_ScreenAction(BrbDisplayBase *display_base, int action_code)
{
	int op_status;

	if (!display_base->screen_arr_ptr)
		return 0;

	display_base->action_code = action_code;
	display_base->flags.on_select = 0;

	if (!display_base->flags.on_action)
	{
		if (display_base->action_code == DISPLAY_ACTION_SELECT)
		{
			display_base->flags.on_select = 1;
			display_base->action_code = -1;
			display_base->screen_last = -1;
		}

		/* Press direction */
		else if (display_base->action_code == DISPLAY_ACTION_NEXT)
			display_base->screen_cur++;

		else if (display_base->action_code == DISPLAY_ACTION_PREV)
			display_base->screen_cur--;
	}

	BrbDisplayScreenPrototype *screen_ptr = NULL;

	for (int i = 0; i < display_base->screen_arr_cnt; i++)
	{
		/* Found */
		if (display_base->screen_arr_ptr[i].cb_func && display_base->screen_arr_ptr[i].code == display_base->screen_cur)
		{
			screen_ptr = (BrbDisplayScreenPrototype *)&display_base->screen_arr_ptr[i];
			break;
		}
	}

	/* Get first screen, with reseted info */
	if (!screen_ptr)
	{
		display_base->flags.on_action = 0;
		display_base->flags.on_select = 0;
		display_base->action_code = -1;
		display_base->screen_last = -1;
		display_base->screen_cur = 0;
		screen_ptr = (BrbDisplayScreenPrototype *)&display_base->screen_arr_ptr[0];
	}

	/* Ignore Screen */
	if (screen_ptr->cb_func == NULL)
		return 0;

	op_status = screen_ptr->cb_func(display_base->brb_base, display_base);

	display_base->screen_last = display_base->screen_cur;

	return op_status;
}
/**********************************************************************************************************************/
int BrbDisplayBase_SetTitle(BrbDisplayBase *display_base, const char *title_str, int x, int y)
{
	display_base->tft->setFont(DISPLAY_FONT_TITLE);
	display_base->tft->fillRect(x, y, 300, 30, ILI9341_STEELBLUE);
	display_base->tft->cursorToXY(x + 10, y + 10);
	display_base->tft->setTextColor(ILI9341_WHITE, ILI9341_STEELBLUE);
	display_base->tft->setTextScale(1);
	display_base->tft->println((const __FlashStringHelper *)title_str);
	// display_base->tft->printAtPivoted(title_str, x + 10, y + 10, gTextPivotBottomCenter); // Value in middle

	display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_SetBg(BrbDisplayBase *display_base)
{
	display_base->tft->drawRect(0, 0, 320, 240, ILI9341_DIMGRAY);
	display_base->tft->drawRect(1, 1, 319, 239, ILI9341_DIMGRAY);
	display_base->tft->fillRect(2, 2, 316, 236, ILI9341_WHITE);

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_Test(BrbDisplayBase *display_base)
{
	int pos_s = 15;
	int pos_x = 15;
	int pos_y = 15;

	int btn_w = 210;
	int btn_h = 50;

	int txt_s = 10;
	int txt_x = pos_x + txt_s;
	int txt_y = pos_y + txt_s;

	// display_base->tft->setRotation(3);

	display_base->tft->fillScreen(ILI9341_BLACK);
	display_base->tft->setTextScale(2);
	display_base->tft->cursorToXY(0, 0);

	display_base->tft->setTextColor(ILI9341_WHITE);

	display_base->tft->fillRect(pos_x, pos_y, btn_w, btn_h, ILI9341_RED);
	pos_y = pos_y + pos_s + btn_h;
	display_base->tft->cursorToXY(txt_x, txt_y);
	txt_y = pos_y + txt_s;

	display_base->tft->println("Button Test 01");

	display_base->tft->fillRect(pos_x, pos_y, btn_w, btn_h, ILI9341_YELLOW);
	pos_y = pos_y + pos_s + btn_h;
	display_base->tft->cursorToXY(txt_x, txt_y);
	txt_y = pos_y + txt_s;

	display_base->tft->println("Button Test 02");

	display_base->tft->fillRect(pos_x, pos_y, btn_w, btn_h, ILI9341_GREEN);
	pos_y = pos_y + pos_s + btn_h;
	display_base->tft->cursorToXY(txt_x, txt_y);
	txt_y = pos_y + txt_s;

	display_base->tft->println("Button Test 03");

	display_base->tft->fillRect(pos_x, pos_y, btn_w, btn_h, ILI9341_RED);
	pos_y = pos_y + pos_s + btn_h;
	display_base->tft->cursorToXY(txt_x, txt_y);
	txt_y = pos_y + txt_s;

	display_base->tft->println("Button Test 04");

	// display_base->tft->fillRect(cx - i2, cy - i2, i, i, color1);

	return 1;
}
/**********************************************************************************************************************/