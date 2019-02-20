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

/**********************************************************************************************************************/
int BrbDisplayBase_Init(BrbDisplayBase *display_base)
{
	/* Sanitize */
	if (!display_base)
		return -1;

	if (!display_base->tft)
	{
		display_base->tft = new ILI9341_due(display_base->pin_cs, display_base->pin_dc, display_base->pin_rst);
	}

    LOG_NOTICE(display_base->brb_base->log_base, "DISPLAY - INIT [%d]\r\n", display_base->screen_arr_cnt);

	display_base->tft->begin();
	display_base->tft->setRotation(DISPLAY_ROTATION_DEFAULT);
	display_base->tft->setFont(DISPLAY_FONT_DEFAULT);

	display_base->screen_last = -1;
	display_base->action_code = -1;
	display_base->flags.on_action = 0;
	display_base->box.text_color = DISPLAY_COLOR_BOX_VALUE;

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

	if (display_base->screen_cur < 0)
		display_base->screen_cur = display_base->screen_arr_cnt - 1;
	else if (display_base->screen_cur >= display_base->screen_arr_cnt)
		display_base->screen_cur = 0;

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
int BrbDisplayBase_SetTitle(BrbDisplayBase *display_base, const char *title_str)
{
	display_base->tft->fillRect(DISPLAY_SZ_MARGIN, DISPLAY_SZ_MARGIN, DISPLAY_SZ_TITLE_W, DISPLAY_SZ_TITLE_H, DISPLAY_COLOR_TITLE_BG);
	display_base->tft->setFont(DISPLAY_FONT_SCREEN_TITLE);
	display_base->tft->setTextColor(DISPLAY_COLOR_TITLE_TEXT, DISPLAY_COLOR_TITLE_BG);

	display_base->tft->cursorToXY(DISPLAY_SZ_MARGIN * 2, DISPLAY_SZ_MARGIN * 2);
	display_base->tft->setTextScale(1);
	display_base->tft->println((const __FlashStringHelper *)title_str);
	display_base->tft->setTextColor(DISPLAY_COLOR_TEXT_DEFAULT, DISPLAY_COLOR_BG);

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_SetBg(BrbDisplayBase *display_base)
{
	for (int i = 0; i < DISPLAY_SZ_BORDER; i++)
	{
		display_base->tft->drawRect(i, i, DISPLAY_SZ_DISPLAY_W - (i + 1), DISPLAY_SZ_DISPLAY_H - (i + 1), DISPLAY_COLOR_BORDER);
	}
	display_base->tft->fillRect(DISPLAY_SZ_BORDER, DISPLAY_SZ_BORDER, DISPLAY_SZ_DISPLAY_W - (DISPLAY_SZ_BORDER * 2), DISPLAY_SZ_DISPLAY_H - (DISPLAY_SZ_BORDER * 2), DISPLAY_COLOR_BG);

	return 0;
}
/**********************************************************************************************************************/