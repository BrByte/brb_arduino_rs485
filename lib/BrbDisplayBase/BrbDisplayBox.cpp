/*
 * BrbDisplayBox.cpp
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
int BrbDisplayBase_BoxTitle(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr)
{
	// display_base->tft->fillRect(pos_x, pos_y, 140, 15, ILI9341_CYAN);
	display_base->tft->setTextColor(DISPLAY_COLOR_TEXT_DEFAULT, DISPLAY_COLOR_BG);
	display_base->tft->setFont(DISPLAY_FONT_BOX_TITLE);
	display_base->tft->setTextScale(1);

	display_base->tft->cursorToXY(pos_x, pos_y);
	display_base->tft->print((const __FlashStringHelper *)title_ptr);

	// BrbDisplayBase_PrintAlignFlash(display_base, title_ptr, pos_x, pos_y, DISPLAY_TEXT_ALIGN_BL);

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_BoxSub(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr, double value, int dots, const char *unit_ptr)
{
	if (display_base->screen_cur != display_base->screen_last)
	{
		BrbDisplayBase_BoxTitle(display_base, pos_x, pos_y, title_ptr);
	}

	BrbDisplayBase_BoxValue(display_base, pos_x, pos_y + 15, value, dots);

	BrbDisplayBase_BoxUnit(display_base, pos_x, pos_y + 15, unit_ptr);

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_BoxMax(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *title_ptr, int value, int max)
{
	if (display_base->screen_cur != display_base->screen_last)
	{
		BrbDisplayBase_BoxTitle(display_base, pos_x, pos_y, title_ptr);
	}

	display_base->tft->setTextColor(ILI9341_MIDNIGHTBLUE, DISPLAY_COLOR_BG);
	display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
	display_base->tft->setTextScale(1);

	display_base->tft->cursorToXY(pos_x, pos_y + 15);

	char value_str[16] = {0};
	sprintf(value_str, "%d/%d", value, max);

	display_base->tft->print(value_str);

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_BoxValue(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, double value, int dots)
{
	display_base->tft->setTextColor(ILI9341_MIDNIGHTBLUE, DISPLAY_COLOR_BG);
	display_base->tft->setFont(DISPLAY_FONT_BOX_VALUE);
	display_base->tft->setTextScale(1);

	display_base->tft->cursorToXY(pos_x, pos_y);
	display_base->tft->print(value, dots);

	// char value_str[16] = {0};
	// sprintf(value_str, "%02.01f", value);
	// pos_x = BrbDisplayBase_PrintAlignStr(display_base, value_str, pos_x, pos_y, DISPLAY_TEXT_ALIGN_BL);

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_BoxUnit(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, const char *unit_ptr)
{
	// display_base->tft->setTextColor(ILI9341_MEDIUMAQUAMARINE, DISPLAY_COLOR_BG);
	display_base->tft->setFont(DISPLAY_FONT_BOX_SUB);
	display_base->tft->setTextScale(1);

	display_base->tft->cursorToXY(display_base->tft->getCursorX() + 5, pos_y);
	display_base->tft->println((const __FlashStringHelper *)unit_ptr);

	// pos_x = BrbDisplayBase_PrintAlignFlash(display_base, unit_ptr, display_base->tft->getCursorX() + 5, pos_y, DISPLAY_TEXT_ALIGN_BL);

	return 0;
}
/**********************************************************************************************************************/