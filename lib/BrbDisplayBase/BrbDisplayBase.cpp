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

static unsigned int rainbow(byte value);

/**********************************************************************************************************************/
int BrbDisplayBase_Init(BrbDisplayBase *display_base)
{
	/* Sanitize */
	if (!display_base)
		return -1;

	display_base->tft->begin();
	// display_base->tft->reset();
	display_base->tft->setRotation(BRB_DISPLAY_ROTATION_DEFAULT);
	display_base->tft->setFont(BRB_DISPLAY_FONT_DEFAULT);

	display_base->screen_cur = DISPLAY_SCREEN_INFO;
	display_base->screen_last = -1;
	display_base->action_code = 0;
	display_base->flags.on_action = 0;

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

	/* TODO, jump to correct interface */
	if (display_base->screen_cur < DISPLAY_SCREEN_INFO)
		display_base->screen_cur = DISPLAY_SCREEN_LASTITEM - 1;
	else if (display_base->screen_cur >= DISPLAY_SCREEN_LASTITEM)
		display_base->screen_cur = DISPLAY_SCREEN_INFO;

	display_base->action_code = action_code;

	if (display_base->flags.on_action)
	{
		/* Ignore Action */
		if (!display_base->cb_action[display_base->screen_cur].cb_func)
			return 0;

		op_status = display_base->cb_action[display_base->screen_cur].cb_func(display_base->brb_base, display_base);

		display_base->screen_last = display_base->screen_cur;

		return op_status;
	}
	
	/* Press direction */
    if (action_code == BRB_BTN_NEXT)
		display_base->screen_cur++;
	else if (action_code == BRB_BTN_PREV)
		display_base->screen_cur--;

	/* Correct interface number */
	if (display_base->screen_cur < DISPLAY_SCREEN_INFO)
		display_base->screen_cur = DISPLAY_SCREEN_LASTITEM - 1;
	else if (display_base->screen_cur >= DISPLAY_SCREEN_LASTITEM)
		display_base->screen_cur = DISPLAY_SCREEN_INFO;

	/* Ignore Screen */
	if (!display_base->cb_show[display_base->screen_cur].cb_func)
		return 0;

	op_status = display_base->cb_show[display_base->screen_cur].cb_func(display_base->brb_base, display_base);

	display_base->screen_last = display_base->screen_cur;
	
	return op_status;
}
 /**********************************************************************************************************************/
int BrbDisplayBase_SetTitle(BrbDisplayBase *display_base, const __FlashStringHelper *title_str, int x, int y)
{
	display_base->tft->setFont(BRB_DISPLAY_FONT_TITLE);
	display_base->tft->fillRect(x, y, 300, 30, ILI9341_BLUE);
	display_base->tft->cursorToXY(x + 10, y + 10);
	display_base->tft->setTextColor(ILI9341_WHITE, ILI9341_BLUE);
	display_base->tft->setTextScale(1);
	display_base->tft->println(title_str);
	// display_base->tft->printAtPivoted(title_str, x + 10, y + 10, gTextPivotBottomCenter); // Value in middle

	display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);

	return 0;
}
 /**********************************************************************************************************************/
int BrbDisplayBase_SetBg(BrbDisplayBase *display_base)
{
	display_base->tft->fillRect(0, 0, 320, 240, ILI9341_DIMGRAY);
	display_base->tft->fillRect(2, 2, 316, 236, ILI9341_WHITE);

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_SetScreenShowCB(BrbDisplayBase *display_base, int screen_code, BrbGenericCBH *cb_func, void *cb_data)
{
	if (screen_code < 0 || screen_code >= DISPLAY_SCREEN_LASTITEM)
		return -1;

	display_base->cb_show[screen_code].cb_func = cb_func;
	display_base->cb_show[screen_code].cb_data = cb_data;

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_SetScreenActionCB(BrbDisplayBase *display_base, int screen_code, BrbGenericCBH *cb_func, void *cb_data)
{
	if (screen_code < 0 || screen_code >= DISPLAY_SCREEN_LASTITEM)
		return -1;

	display_base->cb_action[screen_code].cb_func = cb_func;
	display_base->cb_action[screen_code].cb_data = cb_data;

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
int BrbDisplayBase_DrawArcText(BrbDisplayBase *display_base, double value, int x, int y, int r, const __FlashStringHelper *units, int text_color)
{
	// Convert value to a string
	char buf[10] = {0};
	byte len = 4;

	if (value > 999)
		len = 5;

	dtostrf(value, len, 1, buf);

	// Set the text color to default
	display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
	display_base->tft->setTextColor(text_color, ILI9341_WHITE);

	display_base->tft->setFont(BRB_DISPLAY_FONT_VALUE);

	// Print value, if the meter is large then use big font 6, othewise use 4
	if (r > 84)
	{
		display_base->tft->setTextScale(3);
		display_base->tft->printAtPivoted(buf, x, y - 25, gTextPivotMiddleCenter); // Value in middle
	}
	else
	{
		display_base->tft->setTextScale(2);
		display_base->tft->printAtPivoted(buf, x, y, gTextPivotMiddleCenter); // Value in middle
	}

	if (units)
	{
		display_base->tft->setFont(BRB_DISPLAY_FONT_SUB);

		if (r > 84)
		{
			display_base->tft->setTextScale(2);
			display_base->tft->printAtPivoted(units, x, y + 15, gTextPivotMiddleCenter); // Units display
		}
		else
		{
			display_base->tft->setTextScale(1);
			display_base->tft->printAtPivoted(units, x, y + 20, gTextPivotMiddleCenter); // Units display
		}
	}

	return 0;
}
 /**********************************************************************************************************************/
int BrbDisplayBase_DrawArc(BrbDisplayBase *display_base, double value, int vmin, int vmax, int x, int y, int r, const __FlashStringHelper *units, byte scheme)
{
	// Minimum value of r is about 52 before value text intrudes on ring
	// drawing the text first is an option

	x += r;
	y += r; // Calculate coords of centre of ring

	// Width of outer ring is 1/x of radius
	int w = r / 6;

	int angle = 95; // Half the sweep angle of meter (300 degrees)

	int text_color = ILI9341_BLACK; // To hold the text color

	int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v

	byte seg = 5; // Segments are 5 degrees wide = 60 segments for 300 degrees
	byte inc = 5; // Draw segments every 5 degrees, increase to 10 for segmented ring

	int tl;
	int i;
	float sx;
	float sy;
	uint16_t x0;
	uint16_t y0;
	uint16_t x1;
	uint16_t y1;
	float sx2;
	float sy2;
	int x2;
	int y2;
	int x3;
	int y3;

	display_base->tft->fillRect(x - r, y - r, r * 2, (r * 1.5), ILI9341_WHITE);

	// Draw color blocks every inc degrees
	for (i = -angle; i < angle; i += inc)
	{
		// Choose color from scheme
		int color_cur = 0;
		switch (scheme)
		{
		case ARC_SCHEME_RED2RED:
			color_cur = ILI9341_RED;
			break; // Fixed color
		case ARC_SCHEME_GREEN2GREEN:
			color_cur = ILI9341_GREEN;
			break; // Fixed color
		case ARC_SCHEME_BLUE2BLUE:
			color_cur = ILI9341_BLUE;
			break; // Fixed color
		case ARC_SCHEME_BLUE2RED:
			color_cur = rainbow(map(i, -angle, angle, 0, 127));
			break; // Full spectrum blue to red
		case ARC_SCHEME_GREEN2RED:
			color_cur = rainbow(map(i, -angle, angle, 63, 127));
			break; // Green to red (high temperature etc)
		case ARC_SCHEME_RED2GREEN:
			color_cur = rainbow(map(i, -angle, angle, 127, 63));
			break; // Red to green (low battery etc)
		default:
			color_cur = ILI9341_BLUE;
			break; // Fixed color
		}

		// Calculate pair of coordinates for segment start
		sx = cos((i - 90) * 0.0174532925);
		sy = sin((i - 90) * 0.0174532925);
		x0 = sx * (r - w) + x;
		y0 = sy * (r - w) + y;
		x1 = sx * r + x;
		y1 = sy * r + y;

		// Calculate pair of coordinates for segment end
		sx2 = cos((i + seg - 90) * 0.0174532925);
		sy2 = sin((i + seg - 90) * 0.0174532925);
		x2 = sx2 * (r - w) + x;
		y2 = sy2 * (r - w) + y;
		x3 = sx2 * r + x;
		y3 = sy2 * r + y;

		// Fill in colored segments with 2 triangles
		if (i < v)
		{
			display_base->tft->fillTriangle(x0, y0, x1, y1, x2, y2, color_cur);
			display_base->tft->fillTriangle(x1, y1, x2, y2, x3, y3, color_cur);
			// Save the last color drawn
			text_color = color_cur;
		}
		else // Fill in blank segments
		{
			display_base->tft->fillTriangle(x0, y0, x1, y1, x2, y2, ILI9341_GRAY);
			display_base->tft->fillTriangle(x1, y1, x2, y2, x3, y3, ILI9341_GRAY);
		}

		// Short scale tick length
		if (i % 25 != 0)
			tl = 6;
		else
			tl = 13;

		// Recalculate coords incase tick lenght changed
		x0 = sx * (r + tl) + x;
		y0 = sy * (r + tl) + y;
		x1 = sx * (r + 2) + x;
		y1 = sy * (r + 2) + y;

		// Draw tick
		display_base->tft->drawLine(x0, y0, x1, y1, ILI9341_BLACK);

		// // Check if labels should be drawn, with position tweaks
		// if (i % 25 == 0)
		// {
		//  display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
		// 	// Calculate label positions
		// 	// x0 = sx * (r + tl) + x;
		// 	// y0 = sy * (r + tl) + y;
		// 	switch (i / 25)
		// 	{
		// 	case -2:
		// 		display_base->tft->printAtPivoted("0", x0, y0 - 12, gTextPivotMiddleCenter);
		// 		break;
		// 	case -1:
		// 		display_base->tft->printAtPivoted("25", x0, y0 - 9, gTextPivotMiddleCenter);
		// 		break;
		// 	case 0:
		// 		display_base->tft->printAtPivoted("50", x0, y0 - 6, gTextPivotMiddleCenter);
		// 		break;
		// 	case 1:
		// 		display_base->tft->printAtPivoted("75", x0, y0 - 9, gTextPivotMiddleCenter);
		// 		break;
		// 	case 2:
		// 		display_base->tft->printAtPivoted("100", x0, y0 - 12, gTextPivotMiddleCenter);
		// 		break;
		// 	}
		// }

		// if (i == angle - inc)
		// {
		// }
	}

	// Draw scale arc, don't draw the last part
	sx = cos((i - 90) * 0.0174532925);
	sy = sin((i - 90) * 0.0174532925);
	x0 = sx * (r + tl) + x;
	y0 = sy * (r + tl) + y;
	x1 = sx * (r + 2) + x;
	y1 = sy * (r + 2) + y;

	display_base->tft->drawLine(x0, y0, x1, y1, ILI9341_BLACK);

	BrbDisplayBase_DrawArcText(display_base, value, x, y, r, units, text_color);
	
	display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
	display_base->tft->setFont(BRB_DISPLAY_FONT_DEFAULT);
	display_base->tft->setTextScale(1);
	display_base->tft->cursorToXY(x - r - 5, y + 10);
	display_base->tft->println(vmin);

	display_base->tft->cursorToXY(x + r - 15, y + 10);
	display_base->tft->println(vmax);

	// Calculate and return right hand side x coordinate
	return x + r;
}
 /**********************************************************************************************************************/
/* Return a 16 bit rainbow color */
 /**********************************************************************************************************************/
static unsigned int rainbow(byte value)
{
	// Value is expected to be in range 0-127
	// The value is converted to a spectrum color from 0 = blue through to 127 = red

	// Red is the top 5 bits of a 16 bit color value
	byte red = 0;
	// Green is the middle 6 bits
	byte green = 0;
	// Blue is the bottom 5 bits
	byte blue = 0;

	byte quadrant = value / 32;

	if (quadrant == 0)
	{
		blue = 31;
		green = 2 * (value % 32);
		red = 0;
	}
	if (quadrant == 1)
	{
		blue = 31 - (value % 32);
		green = 63;
		red = 0;
	}
	if (quadrant == 2)
	{
		blue = 0;
		green = 63;
		red = value % 32;
	}
	if (quadrant == 3)
	{
		blue = 0;
		green = 63 - 2 * (value % 32);
		red = 31;
	}
	return (red << 11) + (green << 5) + blue;
}
/**********************************************************************************************************************/