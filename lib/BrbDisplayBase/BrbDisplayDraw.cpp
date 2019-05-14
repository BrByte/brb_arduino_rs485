/*
 * BrbDisplayDraw.cpp
 *
 *  Created on: 2019-02-18
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
/**********************************************************************************************************************/
int BrbDisplayBase_DrawBtn(BrbDisplayBase *display_base, int btn_x, int btn_y, int btn_w, int btn_h, const char *text_ptr, int btn_color, int txt_color)
{
	display_base->tft->fillRect(btn_x, btn_y, btn_w, btn_h, btn_color);
	display_base->tft->setTextColor(txt_color, btn_color);
	display_base->tft->printAtPivoted((const __FlashStringHelper *)text_ptr, (btn_w / 2) + btn_x, (btn_h / 2) + btn_y, gTextPivotMiddleCenter);

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_DrawArcText(BrbDisplayBase *display_base, double value, int x, int y, int r, const char *units, int text_color)
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

	display_base->tft->setFont(DISPLAY_FONT_ARC_VALUE);

	/* Check size and spacing */
	if (r > 130)
	{
		display_base->tft->setTextScale(2);
		display_base->tft->printAtPivoted(buf, x, y - 30, gTextPivotMiddleCenter);
	}
	else if (r > 84)
	{
		display_base->tft->setTextScale(1);
		display_base->tft->printAtPivoted(buf, x, y - 25, gTextPivotMiddleCenter);
	}
	else if (r > 50)
	{
		display_base->tft->setTextScale(1);
		display_base->tft->printAtPivoted(buf, x, y - 15, gTextPivotMiddleCenter);
	}
	else
	{
		display_base->tft->setFont(DISPLAY_FONT_ARC_SUB);
		display_base->tft->setTextScale(2);
		display_base->tft->printAtPivoted(buf, x, y - 10, gTextPivotMiddleCenter);
	}

	if (units)
	{
		display_base->tft->setFont(DISPLAY_FONT_ARC_SUB);

		if (r > 130)
		{
			display_base->tft->setTextScale(3);
			display_base->tft->printAtPivoted((const __FlashStringHelper *)units, x, y + 15, gTextPivotMiddleCenter);
		}
		else if (r > 84)
		{
			display_base->tft->setTextScale(2);
			display_base->tft->printAtPivoted((const __FlashStringHelper *)units, x, y + 15, gTextPivotMiddleCenter);
		}
		else if (r > 50)
		{
			display_base->tft->setTextScale(1);
			display_base->tft->printAtPivoted((const __FlashStringHelper *)units, x, y + 12, gTextPivotMiddleCenter);
		}
		else
		{
			display_base->tft->setTextScale(1);
			display_base->tft->printAtPivoted((const __FlashStringHelper *)units, x, y + 5, gTextPivotMiddleCenter);
		}
	}

	return 0;
}
/**********************************************************************************************************************/
int BrbDisplayBase_DrawArc(BrbDisplayBase *display_base, double value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme)
{
	return BrbDisplayBase_DrawArcSeg(display_base, value, vmin, vmax, x, y, r, units, scheme, 0, ARC_SEG, ARC_INC);
}
/**********************************************************************************************************************/
int BrbDisplayBase_DrawArcSeg(BrbDisplayBase *display_base, double value, int vmin, int vmax, int x, int y, int r, const char *units, byte scheme, int tick, int seg, int inc)
{
	/* Calculate coords of centre of ring */
	x += r;
	y += r;

	/* Width of outer ring is 1/x of radius */
	int w = r / 6;

	int angle = 95; // Half the sweep angle of meter (300 degrees)

	int text_color = ILI9341_BLACK; // To hold the text color

	int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v

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

	display_base->tft->fillRect(x - r, y - r, r * 2, (r * 1), ILI9341_WHITE);

	// Draw color blocks every inc degrees
	for (i = -angle; i < angle; i += inc)
	{
		// Choose color from scheme
		int color_cur = 0;
		switch (scheme)
		{
		case DISPLAY_ARC_RED2RED:
			color_cur = ILI9341_RED;
			break; // Fixed color
		case DISPLAY_ARC_GREEN2GREEN:
			color_cur = ILI9341_GREEN;
			break; // Fixed color
		case DISPLAY_ARC_BLUE2BLUE:
			color_cur = ILI9341_BLUE;
			break; // Fixed color
		case DISPLAY_ARC_BLUE2RED:
			color_cur = BrbDisplayBase_Rainbow(map(i, -angle, angle, 0, 127));
			break; // Full spectrum blue to red
		case DISPLAY_ARC_GREEN2RED:
			color_cur = BrbDisplayBase_Rainbow(map(i, -angle, angle, 63, 127));
			color_cur = color_lt(color_cur, 0.9);
			break; // Green to red (high temperature etc)
		case DISPLAY_ARC_RED2GREEN:
			color_cur = BrbDisplayBase_Rainbow(map(i, -angle, angle, 127, 63));
			color_cur = color_lt(color_cur, 0.9);
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
			display_base->tft->fillTriangle(x0, y0, x1, y1, x2, y2, ILI9341_LIGHTGREY);
			display_base->tft->fillTriangle(x1, y1, x2, y2, x3, y3, ILI9341_LIGHTGREY);
		}

		if (tick > 0)
		{
			/* Short scale tick length */
			if (i % 25 != 0)
				tl = tick;
			else
				tl = tick + 3;

			// Recalculate coords in case tick length changed
			x0 = sx * (r + tl) + x;
			y0 = sy * (r + tl) + y;
			x1 = sx * (r + 2) + x;
			y1 = sy * (r + 2) + y;

			// Draw tick
			display_base->tft->drawLine(x0, y0, x1, y1, ILI9341_BLACK);
		}

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
	}

	if (tick > 0)
	{
		// Draw scale arc, don't draw the last part
		sx = cos((i - 90) * 0.0174532925);
		sy = sin((i - 90) * 0.0174532925);
		x0 = sx * (r + tl) + x;
		y0 = sy * (r + tl) + y;
		x1 = sx * (r + 2) + x;
		y1 = sy * (r + 2) + y;

		display_base->tft->drawLine(x0, y0, x1, y1, ILI9341_BLACK);
	}

	BrbDisplayBase_DrawArcText(display_base, value, x, y, r, units, text_color);

	display_base->tft->setTextColor(ILI9341_BLACK, ILI9341_WHITE);
	display_base->tft->setFont(DISPLAY_FONT_DEFAULT);
	display_base->tft->setTextScale(1);
	display_base->tft->cursorToXY(x - r, y + 10);
	display_base->tft->println(vmin);

	display_base->tft->cursorToXY(x + r - (vmax > 99 ? 20 : 15), y + 10);
	display_base->tft->println(vmax);

	// Calculate and return right hand side x coordinate
	return x + r;
}
/**********************************************************************************************************************/
static unsigned int color_lt(unsigned int color, float pct)
{
	byte r = (color >> 8) & 0x00F8;
	byte g = (color >> 3) & 0x00FC;
	byte b = (color << 3) & 0x00F8;

	r = min(max(0, r * pct), 255);
	g = min(max(0, g * pct), 255);
	b = min(max(0, b * pct), 255);

	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
	// return (r << 11) + (g << 5) + b;
}
/**********************************************************************************************************************/
/* Return a 16 bit rainbow color */
/**********************************************************************************************************************/
unsigned int BrbDisplayBase_Rainbow(byte value)
{
	/* Value is expected to be in range 0-127 */

	/* The value is converted to a spectrum color from 0 = blue through to 127 = red */

	/* RRRRRGGGGGGBBBBB */

	/* Red is the top 5 bits of a 16 bit color value */
	byte r = 0;
	/* Green is the middle 6 bits */
	byte g = 0;
	/* Blue is the bottom 5 bits */
	byte b = 0;

	byte q = value / 32;

	if (q == 0)
	{
		b = 31;
		g = 2 * (value % 32);
		r = 0;
	}
	else if (q == 1)
	{
		b = 31 - (value % 32);
		g = 63;
		r = 0;
	}
	else if (q == 2)
	{
		b = 0;
		g = 63;
		r = value % 32;
	}
	else if (q == 3)
	{
		b = 0;
		g = 63 - 2 * (value % 32);
		r = 31;
	}
	return (r << 11) + (g << 5) + b;
}
/**********************************************************************************************************************/
int BrbDisplayBase_DrawBarGraph(BrbDisplayBase *display_base, int16_t pos_x, int16_t pos_y, int16_t tube_h, double value, double min, double max)
{
	static int tube_h_old = 0;
	static double tube_h_max = 0;

	int tune_diff;
	int tube_h_n;

	int tube_w = 21;

	// if (pos_y > 0)
	// {
	// 	tube_w = 22;
	// }

	int tube_r = tube_w / 2;
	int color = ILI9341_RED;

	if (value < 0)
	{
		color = ILI9341_SLATEBLUE;
		value = value * -1;
		max = min < 0 ? (min * -1) : min;
	}

	pos_x = pos_x + tube_w;
	pos_y = pos_y + tube_w;

	if (display_base->screen_cur != display_base->screen_last)
	{
		display_base->tft->fillCircle(pos_x, pos_y, (tube_w / 2), ILI9341_BLACK);
		display_base->tft->fillRect((pos_x - (tube_w / 2)), pos_y, tube_w, tube_h, ILI9341_BLACK);
		display_base->tft->fillCircle(pos_x, (tube_h + pos_y) + 7, ((tube_w / 2) + 10), ILI9341_BLACK);

		display_base->tft->fillCircle(pos_x, pos_y, (((tube_w - 6) / 2)), ILI9341_WHITESMOKE);
		display_base->tft->fillRect((pos_x - (tube_w / 2) + 3), pos_y, (tube_w - 6), tube_h, ILI9341_WHITESMOKE);
		display_base->tft->fillCircle(pos_x, (tube_h + pos_y) + 7, ((tube_w / 2) + 7), ILI9341_WHITESMOKE);

		tube_h_old = 0;
	}

	// tune_diff = tube_h_max - max; // only draw new part of bar graph for faster display

	// display_base->tft->fillRect((pos_x - (tube_w / 2) + 5), pos_y, (tube_w - 10), tube_h, color);
	display_base->tft->fillCircle(pos_x, (tube_h + pos_y) + 7, ((tube_w / 2) + 5), color);

	tube_h_n = map(value, 0, max, 0, (tube_h - tube_r));

	tune_diff = tube_h_old - tube_h_n; // only draw new part of bar graph for faster display

	if (tune_diff != 0)
	{
		display_base->tft->setFont(DISPLAY_FONT_BOX_SUB);
		display_base->tft->setTextScale(1);
		display_base->tft->setTextColor(ILI9341_BLACK);

		int max_int = (int)(max - ((int)max % 25));

		for (int i = 0; i <= max_int; i = i + 5)
		{
			int map_pct = 100.0 - map(i, 0, max_int, 0, 100);
			double pct = (map_pct / 100.0);

			if (map_pct % 25 == 0)
			{
				display_base->tft->fillRect(pos_x + (tube_w / 2) + 1, (pos_y + (tube_h - tube_r) * pct), 5, 2, ILI9341_BLACK);
				display_base->tft->cursorToXY(((pos_x + (tube_w / 2)) + 10), (pos_y + (tube_h - tube_r) * pct) - 5);
				// display_base->tft->cursorToXY(((pos_x + tube_w) + 8), (pos_y - 5));
				display_base->tft->print(i);
				// display_base->tft->setTextSize(2);
				display_base->tft->print((char)247);
			}
			// else if (((int)(pct * 100) % 50) == 0)
			// {
			// 	display_base->tft->fillRect(pos_x + (tube_w / 2) + 1, (pos_y + (tube_h - tube_r) * pct), 3, 2, ILI9341_BLACK);
			// }
			else
			{
				display_base->tft->fillRect(pos_x + (tube_w / 2) + 1, (pos_y + (tube_h - tube_r) * pct), 3, 1, ILI9341_BLACK);
			}
		}

		display_base->tft->fillRect((pos_x - (tube_w / 2) + 5), pos_y + (tube_h - tube_h_n), (tube_w - 10), tube_h_n, color);

		display_base->tft->fillRect(pos_x + (tube_w / 2) + 5, pos_y + (tube_h - tube_h_n), 30, 1, color);
		display_base->tft->cursorToXY(pos_x + (tube_w / 2) + 10, pos_y + (tube_h - tube_h_n) + 3);
		display_base->tft->setTextColor(color);
		display_base->tft->print(value, 1);
	}

	// display_base->tft->fillRect(pos_x, (pos_y + (tube_h - tube_r) * tube_h_n), 300, 6, ILI9341_SALMON);

	// display_base->tft->fillRect((pos_x - (tube_w / 2) + 5), pos_y, (tube_w - 10), ((tube_h - value) - ((tube_w / 2) + 7)), ILI9341_WHITE);

	if (value > max)
	{
		display_base->tft->fillCircle(pos_x, pos_y, (tube_w / 2) - 5, color);
	}

	// remember how high bar is
	tube_h_old = tube_h_n;

	// display_base->tft->fillCircle(pos_x, (tube_h + pos_y), ((tube_w / 2) + 5), color);

	return 0;
}
/**********************************************************************************************************************/