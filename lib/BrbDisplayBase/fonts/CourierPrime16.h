

/*
 *
 * CourierPrime16
 *
 * created with FontCreator
 * written by F. Maximilian Thiele
 *
 * http://www.apetech.de/fontCreator
 * me@apetech.de
 *
 * File Name           : CourierPrime16.h
 * Date                : 27.01.2019
 * Font size in bytes  : 9398
 * Font width          : 10
 * Font height         : 14
 * Font first char     : 32
 * Font last char      : 128
 * Font used chars     : 96
 *
 * The font data are defined as
 *
 * struct _FONT_ {
 *     uint16_t   font_Size_in_Bytes_over_all_included_Size_it_self;
 *     uint8_t    font_Width_in_Pixel_for_fixed_drawing;
 *     uint8_t    font_Height_in_Pixel_for_all_characters;
 *     unit8_t    font_First_Char;
 *     uint8_t    font_Char_Count;
 *
 *     uint8_t    font_Char_Widths[font_Last_Char - font_First_Char +1];
 *                  // for each character the separate width in pixels,
 *                  // characters < 128 have an implicit virtual right empty row
 *
 *     uint8_t    font_data[];
 *                  // bit field of all characters
 */

#include <inttypes.h>
#include <avr/pgmspace.h>

#ifndef COURIERPRIME16_H
#define COURIERPRIME16_H

#define COURIERPRIME16_WIDTH 10
#define COURIERPRIME16_HEIGHT 14

static const uint8_t CourierPrime16[] PROGMEM = {
    0x24, 0xB6, // size
    0x0A, // width
    0x0E, // height
    0x20, // first char
    0x60, // char count
    
    // char widths
    0x00, 0x02, 0x05, 0x09, 0x06, 0x09, 0x08, 0x02, 0x04, 0x04, 
    0x07, 0x07, 0x03, 0x07, 0x02, 0x06, 0x08, 0x06, 0x07, 0x06, 
    0x07, 0x06, 0x06, 0x06, 0x06, 0x06, 0x02, 0x03, 0x07, 0x07, 
    0x08, 0x06, 0x09, 0x0A, 0x07, 0x08, 0x08, 0x08, 0x09, 0x08, 
    0x08, 0x08, 0x08, 0x08, 0x09, 0x09, 0x08, 0x08, 0x09, 0x08, 
    0x08, 0x08, 0x08, 0x08, 0x0A, 0x0A, 0x09, 0x09, 0x08, 0x04, 
    0x06, 0x04, 0x06, 0x0A, 0x05, 0x08, 0x07, 0x08, 0x08, 0x08, 
    0x07, 0x08, 0x08, 0x07, 0x05, 0x08, 0x07, 0x0A, 0x08, 0x08, 
    0x08, 0x08, 0x07, 0x06, 0x07, 0x08, 0x0A, 0x0A, 0x09, 0x0A, 
    0x07, 0x05, 0x01, 0x05, 0x07, 0x00, 
    
    // font data
    0xFC, 0x0C, 0x18, 0x18, // 33
    0x04, 0x3C, 0x00, 0x0C, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, // 34
    0x80, 0x90, 0x90, 0xF0, 0x9E, 0x90, 0xF8, 0x9E, 0x90, 0x00, 0x00, 0x1C, 0x00, 0x10, 0x0C, 0x00, 0x00, 0x00, // 35
    0x38, 0x64, 0xFE, 0xE4, 0xC8, 0xCC, 0x18, 0x10, 0x3C, 0x10, 0x10, 0x0C, // 36
    0x18, 0x24, 0x24, 0xA4, 0x58, 0x20, 0x90, 0x98, 0x08, 0x00, 0x08, 0x04, 0x00, 0x00, 0x0C, 0x10, 0x10, 0x0C, // 37
    0x80, 0x78, 0xC4, 0x84, 0x04, 0x8C, 0x40, 0x40, 0x0C, 0x10, 0x10, 0x1C, 0x1C, 0x0C, 0x10, 0x10, // 38
    0x3C, 0x0C, 0x00, 0x00, // 39
    0xF0, 0x0C, 0x06, 0x02, 0x1C, 0x60, 0xC0, 0x80, // 40
    0x02, 0x06, 0x0C, 0xF0, 0x80, 0xC0, 0x60, 0x1C, // 41
    0x20, 0x90, 0xF0, 0x7C, 0x60, 0x90, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 42
    0x40, 0x40, 0x40, 0xF8, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, // 43
    0x00, 0x00, 0x00, 0x70, 0x38, 0x08, // 44
    0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 45
    0x00, 0x00, 0x18, 0x18, // 46
    0x00, 0x00, 0xC0, 0x70, 0x1C, 0x06, 0x60, 0x1C, 0x04, 0x00, 0x00, 0x00, // 47
    0xF0, 0x08, 0x04, 0x04, 0x04, 0x04, 0x08, 0xF0, 0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, // 48
    0x08, 0x08, 0x04, 0xFC, 0x00, 0x00, 0x10, 0x10, 0x10, 0x1C, 0x10, 0x10, // 49
    0x00, 0x0C, 0x04, 0x84, 0xC4, 0x78, 0x00, 0x10, 0x18, 0x1C, 0x14, 0x10, 0x10, 0x18, // 50
    0x0C, 0x04, 0x44, 0x44, 0x44, 0xB8, 0x08, 0x10, 0x10, 0x10, 0x10, 0x0C, // 51
    0xC0, 0xE0, 0x90, 0x88, 0xFC, 0x80, 0x80, 0x00, 0x00, 0x10, 0x10, 0x1C, 0x10, 0x10, // 52
    0x7C, 0x24, 0x24, 0x24, 0x24, 0xC0, 0x08, 0x10, 0x10, 0x10, 0x10, 0x0C, // 53
    0xE0, 0x38, 0x28, 0x24, 0x24, 0xC4, 0x0C, 0x10, 0x10, 0x10, 0x10, 0x0C, // 54
    0x0C, 0x04, 0x04, 0xC4, 0x3C, 0x0C, 0x00, 0x10, 0x1C, 0x04, 0x00, 0x00, // 55
    0xB8, 0x44, 0x44, 0x44, 0xC4, 0xB8, 0x0C, 0x10, 0x10, 0x10, 0x10, 0x0C, // 56
    0x38, 0x44, 0x44, 0x44, 0x4C, 0xF0, 0x10, 0x10, 0x10, 0x08, 0x0C, 0x00, // 57
    0x30, 0x30, 0x18, 0x18, // 58
    0x00, 0x30, 0x30, 0x70, 0x38, 0x08, // 59
    0x40, 0xE0, 0xA0, 0xB0, 0x10, 0x18, 0x08, 0x00, 0x00, 0x00, 0x04, 0x04, 0x0C, 0x08, // 60
    0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, // 61
    0x08, 0x08, 0x10, 0x10, 0xA0, 0xA0, 0x40, 0x40, 0x08, 0x08, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, // 62
    0x0C, 0x04, 0xC4, 0x44, 0x24, 0x38, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, // 63
    0xC0, 0x30, 0x10, 0xC8, 0x28, 0xE8, 0x28, 0x10, 0xE0, 0x0C, 0x10, 0x20, 0x2C, 0x28, 0x24, 0x08, 0x08, 0x04, // 64
    0x00, 0x00, 0xC4, 0xB4, 0x8C, 0x9C, 0xF0, 0xC0, 0x00, 0x00, 0x10, 0x18, 0x14, 0x00, 0x00, 0x00, 0x10, 0x14, 0x18, 0x10, // 65
    0x04, 0xFC, 0x44, 0x44, 0x44, 0x44, 0xB8, 0x10, 0x1C, 0x10, 0x10, 0x10, 0x10, 0x0C, // 66
    0xF0, 0x08, 0x04, 0x04, 0x04, 0x04, 0x0C, 0x1C, 0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x08, // 67
    0x04, 0xFC, 0x04, 0x04, 0x04, 0x04, 0x08, 0xF0, 0x10, 0x1C, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, // 68
    0x04, 0xFC, 0x44, 0x44, 0xE4, 0x04, 0x0C, 0x00, 0x10, 0x1C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1C, // 69
    0x04, 0x04, 0xFC, 0x44, 0x44, 0xE4, 0x04, 0x04, 0x1C, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, // 70
    0xF0, 0x08, 0x04, 0x04, 0x84, 0x84, 0x9C, 0x80, 0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x0C, 0x00, // 71
    0x04, 0xFC, 0x44, 0x40, 0x40, 0x44, 0xFC, 0x04, 0x10, 0x1C, 0x10, 0x00, 0x00, 0x10, 0x1C, 0x10, // 72
    0x04, 0x04, 0x04, 0xFC, 0x04, 0x04, 0x00, 0x00, 0x10, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x10, 0x10, // 73
    0x80, 0x00, 0x04, 0x04, 0x04, 0xFC, 0x04, 0x04, 0x0C, 0x10, 0x10, 0x10, 0x10, 0x0C, 0x00, 0x00, // 74
    0x04, 0xFC, 0x44, 0x60, 0x94, 0x0C, 0x04, 0x04, 0x10, 0x1C, 0x10, 0x00, 0x04, 0x1C, 0x10, 0x00, // 75
    0x04, 0x04, 0xFC, 0x04, 0x04, 0x00, 0x00, 0x00, 0x80, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1C, // 76
    0x04, 0xFC, 0x1C, 0x70, 0x80, 0x70, 0x1C, 0xFC, 0x04, 0x10, 0x1C, 0x10, 0x00, 0x04, 0x00, 0x10, 0x1C, 0x10, // 77
    0x04, 0xFC, 0x0C, 0x30, 0xC0, 0x04, 0xFC, 0x04, 0x10, 0x1C, 0x10, 0x10, 0x00, 0x0C, 0x1C, 0x00, // 78
    0xF0, 0x08, 0x04, 0x04, 0x04, 0x04, 0x08, 0xF0, 0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, // 79
    0x04, 0x04, 0xFC, 0x84, 0x84, 0x84, 0x84, 0x4C, 0x78, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, // 80
    0xF0, 0x08, 0x04, 0x04, 0x04, 0x04, 0x08, 0xF0, 0x04, 0x48, 0x70, 0x50, 0x50, 0x50, 0x68, 0x44, // 81
    0x04, 0xFC, 0x44, 0x44, 0xC4, 0xC4, 0x38, 0x00, 0x10, 0x1C, 0x10, 0x00, 0x00, 0x04, 0x1C, 0x10, // 82
    0x00, 0x38, 0x24, 0x64, 0x44, 0x44, 0x4C, 0x8C, 0x1C, 0x1C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0C, // 83
    0x3C, 0x04, 0x04, 0xFC, 0x04, 0x04, 0x04, 0x3C, 0x00, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x00, 0x00, // 84
    0x04, 0xFC, 0x04, 0x00, 0x00, 0x04, 0xFC, 0x04, 0x00, 0x0C, 0x10, 0x10, 0x10, 0x10, 0x0C, 0x00, // 85
    0x04, 0x0C, 0x3C, 0xC4, 0x00, 0x00, 0xC4, 0x3C, 0x0C, 0x04, 0x00, 0x00, 0x00, 0x04, 0x1C, 0x1C, 0x04, 0x00, 0x00, 0x00, // 86
    0x04, 0x7C, 0x84, 0x84, 0xF0, 0xF0, 0x80, 0x84, 0x7C, 0x04, 0x00, 0x00, 0x1C, 0x1C, 0x00, 0x00, 0x1C, 0x1C, 0x00, 0x00, // 87
    0x04, 0x04, 0x1C, 0xB0, 0x60, 0xB0, 0x1C, 0x04, 0x04, 0x10, 0x18, 0x14, 0x00, 0x00, 0x00, 0x14, 0x18, 0x10, // 88
    0x04, 0x0C, 0x1C, 0x64, 0xC0, 0x60, 0x1C, 0x0C, 0x04, 0x00, 0x00, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x00, 0x00, // 89
    0x3C, 0x04, 0x84, 0xE4, 0xF4, 0x3C, 0x1C, 0x84, 0x18, 0x1C, 0x1C, 0x14, 0x10, 0x10, 0x10, 0x1C, // 90
    0xFE, 0x02, 0x02, 0x02, 0xFC, 0x80, 0x80, 0x80, // 91
    0x06, 0x38, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x38, 0x60, // 92
    0x02, 0x02, 0x02, 0xFE, 0x80, 0x80, 0x80, 0xFC, // 93
    0xC0, 0x30, 0x0C, 0x18, 0xE0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 94
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, // 95
    0x03, 0x02, 0x02, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, // 96
    0x00, 0xA0, 0x90, 0x90, 0x90, 0x90, 0xE0, 0x00, 0x0C, 0x10, 0x10, 0x10, 0x10, 0x08, 0x1C, 0x10, // 97
    0x02, 0xFE, 0x20, 0x10, 0x10, 0x10, 0xE0, 0x10, 0x1C, 0x08, 0x10, 0x10, 0x18, 0x0C, // 98
    0xC0, 0x20, 0x10, 0x10, 0x10, 0x10, 0x10, 0x70, 0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x10, 0x08, // 99
    0xC0, 0x20, 0x10, 0x10, 0x12, 0x22, 0xFE, 0x00, 0x04, 0x08, 0x10, 0x10, 0x10, 0x08, 0x1C, 0x10, // 100
    0xC0, 0xA0, 0x90, 0x90, 0x90, 0x90, 0xA0, 0xC0, 0x0C, 0x18, 0x10, 0x10, 0x10, 0x10, 0x08, 0x00, // 101
    0x10, 0x10, 0xFC, 0x12, 0x12, 0x12, 0x02, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x10, 0x00, // 102
    0xC0, 0x20, 0x10, 0x10, 0x10, 0x20, 0xF0, 0x10, 0x04, 0x88, 0x90, 0x90, 0x90, 0x88, 0x7C, 0x00, // 103
    0x02, 0xFE, 0x20, 0x10, 0x10, 0x10, 0xE0, 0x00, 0x10, 0x1C, 0x10, 0x00, 0x00, 0x10, 0x1C, 0x10, // 104
    0x10, 0x10, 0x10, 0xF3, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x10, // 105
    0x10, 0x10, 0x10, 0x10, 0xF3, 0xC0, 0x80, 0x80, 0x80, 0x7C, // 106
    0x02, 0xFE, 0x80, 0xC0, 0xD0, 0x30, 0x10, 0x10, 0x10, 0x1C, 0x14, 0x00, 0x14, 0x1C, 0x18, 0x10, // 107
    0x02, 0x02, 0x02, 0xFE, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x10, // 108
    0x10, 0xF0, 0x10, 0x10, 0xF0, 0x20, 0x10, 0x10, 0xE0, 0x00, 0x10, 0x1C, 0x10, 0x00, 0x1C, 0x10, 0x00, 0x00, 0x1C, 0x10, // 109
    0x10, 0xF0, 0x20, 0x10, 0x10, 0x10, 0xE0, 0x00, 0x10, 0x1C, 0x10, 0x00, 0x00, 0x10, 0x1C, 0x10, // 110
    0xC0, 0x20, 0x10, 0x10, 0x10, 0x10, 0x20, 0xC0, 0x04, 0x08, 0x10, 0x10, 0x10, 0x10, 0x08, 0x04, // 111
    0x10, 0xF0, 0x20, 0x10, 0x10, 0x10, 0x20, 0xC0, 0x80, 0xFC, 0x88, 0x90, 0x10, 0x10, 0x08, 0x04, // 112
    0xC0, 0x20, 0x10, 0x10, 0x10, 0x20, 0xF0, 0x10, 0x04, 0x08, 0x10, 0x10, 0x90, 0x88, 0xFC, 0x80, // 113
    0x10, 0xF0, 0x60, 0x20, 0x10, 0x10, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x10, 0x00, 0x00, // 114
    0x60, 0x90, 0x90, 0x90, 0x90, 0x30, 0x18, 0x10, 0x10, 0x10, 0x10, 0x0C, // 115
    0x10, 0x10, 0xFE, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x0C, 0x10, 0x10, 0x10, 0x18, // 116
    0x10, 0xF0, 0x00, 0x00, 0x10, 0x10, 0xF0, 0x00, 0x00, 0x0C, 0x10, 0x10, 0x10, 0x08, 0x1C, 0x10, // 117
    0x10, 0x30, 0xF0, 0x90, 0x00, 0x00, 0x90, 0x70, 0x10, 0x10, 0x00, 0x00, 0x00, 0x0C, 0x18, 0x1C, 0x04, 0x00, 0x00, 0x00, // 118
    0x10, 0xF0, 0x10, 0x00, 0xC0, 0xE0, 0x00, 0x10, 0xF0, 0x10, 0x00, 0x00, 0x1C, 0x1C, 0x04, 0x04, 0x1C, 0x1C, 0x00, 0x00, // 119
    0x10, 0x10, 0x30, 0xC0, 0x80, 0xC0, 0x30, 0x10, 0x10, 0x10, 0x18, 0x1C, 0x04, 0x00, 0x04, 0x1C, 0x18, 0x10, // 120
    0x10, 0x30, 0xF0, 0x90, 0x00, 0x00, 0xD0, 0x70, 0x10, 0x10, 0x00, 0x80, 0x80, 0x44, 0x38, 0x0C, 0x04, 0x00, 0x00, 0x00, // 121
    0x30, 0x10, 0x90, 0xD0, 0x70, 0x30, 0x10, 0x10, 0x1C, 0x1C, 0x14, 0x10, 0x10, 0x18, // 122
    0x80, 0x80, 0x7C, 0x02, 0x02, 0x00, 0x00, 0x7C, 0x80, 0x80, // 123
    0xFE, 0xFC, // 124
    0x02, 0x02, 0x7C, 0x80, 0x80, 0x80, 0x80, 0x7C, 0x00, 0x00, // 125
    0x80, 0x40, 0x40, 0x40, 0x40, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 126
    
};

#endif