#if defined(ARDUINO_ARCH_AVR)
    #include <avr/pgmspace.h>
#elif defined(ARDUINO_SAM_DUE)
    #define PROGMEM
#endif

const uint16_t brbyte_whiteWidth = 120;
const uint16_t brbyte_whiteHeight = 35;
const uint16_t brbyte_white[4200] PROGMEM={
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DE,0xE77D,0xCF3B,0xBED9,0xCF1B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 0, 120 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE79D,0x85D4,0x85D4,0x85D4,0x85D4,0xE77C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 1, 240 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0x9E36,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 2, 360 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEFBD,0x9EB5,0x6E31,0x55CE,0x356B,0x2D4A,0x1D29,0x1508,0x1508,0x2529,0x3D8C,0x5DEF,0x96B5,0xEFBD,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DF,0x8DF4,0x85D4,0x85D4,0x85D4,0xCF1A,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DE,0xFFDF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 3, 480 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x7631,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x0CE7,0x9694,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF1A,0x85D4,0x85D4,0x85D4,0x8DF4,0xFFDF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF7C,0xB6B8,0xA656,0x8E15,0x8DF4,0xF7DF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 4, 600 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DE,0x1508,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x96B5,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9635,0x85D4,0x85D4,0x85D4,0xB6B8,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9E36,0x85D4,0x85D4,0x85D4,0xB6B8,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 5, 720 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9ED6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x1508,0x2529,0x1D29,0x0CE7,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x1508,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF7C,0x85D4,0x85D4,0x85D4,0x85D4,0xEF9D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE77D,0x85D4,0x85D4,0x85D4,0x85D4,0xE79D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 6, 840 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x356B,0x04C6,0x04C6,0x04C6,0x356B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xD77B,0x6610,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0xE7BD,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xAE97,0x85D4,0x85D4,0x85D4,0xA656,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB698,0x85D4,0x85D4,0x85D4,0x9E56,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 7, 960 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF5A,0x04C6,0x04C6,0x04C6,0x04C6,0x9EB5,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x2D4A,0x04C6,0x04C6,0x04C6,0x04C6,0xEFDE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7BE,0x85F4,0x85D4,0x85D4,0x85D4,0xD73B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DE,0x8DF4,0x85D4,0x85D4,0x85D4,0xD73B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 8, 1080 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x5DEF,0x04C6,0x04C6,0x04C6,0x1508,0xF7DE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x45AD,0x04C6,0x04C6,0x04C6,0x2529,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF5A,0x9694,0x6610,0x4DAD,0x3D8C,0x356B,0x4DAD,0x6E31,0xA6D6,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6FA,0x85D4,0x85D4,0x85D4,0x9615,0xEF9D,0xC6FA,0xA677,0x9E56,0xA657,0xBED9,0xEFBE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xD73B,0xD73B,0xD73B,0xD73B,0xE79D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE79D,0xD73B,0xD73B,0xD73B,0xD73B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6FA,0x85D4,0x85D4,0x85D4,0x85F4,0xCF1B,0xD73B,0xD73B,0xD73B,0xD73B,0xD73B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF5C,0xBED9,0xA677,0x9E36,0x9E36,0xA677,0xC6FA,0xF7DF,0xFFFF, // row 9, 1200 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEFDE,0x0CE7,0x04C6,0x04C6,0x04C6,0x7631,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x1508,0x04C6,0x04C6,0x04C6,0x7E52,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF9C,0x55CE,0x0CE7,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x45AD,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9615,0x85D4,0x85D4,0x85D4,0x8DF4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x8DF4,0xCF1B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x85F4,0x85D4,0x85D4,0x85D4,0xCF1A,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9615,0x85D4,0x85D4,0x85D4,0xA677,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9635,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x9E56,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF1A,0x9615,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x9615,0xF7BE, // row 10, 1320 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8E94,0x04C6,0x04C6,0x04C6,0x04C6,0xDF9C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8673,0x04C6,0x04C6,0x04C6,0x1508,0xEFDE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x6E31,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0xCF5A,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xD75C,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85F4,0xE79D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7BE,0x85D4,0x85D4,0x85D4,0x85D4,0xDF5C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xD73B,0x85D4,0x85D4,0x85D4,0x85F4,0xE79D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF5C,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xD73B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEF9D,0x9E36,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xAE97, // row 11, 1440 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x2529,0x04C6,0x04C6,0x04C6,0x45AD,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7FF,0x7652,0x04C6,0x04C6,0x04C6,0x04C6,0xB718,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DE,0x0CE7,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x55CE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xA677,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85F4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xB6B8,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE79D,0x85D4,0x85D4,0x85D4,0x85D4,0xE79D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9635,0x85D4,0x85D4,0x85D4,0xAE98,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xAE77,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x8E15,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xD75C,0x8DF4,0x85D4,0x85D4,0x85D4,0x85D4,0x8DF4,0x9E36,0x9615,0x85D4,0x85D4,0x85D4,0x85D4,0x85F4, // row 12, 1560 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB718,0x04C6,0x04C6,0x04C6,0x04C6,0x7E73,0xC75A,0xC75A,0xC75A,0xBF39,0xA6D6,0x6E31,0x1508,0x04C6,0x04C6,0x04C6,0x0CE7,0xA6D6,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x96B5,0x04C6,0x04C6,0x04C6,0x0CE7,0xBF39,0xE7BD,0xF7DE,0xEFDE,0xD77B,0x96B5,0xD77B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEFBE,0x85F4,0x85D4,0x85D4,0x85D4,0x9635,0xCF1A,0xF7BE,0xFFFF,0xFFDF,0xC6FA,0x85D4,0x85D4,0x85D4,0x85D4,0x9635,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF5C,0x85D4,0x85D4,0x85D4,0x85D4,0xF7DE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xD73B,0x85D4,0x85D4,0x85D4,0x8DF4,0xEFBE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEFBE,0x85F4,0x85D4,0x85D4,0x85D4,0xC6FA,0xD75C,0xD75C,0xD75C,0xD75C,0xD75C,0xE79D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xD75C,0x85F4,0x85D4,0x85D4,0x85D4,0x9E56,0xDF5C,0xFFFF,0xFFFF,0xFFFF,0xC6F9,0x85D4,0x85D4,0x85D4,0x85D4, // row 13, 1680 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x4DCE,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x4DAD,0xD77B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x2D4A,0x04C6,0x04C6,0x04C6,0x6E31,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xBED9,0x85D4,0x85D4,0x85D4,0x9635,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xA657,0x85D4,0x85D4,0x85D4,0x8DF4,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF3B,0x85D4,0x85D4,0x85D4,0x85F4,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9615,0x85D4,0x85D4,0x85D4,0xBED9,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xBED9,0x85D4,0x85D4,0x85D4,0x9615,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE79D,0x8DF4,0x85D4,0x85D4,0x85D4,0xBEB9,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE79D,0x85D4,0x85D4,0x85D4,0x8DF4, // row 14, 1800 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE79C,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x5DEF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC739,0x04C6,0x04C6,0x04C6,0x04C6,0xD77B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8DF5,0x85D4,0x85D4,0x85D4,0xC6FA,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0x9615,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6FA,0x85D4,0x85D4,0x85D4,0x9615,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6FA,0x85D4,0x85D4,0x85D4,0x9615,0xFFDF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9615,0x85D4,0x85D4,0x85D4,0xC6FA,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9635,0x85D4,0x85D4,0x85D4,0xB6B9,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6FA,0x85D4,0x85D4,0x85D4,0xAE77, // row 15, 1920 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x7E52,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x4DAD,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x55EF,0x04C6,0x04C6,0x04C6,0x3D8C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF3B,0x85D4,0x85D4,0x85D4,0x8DF4,0xF7DE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xAE98,0x85D4,0x85D4,0x85D4,0x9E56,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xBED9,0x85D4,0x85D4,0x85D4,0x9E36,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DE,0x8DF4,0x85D4,0x85D4,0x85D4,0xD73B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xD73B,0x85D4,0x85D4,0x85D4,0x8DF4,0xF7DE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xBED9,0x85D4,0x85D4,0x85D4,0x9E56,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE77D,0x8DF4,0x85D4,0x85D4,0x85D4,0xDF5C, // row 16, 2040 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x1508,0x04C6,0x04C6,0x04C6,0x45AD,0x9EB5,0x9EB5,0x9EB5,0x9EB5,0x96B5,0x7652,0x2D4A,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x9ED6,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE7BD,0x04E7,0x04C6,0x04C6,0x04C6,0xA6F7,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9E56,0x85D4,0x85D4,0x85D4,0xB6B8,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9E36,0x85D4,0x85D4,0x85D4,0xB6B8,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xBED9,0x85D4,0x85D4,0x85D4,0xA677,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB698,0x85D4,0x85D4,0x85D4,0xA677,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xA656,0x85D4,0x85D4,0x85D4,0xB698,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DE,0x8DF4,0x85D4,0x85D4,0x85D4,0xE79D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE79D,0xB6B8,0x8DF4,0x85D4,0x85D4,0x85D4,0xAE98,0xFFFF, // row 17, 2160 pixels
0xFFFF,0xFFFF,0xFFFF,0xA6F7,0x04C6,0x04C6,0x04C6,0x04C6,0xC739,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x6610,0x04C6,0x04C6,0x04C6,0x04C6,0x55CE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8673,0x04C6,0x04C6,0x04C6,0x1508,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE79D,0x85D4,0x85D4,0x85D4,0x85D4,0xE79D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7BE,0x85D4,0x85D4,0x85D4,0x85D4,0xDF5C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0xAE97,0xFFFF,0xFFFF,0xFFFF,0xDF7C,0x85D4,0x85D4,0x85D4,0x8DF4,0xEFBE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEF9D,0x85D4,0x85D4,0x85D4,0x85D4,0xE77D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xBED9,0x85D4,0x85D4,0x85D4,0x9615,0xC6FA,0xBED9,0xB6B8,0xA657,0x8DF4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xAE77,0xFFFF,0xFFFF, // row 18, 2280 pixels
0xFFFF,0xFFFF,0xFFFF,0x3D8C,0x04C6,0x04C6,0x04C6,0x2D6B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF9C,0x04C6,0x04C6,0x04C6,0x04C6,0x45AD,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x1D29,0x04C6,0x04C6,0x04C6,0x7E52,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0x9E56,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6FA,0x85D4,0x85D4,0x85D4,0x8DF5,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0xB6B8,0xFFFF,0xFFFF,0xFFFF,0x9635,0x85D4,0x85D4,0x85D4,0xCF1A,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0x9E36,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9615,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x8DF4,0xC71A,0xFFFF,0xFFFF,0xFFFF, // row 19, 2400 pixels
0xFFFF,0xFFFF,0xD77B,0x04C6,0x04C6,0x04C6,0x04C6,0x96B5,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE7BD,0x04C6,0x04C6,0x04C6,0x04C6,0x6610,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xAEF7,0x04C6,0x04C6,0x04C6,0x04C6,0xE79C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DF,0x8DF4,0x85D4,0x85D4,0x85D4,0xCF3B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DF,0x8E15,0x85D4,0x85D4,0x85D4,0xBED9,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB698,0x85D4,0x85D4,0x85D4,0xB6B8,0xFFFF,0xFFFF,0xBED9,0x85D4,0x85D4,0x85D4,0xA656,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8DF4,0x85D4,0x85D4,0x85D4,0xCF1B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE79D,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85F4,0x9E56,0xC6FA,0xF7DF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 20, 2520 pixels
0xFFFF,0xFFFF,0x6E10,0x04C6,0x04C6,0x04C6,0x0CE7,0xF7DE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xAEF7,0x04C6,0x04C6,0x04C6,0x04C6,0xA6F7,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x45AD,0x04C6,0x04C6,0x04C6,0x4DCE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF1A,0x85D4,0x85D4,0x85D4,0x8DF5,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0x8DF4,0xF7DE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0xBEB9,0xFFFF,0xE77D,0x85F4,0x85D4,0x85D4,0x8DF4,0xEFBE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF3B,0x85D4,0x85D4,0x85D4,0x8DF4,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC71A,0x85D4,0x85D4,0x85D4,0x8DF4,0xA656,0xA677,0xAE98,0xBED9,0xCF1A,0xDF5C,0xF7DE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 21, 2640 pixels
0xFFFF,0xF7DE,0x0CE7,0x04C6,0x04C6,0x04C6,0x6E31,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x356B,0x04C6,0x04C6,0x04C6,0x1508,0xF7DE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF9C,0x04C6,0x04C6,0x04C6,0x04C6,0xB718,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9635,0x85D4,0x85D4,0x85D4,0xBED9,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF1B,0x85D4,0x85D4,0x85D4,0x85D4,0xCF1A,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0xBED9,0xF7DF,0x9635,0x85D4,0x85D4,0x85D4,0xD73B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xA677,0x85D4,0x85D4,0x85D4,0xAE98,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0xAE77,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 22, 2760 pixels
0xFFFF,0x96B5,0x04C6,0x04C6,0x04C6,0x04C6,0xD77B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x5DEF,0x04C6,0x04C6,0x04C6,0x04C6,0x8673,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x6E31,0x04C6,0x04C6,0x04C6,0x2529,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF7C,0x85D4,0x85D4,0x85D4,0x85F4,0xEFBE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF1A,0x85F4,0x85D4,0x85D4,0x85D4,0xAE77,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xB6B8,0x85D4,0x85D4,0x85D4,0xB6B8,0xAE97,0x85D4,0x85D4,0x85D4,0xAE98,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x8DF4,0x85D4,0x85D4,0x85D4,0xCF1A,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xAE77,0x85D4,0x85D4,0x85D4,0xAE97,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 23, 2880 pixels
0xFFFF,0x2D4A,0x04C6,0x04C6,0x04C6,0x458C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DE,0xA6D6,0x2D4A,0x04C6,0x04C6,0x04C6,0x04C6,0x356B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DE,0x1508,0x04C6,0x04C6,0x04C6,0x8E94,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xAE97,0x85D4,0x85D4,0x85D4,0xA677,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEFBE,0xAE97,0x85D4,0x85D4,0x85D4,0x85D4,0x9E36,0xFFDF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xBED9,0x85D4,0x85D4,0x85D4,0x8DF5,0x85D4,0x85D4,0x85D4,0x9635,0xFFDF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEF9D,0x85D4,0x85D4,0x85D4,0x85D4,0xC6FA,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xAE98,0x85D4,0x85D4,0x85D4,0x8DF4,0xEFBE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7BE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 24, 3000 pixels
0xC739,0x04C6,0x04C6,0x04C6,0x04C6,0x2D4A,0x6610,0x7631,0x7652,0x6E31,0x5DEF,0x3D8C,0x0CE7,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x2D4A,0xEFBD,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9EB5,0x04C6,0x04C6,0x04C6,0x0CE7,0xEFDE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7BE,0x85F4,0x85D4,0x85D4,0x85D4,0xAE98,0xD75C,0xD75C,0xCF1A,0xAE77,0x85F4,0x85D4,0x85D4,0x85D4,0x85D4,0x9E56,0xF7DE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6FA,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x8DF4,0xEF9D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF7C,0x85D4,0x85D4,0x85D4,0x85D4,0x8DF4,0xBED9,0xD73B,0xCF3B,0xBED9,0x9E36,0xD75C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6FA,0x85D4,0x85D4,0x85D4,0x85D4,0x8E15,0xBED9,0xD73B,0xDF5C,0xD73B,0xC6F9,0xA677,0x8DF4,0xCF1A,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 25, 3120 pixels
0x55EF,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x55EF,0xF7DE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x356B,0x04C6,0x04C6,0x04C6,0x5DEF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6F9,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xB698,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF1A,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85F4,0xD75B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEF9D,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xE77D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEFBE,0x85F4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xD73B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 26, 3240 pixels
0x04E7,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x3D8C,0xBF18,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC75A,0x04C6,0x04C6,0x04C6,0x04C6,0xCF5A,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9615,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x9635,0xDF5C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xD73B,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xC6F9,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9E36,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xEFBD,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6FA,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xDF5C,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 27, 3360 pixels
0x3D8C,0x0CE7,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x04C6,0x0CE7,0x45AD,0x8673,0xD77B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x5DEF,0x04C6,0x04C6,0x04C6,0x356B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xC6F9,0x9635,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x8DF4,0xA677,0xD75B,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF5C,0x85D4,0x85D4,0x85D4,0x85D4,0xAE98,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEFBD,0x9E36,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x8DF4,0xB698,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF3B,0x9615,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x9635,0xBED9,0xF7DF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 28, 3480 pixels
0xFFFF,0xFFFF,0xF7DE,0xD77B,0xC75A,0xBF39,0xB718,0xB718,0xBF39,0xCF5A,0xEFBD,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEFBE,0xDF5C,0xCF1A,0xC6FA,0xC71A,0xD73B,0xE77D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xF7DE,0x9E36,0x85D4,0x85D4,0x85D4,0xA657,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE79D,0xCF3B,0xC71A,0xCF1A,0xD73B,0xE79D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xE77C,0xCF1A,0xC6FA,0xC6FA,0xCF1A,0xDF5C,0xF7BE,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 29, 3600 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xEF9D,0x9635,0x85D4,0x85D4,0x85D4,0x9E36,0xF7DF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 30, 3720 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xD75C,0xFFDF,0xFFFF,0xFFFF,0xEFBE,0xBED9,0x8DF4,0x85D4,0x85D4,0x85D4,0x9E56,0xF7DF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 31, 3840 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xDF7C,0x85D4,0x85F4,0x9615,0x8E15,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xA677,0xF7DF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 32, 3960 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0x9E36,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x8DF4,0xC6F9,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF, // row 33, 4080 pixels
0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xCF1B,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0x85D4,0xA677,0xEF9D,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF}; // row 34, 4200 pixels
