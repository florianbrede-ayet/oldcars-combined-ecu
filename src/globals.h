#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 5
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 56
#define DISPLAY_BORDER 0

#define PIN_OLED_RES 30
#define PIN_OLED_DC 31
#define PIN_OLED_CS 32
#define PIN_OLED_CLK 33
#define PIN_OLED_DIN 34

#endif