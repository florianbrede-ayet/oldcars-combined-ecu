/*
  Display.h - Library to control a ST7565 based 12864 LCD.
  Thomas Baum, th.baum@online.de, 10.05.2014
*/

#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Display {
  public:
    static const byte WIDTH = 128;
    static const byte HEIGHT = 64;
    Display() {}

    void init(byte contrast);
    void initSoftSPI(byte contrast);
    void writeCommand(byte command);
    void writeData(byte data);
    void resetRamAddress(void);
    void setPageAddress(byte pageAddress);
    void setColumnAddress(byte columnAddress);
    void clearDisplayRAM(void);
    void clearVideoBuffer(void);
    void drawPixel(byte x, byte y);
    void drawLine(byte x1, byte y1, byte x2, byte y2);
    void drawRect(byte x1, byte y1, byte x2, byte y2);

    void clearRect(byte x1, byte y1, byte x2, byte y2);

    void drawString(byte x, byte y, const char string[]);

    void drawBitmapOffset(byte x, byte y, byte renderWidth, byte renderHeight, byte offX, byte offY, byte width, byte height, const byte *bitmap);

    void drawBitmap(byte x, byte y, byte width, byte height, const byte bitmap[]);
    void drawEllipse(byte x, byte y, byte a, byte b);
    void show();


    void fillRoundRect(int16_t x, int16_t y, int16_t w,
            int16_t h, int16_t r, uint16_t color);

    void drawStringAdvanced(byte x, byte y, byte size, boolean inverted, const char *c);

  private:
    boolean isSoftSPI;
};

#endif
