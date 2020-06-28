#include <SPI.h>
#include "Display.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SH1106.h"
#include "globals.h"


Adafruit_SH1106 display(PIN_OLED_DIN /* MOSI */, PIN_OLED_CLK, PIN_OLED_DC, PIN_OLED_RES, PIN_OLED_CS);

void Display::init(byte contrast) {
    display.begin(SH1106_SWITCHCAPVCC);
    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.brightness(contrast*2.5);
}

void Display::initSoftSPI(byte contrast) {
  init(contrast);
}

void Display::writeCommand(byte command) {
}

void Display::writeData(byte data) {
}

void Display::resetRamAddress(void) {

}

void Display::setPageAddress(byte pageAddress) {
}

void Display::setColumnAddress(byte columnAddress) {
}

void Display::clearDisplayRAM() {
}

void Display::clearVideoBuffer() {
  display.clearDisplay();
}

void Display::drawPixel(byte x, byte y) {
  display.drawPixel(x, y, WHITE);
}

/*
 * Bresenham's line algorithm
 * en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
 */
void Display::drawLine(byte x0, byte y0, byte x1, byte y1) {
  display.drawLine(x0, y0, x1, y1, WHITE);
}

void Display::drawRect(byte x1, byte y1, byte x2, byte y2) {
  display.drawRect(x1, y1, x2-x1, y2-y1, WHITE);
}

void Display::clearRect(byte x1, byte y1, byte x2, byte y2) {
  display.drawRect(x1, y1, x2-x1, y2-y1, BLACK);
}

void Display::fillRoundRect(int16_t x, int16_t y, int16_t w,
        int16_t h, int16_t r, uint16_t color) {
          display.fillRoundRect(x,y,w,h,r,color);
}

void Display::drawString(byte x, byte y, const char *c) {
  //display.setCursor(x,y);
  //display.println(c);
  for (byte i=0; i<strlen(c); i++)
    display.drawChar(x+i*6, y, c[i], WHITE, BLACK, 1);
}

void Display::drawStringAdvanced(byte x, byte y, byte size, boolean inverted, const char *c) {
  //display.setCursor(x,y);
  //display.println(c);
  for (byte i=0; i<strlen(c); i++)
    if (inverted)
      display.drawChar(x+i*6*size, y, c[i], BLACK, WHITE, size);
    else
      display.drawChar(x+i*6*size, y, c[i], WHITE, BLACK, size);
}


void Display::drawBitmap(byte x, byte y, byte width, byte height, const byte *bitmap) {
  for (byte j=0; j<height; j++) {
    for (byte i=0; i<width; i++ ) {
      if (pgm_read_byte(bitmap + i + (j/8)*width) & _BV(j%8)) {
        drawPixel(x+i, y+j);
      }
    }
  }
}


void Display::drawBitmapOffset(byte x, byte y, byte renderWidth, byte renderHeight, byte offX, byte offY, byte width, byte height, const byte *bitmap) {
  for (byte j=0; j<renderHeight; j++) {
    for (byte i=0; i<renderWidth; i++ ) {
      if (pgm_read_byte(bitmap + (i+offX) + ((j+offY)/8)*width) & _BV((j+offY)%8)) {
        drawPixel(x+i, y+j);
      }
    }
  }
}


/*
 * Bresenham's ellipse algorithm
 * de.wikipedia.org/wiki/Bresenham-Algorithmus
 */
void Display::drawEllipse(byte x, byte y, byte a, byte b) {
  int dx = 0, dy = b; /* im I. Quadranten von links oben nach rechts unten */
  long a2 = a*a, b2 = b*b;
  long err = b2-(2*b-1)*a2, e2; /* Fehler im 1. Schritt */

  do {
    drawPixel(x+dx, y+dy); /* I. Quadrant */
    drawPixel(x-dx, y+dy); /* II. Quadrant */
    drawPixel(x-dx, y-dy); /* III. Quadrant */
    drawPixel(x+dx, y-dy); /* IV. Quadrant */

    e2 = 2*err;
    if (e2 <  (2*dx+1)*b2) {
      dx++;
      err += (2*dx+1)*b2;
    }
    if (e2 > -(2*dy-1)*a2) {
      dy--;
      err -= (2*dy-1)*a2;
    }
  }
  while (dy >= 0);

  while (dx++ < a) { /* fehlerhafter Abbruch bei flachen Ellipsen (b=1) */
    drawPixel(x+dx, y); /* -> Spitze der Ellipse vollenden */
    drawPixel(x-dx, y);
  }
}

void Display::show() {
  display.display();
}
