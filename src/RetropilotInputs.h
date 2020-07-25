/*
  RetropilotInputs.h - Handling user button inputs (gra on/off, set speed etc.)
  
  The MIT License (MIT)

  Copyright (c) 2020 Florian Brede

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/


#ifndef _RETROPILOTINPUTS_H_
#define _RETROPILOTINPUTS_H_

#include <Arduino.h>

class RetropilotInputs {
  private:

    int buttonstate1;
    int lastbuttonstate1;
    unsigned long debounceTime1 = 0;

    int buttonstate2;
    int lastbuttonstate2;
    unsigned long debounceTime2 = 0;

    int buttonstate3;
    int lastbuttonstate3;
    unsigned long debounceTime3 = 0;

    int buttonstate4;
    int lastbuttonstate4;
    unsigned long debounceTime4 = 0;


    uint8_t ccLastSetSpeed = 90;


    unsigned long lastClutchPressedTime = 0L;
    unsigned long lastBrakePressedTime = 0L;


    
  public:
    void setup();
    void loop();

};

#endif