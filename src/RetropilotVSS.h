/*
  RetropilotVSS.cpp - Handling VSS sensor interrupts, calculating the current wheelspeed and storing it in RetropilotParams
  
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

#ifndef _RETROPILOTVSS_H_
#define _RETROPILOTVSS_H_

#include <Arduino.h>


#define VSS_SENSOR_SMOOTHING 3    // 0 = just ringbuffer*refresh rate smoothing (e.g. over 800ms). highest response rate for reliable sensors 
                                  // 1 = in addition to 0 accounts for debounce effects of the sensor (additional, invalid signals) by limiting the change rate to 10kmh / REFRESH_RATE, e.g. 50kmh/s
                                  // 2 = assumes the sensor might lose revolutions at higher speeds (measures the maximum speed (shortest revolution time) for each refresh rate cycle) 
                                  // 3 = in addition to 2 accounts for debounce effects of the sensor (additional, invalid signals) by limiting the change rate to 10kmh / REFRESH_RATE, e.g. 50kmh/s
#define VSS_MAX_SPEED 160.0f    // the maximum speed in kmh handled by the ECU in smoothing mode 1 & 2
#define VSS_DISTANCE_PER_REVOLUTION 0.525f // 52.5cm driving distance per sensor revolution


#define VSS_RINGBUFFER_SIZE 4
#define VSS_REFRESH_RATE_MS 200

class RetropilotVSS
{
private:
  float vssRingBuffer[VSS_RINGBUFFER_SIZE];
  float vssSpeedKMH = 0;
  float vssSpeedSum = 0;
  float lastValidVssSpeedKMH = 0;

  int vssRingBufferIndex = 0;

  unsigned long vssDuration = 0;
  unsigned long lastVssRefresh = 0;
  unsigned long lastValidVssSpeedTs = 0;

  unsigned long vssLastUnhandledTriggerMicros = 0;

  unsigned long lastWheelLockTime = 0;
  unsigned long vssLastWheelLockCheck = 0;

  void updateVssSensor();

public:
  void setup();
  void loop();
};

#endif