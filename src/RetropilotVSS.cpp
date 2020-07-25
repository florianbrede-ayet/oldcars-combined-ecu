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

#include "globals.h"
#include "RetropilotParams.h"
#include "RetropilotVSS.h"

volatile byte vssSensorRevolutions=0;
volatile unsigned long vssLastTriggerMicros=0;

void interruptVssSensor() {
  vssSensorRevolutions++;
  vssLastTriggerMicros=micros();
}

/* called at initialization */
void RetropilotVSS::setup()
{
    pinMode(VSS_HALL_SENSOR_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(VSS_HALL_SENSOR_INTERRUPT_PIN), interruptVssSensor, FALLING);


    for (int i=0; i<VSS_RINGBUFFER_SIZE; i++)
        vssRingBuffer[i]=0;
}

/**
 * This function is called each loop and determines the current retropilotParams.vssAvgSpeedKMH.
 * It measures the exact micros elapsed between the last handled hall sensor trigger and the latest trigger [interrupt driven].
 * The duration is is used to determine the highest current speed within each VSS_REFRESH_RATE_MS interval 
 * (highest speed because at high frequencies, the hall sensor sometimes loses revolutions [capacitance?] so we use the biggest indiviual speed)
 * The speed is averaged for VSS_RINGBUFFER_SIZE*VSS_REFRESH_RATE_MS (< 1s)
 * */
void RetropilotVSS::updateVssSensor()
{
#if VSS_SENSOR_SMOOTHING == 0 || VSS_SENSOR_SMOOTHING == 1
    if (vssSensorRevolutions > 0)
    {

        vssDuration = (micros() - vssLastUnhandledTriggerMicros);
        uint8_t SaveSREG = SREG;
        noInterrupts();
        byte tmpVssSensorRevolutions = vssSensorRevolutions;
        vssLastUnhandledTriggerMicros = vssLastTriggerMicros;
        vssSensorRevolutions -= tmpVssSensorRevolutions;
        SREG = SaveSREG;

        vssSpeedKMH = tmpVssSensorRevolutions * (VSS_DISTANCE_PER_REVOLUTION / (vssDuration * 0.000001)) * 3.6;
#if VSS_SENSOR_SMOOTHING == 1
        vssSpeedKMH = max(min(vssSpeedKMH, retropilotParams.vssAvgSpeedKMH + 10), retropilotParams.vssAvgSpeedKMH - 10);
#endif
        vssLastWheelLockCheck=millis();
    }
#if CAR_WITHOUT_ABS_BRAKES
    else if (retropilotParams.vssAvgSpeedKMH>20 && millis()-vssLastWheelLockCheck>50) { // ATTENTION: this section relies on having the sensor installed somewhere at the rear axis, where a locked axis would result in a fatal car instability
         unsigned long lastVssDuration = 1.0f/(retropilotParams.vssAvgSpeedKMH/3.6f/VSS_DISTANCE_PER_REVOLUTION);
         lastVssDuration *= 1000000L;
         if (micros() - vssLastUnhandledTriggerMicros > lastVssDuration*3.5) { // if we've been waiting 3.5 times the expected time to trigger assume wheellock (we expect one sensor miss and decelleration, so 3+ a little margin)
            // with a distance of 0.5m/revolution this would trigger ~ 65ms after locking the wheels at 100 km/h
            lastWheelLockTime=millis();
            vssLastWheelLockCheck=millis();
         }        
    }
#endif    
    else if (micros() - vssLastUnhandledTriggerMicros > 1000L * 1000L)
    { // 1 second without hall signal is interpreted as standstill
        vssSpeedKMH = 0;
        vssLastWheelLockCheck=millis();
    }
#elif VSS_SENSOR_SMOOTHING == 2 || VSS_SENSOR_SMOOTHING == 3
    if (vssSensorRevolutions > 0)
    {
        vssDuration = (vssLastTriggerMicros - vssLastUnhandledTriggerMicros);
        uint8_t SaveSREG = SREG;
        noInterrupts();
        byte tmpVssSensorRevolutions = vssSensorRevolutions;
        vssLastUnhandledTriggerMicros = vssLastTriggerMicros;
        vssSensorRevolutions -= tmpVssSensorRevolutions;
        SREG = SaveSREG;

        float tmpSpeedKMH = tmpVssSensorRevolutions * (VSS_DISTANCE_PER_REVOLUTION / (vssDuration * 0.000001)) * 3.6;
        if (tmpSpeedKMH <= VSS_MAX_SPEED) // we cap the speed we measure to max. 150km/h (max. OP speed) because sometimes at high frequencies the hall sensor might bounce and produce incorrect, way too high readings
            vssSpeedKMH = max(vssSpeedKMH, tmpSpeedKMH);
#if VSS_SENSOR_SMOOTHING == 3
        vssSpeedKMH = max(min(vssSpeedKMH, retropilotParams.vssAvgSpeedKMH + 10), retropilotParams.vssAvgSpeedKMH - 10);
#endif
        vssLastWheelLockCheck=millis();
    }
#if CAR_WITHOUT_ABS_BRAKES
    else if (retropilotParams.vssAvgSpeedKMH>20 && millis()-vssLastWheelLockCheck>50) { // ATTENTION: this section relies on having the sensor installed somewhere at the rear axis, where a locked axis would result in a fatal car instability
         unsigned long lastVssDuration = 1.0f/(retropilotParams.vssAvgSpeedKMH/3.6f/VSS_DISTANCE_PER_REVOLUTION);
         lastVssDuration *= 1000000L;
         if (micros() - vssLastUnhandledTriggerMicros > lastVssDuration*3.5) { // if we've been waiting 3.5 times the expected time to trigger assume wheellock (we expect one sensor miss and decelleration, so 3+ a little margin)
            // with a distance of 0.5m/revolution this would trigger ~ 65ms after locking the wheels at 100 km/h
            lastWheelLockTime=millis();
            vssLastWheelLockCheck=millis();
         }        
    }
#endif      
    else if (micros() - vssLastUnhandledTriggerMicros > 1000L * 1000L)
    { // 1 second without hall signal is interpreted as standstill
        vssSpeedKMH = 0;
        vssLastWheelLockCheck=millis();
    }
#endif

    if (millis() - lastVssRefresh >= VSS_REFRESH_RATE_MS)
    {
        lastVssRefresh = millis();

        // this allows us to measure accurate low speeds (~1.5-8 km/h)
        if (vssSpeedKMH > 0)
        {
            lastValidVssSpeedKMH = vssSpeedKMH;
            lastValidVssSpeedTs = millis();
        }
        else if (vssSpeedKMH == 0 && lastValidVssSpeedKMH > 0 && millis() - lastValidVssSpeedTs < 1000)
        {
            vssSpeedKMH = lastValidVssSpeedKMH;
        }

        vssSpeedSum -= vssRingBuffer[vssRingBufferIndex];
        vssSpeedSum += vssSpeedKMH;
        vssRingBuffer[vssRingBufferIndex] = vssSpeedKMH;
        vssSpeedKMH = 0;
        vssRingBufferIndex++;
        if (vssRingBufferIndex >= VSS_RINGBUFFER_SIZE)
            vssRingBufferIndex = 0;
        retropilotParams.vssAvgSpeedKMH = vssSpeedSum / VSS_RINGBUFFER_SIZE;

    }
}

/* called at ~ 1kHz */
void RetropilotVSS::loop()
{
    updateVssSensor();

    if (millis()>2000 && millis()-lastWheelLockTime<2000)
        retropilotParams.OP_WHEELLOCK_DETECTED = true;
    else
        retropilotParams.OP_WHEELLOCK_DETECTED = false;
    
}