/*
  RetropilotInputs.cpp - Handling user button inputs (gra on/off, set speed etc.)
  
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
#include "RetropilotInputs.h"

/* called at initialization */
void RetropilotInputs::setup() {

    pinMode(CLUTCH_CANCEL_PIN, INPUT_PULLUP);
    pinMode(BRAKE_CANCEL_PIN, INPUT_PULLUP);

    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_3_PIN, INPUT_PULLUP);

    delay(1000);
    buttonstate3 = digitalRead(BUTTON_3_PIN);
    buttonstate2 = digitalRead(BUTTON_2_PIN);
    buttonstate1 = digitalRead(BUTTON_1_PIN);
    
    if (!buttonstate2 && !buttonstate3 && buttonstate1) {
        retropilotParams.DEBUGMODE=true;
    }
    else if (!buttonstate2 && !buttonstate3 && !buttonstate1) {
        retropilotParams.NO_CLUCH_BRAKE_MODE=true;
    }

    ccLastSetSpeed = DEFAULT_SET_SPEED;
}

/* called at ~ 1kHz */
void RetropilotInputs::loop() {

    if (!retropilotParams.NO_CLUCH_BRAKE_MODE) {
        int stat = digitalRead(CLUTCH_CANCEL_PIN);
        if (stat) {
            lastClutchPressedTime = millis();
        }
        retropilotParams.OP_CLUTCH_PRESSED = stat || millis()-lastClutchPressedTime<CLUTCH_RELEASE_GRACE_TIME_MS;

        stat = digitalRead(BRAKE_CANCEL_PIN);
        if (stat) {
            lastBrakePressedTime = millis();
        }
        retropilotParams.OP_BRAKE_PRESSED = stat || millis()-lastBrakePressedTime<BRAKE_RELEASE_GRACE_TIME_MS;
    }
    else {
        retropilotParams.OP_CLUTCH_PRESSED = false;
        retropilotParams.OP_BRAKE_PRESSED = false;
    }
    
    // READING BUTTONS AND SWITCHES
    buttonstate3 = digitalRead(BUTTON_3_PIN);
    buttonstate2 = digitalRead(BUTTON_2_PIN);
    buttonstate1 = digitalRead(BUTTON_1_PIN);

    if (buttonstate1 != lastbuttonstate1) {
        debounceTime1=millis();
    }
    if (buttonstate2 != lastbuttonstate2) {
        debounceTime2=millis();
    }
    if (buttonstate3 != lastbuttonstate3) {
        debounceTime3=millis();
    }

    lastbuttonstate1 = buttonstate1;
    lastbuttonstate2 = buttonstate2;
    lastbuttonstate3 = buttonstate3;


    if (buttonstate1==LOW && debounceTime1!=0 && (millis()-debounceTime1>=50L || millis()<debounceTime1)) {
        debounceTime1=0;
        if (retropilotParams.OP_ON == true)
        {
            retropilotParams.OP_ON = false;
        }
        else if(retropilotParams.OP_ON == false && !retropilotParams.DEBUGMODE)
        {
            retropilotParams.OP_ON = true;
            retropilotParams.ccSetSpeed = ccLastSetSpeed;
            retropilotParams.OP_LKAS_ENABLED = DEFAULT_LKAS_STATE;
        }
    }

    if (buttonstate2==LOW && debounceTime2!=0 && (millis()-debounceTime2>=50L || millis()<debounceTime2)) {
    debounceTime2=0;
    if (!retropilotParams.DEBUGMODE && retropilotParams.ccSetSpeed>30) {
    retropilotParams.ccSetSpeed -= 5;
    ccLastSetSpeed=retropilotParams.ccSetSpeed;
    }
    }

    /* if (retropilotParams.DEBUGMODE && buttonstate2==LOW) {
        startActuation(actuatorTargetPosition-50);
        delay(10);
    } */

    if (buttonstate3==LOW && debounceTime3!=0 && (millis()-debounceTime3>=50L || millis()<debounceTime3)) {
        debounceTime3=0;
        if (!retropilotParams.DEBUGMODE && retropilotParams.ccSetSpeed<150) {
            retropilotParams.ccSetSpeed += 5;
            ccLastSetSpeed=retropilotParams.ccSetSpeed;
        }
    }

    if (buttonstate4==LOW && debounceTime4!=0 && (millis()-debounceTime4>=50L || millis()<debounceTime4)) {
        debounceTime4=0;
        if (!retropilotParams.DEBUGMODE && retropilotParams.OP_ON) {
            retropilotParams.OP_LKAS_ENABLED = !retropilotParams.OP_LKAS_ENABLED;
        }
    }

    /* if (retropilotParams.DEBUGMODE && buttonstate3==LOW) {
        startActuation(actuatorTargetPosition+50);
        delay(10);
    } */
}