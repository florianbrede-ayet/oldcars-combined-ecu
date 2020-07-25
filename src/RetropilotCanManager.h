/*
  RetropilotCanManager.cpp - Handles sending and receiving of can messages between the different ECUs and OpenPilot, incl. fake messages 
  
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

#ifndef _RETROPILOTCANMANAGER_H_
#define _RETROPILOTCANMANAGER_H_

#include <Arduino.h>


class RetropilotCanManager
{
private:
  unsigned long lastCanSend100Hz=0L;
  unsigned long lastCanSend50Hz=0L;
  unsigned long lastCanSend33Hz=0L;

  int canMessagesCurrentSecond=0;
  unsigned long canMessagesCurrentSecondMillis=0;

  uint8_t canCounter=0;

  void receiveCanMessages();
  
  void sendFakeToyotaCanMessages(bool trigger100Hz, bool trigger50Hz, bool trigger33Hz);
  void sendStandardToyotaCanMessages(bool trigger100Hz, bool trigger50Hz, bool trigger33Hz);
  void sendRetropilotCanMessages(bool trigger100Hz, bool trigger50Hz, bool trigger33Hz);

public:
  int canMessagesPerSecond=0; // tracks the number of received can messages per second for debugging purposes
  
  unsigned long lastCanReceive=0L;

  // we track can message timings for each module individually. if a module is enabled on the same ecu, the timer is just updated with every can send
  unsigned long lastCanReceiveModuleInputs=0L;    
  unsigned long lastCanReceiveModuleVSS=0L;
  unsigned long lastCanReceiveModuleActuatorThrottle=0L;
  unsigned long lastCanReceiveModuleActuatorBrake=0L;
  unsigned long lastCanReceiveModuleEPS=0L;

  unsigned long lastCanReceiveSteerCommand=0L;
  unsigned long lastCanReceiveSteeringAngle=0L;

  unsigned long lastModuleErrorFlagReceive = 0; // updated whenever an ecu sends a heartbeat with a fault state

  void setup();
  void loopReceive();
  void loopSend();
};

#endif