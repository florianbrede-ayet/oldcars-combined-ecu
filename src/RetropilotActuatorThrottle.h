/*
  RetropilotActuatorThrottle.cpp - This module is controlling a throttle actuator through 2 h-bridges (servo + solenoid / clutch) and verifies its position through a potentiometer
  
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

#ifndef _RETROPILOTACTUATORTHROTTLE_H_
#define _RETROPILOTACTUATORTHROTTLE_H_

#include <Arduino.h>

// NOTE: THIS IS THE CONFIGURATION FOR A BMW/VDO "8 369 027" / "408.201/013/001" ACTUATOR with 30mm actuation distance
#define THROTTLE_ACTUATOR_ALLOWED_PERM_ERROR 25 //will allow a difference between targetPressure and currentPressure, otherwise the actuator permanently jerks around
#define THROTTLE_ACTUATOR_MIN_POT 20 //measured at actuators lowest position
#define THROTTLE_ACTUATOR_MAX_POT 2300 //measured at actuators highest position (ot the maximum actuation length we want to allow)
#define THROTTLE_ACTUATOR_POTI_REFERENCE_RESISTOR 1000 // we measure the resistance of the potentiometer - this is the reference resistor used to cover the ~2.4k ohms potentiometer range

#define THROTTLE_ACTUATOR_SLOW_MOVE_ERROR_THRESHOLD 100 // this defines the error threshold from which on we will use the below defined PWM to move the actuator smoother towards the setpoint
#define THROTTLE_ACTUATOR_SLOW_MOVE_PWM_PULL 225   // when <100 ohms short of setpoint, use these arduino PWM for the driver (pull needs more force!)
#define THROTTLE_ACTUATOR_SLOW_MOVE_PWM_LOOSEN 127 // when <100 ohms above setpoint, use these arduino PWM for the driver

#define THROTTLE_ACTUATOR_RAMP_UP_MS 500 // the "pull" time for which duty cycling is active
#define THROTTLE_ACTUATOR_RAMP_DOWN_MS 400 // the "loosen" time for which duty cycling is active
#define THROTTLE_ACTUATOR_DUTY_CYCLE_LENGTH_MS 20 // each complete cycle is 20 ms
#define THROTTLE_ACTUATOR_RAMP_UP_INITIAL_DUTY 10 // pull initial duty in % 
#define THROTTLE_ACTUATOR_RAMP_DOWN_INITIAL_DUTY 10 // loosen initial duty in %
#define THROTTLE_ACTUATOR_RAMP_UP_PER_CYCLE_DUTY 4 // pull duty-per-cycle increment after each completed cycle in absolute percent
#define THROTTLE_ACTUATOR_RAMP_DOWN_PER_CYCLE_DUTY 5 // loosen duty-per-cycle increment after each completed cycle in absolute percent


class RetropilotActuatorThrottle
{
private:
  unsigned long lastActuationStart = 0;
  unsigned long lastPotiPositionUpdate = 0;
  
  int actuatorTargetPosition = 0;
  int actuatorPotiPosition = 0;

  int actuatorDirection=0;
  unsigned long actuatorDutyCycleStart=0;

  int getActuatorPotiResistance();
  void stopActuation();
  void updateActuation();
  void startActuation(int mTargetPosition);


public:
  void setup();
  void loop();
};

#endif