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

#include "globals.h"
#include "RetropilotParams.h"
#include "RetropilotActuatorThrottle.h"


/* called at initialization */
void RetropilotActuatorThrottle::setup()
{
    pinMode(THROTTLE_ACTUATOR_POTI_PIN, INPUT);    
    pinMode(THROTTLE_ACTUATOR_M_IN1_PIN, OUTPUT);
    pinMode(THROTTLE_ACTUATOR_M_IN2_PIN, OUTPUT);
    pinMode(THROTTLE_ACTUATOR_M_ENA_PIN, OUTPUT);
    pinMode(THROTTLE_ACTUATOR_M_ENB_PIN, OUTPUT);
    pinMode(THROTTLE_ACTUATOR_S_IN3_PIN, OUTPUT);
    digitalWrite(THROTTLE_ACTUATOR_M_IN1_PIN, LOW);
    digitalWrite(THROTTLE_ACTUATOR_M_IN2_PIN, LOW);
    digitalWrite(THROTTLE_ACTUATOR_S_IN3_PIN, LOW);
    digitalWrite(THROTTLE_ACTUATOR_M_ENA_PIN, LOW);
    digitalWrite(THROTTLE_ACTUATOR_M_ENB_PIN, LOW);
}


int RetropilotActuatorThrottle::getActuatorPotiResistance() {
  int potRaw = analogRead(THROTTLE_ACTUATOR_POTI_PIN);
  if(potRaw) 
  {
    float buffer=potRaw * 3.3f;
    float Vout = (buffer)/4096.0; // 12 bit precision
    buffer = (3.3f/Vout) - 1;
    return (int)(THROTTLE_ACTUATOR_POTI_REFERENCE_RESISTOR * buffer);
  }
  return 0;
}



/**
 * This is called whenever a setpoint is reached.
 * Does NOT disconnect the solenoid!
 * */
void RetropilotActuatorThrottle::stopActuation() {
    analogWrite(THROTTLE_ACTUATOR_M_ENA_PIN, 0);  //stop Motor
}

/**
 * This is called ~ 50/s (whenever a 0x200 can package is received) to startActuation (if stopped) and set a new setpoint.
 * The function will check if our current cycle is in the same direction as the new cycle. 
 * If so, just update the endpoints to make sure no new ramp up/down phase begins.
 * */
void RetropilotActuatorThrottle::startActuation(int mTargetPosition) {
  if (mTargetPosition<THROTTLE_ACTUATOR_MIN_POT) mTargetPosition=THROTTLE_ACTUATOR_MIN_POT;
  if (mTargetPosition>THROTTLE_ACTUATOR_MAX_POT) mTargetPosition=THROTTLE_ACTUATOR_MAX_POT;

  // within THROTTLE_ACTUATOR_ALLOWED_PERM_ERROR, stop this actuation sequence
  if (abs(actuatorPotiPosition - mTargetPosition) < THROTTLE_ACTUATOR_ALLOWED_PERM_ERROR) {
    actuatorTargetPosition=mTargetPosition; 
    return stopActuation();
  }

  // continue current actuation, just update endpoint
  if (mTargetPosition>actuatorPotiPosition && actuatorDirection==1) {
    actuatorTargetPosition=mTargetPosition;
    return;
  }

  // continue current actuation, just update endpoint
  if (mTargetPosition<actuatorPotiPosition && actuatorDirection==-1) {
    actuatorTargetPosition=mTargetPosition;
    return;
  }
  
  actuatorTargetPosition=mTargetPosition;
  if (mTargetPosition<actuatorPotiPosition) actuatorDirection=1;
  else actuatorDirection=-1;

  actuatorDutyCycleStart=millis();
}

/**
 * This is called at least 500/s to check the current actuator position, desired setpoint and actuate through the L298 H-BRIDGE
 * Supports ramping up/down and smoothing out movements close to setpoints to make the driving experience softer
 * */
void RetropilotActuatorThrottle::updateActuation() {

  if (actuatorDirection==0) return;

  if (abs(actuatorPotiPosition - actuatorTargetPosition) < THROTTLE_ACTUATOR_ALLOWED_PERM_ERROR)
      return stopActuation();


  if (actuatorPotiPosition<actuatorTargetPosition) {
    digitalWrite(THROTTLE_ACTUATOR_M_IN1_PIN, HIGH); digitalWrite(THROTTLE_ACTUATOR_M_IN2_PIN, LOW); //motor left (== pull wire)
  }
  else {
    digitalWrite(THROTTLE_ACTUATOR_M_IN1_PIN, LOW); digitalWrite(THROTTLE_ACTUATOR_M_IN2_PIN, HIGH); //motor right (== loosen wire)
  }   


  unsigned long currentDutyCycleMs = millis()-actuatorDutyCycleStart;

  boolean isOffDuty=false;

  if (currentDutyCycleMs>(actuatorDirection==1 ? THROTTLE_ACTUATOR_RAMP_UP_MS : THROTTLE_ACTUATOR_RAMP_DOWN_MS)) {
    // nothing to do - we're outside of the ramp phases  
  }
  else {
    int currentCyclePosition = currentDutyCycleMs % THROTTLE_ACTUATOR_DUTY_CYCLE_LENGTH_MS;
    int currentCycleNum = currentDutyCycleMs / THROTTLE_ACTUATOR_DUTY_CYCLE_LENGTH_MS;

    if (currentCyclePosition>(actuatorDirection==1 ? THROTTLE_ACTUATOR_RAMP_UP_INITIAL_DUTY : THROTTLE_ACTUATOR_RAMP_DOWN_INITIAL_DUTY) + currentCycleNum * (actuatorDirection==1 ? THROTTLE_ACTUATOR_RAMP_UP_PER_CYCLE_DUTY : THROTTLE_ACTUATOR_RAMP_DOWN_PER_CYCLE_DUTY)) {
      isOffDuty=true;
    }
  }
  // "PWM simulation" through on/off duty cycle phases
  if (isOffDuty) {
    analogWrite(THROTTLE_ACTUATOR_M_ENA_PIN, 0);  //stop motor - off-duty cycle position - we wait fpr the next currentCycleNum before we pull/loosen a little bit again
  }
  else if (abs(actuatorPotiPosition - actuatorTargetPosition)>THROTTLE_ACTUATOR_SLOW_MOVE_ERROR_THRESHOLD) { // we run at full speed
    analogWrite(THROTTLE_ACTUATOR_M_ENA_PIN, 255);  //run Motor
  } else { // we are close to the setpoint and move the actuator a little slower for smoothness
    analogWrite(THROTTLE_ACTUATOR_M_ENA_PIN, (actuatorDirection==1 ? THROTTLE_ACTUATOR_SLOW_MOVE_PWM_PULL : THROTTLE_ACTUATOR_SLOW_MOVE_PWM_LOOSEN));  //run Motor
  }
  
}


/* called at ~ 1kHz */
void RetropilotActuatorThrottle::loop()
{
    if (millis()-lastPotiPositionUpdate>20) {
      actuatorPotiPosition = (int)(((float)(getActuatorPotiResistance()+getActuatorPotiResistance()+getActuatorPotiResistance()+getActuatorPotiResistance()+getActuatorPotiResistance()))/5.0);
    }

    // calculating GAS_CMD_PERCENT into actuatorTargetPosition 
    if (!retropilotParams.DEBUGMODE && millis()-lastActuationStart>40) {
        lastActuationStart=millis();
        startActuation((int)(((retropilotParams.GAS_CMD_PERCENT / 100) * (THROTTLE_ACTUATOR_MAX_POT - THROTTLE_ACTUATOR_MIN_POT)) + THROTTLE_ACTUATOR_MIN_POT));
    }

    if (!retropilotParams.ALLOW_THROTTLE) {
        stopActuation();
        analogWrite(THROTTLE_ACTUATOR_S_IN3_PIN, LOW); //open solenoid (both in3 AND enb must be high to close the solenoid for safety)
        digitalWrite(THROTTLE_ACTUATOR_M_ENB_PIN, LOW); //open solenoid (both in3 AND enb must be high to close the solenoid for safety)
    }
    else { // we're not braking, changing gears, have an OP brake request or having canbus problems: we can actuate
        digitalWrite(THROTTLE_ACTUATOR_S_IN3_PIN, HIGH); // close solenoid (both in3 AND enb must be high to close the solenoid for safety)
        digitalWrite(THROTTLE_ACTUATOR_M_ENB_PIN, HIGH); // close solenoid (both in3 AND enb must be high to close the solenoid for safety)
    }

    updateActuation();
}