/*
  RetropilotActuatorThrottle.cpp - Controlling a gas throttle actuator
  
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

#include "mcp_can.h"
#include "CanHelper.h"
#include "globals.h"
#include "RetropilotParams.h"
#include "RetropilotCanManager.h"


CanHelper canHelper;

MCP_CAN CAN(CAN_CS_PIN);


/* called at initialization */
void RetropilotCanManager::setup()
{
  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN init fail");
    Serial.println(" Init CAN again");
    delay(100);
  }

  // attention: change / remove the filter and make sure no packages are lost if additional messages from panda should be read
  /* CAN.init_Mask(0, 0, 0x3ff);
  CAN.init_Mask(1, 0, 0x3ff);
  CAN.init_Filt(0, 0, 0x200);
  CAN.init_Filt(1, 0, 0x343); */

  Serial.println("CAN init ok!");
}




void RetropilotCanManager::sendFakeToyotaCanMessages(bool trigger100Hz, bool trigger50Hz, bool trigger33Hz) {

    if (!trigger33Hz) return;

    uint8_t buf[8];
    
    // last but not least: the fake messages are sent by the main ("inputs" ECU)
    #if MODULE_INPUTS
    //0x3b7 msg ESP_CONTROL
    canHelper.resetBuffer(buf);
    buf[7] = 0x08;
    CAN.sendMsgBuf(0x3b7, 0, 8, buf);

    //0x620 msg STEATS_DOORS
    canHelper.resetBuffer(buf);
    buf[0] = 0x10;
    buf[3] = 0x1d;
    buf[4] = 0xb0;
    buf[5] = 0x40;
    CAN.sendMsgBuf(0x620, 0, 8, buf);

    // 0x3bc msg GEAR_PACKET
    canHelper.resetBuffer(buf);
    buf[5] = 0x80;
    CAN.sendMsgBuf(0x3bc, 0, 8, buf);

    //0x224 msg fake brake module
    canHelper.resetBuffer(buf);
    buf[7] = 0x8;
    CAN.sendMsgBuf(0x224, 0, 8, buf);

    //0x260 fake STEER_TORQUE_SENSOR  
    canHelper.resetBuffer(buf);
    buf[0] = 0x08;
    buf[1] = 0xff;
    buf[2] = 0xfb;
    buf[5] = 0xff;
    buf[6] = 0xdc;
    buf[7] = 0x47;
    CAN.sendMsgBuf(0x260, 0, 8, buf);
    #endif
}





void RetropilotCanManager::sendStandardToyotaCanMessages(bool trigger100Hz, bool trigger50Hz, bool trigger33Hz) {
    
    if (!trigger50Hz) return;

    uint8_t buf[8];    
    // SENDING_CAN_MESSAGES
    #if MODULE_INPUTS
    //0x1d2 msg PCM_CRUISE
    canHelper.resetBuffer(buf);
    buf[0] = (retropilotParams.OP_ON << 5) & 0x20 | (!retropilotParams.gas_pedal_state << 4) & 0x10;
    buf[6] = (retropilotParams.OP_ON << 7) & 0x80;
    canHelper.putToyotaChecksum(buf, 0x1d2);
    CAN.sendMsgBuf(0x1d2, 0, 8, buf);

    //0x1d3 msg PCM_CRUISE_2
    canHelper.resetBuffer(buf);
    buf[1] = (retropilotParams.OP_ON << 7) & 0x80 | 0x28;
    buf[2] = retropilotParams.ccSetSpeed;
    canHelper.putToyotaChecksum(buf, 0x1d3);
    CAN.sendMsgBuf(0x1d3, 0, 8, buf);

    // 0x2c1 msg GAS_PEDAL
    canHelper.resetBuffer(buf);
    buf[0] = (!retropilotParams.gas_pedal_state << 3) & 0x08;
    CAN.sendMsgBuf(0x2c1, 0, 8, buf);

    //0x614 msg steering_levers
    canHelper.resetBuffer(buf);
    buf[0] = 0x29;
    buf[2] = 0x01;
    buf[3] = (retropilotParams.blinker_left << 5) & 0x20 |(retropilotParams.blinker_right << 4) & 0x10;
    buf[6] = 0x76;
    canHelper.putToyotaChecksum(buf, 0x614);
    CAN.sendMsgBuf(0x614, 0, 8, buf);
    #endif

    #if MODULE_VSS
    //0xaa msg defaults 1a 6f WHEEL_SPEEDS
    canHelper.resetBuffer(buf);
    uint16_t wheelspeed = 0x1a6f + (retropilotParams.vssAvgSpeedKMH * 100);
    buf[0] = (wheelspeed >> 8) & 0xFF;
    buf[1] = (wheelspeed >> 0) & 0xFF;
    buf[2] = (wheelspeed >> 8) & 0xFF;
    buf[3] = (wheelspeed >> 0) & 0xFF;
    buf[4] = (wheelspeed >> 8) & 0xFF;
    buf[5] = (wheelspeed >> 0) & 0xFF;
    buf[6] = (wheelspeed >> 8) & 0xFF;
    buf[7] = (wheelspeed >> 0) & 0xFF;
    CAN.sendMsgBuf(0xaa, 0, 8, buf);
    #endif

    #if MODULE_EPS
    //0x262 fake EPS_STATUS
    canHelper.resetBuffer(buf);
    buf[3] = 0x3;
    buf[4] = 0x6c;
    CAN.sendMsgBuf(0x262, 0, 8, buf);
    #endif
}




void RetropilotCanManager::sendRetropilotCanMessages(bool trigger100Hz, bool trigger50Hz, bool trigger33Hz) {
    
    if (!trigger100Hz) return;

    uint8_t buf[8];    

    canHelper.resetBuffer(buf);

    #if MODULE_INPUTS
        lastCanReceiveModuleInputs=millis();
    		buf[0] |= 1 << 0; // inputs heartbeat flag
    		buf[1] |= retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR ? 0 : 1 << 0; // inputs status flag (1 == working)

        buf[2] |= (retropilotParams.OP_ON             ? 1 : 0) << 0; // DETAILED inputs flags
        buf[2] |= (retropilotParams.OP_BRAKE_PRESSED  ? 1 : 0) << 1; // DETAILED inputs flags
        buf[2] |= (retropilotParams.OP_CLUTCH_PRESSED ? 1 : 0) << 2; // DETAILED inputs flags
        buf[2] |= (retropilotParams.OP_GAS_PRESSED    ? 1 : 0) << 3; // DETAILED inputs flags
        buf[2] |= (retropilotParams.OP_LKAS_ENABLED   ? 1 : 0) << 4; // DETAILED inputs flags
        // "5" is taken by VSS to send WHEELLOCKS!
        buf[2] |= (retropilotParams.OP_FAULTY_ECU     ? 1 : 0) << 6; // DETAILED inputs flags (this one is just for reference, each ECU sets this error on its own whenever any ecu sends a "0" status flag)
        buf[2] |= (retropilotParams.OP_ERROR_CAN      ? 1 : 0) << 7; // DETAILED inputs flags (this one is just for reference, each ECU sets this error on its own)

        // some data for debugging purposes
        buf[4] = retropilotParams.ccSetSpeed;
        buf[5] = (int)retropilotParams.vssAvgSpeedKMH;

    #endif
    #if MODULE_VSS
        lastCanReceiveModuleVSS=millis();
    		buf[0] |= 1 << 1; // actuator heartbeat flag
    		buf[1] |= retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR ? 0 : 1 << 1; // actuator status flag (1 == working)
        buf[2] |= (retropilotParams.OP_WHEELLOCK_DETECTED   ? 1 : 0) << 5; // DETAILED inputs flags
    #endif
    #if MODULE_THROTTLE_ACTUATOR
        lastCanReceiveModuleActuatorThrottle=millis();
    		buf[0] |= 1 << 2; // throttle heartbeat flag
    		buf[1] |= retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR ? 0 : 1 << 2; // throttle status flag (1 == working)
    #endif
    #if MODULE_BRAKE_ACTUATOR
        lastCanReceiveModuleActuatorBrake=millis();
    		buf[0] |= 1 << 3; // brake heartbeat flag
    		buf[1] |= retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR ? 0 : 1 << 3; // brake status flag (1 == working)
    #endif
    #if MODULE_EPS
        lastCanReceiveModuleEPS=millis();
    		buf[0] |= 1 << 4; // eps heartbeat flag
    		buf[1] |= retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR ? 0 : 1 << 4; // eps status flag (1 == working)
        buf[3] = 1; // DETAILED eps driver status (0 == steering angle synchronization, 1 == working, 2 == serial error, 3 == motor error, 4 == encoder error, 5 == calibration error, 6 == steering angle synchronization error)
    #endif
    
    buf[6]=++canCounter;

    canHelper.putToyotaChecksum(buf, 0x112);

    CAN.sendMsgBuf(0x112, 0, 8, buf);

}


void RetropilotCanManager::receiveCanMessages() {
  while (CAN_MSGAVAIL == CAN.checkReceive())
  {
    lastCanReceive=millis();

    if (lastCanReceive-canMessagesCurrentSecondMillis>=1000) {
      canMessagesPerSecond=canMessagesCurrentSecond;
      canMessagesCurrentSecond=0;
      canMessagesCurrentSecondMillis=lastCanReceive;
    }
    canMessagesCurrentSecond++;
        
    long unsigned int rxId;
    uint8_t len = 0;
    uint8_t rxBuf[8];
    CAN.readMsgBuf(&len, rxBuf);
    rxId=CAN.getCanId();

    switch (rxId) {
      case 0x200: // COMMA PEDAL GAS COMMAND from OP
        float GAS_CMD = (rxBuf[0] << 8 | rxBuf[1] << 0); 
        // scale GAS_CMD into GAS_CMD_PERCENT
        if (GAS_CMD >= OP_MIN_GAS_COMMAND) {
          GAS_CMD = (GAS_CMD>OP_MAX_GAS_COMMAND ? OP_MAX_GAS_COMMAND : GAS_CMD);
        }
        else {
          GAS_CMD = OP_MIN_GAS_COMMAND;
        }
        retropilotParams.GAS_CMD_PERCENT = ((100/(OP_MAX_GAS_COMMAND - OP_MIN_GAS_COMMAND)) * (GAS_CMD - OP_MIN_GAS_COMMAND));
      break; 

      case 0x343: // standard toyota ACC_CONTROL (we want to extract the brake requests from ACCEL_CMD here)
        float BRAKE_CMD = ((rxBuf[0] << 8 | rxBuf[1] << 0) * -1); 

        if (BRAKE_CMD >= OP_MIN_BRAKE_COMMAND) {
          BRAKE_CMD = (BRAKE_CMD>OP_MAX_BRAKE_COMMAND ? OP_MAX_BRAKE_COMMAND : BRAKE_CMD);
        }
        else {
          BRAKE_CMD = OP_MIN_BRAKE_COMMAND;
        }
        retropilotParams.BRAKE_CMD_PERCENT = ((100/(OP_MAX_BRAKE_COMMAND - OP_MIN_BRAKE_COMMAND)) * (BRAKE_CMD - OP_MIN_BRAKE_COMMAND));
      break;

      case 0xaa: // WHEELSPEEDS message for all ecus except vss (which is sending 0xaa)
        #if MODULE_VSS
          retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // there must not be more than one VSS module, if we receive this message as vss ecu, something is wrong
        #endif
        retropilotParams.vssAvgSpeedKMH = canHelper.parseParameterBigEndianFloat(rxBuf, -67.67, 0.01, 7, 16);
      break;
    
      case 0x112: // retropilot status message
        if (canHelper.verifyToyotaChecksum(rxBuf, 0x112)) {
          if (rxBuf[0] >> 0 & 1) { // message includes INPUTS heartbeat
            lastCanReceiveModuleInputs = millis();
            #if MODULE_INPUTS
              retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // collision with another inputs module on this can bus
            #endif
            if (!(rxBuf[1] >> 0 & 1)) { // if the inputs status flag is not "0", the module is in a fault state
              lastModuleErrorFlagReceive = millis(); // will trigger "FAULTY_ECU" in safety
            }

            // parse the status data from the inputs module
            retropilotParams.OP_ON = rxBuf[2] >> 0 & 1;
            retropilotParams.OP_BRAKE_PRESSED = rxBuf[2] >> 1 & 1;
            retropilotParams.OP_CLUTCH_PRESSED = rxBuf[2] >> 2 & 1;
            retropilotParams.OP_GAS_PRESSED = rxBuf[2] >> 3 & 1;
            retropilotParams.OP_LKAS_ENABLED = rxBuf[2] >> 4 & 1;

            retropilotParams.ccSetSpeed = rxBuf[4];
            // for wheelspeeds, we parse the more accurate WHEELSPEEDS message

          }

          if (rxBuf[0] >> 1 & 1) { // message includes VSS heartbeat
            lastCanReceiveModuleVSS = millis();
            #if MODULE_VSS
              retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // collision with another vss module on this can bus
            #endif
            if (!(rxBuf[1] >> 1 & 1)) { // if the vss status flag is not "0", the module is in a fault state
              lastModuleErrorFlagReceive = millis(); // will trigger "FAULTY_ECU" in safety
            }
            // parse the status data from the vss module
            retropilotParams.OP_WHEELLOCK_DETECTED = rxBuf[2] >> 5 & 1;
          }

          if (rxBuf[0] >> 2 & 1) { // message includes throttle heartbeat
            lastCanReceiveModuleActuatorThrottle = millis();
            #if MODULE_THROTTLE_ACTUATOR
              retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // collision with another throttle module on this can bus
            #endif
            if (!(rxBuf[1] >> 2 & 1)) { // if the vss status flag is not "0", the module is in a fault state
              lastModuleErrorFlagReceive = millis(); // will trigger "FAULTY_ECU" in safety
            }
          }

          if (rxBuf[0] >> 3 & 1) { // message includes brake heartbeat
            lastCanReceiveModuleActuatorBrake = millis();
            #if MODULE_BRAKE_ACTUATOR
              retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // collision with another brake module on this can bus
            #endif
            if (!(rxBuf[1] >> 3 & 1)) { // if the vss status flag is not "0", the module is in a fault state
              lastModuleErrorFlagReceive = millis(); // will trigger "FAULTY_ECU" in safety
            }
          }

           if (rxBuf[0] >> 4 & 1) { // message includes eps heartbeat
            lastCanReceiveModuleEPS = millis();
            #if MODULE_EPS
              retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR = true;  // collision with another brake module on this can bus
            #endif
            if (!(rxBuf[1] >> 4 & 1)) { // if the EPS status flag is not "0", the module is in a fault state
              lastModuleErrorFlagReceive = millis(); // will trigger "FAULTY_ECU" in safety
            }

            if (rxBuf[3] != 1) { // if the detailed EPS status flag is not "0", we currently just assume a fault state (and don't allow engaging)
              lastModuleErrorFlagReceive = millis(); // will trigger "FAULTY_ECU" in safety
            }
          }
          
        }

        // TODO: update params, update last can receives, verify that no ecu receives messages from its own kind (otherwise unrecoverable error)
      break;
      
      case 0x2e4: // STEERING_LKA (steer requests from openpilot)
        if (canHelper.verifyToyotaChecksum(rxBuf, 0x2e4)) {
          lastCanReceiveSteerCommand = millis();
          retropilotParams.OP_STEER_REQUEST    = canHelper.parseParameterBigEndianByte(rxBuf, 0, 1, 0, 1);
          retropilotParams.OP_COMMANDED_TORQUE = canHelper.parseParameterBigEndianInt(rxBuf, 0, 1, 15, 16);
        }
      break;

      case 0x25: // STEER_ANGLE_SENSOR (coming from the TSS steering angle sensor)
        lastCanReceiveSteeringAngle = millis();
        float steerAngle    = canHelper.parseParameterBigEndianFloat(rxBuf, 0, 1.5f, 3, 12);
        float steerFraction = canHelper.parseParameterBigEndianFloat(rxBuf, 0, 0.1f, 39, 4);
        retropilotParams.currentSteeringAngle = steerAngle+steerFraction;
      break;
      
    } 
  }
}

/* called at ~ 1kHz */
void RetropilotCanManager::loopReceive()
{
    receiveCanMessages();
}

/* called at ~ 1kHz */
void RetropilotCanManager::loopSend()
{
    bool trigger100Hz = false;
    if (millis()-lastCanSend100Hz>=10) {
        trigger100Hz=true;
        lastCanSend100Hz=millis();
    }
    
    bool trigger50Hz = false;
    if (millis()-lastCanSend50Hz>=20) {
        trigger50Hz=true;
        lastCanSend50Hz=millis();
    }
    
    bool trigger33Hz = false;
    if (millis()-lastCanSend33Hz>=30) {
        trigger33Hz=true;
        lastCanSend33Hz=millis();
    }

    sendFakeToyotaCanMessages(trigger100Hz, trigger50Hz, trigger33Hz);
    sendStandardToyotaCanMessages(trigger100Hz, trigger50Hz, trigger33Hz);
    sendRetropilotCanMessages(trigger100Hz, trigger50Hz, trigger33Hz);
}