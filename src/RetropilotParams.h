#ifndef _RETROPILOTPARAMS_H_
#define _RETROPILOTPARAMS_H_

#include <Arduino.h>

struct RetropilotParams {
    bool DEBUGMODE = false ;
    bool NO_CLUCH_BRAKE_MODE = false;

    bool UNRECOVERABLE_CONFIGURATION_ERROR = false;

    bool ALLOW_THROTTLE = false;
    bool ALLOW_BRAKE = false;
    bool ALLOW_STEERING = false;

    float GAS_CMD_PERCENT = 0;
    float BRAKE_CMD_PERCENT = 0;
    

    uint8_t ccSetSpeed = 90;
    int gas_pedal_state = 0; // we always send 0 since we handle override behaviour here and not in OP
    int brake_pedal_state = 0; // we always send 0 since we handle override behaviour here and not in OP
    bool blinker_left = true;
    bool blinker_right = true;

    float vssAvgSpeedKMH = 0;   // current v_ego, calculated by the vss sensor and streamed through WHEELSPEEDS by the vss module
    float currentSteeringAngle = 0; // current steering angle, taken from the steering angle sensor messages

    // these are extracted from STEERING_LKA openpilot is streaming
    uint8_t OP_STEER_REQUEST    = 0;
    float   OP_COMMANDED_TORQUE = 0;
    
    // "OP_" data shared from the inputs module to other ecus through the retropilot status can message
    bool OP_ON = false;
    bool OP_BRAKE_PRESSED = true;
    bool OP_CLUTCH_PRESSED = true;
    bool OP_GAS_PRESSED = false;
    bool OP_LKAS_ENABLED = false;

    // shared by the VSS sensor over can - triggers for non-abs cars when the VSS sensor detects a wheellock
    bool OP_WHEELLOCK_DETECTED = false;

    // "OP_" params set by each module based on the received data and status
    bool OP_ERROR_CAN = false; // if no messages are received or a particular retropilot ecu doesn't send heartbeats
    bool OP_FAULTY_ECU = false; // if any ecu sent a "fault" status over the last 1 second

};

extern RetropilotParams retropilotParams;

#endif