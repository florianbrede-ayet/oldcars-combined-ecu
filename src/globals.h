#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <Arduino.h>



#define VERSION 4
#define ENABLE_OLED 1 
#define OLED_REFRESH_MS 100


#define CAR_WITHOUT_ABS_BRAKES    1    // set to 1 if the car does not have antilock-braking. if enabled, braking will be interrupted within ~100ms (depending on VSS sensor) if a wheel-lock is detected


#define MODULE_INPUTS             1
#define MODULE_VSS                1
#define MODULE_THROTTLE_ACTUATOR  1
#define MODULE_BRAKE_ACTUATOR     0
#define MODULE_EPS                0



/* V1.1 BMW Actuator 
OLD CARS ECU throttle & cruise combined
This sketch can be used to control a BMW cruise throttle actuator (BMW/VDO "8 369 027" / "408.201/013/001") over CAN and will also handle the cruise part (speed signal, buttons, cruise state). 
based on: https://github.com/Lukilink/actuator_ECU

COMPONENTS:
- ARDUINO MEGA 2560 R3
- MCP2551 module
- L298N H-Bridge module
- 0.96"/1.3" 128x64 OLED DISPLAY I2C
- 3+ Button Keypad (e.g. https://www.amazon.de/gp/product/B079JWDQBW)
- 1x ODB Female Port (e.g. https://www.amazon.de/gp/product/B07TZMXQLK)
- 3x DB9 Connectors Female
- 1x 30mm 5V Fan



RETROPILOT 2560 ECU PINOUT:

DB9 1 / BOTTOM

ACTUATOR 1 orange			DB9 1			L298 OUT2
ACTUATOR 2 black			DB9 2			L298 OUT3
ACTUATOR 3 yellow			DB9 3			L298 OUT4
ACTUATOR 4 red				DB9 4			L298 OUT1
SPD_SENSOR GND				DB9 5			GND
ACTUATOR 6 grey				DB9 6			RES1KOHM (connect 1k resistor from this line to GND) / A3
ACTUATOR 7 white			DB9 7			(ARDUINO) +5V
SPD_SENSOR +5V				DB9 8			(ARDUINO) +5V
SPD_SENSOR SIGNAL			DB9 9			ARDUINO 18


DB9 2 / MID

KEYPAD 1 / GND		DB9 1			GND
KEYPAD 2 / SPD+		DB9 2			ARDUINO 42
KEYPAD 3 / SPD-		DB9 3			ARDUINO 43
KEYPAD 4 / ON		  DB9 4			ARDUINO 44
KEYPAD 5 / LKAS		DB9 5			ARDUINO 41
GND / HALL			  DB9 6			ARDUINO GND
CLUTCH / HALL		  DB9 7			ARDUINO 46
BRAKE / HALL		  DB9 8			ARDUINO 47


DB9 3 / TOP

CAN1 L 				DB9 1			MCP CAN L 	  PANDA OBD 14  --> db9 1 used to connect  STEERING ANGLE SENSOR, steering mcu and TSS RADAR 3
CAN1 H 				DB9 2			MCP CAN H	    PANDA OBD 6   --> db9 2 used to connect  STEERING ANGLE SENSOR, steering mcu and TSS RADAR 2
CAN2 L 				DB9 3			PANDA OBD 11		            --> db9 3 used to connect TSS RADAR 11
CAN2 H 				DB9 4			PANDA OBD 3		              --> db9 4 used to connect TSS RADAR 6

STEERING +12V		DB9 6			+12V  -> RADAR, EPS, STEERING ANGLE SENSOR
STEERING +12V		DB9 7			+12V  -> RADAR, EPS, STEERING ANGLE SENSOR
STEERING GND		DB9 8			GND   -> RADAR, EPS, STEERING ANGLE SENSOR
STEERING GND		DB9 9			GND   -> RADAR, EPS, STEERING ANGLE SENSOR


L298

M_IN1 				  ARDUINO 7
M_IN2 				  ARDUINO 8
M_ENA 				  ARDUINO 9
S_IN3 				  ARDUINO 6
M_ENB				    JUMPER +5V
L298 OUT2 			DB9 1 (UNTEN)
L298 OUT3 			DB9 2	(UNTEN)		
L298 OUT4 			DB9 3	(UNTEN)
L298 OUT1 			DB9 4	(UNTEN)


CAN MCP

CAN CS				ARDUINO 53
CAN INT				ARDUINO 2
CAN SO				ARDUINO MISO 50
CAN SI				ARDUINO MOSI 51
CAN SCK				ARDUINO SCK 52
CAN H				  OBD 6
CAN L				  OBD 14



0.96/1.3 128x64 OLED DISPLAY (scl/sda)

PIN_OLED_SCL 		ARDUINO 21
PIN_OLED_SDA 		ARDUINO 20
PIN_OLED_VCC		(ARDUINO) +5V
PIN_OLED_GND		(ARDUINO) GND


OBD2 PANDA CONNECTOR
OBD 3				      DB9-3 4 -> TSS RADAR 6
ODB 4				      GND
OBD 6				      MCP CAN H
OBD 8 (Ignition)	+12V
OBD 11				    DB9-3 3 -> TSS RADAR 11
OBD 14				    MCP CAN L
OBD 16 (+12V)		  +12V
*/



//////////////////// PIN CONFIG ////////////////////

// MCP 2551 CAN CONTROLLER
#define CAN_CS_PIN 53
#define CAN_INT_PIN 2

// CLUTCH & BRAKE PEDAL PINS
#define CLUTCH_CANCEL_PIN 46       // pulled to GND through default open hall sensor when CLUTCH pedal is NOT pressed
#define BRAKE_CANCEL_PIN 47        // pulled to GND through default open hall sensor when BRAKE pedal is NOT pressed

// BUTTONS AND SWITCHES (PULLDOWN INPUT)
#define BUTTON_3_PIN  42  // SPD+
#define BUTTON_2_PIN  43  // SPD-
#define BUTTON_1_PIN  44  // ON/OFF
#define BUTTON_4_PIN  41  // currently unused / LKAS later

// THROTTLE ACTUATOR POSITION POTENTIOMETER PIN
#define THROTTLE_ACTUATOR_POTI_PIN A3       // connect the potentiometer of your car's throttle

// THROTTLE ACTUATOR H BRIDGE PINS
#define THROTTLE_ACTUATOR_M_IN1_PIN 7 // motor direction (IN1-H, IN2-L = left, IN1-L, IN2-H = right)
#define THROTTLE_ACTUATOR_M_IN2_PIN 8 // motor direction (IN1-H, IN2-L = left, IN1-L, IN2-H = right)
#define THROTTLE_ACTUATOR_M_ENA_PIN 9 // 255 is run / LOW is stopp   // motor speed
#define THROTTLE_ACTUATOR_M_ENB_PIN 0 // SOLENOID / actuator clutch enable pin (for safety, the solenoid will only engage if both ENB + IN3 are high)
#define THROTTLE_ACTUATOR_S_IN3_PIN 6 // SOLENOID / actuator clutch drive pin (for safety, the solenoid will only engage if both ENB + IN3 are high)

// BRAKE ACTUATOR POSITION POTENTIOMETER PIN
#define BRAKE_ACTUATOR_POTI_PIN A3       // connect the potentiometer of your car's throttle

// BRAKE ACTUATOR H BRIDGE PINS
#define BRAKE_ACTUATOR_M_IN1_PIN 7 // motor direction (IN1-H, IN2-L = left, IN1-L, IN2-H = right)
#define BRAKE_ACTUATOR_M_IN2_PIN 8 // motor direction (IN1-H, IN2-L = left, IN1-L, IN2-H = right)
#define BRAKE_ACTUATOR_M_ENA_PIN 9 // 255 is run / LOW is stopp   // motor speed
#define BRAKE_ACTUATOR_M_ENB_PIN 0 // SOLENOID / actuator clutch enable pin (for safety, the solenoid will only engage if both ENB + IN3 are high)
#define BRAKE_ACTUATOR_S_IN3_PIN 6 // SOLENOID / actuator clutch drive pin (for safety, the solenoid will only engage if both ENB + IN3 are high)


#define VSS_HALL_SENSOR_INTERRUPT_PIN 18


#define DEFAULT_LKAS_STATE false    // whenever engaging openpilot, this is the default status for LKAS (false = no steering)
#define DEFAULT_SET_SPEED 90        // the default set speed for acc when the (input) ECU is powered up


// SAFETY
#define CLUTCH_RELEASE_GRACE_TIME_MS 500 // amount of ms to delay before pulling throttle after shifting for example
#define BRAKE_RELEASE_GRACE_TIME_MS 250 //  amount of ms to delay before pulling throttle after braking either automatically or manually


// OPENPILOT / TOYOTA MESSAGE CONFIG 
#define OP_MAX_GAS_COMMAND 1194.0f //the max Value which comes from OP on CAN ID 0x200 (actually 2074, it's being clipped) * this works because the PI tuning can be adapted to most "ranges"
#define OP_MIN_GAS_COMMAND 500.0f //the min Value which comes from OP on CAN ID 0x200 (actually lower, but we clip anything below which will result in "0" throttle)

#define OP_MIN_BRAKE_COMMAND 0.0f   // this is no braking (extracted from 0x343 GAS_CMD message)
#define OP_MAX_BRAKE_COMMAND 500.0f // this is full braking (extracted from 0x343 GAS_CMD message)



#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 5
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 56
#define DISPLAY_BORDER 0


#endif