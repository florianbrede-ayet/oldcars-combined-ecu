#include <Arduino.h>
#include "mcp_can.h"
#include "Display.h"
#include "globals.h"

#define VERSION 3
#define ENABLE_OLED 1 
#define OLED_REFRESH_MS 100



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
const int CAN_CS_PIN = 53;
const int CAN_INT_PIN = 2;
MCP_CAN CAN(CAN_CS_PIN);

// CLUTCH & BRAKE PEDAL PINS
const int CLUTCH_CANCEL_PIN = 46;       // pulled to GND through default open hall sensor when CLUTCH pedal is NOT pressed
const int BRAKE_CANCEL_PIN = 47;        // pulled to GND through default open hall sensor when BRAKE pedal is NOT pressed

// ACTUATOR POSITION POTENTIOMETER PIN
const int ACTUATOR_POTI_PIN = A3;       // connect the potentiometer of your car's throttle

// ACTUATOR H BRIDGE PINS
int M_IN1_PIN = 7; // motor direction (7H, 8L = left, 7L, 8H = right)
int M_IN2_PIN = 8; // motor direction (7H, 8L = left, 7L, 8H = right)
int M_ENA_PIN = 9; // 255 is run / LOW is stopp   // motor speed
// we bridge ENB and just control IN3 to set the clutch closed or to 0 again to open
int S_IN3_PIN = 6; // 255 is run / LOW is stopp   // SOLENOID / actuator clutch!


const int VSS_HALL_SENSOR_INTERRUPT_PIN = 18;



// VSS SENSOR
#define VSS_SENSOR_SMOOTHING 3    // 0 = just ringbuffer*refresh rate smoothing (e.g. over 800ms). highest response rate for reliable sensors 
                                  // 1 = in addition to 0 accounts for debounce effects of the sensor (additional, invalid signals) by limiting the change rate to 10kmh / REFRESH_RATE, e.g. 50kmh/s
                                  // 2 = assumes the sensor might lose revolutions at higher speeds (measures the maximum speed (shortest revolution time) for each refresh rate cycle) 
                                  // 3 = in addition to 2 accounts for debounce effects of the sensor (additional, invalid signals) by limiting the change rate to 10kmh / REFRESH_RATE, e.g. 50kmh/s
#define VSS_MAX_SPEED 160.0f    // the maximum speed in kmh handled by the ECU in smoothing mode 1 & 2
#define VSS_DISTANCE_PER_REVOLUTION 0.525f // 52.5cm driving distance per sensor revolution

const int VSS_RINGBUFFER_SIZE = 4;
const int VSS_REFRESH_RATE_MS = 200;
float vssRingBuffer[VSS_RINGBUFFER_SIZE];
float vssSpeedKMH=0;
float vssSpeedSum=0;
float vssAvgSpeedKMH=0;
float lastValidVssSpeedKMH=0;

int vssRingBufferIndex=0;

unsigned long vssDuration=0;
unsigned long lastVssRefresh=0;
unsigned long lastValidVssSpeedTs=0;

volatile byte vssSensorRevolutions=0;
volatile unsigned long vssLastTriggerMicros=0;
unsigned long vssLastUnhandledTriggerMicros=0;


//////////////////// THROTTLE CONTROL PART ////////////////////

// ACTUATOR SETUP:
// NOTE: THIS IS THE CONFIGURATION FOR A BMW/VDO "8 369 027" / "408.201/013/001" ACTUATOR with 30mm actuation distance
const int ACTUATOR_ALLOWED_PERM_ERROR = 25; //will allow a difference between targetPressure and currentPressure, otherwise the actuator permanently jerks around
const int ACTUATOR_MIN_POT = 20; //measured at actuators lowest position
const int ACTUATOR_MAX_POT = 2300; //measured at actuators highest position (ot the maximum actuation length we want to allow)
const int ACTUATOR_POT_REFERENCE_RESISTOR = 1000; // we measure the resistance of the potentiometer - this is the reference resistor used to cover the ~2.4k ohms potentiometer range

const int SLOW_MOVE_ERROR_THRESHOLD=100; // this defines the error threshold from which on we will use the below defined PWM to move the actuator smoother towards the setpoint
const int SLOW_MOVE_PWM_PULL=225;   // when <100 ohms short of setpoint, use these arduino PWM for the driver (pull needs more force!)
const int SLOW_MOVE_PWM_LOOSEN=127; // when <100 ohms above setpoint, use these arduino PWM for the driver

const unsigned int ACTUATOR_RAMP_UP_MS=500; // the "pull" time for which duty cycling is active
const unsigned int ACTUATOR_RAMP_DOWN_MS=400; // the "loosen" time for which duty cycling is active
const int ACTUATOR_DUTY_CYCLE_LENGTH_MS=20; // each complete cycle is 20 ms
const int ACTUATOR_RAMP_UP_INITIAL_DUTY=10; // pull initial duty in % 
const int ACTUATOR_RAMP_DOWN_INITIAL_DUTY=10; // loosen initial duty in %
const int ACTUATOR_RAMP_UP_PER_CYCLE_DUTY=4; // pull duty-per-cycle increment after each completed cycle in absolute percent
const int ACTUATOR_RAMP_DOWN_PER_CYCLE_DUTY=5; // loosen duty-per-cycle increment after each completed cycle in absolute percent

int actuatorDirection=0;
unsigned long actuatorDutyCycleStart=0;

// throttle variables
int actuatorTargetPosition = 0;
int actuatorPotiPosition = 0;
float GAS_CMD_PERCENT = 0;
float GAS_CMD = 0;
float GAS_CMD1 = 0;
boolean cancelGasActuation = false;

// brake variables
float BRAKE_CMD=0;
float BRAKE_CMD1=0;
float BRAKE_CMD_PERCENT=0;
boolean isBrakeRequest=false;



// SAFETY
unsigned long lastCanReceive=0L;
unsigned long lastClutchPressedTime = 0L;
const int CLUTCH_RELEASE_GRACE_TIME_MS = 500; // amount of ms to delay before pulling throttle after shifting for example



// OPENPILOT CONFIG 
float OP_MAX_GAS_COMMAND = 1194; //the max Value which comes from OP on CAN ID 0x200 (actually higher, it's being clipped)
float OP_MIN_GAS_COMMAND = 500; //the min Value which comes from OP on CAN ID 0x200 (actually lower, but we clip anything below which will result in "0" throttle)

float OP_MIN_BRAKE_COMMAND = 0;   // this is no braking (extracted from 0x343 GAS_CMD message)
float OP_MAX_BRAKE_COMMAND = 500; // this is full braking (extracted from 0x343 GAS_CMD message)



//////////////////// CRUISE PART ////////////////////


// BUTTONS AND SWITCHES (PULLDOWN INPUT)
int BUTTON_3_PIN = 42;  // SPD+
int BUTTON_2_PIN = 43;  // SPD-
int BUTTON_1_PIN = 44;  // ON/OFF
int BUTTON_4_PIN = 41;  // currently unused / LKAS later

int buttonstate3;
int lastbuttonstate3;
unsigned long debounceTime3 = 0;

int buttonstate2;
int lastbuttonstate2;
unsigned long debounceTime2 = 0;

int buttonstate1;
int lastbuttonstate1;
unsigned long debounceTime1 = 0;

boolean DEBUGMODE=false;
boolean NO_CLUCH_BRAKE_MODE=false;



// CAN SEND SPECIFIC VARIABLES
unsigned long lastCanSend=0L;

boolean OP_ON = false;
uint8_t ccSetSpeed = 90;
uint8_t ccLastSetSpeed = 90;

const int gas_pedal_state = 0; // we always send 0 since we handle override behaviour here and not in OP
const int brake_pedal_state = 0; // we always send 0 since we handle override behaviour here and not in OP
const boolean blinker_left = true;
const boolean blinker_right = true;



// DISPLAY / STATISTICS related variables
char msgString[65];
boolean isFirstRender=true;
unsigned long lastOledRefresh=0;
int subOledRefresh=0;
int canMessagesPerSecond=0; // tracks the number of received can messages per second. should be around 100/s without major drops
int canMessagesCurrentSecond=0;
unsigned long canMessagesCurrentSecondMillis=0;


unsigned long lastMainLoop=0;
unsigned long lastMainLoopCurrentSecondMillis=0;

unsigned int vssTotalSensorRevolutions=0;

int lastMainLoopPerSecond=0; // tracks the number of loops() processed per second. target is around 500/s (if can network is active!)
int lastMainLoopCurrentSecond=0;
  
int displayedActuatorStatus=0;





#if ENABLE_OLED
Display lcd = Display();
#endif




int getActuatorPotiResistance() {
  int potRaw = analogRead(ACTUATOR_POTI_PIN);
  if(potRaw) 
  {
    float buffer=potRaw * 5.0f;
    float Vout = (buffer)/1024.0;
    buffer = (5.0f/Vout) - 1;
    return (int)(ACTUATOR_POT_REFERENCE_RESISTOR * buffer);
  }
  return 0;
}

void interruptVssSensor() {
  vssSensorRevolutions++;
  vssLastTriggerMicros=micros();
}

//TOYOTA CAN CHECKSUM
int can_cksum (uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  //uint16_t temp_msg = msg;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}


/**
 * This is called whenever a setpoint is reached.
 * Does NOT disconnect the solenoid!
 * */
void stopActuation() {
    analogWrite(M_ENA_PIN, 0);  //stop Motor
    displayedActuatorStatus=0;
}

/**
 * This is called ~ 50/s (whenever a 0x200 can package is received) to startActuation (if stopped) and set a new setpoint.
 * The function will check if our current cycle is in the same direction as the new cycle. 
 * If so, just update the endpoints to make sure no new ramp up/down phase begins.
 * */
void startActuation(int mTargetPosition) {
  if (mTargetPosition<ACTUATOR_MIN_POT) mTargetPosition=ACTUATOR_MIN_POT;
  if (mTargetPosition>ACTUATOR_MAX_POT) mTargetPosition=ACTUATOR_MAX_POT;

  // within ACTUATOR_ALLOWED_PERM_ERROR, stop this actuation sequence
  if (abs(actuatorPotiPosition - mTargetPosition) < ACTUATOR_ALLOWED_PERM_ERROR) {
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
 * This is called ~ 500/s to check the current actuator position, desired setpoint and actuate through the L298 H-BRIDGE
 * Supports ramping up/down and smoothing out movements close to setpoints to make the driving experience softer
 * */
void updateActuation() {

  actuatorPotiPosition = (int)(((float)(getActuatorPotiResistance()+getActuatorPotiResistance()+getActuatorPotiResistance()+getActuatorPotiResistance()+getActuatorPotiResistance()))/5.0);

  if (actuatorDirection==0) return;

  if (abs(actuatorPotiPosition - actuatorTargetPosition) < ACTUATOR_ALLOWED_PERM_ERROR)
      return stopActuation();


  if (actuatorPotiPosition<actuatorTargetPosition) {
    digitalWrite(M_IN1_PIN, HIGH); digitalWrite(M_IN2_PIN, LOW); //motor left (== pull wire)
  }
  else {
    digitalWrite(M_IN1_PIN, LOW); digitalWrite(M_IN2_PIN, HIGH); //motor right (== loosen wire)
  }   


  unsigned long currentDutyCycleMs = millis()-actuatorDutyCycleStart;

  boolean isOffDuty=false;

  if (currentDutyCycleMs>(actuatorDirection==1 ? ACTUATOR_RAMP_UP_MS : ACTUATOR_RAMP_DOWN_MS)) {
    // nothing to do - we're outside of the ramp phases  
  }
  else {
    int currentCyclePosition = currentDutyCycleMs % ACTUATOR_DUTY_CYCLE_LENGTH_MS;
    int currentCycleNum = currentDutyCycleMs / ACTUATOR_DUTY_CYCLE_LENGTH_MS;

    if (currentCyclePosition>(actuatorDirection==1 ? ACTUATOR_RAMP_UP_INITIAL_DUTY : ACTUATOR_RAMP_DOWN_INITIAL_DUTY) + currentCycleNum * (actuatorDirection==1 ? ACTUATOR_RAMP_UP_PER_CYCLE_DUTY : ACTUATOR_RAMP_DOWN_PER_CYCLE_DUTY)) {
      isOffDuty=true;
    }
  }
  // "PWM simulation" through on/off duty cycle phases
  if (isOffDuty) {
    analogWrite(M_ENA_PIN, 0);  //stop motor - off-duty cycle position - we wait fpr the next currentCycleNum before we pull/loosen a little bit again
    displayedActuatorStatus=0;
  }
  else if (abs(actuatorPotiPosition - actuatorTargetPosition)>SLOW_MOVE_ERROR_THRESHOLD) { // we run at full speed
    analogWrite(M_ENA_PIN, 255);  //run Motor
    displayedActuatorStatus=actuatorDirection*255;
  } else { // we are close to the setpoint and move the actuator a little slower for smoothness
    analogWrite(M_ENA_PIN, (actuatorDirection==1 ? SLOW_MOVE_PWM_PULL : SLOW_MOVE_PWM_LOOSEN));  //run Motor
    displayedActuatorStatus=actuatorDirection*(actuatorDirection==1 ? SLOW_MOVE_PWM_PULL : SLOW_MOVE_PWM_LOOSEN);
  }
  
}




void setupThrottle() {
  // set up pin modes
  pinMode(CLUTCH_CANCEL_PIN, INPUT_PULLUP);
  pinMode(BRAKE_CANCEL_PIN, INPUT_PULLUP);
  pinMode(ACTUATOR_POTI_PIN, INPUT);    
  
  pinMode(M_IN1_PIN, OUTPUT);
  pinMode(M_IN2_PIN, OUTPUT);
  pinMode(M_ENA_PIN, OUTPUT);
  digitalWrite(M_IN1_PIN, LOW);
  digitalWrite(M_IN2_PIN, LOW);
  digitalWrite(M_ENA_PIN, LOW);

  pinMode(S_IN3_PIN, OUTPUT);
  digitalWrite(S_IN3_PIN, LOW);


  // UNCOMMENT FOR DEBUGGING
  /* Serial.println("OPEN / STOP");
  getActuatorPotiResistance();
  while(1) {
    digitalWrite(S_IN3_PIN, HIGH);
    Serial.println("CLOSE SOLENOID");
    delay(5000);

    digitalWrite(M_IN1_PIN, HIGH); //motor driection left
    digitalWrite(M_IN2_PIN, LOW); //motor driection left
    
    analogWrite(M_ENA_PIN, 255);  //run Motor
    Serial.println("RUN MOTOR LEFT until end");
    while(1) {
      int res = getActuatorPotiResistance();
      if (res>=ACTUATOR_MAX_POT) break;
      delay(1);
    }
    analogWrite(M_ENA_PIN, 0);  //stop Motor
    Serial.print("FINAL RESISTANCE  ");
    Serial.println(getActuatorPotiResistance());
    delay(1000);
    Serial.print("FINAL RESISTANCE  AFTER 1s sleep   ");
    Serial.println(getActuatorPotiResistance());
    delay(1000);


    digitalWrite(M_IN1_PIN, LOW); //motor driection right
    digitalWrite(M_IN2_PIN, HIGH); //motor driection right
    analogWrite(M_ENA_PIN, 255);  //run Motor
    Serial.println("RUN MOTOR RIGHT until end");
    while(1) {
      int res = getActuatorPotiResistance();
      if (res<=ACTUATOR_MIN_POT) break;
      delay(1);
    }
    delay(5000);
  
    Serial.print("END CYCLE RESISTANCE  ");
    Serial.println(getActuatorPotiResistance());
  } */
}


void setupCruise() {
  pinMode(VSS_HALL_SENSOR_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VSS_HALL_SENSOR_INTERRUPT_PIN), interruptVssSensor, FALLING);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);


  for (int i=0; i<VSS_RINGBUFFER_SIZE; i++)
    vssRingBuffer[i]=0;
}



void displayOled() {
  #if ENABLE_OLED
  if (millis()-lastOledRefresh<OLED_REFRESH_MS)
    return;

  lastOledRefresh=millis();

  if (isFirstRender) {
    lcd.clearVideoBuffer();
    sprintf(msgString, "RetroPilot V%d ", VERSION);
    lcd.drawString(0, 0, msgString);
    lcd.drawLine(0,12,128,12);    
 
    lcd.drawLine(0,24,128,24);    
    lcd.drawLine(0,35,128,35);          
    lcd.drawLine(0,46,128,46);    
    lcd.drawLine(0,63,128,63);    
    isFirstRender=false;
  }

  if (subOledRefresh==0) {
    if (DEBUGMODE)
      lcd.drawString(93, 0, "DEBUG");
    else if (NO_CLUCH_BRAKE_MODE)
      lcd.drawString(93, 0, "UNSAFE");
    else {
      sprintf(msgString, "%d", (int)(vssTotalSensorRevolutions));  
      lcd.drawString(87, 0, msgString);
    }
  }

  if (subOledRefresh==1) {
    sprintf(msgString, "O:%d C:%-3d V:%-3d G:%3d%%   ", OP_ON, ccSetSpeed, (int)vssAvgSpeedKMH, ((int)GAS_CMD_PERCENT));
    lcd.drawString(0, 15, msgString);
  }

  if (subOledRefresh==2) {  
    sprintf(msgString, "P:%-4d T:%-4d O:%-4d   ", actuatorPotiPosition, actuatorTargetPosition, abs(actuatorTargetPosition-actuatorPotiPosition));
    lcd.drawString(0, 26, msgString);
  }

  if (subOledRefresh==3) {
    sprintf(msgString, "CAN/s:%d  LOOP/s:%d", canMessagesPerSecond, lastMainLoopPerSecond);
    lcd.drawString(0, 37, msgString);
  }

  if (subOledRefresh==4) {
    sprintf(msgString, "BRA:%d  CLU:%d  MOT:%d   ", digitalRead(BRAKE_CANCEL_PIN), digitalRead(CLUTCH_CANCEL_PIN), displayedActuatorStatus);
    lcd.drawString(0, 48, msgString);

  }

  subOledRefresh++;
  if (subOledRefresh>=5) subOledRefresh=0;
  
  if (subOledRefresh==1)
    lcd.show();

  #endif

}


void setup() {
  delay(2000);

  #if ENABLE_OLED
  lcd.initI2C(100);
  lcd.clearVideoBuffer();
  lcd.drawString(10, 10, "Starting RetroPilot 2560...");
  lcd.show();
  #endif

  Serial.begin(115200);

  // init CAN
  #if ENABLE_OLED  
  lcd.drawString(10, 20, "Init CAN... "); lcd.show();
  #endif

  while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))              // init can bus : baudrate = 500k
    {
      Serial.println("CAN BUS Shield init fail");
      Serial.println(" Init CAN BUS Shield again");
      #if ENABLE_OLED
      lcd.drawString(10, 20, "Init CAN... ERR"); lcd.show();
      #endif
      delay(100);
    }
  Serial.println("CAN BUS Shield init ok!");

  #if ENABLE_OLED
  lcd.drawString(10, 20, "Init CAN... OK"); lcd.show();
  #endif
  
  // attention: change / remove the filter and make sure no packages are lost if additional messages from panda should be read
  CAN.init_Mask(0, 0, 0x3ff);
  CAN.init_Mask(1, 0, 0x3ff);
  CAN.init_Filt(0, 0, 0x200);
  CAN.init_Filt(1, 0, 0x343);
  

  setupThrottle();

  #if ENABLE_OLED
  lcd.drawString(10, 30, "Init Throttle... OK"); lcd.show();
  #endif

  setupCruise();
  #if ENABLE_OLED
  lcd.drawString(10, 40, "Init Cruise... OK"); lcd.show();
  #endif

  delay(1000);
  buttonstate3 = digitalRead(BUTTON_3_PIN);
  buttonstate2 = digitalRead(BUTTON_2_PIN);
  buttonstate1 = digitalRead(BUTTON_1_PIN);

  Serial.print("btn1: ");
  Serial.print(buttonstate1);
  Serial.print("  btn2: ");
  Serial.print(buttonstate2);
  Serial.print("  btn3: ");
  Serial.print(buttonstate3);
  Serial.println("");
  
  if (!buttonstate2 && !buttonstate3 && buttonstate1) {
    DEBUGMODE=true;
    #if ENABLE_OLED
    lcd.drawString(10, 50, "SET DEBUGMODE..."); lcd.show();
    delay(2000);
    #endif
  }
  else if (!buttonstate2 && !buttonstate3 && !buttonstate1) {
    NO_CLUCH_BRAKE_MODE=true;
    #if ENABLE_OLED
    lcd.drawString(10, 50, "SET UNSAFE MODE..."); lcd.show();
    delay(2000);
    #endif
  }
  displayOled();
}

/**
 * This function is called each loop and determines the current vssAvgSpeedKMH.
 * It measures the exact micros elapsed between the last handled hall sensor trigger and the latest trigger [interrupt driven].
 * The duration is is used to determine the highest current speed within each VSS_REFRESH_RATE_MS interval 
 * (highest speed because at high frequencies, the hall sensor sometimes loses revolutions [capacitance?] so we use the biggest indiviual speed)
 * The speed is averaged for VSS_RINGBUFFER_SIZE*VSS_REFRESH_RATE_MS (< 1s)
 * */
void loopUpdateVssSensor() {
  #if VSS_SENSOR_SMOOTHING==0 || VSS_SENSOR_SMOOTHING==1
    if (vssSensorRevolutions>0) {

      vssDuration = (micros() - vssLastUnhandledTriggerMicros);
      uint8_t SaveSREG = SREG;
      noInterrupts();
      byte tmpVssSensorRevolutions=vssSensorRevolutions;
      vssLastUnhandledTriggerMicros=vssLastTriggerMicros;
      vssSensorRevolutions -= tmpVssSensorRevolutions;
      SREG = SaveSREG;

      vssSpeedKMH = tmpVssSensorRevolutions * (VSS_DISTANCE_PER_REVOLUTION / (vssDuration * 0.000001)) * 3.6;
      #if VSS_SENSOR_SMOOTHING==1
        vssSpeedKMH=max(min(vssSpeedKMH, vssAvgSpeedKMH+10), vssAvgSpeedKMH-10);
      #endif
      vssTotalSensorRevolutions += tmpVssSensorRevolutions;
    }
    else if (micros()-vssLastUnhandledTriggerMicros>1000L*1000L) { // 1 second without hall signal is interpreted as standstill
      vssSpeedKMH=0;
    }
  #elif VSS_SENSOR_SMOOTHING==2 || VSS_SENSOR_SMOOTHING==3
    if (vssSensorRevolutions>0) {
        vssDuration = (vssLastTriggerMicros - vssLastUnhandledTriggerMicros);
        uint8_t SaveSREG = SREG;
        noInterrupts();
        byte tmpVssSensorRevolutions=vssSensorRevolutions;
        vssLastUnhandledTriggerMicros=vssLastTriggerMicros;
        vssSensorRevolutions -= tmpVssSensorRevolutions;
        SREG = SaveSREG;

        float tmpSpeedKMH = tmpVssSensorRevolutions * (VSS_DISTANCE_PER_REVOLUTION / (vssDuration * 0.000001)) * 3.6;
        if (tmpSpeedKMH<=VSS_MAX_SPEED) // we cap the speed we measure to max. 150km/h (max. OP speed) because sometimes at high frequencies the hall sensor might bounce and produce incorrect, way too high readings
          vssSpeedKMH = max(vssSpeedKMH, tmpSpeedKMH);
        #if VSS_SENSOR_SMOOTHING==3
          vssSpeedKMH=max(min(vssSpeedKMH, vssAvgSpeedKMH+10), vssAvgSpeedKMH-10);
        #endif
        vssTotalSensorRevolutions += tmpVssSensorRevolutions;
    }
    else if (micros()-vssLastUnhandledTriggerMicros>1000L*1000L) { // 1 second without hall signal is interpreted as standstill
      vssSpeedKMH=0;
    }
  #endif

  if (millis()-lastVssRefresh>=VSS_REFRESH_RATE_MS) {
    lastVssRefresh=millis();
    
    // this allows us to measure accurate low speeds (~1.5-8 km/h)
    if (vssSpeedKMH>0) {
      lastValidVssSpeedKMH=vssSpeedKMH;
      lastValidVssSpeedTs=millis();
    }
    else if (vssSpeedKMH==0 && lastValidVssSpeedKMH>0 && millis()-lastValidVssSpeedTs<1000) {
      vssSpeedKMH=lastValidVssSpeedKMH;
    }

    vssSpeedSum-=vssRingBuffer[vssRingBufferIndex];
    vssSpeedSum+=vssSpeedKMH;
    vssRingBuffer[vssRingBufferIndex]=vssSpeedKMH;
    vssSpeedKMH=0;
    vssRingBufferIndex++;
    if (vssRingBufferIndex>=VSS_RINGBUFFER_SIZE)
      vssRingBufferIndex=0;
    vssAvgSpeedKMH = vssSpeedSum / VSS_RINGBUFFER_SIZE;
  }
  
}


void loopSendCan() {
  if ((millis()-lastCanSend>=20L || millis()<lastCanSend) && !DEBUGMODE) {
    lastCanSend=millis();
    // SENDING_CAN_MESSAGES
    //0x1d2 msg PCM_CRUISE
    uint8_t dat[8];
    dat[0] = (OP_ON << 5) & 0x20 | (!gas_pedal_state << 4) & 0x10;
    dat[1] = 0x0;
    dat[2] = 0x0;
    dat[3] = 0x0;
    dat[4] = 0x0;
    dat[5] = 0x0;
    dat[6] = (OP_ON << 7) & 0x80;
    dat[7] = can_cksum(dat, 7, 0x1d2);
    CAN.sendMsgBuf(0x1d2, 0, 8, dat);

    //0x1d3 msg PCM_CRUISE_2
    uint8_t dat2[8];
    dat2[0] = 0x0;
    dat2[1] = (OP_ON << 7) & 0x80 | 0x28;
    dat2[2] = ccSetSpeed;
    dat2[3] = 0x0;
    dat2[4] = 0x0;
    dat2[5] = 0x0;
    dat2[6] = 0x0;
    dat2[7] = can_cksum(dat2, 7, 0x1d3);
    CAN.sendMsgBuf(0x1d3, 0, 8, dat2);

    //0xaa msg defaults 1a 6f WHEEL_SPEEDS
    uint8_t dat3[8];
    uint16_t wheelspeed = 0x1a6f + (vssAvgSpeedKMH * 100);
    dat3[0] = (wheelspeed >> 8) & 0xFF;
    dat3[1] = (wheelspeed >> 0) & 0xFF;
    dat3[2] = (wheelspeed >> 8) & 0xFF;
    dat3[3] = (wheelspeed >> 0) & 0xFF;
    dat3[4] = (wheelspeed >> 8) & 0xFF;
    dat3[5] = (wheelspeed >> 0) & 0xFF;
    dat3[6] = (wheelspeed >> 8) & 0xFF;
    dat3[7] = (wheelspeed >> 0) & 0xFF;
    CAN.sendMsgBuf(0xaa, 0, 8, dat3);

    //0x3b7 msg ESP_CONTROL
    uint8_t dat5[8];
    dat5[0] = 0x0;
    dat5[1] = 0x0;
    dat5[2] = 0x0;
    dat5[3] = 0x0;
    dat5[4] = 0x0;
    dat5[5] = 0x0;
    dat5[6] = 0x0;
    dat5[7] = 0x08;
    CAN.sendMsgBuf(0x3b7, 0, 8, dat5);

    //0x620 msg STEATS_DOORS
    uint8_t dat6[8];
    dat6[0] = 0x10;
    dat6[1] = 0x0;
    dat6[2] = 0x0;
    dat6[3] = 0x1d;
    dat6[4] = 0xb0;
    dat6[5] = 0x40;
    dat6[6] = 0x0;
    dat6[7] = 0x0;
    CAN.sendMsgBuf(0x620, 0, 8, dat6);

    // 0x3bc msg GEAR_PACKET
    uint8_t dat7[8];
    dat7[0] = 0x0;
    dat7[1] = 0x0;
    dat7[2] = 0x0;
    dat7[3] = 0x0;
    dat7[4] = 0x0;
    dat7[5] = 0x80;
    dat7[6] = 0x0;
    dat7[7] = 0x0;
    CAN.sendMsgBuf(0x3bc, 0, 8, dat7);

    // 0x2c1 msg GAS_PEDAL
    uint8_t dat10[8];
    dat10[0] = (!gas_pedal_state << 3) & 0x08;
    dat10[1] = 0x0;
    dat10[2] = 0x0;
    dat10[3] = 0x0;
    dat10[4] = 0x0;
    dat10[5] = 0x0;
    dat10[6] = 0x0;
    dat10[7] = 0x0;
    CAN.sendMsgBuf(0x2c1, 0, 8, dat10);

    //0x224 msg fake brake module
    uint8_t dat11[8];
    dat11[0] = 0x0;
    dat11[1] = 0x0;
    dat11[2] = 0x0;
    dat11[3] = 0x0;
    dat11[4] = 0x0;
    dat11[5] = 0x0;
    dat11[6] = 0x0;
    dat11[7] = 0x8;
    CAN.sendMsgBuf(0x224, 0, 8, dat11);

    //0x614 msg steering_levers
    uint8_t dat614[8];
    dat614[0] = 0x29;
    dat614[1] = 0x0;
    dat614[2] = 0x01;
    dat614[3] = (blinker_left << 5) & 0x20 |(blinker_right << 4) & 0x10;
    dat614[4] = 0x0;
    dat614[5] = 0x0;
    dat614[6] = 0x76;
    dat614[7] = can_cksum(dat614, 7, 0x614);
    CAN.sendMsgBuf(0x614, 0, 8, dat614);


    //0x262 fake EPS_STATUS
    uint8_t dat8[8];
    dat8[0] = 0x0;
    dat8[1] = 0x0;
    dat8[2] = 0x0;
    dat8[3] = 0x3;
    dat8[4] = 0x6c;
    CAN.sendMsgBuf(0x262, 0, 8, dat8);

    //0x260 fake STEER_TORQUE_SENSOR  
    uint8_t dat9[8];
    dat9[0] = 0x08;
    dat9[1] = 0xff;
    dat9[2] = 0xfb;
    dat9[3] = 0x0;
    dat9[4] = 0x0;
    dat9[5] = 0xff;
    dat9[6] = 0xdc;
    dat9[7] = 0x47;
    CAN.sendMsgBuf(0x260, 0, 8, dat9);
  }
}


void loopCruiseButtons() {
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
    if (OP_ON == true)
    {
      OP_ON = false;
    }
    else if(OP_ON == false && !DEBUGMODE)
    {
      OP_ON = true;
      ccSetSpeed = ccLastSetSpeed;
      
    }
    lastCanSend=0L;
  }

  if (buttonstate2==LOW && debounceTime2!=0 && (millis()-debounceTime2>=50L || millis()<debounceTime2)) {
    debounceTime2=0;
    if (!DEBUGMODE && ccSetSpeed>30) {
      ccSetSpeed -= 5;
      ccLastSetSpeed=ccSetSpeed;
    }
  }

  if (DEBUGMODE && buttonstate2==LOW) {
      startActuation(actuatorTargetPosition-50);
      delay(10);
  }

  if (buttonstate3==LOW && debounceTime3!=0 && (millis()-debounceTime3>=50L || millis()<debounceTime3)) {
    debounceTime3=0;
    if (!DEBUGMODE && ccSetSpeed<150) {
      ccSetSpeed += 5;
      ccLastSetSpeed=ccSetSpeed;
    }
  }


  if (DEBUGMODE && buttonstate3==LOW) {
    startActuation(actuatorTargetPosition+50);
    delay(10);
  }
}


void loopReceiveCan() {
  while (CAN_MSGAVAIL == CAN.checkReceive())
  {

      lastCanReceive=millis();

      if (lastCanReceive-canMessagesCurrentSecondMillis>=1000) {
        canMessagesPerSecond=canMessagesCurrentSecond;
        canMessagesCurrentSecond=0;
        canMessagesCurrentSecondMillis=lastCanReceive;
      }
      canMessagesCurrentSecond++;
          
      // read data,  len: da
      long unsigned int rxId;
      uint8_t len = 0;
      uint8_t rxBuf[8];
      CAN.readMsgBuf(&len, rxBuf);
      rxId=CAN.getCanId();


      if (rxId == 0x200) { // PEDAL GAS COMMAND


        GAS_CMD = (rxBuf[0] << 8 | rxBuf[1] << 0); 
        // scale GAS_CMD into GAS_CMD_PERCENT
        if (GAS_CMD >= OP_MIN_GAS_COMMAND) {
          GAS_CMD1 = (GAS_CMD>OP_MAX_GAS_COMMAND ? OP_MAX_GAS_COMMAND : GAS_CMD);
        }
        else {
          GAS_CMD1 = OP_MIN_GAS_COMMAND;
        }
        GAS_CMD_PERCENT = ((100/(OP_MAX_GAS_COMMAND - OP_MIN_GAS_COMMAND)) * (GAS_CMD1 - OP_MIN_GAS_COMMAND));
        // calculating GAS_CMD_PERCENT into actuatorTargetPosition 
        if (!DEBUGMODE) startActuation((int)(((GAS_CMD_PERCENT / 100) * (ACTUATOR_MAX_POT - ACTUATOR_MIN_POT)) + ACTUATOR_MIN_POT));
      } 
      else if (rxId == 0x343) { // GAS_CMD / we want to extract the brake requests
        BRAKE_CMD = ((rxBuf[0] << 8 | rxBuf[1] << 0) * -1); 

        if (BRAKE_CMD > OP_MIN_BRAKE_COMMAND) {
          isBrakeRequest=true;
        }
        else {
          isBrakeRequest=false;
        }

          if (BRAKE_CMD >= OP_MIN_BRAKE_COMMAND) {
          BRAKE_CMD1 = (BRAKE_CMD>OP_MAX_BRAKE_COMMAND ? OP_MAX_BRAKE_COMMAND : BRAKE_CMD);
        }
        else {
          BRAKE_CMD1 = OP_MIN_BRAKE_COMMAND;
        }
        BRAKE_CMD_PERCENT = ((100/(OP_MAX_BRAKE_COMMAND - OP_MIN_BRAKE_COMMAND)) * (BRAKE_CMD1 - OP_MIN_BRAKE_COMMAND));
        // calculating BRAKE_CMD_PERCENT into actuatorTargetPosition 
        //if (!DEBUGMODE) brakeTargetPosition = (int)(((BRAKE_CMD_PERCENT / 100) * (maxBrakePot - minBrakePot)) + minBrakePot);
        // TODO: DEPENDING ON BRAKE_CMD_PERCENT play a warning sound through beeper if brake pedal is NOT pressed yet
      } 
  }
}


void loopThrottle() {
  if (cancelGasActuation || isBrakeRequest) {
    stopActuation();
    analogWrite(S_IN3_PIN, LOW); //open solenoid
  }
  else { // we're not braking, changing gears, have an OP brake request or having canbus problems: we can actuate
    digitalWrite(S_IN3_PIN, HIGH); // close solenoid
  }

  updateActuation();
}


void loopSafety() {
  if (digitalRead(CLUTCH_CANCEL_PIN))  // we implement a clutch-grace time to make sure we don't overrev the engine while releasing the clutch slowly
    lastClutchPressedTime=millis();

  if (NO_CLUCH_BRAKE_MODE)
    cancelGasActuation = ((!DEBUGMODE && (millis()-lastCanReceive>500 || millis()<lastCanReceive)));
  else
    cancelGasActuation = (digitalRead(CLUTCH_CANCEL_PIN) || millis()-lastClutchPressedTime<CLUTCH_RELEASE_GRACE_TIME_MS || digitalRead(BRAKE_CANCEL_PIN) || (!DEBUGMODE && (millis()-lastCanReceive>500 || millis()<lastCanReceive)));

  // TODO: implement angle sensor and eps specific safety checks
}


void loop() { 
  loopCruiseButtons(); // reads cruise button states
  loopUpdateVssSensor(); // checks for received vss / hall sensor interrupts, updates & smoothes speed
  loopReceiveCan(); // reads & parses can messages in the mcp receive buffers
  loopSendCan(); // sends can messages required by openpilot and toyota ecus
  loopSafety(); // checks pedal states, verifies can connection
  loopThrottle(); // reads throttle actuator position, handles throttle movement

  lastMainLoop=millis();

  if (lastMainLoop-lastMainLoopCurrentSecondMillis>=1000) {
    lastMainLoopPerSecond=lastMainLoopCurrentSecond;
    lastMainLoopCurrentSecond=0;
    lastMainLoopCurrentSecondMillis=lastMainLoop;
  }
  lastMainLoopCurrentSecond++;
  
  displayOled();
}