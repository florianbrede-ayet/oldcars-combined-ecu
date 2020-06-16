#include <Arduino.h>
#include "mcp_can.h"
#include "Display.h"
#include "globals.h"

#define VERSION 1
#define ENABLE_OLED 1 
#define OLED_REFRESH_MS 100

/* V1.0 BMW Actuator 
#define lastOledRefresh=0;ECU throttle & cruise combined
This sketch can be used to control a BMW throttle valve actuator over CAN and will also handle the cruise part (speed signal, buttons, cruise state). 
original source: https://github.com/Lukilink/actuator_ECU
*/


const int CAN_CS_PIN = 53;
const int CAN_INT_PIN = 21;
MCP_CAN CAN(CAN_CS_PIN);                                    // Set CS pin
unsigned char flagCANRecv = 0;


//////////////////// THROTTLE CONTROL PART ////////////////////

// ACTUATOR SETUP:
//________________THIS IS THE CONFIGURATION FOR A BMW/VDO "8 369 027" / "408.201/013/001" ACTUATOR with 30mm actuation distance
int PERM_ERROR = 25; //will allow a diffrence between targetPressure and currentPressure
int minPot = 20; //measured at actuators lowest position
int maxPot = 2300; //measured at actuators highest position
const int SLOW_MOVE_PWM=200;

// ACTUATOR POTENTIOMETER:
int potPin = A3; // connect the potentiometer of your cars throttle
int potReferenceResistor = 1000; // we measure the resistance of the potentiometer - this is the reference resistor used to cover the ~2.4k ohms potentiometer range

unsigned long lastCanReceive=0L;


// SAFETY PINS
int cancel_pin_clutch = 46; //pulled to GND through default open hall sensor when CLUTCH pedal is NOT pressed
int cancel_pin_brake = 47; //pulled to GND through default open hall sensor when BRAKE pedal is NOT pressed
unsigned long lastClutchPressedTime = 0L;

const int CLUTCH_RELEASE_GRACE_TIME_MS = 500; // amount of ms to delay before giving throttle after shifting for example

// H BRIDGE PINS
int M_IN1 = 7; // motor direction (7H, 8L = left, 7L, 8H = right)
int M_IN2 = 8; // motor direction (7H, 8L = left, 7L, 8H = right)
int M_ENA = 9; // 255 is run / LOW is stopp   // motor speed
// we bridge ENB and just control IN3 to set the clutch closed or to 0 again to open
int S_IN3 = 6; // 255 is run / LOW is stopp   // SOLENOID / actuator clutch!

// OPENPILOT CONFIG 
float maxACC_CMD = 1194; //the max Value which comes from OP on CAN ID 0x200 (actually higher, it's being clipped)
float minACC_CMD = 500; //the min Value which comes from OP on CAN ID 0x200 (actually lower, but we clip anything below which will result in "0" throttle)


//________________throttle variables
int targetPosition = 0;
int potiPosition = 0;
float ACC_CMD_PERCENT = 0;
float ACC_CMD = 0;
float ACC_CMD1 = 0;
boolean cancel = false;


//////////////////// CRUISE PART ////////////////////


//______________BUTTONS AND SWITCHES (PULLDOWN INPUT)
int button3 = 42;  // SPD+
int button2 = 43;  // SPD-
int button1 = 44;  // ON/OFF
int button4 = 45;  // currently unused / LKAS later

boolean pedalstate = false;


int buttonstate3;
int lastbuttonstate3;
unsigned long debounceTime3 = 0;

int buttonstate2;
int lastbuttonstate2;
unsigned long debounceTime2 = 0;

int buttonstate1;
int lastbuttonstate1;
unsigned long debounceTime1 = 0;

//______________VALUES SEND ON CAN
unsigned long lastCanSend=0L;

boolean OP_ON = false;
uint8_t set_speed = 90;
uint8_t last_set_speed = 90;

int gas_pedal_state = 0; // TODO: Remove gas_pedal_state
int brake_pedal_state = 0; // TODO: Remove brake_pedal_state
double average = 0; 
boolean blinker_left = true;
boolean blinker_right = true;

//______________FOR SMOOTHING SPD
const int numReadings = 160;
float readings[numReadings];
int readIndex = 0; 
double total = 0;

//______________FOR READING VSS SENSOR
const byte interruptPin = 3;
int inc = 0;
int half_revolutions = 0;
int spd;
unsigned long lastmillis;
unsigned long duration;
uint8_t encoder = 0;


boolean DEBUGMODE=false;
boolean NO_CLUCH_BRAKE_MODE=false;



// DISPLAY / STATISTICS related variables
char msgString[65];
boolean isFirstRender=true;
unsigned long lastOledRefresh=0;
int subOledRefresh=0;
int canMessagesPerSecond=0;
int canMessagesCurrentSecond=0;
unsigned long canMessagesCurrentSecondMillis=0;


unsigned long lastMainLoop=0;
unsigned long lastMainLoopCurrentSecondMillis=0;

int lastMainLoopPerSecond=0;
int lastMainLoopCurrentSecond=0;
  
int motorStatus=0;

int potiRingbuffer[128];
int potiRingbufferPosition=0;
unsigned long lastRingbufferRefresh=0;

#if ENABLE_OLED
Display lcd = Display();
#endif




int getResistance() {
  int potRaw = analogRead(potPin);
  if(potRaw) 
  {
    float buffer=potRaw * 5.0f;
    float Vout = (buffer)/1024.0;
    buffer = (5.0f/Vout) - 1;
    return (int)(potReferenceResistor * buffer);
  }
  return 0;
}


void rpm() {
  half_revolutions++;
  if (encoder > 255)
  {
    encoder = 0;
  }
  encoder++;
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




void setupThrottle() {
  //________________set up pin modes
  pinMode(cancel_pin_clutch, INPUT_PULLUP);
  pinMode(cancel_pin_brake, INPUT_PULLUP);
  pinMode(potPin, INPUT);    
  
  pinMode(M_IN1, OUTPUT);
  pinMode(M_IN2, OUTPUT);
  pinMode(M_ENA, OUTPUT);
  digitalWrite(M_IN1, LOW);
  digitalWrite(M_IN2, LOW);
  digitalWrite(M_ENA, LOW);

  pinMode(S_IN3, OUTPUT);
  digitalWrite(S_IN3, LOW);


  

  // UNCOMMENT FOR DEBUGGING
  /* Serial.println("OPEN / STOP");
  getResistance();
  while(1) {
    digitalWrite(S_IN3, HIGH);
    Serial.println("CLOSE SOLENOID");
    delay(5000);

    digitalWrite(M_IN1, HIGH); //motor driection left
    digitalWrite(M_IN2, LOW); //motor driection left
    
    analogWrite(M_ENA, 255);  //run Motor
    Serial.println("RUN MOTOR LEFT until end");
    while(1) {
      int res = getResistance();
      if (res>=maxPot) break;
      delay(1);
    }
    analogWrite(M_ENA, 0);  //stop Motor
    Serial.print("FINAL RESISTANCE  ");
    Serial.println(getResistance());
    delay(1000);
    Serial.print("FINAL RESISTANCE  AFTER 1s sleep   ");
    Serial.println(getResistance());
    delay(1000);


    digitalWrite(M_IN1, LOW); //motor driection right
    digitalWrite(M_IN2, HIGH); //motor driection right
    analogWrite(M_ENA, 255);  //run Motor
    Serial.println("RUN MOTOR RIGHT until end");
    while(1) {
      int res = getResistance();
      if (res<=minPot) break;
      delay(1);
    }
    delay(5000);
  
    Serial.print("END CYCLE RESISTANCE  ");
    Serial.println(getResistance());
  } */
}


void setupCruise() {
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rpm, FALLING);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);

  //______________initialize smoothing inputs
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}



void displayOled() {
  #if ENABLE_OLED
  if (millis()-lastOledRefresh<OLED_REFRESH_MS && !DEBUGMODE)
    return;

  lastOledRefresh=millis();

  if (isFirstRender) {
    lcd.clearVideoBuffer();
    sprintf(msgString, "RetroPilot V%d -", VERSION);
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
      sprintf(msgString, "%d", (int)(millis()/1000));  
      lcd.drawString(95, 0, msgString);
    }
  }

  if (subOledRefresh==1) {
    sprintf(msgString, "ON:%d S:%-3d GAS:%3d%%   ", OP_ON, set_speed, (int)ACC_CMD_PERCENT);
    lcd.drawString(0, 15, msgString);
  }

  if (subOledRefresh==2) {  
    sprintf(msgString, "P:%-4d T:%-4d O:%-4d   ", potiPosition, targetPosition, abs(targetPosition-potiPosition));
    lcd.drawString(0, 26, msgString);
  }

  if (subOledRefresh==3) {
    sprintf(msgString, "CAN/s:%d  LOOP/s:%d", canMessagesPerSecond, lastMainLoopPerSecond);
    lcd.drawString(0, 37, msgString);
  }

  if (subOledRefresh==4) {
    sprintf(msgString, "BRA:%d  CLU:%d  MOT:%d   ", digitalRead(cancel_pin_brake), digitalRead(cancel_pin_clutch), motorStatus);
    lcd.drawString(0, 48, msgString);
    /* int positionHelper=potiRingbufferPosition;
    lcd.clearRect(0,47,127,63);
    for (int i=127; i>=0; i--) {
      lcd.drawPixel(i, 63-potiRingbuffer[positionHelper]);
      positionHelper--;
      if (positionHelper<0) positionHelper=127;      
    } */
  }

  subOledRefresh++;
  if (subOledRefresh>=5) subOledRefresh=0;
  
  if (subOledRefresh==1)
    lcd.show();

  #endif

}


void setup() {

  #if ENABLE_OLED
  lcd.initSoftSPI(100);
  lcd.clearVideoBuffer();
  lcd.drawString(10, 10, "Starting RetroPilot 2560...");
  lcd.show();
  #endif

  //________________begin Monitor - only use it for debugging
  Serial.begin(115200);

  //________________begin CAN
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
  

  setupThrottle();

  #if ENABLE_OLED
  lcd.drawString(10, 30, "Init Throttle... OK"); lcd.show();
  #endif

  setupCruise();
  #if ENABLE_OLED
  lcd.drawString(10, 40, "Init Cruise... OK"); lcd.show();
  #endif

  delay(1000);
  buttonstate3 = digitalRead(button3);
  buttonstate2 = digitalRead(button2);
  buttonstate1 = digitalRead(button1);

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



void loopCruise() {
  //______________READ SPD SENSOR
  /* attachInterrupt(1,rpm, FALLING);

  if (half_revolutions >= 1) {
      detachInterrupt(1);
      duration = (micros() - lastmillis);
      spd = half_revolutions * (0.000135 / (duration * 0.000001)) * 3600;
      lastmillis = micros(); 
      half_revolutions = 0;
      attachInterrupt(1, rpm, FALLING);
    }

  //______________SMOOTH SPD TO AVERAGE
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = spd;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings; */
  // send it to the computer as ASCII digits

  //______________READING BUTTONS AND SWITCHES
  buttonstate3 = digitalRead(button3);
  buttonstate2 = digitalRead(button2);
  buttonstate1 = digitalRead(button1);

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
      set_speed = last_set_speed;
      
    }
    lastCanSend=0L;
  }

  if (buttonstate2==LOW && debounceTime2!=0 && (millis()-debounceTime2>=50L || millis()<debounceTime2)) {
    debounceTime2=0;
    if (!DEBUGMODE && set_speed>30) {
      set_speed -= 5;
      last_set_speed=set_speed;
    }
  }

  if (DEBUGMODE && buttonstate2==LOW) {
      targetPosition=targetPosition-20;
      if (targetPosition<minPot) targetPosition=minPot;
  }

  if (buttonstate3==LOW && debounceTime3!=0 && (millis()-debounceTime3>=50L || millis()<debounceTime3)) {
    debounceTime3=0;
    if (!DEBUGMODE && set_speed<150) {
      set_speed += 5;
      last_set_speed=set_speed;
    }
  }


  if (DEBUGMODE && buttonstate3==LOW) {
    targetPosition=targetPosition+20;
      if (targetPosition>maxPot) targetPosition=maxPot;
  }



  if ((millis()-lastCanSend>=20L || millis()<lastCanSend) && !DEBUGMODE) {
    lastCanSend=millis();
    //______________SENDING_CAN_MESSAGES
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
    /* CAN.beginPacket(0x1d2);
    for (int ii = 0; ii < 8; ii++) {
      CAN.write(dat[ii]);
    }
    CAN.endPacket(); */
    CAN.sendMsgBuf(0x1d2, 0, 8, dat);

    //0x1d3 msg PCM_CRUISE_2
    uint8_t dat2[8];
    dat2[0] = 0x0;
    dat2[1] = (OP_ON << 7) & 0x80 | 0x28;
    dat2[2] = set_speed;
    dat2[3] = 0x0;
    dat2[4] = 0x0;
    dat2[5] = 0x0;
    dat2[6] = 0x0;
    dat2[7] = can_cksum(dat2, 7, 0x1d3);
    /* CAN.beginPacket(0x1d3);
    for (int ii = 0; ii < 8; ii++) {
      CAN.write(dat2[ii]);
    }
    CAN.endPacket(); */
    CAN.sendMsgBuf(0x1d3, 0, 8, dat2);

    //0xaa msg defaults 1a 6f WHEEL_SPEEDS
    average=25;
    uint8_t dat3[8];
    uint16_t wheelspeed = 0x1a6f + (average * 100);
    dat3[0] = (wheelspeed >> 8) & 0xFF;
    dat3[1] = (wheelspeed >> 0) & 0xFF;
    dat3[2] = (wheelspeed >> 8) & 0xFF;
    dat3[3] = (wheelspeed >> 0) & 0xFF;
    dat3[4] = (wheelspeed >> 8) & 0xFF;
    dat3[5] = (wheelspeed >> 0) & 0xFF;
    dat3[6] = (wheelspeed >> 8) & 0xFF;
    dat3[7] = (wheelspeed >> 0) & 0xFF;
    /* CAN.beginPacket(0xaa);
    for (int ii = 0; ii < 8; ii++) {
      CAN.write(dat3[ii]);
    }
    CAN.endPacket(); */
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
    /* CAN.beginPacket(0x3b7);
    for (int ii = 0; ii < 8; ii++) {
      CAN.write(dat5[ii]);
    }
    CAN.endPacket(); */
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
    /* CAN.beginPacket(0x620);
    for (int ii = 0; ii < 8; ii++) {
      CAN.write(dat6[ii]);
    }
    CAN.endPacket(); */
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
    /* CAN.beginPacket(0x3bc);
    for (int ii = 0; ii < 8; ii++) {
      CAN.write(dat7[ii]);
    }
    CAN.endPacket(); */
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
    /* CAN.beginPacket(0x2c1);
    for (int ii = 0; ii < 8; ii++) {
      CAN.write(dat10[ii]);
    }
    CAN.endPacket(); */
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
    /* CAN.beginPacket(0x224);
    for (int ii = 0; ii < 8; ii++) {
      CAN.write(dat11[ii]);
    }
    CAN.endPacket(); */
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
    /* CAN.beginPacket(0x614);
    for (int ii = 0; ii < 8; ii++) {
      CAN.write(dat614[ii]);
    }
    CAN.endPacket();  */
    CAN.sendMsgBuf(0x614, 0, 8, dat614);


    //0x262 fake EPS_STATUS
    uint8_t dat8[8];
    dat8[0] = 0x0;
    dat8[1] = 0x0;
    dat8[2] = 0x0;
    dat8[3] = 0x3;
    dat8[4] = 0x6c;

    /* CAN.beginPacket(0x262);
    for (int ii = 0; ii < 5; ii++) {
      CAN.write(dat8[ii]);
    }
    CAN.endPacket(); */
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
      
    /* CAN.beginPacket(0x260);
    for (int ii = 0; ii < 8; ii++) {
      CAN.write(dat9[ii]);
    }
    CAN.endPacket(); */
    CAN.sendMsgBuf(0x260, 0, 8, dat9);
  }
}

void loopThrottle() {
  //________________check cancel conditions

  if (digitalRead(cancel_pin_clutch))  // we implement a clutch-grace time to make sure we don't overrev the engine while releasing the clutch slowly
    lastClutchPressedTime=millis();

  if (NO_CLUCH_BRAKE_MODE)
    cancel = ((!DEBUGMODE && (millis()-lastCanReceive>500 || millis()<lastCanReceive)));
  else
    cancel = (digitalRead(cancel_pin_clutch) || millis()-lastClutchPressedTime<CLUTCH_RELEASE_GRACE_TIME_MS || digitalRead(cancel_pin_brake) || (!DEBUGMODE && (millis()-lastCanReceive>500 || millis()<lastCanReceive)));

  //________________read poti Position
  potiPosition = (int)(((float)(getResistance()+getResistance()+getResistance()+getResistance()+getResistance()))/5.0);

  if (millis()-lastRingbufferRefresh>=50) {
    lastRingbufferRefresh=millis();
    potiRingbuffer[potiRingbufferPosition]=min(max(0, ((float)potiPosition/(float)maxPot)*16.0),16);
    potiRingbufferPosition++;
    if (potiRingbufferPosition>127) potiRingbufferPosition=0;
  }

  //________________read ACC_CMD from CANbus
     flagCANRecv = 0;                   // clear flag
    while (CAN_MSGAVAIL == CAN.checkReceive())
    {
        // read data,  len: da
        long unsigned int rxId;
        uint8_t len = 0;
        uint8_t rxBuf[8];
        CAN.readMsgBuf(&len, rxBuf);
        rxId=CAN.getCanId();

        if (rxId == 0x200) {
          lastCanReceive=millis();

          if (lastCanReceive-canMessagesCurrentSecondMillis>=1000) {
            canMessagesPerSecond=canMessagesCurrentSecond;
            canMessagesCurrentSecond=0;
            canMessagesCurrentSecondMillis=lastCanReceive;
          }
          canMessagesCurrentSecond++;

          ACC_CMD = (rxBuf[0] << 8 | rxBuf[1] << 0); 
          //________________calculating ACC_CMD into ACC_CMD_PERCENT
          if (ACC_CMD >= minACC_CMD) {
            ACC_CMD1 = (ACC_CMD>maxACC_CMD ? maxACC_CMD : ACC_CMD);
          }
          else {
            ACC_CMD1 = minACC_CMD;
          }
          ACC_CMD_PERCENT = ((100/(maxACC_CMD - minACC_CMD)) * (ACC_CMD1 - minACC_CMD));
          //________________calculating ACC_CMD_PERCENT into targetPosition 
          if (!DEBUGMODE) targetPosition = (int)(((ACC_CMD_PERCENT / 100) * (maxPot - minPot)) + minPot);
        } 
        
    } 


  

  //________________do nothing if ACC_CMD is 0
  if (cancel || (!DEBUGMODE && ACC_CMD_PERCENT == 0)) {
    analogWrite(S_IN3, 0);  //open solenoid
    analogWrite(M_ENA, 0);  //stop Motor
    motorStatus=0;
  }
  else { // we're not braking or changing gears / we can actuate
    //________________close solenoid
    digitalWrite(S_IN3, HIGH);

    //________________press or release the pedal to match targetPosition & respect endpoints
    if (abs(potiPosition - targetPosition) >= PERM_ERROR)
    {
      if ((potiPosition < targetPosition) && (potiPosition < maxPot)) //if we are lower than target and not at endpoint
      { 
        digitalWrite(M_IN1, HIGH); digitalWrite(M_IN2, LOW); //motor left (== pull wire)
        if (abs(potiPosition - targetPosition)>100) {
          analogWrite(M_ENA, 255);  //run Motor
          motorStatus=255;
        } else {
          analogWrite(M_ENA, SLOW_MOVE_PWM);  //run Motor
          motorStatus=SLOW_MOVE_PWM;
        }
      }    
      else if ((potiPosition > targetPosition) && (potiPosition > minPot)) //if we are higher than target and not at endpoint
      {       
        digitalWrite(M_IN1, LOW); digitalWrite(M_IN2, HIGH); //motor right (== loosen wire)
        if (abs(potiPosition - targetPosition)>100) {
          analogWrite(M_ENA, 255);  //run Motor
          motorStatus=-255;
        } else {
          analogWrite(M_ENA, SLOW_MOVE_PWM);  //run Motor
          motorStatus=-SLOW_MOVE_PWM;
        }

      }
      else {
        analogWrite(M_ENA, 0);   //stop Motor (out of bounds stop)
        motorStatus=0;
      }
    }  
    //________________if we match target position, just stay here
    else {
      analogWrite(M_ENA, 0);   //stop Motor
      motorStatus=0;
    }  
  }
  //________________print stuff if you want to DEBUG
  
/*    Serial.print("ACC_CMD_");
  Serial.print(ACC_CMD);
  Serial.print("_____");
  Serial.print(ACC_CMD_PERCENT);
  Serial.print("_%");
  Serial.print("_____");
  Serial.print("target_");
  Serial.print(targetPosition);
  Serial.print("_____");
  Serial.print("Position_");
  Serial.print(potiPosition);
  Serial.print("_____");
  Serial.print("offset_");
  Serial.print(abs(targetPosition-potiPosition));
  Serial.print("_____");
  Serial.print("clutch_");
  Serial.print(digitalRead(cancel_pin_clutch));
  Serial.print("_____");
  Serial.print("brake_");
  Serial.print(digitalRead(cancel_pin_brake));
  Serial.print("_____");
  Serial.print("cants_");
  Serial.print(millis()-lastCanReceive);
  Serial.print("_____");
  Serial.print("clutchts_");
  Serial.print(millis()-lastClutchPressedTime);
  Serial.println(""); 
 */
}

void loop() { 
  loopCruise();
  loopThrottle();

  lastMainLoop=millis();

  if (lastMainLoop-lastMainLoopCurrentSecondMillis>=1000) {
    lastMainLoopPerSecond=lastMainLoopCurrentSecond;
    lastMainLoopCurrentSecond=0;
    lastMainLoopCurrentSecondMillis=lastMainLoop;
  }
  lastMainLoopCurrentSecond++;
  
  displayOled();
}