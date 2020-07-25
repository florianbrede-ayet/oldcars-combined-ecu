#include <Arduino.h>
#include "Display.h"
#include "RetropilotCanManager.h"
#include "RetropilotParams.h"
#include "RetropilotInputs.h"
#include "RetropilotVSS.h"
#include "RetropilotActuatorThrottle.h"


#include "globals.h"




RetropilotCanManager retropilotCanManager;

#if MODULE_INPUTS
RetropilotInputs retropilotInputs;
#endif
#if MODULE_VSS
RetropilotVSS retropilotVSS;
#endif
#if MODULE_THROTTLE_ACTUATOR
RetropilotActuatorThrottle retropilotActuatorThrottle;
#endif
#if MODULE_BRAKE_ACTUATOR
RetropilotActuatorBrake retropilotActuatorBrake;
#endif
#if MODULE_EPS
RetropilotEPS retropilotEPS;
#endif



// DISPLAY / STATISTICS related variables
char msgString[65];
boolean isFirstRender=true;
unsigned long lastOledRefresh=0;
int subOledRefresh=0;

unsigned long lastMainLoop=0;
unsigned long lastMainLoopCurrentSecondMillis=0;

int lastMainLoopPerSecond=0; // tracks the number of loops() processed per second. target is around 500/s (if can network is active!)
int lastMainLoopCurrentSecond=0;
  

#if ENABLE_OLED
Display lcd = Display();
#endif





void displayOled() {
/*   #if ENABLE_OLED
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
      sprintf(msgString, "%d", (int)(millis()/1000));  
      lcd.drawString(87, 0, msgString);
    }
  }

  if (subOledRefresh==1) {
    sprintf(msgString, "O:%d C:%-3d V:%-3d G:%3d%%   ", retropilotParams.OP_ON, retropilotParams.ccSetSpeed, (int)retropilotParams.vssAvgSpeedKMH, ((int)retropilotParams.GAS_CMD_PERCENT));
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

  #endif */
}




void setup() {
  delay(2000);

  #if ENABLE_OLED
  lcd.initI2C(100);
  lcd.clearVideoBuffer();
  lcd.drawString(10, 0, "Starting RetroPilot STM32...");
  lcd.show();
  #endif

  Serial.begin(115200);


  retropilotCanManager.setup();

  #if MODULE_INPUTS
  retropilotInputs.setup();
  #endif

  #if MODULE_VSS
    retropilotVSS.setup();
  #endif
  #if MODULE_THROTTLE_ACTUATOR
    retropilotActuatorThrottle.setup();
  #endif
  #if MODULE_BRAKE_ACTUATOR
    retropilotActuatorBrake.setup();
  #endif
  #if MODULE_EPS
    retropilotEPS.setup();
  #endif


  #if ENABLE_OLED
  lcd.drawString(10, 10, "Init Modules... OK"); lcd.show();
  #endif

  
  displayOled();
}


void loopSafety() {

  retropilotParams.OP_FAULTY_ECU = (millis()-retropilotCanManager.lastModuleErrorFlagReceive<1000);

  retropilotParams.OP_ERROR_CAN = ((!retropilotParams.DEBUGMODE && 
                                      (millis()-retropilotCanManager.lastCanReceiveSteerCommand>50 || millis()-retropilotCanManager.lastCanReceiveSteeringAngle>50 || millis()-retropilotCanManager.lastCanReceive>100 || millis()-retropilotCanManager.lastCanReceiveModuleInputs>100 || millis()-retropilotCanManager.lastCanReceiveModuleVSS>100 
                                      || millis()-retropilotCanManager.lastCanReceiveModuleActuatorThrottle>100 || millis()-retropilotCanManager.lastCanReceiveModuleActuatorBrake>100 || millis()-retropilotCanManager.lastCanReceiveModuleEPS>100)
                                    ));


  retropilotParams.ALLOW_THROTTLE = retropilotParams.OP_ON && !retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR && !retropilotParams.OP_FAULTY_ECU && !retropilotParams.OP_ERROR_CAN && !retropilotParams.OP_BRAKE_PRESSED && !retropilotParams.OP_CLUTCH_PRESSED && retropilotParams.BRAKE_CMD_PERCENT==0;
  retropilotParams.ALLOW_BRAKE    = retropilotParams.OP_ON && !retropilotParams.OP_WHEELLOCK_DETECTED && !retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR && !retropilotParams.OP_FAULTY_ECU && !retropilotParams.OP_ERROR_CAN && !retropilotParams.OP_CLUTCH_PRESSED && !retropilotParams.OP_GAS_PRESSED && retropilotParams.BRAKE_CMD_PERCENT>0;
  retropilotParams.ALLOW_STEERING = retropilotParams.OP_ON && !retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR && !retropilotParams.OP_FAULTY_ECU && !retropilotParams.OP_ERROR_CAN && retropilotParams.OP_LKAS_ENABLED;
    
  // whenever there is any actual error, make sure to send OP_ON=false on the canbus so openpilot knows it's not supposed to try engaging
  if (retropilotParams.UNRECOVERABLE_CONFIGURATION_ERROR || retropilotParams.OP_FAULTY_ECU || retropilotParams.OP_ERROR_CAN) {   
    retropilotParams.OP_ON=false;
    retropilotParams.OP_LKAS_ENABLED=false;
  }

  // TODO: implement angle sensor and eps specific safety checks
}


void loop() { 


  retropilotCanManager.loopReceive();
  loopSafety(); // checks pedal states, verifies can connection / disables operation for each ecu whenever an error state is detected
  retropilotCanManager.loopSend();
  
  #if MODULE_INPUTS
  retropilotInputs.loop();
  #endif
  
  #if MODULE_VSS
  retropilotVSS.loop();
  #endif

  #if MODULE_ACTUATOR_THOTTLE
  retropilotActuatorThrottle.loop();
  #endif

  #if MODULE_ACTUATOR_BRAKE
  retropilotActuatorBrake.loop();
  #endif

  #if MODULE_ACTUATOR_EPS
  retropilotEPS.loop();
  #endif


  lastMainLoop=millis();

  if (lastMainLoop-lastMainLoopCurrentSecondMillis>=1000) {
    lastMainLoopPerSecond=lastMainLoopCurrentSecond;
    lastMainLoopCurrentSecond=0;
    lastMainLoopCurrentSecondMillis=lastMainLoop;
  }
  lastMainLoopCurrentSecond++;
  
  displayOled();
}