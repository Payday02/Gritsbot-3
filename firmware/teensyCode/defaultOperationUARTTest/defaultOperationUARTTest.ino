#include <GritsbotXUART.h>

GritsbotXUART myRobot;
int garbage[2];

void setup() {
  Serial.begin(500000);
  Serial1.begin(500000);
  myRobot.SETUP();
  myRobot.rainbow(10);
  myRobot.turnOffLED();
  myRobot.disableIR();
}

void loop() {
  // put your main code here, to run repeatedly:

  myRobot.getEncoderCounts(garbage);
  myRobot.checkCharging();
  if (myRobot.checkCharging()){ //If we're charging disable unnecessary current draws.
    myRobot.disableIR();
    myRobot.turnOffLED();
  }
  myRobot.checkBattVoltage();

  myRobot.jsonSerialRead();
  myRobot.communicationCheck(500);
  myRobot.followCommands();  
  Serial.print("Through the loop");
}
