// Determine the Motor Characteristics for GRITSBOTX 

// PIN DEFINITIONS 

// Motor A(left) and B(right) are connected to AIN1,AIN2 and BIN1 and BIN2 on the motor controller 

#include <Encoder.h>
#include "Wire.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
#define NUMBER_OF_SENSORS 8
//////DEFINE ENCODER OBJECTS FOR LEFT AND RIGHT WHEEL //////////////
Encoder motorLeft(21, 20);
Encoder motorRight(2,3);

//  Pin Definitions 
int STBY = 6; // motor driver enable 
int led = 13; // Teensy LED Pin 

// Motor L 
int PWMA = 23; // control the speed 
int AIN1 = 7; // direction 
int AIN2 = 8; // direction 

//Motor R 
int PWMB = 4; // speed control 
int BIN1 = 0; // direction 
int BIN2 = 1; // direction 


//Constants

float pi = 3.14159265359;

//Encoder Count Storage
int countR = 0; // Current count for RIGHT encoder.
int countL = 0; // Current count for LEFT encoder.

int oldCountR = 0;
int oldCountL = 0;

//Other Variables
float timeStart=0;
float timeStartStep=0;

float experimentTime = 60; //60 seconds.
float stepExperimentTime = 2000; // 2 Seconds in ms.

float samplingTime = 5; //10 ms sampling time.

//PID Stuff
float PIDMotorsTimeStart = 0;
float oldErrorL = 0;
float oldErrorR = 0;
float oldMotorPIDEncoderCountL = 0;
float oldMotorPIDEncoderCountR = 0;
float encoderCountsPerRotation = 12;
float motorGearRatio = 150.58;
float kpMotor = 6.01;
float kiMotor = 613;
float kdMotor = 0;
int motorL = 0;
int motorR = 0;

float desVel[5] = {random(-8,8), random(-8,8), random(-8,8), random(-8,8), random(-8,8)};//rad/s

int input = 0;
////////////////////// SETUP //////////////////////////////////
void setup() {
  // define the pins as outputs
  pinMode(STBY,OUTPUT);
  pinMode(led,OUTPUT);
  
  pinMode(PWMA,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);

  pinMode(PWMB,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);

  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second

  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  delay(1000);//Allow Time for Matlab to start.
  Wire.begin();
  //Initialize all the sensors
  for (byte x = 0 ; x < NUMBER_OF_SENSORS ; x++)
  {
    enableMuxPort(x); //Tell mux to connect to port X
    lox.begin();
    disableMuxPort(x);
  }
  while (!Serial) delay(10); 
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  if (!ina260.begin())
  {
    Serial.print("Ooops, no INA260 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  

  PIDMotorsTimeStart = millis();
}

//////////////// MAIN LOOP ////////////////////////////////////

void loop()                     
{ 



  for (int i = 0; i<5; i++){
    if (i == 0){
      timeStart = millis();
    }
    timeStartStep=millis();
    while (millis() - timeStartStep < stepExperimentTime){

    //Distance Sensor and MUX switch Start
    
    VL53L0X_RangingMeasurementData_t measure;
      int i = 0;
      Serial.println("");
      for (byte x = 0 ; x < NUMBER_OF_SENSORS ; x++)
      {    
        enableMuxPort(x); //Tell mux to connect to this port, and this port only
        lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
        if (measure.RangeStatus != 4) 
        {  // phase failures have incorrect data
          //Serial.print(" For "); Serial.print(i);Serial.print(" is:"); Serial.print(measure.RangeMilliMeter);Serial.print(".");
          Serial.print(measure.RangeMilliMeter);Serial.print(" ");
        } else 
        {
          Serial.print(" Out of range for ");Serial.print(i);Serial.print(".");
        }
        disableMuxPort(x); //Tell mux to disconnect from this port
        i++;
      }
      //Distance Sensor and MUX End

      //IMU reading Start
        sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
        bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

        printEvent(&orientationData);
        printEvent(&angVelocityData);
        printEvent(&linearAccelData);
        printEvent(&magnetometerData);
        printEvent(&accelerometerData);
        printEvent(&gravityData);

        int8_t boardTemp = bno.getTemp();
        Serial.println();
        Serial.print(F("temperature: "));
        Serial.println(boardTemp);

        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.println();
        Serial.print("Calibration: Sys=");
        Serial.print(system);
        Serial.print(" Gyro=");
        Serial.print(gyro);
        Serial.print(" Accel=");
        Serial.print(accel);
        Serial.print(" Mag=");
        Serial.println(mag);
        Serial.println("--");
      //IMU reading End

      //INA 260 Reading Start
        Serial.println();
        Serial.print("Current: ");
        Serial.print(ina260.readCurrent());
        Serial.println(" mA");

        Serial.print("Bus Voltage: ");
        Serial.print(ina260.readBusVoltage());
        Serial.println(" mV");

        Serial.print("Power: ");
        Serial.print(ina260.readPower());
        Serial.println(" mW");

        Serial.println();
      //INA 260 Reading End

      //Motor Controll Start
      PIDMotorControl(1,5);
      if (millis() - timeStart >= samplingTime){
        readEncoders();
        timeStart = millis();
        //Correct for wrap around
        if (countL < 0 && oldCountL > 0){
          countL = ((countL - (-2147483648)) + (2147483647 - oldCountL));
        }
        if (countR < 0 && oldCountR > 0){
          countR = ((countR - (-2147483648)) + (2147483647 - oldCountR)); // (rad)
        }
        oldCountR = countR;
        oldCountL = countL;
        Serial.print(int(countL));
        Serial.print(",");
        Serial.print(int(countR));
        Serial.print(",");
        Serial.print(int(millis()));
        Serial.print(",");
        Serial.println(desVel[i]);    
        Serial.flush();
       }
    }
  }
  

   stop();
   Serial.flush();
   Serial.print("Done");
   Serial.end();
}
////////////////// FUNCTIONS //////////////////////////////////
void moveR(int motorSpeed){
  //Assume forward at first
  boolean in1 = HIGH;
  boolean in2 = LOW;

  //If motor speed is negative we want to rotate wheel backwards
  if (motorSpeed < 0){
     in1 = LOW;
     in2 = HIGH;
  }
   
  // Truncate PWM input to 255
  if (abs(motorSpeed)>255){
    motorSpeed = 255;
  }

  // Move the motor.
  digitalWrite(STBY, HIGH);
  digitalWrite(BIN1,in1);
  digitalWrite(BIN2,in2);
  analogWrite(PWMB, abs(motorSpeed)); 
}

void moveL(int motorSpeed){
  //Assume forward at first
  boolean in1 = LOW;
  boolean in2 = HIGH;

  //If motor speed is negative we want to rotate wheel backwards
  if (motorSpeed < 0){
     in1 = HIGH;
     in2 = LOW;
  }
   
  // Truncate PWM input to 255
  if (abs(motorSpeed)>255){
    motorSpeed = 255;
  }

  // Move the motor.
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1,in1);
  digitalWrite(AIN2,in2);
  analogWrite(PWMA, abs(motorSpeed));
}

void brake(){
  //Brakes the motors. (Locked to not rotate passively, this can be overcome by enough torque)
  digitalWrite(STBY, HIGH);
  
  analogWrite(PWMA, LOW);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  
  analogWrite(PWMB, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

//Positive speed drives straight
void driveStraight(int motorSpeed){
  moveL(motorSpeed);
  moveR(motorSpeed);
}

//Positive speed turns right
void turn(int motorSpeed){
  moveL(motorSpeed);
  moveR(-motorSpeed);
}

void stop() {
  // Turns the motors off. (They can still rotate passively) 
  digitalWrite(STBY,LOW);
}

void readEncoders(){
  countR = -motorRight.read();
  countL = motorLeft.read();
}

void PIDMotorControl(float desLVel, float desRVel){
    /*Keeps the rotational speeds of the individual motors at setpoints desLVel and desRVel (rad/s).*/

    float timeStep = 10;

    if (millis() - PIDMotorsTimeStart >= timeStep){
      float PIDTimeStep = (millis() - PIDMotorsTimeStart)/1000;//Time step for controller to work on (s).

      readEncoders();

      // Error on individual motors for vel control
      float errorL = desLVel - 2 * pi * (countL - oldMotorPIDEncoderCountL) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      float errorR = desRVel - 2 * pi * (countR - oldMotorPIDEncoderCountR) / (encoderCountsPerRotation * motorGearRatio * PIDTimeStep);
      float integralL = integralL + errorL * PIDTimeStep;
      float integralR = integralR + errorR * PIDTimeStep;
      float diffL = (oldErrorL - errorL) / PIDTimeStep;
      float diffR = (oldErrorR - errorR) / PIDTimeStep;
      oldErrorL = errorL;
      oldErrorR = errorR;
      oldMotorPIDEncoderCountL = countL;
      oldMotorPIDEncoderCountR = countR;

      motorL += int((kpMotor*errorL + kiMotor*integralL + kdMotor*diffL));
      motorR += int((kpMotor*errorR + kiMotor*integralR + kdMotor*diffR));

      moveL(motorL);
      moveR(motorR);
      
      PIDMotorsTimeStart = millis();
    }
}
void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}
