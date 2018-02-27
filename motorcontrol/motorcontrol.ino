#include <Servo.h>
//#include "MegunoLink.h"

//TimePlot data;

//
// Pin assignments
//
const int analogInPin = A0; // VO of ACS712 current sensor1 goes to pin AO on Arduino
const int analogIn2Pin = A1; // VO of ACS712 current sensor2 goes to pin A1 on Arduino
const int analogIn3Pin = A2; // VO of ACS715 current sensor1 goes to pin A2 on Arduino
const int analogIn4Pin = A3; // VO of ACS715 current sensor2 goes to pin A3 on Arduino

// const byte leftSensorPin_A  = 2;
// const byte rightSensorPin_A = 3;
// const byte leftSensorPin_B = 4 ; 
// const byte rightSensorPin_B = 5 ; 

// const byte leftPWMPin_O  = 10;
// const byte rightPWMPin_O = 9;

const byte leftSensorPin_A  = 3;
const byte rightSensorPin_A = 2;
const byte leftSensorPin_B = 5 ; 
const byte rightSensorPin_B = 4 ; 

const byte leftPWMPin_O  = 9;
const byte rightPWMPin_O = 10;

//
// Constants
//
const int avgSamples = 10; //for ACS712 current sensor
const unsigned int maxRPM = 120; 
const unsigned int wheelCircum = 785; //mm (Wheel diameter is 25cm)
const unsigned int pulsesPerRevolution = 500; //encoder rmp changed to 500 from 360 for new encoders
const unsigned int pidPeriod = 100; //ms

//
// Create Servo objects
//

Servo leftWheelServo;
Servo rightWheelServo;

//
// Global variables
//

volatile int timeout = 0 ; 

//Common to both battery sensors
float sensitivity = 100.0 / 500.0; //100mA per 500mV = 0.2
float Vref = 1500; // Output voltage with no current: ~ 1500mV or 1.5V
unsigned long msec = 0;
float time = 0.0;
//for ACS712 Battery Current 1 sensor
int sensorValue = 0; // value read from the carrier board
int sample = 0;

float totalCharge1 = 0.0;
float averageAmps1 = 0.0;
float ampSeconds1 = 0.0;
float ampHours1 = 0.0;
float wattHours1 = 0.0;
float BatteryAmps1 = 0.0;
float batteryVoltage1 = 0;
//for ACS712 Battery Current 2 sensor
int sensorValue1 = 0; // value read from the carrier board
int sample1 = 0;
float totalCharge2 = 0.0;
float averageAmps2 = 0.0;
float ampSeconds2 = 0.0;
float ampHours2 = 0.0;
float wattHours2 = 0.0;
float BatteryAmps2 = 0.0;
float batteryVoltage2 = 0;
//for ACS715 Motor Current sensor 1
int sensorValue2 = 0;        // value read from the carrier board
float motoramps1 = 0.0;     
int outputValue1 = 0;        //value in milliamps
//for ACS715 Motor Current sensor 2
int sensorValue3 = 0;        // value read from the carrier board
float motoramps2 = 0.0;
int outputValue2 = 0;

float leftKp  = 0.36;
float leftKi  = 0.36;
float rightKp = 0.36;
float rightKi = 0.36;

volatile int leftSensorPIDCount  = 0;
volatile int rightSensorPIDCount = 0;

volatile int leftSensorDistanceCount = 0;
volatile int rightSensorDistanceCount = 0;

volatile int leftError = 0;
volatile int rightError = 0;

volatile int leftLastError = 0;
volatile int rightLastError = 0;

volatile int leftOffset = 0;
volatile int rightOffset = 0;

volatile int desiredSpeed = 60;

volatile int desiredLeftPIDCount = 0;
volatile int desiredRightPIDCount = 0;

boolean leftDirection = 0; // 0 - forward, 1 - backward
boolean rightDirection = 0; // 0 - forward, 1 - backward

int skip = 0;
int leftSerialSpeed = 0;
int rightSerialSpeed = 0;

int timercount = 0;
int sendcount = 0;

char startByte;
char leftHex[4] = {0,0,0,'\0'};
char rightHex[4] = {0,0,0,'\0'};
char recNum;
char endByte;

String leftCountString;
String rightCountString;

char dump;
char *ptr;

int leftCurrent;
int rightCurrent;


//
// Intial Setup
//

void setup() {

  // Set up hardware interrupts
  attachInterrupt(digitalPinToInterrupt(leftSensorPin_A) , leftSensorISR , RISING);
  attachInterrupt(digitalPinToInterrupt(rightSensorPin_A), rightSensorISR, RISING);

  // Set up timer interrupt
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  OCR2A = 124; // Compare value

  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 << CS21) | (0 << CS20); // 1024 prescaler
  TIMSK2 |= (1 << OCIE2A);
  
  // Set up outputs
  leftWheelServo.attach(leftPWMPin_O);
  rightWheelServo.attach(rightPWMPin_O);

  leftWheelServo.write(90);
  rightWheelServo.write(90);

  // Initialise serial communication
  Serial.begin(19200);
}

//
// Hardware Interrupts for counting rotary encoder pulses
//

void leftSensorISR () {
  if(leftSensorPin_B == leftSensorPin_A) {
    leftSensorPIDCount --;
    leftSensorDistanceCount --;
  } else {
    leftSensorPIDCount ++;    
    leftSensorDistanceCount ++;
  }
}

void rightSensorISR () {
  if(rightSensorPin_B == rightSensorPin_A) {
    rightSensorPIDCount ++;
    rightSensorDistanceCount ++;
  } else {
    rightSensorPIDCount --;
    rightSensorDistanceCount --;
  }
}


//
// PID Controller - for both wheels 
//

ISR (TIMER2_COMPA_vect) {

  sei(); //Re-enable all interrupts

  if (timercount >= 49) {

  timeout++ ; 

  leftError = desiredLeftPIDCount - leftSensorPIDCount;
  rightError = desiredRightPIDCount - rightSensorPIDCount;

  leftSensorPIDCount = 0;
  rightSensorPIDCount = 0;

  leftOffset  += (leftKp  * (leftError  - leftLastError )) + (leftKi  * leftError );
  rightOffset += (rightKp * (rightError - rightLastError)) + (rightKi * rightError);

  leftLastError = leftError;
  rightLastError = rightError;
  
  leftOffset = constrain(leftOffset, -60, 90); 
  rightOffset = constrain(rightOffset, -60, 90);
   
  leftWheelServo.write(90 + leftOffset);
  rightWheelServo.write(90 + rightOffset);

    leftCountString = String(abs(leftSensorDistanceCount), HEX);
    rightCountString = String(abs(rightSensorDistanceCount), HEX);

    if (leftSensorDistanceCount >= 0) {
      Serial.print("+");
    } else {
      Serial.print("-");
    }
        
    Serial.print(leftCountString);
    Serial.print(",");

    if (rightSensorDistanceCount >= 0) {
      Serial.print("+");
    } else {
      Serial.print("-");
    }
    Serial.println(rightCountString);
    
    leftSensorDistanceCount = 0;
    rightSensorDistanceCount = 0;
  
  timercount = 0;
  } else {
    timercount++;
  }

}
// ACS 712 sensor for sensing Battery current, charge and power
void BatteryCurrentSensor1() {
  int avgBVal = 12;
  // read the analog in value:
  for (int i = 0; i < avgSamples; i++)
  {
    sensorValue += analogRead(analogInPin);
    // wait 10 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(10);
  }
  sensorValue = sensorValue / avgSamples;
  // The on-board ADC is 10-bits -> 2^10 = 1024 -> 5V / 1024 ~= 4.88mV
  // The voltage is in millivolts
  float voltage1 = 4.88 * sensorValue;
  // This will calculate the actual current (in mA)
  // Using the Vref and sensitivity settings you configure
  
  float current1 = (voltage1 - Vref) * sensitivity;
  Serial.print("Battery current:");
  Serial.print(current1);
  Serial.print("mA");
  Serial.print("\n");
  float watts1 = BatteryAmps1 * batteryVoltage1;
  
  //Voltage and Current Values
  batteryVoltage1 = avgBVal; //supplies Battery voltage as 12V for poweer calc
 Serial.print("Volts = " );                      
  Serial.print(batteryVoltage1);  
   Serial.print("\t Power (Watts) = ");  
  Serial.print(watts1);  
  
 //Timing calc
  sample = sample + 1;
  msec = millis();
  time = (float) msec / 1000.0;
  totalCharge1 = totalCharge1 + BatteryAmps1;
  averageAmps1 = totalCharge1 / sample;
  ampSeconds1 = averageAmps1*time;
  ampHours1 = ampSeconds1/3600;
  wattHours1 = batteryVoltage1 * ampHours1;
 
  Serial.print("\t Time (hours) = ");
  Serial.print(time/3600);
  Serial.print("\t Amp Hours (ah) = ");
  Serial.print(ampHours1);
  Serial.print("\t Watt Hours (wh) = ");
  Serial.println(wattHours1);
  // Reset the sensor value for the next reading
  sensorValue = 0;
}
//2nd Battery Current Sensor
void BatteryCurrentSensor2() {
  int avgBVal2 = 12;
  // read the analog in value:
  for (int i = 0; i < avgSamples; i++)
  {
    sensorValue1 += analogRead(analogIn2Pin);
    // wait 10 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(10);
  }
  sensorValue1 = sensorValue1 / avgSamples;
  // The on-board ADC is 10-bits -> 2^10 = 1024 -> 5V / 1024 ~= 4.88mV
  // The voltage is in millivolts
  float voltage2 = 4.88 * sensorValue1;
  // This will calculate the actual current (in mA)
  // Using the Vref and sensitivity settings you configure
  float current2 = (voltage2 - Vref) * sensitivity;
  Serial.print("Battery 2 current:");
  Serial.print(current2);
  Serial.print("mA");
  Serial.print("\n");
  float watts2 = BatteryAmps2 * batteryVoltage2;
  
  //Voltage and Current Values
  batteryVoltage2 = avgBVal2; //supplies Battery voltage as 12V for poweer calc
 Serial.print("Volts = " );                      
  Serial.print(batteryVoltage2);  
   Serial.print("\t Power (Watts) = ");  
  Serial.print(watts2);  
  
 //Timing calc
  sample1 = sample1 + 1;
  msec = millis();
  time = (float) msec / 1000.0;
  totalCharge2 = totalCharge2 + BatteryAmps2;
  averageAmps2 = totalCharge2 / sample1;
  ampSeconds2 = averageAmps2*time;
  ampHours2 = ampSeconds2/3600;
  wattHours2 = batteryVoltage2 * ampHours2;
 
  Serial.print("\t Time (hours) = ");
  Serial.print(time/3600);
  Serial.print("\t Amp Hours (ah) = ");
  Serial.print(ampHours2);
  Serial.print("\t Watt Hours (wh) = ");
  Serial.println(wattHours2);
  // Reset the sensor value for the next reading
  sensorValue1 = 0;
}
//1st Motor Current Sensor
void MotorCurrentSensor1() {
  // read the analog in value:
  for (int i = 0; i < avgSamples; i++)
  {
    sensorValue2 += analogRead(analogIn3Pin);
    // wait 10 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(10);
  }
  sensorValue2 = sensorValue2 / avgSamples;    
  // convert to milli amps
  outputValue1 = (((long)sensorValue2 * 5000 / 1024) - 500 ) * 1000 / 133;  
  motoramps1 = (float) outputValue1 / 1000; 
  Serial.print("\t Motor Current (amps) = ");     
  Serial.print(motoramps1); 
}
//Motor Current Sensor 2
void MotorCurrentSensor2() {
  // read the analog in value:
  for (int i = 0; i < avgSamples; i++)
  {
    sensorValue3 += analogRead(analogIn4Pin);
    // wait 10 milliseconds before the next loop
    // for the analog-to-digital converter to settle
    // after the last reading:
    delay(10);
  }
  sensorValue3 = sensorValue3 / avgSamples;    
  // convert to milli amps
  outputValue2 = (((long)sensorValue3 * 5000 / 1024) - 500 ) * 1000 / 133;  
  motoramps2 = (float) outputValue2 / 1000; 
  //Current Value     
  Serial.print("\t Current (amps) = ");     
  Serial.print(motoramps2); 
}

//
// Movement functions
//
void moveForward (int distance) {
 leftDirection = 0;
 rightDirection = 0;
 desiredLeftPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
 desiredRightPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
}

void moveBackward (int distance) {
 leftDirection = 1;
 rightDirection = 1;
 desiredLeftPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
 desiredRightPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
}

void rotateLeft (int angle) {
 leftDirection = 1;
 rightDirection = 0;
 desiredLeftPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
 desiredRightPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
}

void rotateRight (int angle) {
 leftDirection = 0;
 rightDirection = 1;
 desiredLeftPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
 desiredRightPIDCount = (desiredSpeed / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
  
}

void moveRobot (int left, int right) {


 //desiredLeftPIDCount = (left/100)* 0.6 * pulsesPerRevolution / 0.1  ; 
 desiredLeftPIDCount = (left / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
 desiredRightPIDCount = (right / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);


      //Serial.print(" - Left count is: ");
      //Serial.print(desiredLeftPIDCount);
//      
      //Serial.print(" Right count is: ");
      //Serial.println(desiredRightPIDCount);

}


// Set Speed function - sets the forward/reverse speed in rpm - not intended to be used in normal operation, only for testing
void setSpeed (int speed) { 
  if ((speed >= 0) && (speed <= maxRPM)) {
    desiredSpeed = speed;
  } else {
    desiredSpeed = 0;
//    Serial.print("Error, invalid speed set. Speed must be between 0 and ");
//    Serial.println(maxRPM);
  }
  
}

void stopMoving () {
  desiredLeftPIDCount = 0;
  desiredRightPIDCount = 0;  
}

//
// Main Loop
//

void loop() {

  if (Serial.available() >= 1) {
    timeout = 0 ; 
    startByte = Serial.read();

    if (startByte == '$'){    
      while(Serial.available() < 8){}
      
      leftHex[0] = Serial.read();
      leftHex[1] = Serial.read();
      leftHex[2] = Serial.read();
      rightHex[0] = Serial.read();
      rightHex[1] = Serial.read();
      rightHex[2] = Serial.read();
      recNum = Serial.read();
      endByte = Serial.read();   
               
      leftSerialSpeed = strtol(leftHex, &ptr, 16);
      rightSerialSpeed = strtol(rightHex, &ptr, 16);
      if (endByte == '\n'){
        Serial.println("VALID COMMAND");      
        moveRobot(leftSerialSpeed, rightSerialSpeed);
      } else {  
        Serial.println("INVALID COMMAND");
        moveRobot(0,0);
      }
    } else {
      moveRobot(0,0);
      Serial.println("INVALID COMMAND");
    }

  }else if(timeout == 20){ // if there has been no command for 20 PID iterations(20 x 0.1 seconds) stop robot
    moveRobot(0, 0)
  }

}
