#include <Servo.h>
//#include "MegunoLink.h"

//TimePlot data;

//
// Pin assignments
//
const int analogInPin = A0; // VO of ACS712 current sensor goes to pin AO on Arduino
const int analogIn2Pin = A3;
const byte leftSensorPin_I  = 2;
const byte rightSensorPin_I = 3;

const byte leftPWMPin_O  = 10;
const byte rightPWMPin_O = 9;

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
//for ACS712 Battery Current sensor
int sensorValue = 0; // value read from the carrier board
float sensitivity = 100.0 / 500.0; //100mA per 500mV = 0.2
float Vref = 1500; // Output voltage with no current: ~ 1500mV or 1.5V
unsigned long msec = 0;
float time = 0.0;
int sample = 0;
//Battery values 
float totalCharge = 0.0;
float averageAmps = 0.0;
float ampSeconds = 0.0;
float ampHours = 0.0;
float wattHours = 0.0;
float Amps = 0.0; //Battery current
float batteryVoltage = 0;
//for ACS715 Motor Current sensor
int sensorValue1 = 0;        // value read from the carrier board
float amps = 0.0;             //motor current in milliamps
int outputValue = 0;

float leftKp  = 0.5;
float leftKi  = 0.5;
float rightKp = 0.5;
float rightKi = 0.5;

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
  attachInterrupt(digitalPinToInterrupt(leftSensorPin_I) , leftSensorISR , RISING);
  attachInterrupt(digitalPinToInterrupt(rightSensorPin_I), rightSensorISR, RISING);

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
  if(digitalRead(4) == digitalRead(2)) {
    leftSensorPIDCount --;
    leftSensorDistanceCount --;
  } else {
    leftSensorPIDCount ++;    
    leftSensorDistanceCount ++;
  }
}

void rightSensorISR () {
  if(digitalRead(5) == digitalRead(3)) {
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

  //leftCurrent = analogRead(0);
  //rightCurrent = analogRead(1);


//  if (sendcount >= 5) {

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
//    sendcount = 0;

    //Serial.print(rightCurrent);
    //Serial.print(", ");
    //Serial.println(leftCurrent);
//  } else {
//    sendcount ++;
//  }
  
  timercount = 0;
  } else {
    timercount++;
  }

}
void BatteryCurrentSensor() {
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
  float voltage = 4.88 * sensorValue;
  // This will calculate the actual current (in mA)
  // Using the Vref and sensitivity settings you configure
  
  float current = (voltage - Vref) * sensitivity;
  Serial.print("Battery current:");
  Serial.print(current);
  Serial.print("mA");
  Serial.print("\n");
  float watts = amps * batteryVoltage;
  
  //Voltage and Current Values
  batteryVoltage = avgBVal; //supplies Battery voltage as 12V for poweer calc
 Serial.print("Volts = " );                      
  Serial.print(batteryVoltage);  
   Serial.print("\t Power (Watts) = ");  
  Serial.print(watts);  
  
 //Timing calc
  sample = sample + 1;
  msec = millis();
  time = (float) msec / 1000.0;
  totalCharge = totalCharge + amps;
  averageAmps = totalCharge / sample;
  ampSeconds = averageAmps*time;
  ampHours = ampSeconds/3600;
  wattHours = batteryVoltage * ampHours;
 
  Serial.print("\t Time (hours) = ");
  Serial.print(time/3600);
  Serial.print("\t Amp Hours (ah) = ");
  Serial.print(ampHours);
  Serial.print("\t Watt Hours (wh) = ");
  Serial.println(wattHours);
  // Reset the sensor value for the next reading
  sensorValue = 0;
}
  void MotorCurrentSensor() {
 
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
  // convert to milli amps
  outputValue = (((long)sensorValue * 5000 / 1024) - 500 ) * 1000 / 133;  
  amps = (float) outputValue / 1000; 
  Serial.print("\t Motor Current (amps) = ");     
  Serial.print(amps); 
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

 desiredLeftPIDCount = (left / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
 desiredRightPIDCount = (right / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);


//      Serial.print(" - Left count is: ");
//      Serial.print(desiredLeftPIDCount);
//      
//      Serial.print(" Right count is: ");
//      Serial.println(desiredRightPIDCount);

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

  }

}
