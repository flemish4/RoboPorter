#include <Servo.h>

//Code written by 2016-2017 group. Slightly modified for 2017-2018 group as new encoders were used. 

// Pin assignments
const byte leftSensorPin_I  = 2;
const byte rightSensorPin_I = 3;
const byte leftPWMPin_O  = 9;
const byte rightPWMPin_O = 10;
const unsigned int pulsesPerRevolution = 500;
const unsigned int pidPeriod = 100; //ms
//const unsigned int wheelCircum = 785; //mm (Wheel diameter is 25cm)

// Create Servo objects
Servo leftWheelServo;
Servo rightWheelServo;

// Global variables
float leftKp  = 1.5;
float leftKi  = 0.7;
float rightKp = 1.5;
float rightKi = 0.7;

volatile int leftSensorPIDCount  = 0;
volatile int rightSensorPIDCount = 0;

volatile int leftError = 0;
volatile int rightError = 0;

volatile int leftLastError = 0;
volatile int rightLastError = 0;

volatile int leftOffset = 0;
volatile int rightOffset = 0;

volatile int desiredLeftPIDCount = 0;
volatile int desiredRightPIDCount = 0;

int skip = 0;
int leftSerialSpeed = 0;
int rightSerialSpeed = 0;
int timercount=0;

// Intial Setup
void setup() {

  // Set up hardware interrupts
  attachInterrupt(digitalPinToInterrupt(leftSensorPin_I) , leftSensorISR , CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightSensorPin_I), rightSensorISR, CHANGE);
  
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

// Hardware Interrupts for counting rotary encoder pulses

void leftSensorISR () {
  if(digitalRead(4) == digitalRead(2))
    leftSensorPIDCount --;
  else
    leftSensorPIDCount ++;    

}

void rightSensorISR () {
  if(digitalRead(5) == digitalRead(3))
    rightSensorPIDCount ++;
  else
    rightSensorPIDCount --;
}

// PID Controller - for both wheels 

ISR (TIMER2_COMPA_vect) {
  sei(); //Re-enable all interrupts

  if (timercount >= 49) {

  leftError = desiredLeftPIDCount - leftSensorPIDCount;
  rightError = desiredRightPIDCount - rightSensorPIDCount;

  //data.SendData("Speed", (rightSensorPIDCount*10)/6);
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

  timercount = 0;
  } else 
  {
    timercount++;
  }
}

void moveRobot (int left, int right) {

 desiredLeftPIDCount = (left / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);
 desiredRightPIDCount = (right / (float)60) * pulsesPerRevolution / (float)(1000 / pidPeriod);

      Serial.print(" - Left count is: ");
      Serial.print(desiredLeftPIDCount);
      Serial.print(" Right count is: ");
      Serial.println(desiredRightPIDCount);



}

// Main Loop

void serialFlush(){

  while(Serial.available() > 0) {

    char t = Serial.read();

  }

}
void loop() {
  if (Serial.available() >= 4) {

      leftSerialSpeed = Serial.parseInt();
      skip = Serial.read();
      rightSerialSpeed = Serial.parseInt();  
      serialFlush();
      moveRobot(leftSerialSpeed, rightSerialSpeed);
  }
}