#include <Wire.h>
#include <LIDARLite.h>

LIDARLite myLidarLite;

const int superRes    = 1;
const int highRes     = 2; // 1540 samples - 4 steps move per sample
const int normalRes   = 13; // 237 samples //4; // 770 samples - 8 steps move per sample
const int lowRes      = 8; // 385 samples - 16 steps move per sample
const int maxSteps    = 3081; // == 0

int   motDirPin   = 4 ;
int   motStpPin   = 5 ;
int   motEnPin  = 6 ;
int   zeroSensorPin = 7 ;
int   stepPosition = 0 ;
int clockwiseDir = 1;
int zeroSensorVal;

// Input command string
String inputString = "";
boolean stringComplete = false;
unsigned int stepsPerSample = normalRes;
unsigned int stepDelay      = 75;

unsigned long previousMicros = 0;        // will store last time LED was updated

// 0 : startup, 1 : waiting, 2 : 360
int state = 0;

void setup()
{
  Serial.begin(2000000); // Initialize serial connection to display distance readings
  pinMode(motStpPin, OUTPUT);
  pinMode(motDirPin, OUTPUT);
  pinMode(motEnPin, OUTPUT);
  pinMode(zeroSensorPin, INPUT);
  
  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  
  myLidarLite.write(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
  myLidarLite.write(0x04, 0b00000100); // Use non-default reference acquisition count
  myLidarLite.write(0x12, 0x03); // Reference acquisition count of 3 (default is 5)
  autoHome();
 
  state = 1;
  // reserve 10 bytes for the inputString:
  inputString.reserve(10);
}

void loop()
{
  if (stringComplete) {
    if (isDigit(inputString[0])) {
      if ((inputString[0]=='1') or (inputString[0]=='2') or (inputString[0]=='3') or (inputString[0]=='4')) {
        //Process command and set state
        state = inputString[0] - '0'; // convert char to int
        if (inputString.length()>1) {
          if (isDigit(inputString[1])) {
            switch (inputString[1] - '0') {
              case 0 : // normal
                stepsPerSample = lowRes;
                break;
              case 1 : // low res
                stepsPerSample = normalRes;
                break;
              case 2 : // high res
                stepsPerSample = highRes;
                break;
              case 3 : // super res
                stepsPerSample = superRes;
                break;
            }
          }
        }
      }
    }
    inputString = "";
    stringComplete = false;
  }

  // Run a 360 scan w/ stops
  if (state == 2) {
    unsigned long lastInt = 0;
    unsigned int measurement = 0;
    unsigned long currentMicros = millis();
    unsigned int i,j;
    //loop
    Serial.print("$");
    for (i = 0; i<maxSteps-16; i+=stepsPerSample) {
      //scan
      measurement = distanceFast(true);
      //Calculate time
      currentMicros = micros();
      lastInt = currentMicros - previousMicros;
      previousMicros = currentMicros;
      //send over serial
      //Serial.print(i);
      //Serial.print(", ");
      //Serial.print(lastInt);
      //Serial.print(", ");
      //Serial.print(measurement);
      //Serial.write(lastInt);
      //Serial.write(measurement);
      Serial.write(measurement / 256);
      Serial.write(measurement % 256);
      
      //move steps
      for (j = 0; j<stepsPerSample; j++) {
        digitalWrite(motStpPin, HIGH);   
        delayMicroseconds(stepDelay);               
        digitalWrite(motStpPin, LOW);  
        delayMicroseconds(stepDelay); 
        stepPosition ++;
      }
    }
    autoHome();
    state = 1;
  }
  // Run a 360 scan w/ stops
  if (state == 4) {
    unsigned long lastInt = 0;
    unsigned int measurement = 0;
    unsigned long currentMicros = millis();
    unsigned int i,j;
    //loop
    Serial.println("$");
    for (i = 0; i<maxSteps-16; i+=stepsPerSample) {
      //scan
      measurement = distanceFast(true);
      //Calculate time
      currentMicros = micros();
      lastInt = currentMicros - previousMicros;
      previousMicros = currentMicros;
      //send over serial
      //Serial.print(i);
      //Serial.print(", ");
      //Serial.print(lastInt);
      //Serial.print(", ");
      Serial.println(measurement);
      //Serial.write(lastInt);
      //Serial.write(measurement);
      
      //move steps
      for (j = 0; j<stepsPerSample; j++) {
        digitalWrite(motStpPin, HIGH);   
        delayMicroseconds(stepDelay);               
        digitalWrite(motStpPin, LOW);  
        delayMicroseconds(stepDelay); 
        stepPosition ++;
      }
    }
    autoHome();
    //state = 1;
  }
  
  // Run a 360 scan w/ stops
  if (state == 3) {
    unsigned long lastInt = 0;
    unsigned int measurement = 0;
    unsigned long currentMicros = millis();
    unsigned int i,j;
    //loop
    for (i = 0; i<maxSteps-16; i+=stepsPerSample) {
      //scan
      measurement = distanceFast(true);
      //Calculate time
      currentMicros = micros();
      lastInt = currentMicros - previousMicros;
      previousMicros = currentMicros;
      //send over serial
      //Serial.print(i);
      //Serial.print(", ");
      //Serial.print(lastInt);
      //Serial.print(", ");
      //Serial.print(measurement);
      Serial.write(lastInt);
      Serial.write(measurement);
      
      //move steps
      for (j = 0; j<stepsPerSample; j++) {
        digitalWrite(motStpPin, HIGH);   
        delayMicroseconds(stepDelay);               
        digitalWrite(motStpPin, LOW);  
        delayMicroseconds(stepDelay); 
        stepPosition ++;
      }
    }
    autoHome();
    //state = 1;
  }
}


void autoHome() {
    // Homing
    // Rotate clockwise until falling edge is detected
    // set direction clockwise
    digitalWrite(motDirPin, clockwiseDir);
    // Rotate until wiper is detected
    while (not digitalRead(zeroSensorPin)) { // sensor reads high when detected
      digitalWrite(motStpPin, HIGH);   
      delayMicroseconds(stepDelay);               
      digitalWrite(motStpPin, LOW);  
      delayMicroseconds(stepDelay); 
    }
    // Find the edge
    while (digitalRead(zeroSensorPin)) { // sensor reads high when detected
      digitalWrite(motStpPin, HIGH);   
      delayMicroseconds(stepDelay);               
      digitalWrite(motStpPin, LOW);  
      delayMicroseconds(stepDelay); 
    }
    stepPosition = 0;
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// Read distance. The approach is to poll the status register until the device goes
// idle after finishing a measurement, send a new measurement command, then read the
// previous distance data while it is performing the new command.
int distanceFast(bool biasCorrection)
{
  byte isBusy = 1;
  int distance;
  int loopCount;

  // Poll busy bit in status register until device is idle
  while(isBusy)
  {
    // Read status register
    Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
    isBusy = Wire.read();
    isBusy = bitRead(isBusy,0); // Take LSB of status register, busy bit

    loopCount++; // Increment loop counter
    // Stop status register polling if stuck in loop
    if(loopCount > 9999)
    {
      break;
    }
  }

  // Send measurement command
  Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
  Wire.write(0X00); // Prepare write to register 0x00
  if(biasCorrection == true)
  {
    Wire.write(0X04); // Perform measurement with receiver bias correction
  }
  else
  {
    Wire.write(0X03); // Perform measurement without receiver bias correction
  }
  Wire.endTransmission();

  // Immediately read previous distance measurement data. This is valid until the next measurement finishes.
  // The I2C transaction finishes before new distance measurement data is acquired.
  // Prepare 2 byte read from registers 0x0f and 0x10
  Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
  Wire.write(0x8f);
  Wire.endTransmission();

  // Perform the read and repack the 2 bytes into 16-bit word
  Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 2);
  distance = Wire.read();
  distance <<= 8;
  distance |= Wire.read();

  // Return the measured distance
  return distance;
}
