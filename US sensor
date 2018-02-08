#include <NewPing.h>
#include <stdarg.h>

#define SENSOR_NUM     4             // Define the number of sensors. 4 for each UNO board 
#define MAX_DISTANCE 300             // Define the maximum distance limit (in cm) that can be detected by the sensor to ping.
#define PING_INTERVAL 40             // Define the time interval in milliseconds between sensor pings (29ms is the minimum time interval to avoid cross-sensor echo between each sensor).
unsigned long pingTimer[SENSOR_NUM]; // Holds the times when is the next ping should happen for each sensor.
unsigned int cm[SENSOR_NUM];         // Store the ping distance.
uint8_t currentSensor = 0;           // To keeps track and identify which sensor that is activated.

NewPing sonar[SENSOR_NUM] =          // Sensor reading of detected object in form of array
{                                    
   // Initialize each sensor's trigger pin, echo pin, and max distance that can be detected by the sensor.
  NewPing(6, 7, MAX_DISTANCE),
  NewPing(8, 9, MAX_DISTANCE),
  NewPing(10, 11, MAX_DISTANCE),
  NewPing(12, 13, MAX_DISTANCE),
};

 void printfs(char *str, ...)
// Variadic 'printf'-like function for Arduino.
// Accepts arguments in same format as stdio printf
// function. Prints the resulting formatted string
// to the serial port. Matthew Smith, 2014.
{
  const char BUF_SIZE = 32; // Define size for output buffer
  
  char f_str[BUF_SIZE];
  
  va_list args;
  va_start(args, str);
  vsnprintf(f_str, BUF_SIZE, str, args);
  va_end(args);
  
  Serial.print(f_str);
}

void echoCheck()                                 // The sensor distance is set to array if the ping is received
{                                    
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}
void oneSensorCycle()                            // One sensor ping cycle complete
{ 
 
  for (uint8_t i = 0; i < SENSOR_NUM; i++)       // Loop through all the 4 sensor and do something about the result
  {
     {    
        printfs("%d,", cm[i]);                   // The distance is printed in the form of string by using printfs function
    
     }
  }
 
 printfs("\n");

}

void setup() {
  Serial.begin(9600);                     // Initialize the serial communication at baudrate 9600
  pingTimer[0] = millis() + 75;           // Set the Arduino to initiate the first ping at 75  milliseconds, this gives Arduino certain time to settle down before starting new ping.
  for (uint8_t i = 1; i < SENSOR_NUM; i++)//Loop through all the 4 sensors and the starting time for each sensor is set.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {
  for (uint8_t i = 0; i < SENSOR_NUM; i++) { // Loop through all the 4 sensors starting from sensor 1 until 4 with increment 1.
    if (millis() >= pingTimer[i]) {          // To check whether which sensor to start the ping.
      pingTimer[i] += PING_INTERVAL * SENSOR_NUM;  // Set the time for the current sensor to be pinged in the next round.
      if (i == 0 && currentSensor == SENSOR_NUM - 1) oneSensorCycle(); // All the sensors are ping in one complete cycle,the result obtained is used to do other task or aim.
      sonar[currentSensor].timer_stop();          // To ensure that the previous timer of each sensor is cancelled before new ping starts as insurance.
      currentSensor = i;                          // Sensor number array being accessed.
      cm[currentSensor] = 300;                    // Set the distance detected as 300 cm if there is no echo ping return to the sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (Continue the ping processing, the interrupt function is used to check the echo issued through the echoCheck function)
  }
  }
  // Other code that do not involve in the ping result processing is written below.

}
