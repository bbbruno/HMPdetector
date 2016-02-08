//============================================================================
// Name		: SensingBracelet.ino
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version	: 1.0
// Description	: Data collector for HMP detection
//============================================================================

//----------------------------------------------------------------------------
// LIBRARIES
#include "MPU6050.h"    // classes for the MPU6050 IMU
#include "Wire.h"       // basic classes for I2C communication
#include "I2Cdev.h"     // advanced classes for I2C communication
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// PIN DEFINITIONS
#define LED_PIN 13            // use LED CHG on pin 13 as control mechanism
#define STD_GRAVITY 9.80665   // define standard gravity value
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// CONSTANTS
// sensor-related constants
MPU6050 sensor(0x68);                 // I2C address of the sensor (0x68 --> default)
uint8_t DLPFmode = 0;                 // Digital Low-Pass Filter mode (0 --> disabled)
uint8_t rate = 7;                     // sensor sampling frequency (7 --> 1kHz)
// fall detector-related constants (wrist motion detection)
const unsigned int window_size = 120; // size of the moving window (120 --> 5s circa)
const float MDT = 0.2*STD_GRAVITY;    // Motion Detection Threshold
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// GLOBAL VARIABLES
bool blinkState;              // control LED status
int16_t acc_data[3];          // raw accelerometer data (x,y,z)
int16_t gyro_data[3];         // raw gyroscope data (x,y,z)
float real_acceleration[3];   // decoded raw accelerometer data
float window[window_size];    // moving window of accelerometer data
unsigned int numWritten = 0;  // number of samples inside the window
float currentAcceleration;    // som-of-squares of the currently sampled accelerometer data
float Amin;                   // minimum acceleration within the window
float Amax;                   // maximum acceleration within the window
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// FUNCTIONS
// find the maximum & minimum elements within an array of float
void minmaxArray(float array[], unsigned int &array_size, float &maxArray, float &minArray)
{
  float tempMax = array[0];
  float tempMin = array[0];
  
  for(unsigned int i=0; i < array_size; i++)
  {
    if(array[i] > tempMax)
      tempMax = array[i];
    if(array[i] < tempMin)
      tempMin = array[i];
  }
  
  maxArray = tempMax;
  minArray = tempMin;
  /*****************************************************
  // DEBUG: display array[0], tempMax, tempMin
  Serial.print("array[0]: "); Serial.println(array[0]);
  Serial.print("tempMax: "); Serial.println(tempMax);
  Serial.print("tempMin: "); Serial.println(tempMin);
  *****************************************************/
}

// fill the moving window with sum-of-squares acceleration
void updateWindow()
{
  // compute the current sum-of-squares acceleration
  real_acceleration[0] = (float(acc_data[0]) / 65535.0) * (8*STD_GRAVITY);
  real_acceleration[1] = (float(acc_data[1]) / 65535.0) * (8*STD_GRAVITY);
  real_acceleration[2] = (float(acc_data[2]) / 65535.0) * (8*STD_GRAVITY);
  currentAcceleration = sqrt(pow(real_acceleration[0],2) + pow(real_acceleration[1],2) + pow(real_acceleration[2],2));

  // update the window content
  if(numWritten < window_size)
  {
    window[numWritten] = currentAcceleration;
    numWritten = numWritten + 1;
  }
  else
  {
    for(unsigned int s = 0; s < (window_size-1); s++)
      window[s] = window[s+1];
    window[window_size-1] = currentAcceleration;
  }
  // update Amin and Amax
  minmaxArray(window, numWritten, Amax, Amin);
  /*****************************************************
  // DEBUG: display window[0], Amin, Amax to validate the procedure
  Serial.print("window[0]: "); Serial.println(window[0]);
  Serial.print("Amax: "); Serial.println(Amax);
  Serial.print("Amin: "); Serial.println(Amin);
  *****************************************************/
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
void setup()
{
  // configure Arduino LED for fast check
  pinMode(LED_PIN, OUTPUT);
  blinkState = false;

  // serial communication setup (system control)
  Serial.begin(9600);
  
  // ZigBee communication setup
  Serial1.begin(9600);
  
  // sensor-to-microprocessor communication setup
  Serial.println("Initializing I2C channel...");
  Wire.begin();
  Serial.println("Initializing sensor...");
  sensor.initialize();
  sensor.setDLPFMode(DLPFmode);
  sensor.setRate(rate);  
  // DEBUG: test I2C connection
  Serial.println("Testing I2C connection...");
  Serial.println(sensor.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
void loop()
{
  // acquire the accelerometer raw data
  sensor.getAcceleration(&acc_data[0], &acc_data[1], &acc_data[2]);
  // acquire the gyroscope raw data
  sensor.getRotation(&gyro_data[0], &gyro_data[1], &gyro_data[2]);
  
  // compute 'currentAcceleration' and update 'Amax' and 'Amin'
  updateWindow();
  
  // transmit the data via ZigBee
  Serial1.print("\nH ");
  for(unsigned int i = 0; i < 3; i++)
  { Serial1.print(acc_data[i]); Serial1.print(" "); }
  for(unsigned int i = 0; i < 3; i++)
  { Serial1.print(gyro_data[i]); Serial1.print(" "); }
  // signal the lack/presence of motion
  if(abs(Amax - Amin) < MDT)
    Serial1.print("Still\n");
  else
    Serial1.print("Moving\n");
  // DEBUG: transmit the same data via USB
  Serial.print("\nH ");
  for(unsigned int i = 0; i < 3; i++)
  { Serial.print(acc_data[i]); Serial.print(" "); }
  for(unsigned int i = 0; i < 3; i++)
  { Serial.print(gyro_data[i]); Serial.print(" "); }
  // signal the lack/presence of motion
  if(abs(Amax - Amin) < MDT)
    Serial.print("Still\n");
  else
    Serial.print("Moving\n");
  
  // blink LED to show activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  delay(42);
}
//----------------------------------------------------------------------------
