//============================================================================
// Name		: SensingBelt.ino
// Author(s)	: Barbara Bruno
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Version	: 1.0
// Description	: Embedded real-time posture & fall detector
//============================================================================

//----------------------------------------------------------------------------
// LIBRARIES
#include "MPU6050.h"  // classes for the MPU6050 IMU
#include "Wire.h"     // basic classes for I2C communication
#include "I2Cdev.h"   // advanced classes for I2C communication
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// DEFINITIONS
#define LED_PIN 13           // use LED CHG on pin 13 as control mechanism
#define STD_GRAVITY 9.80665  // define standard gravity value
#define PI 3.14159           // define pi value
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// CONSTANTS
// sensor-related constants
MPU6050 sensor(0x68);                 // I2C address of the sensor (0x68 --> default)
uint8_t DLPFmode = 0;                 // Digital Low-Pass Filter mode (0 --> disabled)
uint8_t rate = 7;                     // sensor sampling frequency (7 --> 1kHz)
uint8_t range = 1;                    // accelerometer sensing range (1 --> +/- 4g)
// fall detector-related constants
const unsigned int window_size = 200; // size of the moving window (200 --> 5s)
const float LFT = 0.41*STD_GRAVITY;   // Lower Fall Threshold
const float UFT = 3.52*STD_GRAVITY;   // Upper Fall Threshold
const unsigned int FTT = 80;          // Fall Time Threshold (80 --> 2s)
const float PAT = 0.4*STD_GRAVITY;    // Post-fall Analysis Threshold
const float MDT = 0.2*STD_GRAVITY;    // Motion Detection Threshold
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// GLOBAL VARIABLES
bool blinkState;              // control LED status
int16_t ax, ay, az;           // raw accelerometer data
int16_t gx, gy, gz;           // raw gyroscope data
float real_acceleration[3];   // decoded raw accelerometer data
float window[window_size];    // moving window of accelerometer data
unsigned int numWritten = 0;  // number of samples inside the window
float currentAcceleration;    // som-of-squares of the currently sampled accelerometer data
float Amin;                   // minimum acceleration within the window
float Amax;                   // maximum acceleration within the window
bool possibleFall;            // flag for possible fall (acceleration below LFT)
bool realFall;                // flag for real fall (acceleration above UFT)
unsigned int timerFall;       // time between LFT detected and UFT detected
float bodyPosture;            // user body posture
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
  real_acceleration[0] = (float(ax) / 65535.0) * (8*STD_GRAVITY);
  real_acceleration[1] = (float(ay) / 65535.0) * (8*STD_GRAVITY);
  real_acceleration[2] = (float(az) / 65535.0) * (8*STD_GRAVITY);
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
  
  // detect the posture
  bodyPosture = acos(-real_acceleration[1]/STD_GRAVITY) * 180.0 / PI;
  // transmit the body posture angle
  Serial.print("P "); Serial.print(bodyPosture); Serial.print("\n");
  Serial1.print("P "); Serial1.print(bodyPosture); Serial1.print("\n");
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
  sensor.setFullScaleAccelRange(range);
  // DEBUG: test I2C connection
  Serial.println("Testing I2C connection...");
  Serial.println(sensor.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.print("MPU6050 sampling frequency: "); Serial.println(sensor.getRate());
  Serial.print("DLPF mode: "); Serial.println(sensor.getDLPFMode());
  Serial.print("Accelerometer sensing range: "); Serial.println(sensor.getFullScaleAccelRange());
  
  // fall detector variables setup
  possibleFall = false;
  realFall = false;
  timerFall = 0;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
void loop()
{
  // acquire the accelerometer raw data
  sensor.getAcceleration(&ax, &ay, &az);
  // acquire the gyroscope raw data
  sensor.getRotation(&gx, &gy, &gz);
  
  // compute 'currentAcceleration' and update 'Amax' and 'Amin'
  updateWindow();
    
  // algorithm for fall detection:
  // 1. detect possible fall (initial stage of a fall)
  if((currentAcceleration < LFT) && (possibleFall == false))
  {
    possibleFall = true;
    timerFall = 0;
  }
  // 2. wait for the impact
  if(possibleFall == true)
    timerFall++;
  // 3a. detect real fall (impact shortly after initial stage)
  if((possibleFall == true) && (timerFall < FTT) && (currentAcceleration > UFT))
  {
    // signal the real fall event and initialize the timer for posture detection
    possibleFall = false;
    realFall = true;    
    // transmit the fall alarm via ZigBee (DEBUG: and via USB)
    Serial1.print("F Fall\n");
    Serial.print("F Fall\n");
  }
  // 4. wait until the end of the fall to detect post-fall movements
  if(realFall == true && abs(Amax - Amin) < PAT)
  {
    // signal the lack/presence of motion
    // case 1: person not moving
    if(abs(Amax - Amin) < MDT)
    {
      // transmit the fall alarm via ZigBee
      Serial1.print("F Still\n");
      Serial.print("F Still\n");
    }
    // case 2: person moving
    else
    {
      // transmit the fall alarm via ZigBee
      Serial1.print("F Moving\n");
      Serial.print("F Moving\n");
    }
    
    realFall = false;
  }
  // 3b. remove possible fall flag if too much time passes with no impact detected
  if(timerFall > FTT)
    possibleFall = false;
    
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  delay(25);  //93 => 10Hz - 25 => 40Hz - 3=>100Hz
}
