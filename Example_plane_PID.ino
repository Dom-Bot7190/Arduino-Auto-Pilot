#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

Servo Servo1;
Servo Servo2;

int Servo1Pos = 0;
int Servo2Pos = 0;
/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

// Set PID gain for both axis
int rollP = 11;
int rollI = 0;
int rollD = 0;

int pitchP = 11;
int pitchI = 0;
int pitchD = 0;

// Set the set point and error for both axis
int rollPoint = 0;
int pitchPoint = 0;
int pitchError;

int pitchVal;
int rollVal;
/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void setup(void)
{
  Servo1.attach(9);
  Servo2.attach(10);
  delay(50);
  Serial.begin(115200);
  Serial.println(F("Adafruit 10 DOF PID test")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();
  
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Roll: "));
    Serial.print(rollVal);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(pitchVal);
    Serial.print(F("; "));
  }
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
  }

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Convert atmospheric pressure, SLP and temp to altitude    */
    Serial.print(F("Alt: "));
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        bmp_event.pressure)); 
    Serial.print(F(" m; "));

  }
  
  Serial.println(F(""));

  //Create error
  int rollError = orientation.roll - rollPoint;
  int pitchError = orientation.pitch - pitchPoint;

  // PID loops
  // Calculate Proportional
 int rollProportional = rollP * rollError;
 int pitchProportional = pitchP * pitchError;

  // Calculate Integral
  static float rollIntegral = 0;
  rollIntegral += rollError * rollI;
  if(rollIntegral > 10000) rollIntegral = 10000;
  if(rollIntegral < -10000) rollIntegral = -10000;

  static float pitchIntegral = 0;
  pitchIntegral += pitchError * pitchI;
  if(pitchIntegral > 10000) pitchIntegral = 10000;
  if(pitchIntegral < -10000) pitchIntegral = -10000;

  // Calculate Derivative
  static float previous_roll_error = 0;
  int rollDerivative = (rollError - previous_roll_error) * rollD;
  previous_roll_error = rollError;

  static float previous_pitch_error = 0;
  int pitchDerivative = (pitchError - previous_pitch_error) * pitchD;
  previous_pitch_error = pitchError;
  
  pitchVal = (pitchProportional + pitchIntegral + pitchDerivative);
  rollVal = (rollProportional + pitchIntegral + pitchDerivative);
  
  if(rollVal > 1000) rollVal = 1000;
  if(rollVal < -1000) rollVal = -1000;
  if(pitchVal > 1000) pitchVal = 1000;
  if(pitchVal < -1000) pitchVal = -1000;

 rollVal = map(rollVal, -1000, 1000, 0, 180);
 Servo1.write(rollVal);
 pitchVal = map(pitchVal, -1000, 1000, 0, 180);
 Servo2.write(pitchVal);

 delay(10);
}
