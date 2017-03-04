/*
  Very Low Accuracy Digital Setting Circles

  This is a simple adapter to connect an accelerometer (MMA7455)
  and a compass (HMC58X3) via WiFi to SkySafari app, in order to
  provide basic (low accuracy) "Push To" features to telescope mounts
  that cant' fit optical encoders, such as travel tripods or tabletops.

  Copyright (c) 2017 Vladimir Atehortua. All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the version 3 GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along with this program.
  If not, see <http://www.gnu.org/licenses/>
*/

/**
    Hardware used:
     ESP8266 development board (version 12E), I used the HiLetgo model https://www.amazon.com/gp/product/B010O1G1ES)
     MMA7455 sensor module, I used this one: https://arduino-info.wikispaces.com/accelerometer-MMA7455
     HMC5883 sensor module, I used the GY271 variant.

     ESP8266 pinout:
     SDA = GPIO2 = PIN_D4
     SCL = GPIO0 = PIN_D3

     MMA7455 pinout:
     SDA, SCL and GND: matching the ESP8266
     VCC = 3V3 pin on the ESP8266
     CS must be set high (3V3 or VCC) or it wont work with the ESP8266, even if its not necesary with a 5V arduino.
     The acelerometer is meant to be used sideways (with the Z axis horizontal, measuring near zero G)

     HMC5883 (or GY271) pinout:
     SDA, SCL and GND: matching the ESP8266
     VCC = 3V3 pin on the ESP8266
     DRDY pin disconnected
     The HMC5883 is meant to be used on a permanently horizontal position in the mount (not attached to the telescope tube)
*/

#define MMA7455_ADDRESS 0x1D      //I2C Adsress for the sensor
#define MMA7455_MODE_CONTROL 0x16 //Call the sensors Mode Control
#define MMA7455_2G_MODE 0x05      //Set Sensitivity to 2g
#define MMA7455_X_OUT 0x06        //Register for reading the X-Axis
#define MMA7455_Y_OUT 0x07        //Register for reading the Y-Axis
#define MMA7455_Z_OUT 0x08        //Register for reading the Z-Axis
#define MMA7455_STATUS 0x09       // Status Register
#define MMA7455_DRDY 0            // bit in the status response that signals "ready"
#define MMA7455_XOFFL 0x10        // Read/Write, Offset Drift X LSB

#include "Wire.h"             // for I2C
#include <ESP8266WiFi.h>      // for WiFi
#include <HMC58X3.h>          // magnetometer library, it's .cpp and .h must be modified to work with ESP8266: replace all "int" with "int16_t"

const char* ssid = "PennyGetYourOwnWiFi";   // WiFi network ID to connect to
const char* password = "****";          // Password for Wifi Network
const int led = 13;                         // ESP8266 in-built led, to indicate some status

WiFiServer server(4030);    // 4030 is the default port Skysafari uses for WiFi connection to telescopes
WiFiClient remoteClient;    // represents the connection to the remote app (Skysafari)

/* Simulated encoders:
   The "basic encoder system" option in skysafari allows us to specify a number of steps per revolution for each encoder.
   Lets call this number of "steps per revolution" M, AZ the azimuth encoder and AL the altitude encoder:
   if altitude was zero: AZ = 0 means North, AZ = M/4 means East, AZ = M/2 means Soutch, and AZ = 3M/4 means West
   if azimuth was zero:  AL = 0 means North, AL = M/4 means Zenith, AL = M/2 means South, AL = 3M/4 means opposite to zenith (down through earth)
*/

#define RADIAN_TO_STEPS   636.61977236758f  // ratio to convert from radians to encoder steps, using a "full circle" of 4000 steps as the Skysafari setting for basic encoder
#define STEPS_IN_FULL_CIRCLE  4000          // number of steps in a full circle, should match the skysafari setting for basic encoder
float azimuthReading;  // integer version of the azimuth reading, as "fraction of full_circle"
float altitudeReading; // integer version of the altitude reading, as "fraction of full_circle"

/** Calibration of accelerometer:
    
    Unlike the magnetometer, the accelerometer calibration values are stable over time/location, so it is best to capture them only once and leave them fixed.

    You need to find these values for each accelerometer sensor/module. 
    You need to set up the accelerometer in a position that can rotate 360º vertically (could be your telescope mount without the OTA, or something else), and
    then execute code that measures X and Y, while very slowly rotating the sensor two full vertical circles.  It must be done slowly to avoid the force
    applied by your hand from altering the measurements of the gravity force.

    Code to measure X and Y during calibration is already contained in the performAltitudeMeasurement() function, but it's commented out.
    Uncomment that code, measure your calibration values, and comment it again for normal DSC use.
    
*/
int16_t ALminX = -61;   // minimum X value at -1G for this particular accelerometer
int16_t ALmaxX = 65;    // maxmimum X value at 1G for this particular accelerometer
int16_t ALminY = -79;   // minimum Y value at -1G for this particular accelerometer
int16_t ALmaxY = 49;    // maxmimum Y value at 1G for this particular accelerometer

float calibratingX;     // only used dured accelerometer calibration
float calibratingY;     // only used dured accelerometer calibration


/**
   Compass offsets:
   When you turn on the device, you need top do a full azimuth rotation in order to calibrate the compass.
   If during use, a new maximum is found, the offset calibration is updated:
*/
int16_t maxX, maxY, minX, minY; // needed to calculate offset for compass
HMC58X3 compass;  // compass API object
#define LOCAL_MAGNETIC_DECLINATION 0.0f  // local magnetic declination, to convert magnetic heading to true north if desired (but Skysafari alignment does it better)

/* To give sufficient CPU time to the TCP Server, a time delta between measurements is enforced: */
long last_measurement = 0;  // millisecond timestamp of last measurement, to measure only every XXX milliseconds
#define MEASUREMENT_PERIOD  25    // how many milliseconds must have elapsed since last measurement in order to take a new one

#define ALT_EMA_WEIGHT   0.05f  // weight coefficient for new values vs old values used in the moving average smoothing algorithm for Altitude
#define AZ_EMA_WEIGHT    0.05f  // weight coefficient for new values vs old values used in the moving average smoothing algorithm for Azimuth


/* The typical Arduino setup function: */
void setup()
{
  // Setup diagnostic led:
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  Serial.println("\nESP266 boot");
  setup_wifi();

  Wire.begin(0, 2); // connect D3(GPIO_0) to sensor's SDA, connect D4(GPIO_2) to sensor's SCL, and connect Sensor "CS" to High!!
  delay(500);

  setup_accelerometer();
  setup_compass();
}

/* The typical Arduino loop function: */
void loop()
{
  attendTcpRequests();  // gets priority to prevent timeouts on Skysafari. Measured AVG execution time = 18ms

  if (millis() - last_measurement > MEASUREMENT_PERIOD) // only take new measurements if enough time has elapsed.
  {
    performAltitudeMeasurement();  // avg exec takes 8ms
    performAzimuthMeasurement();   // avg exec takes 1ms
    last_measurement = millis();
  }
  yield();  // altough ESP8266 is supposed to do its work background TCP/wiFi after each loop, yieldig here can't hurt
}

/**
   Magnetometer setup using the HMC58X3 library.
   The Arduino HMC58X3 library must be modified in order to work with ESP8266, because the ESP8266 is 32 bits.
   The fix is simple: edit the library's HMC58X3.cpp and HMC58X3.h, replacing all occurrences of "int" with "int16_t"
*/
void setup_compass()
{
  Serial.println("\nSetting up Compass");
  compass.init(false);      // initialize compass in single mode conversion
  compass.calibrate(1, 32); // calibrate scale using in-built, known, current induced magnetic field, default gain, 32 samples
  compass.setMode(0);       // Single mode conversion was used in scale calibration, now set continuous mode.

  // initialize the offset calibration variables:
  maxX = maxY = -10000;
  minX = minY = 10000;
  Serial.println("\nCompass ready. Do at least one full rotation to calibrate");
}

/**
   Accelerometer setup.
   Basic testing of MMA7455 and 2G sensitivity selection
*/
void setup_accelerometer()
{
  Serial.println("\nTesting MMA7455 accelerometer");

  unsigned char c;
  do
  {
    Wire.beginTransmission(MMA7455_ADDRESS);
    int error = Wire.write(MMA7455_MODE_CONTROL);
    error = Wire.write(MMA7455_2G_MODE);  //Set Sensitivity to 2g
    error = Wire.endTransmission();

    delay(500);
    // status verification check:
    c = readMMA7455(MMA7455_STATUS);
    Serial.println(c);
    delay(500);
  }
  while (!bitRead(c, MMA7455_DRDY));  // we want to stop here if we dont get the ready signal from the 7455 accelerometer
  Serial.print("OK");
}

void setup_wifi()
{
  Serial.print("\nAttempting to connect to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(led, HIGH); // the led will blink while WIFI connection is attempted
    delay(250);
    digitalWrite(led, LOW);
    delay(250);
  }
  Serial.print("\nConnected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  server.setNoDelay(true);
}

void attendTcpRequests()
{
  // check for new or lost connections:
  if (server.hasClient())
  {
    if (!remoteClient || !remoteClient.connected())
    {
      if (remoteClient)
      {
        remoteClient.stop();
      }
      remoteClient = server.available();
      remoteClient.flush();
      remoteClient.setNoDelay(true);
    }
  }

  // when we have a new incoming connection from Skysafari:
  if (remoteClient.available())
  {
    byte skySafariCommand = remoteClient.read();

    if (skySafariCommand == 81)  // 81 is ascii for Q, which is the only command skysafari sends to "basic encoders"
    {
      char encoderResponse[20];
      int iAzimuthReading = round(azimuthReading);
      int iAltitudeReading = round(altitudeReading);
      sprintf(encoderResponse, "%i\t%i", iAzimuthReading, iAltitudeReading);
      remoteClient.println(encoderResponse);
    }
  }
}

/**
   Method to obtain the azimuth orientation of the scope from the HMC58X3 magnetometer.
   This code keeps account of the observed maximum and minumum values for XY axis in order to remain calibrated.
   This code requires the magetic sensor to be horizontal (parallel to the ground), not attached to the telescope's OTA.
*/
void performAzimuthMeasurement()
{
  int16_t ix, iy, iz;
  compass.getValues(&ix, &iy, &iz); // read as int values from HMC58X3

  // update max and min and adjust offset calculations as device is used:
  if (ix > maxX  && ix < 10000) {  // 10000 limit is to ignore a readings that can be caused by cable/wiring issues.
    maxX = ix;
  }
  if (ix < minX && ix > -10000) {
    minX = ix;
  }
  if (iy > maxY && iy < 10000) {
    maxY = iy;
  }
  if (iy < minY && iy > -1000) {
    minY = iy;
  }
  // notice we ignored Z, so for this to work the magnetometer needs to be horizontal. This code does not do tilt calibration for compass.

  // update offset and scale coefficients:
  float compassOfssetX = (float)maxX - (float)(maxX  - minX) / 2.0;
  float compassOfssetY = (float)maxY - (float)(maxY - minY) / 2.0;
  float scaleY = 1.0;
  if ((maxY - minY) > 0 && (maxX - minX) > 0)
  {
    scaleY  = (float)(maxX - minX) / (float)(maxY - minY);
  }

  // apply offset and scale coefficients to current measurement:
  float fx = (float)ix - compassOfssetX;
  float fy = ((float)iy - (float)compassOfssetY) * scaleY;

  // Calculating the heading requires the magnetometer sensor to be close to horizontal, so attach it to the mount, not the telescope.
  // That's why separate sensors are used, instead of an integrated IMU.
  float heading = atan2(fy, fx) + LOCAL_MAGNETIC_DECLINATION;

  heading -= M_PI / 2; // the compass outputs 90º at north, so we substract 90º (PI/2) to offset the scale to zero, which is what Skysafari expects for north
  if (heading < 0.0)
  {
    heading += 2 * M_PI;    // atan2 produces negative values for west side, we add 360º (2*PI) to put them in the scale Skysafari's expects for west
  }

  float newazimuthReading = heading * RADIAN_TO_STEPS;

  // atan returns negative for angles > 180 degrees, lets map them to the proper encoder scale:
  if (newazimuthReading < 0)
  {
    newazimuthReading += STEPS_IN_FULL_CIRCLE;
  }

  // Begin Smoothing algotithm:
  // First, a special trick for averaging around North, where 3 vs 3997 should average to zero
  if (azimuthReading > STEPS_IN_FULL_CIRCLE / 2 && newazimuthReading < STEPS_IN_FULL_CIRCLE / 4)
  {
    newazimuthReading += STEPS_IN_FULL_CIRCLE;
  }
  else if (newazimuthReading > STEPS_IN_FULL_CIRCLE / 2 && azimuthReading < STEPS_IN_FULL_CIRCLE / 4)
  {
    azimuthReading += STEPS_IN_FULL_CIRCLE;
  }

  // now lets apply the exponential moving average to the historic value and the new reading:
  azimuthReading = newazimuthReading * AZ_EMA_WEIGHT + ((1 - AZ_EMA_WEIGHT) * azimuthReading);

  // if values exceed the encoder scale, fix them (for example: 4002 becomes 0002)
  if (azimuthReading > STEPS_IN_FULL_CIRCLE)
  {
    azimuthReading -= STEPS_IN_FULL_CIRCLE;
  }
}

/**
   Method to obtain the Altitude inclination of the scope from the MMA7455 accelerometer.
   This code needs previously measured maximum and minimum values for X and Y.
   Uncomment the marked code to find those values, put them in AZminX, AZmaxX, AZminY, AZmaxY, and comment back the code.
   
   Notice this code uses X and Y but not Z. This means the MMA7455 accelerometer is meant to be placed sideways on the telescope OTA (Z pointing horizontally)
*/
void performAltitudeMeasurement()
{
  int16_t x, y;
  x = readMMA7455(MMA7455_X_OUT);
  y = readMMA7455(MMA7455_Y_OUT);

/*
// Uncomment these lines to perform a one time calibration of accelerometer:
calibratingX = calibratingX * 0.9 + ((float)x)*0.1;  // smoothing, because for calibration use, repeatable max values are much better than outlier max values.
calibratingY = calibratingY * 0.9 + ((float)y)*0.1;  // smoothing, because for calibration use, repeatable max values are much better than outlier max values.
if (calibratingX > ALmaxX)
{
  ALmaxX = calibratingX;
}
else if (calibratingX < ALminX)
{
  ALminX = calibratingX;
}
if (calibratingY > ALmaxY)
{
  ALmaxY = y;
}
if (calibratingY < ALminY)
{
  ALminY = calibratingY;
}
  Serial.print("X=");
  Serial.print(x);
  Serial.print(" minX=");
  Serial.print(ALminX);
  Serial.print(" maxX=");
  Serial.print(ALmaxX);
  Serial.print(" Y=");
  Serial.print(y);
  Serial.print(" minY=");
  Serial.print(ALminY);
  Serial.print(" maxY=");
  Serial.println(ALmaxY);
  if (ALmaxX == ALminX) {ALmaxX = -100; ALminX =100;}   // these are just to prevent a division by zero while calibrating;
  if (ALmaxY == ALminY) {ALmaxY = -100; ALminY =100;}   // these are just to prevent a division by zero while calibrating;
// <End of Calibration Code>
*/
  
  //  x = map(x, ALminX, ALmaxX, -1000, 1000);    // I inlined the map() code to use floats for a little bit more precision:
  float fx = 2000.0 * ((float)x - (float)ALminX) / ((float)ALmaxX - (float)ALminX) - 1000.0; 
  //  y = map(y, ALminY, ALmaxY, -1000, 1000);    // I inlined the map() code to use floats for a little bit more precision:
  float fy = 2000.0 * ((float)y - (float)ALminY) / ((float)ALmaxY - (float)ALminY) - 1000.0; 

  // calculate the angle from the measurements:
  double radianAngle = atan2(fx, fy);
  // convert angle from radians to encoder steps:
  float newAltitudeReading = radianAngle * RADIAN_TO_STEPS;   

  // atan returns negative for angles > 180 degrees, lets map them to the proper encoder scale:
  if (newAltitudeReading < 0)
  {
    newAltitudeReading += STEPS_IN_FULL_CIRCLE;
  }

  // Begin Smoothing algotithm:
  // First, a special trick for averaging around the horizon, where 3 vs 3997 should average to zero:
  if (altitudeReading > STEPS_IN_FULL_CIRCLE / 2 && newAltitudeReading < STEPS_IN_FULL_CIRCLE / 4)
  {
    newAltitudeReading += STEPS_IN_FULL_CIRCLE;
  }
  else if (newAltitudeReading > STEPS_IN_FULL_CIRCLE / 2 && altitudeReading < STEPS_IN_FULL_CIRCLE / 4)
  {
    altitudeReading += STEPS_IN_FULL_CIRCLE;
  }

  // now lets apply the exponential moving average to the historic value and the new reading:
  altitudeReading = newAltitudeReading * ALT_EMA_WEIGHT + ((1 - ALT_EMA_WEIGHT) * altitudeReading);

  // if values exceed the encoder scale, fix them (for example: 4002 becomes 0002). Skysafari doesn't seem to need it but lets keep things neat
  if (altitudeReading > STEPS_IN_FULL_CIRCLE)
  {
    altitudeReading -= STEPS_IN_FULL_CIRCLE;
  }
}

/**
 * Simplme method to read a register from the MMA7455 accelerometer
 */
signed char readMMA7455(byte address)
{
  Wire.beginTransmission(MMA7455_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.beginTransmission(MMA7455_ADDRESS);
  Wire.requestFrom(MMA7455_ADDRESS, 1);
  while (Wire.available())   // slave may send less than requested
  {
    signed char c = Wire.read();    // receive a byte as signed character
    return c;
  }
}
