/*
  Very Low Accuracy Digital Setting Circles

  This is a simple adapter to connect an accelerometer+compass (LSM303DLHC)
  via WiFi to the SkySafari Android/iOS app, in order to provide basic (low accuracy)
  "Push To" style features to telescope mounts that cant' fit
  optical encoders, such as travel tripods, tabletops, etc.

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

    ESP8266 development board (version 12E) or ESP-01. I used the HiLetgo model https://www.amazon.com/gp/product/B010O1G1ES for development and ESP-01 for final assembly.
    LSM303DLHC sensor module, I used this one: https://www.aliexpress.com/item/1-pcs-GY-511-LSM303DLHC-Module-E-Compass-3-Axis-Accelerometer-3-Axis-Magnetometer-Module-Sensor/1956617486.html

    ESP8266 pinout:
     SDA = GPIO2 = PIN_D4   (use 3.3K pullup to VCC)
     SCL = GPIO0 = PIN_D3   (use 3.3K pullup to VCC)

    LSM303 pinout:
     SDA, SCL and GND: matching the ESP8266
     VCC = 3V3 pin on the ESP8266
     The sensor is meant to be placed sideways (with the Z axis horizontal, on the left side of the telescope OTA)
*/

#include <Wire.h>
#include <LSM303.h>
#include <ESP8266WiFi.h>
#define RADIAN_TO_STEPS   5092.95818f  // ratio to convert from radians to encoder steps, using a "full circle" of 32000 steps as the Skysafari setting for basic encoder
#define STEPS_IN_FULL_CIRCLE  32000    // number of steps in a full circle, should match the skysafari setting for basic encoder
#define EMA_WEIGHT   0.05f  // weight coefficient for new values vs old values used in the exponential moving average smoothing algorithm

// To give sufficient CPU time to the TCP Server, a time delta between measurements is enforced:
#define MEASUREMENT_PERIOD  25    // how many milliseconds must have elapsed since last measurement in order to take a new one
long last_measurement = 0;  // millisecond timestamp of last measurement, to measure only every XXX milliseconds

WiFiServer server(4030);    // 4030 is the default port Skysafari uses for WiFi connection to telescopes
WiFiClient remoteClient;    // represents the connection to the remote app (Skysafari)
#define WiFi_Access_Point_Name "vlaDSC-Telescope"   // Name of the WiFi access point this device will create for your tablet/phone to connect to.

/** Extends the Pololu LSM303 driver class, to add calibration of accelerometer data, and provide both altitude and azimuth readings after smoothing and in optical encoder style.
*/
class IMU : public LSM303
{
  // Calibration of accelerometer: These are the minimum and maximum values measured for X, Y and Z:
  LSM303::vector<int16_t> acc_min = (LSM303::vector<int16_t>) {-16649, -16448, -16654};
  LSM303::vector<int16_t> acc_max = (LSM303::vector<int16_t>) {+16108, +16568, +15599};

  // measurements are stored here after calling calculatePosition();
  public : float azimuthReading;  // multiply this by 180 / PI to get degrees
  public : float altitudeReading; // multiply this by 180 / PI to get degrees

  public : float smoothAzimuthReading;  // value is already converted from radians to steps, and smoothing alorithm applied
  public : float smoothAltitudeReading; // value is already converted from radians to steps, and smoothing alorithm applied

  vector<float> East = {0, 0, 0};

  public : void calculatePosition()
    {
      vector<int> zAxis = (vector<int>) {0, 0, 1};  // the heading will be measured relative to Z axis, make sure to place your sensor in a vertical position (Z axis pointing to the horizon)

      // get raw accelerometer data:
      vector<float> acc_reading = {a.x, a.y, a.z};
      // calibrate accelerometer bias and scale using measured maximums and minimums, and apply smoothing algorithm to reduce outlier values:
      acc_reading.x = acc_reading.x * (1 - EMA_WEIGHT) + EMA_WEIGHT * (32767.0 * ((float)a.x - (float)acc_min.x) / ((float)acc_max.x - (float)acc_min.x) - 16383.5);
      acc_reading.y = acc_reading.y * (1 - EMA_WEIGHT) + EMA_WEIGHT * (32767.0 * ((float)a.y - (float)acc_min.y) / ((float)acc_max.y - (float)acc_min.y) - 16383.5);
      acc_reading.z = acc_reading.z * (1 - EMA_WEIGHT) + EMA_WEIGHT * (32767.0 * ((float)a.z - (float)acc_min.z) / ((float)acc_max.z - (float)acc_min.z) - 16383.5);

      // calculate the altitude as an angle from 0º to 90º, see equation #26 from https://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
      altitudeReading = -1 * atan2(acc_reading.y, sqrt(acc_reading.x * acc_reading.x + acc_reading.z * acc_reading.z));

      // now adapt the 0º to 90º reading to the 0º to 360º style encoder output expected by Skysafari:
      if (acc_reading.x > 0 && acc_reading.y < 0)
      {
        // first quadrant
      }
      else if (acc_reading.x < 0 && acc_reading.y < 0)
      {
        // second quadrant
        altitudeReading = PI - altitudeReading;
      }
      else if (acc_reading.x < 0 && acc_reading.y > 0)
      {
        // third quadrant
        altitudeReading = PI - altitudeReading;
      }
      else if (acc_reading.x > 0 && acc_reading.y > 0)
      {
        altitudeReading = 2 * PI + altitudeReading;
        // 4th quadrant
      }

      // calibrate magnetometer bias and scale using measured maximums and minimums (notice we don't smooth mag readings, it won't help):
      vector<float> mag_reading = {m.x, m.y, m.z};
      mag_reading.x = 10000.0 * ((float)m.x - (float)m_min.x) / ((float)m_max.x - (float)m_min.x) - 5000.0;
      mag_reading.y = 10000.0 * ((float)m.y - (float)m_min.y) / ((float)m_max.y - (float)m_min.y) - 5000.0;
      mag_reading.z = 10000.0 * ((float)m.z - (float)m_min.z) / ((float)m_max.z - (float)m_min.z) - 5000.0;

      // calculate the vector pointing "East" by cross product of magnetometer (north) x accelerometer (up)
      vector_cross(&mag_reading, &acc_reading, &East);

      // normalize both the magnetic North and the new East vector, to be able to use them for scalar projection:
      vector_normalize(&mag_reading);
      vector_normalize(&East);

      // use scalar proyections of Z axis to East and North vectors, to calculate the azimuth heading relative to Z axis:
      float azimuthReading = atan2(vector_dot(&East, &zAxis), vector_dot(&mag_reading, &zAxis)); 

      // for sensors placed left of the scope, the Z axis heading seems to need 90 degree correction
      azimuthReading = azimuthReading - PI / 2;

      // shift negative values to 0-360 range:
      if (azimuthReading < 0)
      {
        azimuthReading += 2 * PI;
      }

      // convert angle from radians to encoder steps (which is what skysafari wants):
      float newazimuthReading = azimuthReading * RADIAN_TO_STEPS;
      float newAltitudeReading = altitudeReading * RADIAN_TO_STEPS;

      /* Final smoothing algotithm: */
      // First, a special trick for averaging around North, where low values such as 3 averaged with high values such as 31997 should average to zero
      if (smoothAzimuthReading > STEPS_IN_FULL_CIRCLE / 2 && newazimuthReading < STEPS_IN_FULL_CIRCLE / 4)
      {
        newazimuthReading += STEPS_IN_FULL_CIRCLE;
      }
      else if (newazimuthReading > STEPS_IN_FULL_CIRCLE / 2 && smoothAzimuthReading < STEPS_IN_FULL_CIRCLE / 4)
      {
        smoothAzimuthReading += STEPS_IN_FULL_CIRCLE;
      }
      if (smoothAltitudeReading > STEPS_IN_FULL_CIRCLE / 2 && newAltitudeReading < STEPS_IN_FULL_CIRCLE / 4)
      {
        newAltitudeReading += STEPS_IN_FULL_CIRCLE;
      }
      else if (newAltitudeReading > STEPS_IN_FULL_CIRCLE / 2 && smoothAltitudeReading < STEPS_IN_FULL_CIRCLE / 4)
      {
        smoothAltitudeReading += STEPS_IN_FULL_CIRCLE;
      }

      // Second, to reduce jittering without making responsiveness slow:
      // When the new readings are less than 0.5 degrees off from the old readings, use 0.05 * EMA_WEIGHT as the alpha, for a highly smoothed result
      if (abs(newazimuthReading - smoothAzimuthReading) < STEPS_IN_FULL_CIRCLE / 720)
      {
        smoothAzimuthReading = newazimuthReading * 0.05 * EMA_WEIGHT + ((1 - 0.05 * EMA_WEIGHT) * smoothAzimuthReading);
      }
      else          // When the new readings are more than 0.5 degrees off from the old readings, the regular EMA_WEIGHT as the alpha, fast, responsive user experience
      {
        smoothAzimuthReading = newazimuthReading * EMA_WEIGHT + ((1 - EMA_WEIGHT) * smoothAzimuthReading);
      }
      // Repeat for altitude: When the new readings are less than 0.5 degrees off from the old readings, use 0.05 * EMA_WEIGHT as the alpha, for a highly smoothed result
      if (abs(newAltitudeReading - smoothAltitudeReading) < STEPS_IN_FULL_CIRCLE / 720)
      {
        smoothAltitudeReading = newAltitudeReading * EMA_WEIGHT * 0.05 + ((1 - EMA_WEIGHT * 0.05) * smoothAltitudeReading);
      }
      else    // Repeat for altitude: When the new readings are more than 0.5 degrees off from the old readings, the regular EMA_WEIGHT as the alpha, fast, responsive user experience
      {
        smoothAltitudeReading = newAltitudeReading * EMA_WEIGHT + ((1 - EMA_WEIGHT) * smoothAltitudeReading);
      }

      // if values exceed the encoder scale, fix them (for example: 32002 becomes 00002)
      if (smoothAzimuthReading > STEPS_IN_FULL_CIRCLE)
      {
        smoothAzimuthReading -= STEPS_IN_FULL_CIRCLE;
      }
      if (smoothAltitudeReading > STEPS_IN_FULL_CIRCLE)
      {
        smoothAltitudeReading -= STEPS_IN_FULL_CIRCLE;
      }
    }
};

IMU imu;

void setup()
{
  Serial.begin(115200);
  Serial.println("\nESP266 boot");

  Wire.begin(0, 2);     // connect D3(GPIO_0) to sensor's SDA, connect D4(GPIO_2) to sensor's SCL
  //  i2cscan();        // uncomment this if you are having I2C problems (quality of solder contacts, lack of pull-up resistors)

  WiFi.mode(WIFI_AP);
  IPAddress ip(1, 2, 3, 4);     // The "telescope IP address" that Skysafari should connect to is 1.2.3.4 which is easy to remember.
  IPAddress gateway(1, 2, 3, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(ip, gateway, subnet);
  WiFi.softAP(WiFi_Access_Point_Name);

  IPAddress myIP = WiFi.softAPIP();

  // Apply compass calibration values:
  //      Min: -336   -368   -239  Max M:    363, 316, 367
  // alt: Min: -320   -359   -227  Max M:    348, 307, 356
  imu.m_min = (LSM303::vector<int16_t>) {-336, -368, -239};
  imu.m_max = (LSM303::vector<int16_t>) {+363, +316, +367};
  imu.init();
  imu.enableDefault();

  Serial.println("IMU ok");
  // tcp listener to receive incoming connections from Skysafari:
  server.begin();
  server.setNoDelay(true);
}

long before, after;
void loop()
{
#ifdef TIMEPROFILE
  after = millis();
  if (after - before > 0)
  {
    Serial.print("TIME: ");
    Serial.println((after - before));
  }
#endif
  attendTcpRequests();  // gets priority to prevent timeouts on Skysafari. Measured AVG execution time = 18ms
  yield();

  if (millis() - last_measurement > MEASUREMENT_PERIOD) // only take new measurements if enough time has elapsed.
  {
    //    Serial.println("reading IMU");
    imu.read();
    //    Serial.println("IMU read");
    imu.calculatePosition();
    last_measurement = millis();
  }

#ifdef TIMEPROFILE
  before = millis();
#endif
  yield();  // altough ESP8266 is supposed to do its work background TCP/wiFi after each loop, yieldig here can't hurt
}

void attendTcpRequests()
{
  // check for new or lost connections:
  if (server.hasClient())
  {
    Serial.println("hasClient!");
    if (!remoteClient || !remoteClient.connected())
    {
      if (remoteClient)
      {
        Serial.print("Client Disconnected\n");
        remoteClient.stop();
      }
      remoteClient = server.available();
      //Serial.print("Inbound connection from: ");
      Serial.println(remoteClient.remoteIP());
      //  remoteClient.flush();
      remoteClient.setNoDelay(true);
    }
  }

  // when we have a new incoming connection from Skysafari:
  while (remoteClient.available())
  {
    byte skySafariCommand = remoteClient.read();

    if (skySafariCommand == 81)  // 81 is ascii for Q, which is the only command skysafari sends to "basic encoders"
    {
      char encoderResponse[20];
      int iAzimuthReading = imu.smoothAzimuthReading;
      int iAltitudeReading = imu.smoothAltitudeReading;
      sprintf(encoderResponse, "%i\t%i\t\n", iAzimuthReading, iAltitudeReading);

      Serial.println(encoderResponse);

      remoteClient.println(encoderResponse);
    }
    else
    {
      Serial.print("*****");
      Serial.println(skySafariCommand);
    }
  }
}


void i2cscan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning for I2C devices");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    digitalWrite(1, LOW);
    // The i2c_scanner uses the return value of the Write.endTransmisstion to see if a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      digitalWrite(1, HIGH);
      Serial.print("I2C device found at address 0x");
      if (address < 16) { Serial.print("0"); }
      Serial.print(address, HEX);
      Serial.println("  !");

      delay(5000);

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {Serial.print("0");}
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
    delay(10000);
  }
  else
    Serial.println("done");
}
