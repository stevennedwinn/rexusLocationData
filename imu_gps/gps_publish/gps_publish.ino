/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include<ros.h>
#include<sensor_msgs/NavSatFix.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Vector3.h>
#include<nav_msgs/Odometry.h>

//static const int RXPin = 3, TXPin = 4; uno
static const int RXPin = 50, TXPin = 52;
static const uint32_t GPSBaud = 9600;
ros::NodeHandle nh;

//sensor_msgs::NavSatFix gpsMsg;
//ros::Publisher gps2("gps", &gpsMsg);

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

TinyGPSCustom pdop(gps, "GNGLL", 1); // $GPGSA sentence, 15th element
TinyGPSCustom hdop(gps, "GNGLL", 3); // $GPGSA sentence, 16th element
//ros::NodeHandle nh;

sensor_msgs::NavSatFix gpsMsg;
ros::Publisher GPS("GPS", &gpsMsg);

void setup()
{
  Serial.begin(57600);
  ss.begin(GPSBaud);
  nh.initNode();
  nh.advertise(GPS);

}

void loop()
{
  Serial.print(F(" LAT=")); Serial.print(atof(pdop.value())/100,7);
  Serial.print(F("\tLON=")); Serial.println(atof(hdop.value())/100,7);
  delay(100);

  while (ss.available() > 0)
    gps.encode(ss.read());



  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
    //Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
    //Serial.print(F("INVALID"));
  }
  gpsMsg.header.stamp = nh.now();
  gpsMsg.header.frame_id = "map";
  gpsMsg.latitude = atof(pdop.value())/100;
  gpsMsg.longitude = atof(hdop.value())/100;
  gpsMsg.altitude = 0;
  GPS.publish(&gpsMsg);

  nh.spinOnce();
  delay(1);
  Serial.println();
}
