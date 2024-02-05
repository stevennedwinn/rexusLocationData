#include <TinyGPS++.h>
#include <ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>

// Define the hardware serial port to use for GPS communication
// For Mega, we can use Serial1, Serial2, or Serial3
// Example: Using Serial3 which is on pins 19 (RX) and 18 (TX) on Mega
TinyGPSPlus gps;

ros::NodeHandle nh;

sensor_msgs::NavSatFix gpsMsg;
ros::Publisher GPS("GPS", &gpsMsg);

void setup()
{
  Serial.begin(57600); // Start serial communication with the computer
  Serial3.begin(9600); // Start serial communication with the GPS module on Serial3
  nh.initNode();
  nh.advertise(GPS);
}

void loop()
{
  while (Serial3.available() > 0) // Check if there is data from the GPS
  {
    if (gps.encode(Serial3.read())) // Encode the GPS data
    {
      // Once valid data is received, publish it
      gpsMsg.header.stamp = nh.now();
      gpsMsg.header.frame_id = "map"; // This frame_id can be changed according to your TF configuration
      gpsMsg.latitude = gps.location.lat();
      gpsMsg.longitude = gps.location.lng();
      gpsMsg.altitude = gps.altitude.meters();
      GPS.publish(&gpsMsg);
    }
  }

  nh.spinOnce(); // Handle ROS callbacks
  delay(200); // Adjust delay as needed
}
