#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <ros.h>
#include <sensor_msgs/Imu.h> // For IMU data
#include <sensor_msgs/MagneticField.h> // For Magnetometer data
#include <sensor_msgs/NavSatFix.h> // For GPS data
#include <TinyGPS++.h> // Assuming TinyGPS++ for GPS parsing

// GPS Serial
// GPS is connected via Serial3 for boards like Arduino Mega
TinyGPSPlus gps;

BNO080 myIMU;
ros::NodeHandle nh;

// IMU Publisher
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

// Magnetometer Publisher
sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetometer/data", &mag_msg);

// GPS Publisher
sensor_msgs::NavSatFix gps_msg;
ros::Publisher gps_pub("gps/fix", &gps_msg);

void setup() {
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(mag_pub);
  nh.advertise(gps_pub); // Advertise the GPS publisher

  myIMU.begin(); // Initialize BNO080
  Wire.setClock(400000); // Increase I2C data rate to 400kHz

  // Initialize sensors
  myIMU.enableAccelerometer(50);
  myIMU.enableGyro(50);
  myIMU.enableMagnetometer(50);

  // Initialize GPS Serial
  Serial3.begin(9600); // Adjust this according to your GPS module's baud rate

  Serial.println(F("Calibrated MEMS readings and GPS enabled"));
}

void loop() {
  if (myIMU.dataAvailable()) {
    // IMU and Magnetometer data handling
    // Populate imu_msg with calibrated IMU data
    imu_msg.angular_velocity.x = myIMU.getGyroX();
    imu_msg.angular_velocity.y = myIMU.getGyroY();
    imu_msg.angular_velocity.z = myIMU.getGyroZ();

    imu_msg.linear_acceleration.x = myIMU.getAccelX();
    imu_msg.linear_acceleration.y = myIMU.getAccelY();
    imu_msg.linear_acceleration.z = myIMU.getAccelZ();

    // Populate mag_msg with magnetometer data
    mag_msg.magnetic_field.x = myIMU.getMagX() * 1e-6; // Convert from uT to Tesla
    mag_msg.magnetic_field.y = myIMU.getMagY() * 1e-6;
    mag_msg.magnetic_field.z = myIMU.getMagZ() * 1e-6;

    mag_msg.header.stamp = nh.now();
    mag_msg.header.frame_id = "imu_link"; // Use the same frame_id as the IMU data

    // Publish the messages
    imu_pub.publish(&imu_msg);
    mag_pub.publish(&mag_msg); // Publish the magnetometer data

    // GPS data handling
    while (Serial3.available() > 0) {
      if (gps.encode(Serial3.read())) {
        if (gps.location.isValid()) {
          gps_msg.latitude = gps.location.lat();
          gps_msg.longitude = gps.location.lng();
        }
        if (gps.altitude.isValid()) {
          gps_msg.altitude = gps.altitude.meters();
        }
        gps_msg.header.stamp = nh.now();
        gps_msg.header.frame_id = "gps";

        gps_pub.publish(&gps_msg);
      }
    }
  }

  nh.spinOnce();
  delay(100);
}
