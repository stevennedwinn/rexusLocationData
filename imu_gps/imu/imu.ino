#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <ros.h>
#include <sensor_msgs/Imu.h> // Using IMU message instead of NavSatFix

// VCC to 3.3V
// GND to GND
// SDA to analog4 for uno
// SCL to analog5 for uno

BNO080 myIMU;
ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

void setup() {
  nh.initNode();
  nh.advertise(imu_pub);

  myIMU.begin(); // Initialize BNO080

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableAccelerometer(50);    //We must enable the accel in order to get MEMS readings even if we don't read the reports.
  myIMU.enableRawAccelerometer(50); //Send data update every 50ms
  myIMU.enableGyro(50);
  myIMU.enableRawGyro(50);
  myIMU.enableMagnetometer(50);
  myIMU.enableRawMagnetometer(50);

  Serial.println(F("Raw MEMS readings enabled"));
}

void loop() {
  if (myIMU.dataAvailable()) {
    // Populate imu_msg with IMU data
    imu_msg.angular_velocity.x = myIMU.getGyroX();
    imu_msg.angular_velocity.y = myIMU.getGyroY();
    imu_msg.angular_velocity.z = myIMU.getGyroZ();

    imu_msg.linear_acceleration.x = myIMU.getAccelX();
    imu_msg.linear_acceleration.y = myIMU.getAccelY();
    imu_msg.linear_acceleration.z = myIMU.getAccelZ();

    // Publish the message
    imu_pub.publish(&imu_msg);
  }
  nh.spinOnce();
  delay(100);
}
