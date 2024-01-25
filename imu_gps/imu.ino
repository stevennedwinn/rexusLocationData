#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <ros.h>
#include <sensor_msgs/Imu.h> // Using IMU message instead of NavSatFix

BNO080 myIMU;
ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

void setup() {
  nh.initNode();
  nh.advertise(imu_pub);

  myIMU.begin(); // Initialize BNO080
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
