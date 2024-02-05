# rexusLocationData

`gps.ino`: serial print gps data with software serial (uno/mega)\
`gps_tiny.ino`: serial print gps data with tinygps++ library (mega)\
`gps_publish.ino`: gps publisher (uno)\
`gps_publish2.ino`: gps publisher (mega)\
\
`imu_serialPrint.ino`: raw data serial print (uno/mega)\
`imu.ino`: imu publisher w/o magnetometer (uno/mega)\
`imu_with_magneto.ino`: imu publisher with magnetometer (uno/mega)\
`imu_n_gps.ino`: imu, magneto, and gps publisher (uno/mega)\
\
For publishing msg with rosserial
|                |In arduino device              |In host devices              |
|----------------|-------------------------------|-----------------------------|
|First step      |Upload the code                |                             |
|Second step     |                               |Run `roscore`                |
|Third step      |                               |Run `rosrun rosserial_python serial_node.py /dev/ttyUSB0` or `rosrun rosserial_arduino serial_node.py _port:=/dev/ttyUSB0`|
> For re-uploading arduino file, please ensure that the terminal at step 3 has stopped before re-uploading

