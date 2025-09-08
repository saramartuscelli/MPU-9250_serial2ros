# MPU-9250_serial2ros

This repository contains two main components:

- **Arduino firmware** for reading raw accelerometer and gyroscope data from the MPU-9250 sensor through I2C communication protocol and streaming them via the serial port.

- **Python script** for reading the serial data and publishing it to ROS as a Vector3Stamped topic.

The goal is to provide a lightweight bridge between the MPU-9250 IMU and ROS, ensuring transparency and full control over the data acquisition process.
