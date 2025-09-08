# MPU-9250_serial2ros

This repository contains two main components:

- **Arduino firmware** for reading raw accelerometer and gyroscope data from the MPU-9250 sensor through I2C communication protocol and streaming them via the serial port.

- **Python script** for reading the serial data and publishing it to ROS as Vector3Stamped topics.

The goal is to provide a lightweight bridge between the MPU-9250 IMU and ROS, ensuring transparency and full control over the data acquisition process.


## Features
- Configurable data acquisition (accelerometer, gyroscope, or both).  
- Arduino firmware for raw IMU data reading and transmission via `Serial.write`.  
- Python ROS node to decode the stream and publish `geometry_msgs/Vector3Stamped`.  
- Timestamp alignment between Arduino `micros()` and ROS time.

## ROS Topics
- `/accelerometer/data` → 3-axis accelerometer (`Vector3Stamped`)  
- `/gyroscope/data` → 3-axis gyroscope (`Vector3Stamped`)  

## Usage
1. Flash the Arduino sketch (`arduino/`) to your board.  
2. Launch the Python ROS node:
   ```bash
   rosrun mpu9250_serial2ros imu_publisher.py
   ```
   or:
   ```bash
   python3 imu_publisher.py
   ```

>NOTE: configure acquisition mode (```READ_ACC```, ```READ_GYRO```) in both Arduino and Python code.

## Repository Structure
```bash
.
├── arduino/
│   └── SensorReadingIMU.ino
├── scripts/
│   └── imu_publisher.py
└── README.md
```
