#!/usr/bin/env python3
import serial
import time
import struct

import rospy
from geometry_msgs.msg import Vector3Stamped


# CONFIGURATION
ARDUINO_PORT = '/dev/ttyACM0'

READ_ACC = True
READ_GYRO = True

if READ_ACC and READ_GYRO: ARDUINO_BAUDRATE = 500000
else: ARDUINO_BAUDRATE = 230400

FLOAT_SIZE = 4
HEADER_SIZE = 1   # sync byte 0xAA
TIME_SIZE = 4     # micros()

DATA_NUM = 0
if READ_ACC:
    DATA_NUM += 3
if READ_GYRO:
    DATA_NUM += 3

DATA_SIZE = (DATA_NUM * FLOAT_SIZE) + TIME_SIZE
fmt = f'={DATA_NUM}fL'


# TIME SYNC
t0_sensor = None
t0_ros = None

def convert_timestamp(micros):
    """Converts Arduino micros() in rospy.Time."""
    global t0_sensor, t0_ros
    if t0_sensor is None:
        t0_sensor = micros
        t0_ros = rospy.Time.now()
    dt = (micros - t0_sensor) * 1e-6
    return t0_ros + rospy.Duration.from_sec(dt)


# MAIN
def main():
    rospy.init_node("imu_publisher", anonymous=True)

    pub_acc = None
    pub_gyro = None

    if READ_ACC:
        pub_acc = rospy.Publisher("/accelerometer/data", Vector3Stamped, queue_size=10)
    if READ_GYRO:
        pub_gyro = rospy.Publisher("/gyroscope/data", Vector3Stamped, queue_size=10)

    arduino.write(b'S')
    print("Reading data from serial port... Press CTRL+C to stop.")

    while not rospy.is_shutdown():
        try:
            sync = arduino.read(HEADER_SIZE)
            if sync != b'\xAA':
                continue

            raw = arduino.read(DATA_SIZE)
            if len(raw) != DATA_SIZE:
                continue

            values = struct.unpack(fmt, raw)
            micros = values[-1]
            stamp = convert_timestamp(micros)

            idx = 0
            if READ_ACC:
                acc_msg = Vector3Stamped()
                acc_msg.header.stamp = stamp
                acc_msg.header.frame_id = "accelerometer_link"
                acc_msg.vector.x = values[idx]; idx += 1
                acc_msg.vector.y = values[idx]; idx += 1
                acc_msg.vector.z = values[idx]; idx += 1
                pub_acc.publish(acc_msg)

            if READ_GYRO:
                gyro_msg = Vector3Stamped()
                gyro_msg.header.stamp = stamp
                gyro_msg.header.frame_id = "gyroscope_link"
                gyro_msg.vector.x = values[idx]; idx += 1
                gyro_msg.vector.y = values[idx]; idx += 1
                gyro_msg.vector.z = values[idx]; idx += 1
                pub_gyro.publish(gyro_msg)

        except Exception as e:
            print(f"\n[IMU] Error: {e}")


# ENTRYPOINT
if __name__ == "__main__":
    try:
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=0.01)
        time.sleep(2)
        arduino.reset_input_buffer()
        time.sleep(0.5)
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        if arduino.is_open:
            arduino.close()
            print("\nSerial closed.")
