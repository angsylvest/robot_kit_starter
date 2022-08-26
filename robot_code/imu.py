<<<<<<< HEAD
import adafruit_bno055
=======
#! /usr/bin/python3

>>>>>>> dfbcb8fa8f18901b01fd4244ab278675756ce29b
# import statements
import time
import rospy
import board
<<<<<<< HEAD
=======
import adafruit_bno055
from std_msgs.msg import String
>>>>>>> dfbcb8fa8f18901b01fd4244ab278675756ce29b

# imu part of robot
class IMU:
    def __init__(self):
        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.imu_publisher = rospy.Publisher('/imu_orientation', String, queue_size=1)
        self.last_val = 0xFFFF

    def temperature(self):
        global last_val  # pylint: disable=global-statement
        result = self.sensor.temperature
        if abs(result - last_val) == 128:
            result = self.sensor.temperature
            if abs(result - last_val) == 128:
                return 0b00111111 & result
        last_val = result
        return result

    def orientation_sendor(self, dist):
        self.imu_publisher.publish(dist)

# example_imu = IMU()

# script to run to test
# while True:
#     print("Temperature: {} degrees C".format(example_imu.sensor.temperature))
#     """
#     print(
#         "Temperature: {} degrees C".format(temperature())
#     )  # Uncomment if using a Raspberry Pi
#     """
#
#     print("Accelerometer (m/s^2): {}".format(example_imu.sensor.acceleration))
#     print("Magnetometer (microteslas): {}".format(example_imu.sensor.magnetic))
#     print("Gyroscope (rad/sec): {}".format(example_imu.sensor.gyro))
#     roll, pitch, yaw = example_imu.sensor.euler
#     print("Euler angle: {}".format(yaw))
#     print("Quaternion: {}".format(example_imu.sensor.quaternion))
#     print("Linear acceleration (m/s^2): {}".format(example_imu.sensor.linear_acceleration))
#     print("Gravity (m/s^2): {}".format(example_imu.sensor.gravity))
#     print()
#
    # time.sleep(1)

