#! /usr/bin/python3

# import parts of robot to control & coordinate
import motors
import imu
import ultrasonic

import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
from std_msgs.msg import Float32


class robot_car:
    def __init__(self):
        self.motors = motors.MotorDriver
        self.imu = imu.IMU
        # self.ultrasonic = ultrasonic.sonar

        rospy.init_node('motors', anonymous=True)
        self.distance_publisher = rospy.Publisher('/motor_status', Float32, queue_size=1)
        self.r = rospy.Rate(15)

        rospy.init_node('imu', anonymous=True)
        self.distance_publisher = rospy.Publisher('/imu_info', Float32, queue_size=1)
        self.r = rospy.Rate(15)

        rospy.init_node('sonar', anonymous=True)
        self.distance_publisher = rospy.Publisher('/sonar_dist', Float32, queue_size=1)
        self.r = rospy.Rate(15)

    # will need to adapt to fit with imu and motor info
    def motor_sendor(self, data):
        # data = Float32()
        # data.data = dist
        self.distance_publisher.publish(data)

    def imu_sendor(self, data):
        # data = Float32()
        # data.data = dist
        self.distance_publisher.publish(data)

    def dist_sendor(self, dist):
        data = Float32()
        data.data = dist
        self.distance_publisher.publish(data)


# sample that runs ROS
robo_car = robot_car

print('-----------------------------------------------------------------sonar')
try:
    while True:
        # ultrasonic stuff
        gpio.output(robo_car.ultrasonic.sonar.trig, False)
        time.sleep(0.1)
        gpio.output(robo_car.ultrasonic.sonar.trig, True)
        time.sleep(0.00001)
        gpio.output(robo_car.ultrasonic.sonar.trig, False)
        while gpio.input(robo_car.ultrasonic.sonar.echo) == 0:
            pulse_start = time.time()
        while gpio.input(robo_car.ultrasonic.sonar.echo) == 1:
            pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17000
        if pulse_duration >= 0.01746:
            print('time out')
            continue
        elif distance > 300 or distance == 0:
            print('out of range')
            continue
        distance = round(distance, 3)
        print ('Distance : %f cm'%distance)

        ## sample code to make sure stuff still works
        # motor stuff - remember to add PCA9685 import PCA9685
        print("this is a motor driver test code")
        Motor = robo_car.motors.MotorDriver()

        print("forward 2 s")
        Motor.MotorRun(0, 'forward', 100)
        Motor.MotorRun(1, 'forward', 100)
        time.sleep(2)

        print("backward 2 s")
        Motor.MotorRun(0, 'backward', 100)
        Motor.MotorRun(1, 'backward', 100)
        time.sleep(2)

        print("stop")
        Motor.MotorStop(0)
        Motor.MotorStop(1)

        # imu
        print("Temperature: {} degrees C".format(robo_car.imu.sensor.temperature))
        """
        print(
            "Temperature: {} degrees C".format(temperature())
        )  # Uncomment if using a Raspberry Pi
        """

        print("Accelerometer (m/s^2): {}".format(robo_car.imu.sensor.acceleration))
        print("Magnetometer (microteslas): {}".format(robo_car.imu.sensor.magnetic))
        print("Gyroscope (rad/sec): {}".format(robo_car.imu.sensor.gyro))
        print("Euler angle: {}".format(robo_car.imu.sensor.euler))
        print("Quaternion: {}".format(robo_car.imu.sensor.quaternion))
        print("Linear acceleration (m/s^2): {}".format(robo_car.imu.sensor.linear_acceleration))
        print("Gravity (m/s^2): {}".format(robo_car.imu.sensor.gravity))
        print()

        # moving sensor data to topic
        robo_car.ultrasonic.sensor.dist_sendor(distance)
        robo_car.ultrasonic.sensor.r.sleep()


except (KeyboardInterrupt, SystemExit):
    gpio.cleanup()
    sys.exit(0)
except:
    gpio.cleanup()

