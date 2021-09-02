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
        self.ultrasonic = ultrasonic.sonar
        self.distance = 0
        self.pulse_duration = 0

        # self.r = rospy.Rate(15)

        rospy.init_node('imu', anonymous=True)
        self.imu_publisher = rospy.Publisher('/imu_info', Float32, queue_size=1)
        # self.r = rospy.Rate(15)

        rospy.init_node('sonar', anonymous=True)
        self.distance_publisher = rospy.Publisher('/sonar_dist', Float32, queue_size=1)
        self.r = rospy.Rate(15)

    # will need to adapt to fit with imu and motor info

    def imu_sendor(self, data):
        # data = Float32()
        # data.data = dist
        self.imu_publisher.publish(data)

    def dist_sendor(self, dist):
        data = Float32()
        data.data = dist
        self.distance_publisher.publish(data)

    def ultra_sonic_pulse(self):
        gpio.output(self.ultrasonic.sonar.trig, False)
        time.sleep(0.1)
        gpio.output(self.ultrasonic.sonar.trig, True)
        time.sleep(0.00001)
        gpio.output(self.ultrasonic.sonar.trig, False)
        while gpio.input(self.ultrasonic.sonar.echo) == 0:
            pulse_start = time.time()
        while gpio.input(self.ultrasonic.sonar.echo) == 1:
            pulse_end = time.time()
        self.pulse_duration = pulse_end - pulse_start
        self.distance = self.pulse_duration * 17000


# sample that runs ROS
robo_car = robot_car()

print('-----------------------------------------------------------------sonar')
try:
    while True:
        # ultrasonic stuff
        robo_car.ultra_sonic_pulse()
        if robo_car.pulse_duration >= 0.01746:
            print('time out')
            continue
        elif robo_car.distance > 300 or robo_car.distance == 0:
            print('out of range')
            continue
        robo_car.distance = round(robo_car.distance, 3)
        print ('Distance : %f cm'%robo_car.distance)

        ## sample code to make sure stuff still works
        # motor updates will occur as function call is made

        print("this is a motor driver test code")
        Motor = robo_car.motors.MotorDriver()

        print("forward 2 s")
        Motor.forward(2)
        # Motor.MotorRun(0, 'forward', 100)
        # Motor.MotorRun(1, 'forward', 100)
        # time.sleep(2)

        print("backward 2 s")
        Motor.backward(2)
        # Motor.MotorRun(0, 'backward', 100)
        # Motor.MotorRun(1, 'backward', 100)
        # time.sleep(2)

        print("stop")
        Motor.stop()
        # Motor.MotorStop(0)
        # Motor.MotorStop(1)

        # imu
        # print("Temperature: {} degrees C".format(robo_car.imu.sensor.temperature))
        # """
        # print(
        #     "Temperature: {} degrees C".format(temperature())
        # )  # Uncomment if using a Raspberry Pi
        # """
        #
        # print("Accelerometer (m/s^2): {}".format(robo_car.imu.sensor.acceleration))
        # print("Magnetometer (microteslas): {}".format(robo_car.imu.sensor.magnetic))
        # print("Gyroscope (rad/sec): {}".format(robo_car.imu.sensor.gyro))
        # print("Euler angle: {}".format(robo_car.imu.sensor.euler))
        # print("Quaternion: {}".format(robo_car.imu.sensor.quaternion))
        # print("Linear acceleration (m/s^2): {}".format(robo_car.imu.sensor.linear_acceleration))
        # print("Gravity (m/s^2): {}".format(robo_car.imu.sensor.gravity))
        # print()

        # moving sensor data to topic
        robo_car.ultrasonic.dist_sendor(robo_car.distance)
        robo_car.imu_sendor(robo_car.imu.sensor.euler)

        robo_car.ultrasonic.sensor.r.sleep()


except (KeyboardInterrupt, SystemExit):
    gpio.cleanup()
    sys.exit(0)
except:
    gpio.cleanup()

