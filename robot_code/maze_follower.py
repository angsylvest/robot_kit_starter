#! /usr/bin/python3
from math import sqrt, atan2
import rospy

import motors
import imu
import ultrasonic

# from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, String

import RPi.GPIO as gpio
import time

class DemoRobot:
    def __init__(self):

        # set up relevant statuses to robot as it navigates
        self.isAvoiding = False
        self.isReorienting = False
        self.roll = 0
        self.pitch = 0
        self.theta = 0

        #
        self.distance = 0
        self.status = 'forward'

        # defines robot parts
        self.motors = motors.MotorDriver
        self.imu = imu.IMU
        self.ultrasonic = ultrasonic.sonar

        # init node
        rospy.init_node('simulation', anonymous=False)

        # rospy.init_node('imu', anonymous=True)
        # self.imu_publisher = rospy.Publisher('/imu_orientation', Float32, queue_size=1)

        # rospy.init_node('sonar', anonymous=True)
        self.distance_publisher = rospy.Publisher('/sonar_dist', Float32, queue_size=1)
        self.r = rospy.Rate(15)

        # current pose of robot
        # self.orientation_subscriber = rospy.Subscriber('/imu_orientation', Vector3, self.update_orientation) # will only store orientation info
        self.ultrasonic_subscriber = rospy.Subscriber('/sonar_dist', Float32, self.sonar_info)
        self.motor_subscriber = rospy.Subscriber('/motor_status', String, self.motor_status)

        self.rate = rospy.Rate(10)

    # status updates will change the status of the robot over maze follower sequence
    # will update orientation when encountering an obstacle (will then navigate by turning 90 degrees right or left

    # def update_orientation(self, data):
    #     (roll, pitch, yaw) = data.pose.pose.orientation
    #     self.theta = yaw

    def sonar_info(self, data):
        self.distance = data
        if (self.distance >= 4.0):
            self.isAvoiding = True
        else:
            self.isAvoiding = False


    def motor_status(self, data):
        self.status = data

    ## Make abilities more easy to comprehend (more straightforward)
    def moveRight(self):
        motors.right()
        # Publishing our vel_msg for RVis
        self.motors.motor_publisher.publish('right')

    def moveLeft(self):
        motors.left()
        self.motors.motor_publisher.publish('left')

    def moveBackwards(self):
        motors.backward()
        self.motors.motor_publisher.publish('backward')

    def moveForwards(self):
        motors.forward()
        self.motors.motor_publisher.publish('forward')

    def stop(self):
        motors.stop()
        self.motors.motor_publisher.publish('stop')

    def shutdown(self):
        print('successfully reached goal .. shutting down')

    ## Example tasks that we would make robot do
    def initiateMazeBehavior(self):
        while (True): # must change condition so that it will complete maze course or just end after certain amount of time

            print('beginning loop')
            # ultrasonic sensor stuff
            gpio.output(ultrasonic.trig, False)
            time.sleep(0.1)
            gpio.output(ultrasonic.trig, True)
            time.sleep(0.00001)
            gpio.output(ultrasonic.trig, False)
            while gpio.input(ultrasonic.echo) == 0:
                pulse_start = time.time()
            while gpio.input(ultrasonic.echo) == 1:
                pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17000
            if pulse_duration >= 0.01746:
                # print('time out')
                continue
            elif distance > 300 or distance == 0:
                # print('out of range')
                continue
            distance = round(distance, 3)
            # print ('Distance : %f cm'%distance)

            ultrasonic.dist_sendor(distance) # will send data to
            # ultrasonic.r.sleep()

            while (self.isAvoiding):
                print('initating avoidance behavior')
                while (self.theta > -1.5708):
                    self.moveBackwards()  # intended to rotate robot away from obstacle
                    self.moveRight()  # slight movement backward along the obstacle
                    # Angular velocity in the z-axis.
                    self.rate.sleep()

                while (self.theta < 3.14):
                    self.moveBackwards()  # intended to rotate robot away from obstacle
                    self.moveLeft()  # slight movement backward along the obstacle
                    # Angular velocity in the z-axis.
                    self.rate.sleep()

            self.moveForwards()
            print('moving forward')
            self.rate.sleep()

        # self.stop()
        # print('stopping -- reached end of maze')

## potentially use different bug algorithms to demo to students

if __name__ == '__main__':
    ## Put your functions here to start
    demo_robot = DemoRobot()
    while not rospy.is_shutdown():
        demo_robot.initiateMazeBehavior()
        rospy.spin()

    while rospy.is_shutdown():
        demo_robot.stop()
