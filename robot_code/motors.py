#! /usr/bin/python3
# MOTOR STUFF

# will have to update so that import goes to correct place
from PCA9685 import PCA9685
import time
import rospy
from std_msgs.msg import String

class MotorDriver():
    def __init__(self):
        self.Dir = [
            'forward',
            'backward',
        ]
        self.pwm = PCA9685(0x40, debug=False)
        self.pwm.setPWMFreq(50)

        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

        self.status = ''

        # sets up ros node to send status messages
        rospy.init_node('motors', anonymous=True)
        self.motor_publisher = rospy.Publisher('/motor_status', String, queue_size=1)

    def MotorRun(self, motor, index, speed):
        if speed > 100:
            return
        if(motor == 0):
            self.pwm.setDutycycle(self.PWMA, speed)
            if(index == self.Dir[0]):
                print ("1")
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            else:
                print ("2")
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)

        else:
            self.pwm.setDutycycle(self.PWMB, speed)
            if(index == self.Dir[0]):
                print ("3")
                self.pwm.setLevel(self.BIN1, 0)
                self.pwm.setLevel(self.BIN2, 1)
            else:
                print ("4")
                self.pwm.setLevel(self.BIN1, 1)
                self.pwm.setLevel(self.BIN2, 0)


    def MotorStop(self, motor):
        if (motor == 0):
            self.pwm.setDutycycle(self.PWMA, 0)
        else:
            self.pwm.setDutycycle(self.PWMB, 0)

    def motor_sendor(self, data):
        self.motor_publisher.publish(data)

    def forward(self,  duration):
        self.MotorRun(0, 'forward', 100)
        self.MotorRun(1, 'forward', 100)
        self.motor_sendor('forward')
        time.sleep(duration)

    def backward(self, duration):
        self.MotorRun(0, 'backward', 100)
        self.MotorRun(1, 'backward', 100)
        self.motor_sendor('backward')
        time.sleep(duration)

    # need to troubleshoot to make sure correct motor is being moved
    def right(self, duration):
        self.MotorRun(0, 'forward', 100)
        self.MotorRun(1, 'backward', 100)
        self.motor_sendor('right')
        time.sleep(duration)

    def left(self, duration):
        self.MotorRun(0, 'backward', 100)
        self.MotorRun(1, 'forward', 100)
        self.motor_sendor('left')
        time.sleep(duration)

    def stop(self):
        self.MotorStop(0)
        self.MotorStop(1)

# sample script
# print("this is a motor driver test code")
Motor = MotorDriver()
Motor.stop()
#
# print("forward 2 s")
# Motor.MotorRun(0, 'forward', 100)
# Motor.MotorRun(1, 'forward', 100)
# time.sleep(2)
#
# print("backward 2 s")
# Motor.MotorRun(0, 'backward', 100)
# Motor.MotorRun(1, 'backward', 100)
# time.sleep(2)
#
# print("stop")
# Motor.MotorStop(0)
# Motor.MotorStop(1)

