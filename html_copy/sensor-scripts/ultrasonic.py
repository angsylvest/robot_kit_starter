# ULTRASONIC SENSOR
import RPi.GPIO as gpio
import time
import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class sonar:
    def __init__(self):
        gpio.setmode(gpio.BCM)
        self.trig = 27  # 7th
        self.echo = 17  # 6th

        gpio.setup(self.trig, gpio.OUT)
        gpio.setup(self.echo, gpio.IN)

s=sonar()
# print(s.trig)
# time.sleep(0.5)
#
# # sample script
# print ('-----------------------------------------------------------------sonar')
try :
    while True :
        gpio.output(s.trig, False)
        time.sleep(0.1)
        gpio.output(s.trig, True)
        time.sleep(0.00001)
        gpio.output(s.trig, False)
        while gpio.input(s.echo) == 0 :
            pulse_start = time.time()
        while gpio.input(s.echo) == 1 :
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

except (KeyboardInterrupt, SystemExit):
    # gpio.cleanup()
    sys.exit(0)
#
