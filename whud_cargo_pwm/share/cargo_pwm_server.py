#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import time

class CargoPWM:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/cargo_control",String,callback=self.control_callback)
        self.curr_val = GPIO.HIGH
        # 可用引脚，共12、35、38、40四个，为线路集中选后三
        self.avail_pin = [35, 38, 40]
        # region to drop cargo, E for empty(no operation)
        self.control = "E"
        # pulse width, second, decides final angle
        self.pulsewidth = 0.0005

        #GPIO设置，BOARD40pin，用作输出，高电平起始
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.avail_pin, GPIO.OUT)
        GPIO.output(self.avail_pin, GPIO.LOW)

        rospy.spin()

    def control_callback(self,data):
        self.control = data.data
        GPIO.output(self.avail_pin, GPIO.LOW)
        if self.control == "E":
            return
        elif self.control == "A":
            cargo = 0
        elif self.control == "B":
            cargo = 1
        elif self.control == "C":
            cargo = 2
        else: return
        # 20 is to gurantee the time enough for switching
        # out of experience(test), different from calculation(12)
        for i in range(20):
            GPIO.output(self.avail_pin[cargo], GPIO.HIGH)
            time.sleep(self.pulsewidth)
            GPIO.output(self.avail_pin[cargo], GPIO.LOW)
            time.sleep(0.02 - self.pulsewidth)
        GPIO.output(self.avail_pin[cargo], GPIO.LOW)

def main():
    rospy.init_node("cargo_pwm_node",anonymous=True)
    CargoControl = CargoPWM()

main()