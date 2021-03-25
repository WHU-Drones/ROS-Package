#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import time
import threading

#为解决Python无spinOnce函数，使用多线程执行spin
def spin():
    rospy.spin()

class CargoPWM:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/cargo_control",String,callback=self.control_callback)
        self.curr_val = GPIO.HIGH
        self.avail_pin = [35, 38, 40] #使用的引脚
        self.control = "111" #对应三个货仓的控制信号
        #GPIO设置
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.avail_pin, GPIO.OUT)
        GPIO.output(self.avail_pin, GPIO.HIGH)

        add_thread = threading.Thread(target = spin)
        add_thread.start()

    def pwm_generation(self):
        try:
            while True:
                # parse control signal
                for i in range(3):
                    if self.control[i] == '0':
                        # low if released
                        GPIO.output(self.avail_pin[i], GPIO.LOW)
                    else:
                        # pwm if hold
                        GPIO.output(self.avail_pin[i], self.curr_val)
                if self.curr_val == GPIO.HIGH:
                    time.sleep(0.008)  # 0.0001 * duty
                else: 
                    time.sleep(0.002)  # 1 - 0.0001 * duty
                # change state
                self.curr_val ^= GPIO.HIGH
        finally:
            GPIO.cleanup()

    def control_callback(self,data):
        self.control = data.data

def main():
    rospy.init_node("cargo_pwm_node",anonymous=True)
    CargoControl = CargoPWM()
    CargoControl.pwm_generation()

main()