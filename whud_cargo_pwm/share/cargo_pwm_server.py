#!/usr/bin/env python
#-*- coding:utf-8 -*-
import rospy
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import time
import threading

# 本程序所用引脚基于Jetson Xavier NX
#为解决Python无spinOnce函数，使用多线程执行spin
def spin():
    rospy.spin()

class CargoPWM:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/cargo_control",String,callback=self.control_callback)
        self.curr_val = GPIO.HIGH
        # 可用引脚，共12、35、38、40四个，为线路集中选后三
        self.avail_pin = [35, 38, 40]
        # 从左到右对应引脚号从小到大三个货仓的控制信号
        self.control = "111"
        # pwm默认占空比，0～1
        self.duty = 0.8
        # pwm默认频率(Hz)，建议尽量低
        self.frequency = 100
        # 计算高、低电平时间，避免传入除法式
        self.HL_time = [self.duty / self.frequency, (1 - self.duty) / self.frequency]

        #GPIO设置，BOARD40pin，用作输出，高电平起始
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.avail_pin, GPIO.OUT)
        GPIO.output(self.avail_pin, GPIO.HIGH)

        # 用另一线程运行spin
        add_thread = threading.Thread(target = spin)
        add_thread.start()

    def pwm_generation(self):
        try:
            while True:
                # 解析控制信号
                for i in range(3):
                    if self.control[i] == '0':
                        # 为0时投下货仓（电磁铁无磁力）
                        GPIO.output(self.avail_pin[i], GPIO.LOW)
                    else:
                        # 为1时保持货仓（电磁铁有磁力）
                        GPIO.output(self.avail_pin[i], self.curr_val)
                # 控制占空比，频率100Hz
                if self.curr_val == GPIO.HIGH:
                    time.sleep(self.HL_time[0]) 
                else: 
                    time.sleep(self.HL_time[1]) 
                # pwm高低电平翻转
                self.curr_val ^= GPIO.HIGH
        finally:
            # GPIO恢复初始设置
            GPIO.cleanup()

    def control_callback(self,data):
        self.control = data.data

def main():
    rospy.init_node("cargo_pwm_node",anonymous=True)
    CargoControl = CargoPWM()
    CargoControl.pwm_generation()

main()