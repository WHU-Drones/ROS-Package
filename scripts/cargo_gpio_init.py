import Jetson.GPIO as GPIO

avail_pin = [35, 38, 40]
GPIO.setmode(GPIO.BOARD)
GPIO.setup(avail_pin, GPIO.OUT)
GPIO.output(avail_pin, GPIO.LOW)
