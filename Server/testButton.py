import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(6,GPIO.OUT)
GPIO.setup(5,GPIO.OUT)
#GPIO.output(5,1)
GPIO.output(6,1)