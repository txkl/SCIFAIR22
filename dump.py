import RPi.GPIO as GPIO
from time import sleep
GPIO.setwarnings(False)
servoPIN = 11
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(1) # Initialization

sleep(1)
p.ChangeDutyCycle(12)
sleep(1)
p.ChangeDutyCycle(4)
