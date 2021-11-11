#TIERNAN LINDAUER
#LAST EDIT 11/10/21
#LAST BACKED UP VERSION 11/10/21 6:04 PM
#EXPERIMENTAL SCIFAIR MAIN PROGRAM

#IMPORT MODULES
import cv2
import RPi.GPIO as GPIO
from time import time
from time import sleep
import math
import encoder
from gpiozero import DistanceSensor
import numpy as np
import matplotlib.pyplot as plt

#ROBOT CLASS
class robot:
  def __init__(self):
    #OBJECT DEFINITIONS AND SETUP
    left_enc = Encoder.Encoder(23, 24)
    right_enc = Encoder.Encoder(20, 21)

    forward_sensor = DistanceSensor(echo=7, trigger=8)
    bottom_sensor = DistanceSensor(echo=15, trigger=14)

    cap = cv2.VideoCapture(0)
    detector = cv2.QRCodeDetector()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(13, GPIO.OUT)
    GPIO.setup(12, GPIO.OUT)

    right_pwm = GPIO.PWM(12, 400)
    right_pwm.start(0)
    left_pwm = GPIO.PWM(13, 400)
    left_pwm.start(0)
	#VARIABLES
	stage = 0
	endgame = False
	cooldown = 0
	last_movement = ""
	target = False
  
	alphaL = 0.11
	alphaR = 0.25
	prev_l_speed = 400
	prev_r_speed = 400

	LI = 0
	RI = 0
	mark_now = 0
	
  #FUNCTIONS
  def make_pulse(power):
      if power > 0:
          abs(power)
          return 990*power + 1490
      elif power < 0:
          return 990*power + 1510
      else:
          return 1500

  def duty_cycle(pulse_width):
      return(float(pulse_width)/1000000.0)*400*100

  def drive_right_motor(pwr):
      right_pwm.ChangeDutyCycle(duty_cycle(make_pulse(-pwr)))
  def drive_left_motor(pwr):
      left_pwm.ChangeDutyCycle(duty_cycle(make_pulse(pwr)))
  def left_stop():
      left_pwm.ChangeDutyCycle(0)
  def right_stop():
      right_pwm.ChangeDutyCycle(0)
  def check(stage):
      if bottom_sensor.distance * 100 >= 10:
          exit()
      if forward_sensor.distance * 100 <= 20:
          left_stop()
          right_stop()
          time.sleep(5)
          if forward_sensor.distance * 100 <= 15 and stage == 0:
              exit()

	def forward_PID():
		mark_early = time.time()

		drive_left_motor(alphaL)
		drive_right_motor(alphaR)

		self.left_speed, self.mark_now = getSpeed("left")
		self.LI += 0.001*(410-self.left_speed)
		self.alphaL += (0.05*(410-self.left_speed)+LI+(0.001*(self.prev_l_speed-self.left_speed)/(self.mark_now-self.mark_early)))*0.00134

		right_speed, mark_now = getSpeed("right")
		RI += 0.001*(430-right_speed)
		alphaR += (0.05*(430-right_speed)+RI+(0.001*(prev_r_speed-right_speed)/(mark_now-mark_early)))*0.00134

		plt.scatter(float("{:.2f}".format(time.time()-time_before)),alphaR,color="blue")

		plt.pause(0.05)

		self.prev_l_speed = self.left_speed
		self.prev_r_speed = self.right_speed
