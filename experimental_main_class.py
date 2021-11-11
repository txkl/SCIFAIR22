# TIERNAN LINDAUER
# LAST EDIT 11/10/21
# LAST BACKED UP VERSION 11/10/21 6:04 PM
# EXPERIMENTAL SCIFAIR MAIN PROGRAM

# IMPORT MODULES
#import cv2
# import math
# import matplotlib.pyplot as plt
#import numpy as np

from cv2 import VideoCapture
from cv2 import QRCodeDetector
import RPi.GPIO as GPIO
from time import time
from time import sleep
import Encoder
from gpiozero import DistanceSensor

# ROBOT CLASS
class Robot:
    def __init__(self):
        # OBJECT DEFINITIONS AND SETUP
        left_enc = Encoder.Encoder(23, 24)
        right_enc = Encoder.Encoder(20, 21)

        forward_sensor = DistanceSensor(echo=7, trigger=8)
        bottom_sensor = DistanceSensor(echo=15, trigger=14)

        cap = VideoCapture(0)
        detector = QRCodeDetector()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(12, GPIO.OUT)

        right_pwm = GPIO.PWM(12, 400)
        right_pwm.start(0)
        left_pwm = GPIO.PWM(13, 400)
        left_pwm.start(0)

        # VARIABLES
        self.stage = 0
        self.alphaL = 0.11
        self.alphaR = 0.25
        self.prev_l_speed = 400
        self.prev_r_speed = 400
        self.left_speed = 400
        self.right_speed = 400
        self.LI = 0
        self.RI = 0
        self.mark_now = 0

        # define objects as global
        global left_enc, right_enc, right_pwm, left_pwm, bottom_sensor, forward_sensor, cap, detector

    # FUNCTIONS

    @staticmethod
    def get_image(self):
        _, img = cap.read()
        data, bbox, _ = detector.detectAndDecode(img)

        if (bbox is not None):
            box_width = abs(int(bbox[0][1][0]) - int(bbox[0][0][0]))
            x_loc = (int(bbox[0][0][0]) + int(bbox[0][1][0])) / 2
            _, width, _ = img.shape

            if data == "left" or data == "right" or data == "target":
                return box_width, x_loc, data, width
            return box_width, x_loc, "null", width
        return -1, -1, "null", -1

    @staticmethod
    def make_pulse(power):
        if power > 0:
            abs(power)
            return 990 * power + 1490
        elif power < 0:
            return 990 * power + 1510
        else:
            return 1500

    @staticmethod
    def duty_cycle(self, pulse_width):
        return (float(pulse_width) / 1000000.0) * 400 * 100

    def drive_right_motor(self, pwr):
        right_pwm.ChangeDutyCycle(self.duty_cycle(self.make_pulse(-pwr)))

    def drive_left_motor(self, pwr):
        left_pwm.ChangeDutyCycle(self.duty_cycle(self.make_pulse(pwr)))

    @staticmethod
    def left_stop(self):
        left_pwm.ChangeDutyCycle(0)

    @staticmethod
    def right_stop(self):
        right_pwm.ChangeDutyCycle(0)

    def check(self):
        if bottom_sensor.distance * 100 >= 10:
            exit()
        if forward_sensor.distance * 100 <= 20:
            self.left_stop()
            self.right_stop()
            sleep(5)
            if forward_sensor.distance * 100 <= 15 and self.stage == 0:
                exit()

    @staticmethod
    def get_speed(self, x):
        if x == "left":
            left_enc.write()

            bt = time()
            while abs(left_enc.read()) < 10:
                sleep(.0001)
            mn = time()
            ls = 10 / (mn - bt)

            return ls, mn
        elif x == "right":
            right_enc.write()
            bt = time.time()
            while abs(right_enc.read()) < 10:
                sleep(.0001)
            mn = time()
            rs = 10 / (mn - bt)

            return rs, mn

    def forward_PID(self):
        mark_early = time()

        self.drive_left_motor(self.alphaL)
        self.drive_right_motor(self.alphaR)

        self.left_speed, self.mark_now = self.get_speed("left")
        self.LI += 0.001 * (410 - self.left_speed)
        self.alphaL += (0.05 * (410 - self.left_speed) + self.LI + (
                0.001 * (self.prev_l_speed - self.left_speed) / (self.mark_now - mark_early))) * 0.00134

        self.right_speed, self.mark_now = self.get_speed("right")
        self.RI += 0.001 * (430 - self.right_speed)
        self.alphaR += (0.05 * (430 - self.right_speed) + self.RI + (
                0.001 * (self.prev_r_speed - self.right_speed) / (self.mark_now - mark_early))) * 0.00134

        self.prev_l_speed = self.left_speed
        self.prev_r_speed = self.right_speed

    def turn_PID(self, x,y):
        mark_early = time()
        time_before = time()

        if x=="left":
            t_alpha_l = -0.25
            t_alpha_r = 0.25
            t_prev_l_speed = 400
            t_LI = 0

            while abs(left_enc.read()) < 480*(y / 90) or time()-time_before > 7:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_left_speed, t_mark_now = self.get_speed("left")
                t_LI += 0.001*(400-t_left_speed)
                t_alpha_l -= (0.05*(400-t_left_speed)+t_LI+(0.001*(t_prev_l_speed-t_left_speed)/(t_mark_now-mark_early)))*0.00134

                t_prev_l_speed = t_left_speed

        elif x=="right":
            t_alpha_l = 0.13
            t_alpha_r = -0.13

            t_prev_r_speed = 400

            t_RI = 0

            while abs(right_enc.read()) < 480*(y / 90) or time()-time_before > 7:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_right_speed, t_mark_now = self.get_speed("right")
                t_RI += 0.001*(400-abs(t_right_speed))
                t_alpha_r += 0.01*(0.001*abs(400-abs(t_right_speed))+t_RI+(0.001*(t_prev_r_speed-t_right_speed)/(t_mark_now-mark_early)))*0.00134

                t_prev_r_speed = t_right_speed
        return time()-time_before

robot = Robot()

runtime_loop = True

while runtime_loop:
    robot.forward_PID()
    width, loc, info, size = robot.get_image()
    if width != -1:
        if info == "target":
            robot.stage = 1
        if info == "left" or info == "right":
            if width > 200:
                robot.turn_PID(info, 90)
