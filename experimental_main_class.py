# TIERNAN LINDAUER
# LAST EDIT 11/16/21
# LAST BACKED UP VERSION 11/16/21 7:56 AM
# EXPERIMENTAL SCIFAIR MAIN PROGRAM

# IMPORT MODULES
#UNUSED MODULES
# import cv2
# import math
# import matplotlib.pyplot as plt
# import numpy as np

from cv2 import VideoCapture
from cv2 import QRCodeDetector
import RPi.GPIO as GPIO
from time import time
from time import sleep
import Encoder
from gpiozero import DistanceSensor

GPIO.setmode(GPIO.BOARD)


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
        GPIO.setup(11, GPIO.OUT)

        right_pwm = GPIO.PWM(12, 400)
        right_pwm.start(0)
        left_pwm = GPIO.PWM(13, 400)
        left_pwm.start(0)
        dropper = GPIO.PWM(11, 50)
        dropper.start(0)

        # VARIABLES
        self.stage = 0
        self.delivery = False

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
        global left_enc, right_enc, right_pwm, dropper, left_pwm, bottom_sensor, forward_sensor, cap, detector

    # FUNCTIONS

    @staticmethod
    def get_image():
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
    def duty_cycle(pulse_width):
        return (float(pulse_width) / 1000000.0) * 400 * 100

    def drive_right_motor(self, pwr):
        right_pwm.ChangeDutyCycle(self.duty_cycle(self.make_pulse(-pwr)))

    def drive_left_motor(self, pwr):
        left_pwm.ChangeDutyCycle(self.duty_cycle(self.make_pulse(pwr)))

    @staticmethod
    def left_stop():
        left_pwm.ChangeDutyCycle(0)

    @staticmethod
    def right_stop():
        right_pwm.ChangeDutyCycle(0)

    @staticmethod
    def check():
        return bottom_sensor.distance, forward_sensor.distance

    @staticmethod
    def get_speed(x):
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

    def forward_PID(self, x):
        mark_early = time()

        self.drive_left_motor(self.alphaL)
        self.drive_right_motor(self.alphaR)

        self.left_speed, self.mark_now = self.get_speed("left")
        self.LI += 0.001 * ((x / 400) * 410 - self.left_speed)
        self.alphaL += (0.05 * ((x / 400) * 410 - self.left_speed) + self.LI + (
                0.001 * (self.prev_l_speed - self.left_speed) / (self.mark_now - mark_early))) * 0.00134

        self.right_speed, self.mark_now = self.get_speed("right")
        self.RI += 0.001 * ((x / 400) * 430 - self.right_speed)
        self.alphaR += (0.05 * ((x / 400) * 430 - self.right_speed) + self.RI + (
                0.001 * (self.prev_r_speed - self.right_speed) / (self.mark_now - mark_early))) * 0.00134

        self.prev_l_speed = self.left_speed
        self.prev_r_speed = self.right_speed

    def turn_PID(self, x, y):
        mark_early = time()
        time_before = time()

        if x == "left":
            t_alpha_l = -0.25
            t_alpha_r = 0.25
            t_prev_l_speed = 400
            t_LI = 0

            while abs(left_enc.read()) < 480 * (y / 90) or time() - time_before > 7:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_left_speed, t_mark_now = self.get_speed("left")
                t_LI += 0.001 * (400 - t_left_speed)
                t_alpha_l -= (0.05 * (400 - t_left_speed) + t_LI + (
                        0.001 * (t_prev_l_speed - t_left_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_l_speed = t_left_speed

        elif x == "right":
            t_alpha_l = 0.13
            t_alpha_r = -0.13

            t_prev_r_speed = 400

            t_RI = 0

            while abs(right_enc.read()) < 480 * (y / 90) or time() - time_before > 7:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_right_speed, t_mark_now = self.get_speed("right")
                t_RI += 0.001 * (400 - abs(t_right_speed))
                t_alpha_r += 0.01 * (0.001 * abs(400 - abs(t_right_speed)) + t_RI + (
                        0.001 * (t_prev_r_speed - t_right_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_r_speed = t_right_speed
        return time() - time_before

    @staticmethod
    def dump():
        # ADJUST THESE POSITIONS
        dropper.ChangeDutyCycle(5)
        sleep(3)
        dropper.ChangeDutyCycle(10)

    def cleanup(self):
        self.left_stop()
        self.right_stop()
        left_pwm.stop()
        right_pwm.stop()
        dropper.stop()
        cap.release()
        GPIO.cleanup()


robot = Robot()

runtime_loop = True

# main loop
while runtime_loop:
    robot.forward_PID(400)

    width, loc, info, size = robot.get_image()
    if info != "null":
        if info == "target" and robot.stage != 1:
            robot.stage = 1
        if info == "left" or info == "right":
            if width > 200:
                robot.turn_PID(info, 90)

    # center on target
    if robot.stage == 1 and width != -1:
        if loc < (width / 2) - (width / 20):
            robot.drive_right_motor(0.15)
            robot.drive_right_motor(-0.15)
            sleep(0.03)
        elif loc > (width / 2) + (width / 20):
            robot.drive_right_motor(-0.15)
            robot.drive_right_motor(0.15)
            sleep(0.03)
        else:
            robot.stage = 2
        robot.left_stop()
        robot.right_stop()

    if robot.stage == 2 and width != -1:
        if width < 500:
            robot.forward_PID(100)
            sleep(0.1)
            robot.left_stop()
            robot.right_stop()
        else:
            robot.stage = 3

    if robot.stage == 3:
        _, forward = robot.check()
        while forward > 50:
            robot.forward_PID(50)
            _, forward = robot.check()
        if not robot.delivery:
            robot.stage = 4
            # COMMENT OUT TO CONTINUE LOOPING
            runtime_loop = False
        else:
            robot.stage = 5

    # REMOVE "and False" WHEN FIRST HALF OF PROGRAM WORKS
    if robot.stage == 4 and False:
        sleep(20)
        robot.drive_left_motor(-0.23)
        robot.drive_right_motor(-0.15)
        sleep(0.5)
        robot.left_stop()
        robot.right_stop()
        robot.turn_PID("right", 90)
        robot.stage = 0
        robot.delivery = True

    if robot.stage == 5:
        # DUMP CARGO
        # ADJUST DUMP SETTINGS!!!
        robot.dump()
        runtime_loop = False

    # CHECK TO NOT GO OVER CLIFF/RUN INTO THINGS
    down, forward = robot.check()
    if down > 90 or forward < 90:
        robot.left_stop()
        robot.right_stop()
        runtime_loop = False
        print("EXITED DUE TO ERROR: OBJECT DETECTED")

robot.cleanup()
