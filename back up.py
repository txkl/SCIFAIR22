# IMPORT MODULES
from cv2 import VideoCapture
from cv2 import QRCodeDetector
import RPi.GPIO as GPIO
from time import time
from time import sleep
from math import atan
import Encoder
from gpiozero import DistanceSensor
GPIO.setwarnings(False)

# ROBOT CLASS
class Robot:
    #ROBOT INPUT/OUTPUT MAPPING
        #23, 24: LEFT ENCODER
        #20, 21: RIGHT ENCODER
        #7, 8: FORWARD DISTANCE SENSOR
        #15, 14: BOTTOM DISTANCE SENSOR
        #12: RIGHT MOTOR PWM
        #13: LEFT MOTOR PWM
        #11: DUMP SERVO PWM
    def __init__(self):
        # OBJECT DEFINITIONS AND SETUP
        self.left_enc = Encoder.Encoder(23, 24)
        self.right_enc = Encoder.Encoder(20, 21)

        self.forward_sensor = DistanceSensor(echo=7, trigger=8)
        self.bottom_sensor = DistanceSensor(echo=15, trigger=14)

        self.cap = VideoCapture(0)
        self.detector = QRCodeDetector()

        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(11, GPIO.OUT)

        self.right_pwm = GPIO.PWM(12, 400)
        self.right_pwm.start(0)
        self.left_pwm = GPIO.PWM(13, 400)
        self.left_pwm.start(0)
        self.dropper = GPIO.PWM(11, 50)
        self.dropper.start(0)

        # VARIABLES
        self.stage = 4
        self.alphaL = 0.11
        self.alphaR = 0.25
        self.prev_l_speed = 400
        self.prev_r_speed = 400
        self.left_speed = 400
        self.right_speed = 400
        self.LI = 0
        self.RI = 0
        self.mark_now = 0
        self.delivery = False
        self.flag_1 = False
        self.cl = 0
        
    # FUNCTIONS

    def get_image(self):
        _, img = self.cap.read()
        data, bbox, _ = self.detector.detectAndDecode(img)

        if (bbox is not None):
            box_width = abs(int(bbox[0][1][0]) - int(bbox[0][0][0]))
            x_loc = (int(bbox[0][0][0]) + int(bbox[0][1][0])) / 2

            if data == "left" or data == "right" or data == "target":
                return box_width, x_loc, data, bbox #width is always 1920 pixels
            return box_width, x_loc, "null", bbox
        return -1, -1, "null", -1

    @staticmethod
    def make_pulse(power):
        #POWER
        if power > 0:
            abs(power)
            return 990 * power + 1490
        elif power < 0:
            return 990 * power + 1510
        else:
            return 1500

    @staticmethod
    def duty_cycle(pulse_width):
        #PULSE WIDTH
        return (float(pulse_width) / 1000000.0) * 400 * 100

    def drive_right_motor(self, pwr):
        #POWER
        self.right_pwm.ChangeDutyCycle(self.duty_cycle(self.make_pulse(-pwr)))

    def drive_left_motor(self, pwr):
        #POWER
        self.left_pwm.ChangeDutyCycle(self.duty_cycle(self.make_pulse(pwr)))

    def left_stop(self):
        self.left_pwm.ChangeDutyCycle(0)

    def right_stop(self):
        self.right_pwm.ChangeDutyCycle(0)

    def check(self):
        return self.bottom_sensor.distance*100, self.forward_sensor.distance*100

    def get_speed(self, MOTOR):
        #L/R ENCODER
        if MOTOR == "left":
            offset = self.left_enc.read()
            bt = time()
            while abs(self.left_enc.read()-offset) < 10:
                sleep(.0001)
            mn = time()
            ls = 10 / (mn - bt)

            return ls, mn
        elif MOTOR == "right":
            offset = self.right_enc.read()
            bt = time()
            while abs(self.right_enc.read()-offset) < 10:
                sleep(.0001)
            mn = time()
            rs = 10 / (mn - bt)
        
            return rs, mn

    def forward_PID(self, SPEED):
        #SPEED IN TPS
        x = SPEED
        mark_early = time()

        self.drive_left_motor(self.alphaL)
        self.drive_right_motor(self.alphaR)

        self.left_speed, self.mark_now = self.get_speed("left")
        self.LI += 0.001 * ((x / 400) * 400 - self.left_speed)
        self.alphaL += (0.05 * ((x / 400) * 400 - self.left_speed) + self.LI + (
                0.001 * (self.prev_l_speed - self.left_speed) / (self.mark_now - mark_early))) * 0.00134
        #self.alphaL+=0.0025
        
        self.right_speed, self.mark_now = self.get_speed("right")
        self.RI += 0.001 * ((x / 400) * 430 - self.right_speed)
        self.alphaR += (0.05 * ((x / 400) * 430 - self.right_speed) + self.RI + (
                0.001 * (self.prev_r_speed - self.right_speed) / (self.mark_now - mark_early))) * 0.00134
        self.alphaR += 0.001

        # record speed
        self.prev_l_speed = self.left_speed
        self.prev_r_speed = self.right_speed

    def turn_PID(self, DIRECTION, MAGNITUDE):
        #DIRECTION, MAGNITUDE(DEGREES)
        x = DIRECTION
        y = MAGNITUDE
        
        mark_early = time()
        time_before = time()
        y = 560*(y/90)
        
        if x == "left":
            t_alpha_l = -0.25
            t_alpha_r = 0.25
            t_prev_l_speed = 400
            t_LI = 0
            offset = self.left_enc.read()
            while abs(self.left_enc.read()-offset) < 50:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_left_speed, t_mark_now = self.get_speed("left")
                t_LI += 0.001 * (400 - t_left_speed)
                t_alpha_l -= (0.05 * (400 - t_left_speed) + t_LI + (
                        0.001 * (t_prev_l_speed - t_left_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_l_speed = t_left_speed
            self.left_stop()
            self.right_stop()
            sleep(1)
            while abs(self.left_enc.read()-offset) < y-50 or time() - time_before > 7:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_left_speed, t_mark_now = self.get_speed("left")
                t_LI += 0.001 * (400 - t_left_speed)
                t_alpha_l -= (0.05 * (400 - t_left_speed) + t_LI + (
                        0.001 * (t_prev_l_speed - t_left_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_l_speed = t_left_speed

        elif x == "right":
            y-=40
            t_alpha_l = 0.13
            t_alpha_r = -0.13

            t_prev_r_speed = 400
            t_RI = 0
            offset = self.right_enc.read()
            while abs(self.left_enc.read()-offset) < 50:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_right_speed, t_mark_now = self.get_speed("right")
                t_RI += 0.001 * (400 - abs(t_right_speed))
                t_alpha_r += 0.01 * (0.001 * abs(400 - abs(t_right_speed)) + t_RI + (
                        0.001 * (t_prev_r_speed - t_right_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_r_speed = t_right_speed
            self.left_stop()
            self.right_stop()
            sleep(1)
            while abs(self.right_enc.read()-offset) < y-50 or time() - time_before > 7:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_right_speed, t_mark_now = self.get_speed("right")
                t_RI += 0.001 * (400 - abs(t_right_speed))
                t_alpha_r += 0.01 * (0.001 * abs(400 - abs(t_right_speed)) + t_RI + (
                        0.001 * (t_prev_r_speed - t_right_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_r_speed = t_right_speed
        self.left_stop()
        self.right_stop()
        self.left_enc.write()
        self.right_enc.write()
        return time() - time_before
    
    def dump(self):
        self.dropper.ChangeDutyCycle(12)
        sleep(2)
        self.dropper.ChangeDutyCycle(4)

    def cleanup(self):
        self.left_stop()
        self.right_stop()
        self.left_pwm.stop()
        self.right_pwm.stop()
        self.dropper.stop()
        self.cap.release()
        GPIO.cleanup()

robot = Robot()
runtime_loop = True

sleep(1)
# main loop
while runtime_loop:
    width, loc, info, bbox = robot.get_image()
    if info != "null":
        print("QR detected at",loc, end=", ")

    if robot.stage == 4: #and False: # REMOVE "and False" WHEN FIRST HALF OF PROGRAM WORKS
        print("Loop running")
        sleep(2)
        robot.drive_left_motor(-0.23)#add vision after reversed to straighten out
        robot.drive_right_motor(-0.15)
        sleep(0.75)
        robot.right_stop()
        sleep(0.3)
        robot.left_stop()
        robot.stage = 5
    if robot.stage == 5:
        print("stage 5")
        if width != -1:
            prop = (bbox[0][2][0]-bbox[0][0][0])/(bbox[0][3][1]-bbox[0][0][1])
            print(prop)
            if prop > 1:
                prop = 1-(prop-1)
            if prop < 0.88:
                robot.drive_left_motor(0.02)
                robot.drive_right_motor(-0.02)
                sleep(0.2)
                robot.left_stop()
                robot.right_stop()
            else:
                robot.stage = 6
        else:
            robot.drive_left_motor(0.1)
            robot.drive_right_motor(-0.1)
            sleep(0.2)
            robot.left_stop()
            robot.right_stop()
            robot.cl += 1;
        if robot.cl > 5:
            #doesn't detect QR
            robot.stage = 6
                
robot.cleanup()
