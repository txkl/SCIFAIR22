# TIERNAN LINDAUER
# LAST EDIT 11/21/21
# LAST BACKED UP VERSION 11/21/21 17:13
# EXPERIMENTAL SCIFAIR MAIN PROGRAM

# IMPORT MODULES

from cv2 import VideoCapture
from cv2 import QRCodeDetector
import RPi.GPIO as GPIO
from time import time
from time import sleep
import Encoder
from gpiozero import DistanceSensor
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)

# ROBOT CLASS
class Robot:
    #ROBOT MAPPING
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
        self.delivery = False

    # FUNCTIONS

    def get_image(self):
        _, img = self.cap.read()
        data, bbox, _ = self.detector.detectAndDecode(img)

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

    def get_speed(self, x):
        #L/R ENCODER
        if x == "left":
            self.left_enc.write()
            bt = time()
            while abs(self.left_enc.read()) < 10:
                sleep(.0001)
            mn = time()
            ls = 10 / (mn - bt)

            return ls, mn
        elif x == "right":
            self.right_enc.write()
            bt = time()
            while abs(self.right_enc.read()) < 10:
                sleep(.0001)
            mn = time()
            rs = 10 / (mn - bt)
        
            return rs, mn

    def forward_PID(self, x):
        #SPEED IN TPS
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
        #DIRECTION, MAGNITUDE(DEGREES)
        mark_early = time()
        time_before = time()
        y = 48*(y/90)
        times_run = 0
        
        if x == "left":
            t_alpha_l = -0.25
            t_alpha_r = 0.25
            t_prev_l_speed = 400
            t_LI = 0

            while abs(times_run <= y or time() - time_before > 7):
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_left_speed, t_mark_now = self.get_speed("left")
                t_LI += 0.001 * (400 - t_left_speed)
                t_alpha_l -= (0.05 * (400 - t_left_speed) + t_LI + (
                        0.001 * (t_prev_l_speed - t_left_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_l_speed = t_left_speed
                times_run += 1

        elif x == "right":
            t_alpha_l = 0.13
            t_alpha_r = -0.13

            t_prev_r_speed = 400
            t_RI = 0

            while abs(times_run <= y or time() - time_before > 7):
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_right_speed, t_mark_now = self.get_speed("right")
                t_RI += 0.001 * (400 - abs(t_right_speed))
                t_alpha_r += 0.01 * (0.001 * abs(400 - abs(t_right_speed)) + t_RI + (
                        0.001 * (t_prev_r_speed - t_right_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_r_speed = t_right_speed
                times_run += 1
        self.left_stop()
        self.right_stop()
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

# main loop
while runtime_loop:
    #robot.forward_PID(400)
    print("Loop running, stage",robot.stage)
    width, loc, info, size = robot.get_image()
    print(info)
    if info != "null":
        print("QR detected", end=" ")
        if info == "target" and robot.stage != 1:
            print("Target detected")
            robot.stage = 1
        if info == "left" or info == "right":
            if width > 100:
                robot.turn_PID(info, 90)
                sleep(1)
                #even out the caster wheel
                start_time = time()
                while time()-start_time < 1:
                    robot.forward_PID(100)
                    robot.left_stop()
                    robot.right_stop()

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
            runtime_loop = False # COMMENT OUT TO CONTINUE LOOPING
        else:
            robot.stage = 5

    if robot.stage == 4 and False: # REMOVE "and False" WHEN FIRST HALF OF PROGRAM WORKS
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
        robot.dump()
        runtime_loop = False

    # CHECK TO NOT GO OVER CLIFF/RUN INTO THINGS
    down, forward = robot.check()
    if down > 11 or forward < 10:
        robot.left_stop()
        robot.right_stop()
        runtime_loop = False
        print(down," ",forward)
        print("EXITED DUE TO OBJECT/CLIFF DETECTED")

robot.cleanup()
