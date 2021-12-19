# TIERNAN LINDAUER
# LAST EDIT 12/19/21
# LAST BACKED UP VERSION 12/19/21 0804
# OFFICIAL SCIFAIR MAIN PROGRAM


# IMPORT MODULES
from cv2 import VideoCapture
from cv2 import QRCodeDetector
import RPi.GPIO as GPIO
from time import time
from time import sleep
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
    def __init__(self, stage, src):
        # OBJECT DEFINITIONS AND SETUP
        self.left_enc = Encoder.Encoder(23, 24)
        self.right_enc = Encoder.Encoder(20, 21)

        self.forward_sensor = DistanceSensor(echo=7, trigger=8)
        self.bottom_sensor = DistanceSensor(echo=15, trigger=14)

        self.cap = VideoCapture(src)
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
        self.stage = stage
        self.alphaL = 0.25
        self.alphaR = 0.25
        self.prev_l_speed = 400
        self.prev_r_speed = 400
        self.left_speed = 400
        self.right_speed = 400
        self.LI = 0
        self.RI = 0
        self.mark_now = 0
        self.action_time = 0
        self.cl = time()
        self.delivery = False

        self.get_image()
    # FUNCTIONS

    def get_image(self):
        _, img = self.cap.read()
        try:
            data, bbox, _ = self.detector.detectAndDecode(img)
        except(Exception):
            return -1, -1, -1, -1
        if (bbox is not None):
            box_width = abs(int(bbox[0][1][0]) - int(bbox[0][0][0]))
            x_loc = (int(bbox[0][0][0]) + int(bbox[0][1][0])) / 2

            if data == "left" or data == "right" or data == "target":
                print(data)
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
    def stop(self):
        self.left_stop()
        self.right_stop()
    def check(self):
        try:
            return self.bottom_sensor.distance*100, self.forward_sensor.distance*100
        except(Exception):
            return -1, -1
        
    def get_speed(self, MOTOR):
        #L/R ENCODER
        if MOTOR == "left":
            offset = self.left_enc.read()
            bt = time()
            while abs(self.left_enc.read()-offset) < 10:
                sleep(.0001)
            mn = time()
            ls = 10 / (mn - bt)
            print("LS: ",ls)
            return ls, mn
        elif MOTOR == "right":
            offset = self.right_enc.read()
            bt = time()
            while abs(self.right_enc.read()-offset) < 10:
                sleep(.0001)
            mn = time()
            rs = 10 / (mn - bt)
            print("RS: ",rs)
            return rs, mn

    def forward_PID(self, SPEED, reset):
        #SPEED IN TPS
        x = SPEED
        mark_early = time()

        if reset:
            self.prev_l_speed = 400
            self.prev_r_speed = 400

        self.drive_left_motor(self.alphaL)
        self.drive_right_motor(self.alphaR)

        self.left_speed, self.mark_now = self.get_speed("left")
        self.LI += 0.0005 * ((x / 400) * 400 - self.left_speed)
        self.alphaL += (0.05 * ((x / 400) * 400 - self.left_speed) + self.LI + (
                0.001 * (self.prev_l_speed - self.left_speed) / (self.mark_now - mark_early))) * 0.00134
        self.alphaL += 0.003
        
        self.right_speed, self.mark_now = self.get_speed("right")
        self.RI += 0.001 * ((x / 400) * 430 - self.right_speed)
        self.alphaR += (0.05 * ((x / 400) * 430 - self.right_speed) + self.RI + (
                0.001 * (self.prev_r_speed - self.right_speed) / (self.mark_now - mark_early))) * 0.00134
        # record speed
        self.prev_l_speed = self.left_speed
        self.prev_r_speed = self.right_speed

    def turn_PID(self, DIRECTION, MAGNITUDE):
        #DIRECTION, MAGNITUDE(DEGREES)
        sleep(3)
        
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
            self.stop()
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
            self.stop()
            print("sleeping")
            sleep(1)
            while abs(self.right_enc.read()-offset) < y-50 or time() - time_before > 7:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_right_speed, t_mark_now = self.get_speed("right")
                t_RI += 0.001 * (400 - abs(t_right_speed))
                t_alpha_r += 0.01 * (0.001 * abs(400 - abs(t_right_speed)) + t_RI + (
                        0.001 * (t_prev_r_speed - t_right_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_r_speed = t_right_speed
        self.stop()
        
        #needed otherwise PID will be messed up
        self.left_enc.write()
        self.right_enc.write()        
        sleep(2)
        
        #necessary to not mess up turning PID
        self.left_enc.write()
        self.right_enc.write()
        return time() - time_before
    
    def dump(self):
        self.dropper.ChangeDutyCycle(12)
        sleep(2)
        self.dropper.ChangeDutyCycle(4)

    def cleanup(self):
        self.stop()
        self.left_pwm.stop()
        self.right_pwm.stop()
        self.dropper.stop()
        self.cap.release()
        GPIO.cleanup()
        del self

robot = Robot(1,0) #stage and camera source
runtime_loop = True
sleep(0)
# main runtime loop
robot.action_time = time()
while runtime_loop:
    
    #TIME-OUTS
    if time()-robot.action_time > 6 and robot.stage == 1:
        robot.stage = 0
        robot.action_time = time()
    if time()-robot.action_time > 10 and robot.stage == 5:
        robot.stage = 6
        robot.action_time = time()

    # CHECK TO NOT GO OVER CLIFF/RUN INTO THINGS
    down, forward = robot.check()
    if forward == -1:
        print("NO SENSOR DETECTED!")
    if (down > 11 or forward < 10) and robot.stage == 0:
        robot.stop()
        runtime_loop = False
        print("Distances: ",down," ",forward)
        print("EXITED DUE TO AN OBJECT/CLIFF WHICH WAS DETECTED")    
        
        
    print("Loop running, stage",robot.stage)
    width, loc, info, bbox = robot.get_image()
    if info != "null":
        print(loc)
                
        print("QR detected at",loc, end=", ")
        # target mode
        if info == "target" and robot.stage == 0 and width > 130 and time()-robot.action_time > 20:
            print("target detected")
            robot.stage = 1
            robot.action_time = time()
            
        # turn left or right
        if (info == "left" or info == "right"):
                print("turning ",info)
                if width > 125 and time() - robot.action_time > 7:
                    robot.stop()
                    robot.turn_PID(info, 90)
                    start_time = time()
                    while time()-start_time < 2:
                        robot.forward_PID(400, True)
                    robot.stop()
                    sleep(1)
                    robot.action_time = time()
    

    #ROBOT ALGORITHM BELOW
    #-------------------------
    #NOTE: STAGES START AT 0
    #HOWEVER TO AVOID CONFLICT
    #THEY ARE ANALYZED IN
    #DESCENDING ORDER

    if robot.stage == 7:
        # dump cargo and end program
        robot.dump()
        runtime_loop = False

    if robot.stage == 6:
        # turn right and start delivery segment
        robot.turn_PID("right",90)
        robot.stage = 0
        robot.delivery = True

    if robot.stage == 5:
        #align to be straight to the QR for accurate turning
        print("stage 5")
        if width != -1:
            height_difference = (bbox[0][0][1]-bbox[0][1][1])
            if(height_difference < 0):
                    add = ((bbox[0][1][0]+bbox[0][0][0])/2-345)*0.1
                    height_difference += add
            elif(height_difference < 0):
                    add = (345-(bbox[0][1][0]+bbox[0][0][0])/2)*0.1
                    height_difference += add
            print("height difference: ",height_difference)
            robot.action_time = time()
            if height_difference > 10:
                #turn right
                robot.drive_right_motor(-0.098)
                robot.drive_left_motor(0.098)
                sleep(0.1)
                robot.stop()
            elif height_difference < -20:
                #turn left
                robot.drive_right_motor(0.15)
                robot.drive_left_motor(-0.15)
                sleep(0.1)
                robot.stop()
            elif height_difference != 0:
                robot.stage = 6

    if robot.stage == 4:
        #reverse after being loaded
        print("Loop running")
        sleep(5)# wait to be loaded
        robot.drive_left_motor(-0.3)#add vision after reversed to straighten out
        robot.drive_right_motor(-0.15)
        sleep(0.75)
        robot.right_stop()
        sleep(0.3)
        robot.left_stop()
        robot.stage = 5
                
    if robot.stage == 3:
        #use ultrasonic distance sensors in a final approach - might completely skip stage 2
        print("Loop running, stage 3")
        _, forward = robot.check()
        while forward > 10:
            robot.forward_PID(100, False)
            _, forward = robot.check()
        robot.stop()
        if not robot.delivery:
            robot.stage = 4
            #runtime_loop = False # COMMENT OUT TO CONTINUE LOOPING
        else:
            robot.stage = 7
            
    if robot.stage == 2 and width != -1:
        # controlled bursts forwards to target while adjusting angle
        if width < 200:
            # reorient
            if loc < 300:
                robot.drive_right_motor(0.1)
                robot.drive_left_motor(-0.1)
                sleep(0.1)
            elif loc > 390:
                robot.drive_right_motor(-0.1)
                robot.drive_left_motor(0.1)
                sleep(0.1)
            robot.stop()

            # forward, unable to do PID
            robot.drive_right_motor(0.23)
            robot.drive_left_motor(0.15)            
            sleep(0.3)
            robot.stop()             
        else:
            robot.stage = 3
            
    # center on target
    if robot.stage == 1 and width != -1:
        robot.action_time = time()
        if loc < 330:
            robot.drive_right_motor(0.15)
            robot.drive_left_motor(-0.15)
            sleep(0.05)
        elif loc > 360:
            robot.drive_right_motor(-0.098)
            robot.drive_left_motor(0.098)
            sleep(0.05)
        else:
            robot.stage = 2
        robot.stop()

    #default state: move forward
    if robot.stage == 0:
        robot.forward_PID(400, False)

# take care of the robot
robot.cleanup()





# Thank you to all those who came
# before me and made this project
# possible.
#
#     -Tiernan Lindauer
