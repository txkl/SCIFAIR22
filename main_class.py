# TIERNAN LINDAUER
# LAST EDIT 1/6/22
# LAST BACKED UP VERSION 1/6/22 14:39
# OFFICIAL SCIFAIR MAIN PROGRAM


# IMPORT FUNCTIONS FROM MODULES
from cv2 import VideoCapture
from cv2 import QRCodeDetector
import RPi.GPIO as GPIO
from time import time
from time import sleep
import Encoder
import math
from gpiozero import DistanceSensor
from scipy.misc import derivative

# set warnings to not show up
GPIO.setwarnings(False)

# code to record first person video - typically leave commented out
from cv2 import VideoWriter
from cv2 import VideoWriter_fourcc

# ROBOT CLASS
class Robot:
    # ROBOT GENERAL PURPOSE INPUT/OUTPUT MAPPING
        # 23, 24: LEFT ENCODER
        # 20, 21: RIGHT ENCODER
        # 7, 8: FORWARD DISTANCE SENSOR
        # 15, 14: BOTTOM DISTANCE SENSOR
        # 12: RIGHT MOTOR PWM
        # 13: LEFT MOTOR PWM
        # 11: DUMP SERVO PWM
  
    # initialization method
    def __init__(self, stage):
        # object definitions and setup
        self.left_enc = Encoder.Encoder(23, 24)
        self.right_enc = Encoder.Encoder(20, 21)

        self.forward_sensor = DistanceSensor(echo=7, trigger=8)
        self.bottom_sensor = DistanceSensor(echo=15, trigger=14)

        # auto camera source finder
        self.src = 0
        self.cap = VideoCapture(self.src)
        while not self.cap.isOpened():
            self.src += 1
            self.cap = VideoCapture(self.src)
            
        self.detector = QRCodeDetector()

        # code to record video - typically leave commented out
        frame_width = int(self.cap.get(3))
        frame_height = int(self.cap.get(4))

        size = (frame_width, frame_height)

        self.result = VideoWriter('RobotPerspective1.avi', 
                                 VideoWriter_fourcc(*'MJPG'),
                                 10, size)
        # video recording code ends here
        
        # set up GPIO objects
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
        self.alphaL = 0.12
        self.alphaR = 0.25
        self.prev_l_speed = 400
        self.prev_r_speed = 400
        self.prev_time = 0
        self.left_speed = 400
        self.right_speed = 400
        self.LI = 0
        self.RI = 0
        self.a = 0
        self.b = 0
        self.mark_now = 0
        self.action_time = 0
        self.to_turn = "none"
        self.delivery = False

        self.get_image()
        
    # FUNCTIONS
    def get_image(self):
        _, img = self.cap.read()
        #comment the line below out if not recording robot FOV
        self.result.write(img)
        try:
            data, bbox, _ = self.detector.detectAndDecode(img)
        except(Exception):
            return -1, -1, -1, -1
        if (bbox is not None):
            box_width = abs(int(bbox[0][1][0]) - int(bbox[0][0][0]))
            x_loc = (int(bbox[0][0][0]) + int(bbox[0][1][0])) / 2

            if data == "left" or data == "right" or data == "target":
                print(box_width)
                return box_width, x_loc, data, bbox #width is always 1920 pixels
            return box_width, x_loc, "null", bbox
        return -1, -1, "null", -1
    
    # change the power value from 0-1 to a pulse width in microseconds
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

    # derivative function
    @staticmethod
    def f(x):
        return self.a*self.b**x
    
    # calculate derivative - takes the two most recent time and velocity measurements
    # and create an exponential function through them, then calculate the derivative at the latest instant.
    @staticmethod
    def deriv(x1, x2, y1, y2):
        self.b = (y2/y1)**(1/(x2-x1))
        self.a = y1/(self.b**x1)
        return derivative(f, 1.0, dx=x2)
    
    # change pulse width to duty cycle for pulse width modulation
    @staticmethod
    def duty_cycle(pulse_width):
        #PULSE WIDTH
        return (float(pulse_width) / 1000000.0) * 400 * 100
    
    # power the right motor with a certain power from 0-1
    def drive_right_motor(self, pwr):
        #POWER
        self.right_pwm.ChangeDutyCycle(self.duty_cycle(self.make_pulse(-pwr)))
    
    # power the left motor with a certain power from 0-1
    def drive_left_motor(self, pwr):
        #POWER
        self.left_pwm.ChangeDutyCycle(self.duty_cycle(self.make_pulse(pwr)))
    
    # brake the left motor
    def left_stop(self):
        self.left_pwm.ChangeDutyCycle(0)
    
    # brake the right motor
    def right_stop(self):
        self.right_pwm.ChangeDutyCycle(0)
    
    # brake both motors
    def stop(self):
        self.left_stop()
        self.right_stop()
        
    # return distance sensor values
    def check(self):
        try:
            return self.bottom_sensor.distance*100, self.forward_sensor.distance*100
        except(Exception):
            return -1, -1
    
    # get the current speed of a certain encoder by calculating the time it takes to rotate 10 ticks
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
     
    # arrange the robot so it is perpendicular to the planar surface it faces
    # - used when turning.
    def center(self):
        self.drive_right_motor(-0.098)
        self.drive_left_motor(0.098)
        sleep(1)
        self.stop()
        sleep(1)
        turn_loop = True
        last_dist = 100
        while turn_loop:
            _, dist = self.check()
            print(dist)
            #while abs(dist-last_dist) > 30 and dist != 100: #why was this even a thing
            #    _, dist = self.check() #the thought process here... rip
            if int(last_dist) >= int(dist):
                self.drive_right_motor(0.15)
                self.drive_left_motor(-0.15)
                sleep(0.1)
                self.stop()
            else:
                if dist < 60:
                    turn_loop = False
            last_dist = dist
        self.stop()
        sleep(0.25)
        self.drive_right_motor(-0.098)
        self.drive_left_motor(0.098)
        sleep(0.1)
        self.stop()
    
    # update the PID control loop based on the latest speed information
    def forward_PID(self, SPEED, reset):
        #SPEED IN TPS
        x = SPEED
        mark_early = time()
        
        # useful for resetting behavior labeled as "bad"
        if reset:
            self.prev_l_speed = 400
            self.prev_r_speed = 400

        #add the current alpha values (the motor power values)
        self.drive_left_motor(self.alphaL)
        self.drive_right_motor(self.alphaR)
        
        # get the current speed for the left motor
        self.left_speed, self.mark_now = self.get_speed("left")
        
        # Integral/Summation term
        self.LI += 0.001 * ((x / 400) * 400 - self.left_speed)
        
        # Proportional term
        self.alphaL += (0.05 * ((x / 400) * 400 - self.left_speed) + self.LI + (
                0.001 * (self.prev_l_speed - self.left_speed) / (self.mark_now - mark_early))) * 0.00134
        
        
        # calculate the derivative
        left_deriv = deriv(self.prev_time, mark_early, self.prev_l_speed, self.left_speed)
        
        # Derivative term
        self.alphaL += (left_deriv)*0.001
        
        # get the current speed for the right  motor
        self.right_speed, self.mark_now = self.get_speed("right")
        
        # Integral/Summation term
        self.RI += 0.001 * ((x / 400) * 400 - self.right_speed)
        
        # Proportional term
        self.alphaR += (0.05 * ((x / 400) * 400 - self.right_speed) + self.RI + (
                0.001 * (self.prev_r_speed - self.right_speed) / (self.mark_now - mark_early))) * 0.00134
        # calculate the derivative
        right_deriv = deriv(self.prev_time, mark_early, self.prev_r_speed, self.right_speed)
        
        # Derivative term
        self.alphaR += (right_deriv)*0.001
        
        # record speed for future use
        self.prev_time = time()
        self.prev_l_speed = self.left_speed
        self.prev_r_speed = self.right_speed
    
    # specially tuned PID loop to turn accurately - much the same concept as above
    def turn_PID(self, DIRECTION, MAGNITUDE):
        #DIRECTION, MAGNITUDE(DEGREES)
        robot.stop()
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
            sleep(0.5)
            while abs(self.left_enc.read()-offset) < 100:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_right_speed, t_mark_now = self.get_speed("right")
                t_RI += 0.001 * (400 - abs(t_right_speed))
                t_alpha_r += 0.01 * (0.001 * abs(400 - abs(t_right_speed)) + t_RI + (
                        0.001 * (t_prev_r_speed - t_right_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_r_speed = t_right_speed
            self.stop()
            print("sleeping")
            sleep(1.5)
            offset -= 50
            while abs(self.right_enc.read()-offset) < y-50 or time() - time_before > 7:
                self.drive_left_motor(t_alpha_l)
                self.drive_right_motor(t_alpha_r)

                t_right_speed, t_mark_now = self.get_speed("right")
                t_RI += 0.001 * (400 - abs(t_right_speed))
                t_alpha_r += 0.01 * (0.001 * abs(400 - abs(t_right_speed)) + t_RI + (
                        0.001 * (t_prev_r_speed - t_right_speed) / (t_mark_now - mark_early))) * 0.00134

                t_prev_r_speed = t_right_speed
        else:
            print("invalid turn id of ",x," requested!")
        self.stop()
        
        # needed otherwise PID will be messed up
        self.left_enc.write()
        self.right_enc.write()        
        sleep(2)
        
        # necessary to not mess up forward PID - encoders will be at different positions
        self.left_enc.write()
        self.right_enc.write()
        return time() - time_before
   
    # dump the cargo
    def dump(self):
        self.dropper.ChangeDutyCycle(12)
        sleep(2)
        self.dropper.ChangeDutyCycle(4)
    
    # exit the program safely, and delete the object
    def cleanup(self):
        self.stop()
        self.left_pwm.stop()
        self.right_pwm.stop()
        self.dropper.stop()
        self.cap.release()
        GPIO.cleanup()
        del self

# create robot object
robot = Robot(0) #preset stage

# reset encoders using a custom method in the Encoder library
robot.left_enc.write()
robot.right_enc.write()

# boolean for the loop being on/off
runtime_loop = True
robot.delivery = True #get rid of this eventually
# wait until starting signal- waving in front of the distance sensor
_, fwd = robot.check()
while fwd > 7:
    sleep(0.01)
    _, fwd = robot.check()
print("starting...")
sleep(4)

# main runtime loop
robot.action_time = time()
while runtime_loop:
    robot.left_enc.write()
    robot.right_enc.write()
    
    # see if camera is opened, if not, auto-select
    if not robot.cap.isOpened():
        robot.src = 0
    while (not robot.cap.isOpened()) and runtime_loop:
            robot.src += 1
            robot.cap = VideoCapture(robot.src)
            if robot.src > 10:
                runtime_loop = False
    
    # time-outs
    if time()-robot.action_time > 6 and robot.stage == 1:
        robot.stage = 0
        robot.action_time = time()

    # check to not run off of a cliff or hit an obstacle
    down, forward = robot.check()
    if forward == -1:
        print("NO SENSOR DETECTED!")
    if (down > 11 or forward < 10) and robot.stage == 0:
        robot.cleanup() # exit safely to prevent corruption
        runtime_loop = False
        print("Distances: ",down," ",forward)
        print("EXITED DUE TO AN OBJECT/CLIFF WHICH WAS DETECTED")    
        
    # get frame and analyze
    print("Loop running, stage",robot.stage)
    width, loc, info, bbox = robot.get_image() # width is the width of the box, loc is the center of the box, info is the data in the QR, bbox is the set of points of the QR
    
    # if a QR is detected
    if info != "null":               
        print("QR detected at",loc, end=", ")
        
        # target mode
        if info == "target" and robot.stage == 0 and width > 100:# and time()-robot.action_time > 10: #last qualification is important when NOT used in testing
            print("target detected")
            robot.stage = 1
            robot.action_time = time()
            
        # turn left or right
        if (info == "left" or info == "right") and width > 100:#this number might need tweaking
            if time()-robot.action_time > 5: # action time might cause some issues depending on
                robot.stage = 0.5            # distance to the next closest action time
                robot.to_turn = info
                robot.action_time = time()
            
            # change to use old turning method (here for documentation)
            if False:
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
    

    # ROBOT ALGORITHM BELOW
    # -------------------------
    # NOTE: STAGES START AT 0
    # HOWEVER TO AVOID A
    # LACK OF INFORMATION
    # THEY ARE ANALYZED IN
    # DESCENDING ORDER

    if robot.stage == 7:
        # dump cargo and end program
        robot.dump()
        runtime_loop = False

    if robot.stage == 6:
        # turn right and start delivery segment
        sleep(1)
        robot.turn_PID("right",90)
        robot.stage = 0
        robot.delivery = True

    if robot.stage == 5:
        # align to be straight to the QR for accurate turning
        robot.center()
        robot.stage = 6

    if robot.stage == 4:
        # reverse after being loaded
        print("Loop running")
        sleep(15)# wait to be loaded
        robot.drive_left_motor(-0.3)
        robot.drive_right_motor(-0.15)
        sleep(0.25)
        robot.right_stop()
        sleep(0.3)
        robot.left_stop()
        robot.stage = 5
                
    if robot.stage == 3:
        # use ultrasonic distance sensors in a final approach - might completely skip stage 2
        print("Loop running, stage 3")
        _, forward = robot.check()
        while forward > 10:
            robot.forward_PID(100, False)
            _, forward = robot.check()
        robot.stop()
        if not robot.delivery:
            robot.stage = 4
        else:
            robot.stage = 7
            
    if robot.stage == 2 and width != -1:
        # controlled bursts forwards to target while adjusting angle
        if width < 200:
            # reorient to the QR
            if loc < 300:
                robot.drive_right_motor(0.1)
                robot.drive_left_motor(-0.1)
                sleep(0.1)
            elif loc > 390:
                robot.drive_right_motor(-0.1)
                robot.drive_left_motor(0.1)
                sleep(0.1)
            robot.stop()

            tbf = time()    
            while time() - tbf < 0.3:
                robot.forward_PID(200, False)
            robot.stop()             
        else:
            robot.stage = 3
            
    # center on target QR
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
    
    if robot.stage == 0.5:
        #stage for centering and turning, not used in old version of the code
        robot.stop()
        sleep(2)
        
        # get distance information
        _, forward = robot.check()
        
        prev_cam_width = 1000
        while(forward > 30):
            # gets QR info here - unfortunately needed for the loop (too slow if the entire loop is used)
            width, loc, info, bbox = robot.get_image()
            
            # still too close to QR
            if width > 350:
                robot.drive_left_motor(-0.30)
                robot.drive_right_motor(-0.15)

            # sensor fusion - both USDS and camera
            if width > last_width:
                forwad = 0
            last_width = width
            
            # use PID to move forward accurately, at 200 ticks per second (~0.5 m/s)
            robot.forward_PID(200, False)
            
            # retreive distance information
            _, forward = robot.check()
        
        # stop, center the robot, and then actually turn
        robot.stop()
        robot.center()
        print("centered")
        robot.turn_PID(robot.to_turn, 90)
        
        # set variables, renew the time out variable
        robot.to_turn = "none"
        robot.stage = 0
        robot.action_time = time()

    if robot.stage == 0:
        # default state: move forward
        
        # dynamically adjust using a QR, if detected
        if loc > 360:
            robot.alphaL += 0.0001
            robot.alphaR -= 0.0001
        elif loc < 360 and loc != -1:
            robot.alphaL -= 0.0001
            robot.alphaR += 0.0001

        # use PID loop - reset if too long
        if time() - robot.action_time > 30:
            robot.forward_PID(400, True)
        else:
            robot.forward_PID(400, False)
            

# safely remove the robot object
robot.cleanup()





#   Thank you to all those who came
#   before me and made this project
#   possible.
#
#         -Tiernan Lindauer
