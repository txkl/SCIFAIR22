#TIERNAN LINDAUER
#LAST EDIT 9/6/21
#LAST BACKED UP VERSION 9/6/21 15:53
#SCIFAIR MAIN PROGRAM

#IMPORTED MODULES
import cv2
import RPi.GPIO as GPIO
import time as time
import math
import Encoder
from gpiozero import DistanceSensor

#OBJECT DEFINITIONS
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

def forward_at_speed(speed):#speed in ticks per second
    tot_time = time.time()
    left_speed = 0
    right_speed = 0

    baseL = 0.11
    baseR = 0.26
    
    while (time.time() - tot_time) < 0.2:

        drive_left_motor(baseL)
        drive_right_motor(baseR)
        
        left_enc.write()
        right_enc.write()

        before_time = time.time()
        while abs(left_enc.read()) < 20:
            time.sleep(.0001)   
        left_time = time.time() - before_time
        left_speed = 20/left_time
        #print("left speed: ",left_speed)

        left_enc.write()
        right_enc.write()

        before_time = time.time()
        while abs(right_enc.read()) < 20:
            time.sleep(.0001)
            #print(right_enc.read())
        right_time = time.time() - before_time
        right_speed = 20/right_time
        #print("right speed: ",right_speed)
        
        if left_speed < speed-5:
            baseL += 0.01
        elif left_speed > speed+5:
            baseL -= 0.01

        if right_speed < speed+2:
            baseR += 0.01
        elif right_speed > speed+12:
            baseR -= 0.01

        time.sleep(.01)
        
    left_stop()
    right_stop()

def check(stage):
    if bottom_sensor.distance * 100 >= 10:
        exit()
    if forward_sensor.distance * 100 <= 20:
        left_stop()
        right_stop()
        time.sleep(5)
        if forward_sensor.distance * 100 <= 15 and stage == 0:
            exit()
    
def turn_right():
    left_stop()
    right_stop()
    time.sleep(0.5)
    left_deg = 0
    right_deg = 0
    shut_off = time.time()
    drive_left_motor(0.075)
    drive_right_motor(-0.075) 
    left_enc.write()
    right_enc.write()
    x = 310
    while (abs(left_deg) < x) or (abs(right_deg) < x):
        print(left_deg," ", right_deg)
        if abs(left_deg) >= x:
            left_stop()
        if abs(right_deg) >= x:
            right_stop()
            
        left_deg = left_enc.read()/1.46
        right_deg = right_enc.read()/1.46
        if time.time() - shut_off > 5:
            exit()
    left_stop()
    right_stop()

    left_enc.write()
    right_enc.write()

def turn_left():
    left_stop()
    right_stop()
    time.sleep(0.5)
    left_deg = 0
    right_deg = 0
    shut_off = time.time()
    drive_left_motor(-0.2)
    drive_right_motor(0.2) 
    left_enc.write()
    right_enc.write()
    x = 310
    while (abs(left_deg) < x) or (abs(right_deg) < x):
        if abs(left_deg) >= x:
            left_stop()
        if abs(right_deg) >= x:
            right_stop()
            
        left_deg = left_enc.read()/1.46
        right_deg = right_enc.read()/1.46
        print(left_deg)
        if time.time() - shut_off > 7:
            exit()
    left_stop()
    right_stop()
        
#VARIABLES
stage = 0
endgame = False
cooldown = 0
last_movement = ""
target = False

time.sleep(10)
#LOOP
while True:
    check(stage)
    _, img = cap.read()
    data, bbox, _ = detector.detectAndDecode(img)
    
    if(bbox is not None):
        box_width = abs(int(bbox[0][1][0])-int(bbox[0][0][0]))
        x_loc = (int(bbox[0][0][0])+int(bbox[0][1][0]))/2
        height, width, channels = img.shape
        if data == "target":
            stage == 1
        if stage == 1:
            if x_loc < (width/2)-(width/20):
                drive_right_motor(0.2)
                drive_left_motor(-0.2)
                print("left")
                time.sleep(.03)
                left_stop()
                right_stop()
                last_movement = "left"
            elif x_loc > (width/2)+(width/20):
                drive_right_motor(-0.1)
                drive_left_motor(0.1)
                print("right")
                time.sleep(0.03)
                left_stop()
                right_stop()
                last_movement = "right"
            else:
                left_stop()
                right_stop()
                print("stage 1 complete")
                stage = 2
        if stage == 2:
            qr_detected = True
            print(box_width)
            while(box_width < 650) and qr_detected:
                forward_at_speed(400)
                time.sleep(0.1)
                _, img = cap.read()
                data, bbox, _ = detector.detectAndDecode(img)
                
                if(bbox is not None):
                    box_width = abs(int(bbox[0][1][0])-int(bbox[0][0][0]))
                    print("w: ",box_width)
                else:
                    qr_detected = False
                    print("Exited from no QR")
            stage = 3
            print("stage 2 done")
        if stage == 3:
            while forward_sensor.distance*100 > 5:
                forward_at_speed(200)
                time.sleep(0.1)
                print(forward_sensor.distance*100)

                
            left_stop()
            right_stop()
            print("stage 3 done")
            stage = 4
            exit()

    if data:
        print("data")
        if data == "target":
            stage = 1
        elif (data == "left") and box_width > 150:
            turn_left()
            cooldown = time.time()
        elif (data == "right") and box_width > 150:
            turn_right()
            cooldown = time.time()
    else:
        if stage != 0:
            left_stop()
            right_stop()
        else:
            L = .1
            R = .26
            while not target:
                check(stage)
                _, img = cap.read()
                data, bbox, _ = detector.detectAndDecode(img)
                box_width = 0
                if bbox is not None:
                    box_width = abs(int(bbox[0][1][0])-int(bbox[0][0][0]))
                if((bbox is not None) and (data == "target")):
                    target = True
                elif ((bbox is not None) and (data == "left") and box_width > 150):
                    right_stop()
                    left_stop()
                
                    turn_left()
                elif ((bbox is not None) and (data == "right") and box_width > 150):
                    right_stop()
                    left_stop()
                
                    turn_right()
                else:
                    print("L and R: ",L," ",R)

                    drive_left_motor(L)
                    drive_right_motor(R)

                    left_enc.write()
                    right_enc.write()

                    before_time = time.time()
                    while abs(left_enc.read()) < 20:
                        time.sleep(.0001)
                    left_time = time.time() - before_time
                    left_speed = 20/left_time
                    print("left speed: ",left_speed)

                    left_enc.write()
                    right_enc.write()

                    before_time = time.time()
                    while abs(right_enc.read()) < 20:
                        time.sleep(.0001)
                        #print(right_enc.read())
                    right_time = time.time() - before_time
                    right_speed = 20/right_time
                    print("right speed: ",right_speed)
                    
                    if left_speed < 400-5:
                        L += 0.01
                    elif left_speed > 400+5:
                        L -= 0.01

                    if right_speed < 400:
                        R += 0.01
                    elif right_speed > 400+10:
                        R -= 0.01
                    

        if (last_movement == "right") and (stage == 1):
            drive_right_motor(0.2)
            drive_left_motor(-0.2)
            print("left to get it again")
            time.sleep(.05)
            left_stop()
            right_stop()

        if (last_movement == "left") and (stage == 1):
            drive_right_motor(-0.1)
            drive_left_motor(0.1)
            print("right to get it again")
            time.sleep(0.05)
            left_stop()
            right_stop()
            
    if(cv2.waitKey(1) == ord("q")):
        break

#CLEANUP  
cap.release()
cv2.destroyAllWindows()
right_pwm.stop()
left_pwm.stop()
GPIO.cleanup()

