import cv2
import RPi.GPIO as GPIO
import time as time
import math
import Encoder
enc = Encoder.Encoder(23, 24)

GPIO.setup(13, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setmode(GPIO.BCM)
right_pwm = GPIO.PWM(12, 400)
right_pwm.start(0)
left_pwm = GPIO.PWM(13, 400)
left_pwm.start(0)

#print("Sleeping!")
#time.sleep(5)

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

def turn_right():
    deg = 0
    shut_off = time.time()
    drive_left_motor(0.2)
    drive_right_motor(-0.2) 
    enc.write()
    
    while abs(deg) < 275:
        ticks = enc.read()
        deg = ticks/1.46
        print(deg)
        if time.time() - shut_off > 5:
            exit()
    left_stop()
    right_stop()
    offset = enc.read()

def turn_left():
    deg = 0
    shut_off = time.time()
    drive_left_motor(-0.37)
    drive_right_motor(0.32) 
    enc.write()

    while abs(deg) < 260:
        ticks = enc.read()
        deg = ticks/1.46
        print(deg)
        if time.time() - shut_off > 5:
            exit()
    left_stop()
    right_stop()
    offset = enc.read()

# set up camera object
cap = cv2.VideoCapture(0)

# QR code detection object
detector = cv2.QRCodeDetector()
endgame = False
stage = 0
cooldown = 100
while True:
    # get the image
    _, img = cap.read()
    
    # get bounding box coords and data
    data, bbox, _ = detector.detectAndDecode(img)
    
    # if there is a bounding box, draw one, along with the data
    if(bbox is not None):
        print("qr code detected!")
        for i in range(len(bbox)):

            #cv2.line(img,int(bbox[0][0][0]), int(bbox[0][0][1]), (0, 255, 0), 5)
        box_width = int(bbox[0][1][0])-int(bbox[0][0][0])
        print(box_width)
        
        x_loc = (int(bbox[0][0][0])+int(bbox[0][1][0]))/2
        height, width, channels = img.shape
            
        if endgame:
            if stage == 0:
                if x_loc < (width/2)-(width/100):
                    drive_right_motor(-0.2)
                    drive_left_motor(0.2)
                elif x_loc > (width/2)+(width/100):
                    drive_right_motor(0.05)
                    drive_left_motor(-0.05)
                else:
                    left_stop()
                    right_stop()
                   #print("stage 0 complete")
                    stage = 1
            if stage == 1:
                if box_width < 200:
                    drive_right_motor(-0.15)
                    drive_left_motor(-0.15)
                else:
                    left_stop()
                    right_stop()
                    #print("stage 1 complete, seeking complete!")
                    exit()
                    
            #cv2.putText(img, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
             #       0.5, (0, 255, 0), 2)
        if data:
            if ((width > 250) and (time.time()-cooldown > 5)):
                if str(data) == "left":
                    turn_left()
                    cooldown = time.time()
                if str(data) == "right":
                    turn_right()
                    cooldown = time.time()
                if data == "target":
                    endgame = True
        else:
            left_stop()
            right_stop()
    else:
        if endgame:
            left_stop()
            right_stop()
        else:
            drive_left_motor(-0.15)
            drive_right_motor(-0.32)     
        
    # display the image preview
    M = cv2.getRotationMatrix2D((img.shape[1]//2, img.shape[0]//2),180,1)
    img = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))
    cv2.imshow("yeet", img)
    if(cv2.waitKey(1) == ord("q")):
        break
# destroy camera object, exit
cap.release()
cv2.destroyAllWindows()

right_pwm.stop()
left_pwm.stop()
GPIO.cleanup()
