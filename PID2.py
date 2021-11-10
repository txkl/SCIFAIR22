import RPi.GPIO as GPIO
import time as time
import math
import Encoder
import numpy as np
import matplotlib.pyplot as plt
#GPIO.setmode(GPIO.BCM)

left_enc = Encoder.Encoder(23, 24)
right_enc = Encoder.Encoder(20, 21)

GPIO.setup(13, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)

right_pwm = GPIO.PWM(12, 400)
right_pwm.start(0)
left_pwm = GPIO.PWM(13, 400)
left_pwm.start(0)


def make_pulse(power):
    if power > 0:
        return 990*power + 1490
    elif power < 0:
        return 990*power + 1510
    else:
        return 1500

def duty_cycle(pulse_width):
    #print("PW ",pulse_width," ",(float(pulse_width)/1000000.0)*400*100)
    return(float(pulse_width)/1000000.0)*400*100
def drive_right_motor(pwr):
    if pwr>1:
        pwr=0.9
    if pwr<0:
        pwr=0
    right_pwm.ChangeDutyCycle(duty_cycle(make_pulse(-pwr)))
def drive_left_motor(pwr):
    left_pwm.ChangeDutyCycle(duty_cycle(make_pulse(pwr)))
def left_stop():
    left_pwm.ChangeDutyCycle(0)
def right_stop():
    right_pwm.ChangeDutyCycle(0)
def getSpeed(x):
    if x=="left":
        left_enc.write()

        bt = time.time()
        while abs(left_enc.read()) < 10:
            time.sleep(.0001)
        mn = time.time()
        lt = mn-bt
        ls = 10/lt

        return ls, mn
    elif x=="right":
        right_enc.write()

        bt = time.time()
        while abs(right_enc.read()) < 10:
            time.sleep(.0001)
        mn = time.time()
        rt = mn-bt
        rs = 10/rt

        return rs, mn


plt.axis([0,10,0,1])
plt.ylim(0,0.5)
plt.xlim(0,10)
def forward_PID():
    time_before = time.time()
    alphaL = 0.11
    alphaR = 0.25

    prev_l_speed = 400
    prev_r_speed = 400

    LI = 0
    RI = 0
    while(time.time()-time_before < 30):
        mark_early = time.time()

        drive_left_motor(alphaL)
        drive_right_motor(alphaR)

        left_speed, mark_now = getSpeed("left")
        LI += 0.001*(410-left_speed)
        alphaL += (0.05*(410-left_speed)+LI+(0.001*(prev_l_speed-left_speed)/(mark_now-mark_early)))*0.00134

        right_speed, mark_now = getSpeed("right")
        RI += 0.001*(430-right_speed)
        alphaR += (0.05*(430-right_speed)+RI+(0.001*(prev_r_speed-right_speed)/(mark_now-mark_early)))*0.00134

        plt.scatter(float("{:.2f}".format(time.time()-time_before)),alphaR,color="blue")

        plt.pause(0.05)

        prev_l_speed = left_speed
        prev_r_speed = right_speed
time.sleep(20)
forward_PID()

#cleanup
right_pwm.stop()
left_pwm.stop()
GPIO.cleanup()
