import RPi.GPIO as GPIO
import time as time
import math
import Encoder
import numpy as np
import matplotlib.pyplot as plt

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
    left_deg = 0
    right_deg = 0
    shut_off = time.time()
    drive_left_motor(0.075)
    drive_right_motor(-0.075)
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

    def turn_left():
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
def get_speed(x):
    if x=="left":
        offset = left_enc.read()

        bt = time.time()
        while abs(abs(left_enc.read())-offset) < 10:
            time.sleep(.0001)
        mn = time.time()
        lt = mn-bt
        ls = 10/lt

        return ls, mn
    elif x=="right":
        offset = right_enc.read()

        bt = time.time()
        while abs(abs(right_enc.read())-offset) < 10:
            time.sleep(.0001)
        mn = time.time()
        rt = mn-bt
        rs = 10/rt

        return rs, mn

      
def turn_PID(x,y):
    mark_early = time.time()
    time_before = time.time()

    if x=="left":
        alpha_l = -0.25
        alpha_r = 0.25
        prev_l_speed = 400
        prev_r_speed = 400
        LI = 0
        RI = 0

        while(abs(left_enc.read()) < 480*(y/90)):
            drive_left_motor(alpha_l)
            drive_right_motor(alpha_r)

            left_speed, mark_now = get_speed("left")
            LI += 0.001*(400-left_speed)
            alpha_l -= (0.05*(400-left_speed)+LI+(0.001*(prev_l_speed-left_speed)/(mark_now-mark_early)))*0.00134

            #right_speed, mark_now = get_speed("right")
            #RI += 0.001*(400-abs(right_speed))
            #print(RI)
             #alpha_r += 0.01*(0.001*abs(400-abs(right_speed))+RI+(0.001*(prev_r_speed-right_speed)/(mark_now-mark_early)))*0.00134
            #print(alpha_l," ",alpha_r)

            #plt.scatter(float("{:.2f}".format(time.time()-time_before)),alpha_r,color="blue")

            plt.pause(0.05)

            prev_l_speed = left_speed
            #prev_r_speed = right_speed

            elif x=="right":
        alpha_l = 0.13
        alpha_r = -0.13
        prev_l_speed = 400
        prev_r_speed = 400
        LI = 0
        RI = 0

        while(abs(right_enc.read()) < 480*(y/90)):
            drive_left_motor(alpha_l)
            drive_right_motor(alpha_r)

            right_speed, mark_now = get_speed("right")
            RI += 0.001*(400-abs(right_speed))
            #print(RI)
            alpha_r += 0.01*(0.001*abs(400-abs(right_speed))+RI+(0.001*(prev_r_speed-right_speed)/(mark_now-mark_early)))*0.00134
            #print(alpha_l," ",alpha_r)

            #plt.scatter(float("{:.2f}".format(time.time()-time_before)),alpha_r,color="blue")

            plt.pause(0.05)

            #prev_l_speed = left_speed
            prev_r_speed = right_speed


turn_PID("right",90)
time.sleep(10)
turn_PID("left",90)


#cleanup
right_pwm.stop()
left_pwm.stop()
GPIO.cleanup()
