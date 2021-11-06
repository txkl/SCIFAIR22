import RPi.GPIO as GPIO
import time as time
import math
import Encoder
import numpy as np
import matplotlib.pyplot as plt
GPIO.setmode(GPIO.BCM)

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
    print("PW ",pulse_width," ",(float(pulse_width)/1000000.0)*400*100)
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

time_before = time.time()
alphaL = 0.11
prevSpeed = 400
mark_early = time.time()

a = 0
b = 0
c = 0

##time.sleep(1)
##drive_right_motor(0.25)
##time.sleep(1)
##right_enc.write()
##while abs(right_enc.read()) < 120:
##    print(right_enc.read())
##right_stop()
##print(time.time()-time_before)
plt.axis([0,10,0,1])
plt.ylim(0,0.3)
plt.xlim(0,60)
while(time.time()-time_before < 60):
    drive_left_motor(alphaL)
    
    left_enc.write()
    right_enc.write()

    btime = time.time()
    while abs(left_enc.read()) < 20:
        time.sleep(.0001)   
    left_time = time.time() - btime
    left_speed = 20/left_time
    mark_now = time.time()
    print("left speed: ",left_speed)

    a = 0.05*(400-left_speed)
    b += 0.001*(400-left_speed)
    c = (0.001*(prevSpeed-left_speed)/(mark_now-mark_early))
    print(a,"  ",b,"  ",c)
    abc = (a+b+c)*0.00134
    
    alphaL+=abc
    plt.scatter(float("{:.2f}".format(time.time()-time_before)),alphaL)
    plt.pause(0.05)
    
    prevSpeed = left_speed
    mark_early = time.time()

#cleanup
right_pwm.stop()
left_pwm.stop()
GPIO.cleanup()
