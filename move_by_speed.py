import RPi.GPIO as GPIO
import time as time
import math
import Encoder
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
    
    while (time.time() - tot_time) < 10:

        drive_left_motor(baseL)
        drive_right_motor(baseR)
        print(baseL)
        print(baseR)
        print(" ")
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
        
        if left_speed < speed-5:
            baseL += 0.01
        elif left_speed > speed+5:
            baseL -= 0.01

        if right_speed < speed+3:
            baseR += 0.01
        elif right_speed > speed+13:
            baseR -= 0.01

        time.sleep(.01)
        
    left_stop()
    right_stop()

time.sleep(20)
forward_at_speed(400)


#cleanup
right_pwm.stop()
left_pwm.stop()
GPIO.cleanup()
