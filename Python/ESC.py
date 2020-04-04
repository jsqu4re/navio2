#!/usr/bin/sudo python

import sys
import time

import navio.pwm
import navio.util
import datetime

navio.util.check_apm()

PWM_OUTPUT = 13
SERVO_MIN = 1.000 #ms
SERVO_START = 1.120 #ms
SERVO_MAX = 2.000 #ms
SERVO_NOM = 1.700 #ms


def loop_for(seconds, func, *args):
    endTime = datetime.datetime.now() + datetime.timedelta(seconds=seconds)

    while True:
        if datetime.datetime.now() >= endTime:
            break
	print (args)
        func(*args)

def ramp(func, start, target):
   actual = start
   if (start < target):
      while(target > actual):
         actual += 0.001
         if actual > SERVO_MAX:
            actual = SERVO_MAX
         loop_for(0.05, func, actual)
   else:
      while(target < actual):
         actual -= 0.01
         if actual < SERVO_MIN:
            actual = SERVO_MIN
         loop_for(0.2, func, actual)


def main():
    pwm =  navio.pwm.PWM(PWM_OUTPUT)
    pwm.initialize()
    pwm.set_period(50)
    pwm.enable()

    loop_for(3, pwm.set_duty_cycle, SERVO_MAX)
    loop_for(5, pwm.set_duty_cycle, SERVO_MIN)
    ramp(pwm.set_duty_cycle, SERVO_START, SERVO_NOM)
    loop_for(15, pwm.set_duty_cycle, SERVO_NOM)
    ramp(pwm.set_duty_cycle, SERVO_NOM, SERVO_MIN)
    loop_for(3, pwm.set_duty_cycle, SERVO_MIN)

main()
