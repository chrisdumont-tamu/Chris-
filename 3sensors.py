import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

Trigger1 = 8
Trigger2=36
Trigger3=16
Echo1 = 38
Echo2=32
Echo3=18

GPIO.setup(Trigger1, GPIO.OUT)
GPIO.setup(Echo1, GPIO.IN)
GPIO.setup(Trigger2,GPIO.OUT)
GPIO.setup(Echo2,GPIO.IN)
GPIO.setup(Trigger3, GPIO.OUT)
GPIO.setup(Echo3,GPIO.IN)


def sensor1():
  GPIO.output(Trigger1, GPIO.LOW)

  print ("Waiting for sensor to settle")
  time.sleep(0.5)
  print ("Calculating distance")
  GPIO.output(Trigger1, GPIO.HIGH)
  time.sleep(0.00001)
  GPIO.output(Trigger1, GPIO.LOW)
  while GPIO.input(Echo1)==0:
    pulse_start_time = time.time()
  while GPIO.input(Echo1)==1:
    pulse_end_time = time.time()

  pulse_duration = pulse_end_time - pulse_start_time
  distance = round(pulse_duration * 17150, 2)
  print ("Distance:",distance,"cm")

def sensor2():
  GPIO.output(Trigger2, GPIO.LOW)

  print ("Waiting for sensor to settle")
  time.sleep(0.5)
  print ("Calculating distance")
  GPIO.output(Trigger2, GPIO.HIGH)
  time.sleep(0.00001)
  GPIO.output(Trigger2, GPIO.LOW)
  while GPIO.input(Echo2)==0:
    pulse_start_time = time.time()
  while GPIO.input(Echo2)==1:
    pulse_end_time = time.time()

  pulse_duration = pulse_end_time - pulse_start_time
  distance = round(pulse_duration * 17150, 2)
  print ("Distance:",distance,"cm")

def sensor3():
  GPIO.output(Trigger3, GPIO.LOW)

  print ("Waiting for sensor to settle")
  time.sleep(0.5)
  print ("Calculating distance")
  GPIO.output(Trigger3, GPIO.HIGH)
  time.sleep(0.00001)
  GPIO.output(Trigger3, GPIO.LOW)
  while GPIO.input(Echo3)==0:
    pulse_start_time = time.time()
  while GPIO.input(Echo3)==1:
    pulse_end_time = time.time()

  pulse_duration = pulse_end_time - pulse_start_time
  distance = round(pulse_duration * 17150, 2)
  print ("Distance:",distance,"cm")


sensor1()
sensor2()
sensor3()
