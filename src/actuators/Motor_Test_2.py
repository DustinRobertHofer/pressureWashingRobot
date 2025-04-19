import RPi.GPIO as GPIO
import time

# Define GPIO pins for motor 1
Dir_pin1 = 20
Step_pin1 = 21

# Define GPIO pins for motor 2
Step_pin2 = 17
Dir_pin2 = 27

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(Dir_pin1, GPIO.OUT)
GPIO.setup(Step_pin1, GPIO.OUT)
GPIO.setup(Dir_pin2, GPIO.OUT)
GPIO.setup(Step_pin2, GPIO.OUT)

# Set direction: true = backwards, false = forwards
GPIO.output(Dir_pin1, False)
GPIO.output(Dir_pin2, True)

# step motor forward 200 steps
def step_motors(steps, delay):
    for x in range(steps):
        GPIO.output(Step_pin1, True)
        GPIO.output(Step_pin2, True)
        time.sleep(delay) # Adjust step delay for speed
        GPIO.output(Step_pin1, False)
        GPIO.output(Step_pin2, False)
        time.sleep(delay)
    
print("Stepping motors...")
step_motors(2000, 0.002) # 400 steps = 1 revolution for 0.9 
print("Done.")

GPIO.cleanup()
