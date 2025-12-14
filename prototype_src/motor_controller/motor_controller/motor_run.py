import RPi.GPIO as GPIO
import time
import threading

# -------------------------
# PIN CONFIG
# -------------------------
IN1 = 5
IN2 = 6
ENA = 26

PWM_FREQ = 1000
SPEED = 60          # %
FORWARD_TIME = 1.5
REVERSE_TIME = 1.5
PAUSE_TIME = 0.3

# -------------------------
# GPIO SETUP
# -------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

pwm = GPIO.PWM(ENA, PWM_FREQ)
pwm.start(0)

# -------------------------
# MOTOR ACTIONS
# -------------------------
def forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(SPEED)

def reverse():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(SPEED)

def stop():
    pwm.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

# -------------------------
# SEQUENCE
# -------------------------
def run_forward_reverse(on_finished_cb=None):

    def _worker():
        # Forward
        forward()
        time.sleep(FORWARD_TIME)

        # Pause
        stop()
        time.sleep(PAUSE_TIME)

        # Reverse
        reverse()
        time.sleep(REVERSE_TIME)

        # Stop
        stop()

        if on_finished_cb:
            on_finished_cb()
