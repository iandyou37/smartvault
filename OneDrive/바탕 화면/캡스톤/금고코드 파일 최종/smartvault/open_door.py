import RPi.GPIO as GPIO
import time
import board
GPIO.setmode(GPIO.BCM)

    # 릴레이1
GPIO.setup(21, GPIO.OUT)
    # 릴레이2
GPIO.setup(26, GPIO.OUT)

    #버튼제어
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_UP)


timer=4
while (timer !=0):
    timer = timer-1
    GPIO.output(26, GPIO.LOW)
    GPIO.output(21, GPIO.HIGH)
    time.sleep(1)
#off
GPIO.output(26, GPIO.HIGH)
GPIO.output(21, GPIO.HIGH)


