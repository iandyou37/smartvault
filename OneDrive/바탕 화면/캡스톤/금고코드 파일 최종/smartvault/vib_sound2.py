import RPi.GPIO as GPIO
import time
from subprocess import call

class Sw420(object):
    def __init__(self, pin , buzzer):
        self.buzzer = buzzer
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.buzzer,GPIO.OUT)
        GPIO.setwarnings(False)
        self.pin = pin
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        GPIO.add_event_detect(self.pin, GPIO.RISING, callback=self.callback, bouncetime=1)
        self.count = 0 

    def callback(self , pin):
        self.count += 1

    def buzzerOn(self):
        pwm = GPIO.PWM(self.buzzer, 1.0)  # 초기 주파수 설정을 1Hz로 함.
        pwm.start(50.0)

        for cnt in range(0,2):
            pwm.ChangeFrequency(262) #낮은 도
            time.sleep(0.5)
            pwm.ChangeFrequency(523) #높은 도
            time.sleep(0.5)

        pwm.ChangeDutyCycle(0.0)
             
        pwm.stop()
        print("진동이 감지되었습니다.")

    def buzzerOff(self):
        GPIO.output(self.buzzer , 0)
        print("진동이 없습니다.")



def vib_buzzer():
    sensor = Sw420(17,16) #sw-420 시그널 GPIO, buzzer GPIO로 객체 생성
    try:
        while True:
            time.sleep(1)
            if sensor.count >=10: #민감도? 진동지속시간?
                sensor.buzzerOn() #LED를 켠다.
                
            else:
                sensor.buzzerOff()
            sensor.count = 0

    except KeyboardInterrupt:
        GPIO.cleanup()

vib_buzzer()