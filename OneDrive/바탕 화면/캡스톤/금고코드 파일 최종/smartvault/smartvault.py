import RPi.GPIO as GPIO
import time
import board
import busio
from digitalio import DigitalInOut, Direction
import adafruit_fingerprint
import cv2
import numpy as np
import os
from logging.config import dictConfig
import logging
import datetime
import pygame
from PIL import Image


def face_enroll():
    #이 두개의 파일은 opencv폴더의 data/haarcascades/ 아래파일명.xml을 복붙해서 같은폴더에 넣어두자.
    face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    eye_detector = cv2.CascadeClassifier('haarcascade_eye.xml')

    #비디오 소스를 캡쳐 클래스로 영상의 이미지연속캡쳐하여 다시 영상으로 만듬
    cap = cv2.VideoCapture("http://192.168.0.28:8080/stream/video.mjpeg")
    #cap = cv2.VideoCapture(0) #UV4L의 위 경로가 아닐경우 pi카메라가 이미 UV4L로 사용중이므로 충돌이 난다.
    #UV4L에도 최대 3명의 클라이언트만 들어갈수있다. 다음 코딩때 참고할것
    #따라서 웹스크래핑 socket()을 활용하여 UV4L에 있는 영상을 스크랩핑 하기로 하였다.
    if cap.isOpened():
        print ("width: {0}, height: {1}". format(cap.get(3),cap.get(4)))

    current_status = 0
    eyes_status = 0

    #재생할 파일의 넓이 얻기 저장할것과 저장하는 영상의 크기가 같아야함.
    width = cap.get(3)
    #재생할 파일의 높이 얻기
    height = cap.get(4)
    #재생할 파일의 프레임 레이트 얻기
    fps = cap.get(cv2.CAP_PROP_FPS)
    #linux 계열 DIVX, XVID, MJPG, X264, WMV1, WMV2. #XVID가 제일 낫다고 함.
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    filename = '얼굴인식영상.avi'
    src = '/home/pi/Desktop/cctv'
    #파일 stream 생성
    out = cv2.VideoWriter(filename, fourcc, fps, (int(width), int(height)))

    pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/face_data.mp3")
    pygame.mixer.music.play()
    time.sleep(2)
    # For each person, enter one numeric face id
    face_id = input('\n 유저 id를 입력 해 주세요. <return> ==>  ')
    print("\n 카메라 캡쳐중입니다. 카메라를 보고 기다려주세요.")
    pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/face_data1.mp3")
    pygame.mixer.music.play()
    # Initialize individual sampling face count
    count = 0
    while(True):
        ret, img = cap.read()
        #img = cv2.flip(img, -1) # flip video image vertically
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_detector.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0), 2)     
            count += 1
            # Save the captured image into the datasets folder
            cv2.imwrite("/home/pi/dataset/User." + str(face_id) + '.' + str(count) + ".jpg", gray[y:y+h,x:x+w])
            cv2.imshow('image', img)
        k = cv2.waitKey(100) & 0xff # Press 'ESC' for exiting video
        if k == 27:
            break
        elif count >= 30: # Take 30 face sample and stop video
             break
    # Do a bit of cleanup
    print("\n 프로그램 종료중입니다~")
    cap.release()
    cv2.destroyAllWindows()
    pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/face_data2.mp3")
    pygame.mixer.music.play()
    # Path for face image database
    path = '/home/pi/dataset'
    recognizer = cv2.face.LBPHFaceRecognizer_create()
    detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml");

    # function to get the images and label data
    def getImagesAndLabels(path):
        imagePaths = [os.path.join(path,f) for f in os.listdir(path)]     
        faceSamples=[]
        ids = []
        for imagePath in imagePaths:
            PIL_img = Image.open(imagePath).convert('L') # convert it to grayscale
            img_numpy = np.array(PIL_img,'uint8')
            id = int(os.path.split(imagePath)[-1].split(".")[1])
            faces = detector.detectMultiScale(img_numpy)
            for (x,y,w,h) in faces:
                faceSamples.append(img_numpy[y:y+h,x:x+w])
                ids.append(id)
        return faceSamples,ids
    print ("\n 얼굴인식트레이닝중.... 기다려주세요...")
    faces,ids = getImagesAndLabels(path)
    recognizer.train(faces, np.array(ids))

    # Save the model into trainer/trainer.yml
    recognizer.write('/home/pi/Desktop/smartvault/trainer/trainer.yml') # recognizer.save() worked on Mac, but not on Pi
    # Print the numer of faces trained and end program
    print("\n 프로그램 종료중~".format(len(np.unique(ids))))
    pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/face_data3.mp3")
    pygame.mixer.music.play()
    time.sleep(5)

door_open =0
#음성파일 재생
pygame.mixer.init()

pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/booting.mp3")
pygame.mixer.music.play() 
#비발디 사계 봄
buzzer = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzer, GPIO.OUT)
GPIO.setwarnings(False)

#금고 잠그기
# 릴레이1
GPIO.setup(21, GPIO.OUT)
# 릴레이2
GPIO.setup(26, GPIO.OUT)



#TCP통신 백그라운드 실행
time.sleep(10) #부팅이후 포트포워딩된 포트와 연결시간 고려하여 10초후 재생
terminal_command_TCP = "python3 /home/pi/Desktop/smartvault/TCP_android.py &" #백그라운드 실행은 &을 붙혀준다.
os.system(terminal_command_TCP) 
 
roop=1 
while True:
        #문닫기 
    timer=4
    while (timer !=0):
        timer = timer-1
        GPIO.output(26, GPIO.LOW)
        GPIO.output(21, GPIO.HIGH)
        time.sleep(1)
    #off
    GPIO.output(26, GPIO.HIGH)
    GPIO.output(21, GPIO.HIGH)
    #진동센서 부저 백그라운드 실행
    terminal_command_vib = "python3 /home/pi/Desktop/smartvault/vib_sound2.py &" #백그라운드 실행은 &을 붙혀준다.
    os.system(terminal_command_vib)
    #스마트 금고 보안 시작녹음실행
    pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/start1.mp3")
    pygame.mixer.music.play()    
# 4옥타브:도(1)/ 레(2)/ 미(3)/ 파(4)/ 솔(5)/ 라(6) / 시(7)   
    #scale = [ 262 , 294  , 330 ,  349 , 392 ,  440 ,   494 ]
    #spring = [ 1, 3, 3, 3, 2, 1, 5, 5, 4, 3, 3, 3, 2, 1, \
    #            5, 5, 4, 3, 4, 5, 4, 3, 2, 2 ]
    scale = [226, 294, 330, 349, 392, 440,494, 523]


    #변수 설정
    Tcp_setting = 18181818
    face_training = 73107310
    pwd = 1234
    count = 0

    while True:
        try:
            pwd_input = int(input("비밀번호 입력"))
            #FACE_REGISTER
            if pwd_input == Tcp_setting:
                os.system('pkill -9 -f TCP_android.py')
                os.system(terminal_command_TCP)
            if pwd_input == face_training:
                os.system('pkill -9 -f vib_sound2.py')
                face_enroll()

            if pwd_input == pwd:
                os.system('pkill -9 -f vib_sound2.py')
                pwm = GPIO.PWM(buzzer, 1.0) # 초기 주파수를 1Hz로 설정
                pwm.start(50.0) # 듀티비를 90%로 높여 설정함(음 구분이 더 잘되고 조금 더 부드럽게 들림)
                for i in range(0,8):
                    pwm.ChangeFrequency(scale[i])
                    time.sleep(1.0)
                pwm.stop()
                #    pwm.ChangeFrequency(scale[spring[i]])
                #    if i==1 or i==6 or i==14 or i==20:
                #        time.sleep(0.2)   # 짧은 음표 부분을 모두 0.2초로 출력
                #    if i==7 or i==15:     # 긴 음표부분을 모두 1로 출력
                #        time.sleep(1)
                #    else :
                #        time.sleep(0.4)   # 기타 4분음표는 모두 0.4초로 출력함
     
                print("로그인")
                break
            
            else:

                count +=1 #비밀번호 틀리면 카운트하기
                if count < 5:
                    pwm = GPIO.PWM(buzzer, 262) # '262'는 음의 높이에 해당하는 특정 주파수
                    pwm.start(50.0)
                    time.sleep(1.5) # 1.5초간 음이 울리도록 시간지연
                    pwm.stop() 

                if count == 5: #5회이상 틀리면 아래 코드 시행
                    pwm = GPIO.PWM(buzzer, 1.0)  # 초기 주파수 설정을 1Hz로 함.
                    pwm.start(50.0)
     
                    for cnt in range(0,2):
                        pwm.ChangeFrequency(262) #낮은 도
                        time.sleep(0.5)
                        pwm.ChangeFrequency(523) #높은 도
                        time.sleep(0.5)

                    pwm.ChangeDutyCycle(0.0)
         
                    pwm.stop()
                    
                    print("출력 10초 후에 다시 입력")
                    pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/pw_fail.mp3")
                    pygame.mixer.music.play()
                    for i in range(10):
                        time.sleep(1) #1초 텀 주기
                        print(10-i,'초 기다리기')
                        count = 0 #count 초기화        
                    continue

        except ValueError:
            pass




    led = DigitalInOut(board.D13)
    led.direction = Direction.OUTPUT

    #uart = busio.UART(board.TX, board.RX, baudrate=57600)

    # If using with a computer such as Linux/RaspberryPi, Mac, Windows with USB/serial converter:
    import serial
    uart = serial.Serial("/dev/ttyUSB0", baudrate=57600, timeout=1)

    # If using with Linux/Raspberry Pi and hardware UART:
    # import serial
    # uart = serial.Serial("/dev/ttyS0", baudrate=57600, timeout=1)

    finger = adafruit_fingerprint.Adafruit_Fingerprint(uart)

    ##################################################


    def get_fingerprint():
        """Get a finger print image, template it, and see if it matches!"""
        print("지문을 기다리는 중 입니다...")
        while finger.get_image() != adafruit_fingerprint.OK:
            pass
        print("Templating...")
        if finger.image_2_tz(1) != adafruit_fingerprint.OK:
            return False
        print("서치중...")
        if finger.finger_search() != adafruit_fingerprint.OK:
            return False
        return True


    # pylint: disable=too-many-branches
    def get_fingerprint_detail():
        """Get a finger print image, template it, and see if it matches!
        This time, print out each error instead of just returning on failure"""
        print("Getting image...", end="", flush=True)
        i = finger.get_image()
        if i == adafruit_fingerprint.OK:
            print("Image taken")
        else:
            if i == adafruit_fingerprint.NOFINGER:
                print("No finger detected")
            elif i == adafruit_fingerprint.IMAGEFAIL:
                print("Imaging error")
            else:
                print("Other error")
            return False

        print("Templating...", end="", flush=True)
        i = finger.image_2_tz(1)
        if i == adafruit_fingerprint.OK:
            print("Templated")
        else:
            if i == adafruit_fingerprint.IMAGEMESS:
                pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/finger_messy.mp3")
                pygame.mixer.music.play()
                print("Image too messy")
            elif i == adafruit_fingerprint.FEATUREFAIL:
                print("Could not identify features")
            elif i == adafruit_fingerprint.INVALIDIMAGE:
                print("Image invalid")
            else:
                print("Other error")
            return False

        print("Searching...", end="", flush=True)
        i = finger.finger_fast_search()
        # pylint: disable=no-else-return
        # This block needs to be refactored when it can be tested.
        if i == adafruit_fingerprint.OK:
            print("Found fingerprint!")
            return True
        else:
            if i == adafruit_fingerprint.NOTFOUND:
                print("No match found")
            else:
                print("Other error")
            return False


    # pylint: disable=too-many-statements
    def enroll_finger(location):
        """Take a 2 finger images and template it, then store in 'location'"""
        for fingerimg in range(1, 3):
            if fingerimg == 1:
                print("Place finger on sensor...", end="", flush=True)
            else:
                pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/finger_enroll.mp3")
                pygame.mixer.music.play()
                time.sleep(4)
                print("Place same finger again...", end="", flush=True)

            while True:
                i = finger.get_image()
                if i == adafruit_fingerprint.OK:
                    print("Image taken")
                    break
                if i == adafruit_fingerprint.NOFINGER:
                    print(".", end="", flush=True)
                elif i == adafruit_fingerprint.IMAGEFAIL:
                    print("Imaging error")
                    return False
                else:
                    print("Other error")
                    return False

            print("Templating...", end="", flush=True)
            i = finger.image_2_tz(fingerimg)
            if i == adafruit_fingerprint.OK:
                print("Templated")
            else:
                if i == adafruit_fingerprint.IMAGEMESS:
                    print("Image too messy")
                elif i == adafruit_fingerprint.FEATUREFAIL:
                    print("Could not identify features")
                elif i == adafruit_fingerprint.INVALIDIMAGE:
                    print("Image invalid")
                else:
                    print("Other error")
                return False

            if fingerimg == 1:
                print("Remove finger")
                time.sleep(1)
                while i != adafruit_fingerprint.NOFINGER:
                    i = finger.get_image()

        print("Creating model...", end="", flush=True)
        i = finger.create_model()
        if i == adafruit_fingerprint.OK:
            print("Created")
            pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/finger_complete.mp3")
            pygame.mixer.music.play()
            time.sleep(3.5)
        else:
            if i == adafruit_fingerprint.ENROLLMISMATCH:
                print("Prints did not match")
                pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/finger_enroll_fail.mp3")
                pygame.mixer.music.play()
                time.sleep(3.5)
            else:
                print("Other error")
            return False

        print("Storing model #%d..." % location, end="", flush=True)
        i = finger.store_model(location)
        if i == adafruit_fingerprint.OK:
            print("Stored")
        else:
            if i == adafruit_fingerprint.BADLOCATION:
                print("Bad storage location")
            elif i == adafruit_fingerprint.FLASHERR:
                print("Flash storage error")
            else:
                print("Other error")
            return False

        return True


    ##################################################


    def get_num():
        """Use input() to get a valid number from 1 to 127. Retry till success!"""
        i = 0
        while (i > 127) or (i < 1):
            try:
                i = int(input("ID를 입력해주세요. # 1부터 127까지: "))
            except ValueError:
                pass
        return i


    while True:
        pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/finger_start.mp3")
        pygame.mixer.music.play()
        print("----------------")
        if finger.read_templates() != adafruit_fingerprint.OK:
            raise RuntimeError("Failed to read templates")
        print("이미 등록된 지문 번호:", finger.templates)
        print("9) 지문을 입력하여 등록해주세요.")
        print("1) 지문을 입력하여 신원 인증해주세요.")
        print("0) 지문을 삭제합니다.")
        print("----------------")
        c = input("> ")

        
        if c == "9":
            pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/finger_number.mp3")
            pygame.mixer.music.play()
            enroll_finger(get_num())
 
        if c == "1":
            pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/fingerprint_recognize.mp3")
            pygame.mixer.music.play()
            if get_fingerprint():
                
                pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/fingerprint_success.mp3")
                pygame.mixer.music.play()
                
                print("신원 정보 : ", finger.finger_id, "정확도 : ", finger.confidence)
                break
            else:
                pygame.mixer.music.load('/home/pi/Desktop/smartvault/voice/fingerprint_fail.mp3')
                pygame.mixer.music.play()
                
                print("지문을 찾지못하였습니다.")
                time.sleep(4)
        if c == "0":
            pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/finger_del.mp3")
            pygame.mixer.music.play()
            if finger.delete_model(get_num()) == adafruit_fingerprint.OK:
                pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/finger_del_s.mp3")
                pygame.mixer.music.play()
                
                print("삭제 하였습니다.")
                time.sleep(4)
            else:
                print("삭제에 실패 하였습니다.")
            continue


    GPIO.setmode(GPIO.BCM)

    # 릴레이1
    GPIO.setup(21, GPIO.OUT)
    # 릴레이2
    GPIO.setup(26, GPIO.OUT)

    #버튼제어
    GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    recognizer = cv2.face.LBPHFaceRecognizer_create()
    recognizer.read('/home/pi/Desktop/smartvault/trainer/trainer.yml')
    cascadePath = "/home/pi/Desktop/smartvault/haarcascade_frontalface_default.xml"
    faceCascade = cv2.CascadeClassifier(cascadePath);
    font = cv2.FONT_HERSHEY_SIMPLEX

    #iniciate id counter
    id = 0

    # names related to ids: example ==> loze: id=1,  etc
    # 이런식으로 사용자의 이름을 사용자 수만큼 추가해준다.
    names = ['None', '1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14', '15', '16', '17', '18', '19', '20']

    # Initialize and start realtime video capture
    cam = cv2.VideoCapture("http://192.168.0.28:8080/stream/video.mjpeg")
    if not cam.isOpened():
        pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/camera_fail.mp3")
        pygame.mixer.music.play()
        print('카메라 연결안됨')
        
    cam.set(3, 640) # set video widht
    cam.set(4, 480) # set video height

    # Define min window size to be recognized as a face
    minW = 0.1*cam.get(3)
    minH = 0.1*cam.get(4)

    camsuccess =0
    camfail=0
    while True:
        #두번성공하면 캠꺼지기 5번실패해도 캠꺼지기
        if camfail == 5:
            break
        if camsuccess == 2:
            door_open =1
            break
        if GPIO.input(20) == False:
                #문닫기 
            timer=4
            while (timer !=0):
                timer = timer-1
                GPIO.output(26, GPIO.LOW)
                GPIO.output(21, GPIO.HIGH)
                time.sleep(1)
                #off
            GPIO.output(26, GPIO.HIGH)
            GPIO.output(21, GPIO.HIGH)
            print("문을 강제로 닫습니다.")
            break
        #로그 남기기
        dictConfig({
            'version': 1,
            'formatters': {
                'default': {
                    'format': '[%(asctime)s] %(message)s',
                }
            },
            'handlers': {
                'file': {
                    'level': 'DEBUG',
                    'class': 'logging.FileHandler',
                    'filename': '/home/pi/Desktop/smartvault/log/logfile_{:%Y%m%d %H:%M}.log'.format(datetime.datetime.now()),
                    'formatter': 'default',
                },
            },
            'root': {
                'level': 'DEBUG',
                'handlers': ['file']
            }
        })

        ret, img =cam.read()
        img = cv2.flip(img, 3) # 1넣으면 좌우반전 0넣으면 상하반전
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        faces = faceCascade.detectMultiScale( 
            gray,
            scaleFactor = 1.2,
            minNeighbors = 5,
            minSize = (int(minW), int(minH)),
           )

        for(x,y,w,h) in faces:
            cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)
            id, confidence = recognizer.predict(gray[y:y+h,x:x+w])
            # Check if confidence is less them 100 ==> "0" is perfect match
            if (confidence < 100):
                id = names[id]
                confidence = "  {0}%".format(round(100 - confidence))

                #문열기 
                timer=4
                while (timer !=0):
                    timer = timer-1
                    GPIO.output(26, GPIO.HIGH)
                    GPIO.output(21, GPIO.LOW)
                    time.sleep(1)
                #off
                GPIO.output(26, GPIO.HIGH)
                GPIO.output(21, GPIO.HIGH)

                pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/face_success.mp3")
                pygame.mixer.music.play()
                print('{}님의 얼굴인식 완료,문이 열립니다.'.format(id))
                camsuccess +=1
                def myfunc():
                    logging.debug("{}가 식별 되었습니다.".format(id))
                
                myfunc()
            else:
                id = "unknown"
                confidence = "  {0}%".format(round(100 - confidence))
                #문닫기 
                timer=4
                while (timer !=0):
                    timer = timer-1
                    GPIO.output(26, GPIO.LOW)
                    GPIO.output(21, GPIO.HIGH)
                    time.sleep(1)
                #off
                GPIO.output(26, GPIO.HIGH)
                GPIO.output(21, GPIO.HIGH)
                

                pygame.mixer.music.load("/home/pi/Desktop/smartvault/voice/face_fail.mp3")
                pygame.mixer.music.play()
                print('문을 닫습니다.')
                camfail +=1
                def myfunc():
                    logging.debug("미인식 등록자가 식별되었습니다.")


                myfunc()
                time.sleep(1)
            cv2.putText(img, str(id), (x+5,y-5), font, 1, (255,255,255), 2)
            cv2.putText(img, str(confidence), (x+5,y+h-5), font, 1, (255,255,0), 1)  
        
        cv2.imshow('camera',img) 
        cv2.waitKey(10) & 0xff



    # Do a bit of cleanup
    time.sleep(15)
    print("\n 프로그램 닫고 화면 청소")
    cam.release()
    cv2.destroyAllWindows()






