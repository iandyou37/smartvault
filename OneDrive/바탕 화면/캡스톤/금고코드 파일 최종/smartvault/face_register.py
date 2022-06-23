import cv2
import numpy as np
import socket #소켓 불러오기
import time
from PIL import Image
import os
import pygame
import time

#음성파일 재생
pygame.mixer.init()

#이 두개의 파일은 opencv폴더의 data/haarcascades/ 아래파일명.xml을 복붙해서 같은폴더에 넣어두자.
face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_detector = cv2.CascadeClassifier('haarcascade_eye.xml')

#비디오 소스를 캡쳐 클래스로 영상의 이미지연속캡쳐하여 다시 영상으로 만듬
cap = cv2.VideoCapture("http://192.168.0.22:8080/stream/video.mjpeg")
#cap = cv2.VideoCapture(0) #UV4L의 위 경로가 아닐경우 pi카메라가 이미 UV4L로 사용중이므로 충돌이 난다.
#UV4L에도 최대 3명의 클라이언트만 들어갈수있다. 다음 코딩때 참고할것
#따라서 웹스크래핑 socket()을 활용하여 UV4L에 있는 영상을 스크랩핑 하기로 하였다.
if cap.isOpened():
	print ("width: {0}, height: {1}". format(cap.get(3),cap.get(4)))

current_status = 0
eyes_status = 0

#Create a UDP socket , # 파이썬 웹스크래핑 교수님께 물어볼 때 참조 https://blog.naver.com/jay-bird/221924699311 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #TCP/IP(SOCKET)소켓 생성 AF_INET - IPv4를 사용해서 통신하는거
pi = ('192.168.0.22', 4210) 
message = 'Hey google'.encode()

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

