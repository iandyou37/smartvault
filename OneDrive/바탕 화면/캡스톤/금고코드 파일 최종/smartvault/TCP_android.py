import socket
from _thread import *
import time
import board
import adafruit_dht
import schedule
import datetime
import sys
import RPi.GPIO as GPIO

host = '192.168.0.28' # 호스트 ip를 적어주세요
port = 55555           # 포트번호를 임의로 설정해주세요

server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#재사용 명령어 중요!!!!
server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)


#dht
dhtDevice = adafruit_dht.DHT22(board.D4, use_pulseio=False)

#진동센서가 감지되면 10초동안 안드로이드에 경고 문구 보냄.
def vib1():
    for i in range(10):
        temperature_c = dhtDevice.temperature
        humidity = dhtDevice.humidity
        
        dht=[26.8,40.1,0]
        dht[0] =temperature_c
        dht[1] =humidity
        dht[2] =1
        dht = str(dht)[1:-1]#str 로 변환중에 괄호 제거
        dht = dht.encode("UTF-8")
        print (dht)
        client_sock.send(dht)
        time.sleep(1)
    
def threaded(client_sock, addr):
    while True:
        try:
            temperature_c = dhtDevice.temperature
            humidity = dhtDevice.humidity
                            
            dht=[26.8,40.1,1]
            dht[0] =temperature_c
            dht[1] =humidity
                            
            #진동센서
            GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            result = GPIO.input(17)
            if result == 1:
                vib1()
                                                      
            #진동값 0이면 평상시 1이면 진동감지 dht[2]가 dht의 3번째값인 진동센서값이다.
            elif result == 0:
                dht[2] =0



            #print(data2.encode())
            dht = str(dht)[1:-1] #str로 변환중 생기는 괄호 제거
            dht = dht.encode("UTF-8")

            print (dht)
            client_sock.send(dht)
            time.sleep(0.1)
            
                
        except RuntimeError as error:
            # DHT센서 런타임 오류 났을때 계속 진행 하게끔 오류처리.
            print(error.args[0])
            time.sleep(2.0)
            continue

        except Exception as error:
            dhtDevice.exit()
            raise error

            
        except ConnectionResetError as e:
            break
        except socket.error as err:
            print('서버 에러: ' ,err)




server_sock.bind((host, port))#튜플로 bind 받아오깅
server_sock.listen(5)# 클라이언트 동시 접속 5명으로 임의 설정해둠ㅎ

            
while True:
    print("기다리는 중") 
    client_sock, addr = server_sock.accept()
    print('Connected by', addr)
    start_new_thread(threaded, (client_sock,addr))
        
               


client_sock.close()
server_sock.close()










