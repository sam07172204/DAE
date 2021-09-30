# USAGE
# python pi_face_recognition.py --cascade haarcascade_frontalface_default.xml --encodings encodings.pickle

# import the necessary packages
import speech_recognition as sr
import jieba
import time
import threading
import sys
from time import sleep
sys.path.insert(0, '../')
from SX127x.LoRa import *
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD
from gtts import gTTS
from pygame import mixer
import RPi.GPIO as GPIO
import json
from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
import argparse
import imutils
import pickle
import cv2
import socket
import os
import struct
import datetime
from datetime import datetime, timedelta
from picamera.array import PiRGBArray
from picamera import PiCamera
import smbus2
sys.modules['smbus'] = smbus2
from RPLCD.i2c import CharLCD
import numpy as np
import select
import subprocess
from math import acos, degrees
import operator
import MySQLdb
import random

def angle_to_duty_cycle(angle=0):
    PWM_FREQ = 50
    duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * angle / 180)
    return duty_cycle

def initEnv(irtransmit_pin, irreceive_pin, state_pin, send_pin):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(irtransmit_pin, GPIO.OUT)
    GPIO.setup(irreceive_pin, GPIO.IN)
    GPIO.setup(state_pin, GPIO.IN)
    GPIO.setup(send_pin, GPIO.IN)

def endEnv():
    GPIO.cleanup()

def servo_rotate(deg, servo_deg, pwm):
    step_deg = 2
    start_deg = 0
    stop_deg = 0
    if (deg > 170):
        deg = 170
        print("limit to 170 deg")
    if(deg >= servo_deg):
        while(servo_deg < deg):
            if((servo_deg + step_deg) > deg):
                servo_deg = deg
            else:
                servo_deg = servo_deg + step_deg
            deg_pwm = angle_to_duty_cycle(servo_deg)
            pwm.ChangeDutyCycle(deg_pwm)
            time.sleep(0.1)
            pwm.ChangeDutyCycle(0)
        print("Turn to deg: {0}".format(servo_deg))
    else:
        while(servo_deg > deg):
            if((servo_deg - step_deg) < deg):
                servo_deg = deg
            else:
                servo_deg = servo_deg - step_deg
            deg_pwm = angle_to_duty_cycle(servo_deg)
            pwm.ChangeDutyCycle(deg_pwm)
            time.sleep(0.1)
            pwm.ChangeDutyCycle(0)
        print("Turn to deg: {0}".format(servo_deg))
    time.sleep(1)
    return servo_deg

def servo_ready_to_record(motortype, now_deg, pwm, triger_pin):
    record_deg = []
    if(motortype == 180):
        print("round record")
        degree_path = "/home/pi/pi-face-recognition/degree.json"
        src = open(degree_path, 'r')
        try:
            record_deg = json.loads(src.read())
            print("\r\nRecord degree number: {0}, degree: {1}\r\n".format(len(record_deg), record_deg))
        except Exception as e:
            print(e)
            print("Fail to open degree.json in round record")
        src.close()
    elif(motortype == 360):
        while((now_deg-90) >= 0):
            while(GPIO.input(triger_pin) != 1):
                pwm.ChangeDutyCycle(6.5)
                time.sleep(0.08)
                pwm.ChangeDutyCycle(7.25)
                time.sleep(0.1)
            pwm.ChangeDutyCycle(6.5)
            time.sleep(0.08)
            pwm.ChangeDutyCycle(7.25)
            time.sleep(0.1)
            pwm.ChangeDutyCycle(0)
            time.sleep(0.5)
            while(GPIO.input(triger_pin) != 0):
                pwm.ChangeDutyCycle(6.5)
                time.sleep(0.08)
                pwm.ChangeDutyCycle(7.25)
                time.sleep(0.1)
            pwm.ChangeDutyCycle(6.5)
            time.sleep(0.08)
            pwm.ChangeDutyCycle(7.25)
            time.sleep(0.1)
            pwm.ChangeDutyCycle(0)
            time.sleep(0.5)
            now_deg -= 90
        record_deg = [90, 180, 270, 360]
        pass
    else:
        print("MotorType is unknow")
    return record_deg

def getSignal(irreceive_pin, state_pin, send_pin):
    start, stop = 0, 0
    signals = []
    get = GPIO.input(irreceive_pin)
    start = time.time()
    global dynamic_static_record
    send_pin_state = GPIO.input(send_pin)
    while True:
        get2 = GPIO.input(irreceive_pin)
        if get2 != get:
            end = time.time()
            duration = end - start
            start = end
            signals.append(duration)
            get = get2
        if ((time.time() - start) > 0.1) and len(signals) > 0:
            print("len(signals): {0}".format(len(signals[1:])))
            tmp = signals[1:]
            signals = []
            return tmp
        if GPIO.input(state_pin) == 0:
            print("change mode to send1")
            tmp = []
            return tmp
        if send_pin_state != GPIO.input(send_pin):
            send_pin_state = GPIO.input(send_pin)
            if GPIO.input(send_pin) == 0:
                dynamic_static_record = False
                print("Static record")
                LCD_set("Static record")
            else:
                dynamic_static_record = True
                print("Dynamic record")
                LCD_set("Dynamic record")

def record(irreceive_pin, state_pin, send_pin):
    tmp = getSignal(irreceive_pin, state_pin, send_pin)
    if (GPIO.input(state_pin) == 1) and (len(tmp) > 0):
        try:
            low = []
            high = []
            avg_tmp = sum(tmp[2:len(tmp)])/(len(tmp)-2)
            for i in range(2, len(tmp)):
                if tmp[i] >= (avg_tmp + 0.3*avg_tmp):
                    high.append(tmp[i])
                else:
                    low.append(tmp[i])

            low_avg = sum(low)/(len(low))
            high_avg = sum(high)/(len(high))
            record_array = []
            record_array.append(round(tmp[0],10))
            record_array.append(round(tmp[1],10))
            for i in range(2, len(tmp)):
                if tmp[i] >= (avg_tmp + 0.3*avg_tmp):
                    record_array.append(round(high_avg,10))
                else:
                    record_array.append(round(low_avg,10))

            key_name = "close"
            Sig_keys = {}
            Sig_keys[key_name] = record_array
            key_name = "ori_close"
            Sig_keys[key_name] = tmp
            print("len(Sig_keys[key_name]): {0}".format(len(Sig_keys[key_name])))
            LCD_set("Sig len:"+str(len(Sig_keys[key_name])))
            OUT_FILE = "/home/pi/pi-voice-recognition/key_map2.json"
            src = open(OUT_FILE, 'w')
            src.write(json.dumps(Sig_keys))
            src.close()
        except:
            print("fail record")

    else:
        print("change mode to send2")

class WifireconnectThread(threading.Thread):
    def __init__(self, num):
        threading.Thread.__init__(self)
        self.num = num
    def run(self):
        while True:
            if '192' not in os.popen('ifconfig | grep 192').read():
                print('\n****** wifi is down, restart... ******\n')
                os.system('sudo /etc/init.d/networking restart')
            time.sleep(5*60) #5 minutes

class IRThread(threading.Thread):
    def __init__(self, num):
        threading.Thread.__init__(self)
        self.num = num

    def run(self):
        global button_pin_state
        global dynamic_static_record
        irtransmit_PIN = 5
        irreceive_PIN = 6
        state_PIN = 19
        send_PIN = 26
        initEnv(irtransmit_PIN, irreceive_PIN, state_PIN, send_PIN)
        p = GPIO.PWM(irtransmit_PIN, 38000)
        p.ChangeDutyCycle(0)
        p.start(0)
        send_pin_state = GPIO.input(send_PIN)
        if send_pin_state == 0:
            dynamic_static_record = False
        elif send_pin_state == 1:
            dynamic_static_record = True
        else:
            pass
        while True:
            time.sleep(1)
            if GPIO.input(state_PIN) == 1:
                button_pin_state = 1
                LCD_set("Receive IR")
                record(irreceive_PIN, state_PIN, send_PIN)
                SIGNAL_MAP = "/home/pi/pi-voice-recognition/key_map2.json"
                src = open(SIGNAL_MAP, 'r')
                signal_map = json.loads(src.read())
                src.close()
                time.sleep(1)
            else:
                button_pin_state = 0
                if send_pin_state != GPIO.input(send_PIN):
                    send_pin_state = GPIO.input(send_PIN)
                    button_pin_state = 1
                    time.sleep(1)
                    SIGNAL_MAP = "/home/pi/pi-voice-recognition/key_map2.json"
                    src = open(SIGNAL_MAP, 'r')
                    signal_map = json.loads(src.read())
                    src.close()
                    LCD_set("Send sig:"+str(len(signal_map["ori_close"])))
                    for i in range(1):
                        time.sleep(0.3)
                        count = 0
                        key_name = "ori_close"
                        for name in signal_map.keys():
                            if key_name == name:
                                high_low = True
                                start = time.time()
                                for t in signal_map[key_name]:
                                    if high_low == False:
                                        p.ChangeDutyCycle(0)
                                        high_low = True
                                    else:
                                        p.ChangeDutyCycle(33)
                                        high_low = False
                                    start = time.time()
                                    while (time.time() - start) < float(t):
                                        pass
                                    count = count + 1
                        print("send {0}th, count: {1}".format(i, count))
                        p.ChangeDutyCycle(0)
                    button_pin_state = 0
        p.stop()
        endEnv()

class LoRaRcvCont(LoRa):
    def __init__(self, verbose=False):
        super(LoRaRcvCont, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)

    def on_rx_done(self):
        global send_receive_flag
        global lora_receive_msg
        BOARD.led_on()
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)
        lora_receive_msg = ''.join([chr(c) for c in payload])
        print("Receive lora_receive_msg: {0}".format(lora_receive_msg))
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        BOARD.led_off()
        self.set_mode(MODE.RXCONT)

    def on_tx_done(self):
        global send_receive_flag
        global lora_send_msg
        if send_receive_flag == "send":
            send_msg = lora_send_msg
            send_receive_flag = "receive"
            self.set_mode(MODE.STDBY)
            self.clear_irq_flags(TxDone=1)
            sys.stdout.flush()
            self.tx_counter += 1
            BOARD.led_off()
            data = [int(hex(ord(c)), 0) for c in send_msg]
            #self.write_payload([0x0f])
            self.write_payload(data)
            BOARD.led_on()
            self.set_mode(MODE.TX)

    def on_cad_done(self):
        print("\non_CadDone")
        print(self.get_irq_flags())

    def on_rx_timeout(self):
        print("\non_RxTimeout")
        print(self.get_irq_flags())

    def on_valid_header(self):
        print("\non_ValidHeader")
        print(self.get_irq_flags())

    def on_payload_crc_error(self):
        print("\non_PayloadCrcError")
        print(self.get_irq_flags())

    def on_fhss_change_channel(self):
        print("\non_FhssChangeChannel")
        print(self.get_irq_flags())

    def start(self):
        global send_receive_flag
        global lora_send_msg

        send_receive_flag = "receive"
        while True:
            if send_receive_flag == "receive":
                self.set_mode(MODE.SLEEP)
                self.set_dio_mapping([0] * 6)

                self.reset_ptr_rx()
                self.set_mode(MODE.RXCONT)
                sleep(.5)
                rssi_value = self.get_rssi_value()
            else:
                self.set_mode(MODE.SLEEP)
                self.set_dio_mapping([1,0,0,0,0,0])
                self.tx_counter = 0
                BOARD.led_on()
                self.write_payload([0x0f])
                #self.write_payload([0x0f, 0x65, 0x6c, 0x70])
                self.set_mode(MODE.TX)

            '''
            status = self.get_modem_status()
            sys.stdout.flush()
            sys.stdout.write("\r%d %d %d" % (rssi_value, status['rx_ongoing'], status['modem_clear']))
            '''

class LoraThread(threading.Thread):
    def __init__(self, num):
        threading.Thread.__init__(self)
        self.num = num

    def run(self):
        BOARD.setup()
        parser = LoRaArgumentParser("Continous LoRa receiver.")

        lora = LoRaRcvCont(verbose=False)
        args = parser.parse_args(lora)

        lora.set_mode(MODE.STDBY)
        lora.set_pa_config(pa_select=1)
        lora.set_freq(434.0)
        #lora.set_rx_crc(True)
        #lora.set_coding_rate(CODING_RATE.CR4_6)
        #lora.set_pa_config(max_power=0, output_power=0)
        #lora.set_lna_gain(GAIN.G1)
        #lora.set_implicit_header_mode(False)
        #lora.set_low_data_rate_optim(True)
        #lora.set_pa_ramp(PA_RAMP.RAMP_50_us)
        #lora.set_agc_auto_on(True)

        print("lora: {0}".format(lora))
        assert(lora.get_agc_auto_on() == 1)
        time.sleep(1)
        try:
            lora.start()
        except KeyboardInterrupt:
            sys.stdout.flush()
            sys.stderr.write("KeyboardInterrupt\n")
        finally:
            sys.stdout.flush()
            lora.set_mode(MODE.SLEEP)
            print(lora)
            BOARD.teardown()

def callback(recognizer, audio):
    global get_voice
    global receive
    receive = ""
    # recognize speech using Google Speech Recognition
    try:
        #print("Google Speech Recognition thinks you said:")
        receive = str(recognizer.recognize_google(audio, language="zh-TW"))
        print("          Google Speech Recognition thinks you said:\r\n{0}".format(receive))
        get_voice = True
    except sr.UnknownValueError:
        print("          Google Speech Recognition could not understand audio", end='\r')
    except sr.RequestError as e:
        print("          No response from Google Speech Recognition service: {0}".format(e), end='\r')

class SoundtotextThread(threading.Thread):
    def __init__(self, num):
        threading.Thread.__init__(self)
        self.num = num

    def run(self):
        #obtain audio from the microphone
        global send_receive_flag
        global lora_send_msg
        global button_pin_state
        global get_voice
        global receive
        get_voice = False
        receive = ""
        ready_record = False
        r=sr.Recognizer()
        with sr.Microphone() as source:
            print("Please wait. Calibrating microphone...")
            #listen for 5 seconds and create the ambient noise energy level
            r.adjust_for_ambient_noise(source, duration=5)
        stop_listening = r.listen_in_background(sr.Microphone(), callback)
        ready_record = True
        seg_list = jieba.cut("初始化麥克風")
        receive_seg_list = ",".join(seg_list)
        print("seg_list: {0}".format(receive_seg_list))
        LCD_show("Dictionary Ready")

        while True:
            if button_pin_state == 1:
                stop_listening(wait_for_stop=False)
                print("stop recording")
                try:
                    mixer.music.stop()
                except:
                    pass
            else:
                if ready_record == False:
                    stop_listening(wait_for_stop=True)
                    print("Say somethings")
                    ready_record = True
            while (button_pin_state == 1):
                time.sleep(1)

            if get_voice == True:
                seg_list = jieba.cut(receive)
                receive_seg_list = ",".join(seg_list)
                print("seg_list: {0}".format(receive_seg_list))
                seg_str = receive_seg_list.split(",")
                light = 0
                on_off = 0
                send_content = ""
                for i in seg_str:
                    if i == "關燈":
                        print("關燈")
                        on_off = 0
                        for j in seg_str:
                            if j == "第一排" or j == "第一盞":
                                light = 1
                                break
                            elif j == "第二排" or j == "第二盞":
                                light = 2
                                break
                            elif j == "第三排" or j == "第三盞":
                                light = 3
                                break
                            elif (j == "全部") or (j == "所有"):
                                light = 0
                                break
                        send_content = "LightController-1 turn_off light " + str(light)
                        break
                    elif i == "開燈":
                        print("開燈")
                        on_off = 1
                        for j in seg_str:
                            if j == "第一排" or j == "第一盞":
                                light = 1
                                break
                            elif j == "第二排" or j == "第二盞":
                                light = 2
                                break
                            elif j == "第三排" or j == "第三盞":
                                light = 3
                                break
                            else:
                                light = 0
                        send_content = "LightController-1 turn_on light " + str(light)
                        break
                lora_send_msg = send_content
                if lora_send_msg != "":
                    input_text = 'Last_Call'
                    tts=gTTS(text=input_text, lang='zh-tw')
                    download_path = "/home/pi/" + input_text + ".mp3"
                    #tts.save(download_path)
                    mixer.init()
                    mixer.music.load(download_path)
                    mixer.music.play()
                    global lora_receive_msg
                    lora_receive_msg = ""
                    print("send: {0}".format(lora_send_msg))
                    send_receive_flag = "send"
                    tmp = datetime.now()
                    while lora_receive_msg == "":
                        if (datetime.now() - tmp)>5:
                            tmp = tmp = datetime.now()
                            print("send: {0}".format(lora_send_msg))
                            send_receive_flag = "send"
                    print("Receive lora respones: {0}".format())
                    print("The {0}th light is {1} (off/on)".format(light, on_off))
                get_voice = False
                ready_record = False

class People_node():
    def __init__(self, th, x, y):
        self.x = x
        self.y = y
        self.th = th
        self.id = None
        self.status = None

class node_diff():
    def __init__(self, dif, th_after, th_before):
        self.dif = dif
        self.th_after = th_after
        self.th_before = th_before

class ClientThread(threading.Thread):
    def __init__(self, num):
        threading.Thread.__init__(self)
        self.num = num
    def run(self):
        host = '140.116.39.249'
        port = 9999
        SIZE = 1024
        lastminute_new = datetime.now()
        global button_pin_state

        global horizontal_vertical
        global go_right, go_left, go_up, go_down
        global area_of_intrest_count
        global respones_people_count
        global people_go_dir
        global people_count_inside
        global round_record_time
        go_right = 0
        go_left = 0
        go_up = 0
        go_down = 0
        horizontal_count = 0
        vertical_count = 0
        people_count_inside = 0
        record_people_count = 0
        response_people_count = 0
        last_photo_time = 0
        center_node = []
        before_center_node = []
        round_record_time = time.localtime( time.time())
        device_number = 0
        device_path = "/home/pi/pi-face-recognition/device.json"
        if os.path.isfile(device_path):
            src = open(device_path, 'r')
            device_number = json.loads(src.read())
            src.close()
        else:
            device_number = 1
            src = open(device_path, 'w')
            src.write(json.dumps(device_number))
            src.close()
        Device = "Camera-" + str(device_number)

        while True:
            try:
                print("try to connect SQL server")
                db = MySQLdb.connect(host="140.116.39.249", user="rootroot", passwd="nckuesnclabdb", db="DAE")
                print("connect SQL server")
                cursor = db.cursor()
                db.ping(True)
                break
            except:
                print("fail to connect SQL server, try again")
        last_people_count_inside = [0, 0, 0]
        while True:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                while (button_pin_state == 1):
                    time.sleep(1)
                tmp_photo = os.listdir('/home/pi/pi-face-recognition/pi-face-recognition/tmp_photo/')
                if tmp_photo != None:
                    tmp_photo = sorted(tmp_photo)
                else:
                    tmp_photo = []
                time.sleep(1)
                print("          go_in: {0}, go_out: {1}, people_count: {2}".format(last_people_count_inside[0], last_people_count_inside[1], last_people_count_inside[2]), end='\r')
                if (len(tmp_photo) > 0):
                    for i in range(len(tmp_photo)):
                        photo_dir = '/home/pi/pi-face-recognition/pi-face-recognition/tmp_photo/' + str(tmp_photo[i])
                        img_name = Device + "_" + str(tmp_photo[i])
                        img_name_length = len(img_name)
                        f = open(photo_dir, 'rb')
                        data = f.read()
                        data_length = f.tell()
                        f.close()
                        s.settimeout(2)
                        s.connect((host, port))
                        pack_img_name_length = struct.pack('i', img_name_length)
                        form_1 = str(img_name_length) + 's'
                        pack_img_name = struct.pack(form_1, img_name.encode('utf-8'))
                        pack_data_length = struct.pack('i', data_length)
                        info = pack_img_name_length + pack_img_name + pack_data_length
                        #fhead = info + data
                        print('Client Ready to send:\r\n   img_name_length: {0}\r\n   img_name: {1}\r\n   data_length: {2}'.format(img_name_length, img_name, data_length))
                        send_msg_time = time.time()
                        #s.send(fhead)
                        s.send(info)
                        time.sleep(0.005)
                        if len(data) == 0:
                            os.remove(photo_dir)
                        if len(data) > 20000:
                            for i in range((len(data)//20000)):
                                s.send(data[20000*i:20000*(i+1)])
                                time.sleep(0.005)
                            s.send(data[(len(data)//20000)*20000:])
                        else:
                            s.send(data)
                        print('Client already send:\r\n   img_name_length: {0}\r\n   img_name: {1}\r\n   data_length: {2}'.format(img_name_length, img_name, data_length))
                        back = s.recv(SIZE)
                        response_time = time.time()-send_msg_time
                        print("Got Response: {0}, Response duration: {1}".format(back.decode(), response_time))
                        tmp = back.decode()
                        tmp = tmp.split(" ")
                        if len(tmp) == 1 or str(tmp[0]) == "fail":
                            if str(tmp[0]) == "fail":
                                print("fail send img_name: {0}".format(img_name))
                            else:
                                response_people_count = int(tmp[0])
                                people_count = str(tmp[0])
                                people_count = people_count + " People c"
                                print(people_count)
                                LCD_show(people_count)
                        else:
                            people_count = str(tmp[0])
                            center_point = str(tmp[1]).split(",")
                            global center_array
                            center_array = []
                            respones_people_count = int(people_count)
                            for i in range(int(people_count)):
                                center_array.append([int(center_point[2*i]), int(center_point[(2*i)+1])])
                            people_count = people_count + " People"
                            print(people_count)
                            print("center_array: {0}".format(center_array))
                            LCD_show(people_count)
                        s.close()

                        detect_frame = cv2.imread(photo_dir)
                        os.remove(photo_dir)
                        #--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                        if photo_dir[len(photo_dir)-9:] == "check.png":
                            tmp_photo = photo_dir.split("/")
                            tmp_photo = tmp_photo[len(tmp_photo) - 1]
                            tmp_photo_name = tmp_photo[:len(tmp_photo)-10]
                            print("tmp_photo_name: {0}".format(tmp_photo_name))
                            now_photo_time = datetime.strptime(tmp_photo_name, "%Y-%m-%d_%H:%M:%S")
                            if(last_photo_time == 0):
                                record_people_count = response_people_count
                            else:
                                if((now_photo_time-last_photo_time) > timedelta(seconds=60)):
                                    print("More than 60s: {0}".format((now_photo_time-last_photo_time)))
                                    record_people_count = response_people_count
                                elif((now_photo_time-last_photo_time) < timedelta(seconds=60)):
                                    print("Less than 60s: {0}".format((now_photo_time-last_photo_time)))
                                    record_people_count = record_people_count + response_people_count
                            last_photo_time = now_photo_time

                            save_dir = '/home/pi/pi-face-recognition/pi-face-recognition/tmp_photo/'
                            photo_dir = os.listdir(save_dir)
                            if(len(photo_dir) == 0):
                                '''
                                tmp = str(tmp_photo_name).split("_")
                                detect_time = tmp[0] + ' ' + tmp[1]
                                '''
                                detect_time = time.strftime('%Y-%m-%d %H:%M:%S', round_record_time)
                                command = "INSERT INTO Dream_classroom_check (Time, Device, People) VALUES ('{0}', '{1}', '{2}')".format(detect_time, Device, record_people_count)
                                print("\r\n\r\ncommand: {0}\r\n\r\n".format(command))
                                cursor.execute(command)
                                cursor.execute("commit")

                            print("\r\n check !!! , record_people_count, {0}, response_people_count: {1}, tmp_photo_name: {2}\r\n".format(record_people_count, response_people_count, tmp_photo_name))
                        else:
                            center_node = []
                            for i in range(len(center_array)):
                                center_node.append(People_node(i, center_array[i][0], center_array[i][1]))

                            center_node_dif = []
                            for i in range(len(center_node)):
                                for j in range(len(before_center_node)):
                                    dif = (center_node[i].x-before_center_node[j].x)**2 + (center_node[i].y-before_center_node[j].y)**2
                                    center_node_dif.append(node_diff(dif, center_node[i].th, before_center_node[j].th))
                            if len(center_node_dif)>0:
                                center_node_dif = sorted(center_node_dif, key=operator.attrgetter('dif'))
                            before_pair_bool = []
                            after_pair_bool = []
                            for i in range(len(before_center_node)):
                                before_pair_bool.append(False)
                            for i in range(len(center_node)):
                                after_pair_bool.append(False)
                            id_count = 0
                            pair_center_node = []
                            for i in range(len(center_node_dif)):
                                if ((before_pair_bool[center_node_dif[i].th_before] == False) and (after_pair_bool[center_node_dif[i].th_after] == False)):
                                    before_pair_bool[center_node_dif[i].th_before] = True
                                    after_pair_bool[center_node_dif[i].th_after] = True
                                    pair_center_node.append(center_node_dif[i])
                                    if before_center_node[center_node_dif[i].th_before].id != None:
                                        center_node[center_node_dif[i].th_after].id =  before_center_node[center_node_dif[i].th_before].id
                                    else:
                                        for i in range(len(before_center_node)):
                                            if before_center_node[i].id != None:
                                                if  before_center_node[i].id >= id_count:
                                                    id_count = before_center_node[i].id + 1
                                        center_node[center_node_dif[i].th_after].id =  id_count
                                    cv2.line(detect_frame, (center_node[center_node_dif[i].th_after].x, center_node[center_node_dif[i].th_after].y), (before_center_node[center_node_dif[i].th_before].x, before_center_node[center_node_dif[i].th_before].y), (0, 0, 0), 1)
                                    if center_node[center_node_dif[i].th_after].id >= id_count:
                                        id_count = center_node[center_node_dif[i].th_after].id + 1
                            for i in range(len(center_node)):
                                if (center_node[i].id == None):
                                    center_node[i].id = id_count
                                    id_count = id_count + 1
                            detect_going_path = "/home/pi/detect_going"
                            for i in range(len(pair_center_node)):
                                if horizontal_vertical == True:
                                    if (before_center_node[pair_center_node[i].th_before].x <= ((area_of_intrest[2]-area_of_intrest[0])//2)) and (center_node[pair_center_node[i].th_after].x > ((area_of_intrest[2]-area_of_intrest[0])//2)):
                                        center_node[pair_center_node[i].th_after].status = "go_right"
                                        if center_node[pair_center_node[i].th_after].id != None:
                                            tmp_text = str(center_node[pair_center_node[i].th_after].id) + "_R"
                                        else:
                                            tmp_text = "None_R"
                                        cv2.circle(detect_frame,(center_node[pair_center_node[i].th_after].x, center_node[pair_center_node[i].th_after].y), 3, (0,0,255), 2)
                                        cv2.putText(detect_frame, tmp_text, (center_node[pair_center_node[i].th_after].x, center_node[pair_center_node[i].th_after].y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
                                        go_right = go_right + 1

                                    elif (before_center_node[pair_center_node[i].th_before].x > ((area_of_intrest[2]-area_of_intrest[0])//2)) and (center_node[pair_center_node[i].th_after].x <= ((area_of_intrest[2]-area_of_intrest[0])//2)):
                                        center_node[pair_center_node[i].th_after].status = "go_left"
                                        if center_node[pair_center_node[i].th_after].id != None:
                                            tmp_text = str(center_node[pair_center_node[i].th_after].id) + "_L"
                                        else:
                                            tmp_text = "None_L"
                                        cv2.circle(detect_frame,(center_node[pair_center_node[i].th_after].x, center_node[pair_center_node[i].th_after].y), 3, (0,255,0), 2)
                                        cv2.putText(detect_frame, tmp_text, (center_node[pair_center_node[i].th_after].x, center_node[pair_center_node[i].th_after].y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                                        go_left = go_left + 1
                                    else:
                                        pass
                                else:
                                    if (before_center_node[pair_center_node[i].th_before].y <= ((area_of_intrest[3]-area_of_intrest[1])//2)) and (center_node[pair_center_node[i].th_after].y > ((area_of_intrest[3]-area_of_intrest[1])//2)):
                                        center_node[pair_center_node[i].th_after].status = "go_down"
                                        if center_node[pair_center_node[i].th_after].id != None:
                                            tmp_text = str(center_node[pair_center_node[i].th_after].id) + "_D"
                                        else:
                                            tmp_text = "None_D"
                                        cv2.circle(detect_frame,(center_node[pair_center_node[i].th_after].x, center_node[pair_center_node[i].th_after].y), 3, (0, 0, 255), 2)
                                        cv2.putText(detect_frame, tmp_text, (center_node[pair_center_node[i].th_after].x, center_node[pair_center_node[i].th_after].y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
                                        go_down = go_down + 1
                                    elif (before_center_node[pair_center_node[i].th_before].y > ((area_of_intrest[3]-area_of_intrest[1])//2)) and (center_node[pair_center_node[i].th_after].y <= ((area_of_intrest[3]-area_of_intrest[1])//2)):
                                        center_node[pair_center_node[i].th_after].status = "go_up"
                                        if center_node[pair_center_node[i].th_after].id != None:
                                            tmp_text = str(center_node[pair_center_node[i].th_after].id) + "_U"
                                        else:
                                            tmp_text = "None_U"
                                        cv2.circle(detect_frame,(center_node[pair_center_node[i].th_after].x, center_node[pair_center_node[i].th_after].y), 3, (0, 255, 0), 2)
                                        cv2.putText(detect_frame, tmp_text, (center_node[pair_center_node[i].th_after].x, center_node[pair_center_node[i].th_after].y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                                        go_up = go_up + 1
                                    else:
                                        pass

                            for i in range(len(center_node)):
                                cv2.circle(detect_frame,(center_node[i].x, center_node[i].y), 3, (0,255,255), 2)
                                cv2.putText(detect_frame, str(center_node[i].id), (center_node[i].x, center_node[i].y-3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1, cv2.LINE_AA)
                            if horizontal_vertical == True:
                                cv2.line(detect_frame, (((area_of_intrest[2]-area_of_intrest[0])//2),0), (((area_of_intrest[2]-area_of_intrest[0])//2),(area_of_intrest[3]-area_of_intrest[1])), (0,0,0), 2)
                            else:
                                cv2.line(detect_frame, (0,((area_of_intrest[3]-area_of_intrest[1])//2)), ((area_of_intrest[2]-area_of_intrest[0]),((area_of_intrest[3]-area_of_intrest[1])//2)), (0,0,0), 2)
                            if (len(center_node) > 0) or (len(before_center_node) > 0):
                                tmp = photo_dir.split("/")
                                save_name = tmp[len(tmp)-1]
                                save_path = str("/home/pi/detect_going/") + str(save_name)
                                cv2.imwrite(save_path, detect_frame)
                                print("Save save_path")
                                tmp = str(save_name).split('.')
                                tmp = tmp[0].split('_')
                                detect_time = tmp[0] + ' ' + tmp[1]
                            before_center_node = center_node
                            horizontal_count = go_right - go_left
                            vertical_count = go_up - go_down
                            go_in = 0
                            go_out = 0
                            if people_go_dir == "up_to_down":
                                go_in = go_down
                                go_out = go_up
                            elif people_go_dir == "down_to_up":
                                go_in = go_up
                                go_out = go_down
                            elif people_go_dir == "left_to_right":
                                go_in = go_right
                                go_out = go_left
                            elif people_go_dir == "right_to_left":
                                go_in = go_left
                                go_out = go_right
                            else:
                                pass
                            people_count_right_to_left = go_left - go_right
                            people_count_dif = go_in - go_out

                            device_number = 0
                            device_path = "/home/pi/pi-face-recognition/device.json"
                            if os.path.isfile(device_path):
                                src = open(device_path, 'r')
                                device_number = json.loads(src.read())
                                src.close()
                            else:
                                device_number = 1
                                src = open(device_path, 'w')
                                src.write(json.dumps(device_number))
                                src.close()
                            Device = "Camera-" + str(device_number)

                            print("    people_go_dir: {0}, (in, out, dif): ({1}, {2}, {3})".format(people_go_dir, go_in, go_out, people_count_dif))
                            if (last_people_count_inside[0] != go_in) or (last_people_count_inside[1] != go_out) or (last_people_count_inside[2] != people_count_dif):
                                print("last_people_count_inside change")
                                people_count_inside = people_count_inside + people_count_dif - last_people_count_inside[2]

                                last_people_count_inside[0] = go_in
                                last_people_count_inside[1] = go_out
                                last_people_count_inside[2] = people_count_dif
                                if detect_time[len(detect_time)-2] == "-":
                                    detect_time = detect_time[:len(detect_time)-2]
                                print("(in, out, count): ({0}, {1}, {2})".format(go_in, go_out, people_count_inside))
                                command = "INSERT INTO Dream_classroom (Time, Device, Go_In, Go_Out, People) VALUES ('{0}', '{1}', '{2}', '{3}', '{4}')".format(detect_time, Device, go_in, go_out, people_count_inside)
                                #command = "INSERT INTO nclab_inside (Time, Device, Go_In, Go_Out, People) VALUES ('{0}', '{1}', '{2}', '{3}', '{4}')".format(detect_time, Device, go_in, go_out, people_count_inside)
                                print("\r\n\r\ncommand: {0}\r\n\r\n".format(command))
                                cursor.execute(command)
                                cursor.execute("commit")
                            #print("          (r, l, u, d): ({0}, {1}, {2}, {3}), Count(h, v): ({4}, {5})".format(go_right, go_left, go_up, go_down, horizontal_count, vertical_count), end='\r')
                        #--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                else:
                    nowtime = datetime.now()
                    tmp = nowtime - lastminute_new
                    if ((nowtime - lastminute_new) > timedelta(minutes=1)):
                        device_number = 0
                        device_path = "/home/pi/pi-face-recognition/device.json"
                        if os.path.isfile(device_path):
                            src = open(device_path, 'r')
                            device_number = json.loads(src.read())
                            src.close()
                        else:
                            device_number = 1
                            src = open(device_path, 'w')
                            src.write(json.dumps(device_number))
                            src.close()
                        Device = "Camera-" + str(device_number)
                        lastminute_new = nowtime
                        send_time = Device + nowtime.strftime(' %Y-%m-%d %H:%M:%S')
                        time_length = len(send_time)
                        pack_time_length = struct.pack('i', time_length)
                        form_1 = str(time_length) + 's'
                        pack_time = struct.pack(form_1, send_time.encode('utf-8'))
                        data_length = 0
                        pack_data_length = struct.pack('i', data_length)
                        info = pack_time_length + pack_time + pack_data_length
                        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        s.settimeout(2)
                        s.connect((host, port))
                        s.send(info)
                        print("Client send_time: {0}, time_length: {1}".format(send_time, time_length))
                        s.close()
            except Exception as e:
                s.close()
                Exception_text = "          Exception: " + str(e) + " reconnecting...                 "

                sleeptime = random.uniform(1, 10)
                sleeptime = (round(sleeptime, 3))
                time.sleep(sleeptime)

                print(Exception_text, end='\r')

class ServerThread(threading.Thread):
    def __init__(self, num):
        threading.Thread.__init__(self)
        self.num = num
    def run(self):
            try:
            	while True:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    #s.bind(('192.168.11.4', 9999))
                    s.bind(('140.116.39.249', 9999))
                    s.listen(5)
                    s.settimeout(2)
                    sock, addr = s.accept()
                    save_path = ''

                    SIZE = 1024
                    receive = sock.recv(SIZE)
                    img_name_length = struct.unpack('i',receive[0:4])[0]
                    img_name = struct.unpack((str(img_name_length) + 's'),receive[4:4+img_name_length])[0].decode('utf-8')
                    print('Server img_name: {0}'.format(img_name))
                    data_length = struct.unpack('i',receive[4+img_name_length:8+img_name_length])[0]
                    if data_length > 0:
                    	count = len(receive[8+img_name_length:])
                    	data = receive[8+img_name_length:]
                    	while (count < data_length):
                    		data += sock.recv(SIZE)
                    		count = len(data)

                    	save_path = 'D:/nclab_photo/' + img_name
                    	#save_path = str(os.getcwd()) + '/' + img_name
                    	print("Server get save_path: {0}".format(save_path))
                    	if save_path != '':
                    		f = open(save_path, 'wb')
                    		f.write(data)
                    		print("Server write data")
                    		f.close()
                    else:
                    	print("receive: {0}, img_name_length: {1}, img_name: {2}, data_length: {3}".format(receive, img_name_length, img_name, data_length))
                    	print("time: {0}".format(img_name))

                    '''
                    count = len(receive[8+img_name_length:])
                    data = receive[8+img_name_length:]
                    while (count < data_length):
                    	data += sock.recv(SIZE)
                    	count = len(data)

                    save_path = 'D:/nclab_photo/' + img_name
                    #save_path = str(os.getcwd()) + '/' + img_name
                    print("Server get save_path: {0}".format(save_path))
                    if save_path != '':
                    	f = open(save_path, 'wb')
                    	f.write(data)
                    	print("Server write data")
                    	f.close()
                    '''
                    s.close()
            except:
            	print('reconnecting')

class OpencvThread(threading.Thread):
    def __init__(self, num):
        threading.Thread.__init__(self)
        self.num = num

    def run(self):
        global send_receive_flag
        global lora_send_msg
        global lora_receive_msg
        global button_pin_state
        global dynamic_static_record
        global area_of_intrest
        global center_array
        global line_of_split
        global area_of_intrest_count
        global horizontal_vertical
        global go_right, go_left, go_up, go_down
        global respones_people_count
        global stop_frame
        global people_go_dir
        global round_record_time
        stop_frame = []
        respones_people_count = -1
        go_right = 0
        go_left = 0
        go_up = 0
        go_down = 0
        center_node = []
        before_center_node = []
        center_array = []
        send_PIN = 26
        send_pin_state = GPIO.input(send_PIN)
        if send_pin_state == 0:
            dynamic_static_record = False
        elif send_pin_state == 1:
            dynamic_static_record = True
        else:
            pass
        CONTROL_PIN = 21
        TRIGER_PIN = 0
        TRIGER_VOLTAGE = 0
        PWM_FREQ = 50
        SERVO_DEG = 0

        MotorType_path = "/home/pi/pi-face-recognition/MotorType.json"
        record_MotorType = []
        src = open(MotorType_path, 'r')
        try:
            record_MotorType = json.loads(src.read())
            print("\r\nrecord_deg: {0}\r\n".format(record_MotorType))
        except:
            print("Fail to open degree.json")
        src.close()
        if(len(record_MotorType) > 0):
            print("Motor　Type: {0}".format(record_MotorType[0]))
        else:
            record_MotorType = [360, 0]
            src = open(MotorType_path, 'w')
            src.write(json.dumps(record_MotorType))
            src.close()

        GPIO.setup(CONTROL_PIN, GPIO.OUT)
        pwm = GPIO.PWM(CONTROL_PIN, PWM_FREQ)
        pwm.start(0)
        TRIGER_PIN = 20
        if (record_MotorType[0] == 180):
            deg_pwm = angle_to_duty_cycle(90)
            pwm.ChangeDutyCycle(deg_pwm)
            SERVO_DEG = 90
            print("Turn to deg: {0}".format(90))
            time.sleep(2)
            pwm.ChangeDutyCycle(0)
            deg_pwm = angle_to_duty_cycle(0)
            pwm.ChangeDutyCycle(deg_pwm)
            SERVO_DEG = 0
            print("Turn to deg: {0}".format(0))
            time.sleep(2)
            pwm.ChangeDutyCycle(0)
            time.sleep(2)
        elif(record_MotorType[0] == 360):
            if(len(record_MotorType) > 1):
                SERVO_DEG = record_MotorType[1]
            else:
                record_MotorType = [360, 0]
                src = open(MotorType_path, 'w')
                src.write(json.dumps(record_MotorType))
                src.close()
                SERVO_DEG  = 0
            GPIO.setup(TRIGER_PIN, GPIO.IN)
            TRIGER_VOLTAGE = GPIO.input(TRIGER_PIN)
            print("TRIGER_VOLTAGE: {0}".format(TRIGER_VOLTAGE))
            pass
        else:
            print("MotorType is unknow")
        no_one_time = 60
        play_music_time = 30
        if_msg_send = False
        start_play_time = 0
        last_send_time = 0
        already_close = False
        input_text = 'Last_Call'
        download_path = "/home/pi/" + input_text + ".mp3"
        mixer.init()
        mixer.music.load(download_path)
        camera_resolution = [480, 320]
        area_of_intrest = [0, 0, 480, 320]
        line_of_split =  [240, 0, 240, 320, 0]

        try:
            area_of_intrest_path = "/home/pi/pi-face-recognition/area_of_intrest.json"
            src = open(area_of_intrest_path, 'r')
            area_of_intrest = json.loads(src.read())
            src.close()
        except:
            area_of_intrest = [0, 0, 480, 320]
        #area_of_intrest_count = [((area_of_intrest[2]-area_of_intrest[0])//3), ((area_of_intrest[3]-area_of_intrest[1])//3), 2*((area_of_intrest[2]-area_of_intrest[0])//3), 2*((area_of_intrest[3]-area_of_intrest[1])//3)]

        try:
            line_of_split_path = "/home/pi/pi-face-recognition/line_of_split.json"
            src = open(line_of_split_path, 'r')
            line_of_split = json.loads(src.read())
            src.close()
        except:
            line_of_split = [240, 0, 240, 320, 0]
        if line_of_split[4] >= 45:
            horizontal_vertical = False
            if line_of_split[3] >= line_of_split[1]:
                people_go_dir = "up_to_down"
            else:
                people_go_dir = "down_to_up"
        else:
            horizontal_vertical = True
            if line_of_split[2] >= line_of_split[0]:
                people_go_dir = "left_to_right"
            else:
                people_go_dir = "right_to_left"

        if horizontal_vertical == True:
            area_of_intrest_count = [((area_of_intrest[2]-area_of_intrest[0])//3), 0, 2*((area_of_intrest[2]-area_of_intrest[0])//3), (area_of_intrest[3]-area_of_intrest[1])]
        else:
            area_of_intrest_count = [0, ((area_of_intrest[3]-area_of_intrest[1])//3), (area_of_intrest[2]-area_of_intrest[0]), 2*((area_of_intrest[3]-area_of_intrest[1])//3)]

        # initialize the video stream and allow the camera sensor to warm up
        print("[INFO] starting video stream...")

        camera = PiCamera()
        camera.resolution = (camera_resolution[0], camera_resolution[1])
        camera.framerate = 40
        rawCapture = PiRGBArray(camera, size=(camera_resolution[0], camera_resolution[1]))
        stream = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)

        kernel_5 = cv2.getStructuringElement(cv2.MORPH_RECT,(9, 9))
        kernel_13 = cv2.getStructuringElement(cv2.MORPH_RECT,(13, 13))

        frame = []
        time.sleep(2.0)
        for (i, f) in enumerate(stream):
            frame = f.array
            rawCapture.truncate(0)
            break
        frame = frame[area_of_intrest[1]:area_of_intrest[3], area_of_intrest[0]:area_of_intrest[2]]

        time.sleep(2)
        print("save the first frame")
        gray_before = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # loop over frames from the video file stream
        no_people_detect = 0
        people_detect = time.time()
        fps_sec = time.time()
        global fps
        motormoving = False
        now_position = 0
        step_degree = 22.5
        rotate_deg = []
        get_photo_flag = 0
        round_record_time = time.time()

        while True:
                stream = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)
                print("i should not here")
                for (i, f) in enumerate(stream):
                    # grab the frame from the threaded video stream and resize it
                    # to 500px (to speedup processing)
                    if button_pin_state == 1:
                        while True:
                            time.sleep(1)
                            if button_pin_state == 0:
                                print("restart camera")
                                break
                    #frame = vs.read()
                    tmp = time.time()
                    fps = round((1/(tmp - fps_sec)), 2)
                    fps_sec = tmp
                    print("fps: {0}".format(fps),end='\r')
                    frame = f.array
                    rawCapture.truncate(0)
                    frame = frame[area_of_intrest[1]:area_of_intrest[3], area_of_intrest[0]:area_of_intrest[2]]
                    #cv2.imshow("frame_cutting", frame)
                    # convert the input frame from (1) BGR to grayscale (for face
                    # detection) and (2) from BGR to RGB (for face recognition)
                    #rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    gray_after = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    if (len(gray_after[0]) != len(gray_before[0])) or (len(gray_after[1]) != len(gray_before[1])):
                        gray_before = gray_after
                    try:
                        gray = gray_after - gray_before
                    except:
                        gray = gray_after - gray_after
                    gray_before = gray_after
                    gray = cv2.erode(gray,kernel_5)
                    #cv2.imshow('kernel_5',gray)
                    gray = cv2.dilate(gray,kernel_13)
                    #cv2.imshow('kernel_13',gray)
                    gray[gray < 10] = 0
                    area = 0
                    key = cv2.waitKey(1) & 0xFF
                    tmp = np.mat(gray)
                    nonzero_gray = tmp.nonzero()
                    if len(nonzero_gray[0]) > 0:
                        area = 1
                    else:
                        area = 0

                    if area != 0:
                        print("Someone here")
                        LCD_show("Someone here", str(time.strftime("%H:%M:%S")) + "FPS" + str(fps))
                        if dynamic_static_record == True:
                            #save_dir = str(os.getcwd()) + '/tmp_photo/'
                            save_dir = '/home/pi/pi-face-recognition/pi-face-recognition/tmp_photo/'
                            save_name = time.strftime('%Y-%m-%d_%H:%M:%S', time.localtime( time.time()))
                            photo_dir = os.listdir(save_dir)
                            same_file_count = 0
                            for i in range(len(photo_dir)):
                                if photo_dir[i][:len(save_name)] == save_name:
                                    same_file_count = same_file_count + 1
                            save_path = save_dir + str(save_name) + "-" + str(same_file_count) + ".png"
                            cv2.imwrite(save_path, frame)
                            key = cv2.waitKey(1) & 0xFF
                            print("imwrite: {0}".format(save_path))

                        start_play_time = 0
                        people_detect = time.time()
                        already_close = False
                        if mixer.music.get_busy() == 1:
                            mixer.music.fadeout(1000)
                    else:
                        LCD_show("No one here", str(time.strftime("%H:%M:%S")) + "FPS" + str(fps))
                        no_people_detect = time.time() - people_detect

                    if already_close == True:
                        LCD_show("Turn off", str(time.strftime("%H:%M:%S")) + "FPS" + str(fps))
                        no_people_detect = 0
                    '''
                    if (no_people_detect > no_one_time) and (already_close == False):
                        no_people_detect = 0
                        #print("Play music")
                        LCD_show("Play music", str(time.strftime("%H:%M:%S")) + "FPS" + str(fps))
                        if start_play_time == 0:
                            mixer.music.play()
                            start_play_time = time.time()
                            if dynamic_static_record == False:
                                pass
                            respones_people_count = -1
                            save_name = time.strftime('%Y-%m-%d_%H:%M:%S', time.localtime( time.time())) + '-check.png'
                            save_path = str(os.getcwd()) + '/tmp_photo/' + str(save_name)
                            check_frame = f.array
                            rawCapture.truncate(0)
                            cv2.imwrite(save_path, check_frame)
                    '''
                    if (time.time() - start_play_time > play_music_time) and (start_play_time != 0):
                        if mixer.music.get_busy() == 1:
                            mixer.music.fadeout(3000)
                        if (respones_people_count == 0):
                            respones_people_count = -1
                            start_play_time = 0
                            if if_msg_send == False:
                                lora_receive_msg = ""
                                lora_send_msg = "LightController-1 turn_off light 0"
                                send_receive_flag = "send"
                                last_send_time = time.time()
                                print("lora send: {0}".format(lora_send_msg))
                                people_detect = time.time()
                                if_msg_send = True
                    if if_msg_send == True:
                        if lora_receive_msg == "":
                            lora_send_msg = "LightController-1 turn_off light 0"
                            if ((time.time() - last_send_time) > 60) and (last_send_time != 0):
                                last_send_time = time.time()
                                send_receive_flag = "send"
                                print("lora send: {0}".format(lora_send_msg))
                            #---------------------------------------------------------------------------------------------------------------------------------------
                            already_close = True
                            #---------------------------------------------------------------------------------------------------------------------------------------
                        else:
                            if_msg_send = False
                            last_send_time = 0
                            already_close = True
                            print("Receive lora: {0}".format(lora_receive_msg))
                    cv2.line(frame, (line_of_split[0],line_of_split[1]), (line_of_split[2],line_of_split[3]), (255,255,255), 2)
                    if horizontal_vertical == True:
                        cv2.line(frame, (((area_of_intrest[2]-area_of_intrest[0])//2),0), (((area_of_intrest[2]-area_of_intrest[0])//2),(area_of_intrest[3]-area_of_intrest[1])), (0,0,0), 2)
                    else:
                        cv2.line(frame, (0,((area_of_intrest[3]-area_of_intrest[1])//2)), ((area_of_intrest[2]-area_of_intrest[0]),((area_of_intrest[3]-area_of_intrest[1])//2)), (0,0,0), 2)
                    cv2.imshow("Frame", frame)
                    rotate_time = time.localtime( time.time())
                    rotate_time_m_s = time.strftime('%M:%S', rotate_time)
                    compare_rotate_time = rotate_time_m_s.split(":")
                    if(compare_rotate_time[1] == "00"):
                        if((compare_rotate_time[0] == "00") or (compare_rotate_time[0] == "15") or (compare_rotate_time[0] == "30") or (compare_rotate_time[0] == "45")):
                            rotate_deg = servo_ready_to_record(record_MotorType[0], SERVO_DEG, pwm, TRIGER_PIN)
                            round_record_time = rotate_time
                            if(record_MotorType[0] == 360):
                                SERVO_DEG = 0

                    if(get_photo_flag > 0):
                        get_photo_flag = 0
                        print("Ready to save specify degree photo")
                        save_dir = '/home/pi/pi-face-recognition/pi-face-recognition/rotate_photo/'
                        save_name = time.strftime('%Y-%m-%d_%H:%M:%S', time.localtime( time.time()))
                        save_path = save_dir + str(save_name) +  '-check.png'
                        cv2.imwrite(save_path, frame)
                        if(len(rotate_deg) == 0):
                            rotate_photo_dir = '/home/pi/pi-face-recognition/pi-face-recognition/rotate_photo/'
                            send_photo_dir = '/home/pi/pi-face-recognition/pi-face-recognition/tmp_photo/'
                            rotate_photo_file = os.listdir(rotate_photo_dir)
                            if(len(rotate_photo_file) > 0):
                                for photo_name in rotate_photo_file:
                                    sour_dir = rotate_photo_dir + photo_name
                                    dest_dir = send_photo_dir + photo_name
                                    os.replace(sour_dir, dest_dir)

                    if(len(rotate_deg) > 0):                                    #   need to rotate and get photo
                        stream.close()
                        if (record_MotorType[0] == 180):
                            time.sleep(2)
                            SERVO_DEG = servo_rotate(rotate_deg[0], SERVO_DEG, pwm)
                        elif (record_MotorType[0] == 360):
                            if((SERVO_DEG+90) <= record_MotorType[0]):
                                while(GPIO.input(TRIGER_PIN) != 1):
                                    pwm.ChangeDutyCycle(7.5)
                                    time.sleep(0.08)
                                    pwm.ChangeDutyCycle(7.25)
                                    time.sleep(0.1)
                                pwm.ChangeDutyCycle(7.25)
                                time.sleep(0.1)
                                pwm.ChangeDutyCycle(0)
                                time.sleep(0.5)
                                while(GPIO.input(TRIGER_PIN) != 0):
                                    pwm.ChangeDutyCycle(7.5)
                                    time.sleep(0.08)
                                    pwm.ChangeDutyCycle(7.25)
                                    time.sleep(0.1)
                                pwm.ChangeDutyCycle(7.5)
                                time.sleep(0.08)
                                pwm.ChangeDutyCycle(7.25)
                                time.sleep(0.1)
                                pwm.ChangeDutyCycle(0)
                                time.sleep(2)
                                SERVO_DEG += 90
                                record_MotorType = [360, SERVO_DEG]
                                src = open(MotorType_path, 'w')
                                src.write(json.dumps(record_MotorType))
                                src.close()
                        else:
                            print("MotorType is unknow")
                        if(len(rotate_deg) > 1):
                            rotate_deg = rotate_deg[1:]
                        else:
                            rotate_deg = []
                        get_photo_flag = 1
                        break

                    key = cv2.waitKey(1) & 0xFF
                    i, o, e = select.select( [sys.stdin], [], [], 0.01 )
                    if (i):
                        keys = sys.stdin.readline().strip()
                        print("keyboad: {0}".format(keys))
                        if keys == "C":
                            global frame_cut
                            frame_cut = f.array
                            rawCapture.truncate(0)
                            stop_frame = frame_cut
                            cv2.imshow('frame_to_cut',frame_cut)
                            k=cv2.waitKey(1)
                            cv2.setMouseCallback('frame_to_cut',draw_rectangle)
                            while(1):
                                cv2.imshow('frame_to_cut',frame_cut)
                                frame_cut = stop_frame
                                k=cv2.waitKey(1)
                                if k==ord('q'):
                                    cv2.destroyAllWindows()
                                    break
                        elif keys == "clear":
                            print("clear right left up down")
                            go_right = 0
                            go_left = 0
                            go_up = 0
                            go_down = 0
                            horizontal_count = 0
                            vertical_count = 0
                        elif keys == "set people":
                            global people_count_inside
                            while(1):
                                if (i):
                                    keys = sys.stdin.readline().strip()
                                    print("\r\n\r\nSet people_count_inside: {0}\r\n\r\n".format(keys))
                                    people_count_inside = int(keys)
                                    break
                        elif keys[:10] == "set device":
                            device_path = "/home/pi/pi-face-recognition/device.json"
                            device_number = int(keys[10:])
                            src = open(device_path, 'w')
                            src.write(json.dumps(device_number))
                            src.close()

                        elif keys[:10] == "set degree":
                            if (record_MotorType[0] == 180):
                                stream.close()
                                time.sleep(2)
                                deg = int(keys[10:])
                                SERVO_DEG = servo_rotate(deg, SERVO_DEG, pwm)
                            else:
                                print("MotorType not 180")
                        elif keys[:10] == "now degree":
                            if (record_MotorType[0] == 180):
                                print("\r\nNow degree: {0}\r\n".format(SERVO_DEG))
                            else:
                                print("\r\nNow degree: {0}\r\n".format(SERVO_DEG))
                        elif keys[:10] == "add degree":
                            if(record_MotorType[0] == 180):
                                print("Add degree: {0}".format(SERVO_DEG))
                                degree_path = "/home/pi/pi-face-recognition/degree.json"
                                record_deg = []
                                src = open(degree_path, 'r')
                                try:
                                    record_deg = json.loads(src.read())
                                    print("\r\nrecord_deg: {0}\r\n".format(record_deg))
                                except:
                                    print("Fail to open degree.json")
                                src.close()
                                if(len(record_deg) > 0):
                                    record_deg.append(SERVO_DEG)
                                    record_deg.sort()
                                else:
                                    record_deg.append(SERVO_DEG)
                                src = open(degree_path, 'w')
                                src.write(json.dumps(record_deg))
                                src.close()
                            else:
                                print("MotorType not 180")
                        elif keys[:11] == "read degree":
                            if(record_MotorType[0] == 180):
                                degree_path = "/home/pi/pi-face-recognition/degree.json"
                                src = open(degree_path, 'r')
                                try:
                                    record_deg = json.loads(src.read())
                                    print("\r\nRecord degree number: {0}, degree: {1}\r\n".format(len(record_deg), record_deg))
                                except Exception as e:
                                    print(e)
                                    print("Fail to open degree.json")
                                src.close()
                            else:
                                print("MotorType not 180")
                        elif keys[:14] == "remove degree:":
                            if(record_MotorType[0] == 180):
                                degree_path = "/home/pi/pi-face-recognition/degree.json"
                                if(keys[14:] != "all"):
                                    remove_deg = int(keys[14:])
                                    json_deg = []
                                    src = open(degree_path, 'r')
                                    try:
                                        record_deg = json.loads(src.read())
                                        for i in record_deg:
                                            if (int(i) != remove_deg):
                                                json_deg.append(i)
                                    except:
                                        print("Fail to open degree.json")
                                    src.close()
                                    if(len(json_deg) > 0):
                                        src = open(degree_path, 'w')
                                        src.write(json.dumps(json_deg))
                                        src.close()
                                        print("\r\nRecord degree number: {0}, degree: {1}\r\n".format(len(record_deg), record_deg))
                                    else:
                                        print("degree.json without data")
                                else:
                                    os.remove(degree_path)
                                    src = open(degree_path,'w')
                                    src.close()
                                    print("Remove: {0}".format(degree_path))
                            else:
                                print("MotorType not 180")
                        elif keys[:12] == "round record":
                            round_record_time = time.localtime( time.time())
                            rotate_deg = servo_ready_to_record(record_MotorType[0], SERVO_DEG, pwm, TRIGER_PIN)
                            if(record_MotorType[0] == 360):
                                SERVO_DEG = 0
                        elif keys[:11] == "read triger":
                            if(record_MotorType[0] == 360):
                                TRIGER_VOLTAGE = GPIO.input(TRIGER_PIN)
                                print("\r\nTRIGER_VOLTAGE: {0}\r\n".format(TRIGER_VOLTAGE))
                            else:
                                print("MotorType not 360")
                        elif keys[:13] == "turn right":
                            if((SERVO_DEG+90) <= 360):
                                print("Ready turn right")
                                while(GPIO.input(TRIGER_PIN) != 1):
                                    pwm.ChangeDutyCycle(7.5)
                                    time.sleep(0.1)
                                    pwm.ChangeDutyCycle(7.25)
                                    time.sleep(0.1)
                                pwm.ChangeDutyCycle(7.25)
                                time.sleep(0.1)
                                pwm.ChangeDutyCycle(0)
                                time.sleep(0.5)
                                while(GPIO.input(TRIGER_PIN) != 0):
                                    pwm.ChangeDutyCycle(7.5)
                                    time.sleep(0.1)
                                    pwm.ChangeDutyCycle(7.25)
                                    time.sleep(0.1)
                                pwm.ChangeDutyCycle(7.5)
                                time.sleep(0.1)
                                pwm.ChangeDutyCycle(7.25)
                                time.sleep(0.1)
                                pwm.ChangeDutyCycle(0)
                                time.sleep(0.5)
                                SERVO_DEG += 90
                                print("Turn right, now degree: {0}".format(SERVO_DEG))
                            else:
                                print("SERVO_DEG+90 = {0}, out of 360".format(SERVO_DEG+90))

                        elif keys[:12] == "turn left":
                            if((SERVO_DEG-90) >= 0):
                                print("Ready turn left")
                                while(GPIO.input(TRIGER_PIN) != 1):
                                    pwm.ChangeDutyCycle(6.5)
                                    time.sleep(0.1)
                                    pwm.ChangeDutyCycle(7.25)
                                    time.sleep(0.1)
                                pwm.ChangeDutyCycle(7.25)
                                time.sleep(0.1)
                                pwm.ChangeDutyCycle(0)
                                time.sleep(0.5)
                                while(GPIO.input(TRIGER_PIN) != 0):
                                    pwm.ChangeDutyCycle(6.5)
                                    time.sleep(0.1)
                                    pwm.ChangeDutyCycle(7.25)
                                    time.sleep(0.1)
                                pwm.ChangeDutyCycle(6.5)
                                time.sleep(0.1)
                                pwm.ChangeDutyCycle(7.25)
                                time.sleep(0.1)
                                pwm.ChangeDutyCycle(0)
                                time.sleep(0.5)
                                SERVO_DEG -= 90
                                print("Turn right, now degree: {0}".format(SERVO_DEG))
                            else:
                                print("SERVO_DEG-90 = {0}, out of 0".format(SERVO_DEG-90))
                        elif keys[:15] == "set dae command":
                            from rs485 import RS485_Conversation
                            command = []
                            command_str = input("Enter your input: ")
                            tmp = command_str.split(" ")
                            for i in range(len(tmp)):
                                command.append(int(tmp[i]))
                            print("command: {0}".format(command))
                            receive_data = RS485_Conversation(command[0], command[1:])
                            print("receive_data: {0}".format(receive_data))
                            time.sleep(0.5)

        # do a bit of cleanup
        cv2.waitKeycv2.destroyAllWindows()

def draw_rectangle(event,x,y,flags,param):
    global area_of_intrest,drawing
    global frame_cut
    global line_of_split
    global area_of_intrest_count
    global horizontal_vertical
    global stop_frame
    global people_go_dir
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        area_of_intrest[0] = x
        area_of_intrest[1] = y
        print("area_of_intrest\r\nstart :[x, y]: [{0}, {1}]".format(area_of_intrest[0], area_of_intrest[1]))
    elif event == cv2.EVENT_MOUSEMOVE and flags==cv2.EVENT_FLAG_LBUTTON:
        if drawing == True:
            cv2.rectangle(frame_cut,(area_of_intrest[0],area_of_intrest[1]),(x,y),(0,0,0),2)
            k=cv2.waitKey(1)

    elif event == cv2.EVENT_LBUTTONUP:
        area_of_intrest[2] = x
        area_of_intrest[3] = y
        print("stop  :[x, y]: [{0}, {1}]".format(area_of_intrest[2], area_of_intrest[3]))
        if area_of_intrest[0] > area_of_intrest[2]:
            tmp = area_of_intrest[2]
            area_of_intrest[2] = area_of_intrest[0]
            area_of_intrest[0] = tmp
        if area_of_intrest[1] > area_of_intrest[3]:
            tmp = area_of_intrest[3]
            area_of_intrest[3] = area_of_intrest[1]
            area_of_intrest[1] = tmp
        OUT_FILE = "/home/pi/pi-face-recognition/area_of_intrest.json"
        src = open(OUT_FILE, 'w')
        src.write(json.dumps(area_of_intrest))
        src.close()
        if horizontal_vertical == True:
            area_of_intrest_count = [((area_of_intrest[2]-area_of_intrest[0])//3), 0, 2*((area_of_intrest[2]-area_of_intrest[0])//3), (area_of_intrest[3]-area_of_intrest[1])]
        else:
            area_of_intrest_count = [0, ((area_of_intrest[3]-area_of_intrest[1])//3), (area_of_intrest[2]-area_of_intrest[0]), 2*((area_of_intrest[3]-area_of_intrest[1])//3)]

        drawing = False
    elif event == cv2.EVENT_RBUTTONDOWN:
        drawing = True
        line_of_split[0] = x
        line_of_split[1] = y
        print("line_of_split:\r\nstart :[x, y]: [{0}, {1}]".format(line_of_split[0], line_of_split[1]))
    elif event == cv2.EVENT_MOUSEMOVE and flags==cv2.EVENT_FLAG_RBUTTON:
        if drawing == True:
            cv2.line(frame_cut, (line_of_split[0],line_of_split[1]), (x,y), (255,255,255), 3)
            k=cv2.waitKey(1)
    elif event == cv2.EVENT_RBUTTONUP:
        line_of_split[2] = x
        line_of_split[3] = y
        print("stop  :[x, y]: [{0}, {1}]".format(line_of_split[2], line_of_split[3]))
        y_dif = abs(line_of_split[3] - line_of_split[1])
        x_dif = abs(line_of_split[2] - line_of_split[0])
        tri_dif = (y_dif*y_dif + x_dif*x_dif)** 0.5
        cosine = abs(x_dif)/abs(tri_dif)
        deg = degrees(acos(cosine))
        people_go_dir = ""
        if deg >= 45:
            horizontal_vertical = False
            if line_of_split[3] >= line_of_split[1]:
                people_go_dir = "up_to_down"
            else:
                people_go_dir = "down_to_up"
        else:
            horizontal_vertical = True
            if line_of_split[2] >= line_of_split[0]:
                people_go_dir = "left_to_right"
            else:
                people_go_dir = "right_to_left"
        line_of_split[4] = deg
        print("deg: {0}".format(deg))
        line_of_split[0] = line_of_split[0] - area_of_intrest[0]
        line_of_split[1] = line_of_split[1] - area_of_intrest[1]
        line_of_split[2] = line_of_split[2] - area_of_intrest[0]
        line_of_split[3] = line_of_split[3] - area_of_intrest[1]
        OUT_FILE = "/home/pi/pi-face-recognition/line_of_split.json"
        src = open(OUT_FILE, 'w')
        src.write(json.dumps(line_of_split))
        src.close()
        drawing = False
        if horizontal_vertical == True:
            area_of_intrest_count = [((area_of_intrest[2]-area_of_intrest[0])//3), 0, 2*((area_of_intrest[2]-area_of_intrest[0])//3), (area_of_intrest[3]-area_of_intrest[1])]
        else:
            area_of_intrest_count = [0, ((area_of_intrest[3]-area_of_intrest[1])//3), (area_of_intrest[2]-area_of_intrest[0]), 2*((area_of_intrest[3]-area_of_intrest[1])//3)]

def LCD_show(firstline = "", secondline = ""):
    global lcd_tmp_firstline
    global lcd_tmp_secondline
    lcd_tmp_firstline = firstline
    lcd_tmp_secondline = secondline

def LCD_init():
    global lcd
    global lcd_busy
    global last_firstline
    global last_secondline
    last_firstline = ""
    last_secondline = ""
    lcd_busy = False
    lcd = CharLCD('PCF8574', address=0x27, port=1, backlight_enabled=False)

def LCD_reset():
    global lcd
    global lcd_busy
    while lcd_busy == True:
        pass
    lcd_busy = True
    lcd.clear()
    lcd.cursor_pos = (0, 0)
    lcd.write_string("Date: {}".format(time.strftime("%Y/%m/%d")))
    lcd.cursor_pos = (1, 0)
    lcd.write_string(str(time.strftime("%H:%M:%S")))
    lcd_busy = False

def LCD_set(firstline = "", secondline = ""):
    global lcd
    global lcd_busy
    while lcd_busy == True:
        pass
    global last_firstline
    global last_secondline

    null_srt = "                "
    lcd_busy = True
    #lcd.clear()
    if (last_firstline != firstline) and (firstline != ""):
        start_show = 0
        len_firstline = len(firstline)
        len_last_firstline = len(last_firstline)
        if len_firstline < len_last_firstline:
            for i in range(len_firstline):
                if firstline[i] != last_firstline[i]:
                    start_show = i
                    break
        else:
            for i in range(len_last_firstline):
                if firstline[i] != last_firstline[i]:
                    start_show = i
                    break
        if start_show >= 15:
            start_show = 0
        last_firstline = firstline
        show_content = firstline
        if len(show_content) < 16:
            for i in range(16 - len(show_content)):
                show_content = show_content + " "
        lcd.cursor_pos = (0, start_show)
        lcd.write_string(show_content[start_show:])

    if (last_secondline != secondline) and (secondline !=""):
        start_show = 0
        len_secondline = len(secondline)
        len_last_secondline = len(last_secondline)
        if len_secondline < len_last_secondline:
            for i in range(len_secondline):
                if secondline[i] != last_secondline[i]:
                    start_show = i
                    break
        else:
            for i in range(len_last_secondline):
                if secondline[i] != last_secondline[i]:
                    start_show = i
                    break
        if start_show >= 15:
            start_show = 0
        last_secondline = secondline
        show_content = secondline
        if len(show_content) < 15:
            for i in range(15 - len(show_content)):
                show_content = show_content + " "
        if dynamic_static_record == True:
            show_content = show_content + 'D'
        else:
            show_content = show_content + 'S'
        lcd.cursor_pos = (1, start_show)
        lcd.write_string(show_content[start_show:])

    lcd_busy = False

class LCDThread(threading.Thread):
    def __init__(self, num):
        threading.Thread.__init__(self)
        self.num = num

    def run(self):
        global lcd_tmp_firstline
        global lcd_tmp_secondline
        global button_pin_state
        lcd_tmp_firstline = ""
        lcd_tmp_secondline = ""
        while True:
            time.sleep(0.5)
            LCD_set(lcd_tmp_firstline, lcd_tmp_secondline)
            if button_pin_state == 1:
                while True:
                    time.sleep(1)
                    if button_pin_state == 0:
                        print("restart LCD")
                        break

def main():
    ps_a = subprocess.check_output("ps -a", shell=True)
    ps_a = str(ps_a).strip()
    ps_a_split = ps_a.split()
    python_count = 0
    for i in range(len(ps_a_split)):
        tmp = ps_a_split[i].strip("'")
        ps_a_split[i] = tmp.strip("\\n")
        if ps_a_split[i] == "python3":
            python_count = python_count + 1
            if python_count > 1:
                kill_python_cmd = "sudo kill " + str(ps_a_split[i-3])
                return_status = subprocess.call(kill_python_cmd, shell=True)
                if return_status == 0:
                    print("kill python3" + str(ps_a_split[i-3]))
    time.sleep(1)

    LCD_init()
    LCD_reset()
    # construct the argument parser and parse the arguments
    global button_pin_state
    button_pin_state = 1
    LoraThread(0).start()
    time.sleep(0.5)
    IRThread(1).start()
    #SoundtotextThread(2).start()
    while button_pin_state == 1:
        time.sleep(1)
        pass
    OpencvThread(3).start()
    ClientThread(4).start()
    LCDThread(5).start()
    #WifireconnectThread(6).start()
    while True:
        if '192' not in os.popen('ifconfig | grep 192').read():
            print('\n****** wifi is down, restart... ******\n')
            os.system('sudo /etc/init.d/networking restart')
        time.sleep(5*60) #5 minutes
    '''
    while True:
        time.sleep(1)
        pass
    '''

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
        print("Fail in main()")
    finally:
        GPIO.cleanup()
