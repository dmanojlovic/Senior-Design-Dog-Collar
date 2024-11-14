import serial
import time
import sys
from UART import *
from dataSave import *
from conversion import *
import RPi.GPIO as GPIO

def master():
    ser = init_uart(port='/dev/ttyS0', baudrate=115200, timeout=.1)
    if ser:
        send_at_command(ser, 'AT+ADDRESS=123')  # Example to set LoRa address
        send_at_command(ser, 'AT+NETWORKID=7')  # Example to set LoRa network ID
        send_at_command(ser, 'AT+CPIN=102C064CA409E69030F73E7CABAA4B71')
    return ser

def GPIOSetup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(5,GPIO.OUT)
    GPIO.output(5,0)
    GPIO.setup(6,GPIO.OUT)
    GPIO.output(6,0)
    
def sendCommand(channel):
    if(GPIO.input(6) & GPIO.input(5)):
        print("3")
        GPIOReset()
    elif(GPIO.input(6) & ~GPIO.input(5)):
        print("2")
        GPIOReset()
    elif(~GPIO.input(6) & GPIO.input(5)):
        print("1")
        GPIOReset()

def GPIOReset():
    GPIO.output(5,0)
    GPIO.output(6,0)
    
def playAudio(audio:str):
    start = time.time()
    send_at_command(ser, 'AT+SEND=124,1,' + audio)
    while True:
        if((time.time() - start) > 3):
            send_at_command(ser, 'AT+SEND=124,1,' + audio)
            start = time.time()
        response = ser.readline()
        if(b'Received' in response):
            break
    
    print("Data sent sucessfully") 

def borderCheck(lat, lng):
    data = {}
    with open('public/geofence.json', 'r') as file:
        data = json.load(file)
    if lng > data["west"] and lng < data["east"] and lat > data["south"] and lat < data["north"]:
        pass
    else:
        playAudio("B")

if __name__ == '__main__':
    ser = master()
    GPIOSetup()
    
    while True:
        send = "0"
        response = ""
        response = ser.readline()
        if(b'RCV' in response):
            print(response)
            try:
                packet = str(response).split(",")
                data = packet[2:-2]
                lat = data[2] + data[3]
                lng = data[4] + data[5]
                lng, lat = NMEAConv(lng, lat)
                RSSI = int(packet[-2])
                SNR = packet[-1]
                SNR = int(SNR[:-5])
                
                strengthNum = 0
                if(RSSI < -115 and RSSI >= -126):
                    strengthNum += 1
                elif(RSSI < -126):
                    strengthNum += 2
                if(SNR < -7 and SNR >= -15):
                    strengthNum += 1
                elif(SNR < -15):
                    strengthNum += 2
                
                match strengthNum:
                    case 0:
                        strength = "Good"
                    case 1:
                        strength = "Fair"
                    case 2:
                        strength = "Poor"
                    case 3:
                        strength = "Poor"
                    case _:
                        strength = "Bad"
                print("saving")
                conv(lng, lat, strength)
                print("Checking bounds")
                borderCheck(lat,lng)
            except:
                print("Improper data")
        elif(GPIO.input(5) and not GPIO.input(6)):
            GPIOReset()
            playAudio("A")
        elif(GPIO.input(6) and not GPIO.input(5)):
            GPIOReset()
            playAudio("B")
        elif(GPIO.input(5) and GPIO.input(6)):
            GPIOReset()
            playAudio("C")
        sys.stdout.flush()
    sys.exit(master(sys.argv))
