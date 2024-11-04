import serial
import time
import sys
from UART import *
from dataSave import *
import RPi.GPIO as GPIO

def master():
    ser = init_uart(port='/dev/ttyS0', baudrate=115200, timeout=1)
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

if __name__ == '__main__':
    ser = master()
    GPIOSetup()
    
    while True:
        response = ser.readline()
        if(b'RCV' in response):
            packet = response.split(",")
            data = packet[2].split(",")
            lat = data[2] + data[3]
            lng = data[4] + data[5]
            
            RSSI = int(packet[3])
            SNR = int(packet[4])
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
            conv(lng, lat, strength)
        elif(GPIO.input(5) and not GPIO.input(6)):
            GPIOReset()
            print("1")
        elif(GPIO.input(6) and not GPIO.input(5)):
            GPIOReset()
            print("2")
        elif(GPIO.input(5) and GPIO.input(6)):
            GPIOReset()
            print("3")
    
    sys.exit(master(sys.argv))
