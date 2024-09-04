#do pip install pyserial
#Make sure GPIO14(TXD) and GPIO15(RXD) are connected to the LoRaRX and LoraTX respectively
#Make sure ground is connected to ground

import serial
import time

#Initialize the UART on Raspberry Pi
def init_uart(port='/dev/ttyS0', baudrate=9600, timeout=1): #Might have to alter port???????????????????????????????????????????
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        if ser.is_open:
            print(f"connected to {port} at {baudrate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None
    
#Send at command to LoRa module
def send_at_command(ser, command):
    if ser:
        ser.write((command + '\r\n').encode('utf-8'))  # Send the command followed by a new line
        #ser.write(b'AT\r\n')
        #print(f"{command}+'\r\n')
        #print(ser.write((command + '\r\n').encode()))
        #ser.inWaiting()
        ##time.sleep(.5)  # Wait for the LoRa chip to process
        #response = ser.read().decode()  # Read all available response
        response = ser.readline()
        #response = ser.read(2)  # Read all available response
        print(f"Sent: {command}, Received: {response}")
    else:
        print("Serial port not initialized.")
        return None
    
#Close UART connection
def close_uart(ser):
    if ser and ser.is_open:
        ser.close()
        print("UART closed.")
    else:
        print("UART not initialized or already closed.")

if __name__ == '__main__':
    ser = init_uart(port='/dev/ttyS0', baudrate=115200, timeout=1)
    if ser:
        print("Hello")
        send_at_command(ser, 'AT')
        send_at_command(ser, 'AT+CRFOP?')  # Example to configure UART settings
        send_at_command(ser, 'AT+ADDRESS=123')  # Example to set LoRa address
        send_at_command(ser, 'AT+NETWORKID=7')  # Example to set LoRa network ID
        send_at_command(ser, 'AT+MODE=1')  # Example to set LoRa mode
        close_uart(ser)
       
#NOTES 
'''
We will have to alter the send_at_command function to send the correct AT commands to the LoRa module. 
We will also have to find the port we are on and alter that. I put in a placeholder port for now

'''
