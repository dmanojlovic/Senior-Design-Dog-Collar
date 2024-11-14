import sys
import RPi.GPIO as GPIO

if __name__ == "__main__":
    try:
        button = sys.argv[1]
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(6,GPIO.OUT)
        GPIO.setup(5,GPIO.OUT)
        GPIO.output(5,0)
        GPIO.output(6,0)
        if(button == "Buzzer"):
            GPIO.output(5,1)
        elif(button == "Stop"):
            GPIO.output(6,1)
        elif(button == "Return"):
            GPIO.output(5,1)
            GPIO.output(6,1)
    except:
        print("Failed")
        pass
    sys.stdout.flush()
    sys.exit()
