# to be called manually from pi console using alias to unlock the door for 7 seconds
import time
import sys
import logging
import RPi.GPIO as GPIO

# Constants
#PINS
DOOR_PIN = 17
INVALID_PIN = 18
SERVO_PIN = 19
RELAY_PIN = 21
MAGSWITCH_PIN = 13;
BUTTON_PIN = 6;

#SERVO
PWM_FREQ = 350
DUTY_CLOSED = 50 # 50
DUTY_OPEN = 80 # 10


# Setup GPIO
GPIO.setmode(GPIO.BCM)
#stop gpio warnings
GPIO.setwarnings(False);

GPIO.setup(DOOR_PIN, GPIO.OUT)
GPIO.setup(INVALID_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.LOW)
GPIO.output(DOOR_PIN, GPIO.LOW)
GPIO.output(INVALID_PIN, GPIO.LOW)

#magswitch
#GPIO.setup(MAGSWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)  # external 10k to GND
GPIO.setup(MAGSWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # use internal pull down too
#internal unlock button
#GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)  # using external 10k to GND
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # use internal pull down too


# Setup Servo PWM
pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
pwm.start(DUTY_CLOSED)  # Default closed position

# Logging
logging.basicConfig(
    filename='rfid_log.txt',
    level=logging.INFO,
    format='%(asctime)s - %(message)s'
)



        
def unlockServo():    
    
    GPIO.output(RELAY_PIN, GPIO.HIGH)   # Turn relay ON (activate)
    GPIO.output(DOOR_PIN, GPIO.HIGH) # led
    pwm.ChangeDutyCycle(DUTY_OPEN) # open
    #let servo unlock
    logging.info("[INFO] Door Unlocked");
    time.sleep(3)
    GPIO.output(RELAY_PIN, GPIO.LOW)    # turn relay OFF (saves servo)
    time.sleep(7) # extra 7 second grace on unlock to prevent issues


def lockServo():
    if GPIO.input(MAGSWITCH_PIN) == GPIO.LOW: # door opened
        logging.info("[WARN] Door is open, prevented lock");
        return
    GPIO.output(RELAY_PIN, GPIO.HIGH) # Turn relay ON
    pwm.ChangeDutyCycle(DUTY_CLOSED) # close
    logging.info("[INFO] Door Locked");
    time.sleep(3) # pause 3 seconds
    GPIO.output(RELAY_PIN, GPIO.LOW)

    
if __name__ == "__main__":
    
    logging.info("[OVERRIDE] Manual override: Unlocking");
    unlockServo()
    pwm.stop()
    GPIO.cleanup()
