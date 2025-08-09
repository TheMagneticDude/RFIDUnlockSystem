import time
import sys
import logging
import RPi.GPIO as GPIO
sys.path.append("/home/themagneticdude/MFRC522-python")
from mfrc522 import SimpleMFRC522


import serial
import adafruit_fingerprint
import threading

import os
#//reset gpio 14 and 15 to serial data

import subprocess
subprocess.run(['raspi-gpio', 'set', '14', 'alt0'])
subprocess.run(['raspi-gpio', 'set', '15', 'alt0'])

DEBUGMODE = True;
# Constants
#PINS
DOOR_PIN = 17
INVALID_PIN = 18
SERVO_PIN = 19

#SERVO
PWM_FREQ = 350
DUTY_CLOSED = 15
DUTY_OPEN = 45

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(DOOR_PIN, GPIO.OUT)
GPIO.setup(INVALID_PIN, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)
GPIO.output(DOOR_PIN, GPIO.LOW)
GPIO.output(INVALID_PIN, GPIO.LOW)

# Setup Servo PWM
pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
pwm.start(DUTY_CLOSED)  # Default closed position

# RFID Reader
reader = SimpleMFRC522()

# Valid IDs
valid_keys = [584188916640, 700944024593, 466974685233]
id_names = ['Nathan Cheng BuckID', 'DevCard', 'DevTag']

#Valid IDs for fingerprint
valid_fingers = [0];
finger_names = ["Nathan Cheng Pointer Finger"]

# Logging
logging.basicConfig(
    filename='rfid_log.txt',
    level=logging.INFO,
    format='%(asctime)s - %(message)s'
)
logging.info("RFID Program started")

last_read_time = 0



##prevent multiple threads from accessing the sensor at once
sensor_lock = threading.Lock()


# Open UART serial port
uart = serial.Serial("/dev/serial0", baudrate=57600, timeout=1)

# Initialize the fingerprint sensor
finger = adafruit_fingerprint.Adafruit_Fingerprint(uart)


def fingerprint_listener():
    while True:
        try:
            with sensor_lock:
                if finger.get_image() == adafruit_fingerprint.OK:
                    if finger.image_2_tz(1) != adafruit_fingerprint.OK:
                        if(DEBUGMODE): print("Invalid fingerprint: failed to convert image");
                        continue
                    if finger.finger_search() != adafruit_fingerprint.OK:
                        if(DEBUGMODE): print("Fingerprint not recognized");
                        continue
                    if(DEBUGMODE): print(f"\n Fingerprint recognized: ID #{finger.finger_id} (confidence {finger.confidence})");
                    
                    #//Set flag for fingerprint recognized + id +
                    try:
                        index = valid_fingers.index(finger.finger_id)
                        if(DEBUGMODE): print("Authorized! Unlocking door...")
                        if(DEBUGMODE): print("Welcome", finger_names[index])
                        logging.info(f"[INFO] Authorized! Door unlocked to {finger_names[index]}")
                        #unlock servo
                        unlockServo();
                    except ValueError:
                        if DEBUGMODE:
                            print(f"Fingerprint ID {finger.finger_id} not authorized")
                        continue
                    
            time.sleep(1)  # prevent rapid repeat
        except RuntimeError as e:
            if(DEBUGMODE): print("Fingerprint error:", e)
            time.sleep(1)


def get_fingerprint():
    with sensor_lock:
        if(DEBUGMODE): print("Waiting for finger...")
        if finger.get_image() == adafruit_fingerprint.OK:
            if finger.image_2_tz(1) != adafruit_fingerprint.OK:
                return False
            if finger.finger_search() != adafruit_fingerprint.OK:
                return False
            if(DEBUGMODE): print("Found ID #", finger.finger_id, "with confidence", finger.confidence)
            return True
        else:
            return False
def unlockServo():
    GPIO.output(DOOR_PIN, GPIO.HIGH)
    pwm.ChangeDutyCycle(DUTY_OPEN)
    time.sleep(3)
    GPIO.output(DOOR_PIN, GPIO.LOW)
    pwm.ChangeDutyCycle(DUTY_CLOSED)


#stop gpio warnings
GPIO.setwarnings(False);

if __name__ == "__main__":
    ##start async thread
    listener_thread = threading.Thread(target=fingerprint_listener, daemon=True)
    listener_thread.start()
    try:
        while True:
            try:
                card_id = reader.read_id_no_block()
                if card_id and time.time() - last_read_time > 1:
                    if(DEBUGMODE): print(f"Tag detected: {card_id}")
                    last_read_time = time.time()

                    if card_id in valid_keys:
                        index = valid_keys.index(card_id)
                        if(DEBUGMODE): print("Authorized! Unlocking door...")
                        if(DEBUGMODE): print("Welcome", id_names[index])
                        logging.info(f"[INFO] Authorized! Door unlocked to {id_names[index]}")

                        unlockServo();

                    else:
                        logging.info(f"[WARNING] Unauthorized card id: {card_id}")
                        if(DEBUGMODE): print("? Unauthorized card")
                        GPIO.output(INVALID_PIN, GPIO.HIGH)
                        time.sleep(3)
                        GPIO.output(INVALID_PIN, GPIO.LOW)
            except Exception as e:
                logging.error(f"[ERROR] RFID read error: {e}")
                if(DEBUGMODE): print("Skipping read error:", e)

            time.sleep(0.2)

    except KeyboardInterrupt:
        logging.info("RFID Program interrupted by user.")
    except Exception as e:
        logging.error(f"[ERROR] Unhandled error: {e}")
        if(DEBUGMODE): print("Unhandled error occurred:", e)
    finally:
        pwm.stop()
        GPIO.cleanup()
        if(DEBUGMODE): print('GPIO cleaned up.')

