import time
import sys
import logging
import RPi.GPIO as GPIO
sys.path.append("/home/themagneticdude/MFRC522-python")
from mfrc522 import SimpleMFRC522

DEBUGMODE = True;

# Constants
DOOR_PIN = 17
INVALID_PIN = 18
SERVO_PIN = 19
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

# Logging
logging.basicConfig(
    filename='rfid_log.txt',
    level=logging.INFO,
    format='%(asctime)s - %(message)s'
)
logging.info("RFID Program started")

last_read_time = 0

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

                    GPIO.output(DOOR_PIN, GPIO.HIGH)
                    pwm.ChangeDutyCycle(DUTY_OPEN)
                    time.sleep(3)
                    GPIO.output(DOOR_PIN, GPIO.LOW)
                    pwm.ChangeDutyCycle(DUTY_CLOSED)

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

