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
#stop gpio 14 and 15 from being changed away from uart pins
subprocess.run(['raspi-gpio', 'set', '14', 'a0'])
subprocess.run(['raspi-gpio', 'set', '15', 'a0'])
 
DEBUGMODE = True;
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


#pulse reset pin
GPIO.setup(25, GPIO.OUT)
GPIO.output(25, GPIO.LOW)
time.sleep(0.1)
GPIO.output(25, GPIO.HIGH)

# RFID Reader
reader = SimpleMFRC522();

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


finger = None
uart = None
fail_count = 0   #failure counter

##prevent multiple threads from accessing the sensor at once
sensor_lock = threading.Lock()




#debounce stuffs
def read_debounced(pin, stable_ms=100):
    """Return stable HIGH/LOW only after it stays the same for stable_ms."""
    initial = GPIO.input(pin)
    time.sleep(stable_ms / 1000.0)
    return initial if GPIO.input(pin) == initial else None


def init_fingerprint():
    global finger, uart
    try:
        if uart and uart.is_open:   # <--- safer close
            uart.close()
        uart = serial.Serial("/dev/serial0", baudrate=57600, timeout=1)
        finger = adafruit_fingerprint.Adafruit_Fingerprint(uart)
        logging.info("[OK] Fingerprint sensor initialized")
        return True
    except Exception as e:
        logging.error(f"[ERROR] Failed to init fingerprint: {e}")
        return False


# === Fingerprint Listener Thread ===

def fingerprint_listener():
    global finger, uart, fail_count

    while True:
        
        try:
            with sensor_lock:
                if finger is None:
                    logging.warning("[WARN] Fingerprint not initialized, retrying...")
                    if init_fingerprint():
                        fail_count = 0
                    time.sleep(2)
                    continue

                # Try capture and match
                if finger.get_image() == adafruit_fingerprint.OK:
                    if finger.image_2_tz(1) == adafruit_fingerprint.OK:
                        if finger.finger_search() == adafruit_fingerprint.OK:
                            logging.info(f"[INFO] Fingerprint recognized: ID #{finger.finger_id}")
                            try:
                                idx = valid_fingers.index(finger.finger_id)
                                if DEBUGMODE: print("Authorized finger:", finger_names[idx])
                                logging.info(f"[INFO] Authorized! Door unlocked to {id_names[finger.finger_id]}")
                                unlockServo()
                            except ValueError:
                                logging.warning(f"[WARN] Unauthorized fingerprint ID {finger.finger_id}")
                        else:
                            if DEBUGMODE: print("Fingerprint not recognized")
                            logging.warning(f"[WARN] Fingerprint not recognized")
                    else:
                        if DEBUGMODE: print("Failed to convert fingerprint image")

        except Exception as e:
            fail_count += 1
            logging.error(f"[ERROR] Fingerprint error: {e} (fail #{fail_count})")

            try:
                if uart and uart.is_open:
                    uart.close()
            except Exception as ce:
                logging.error(f"[CLOSE ERROR] {ce}")

            time.sleep(2)

            if init_fingerprint():
                fail_count = 0
            elif fail_count > 5:
                logging.error("[HALT] Too many failures, waiting 30s before retry")
                time.sleep(30)
                fail_count = 0


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
        
        
        
        
        
# Global lock for MFRC522 reset
rfid_lock = threading.Lock()
def reset_mfrc522():
    global reader
    with rfid_lock:
        try:
            del reader
        except NameError:
            pass
        GPIO.output(25, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(25, GPIO.HIGH)
        time.sleep(0.1)
        reader = SimpleMFRC522()
        if DEBUGMODE:
            print("MFRC522 reset successfully")
            
            
def unlockServo():
    global doorUnlockedState, unlockGraceActive
    if unlockGraceActive:  # already running, ignore
        return
    
    
    unlockGraceActive = True;
    
    
    GPIO.output(RELAY_PIN, GPIO.HIGH)   # Turn relay ON (activate)
    
    GPIO.output(DOOR_PIN, GPIO.HIGH)
    pwm.ChangeDutyCycle(DUTY_OPEN)
    doorUnlockedState = True;
    #let servo unlock
    time.sleep(3)
    GPIO.output(RELAY_PIN, GPIO.LOW)    # turn relay OFF (saves servo)
    unlockGraceActive = False;
    
    # Reset MFRC522 safely after unlocking
    reset_mfrc522()
    time.sleep(3) # extra 3 second grace on unlock to prevent issues

def lockServo():
    global doorUnlockedState, unlockGraceActive

    unlockGraceActive = True
    GPIO.output(RELAY_PIN, GPIO.HIGH)

    start_time = time.time()
    LOCK_TIME = 3.0   # seconds total lock attempt

    pwm.ChangeDutyCycle(DUTY_CLOSED)

    while time.time() - start_time < LOCK_TIME:
        if is_door_open():
            if DEBUGMODE:
                print("⚠ Door opened while locking — aborting lock")
            logging.warning("[WARNING] Door opened during lock, re-unlocking")

            GPIO.output(RELAY_PIN, GPIO.LOW)
            unlockGraceActive = False
            unlockServo()
            return

        time.sleep(0.05)

    GPIO.output(RELAY_PIN, GPIO.LOW)
    doorUnlockedState = False
    unlockGraceActive = False

    
 

#door state to track if door is open or not
    #closed by default
doorState = False;

lastDoorState = False


doorUnlockedState = False;

unlockGraceActive = False;



def mag_switch_thread():
    global doorState, lastDoorState, doorUnlockedState
    while True:
        raw = GPIO.input(MAGSWITCH_PIN)

        # --- INSTANT OPEN DETECTION ---
        if raw == GPIO.LOW:   # door open (switch open)
            if lastDoorState != raw:
                doorState = True
                doorUnlockedState = True
                lastDoorState = raw
                if DEBUGMODE: print("Door Open (instant)")
                logging.info("[INFO] Door Open (instant)")
            time.sleep(0.01)
            continue

        # --- DEBOUNCED CLOSE DETECTION ---
        # raw == HIGH, so door might be closed — confirm stability
        stable = read_debounced(MAGSWITCH_PIN, stable_ms=80)
        if stable == GPIO.HIGH and lastDoorState != GPIO.HIGH:
            time.sleep(0.05) # smol delay before lock
            doorState = False
            lastDoorState = GPIO.HIGH
            if DEBUGMODE: print("Door Closed (debounced)")
            logging.info("[INFO] Door Closed (debounced)")

        time.sleep(0.01)



def button_pressed(pin, hold_ms=200):
    if GPIO.input(pin) == GPIO.HIGH:
        time.sleep(hold_ms / 1000)
        return GPIO.input(pin) == GPIO.HIGH
    return False


def is_door_open():
    return GPIO.input(MAGSWITCH_PIN) == GPIO.LOW



if __name__ == "__main__":
    ##start async thread
    # === Start Fingerprint Thread ===
    listener_thread = threading.Thread(target=fingerprint_listener, daemon=True)
    listener_thread.start()
    #scan magswitch
    mag_thread = threading.Thread(target=mag_switch_thread, daemon=True)
    mag_thread.start()
    
    
    #INIT=================================================
    # Initial door state from magswitch
    initial_state = GPIO.input(MAGSWITCH_PIN)
    if initial_state:  # HIGH = closed
        doorState = False
        lastDoorState = True
    else:  # LOW = open
        doorState = True
        lastDoorState = False

    try:
        
        while True:
            
            if unlockGraceActive:
                time.sleep(0.1) # dont run code if unlockGrace is active
                continue
                
            #print("Button Pin State: ", GPIO.input(BUTTON_PIN));
            #Internal unlock Button logic
   
            if(button_pressed(BUTTON_PIN, 200)): #HIGH when pressed down
                if(DEBUGMODE): print("Door unlocked from inside")
                logging.info(f"[INFO] Door Unlocked from inside")
                unlockServo();
            
            
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
            # door closing logic (only lock when door just closed)
            if doorState == False and doorUnlockedState == True and unlockGraceActive == False:
                # door just transitioned from open -> closed
                lockServo()
                if(DEBUGMODE): print("Door closed, locking...")
                logging.info("[INFO] Door closed, locking")
                

    except KeyboardInterrupt:
        logging.info("RFID Program interrupted by user.")
    except Exception as e:
        logging.error(f"[ERROR] Unhandled error: {e}")
        if(DEBUGMODE): print("Unhandled error occurred:", e)
    finally:
        pwm.stop()
        GPIO.cleanup()
        if(DEBUGMODE): print('GPIO cleaned up.')

