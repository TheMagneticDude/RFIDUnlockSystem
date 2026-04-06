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

import discord
from discord.ext import commands, tasks
from discord import app_commands

#stop gpio 14 and 15 from being changed away from uart pins
subprocess.run(['raspi-gpio', 'set', '14', 'a0'])
subprocess.run(['raspi-gpio', 'set', '15', 'a0'])
 
DEBUGMODE = False;
RFIDREADER = False;
FINGERPRINT = False;
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
id_names = ['Nathan BuckID', 'DevCard', 'DevTag'] 

#Valid IDs for fingerprint
valid_fingers = [0,2,3,4,5,6,7,8,9,10,12,11];
finger_names = ["Nathan Left Pointer Finger", "Nathan Right Pointer Finger", "Ronak Right Pointer Finger", "Ronak Left Pointer Finger", "Jacob Right Pointer Finger", "Jacob Right Thumb","Ronak Right Thumb", "Jaylen Right Pointer Finger", "Jaylen Right Thumb", "Nathan Right Thumb","Nathan Left Thumb","Ronak Left Thum"]

# Logging
logging.basicConfig(
    filename='rfid_log.txt',
    level=logging.INFO,
    format='%(asctime)s - %(message)s'
)
logging.info("RFID Program started")




#========================= Discord Bot Integration =========================

#========================= Constants =========================
#will fill in after to avoid secret being in repo
botToken = '';
#grab token from local env variable
botToken = os.getenv("DISCORD_TOKEN");

GUILD_ID = discord.Object(id=1490450047085838446);

#========================= Global States =========================
embed_door_open = False
door_message = None # discord message
embed_unlocked = False;

#door state to track if door is open or not
    #closed by default
doorState = False;
#door unlocked hardware state
doorUnlockedState = False;


embedColor = discord.Color.red();
embedTitle = "Door State: Locked 🔒"
#discord.Color.green();
#"Door State: Unlocked 🔓"


def build_door_embed():
    global embed_unlocked, embed_door_open
    
    lock_state = "Unlocked 🔓" if embed_unlocked else "Locked 🔒"
    door_state = "Open 🚪" if embed_door_open else "Closed 🚪"
    
    color = discord.Color.green() if embed_unlocked else discord.Color.red()
    
    embed = discord.Embed(
        title=f"Door is {lock_state}",
        description=f"Physical Door State: **{door_state}**",
        color=color
    )
    return embed

#========================= Class =========================
class Client(commands.Bot):
    async def on_ready(self):
        print(f'Logged on as {self.user}!');
        try:
            synced = await self.tree.sync(guild=GUILD_ID)
            print(f"Synced {len(synced)} command(s) to guild.")
        except Exception as e:
            print(e)
        
        # START THE LOOP HERE
        if not self.hardware_monitor.is_running():
            self.hardware_monitor.start()
        
    async def on_message(self, message):
        if message.author == self.user:
            return

        if message.content.startswith('!sesame'):
            await message.channel.send(
                f'Command Recieved: {message.author} executed: {message.content}'
            )

        await self.process_commands(message)
        
        
        
        
    #async background thread to update door message
    @tasks.loop(seconds=1) # updates every 1 second
    async def hardware_monitor(self):
        global door_message, embed_door_open, embed_unlocked
        if door_message is None:
                    return # door command has not been run yet
        if doorState != embed_door_open or doorUnlockedState != embed_unlocked:
            embed_door_open = doorState;
            embed_unlocked = doorUnlockedState;
            #update variables if any of them are out of sync
            
            try:
                # Edit the saved message directly using the Discord API
                await door_message.edit(embed=build_door_embed(), view=ViewButton())
                print("Discord message dynamically updated from hardware state!")
            except discord.NotFound:
                # The message was deleted in Discord
                door_message = None





#========================= UI Components =========================
class ViewButton(discord.ui.View):
    def __init__(self):
        super().__init__(timeout=None) # Prevents the button from expiring
        
    @discord.ui.button(label="Unlock", style=discord.ButtonStyle.primary, emoji="🧲")
    async def button_callback(self, interaction: discord.Interaction, button: discord.ui.Button):
        global embed_unlocked
        #send door command on button press
        #run unlock servo in separate thread
        threading.Thread(target=unlockServo, daemon=True).start()
            
        
        #update message
        await interaction.response.edit_message(embed=build_door_embed(), view=self);
        
        
 #========================= Init =========================
intents = discord.Intents.default();
intents.message_content = True;
client = Client(command_prefix = "!sesame", intents = intents);

#========================= Commands =========================
@client.tree.command(name="test", description="test", guild = GUILD_ID)
async def test(interaction: discord.Interaction):
    await interaction.response.send_message("Test");
    
@client.tree.command(name="door", description="Controls Door", guild = GUILD_ID)
async def doorCommand(interaction: discord.Interaction):
    global door_message
    await interaction.response.send_message(embed = build_door_embed(), view=ViewButton());
    door_message = await interaction.original_response()










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
    if(FINGERPRINT): # only run thread if fingerprint is enabled
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
                                    logging.info(f"[INFO] Authorized! Door unlocked to {finger_names[idx]}")
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
    time.sleep(7) # extra 7 second grace on unlock to prevent issues

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
            time.sleep(3) # pause 3 seconds
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
            time.sleep(2) # smol delay before lock
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



def hardware_main_loop():
    # Initial door state from magswitch
    initial_state = GPIO.input(MAGSWITCH_PIN)
    if initial_state:  # HIGH = closed
        global doorState, lastDoorState, doorUnlockedState
        doorState = False
        lastDoorState = True
        lockServo()
        logging.info("[INFO] Init door closed, locking")
        doorUnlockedState = False
    else:  # LOW = open
        doorState = True
        lastDoorState = False
        doorUnlockedState = True # assume unlocked if door open

    try:
        while True:
            if unlockGraceActive:
                time.sleep(0.1)
                continue
                
            if button_pressed(BUTTON_PIN, 200):
                if DEBUGMODE: print("Door unlocked from inside")
                logging.info(f"[INFO] Door Unlocked from inside")
                unlockServo()
            
            try:
                if RFIDREADER:
                    global last_read_time
                    card_id = reader.read_id_no_block()
                    if card_id and time.time() - last_read_time > 1:
                        if DEBUGMODE: print(f"Tag detected: {card_id}")
                        last_read_time = time.time()

                        if card_id in valid_keys:
                            index = valid_keys.index(card_id)
                            if DEBUGMODE: print("Authorized! Unlocking door...")
                            logging.info(f"[INFO] Authorized! Door unlocked to {id_names[index]}")
                            unlockServo()
                        else:
                            logging.info(f"[WARNING] Unauthorized card id: {card_id}")
                            GPIO.output(INVALID_PIN, GPIO.HIGH)
                            time.sleep(3)
                            GPIO.output(INVALID_PIN, GPIO.LOW)
            except Exception as e:
                logging.error(f"[ERROR] RFID read error: {e}")

            time.sleep(0.2)
            
            # door closing logic
            if doorState == False and doorUnlockedState == True and unlockGraceActive == False:
                lockServo()
                if DEBUGMODE: print("Door closed, locking...")
                logging.info("[INFO] Door closed, locking")

    except Exception as e:
        logging.error(f"[ERROR] Unhandled error: {e}")
    finally:
        pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    # 1. Start Hardware Background Threads
    if FINGERPRINT:
        threading.Thread(target=fingerprint_listener, daemon=True).start()
    
    threading.Thread(target=mag_switch_thread, daemon=True).start()
    
    # 2. Start the main hardware loop in a background thread
    threading.Thread(target=hardware_main_loop, daemon=True).start()
    
    # 3. Start the Discord Bot on the MAIN thread
    client.run(botToken)

