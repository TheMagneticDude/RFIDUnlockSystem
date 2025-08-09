import time
import serial
import adafruit_fingerprint
import threading

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
                        print("Invalid fingerprint: failed to convert image");
                        continue
                    if finger.finger_search() != adafruit_fingerprint.OK:
                        print("Fingerprint not recognized");
                        continue
                    print(f"\n Fingerprint recognized: ID #{finger.finger_id} (confidence {finger.confidence})")
            time.sleep(1)  # prevent rapid repeat
        except RuntimeError as e:
            print("Fingerprint error:", e)
            time.sleep(1)


def get_fingerprint():
    with sensor_lock:
        print("Waiting for finger...")
        if finger.get_image() == adafruit_fingerprint.OK:
            if finger.image_2_tz(1) != adafruit_fingerprint.OK:
                return False
            if finger.finger_search() != adafruit_fingerprint.OK:
                return False
            print("Found ID #", finger.finger_id, "with confidence", finger.confidence)
            return True
        else:
            return False

def enroll_fingerprint(location):
    with sensor_lock:
        print("Enrolling finger at ID", location)

        for i in range(1, 3):
            print("Place finger on sensor...", f"Step {i}/2")
            while finger.get_image() != adafruit_fingerprint.OK:
                pass
            if finger.image_2_tz(i) != adafruit_fingerprint.OK:
                ##scan to template buffer
                print("Failed to convert image")
                return False
            print("Remove finger")
            time.sleep(2)

        if finger.create_model() != adafruit_fingerprint.OK:
            print("Failed to create model")
            return False
        if finger.store_model(location) != adafruit_fingerprint.OK:
            print("Failed to store model")
            return False

        print("Fingerprint enrolled successfully at ID", location)
        return True

def clear_fingerprints():
    with sensor_lock:
        if finger.empty_library() == adafruit_fingerprint.OK:
            print("All fingerprints have been cleared.")
        else:
            print("Failed to clear fingerprint library.")
        
def delete_fingerprint(ID):
    with sensor_lock:
        if finger.delete_model(ID) == adafruit_fingerprint.OK:
            print(f"ID: {ID} Deleted")
        else:
            print(f"Failed to delete fingerprint at ID {ID}. It may not exist.")



# Example usage
if __name__ == "__main__":
    ##start async thread
    listener_thread = threading.Thread(target=fingerprint_listener, daemon=True)
    listener_thread.start()
    while True:
        print("1: Enroll fingerprint")
        print("2: Check fingerprint")
        print("3: Delete fingerprint ID");
        print("4: Clear all fingerprints")
        print("q: Quit")

        choice = input("> ").strip()##//remove leading and trailing whitespace

        if choice == "1":
            id_to_use = int(input("Enter ID number (0~127): "))
            enroll_fingerprint(id_to_use)
        elif choice == "2":
            try:
                get_fingerprint()
            except RuntimeError as e:
                print("Sensor error:", e)
            
        elif choice == "3":
            delID = int(input("Enter ID to delete (0-127): "))
            delete_fingerprint(delID);
        elif choice == "4":
            clear_fingerprints();
        elif choice == "q":
            break


