# RFIDUnlockSystem


For manual overrides:

Aliases that can be run

RFIDunlock : unlocks door no matter what

RFIDlock : locks door if door is detected to be closed

RFIDforcelock : locks door even if door is open




under ~/.bashrc

alias rfidStart="rfidenv && cd ~/RFIDUnlock"

RFIDunlock() {
    rfidStart || return 1

    cd ~/RFIDUnlock/manualoverrides || {
        echo "Failed to cd into manualoverrides"
        return 1
    }

    sudo systemctl stop rfid-door.service || return 1
    sudo python3 Unlock.py
    sudo systemctl restart rfid-door.service
}


RFIDlock() {
    rfidStart || return 1

    cd ~/RFIDUnlock/manualoverrides || {
        echo "Failed to cd into manualoverrides"
        return 1
    }

    sudo systemctl stop rfid-door.service || return 1
    sudo python3 Lock.py
    sudo systemctl restart rfid-door.service
}


RFIDforcelock() {
    rfidStart || return 1

    cd ~/RFIDUnlock/manualoverrides || {
        echo "Failed to cd into manualoverrides"
        return 1
    }

    sudo systemctl stop rfid-door.service || return 1
    sudo python3 ForceLock.py
    sudo systemctl restart rfid-door.service
}