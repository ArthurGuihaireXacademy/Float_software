from __future__ import print_function
import time
import sys
import math
import qwiic_scmd


myMotor = qwiic_scmd.QwiicScmd()

def runExample():
    print("Motor Test.")
    R_MTR = 0
    FWD = 0
    BWD = 1

    if myMotor.connected == False:
        print("Motor Driver not connected. Check connections.", \
            file=sys.stderr)
        return
    myMotor.begin()
    print("Motor initialized.")
    time.sleep(.250)

    # Zero Motor Speeds
    myMotor.set_drive(0,0,0)

    myMotor.enable()
    print("Motor enabled")
    time.sleep(.250)


        # Main loop
    while True:
        speed = 255  # Speed (0-255)
        print("Running motor forward")
        for i in range(3): # Run forward for 10 seconds
            myMotor.set_drive(R_MTR, FWD, speed)
            time.sleep(1)
        myMotor.set_drive(0,0,0)
        time.sleep(0.5) 
        print("Running motor backward")
        for i in range(3):  # Run backward for 10 seconds
            myMotor.set_drive(R_MTR, BWD, speed)
            time.sleep(1)
        myMotor.set_drive(0,0,0)
        time.sleep(0.5) 

if __name__ == '__main__':
    try:
        runExample()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("Ending example.")
        myMotor.disable()
        sys.exit(0) 


