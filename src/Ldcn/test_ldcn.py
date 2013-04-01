###############################################################################
# test_ldcn.py 
# 
# test script for the linux implementation of LDCN.LIB version 1.52N
# Based on Source From http://www.logosolinc.com/software.htm
# 
# author:       Craig Wm. Versek, Yankee Environmental Systems 
# author_email: cwv@yesinc.com
###############################################################################
import sys, time
from ctypes import *

#module constants
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE    = 19200

#Ldcn Constants
STEPMODTYPE     = 3
SEND_ID         = 0x20
SPEED_8X        = 0x00      #use 8x timing
IGNORE_LIMITS   = 0x04      #Do not stop automatically on limit switches
POWER_ON        = 0x08      #set when motor power is on
STOP_SMOOTH     = 0x08      #set to decelerate motor smoothly
STP_ENABLE_AMP  = 0x01      #raise amp enable output
STP_DISABLE_AMP = 0x00      #lower amp enable output
STP_AMP_ENABLED = 0x04      #set if amplifier is enabled
MOTOR_MOVING    = 0x01      #set if motot is moving
START_NOW       = 0x80
LOAD_SPEED      = 0x02
LOAD_ACC        = 0x04
LOAD_POS        = 0x01
SEND_POS        = 0x01
STEP_REV        = 0x10      #reverse dir
TYPE_TINY       = 0x10
TYPE_STD        = 0x00
SET_CH_A        = 0x00
SET_CH_B        = 0x01



if __name__ == "__main__":
    ldcn = cdll.LoadLibrary("./libLdcn.so")
    num_modules = ldcn.LdcnInit(SERIAL_PORT, BAUDRATE)
    
    if num_modules == 0:
        num_modules = ldcn.LdcnFullInit(c_char_p(SERIAL_PORT), c_long(BAUDRATE))
    if num_modules == 0:   
        print "No Modules found at %s" % SERIAL_PORT
        sys.exit(1)

    #look for pico motor drivers
    pico_addr = 0
    for addr in range(1,num_modules+1):
        ldcn.LdcnReadStatus(addr, SEND_ID)
        mod_type    = ldcn.LdcnGetModType(addr)
        mod_version = ldcn.LdcnGetModVer(addr)
        print "mod_type: ", mod_type
        print "mod_version: ", mod_version
        if (mod_type == STEPMODTYPE) and (mod_version >= 50) and (mod_version < 60):
            pico_addr = addr
            break

    if pico_addr:
        # set parameters ----------------------------------------
        min_speed = 1
        run_current = 0
        hld_current = 0
        ADLimit = 0
        em_acc = 255
        mode = SPEED_8X  #or mode = SPEED_2X or  mode = SPEED_4X
        mode |= IGNORE_LIMITS
        res = ldcn.StepSetParam(pico_addr, mode, min_speed, run_current, hld_current, ADLimit, em_acc)
        if not res:
            print "Communication Error"
            ldcn.LdcnShutdown()
            sys.exit(1)
        # Select Motor Type and Channel -------------------------
        outval = TYPE_STD | SET_CH_A
  
        # Send output value to the device
        ldcn.StepSetOutputs(pico_addr, outval)
 
        # Read device status
        ldcn.LdcnNoOp(pico_addr)
        if ldcn.LdcnGetStat(pico_addr) & POWER_ON == 0:
            print "Invalid channel"

        # Reset Position
        ldcn.StepResetPos(pico_addr)

        # Enable Driver Driver ---------------------------------------
        ldcn.StepStopMotor(pico_addr, STOP_SMOOTH | STP_ENABLE_AMP)
        
        # Load Trajectory ---------------------------------------
        # Position mode (Velocity mode: mode = START_NOW | LOAD_SPEED | LOAD_ACC)
        mode = START_NOW | LOAD_SPEED | LOAD_ACC | LOAD_POS
        pos   = 25*2000 #2000 steps
        speed = 250     #2000 Hz
        acc   = 255     #max. acc.
        ldcn.StepLoadTraj(pico_addr, mode, pos, speed, acc, 0)

        # wait end of the motion
        while True:
            ldcn.LdcnReadStatus(pico_addr, SEND_POS)
            #read device status and current position
            pos = ldcn.StepGetPos(pico_addr)/25  #read steps
            print "Position: %d" % pos
            status_byte = ldcn.LdcnGetStat(pico_addr)
            if not (ldcn.LdcnGetStat(pico_addr) & MOTOR_MOVING):
                break
        
        # Disable driver amp (STOP_ABRUPT can also be used instead of STOP_SMOOTH)
        ldcn.StepStopMotor(pico_addr, STOP_SMOOTH)

        # Wait 2 secs.
        time.sleep(2)
        
        ldcn.LdcnShutdown()
