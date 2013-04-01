from ctypes import c_char_p, c_long, c_ubyte, c_ushort
from Ldcn import LdcnError, LdcnWarning, BaseModule

#Stepper Module Constants
SEND_ID         = 0x20
IGNORE_LIMITS   = 0x04      #Do not stop automatically on limit switches
POWER_ON        = 0x08      #set when motor power is on
STOP_SMOOTH     = 0x08      #set to decelerate motor smoothly
STOP_ABRUPT     = 0x04     #set to stop motor immediately
STP_ENABLE_AMP  = 0x01      #raise amp enable output
STP_DISABLE_AMP = 0x00      #lower amp enable output
STP_AMP_ENABLED = 0x04      #set if amplifier is enabled
MOTOR_MOVING    = 0x01      #set if motor is moving
START_NOW       = 0x80
LOAD_SPEED      = 0x02
LOAD_ACC        = 0x04
LOAD_POS        = 0x01
SEND_POS        = 0x01
STEP_REV        = 0x10      #reverse dir
TYPE_TINY       = 0x10
TYPE_STD        = 0x00

SPEED_FACTOR_TABLE = {
    '8x': 0x00,
    '4x': 0x01,
    '2x': 0x02,
    '1x': 0x03, 
}
CHANNEL_TABLE = {
    'A': 0x00,
    'B': 0x01,
    'C': 0x10,
}

###############################################################################
class StepperModule(BaseModule):
    """
    """    

    def __init__(self, addr, mod_type, mod_version):
        BaseModule.__init__(self,addr, mod_type = mod_type, mod_version = mod_version)
    
    def setParams(self,
                  speed_factor = "1x", 
                  minspeed   = 1, 
                  runcur     = 0,
                  holdcur    = 0,
                  thermlimit = 0,
                  em_acc     = 0,
                  ignore_limits = True,
                 ):
        try:
            speed_factor_bits = SPEED_FACTOR_TABLE[speed_factor]
        except KeyError:
            keys = sorted(SPEED_FACTOR_TABLE.keys())
            msg = "speed_factor '%s' is invalid, should be one of '%s'" % (speed_factor, keys)
            raise LdcnError(msg)
        mode_byte = speed_factor_bits
        if ignore_limits:
            mode_byte |= IGNORE_LIMITS
        res = self._libldcn.StepSetParam(c_ubyte(self.addr), mode_byte, minspeed, runcur, holdcur, thermlimit, em_acc)
        if not res:
            msg = "Communication Error, shuting down"
            self._libldcn.LdcnShutdown()
            raise LdcnError(msg)

    def setOutputs(self, channel, motor_type = "standard"):
        try:
            channel_bits = CHANNEL_TABLE[channel]
        except KeyError:
            keys = sorted(CHANNEL_TABLE.keys())
            msg = "channel '%s' is invalid, should be one of '%s'" % (channel, keys)
            raise LdcnError(msg)
        
        motor_type_bits = 0x00
        if motor_type == 'standard':
            motor_type_bits = TYPE_STD
        elif mod_type == 'tiny':
            motor_type_bits = TYPE_TINY
        else:
            msg = "motor_type must be 'standard' or 'tiny', instead got '%s'" % motor_type
            raise LdcnError(msg)

        output_byte = channel_bits | motor_type_bits
        # Send output value to the device
        self._libldcn.StepSetOutputs(c_ubyte(self.addr), c_ubyte(output_byte))
    
    def getStatusByte(self):
        return self._libldcn.LdcnGetStat(c_ubyte(self.addr))

    def isPowerOn(self):
        self._libldcn.LdcnNoOp(c_ubyte(self.addr))
        status_byte = self.getStatusByte()
        return bool(status_byte | POWER_ON)
    
    def getPosition(self):
        self._libldcn.LdcnReadStatus(c_ubyte(self.addr), SEND_POS)
        #read device status and current position
        return  self._libldcn.StepGetPos(c_ubyte(self.addr))/25  #read steps

    def resetPosition(self):
        self._libldcn.StepResetPos(c_ubyte(self.addr))

    def enableDriver(self, stop_mode = 'smooth'):
        mode_byte = STP_ENABLE_AMP
        if stop_mode == 'smooth':
            mode_byte |= STOP_SMOOTH
        elif stop_mode == 'abrupt':
            mode_byte |= STOP_ABRUPT
        else:
            msg = "invalid stop_mode, must be 'smooth' or 'abrupt'"
            raise LdcnError(msg)
        self._libldcn.StepStopMotor(c_ubyte(self.addr), c_ubyte(mode_byte))

    def stopMotor(self, stop_mode = 'smooth'):
        mode_byte = STP_DISABLE_AMP
        if stop_mode == 'smooth':
            mode_byte |= STOP_SMOOTH
        elif stop_mode == 'abrupt':
            mode_byte |= STOP_ABRUPT
        else:
            msg = "invalid stop_mode, must be 'smooth' or 'abrupt'"
            raise LdcnError(msg)
        self._libldcn.StepStopMotor(c_ubyte(self.addr), c_ubyte(mode_byte))

    def loadTrajectory(self, pos, speed, acc):    
        # Load Trajectory ---------------------------------------
        # Position mode (Velocity mode: mode = START_NOW | LOAD_SPEED | LOAD_ACC)
        mode_byte = START_NOW | LOAD_SPEED | LOAD_ACC | LOAD_POS
        pos     *= 25   #multiple by 25 for pico motor FIXME
        steptime = 0  #always should be zero?
        self._libldcn.StepLoadTraj(c_ubyte(self.addr), 
                                   c_ubyte(mode_byte), 
                                   c_long(pos), 
                                   c_ubyte(speed), 
                                   c_ubyte(acc), 
                                   c_ushort(steptime), 
                                  )
    def isMotorMoving(self):
        self._libldcn.LdcnReadStatus(c_ubyte(self.addr), SEND_POS)
        status_byte = self._libldcn.LdcnGetStat(c_ubyte(self.addr))      
        return bool(status_byte & MOTOR_MOVING)
             
###############################################################################
#  TEST CODE
#  - connect a stepper module tp the Ldcn network
#  - run with "ipython -i" and explore 
###############################################################################
if __name__ == "__main__":
    from network import Network
    port = "/dev/ttyUSB0"    
    net = Network(port)
    net.initialize()
    stepper = net.modules[0]
    stepper.setParams(speed_factor='8x')
    stepper.setOutputs('A') #set the motor channel
    stepper.resetPosition()
    stepper.enableDriver()
    stepper.loadTrajectory(200,100,255)
    print stepper.isPowerOn()
    ldcn = stepper._libldcn 
    # wait end of the motion
    while True:
        ldcn.LdcnReadStatus(1, SEND_POS)
        #read device status and current position
        pos = ldcn.StepGetPos(1)/25  #read steps
        print "Position: %d" % pos
        status_byte = ldcn.LdcnGetStat(1)
        print status_byte
        if not (ldcn.LdcnGetStat(1) & MOTOR_MOVING):
            break
