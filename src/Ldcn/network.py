from ctypes import c_char_p, c_long, c_ubyte
from Ldcn import LdcnError, LdcnWarning, LdcnLibrary, BaseModule
from stepper import StepperModule


DEFAULT_BAUDRATE = 19200

#Ldcn Constants
SEND_ID         = 0x20
STEPMODTYPE = 3

###############################################################################
MODULE_CLASS_MAP = {
  STEPMODTYPE: StepperModule,
}
         
class Network(object):
    def __init__(self, port, baudrate = DEFAULT_BAUDRATE):
        self.port     = port
        self.baudrate = baudrate        
        self.modules  = []
        self._libldcn = LdcnLibrary.getDll()

    def initialize(self):
        num_modules = self._libldcn.LdcnInit(c_char_p(self.port),c_long(self.baudrate))
        if num_modules == 0:
            num_modules = self._libldcn.LdcnFullInit(c_char_p(self.port), c_long(self.baudrate))
            if num_modules == 0:   
                msg = "No Modules found at %s" % self.port
                raise LdcnError(msg)
        #query module types
        for addr in range(1,num_modules+1):
            self._libldcn.LdcnReadStatus(c_ubyte(addr), SEND_ID)
            mod_type    = self._libldcn.LdcnGetModType(c_ubyte(addr))
            mod_version = self._libldcn.LdcnGetModVer(c_ubyte(addr))
            mod_class = None
            try:
                mod_class = MODULE_CLASS_MAP[mod_type]
            except KeyError:  #instead warn that this module type has not been implemented
                msg = "Control for mod_type=%d, mod_version=%d has not been implemented, defaulting to BaseModule interface"
                warnings.warn(msg,LdcnWarning)
                mod_class = BaseModule
            mod = mod_class(addr, mod_type, mod_version)                
            self.modules.append(mod)

    def getModule(self,addr):
        return self.modules[addr-1]
    
    def shutdown(self):
        self._libldcn.LdcnShutdown()
        self.modules = []
    

###############################################################################
#  TEST CODE
###############################################################################
if __name__ == "__main__":
    port = "/dev/ttyUSB0"    
    net = Network(port)
    net.initialize()
    print "Found %d modules:" % len(net.modules)
    for mod in net.modules:
        print "\tmod_type = %d, mod_version = %d: %s" % (mod.mod_type, mod.mod_version, mod)
