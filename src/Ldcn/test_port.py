import time
from ctypes import *
ldcn = cdll.LoadLibrary("./libLdcn.so")

port = c_char_p("/dev/ttyUSB0")
baudrate = c_long(9600)

#handle = ldcn.SioOpen(port,baudrate)

#print "handle =", handle

#test = "Hello! ABCDEFGHIJKLMNOPQRSTUVWXYZ 1234567890 abcdefghijklmnopqrstuvwxyz !@#$%^&*()"
#n = c_long(len(test))
#success = ldcn.SioPutChars(handle, c_char_p(test),n)
#print success
##time.sleep(0.1)

#ldcn.LdcnHardReset()
