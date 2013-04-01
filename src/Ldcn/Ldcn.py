"""
 Ldcn.py 
 
 Python interface to the linux implementation of the Ldcnlibrary from 
 from http://www.logosolinc.com/software.htm
 
 author:       Craig Wm. Versek, Yankee Environmental Systems 
 author_email: cwv@yesinc.com
"""

__author__ = 'Craig Wm. Versek'
__date__ = '2012-07-11'

import sys, os
from ctypes import cdll, c_char_p, c_long, c_ubyte

LIBRARY_NAME = "libLdcn.so"

###############################################################################
import warnings

class LdcnError(Exception):
    pass

class LdcnWarning(Warning):
    pass

###############################################################################
class LdcnLibrary:
    __dll = None
    @staticmethod
    def getDll():
        if LdcnLibrary.__dll is None:
            if sys.platform == 'linux2':
                this_dir = os.path.dirname(os.path.realpath(__file__))
                lib_path = os.path.sep.join((this_dir,LIBRARY_NAME))
                LdcnLibrary.__dll = cdll.LoadLibrary(lib_path)
            else:
                raise RuntimeError("Platform not supported")
        return LdcnLibrary.__dll

###############################################################################
class BaseModule(object):
    def __init__(self, addr, mod_type, mod_version):
        self.addr        = addr
        self.mod_type    = mod_type
        self.mod_version = mod_version
        self._libldcn = LdcnLibrary.getDll() 

###############################################################################
#  TEST CODE
###############################################################################
if __name__ == "__main__":
    libldcn = LdcnLibrary.getDll()
