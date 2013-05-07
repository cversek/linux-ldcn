/*****************************************************************************
LDCN.LIB version 1.52N

Linux Implementation
Based on Source From http://www.logosolinc.com/software.htm 

author:       Craig Wm. Versek, Yankee Environmental Systems 
author_email: cwv@yesinc.com
******************************************************************************/
#include <stdio.h>
#include <string.h> /* memset */
#include <unistd.h> /* close */
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>  /* select */
#include <sys/time.h>
#include <sys/ioctl.h>
#include <math.h>
#include "Ldcn.h"
#include <errno.h>

// Uncomment the following line for debugging info
//#define DEBUG

extern int errno;

#define SERIAL_WRITE_TIMEOUT_SECONDS 0
#define SERIAL_WRITE_TIMEOUT_MIRCOSECONDS 100000
#define SERIAL_READ_TIMEOUT_SECONDS 0
#define SERIAL_READ_TIMEOUT_MIRCOSECONDS 100000
// ----------------------------------------------------------------------------
// S E R I A L  I O  ----------------------------------------------------------
// ----------------------------------------------------------------------------

/******************************************************************************
SioOpen                    <Ldcn.lib>

SYNTAX: handle SioOpen (int port, long baudrate);

PARAMETER1: <port>     - valid Linux tty device
PARAMETER2: <baudrate> - Valid baud rates :
                         9600, 19200, 38400
KEYWORDS: SerialIO

DESCRIPTION: Opens a COM port at the specified baud rate

RETURN VALUE: If the function succeeds the return value is an open
              handle to the specified file descriptor of the port, 
              if the function fails, the return value is 
              INVALID_HANDLE_VALUE.
******************************************************************************/

// Opens a COM port, returns a handle to be used by other
// SIO operations.

/*handle SioOpen(int port, long baudrate) {*/
/*  char port_str[255];*/
/*  sprintf(port_str,"/dev/ttyS%d",port);*/
/*  return SioOpen(port_str, baudrate);*/
/*}*/

struct termios Old_TIO;      //keep track of previous port settings
struct termios Current_TIO;  //keep track of current port settings
 
handle SioOpen(char *port, long baudrate) {
  int fd;
  speed_t baudrate_cflag;

  #ifdef DEBUG  
    printf("***DEBUG*** IN SioOpen\n");
  #endif

  switch (baudrate) {
    case 9600:   baudrate_cflag = B9600;  break;
    case 19200:  baudrate_cflag = B19200; break;
    case 38400:  baudrate_cflag = B38400; break;
    default:     baudrate_cflag = B19200;
  }

  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  //fd = open(port, O_RDWR | O_NOCTTY);
  if (fd < 0) { return INVALID_HANDLE_VALUE; }
  ComPortDevice = port; //set global variable
  #ifdef DEBUG
    printf("***DEBUG***\tOpening fd(ComPort) = %d, ComPortDevice = %s\n", fd, ComPortDevice);
  #endif

  tcgetattr(fd,&Old_TIO); //save current port settings

  memset(&Current_TIO, '\0', sizeof(struct termios));
  Current_TIO.c_cflag &= ~(CSIZE | PARENB);
  Current_TIO.c_cflag = baudrate_cflag | CS8 | CLOCAL | CREAD;
  
  Current_TIO.c_iflag = IGNPAR;
  Current_TIO.c_oflag = 0;

  //set input mode (non-canonical, no echo,...)
  Current_TIO.c_lflag = 0;

  Current_TIO.c_cc[VTIME]     = 0;//10;
  Current_TIO.c_cc[VMIN]      = 1;//0;

  // Finally, apply the configuration
  //tcflush(fd, TCIFLUSH);
  //tcsetattr(fd, TCSANOW, &Current_TIO);  
  if(tcsetattr(fd, TCSAFLUSH, &Current_TIO) < 0) {
    #ifdef DEBUG
      perror ("***DEBUG***\tThe following error occurred\n");
      printf( "***DEBUG***\tValue of errno: %d\n", errno );
      printf("***DEBUG*** OUT SioOpen -> %d (INVALID_HANDLE_VALUE)\n", INVALID_HANDLE_VALUE);
    #endif
      return INVALID_HANDLE_VALUE;
  }

  #ifdef DEBUG
    printf("***DEBUG*** OUT SioOpen -> %d\n", fd);
  #endif
  return fd;
}
//-----------------------------------------------------------------------------

/******************************************************************************
SioClose                  <Ldcn.lib>

SYNTAX: bool SioClose (handle ComPort);

PARAMETER1: <ComPort> - file descriptor of the port

KEYWORDS: SerialIO

DESCRIPTION: Close a previously opened COM port

RETURN VALUE: true on success, false on failure
*******************************************************************************/

// Close a previously opened COM port

bool SioClose( handle ComPort) {
  int fd, res;
  #ifdef DEBUG
    printf("***DEBUG*** IN SioClose\n");
  #endif
  #ifdef DEBUG
    printf("***DEBUG***\tClosing ComPort = %d, ComPortDevice = %s", ComPort, ComPortDevice);
  #endif
  fd = ComPort;
  res = tcsetattr(fd, TCSANOW, &Old_TIO);  //restore previous port settings
  if (res == -1 ){
    #ifdef DEBUG
      perror ("***DEBUG***\tThe following error occurred\n");
      printf( "***DEBUG***\tValue of errno: %d\n", errno );
      printf("***DEBUG***OUT SioClose -> false");
    #endif 
    return false;            //some error occured
  }
  res = close(fd);
  if (res == -1 ){
    #ifdef DEBUG
      perror ("***DEBUG***\tThe following error occurred\n");
      printf( "***DEBUG***\tValue of errno: %d\n", errno );
      printf("***DEBUG*** OUT SioClose -> false\n");
    #endif 
    return false;            //some error occured
  }
  
  #ifdef DEBUG
    printf("***DEBUG*** OUT SioClose -> true\n");
  #endif
  return true;
}
//-----------------------------------------------------------------------------

/******************************************************************************
SioClrInbuf                  <Ldcn.lib>

SYNTAX: bool SioClrInbuf (handle ComPort);

PARAMETER1: <ComPort> - file descriptor of the port

KEYWORDS: SerialIO

DESCRIPTION: Clears all characters in ComPort's input buffer.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool SioClrInbuf( handle ComPort) {
  int fd, res;
  #ifdef DEBUG
    printf("***DEBUG*** IN SioClrInbuf\n");
  #endif
  fd = ComPort;
  res = tcflush(fd, TCIFLUSH); //flush non-transmitted output data, non-read input data or both
  if (res == -1 ) return false; //some error occured
  #ifdef DEBUG
    printf("***DEBUG*** OUT SioClrInbuf\n");
  #endif
  return true;
}
//-----------------------------------------------------------------------------

/******************************************************************************
SioChangeBaud                  <Ldcn.lib>

SYNTAX: bool SioChangeBaud (handle ComPort, long baudrate);

PARAMETER1: <port>     - file descriptor of the port
PARAMETER2: <baudrate> - Valid baud rates :
                         9600, 19200, 38400, 57600 and 115200
KEYWORDS: SerialIO

DESCRIPTION: Changes the baud rate of ComPort. The handle ComPort
             must have been created by SioOpen() function.

RETURN VALUE: Returns false on failure, true on success.
******************************************************************************/

// Change the baud rate to the specified values.  Valid rates are:
// 9600, 19200, 38400, 57600, 115200.  Returns TRUE on success.

bool SioChangeBaud( handle ComPort, long baudrate) {
  int res, fd;
  speed_t baudrate_cflag;
  #ifdef DEBUG
    printf("***DEBUG*** IN SioChangeBaud\n");
  #endif
  switch (baudrate) {
    case 9600:   baudrate_cflag = B9600;  break;
    case 19200:  baudrate_cflag = B19200; break;
    case 38400:  baudrate_cflag = B38400; break;
    default:     baudrate_cflag = B19200;
  }

  //SioClose(ComPort);
  //ComPort = SioOpen(ComPortDevice, baudrate);
  res = cfsetospeed(&Current_TIO,baudrate_cflag);
  if(res == -1){return false;} //Error in setting baudrate
  res = cfsetispeed(&Current_TIO,baudrate_cflag);
  if(res == -1){return false;} //Error in setting baudrate
  
  fd = ComPort;
  if(tcsetattr(fd, TCSAFLUSH, &Current_TIO) < 0) {
    #ifdef DEBUG
      perror ("***DEBUG***\tThe following error occurred\n");
      printf( "***DEBUG***\tValue of errno: %d\n", errno );
      printf("***DEBUG*** OUT SioChangeBaud -> false\n");
    #endif
      return false;
  }
  #ifdef DEBUG
    printf("***DEBUG*** OUT SioChangeBaud\n");
  #endif
  return true; //FIXME should we even bother?

}
//-----------------------------------------------------------------------------

/******************************************************************************
SioPutChars                  <Ldcn.lib>

SYNTAX: bool SioPutChars (handle ComPort, char *stuff, int n);

PARAMETER1: <ComPort> - file descriptor of the port
PARAMETER2: <stuff>   - data buffer
PARAMETER3: <n>       - number of character to be written

KEYWORDS: SerialIO

DESCRIPTION: Writes out n characters to the COM port specified by the handle
             ComPort created by SioOpen(), returns only after chars have been
             sent. Stuff points to the buffer containing the data to be sent.

RETURN VALUE: Return 0 on failure, non-zero on success.
******************************************************************************/

bool SioPutChars( handle ComPort, char *stuff, int n) {
  int fd,nums,selectval;
  fd_set set;
  struct timeval timeout;
  #ifdef DEBUG
    //printf("IN SioPutChars\n");
    int i;
    printf("***DEBUG***\tsending chars: ");
    for(i=0; i < n; i++){
      printf("%02x ",(unsigned char) stuff[i]);
    }
    printf("\n");
  #endif
  /* Initialize the file descriptor set. */
  fd = ComPort;
  FD_ZERO (&set);
  FD_SET (fd, &set);

  /* Initialize the timeout data structure. */
  timeout.tv_sec  = SERIAL_WRITE_TIMEOUT_SECONDS;
  timeout.tv_usec = SERIAL_WRITE_TIMEOUT_MIRCOSECONDS;

  //loop to write out whole buffer 
  nums = 0;
  while( n > 0 ){
    //block until file is ready for writing
    selectval = select(FD_SETSIZE, NULL, &set, NULL, &timeout);
    nums = write(fd, stuff, n);    
    if(nums == -1){ //error condition
      #ifdef DEBUG
        perror ("***DEBUG***\tThe following error occurred\n");
        printf( "***DEBUG***\tValue of errno: %d\n", errno );
        printf("***DEBUG*** OUT SioPutChars -> 0\n");
      #endif
      return 0;
    }  
    stuff += nums;  //increment pointer position     
    n -= nums;      //decrement remaining data bytes
  }
  #ifdef DEBUG
    printf("***DEBUG*** OUT SioPutChars -> true\n");
  #endif
  return true;
}
//-----------------------------------------------------------------------------

/******************************************************************************
SioTest                       <Ldcn.lib>

SYNTAX: int SioTest (handle ComPort);

PARAMETER1: <ComPort> - file descriptor of the port

KEYWORDS: SerialIO

RETURN VALUE: Returns the number of characters in the ComPort's input buffer
******************************************************************************/

int SioTest( handle ComPort) {
  int fd, nums;
  #ifdef DEBUG
    printf("***DEBUG*** IN SioTest\n");
  #endif
  fd = ComPort;
  ioctl(fd, FIONREAD, &nums);
  #ifdef DEBUG
    printf("***DEBUG*** OUT SioTest\n");
  #endif
  return nums;
}
//-----------------------------------------------------------------------------

/******************************************************************************
SioGetChars                 <Ldcn.lib>

SYNTAX: int SioGetChars (handle ComPort, char *stuff, int n);

PARAMETER1: <ComPort> - file descriptor of the port
PARAMETER2: <stuff>   - data buffer
PARAMETER3: <n>       - number of character to read

KEYWORDS: SerialIO

DESCRIPTION: Read in <n> characters from the COM port specified by the handle 
             ComPort, created by SioOpen() and puts them in the array pointed 
             to by stuff. This function has a timeout value of approximately 
             100 milliseconds.

RETURN VALUE: Returns the number of characters actually read.
******************************************************************************/

int SioGetChars( handle ComPort, char *stuff, int n) {
  int fd, nums, numread, selectval, waiting;
  fd_set set;
  struct timeval timeout;
  
  #ifdef DEBUG
    printf("***DEBUG*** IN SioGetChars(handle=%d,n=%d)\n", ComPort, n);
  #endif
  /* Initialize the file descriptor set. */
  fd = ComPort;
  
  FD_ZERO (&set);
  FD_SET (fd, &set);

  /* Initialize the timeout data structure. */
  timeout.tv_sec  = SERIAL_READ_TIMEOUT_SECONDS;
  timeout.tv_usec = SERIAL_READ_TIMEOUT_MIRCOSECONDS;

  nums    = 0;
  numread = 0;
  while( n > 0){
      //block until file is ready for reading
      selectval = select(FD_SETSIZE, &set, NULL, NULL, &timeout);
      #ifdef DEBUG
        printf("***DEBUG***\tselectval=%d\n", selectval); //DEBUG
      #endif
      if(selectval == 0){ //select timed out...wait and then test for chars
        #ifdef DEBUG
          printf("***DEBUG***\tselect timed out\n");
        #endif
        usleep(SERIAL_READ_TIMEOUT_MIRCOSECONDS);
        waiting = SioTest(ComPort); 
        if(waiting == 0){ //failed the timeout test
           #ifdef DEBUG
             printf("***DEBUG***\tSioTest failed\n");
             printf("***DEBUG*** OUT SioGetChars -> 0\n");
           #endif 
          return 0;
        } 
      }
      nums = read(fd, stuff, n);
      #ifdef DEBUG
        printf("***DEBUG***\tnums=%d\n", nums); //DEBUG
      #endif    
      if(nums == -1){ //error condition
        #ifdef DEBUG
          perror ("***DEBUG***\tThe following error occurred\n");
          printf( "***DEBUG***\tValue of errno: %d\n", errno );
        #endif
        return 0;
      }
      stuff   += nums;  //increment pointer position     
      n       -= nums;  //decrement remaining data bytes
      numread += nums;  //increment numbers of characters read  
  }
  //DEBUG
  #ifdef DEBUG
    int i;
    printf("***DEBUG***\tgot chars: ");
    for(i=0; i < numread; i++){
      printf("%02x ",(unsigned char) stuff[i]);
    }
    printf("\n");
    //END DEBUG
    printf("***DEBUG*** OUT SioGetChars -> %d\n", numread);
  #endif
  return numread;
}
//---------------------------------------------------------------------------

/****************************************************************************
SioPDmode                  <Ldcn.lib>

SYNTAX: bool SioPDmode (int mode);

PARAMETER1: <mode> -  New mode status

KEYWORDS: SerialIO

DESCRIPTION: (NOT IMPLEMENTED) Set PDstatus in <mode> if PD not open.

RETURN VALUE: false if PD is already open, else - true
****************************************************************************/

bool SioPDmode(int mode) {
  return false;  //FIXME not implemented
}

/******************************************************************************
LdcnSetNet                  <Ldcn.lib>

SYNTAX: bool LdcnSetNet(int netN);

PARAMETER1: <netN> - network number

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Ldcn.Lib supports up to four RS485 networks, connected to
             com ports 1, 2, 3 or 4.
             If you work with more than one network, use this function to switch
             to certain network.
             To reduce used memory include following definitions:

                #define MAXNUMNET  n  - the number of RS485 networks

             and for every network define:

                #define MAXNUMMOD1 x - the maximum number of modules for network 1
                #define MAXNUMMOD2 x - the maximum number of modules for network 2
                ...
             before

                #use "Ldcn.Lib" statement in your program.

             Default value for MAXNUMNET is 1 and for MAXNUMMOD1 is 32.

RETURN VALUE:  true on success, false if netN is <= 0 or greater than MAXNUMNET
****************************************************************************/

bool LdcnSetNet(int netN) {
  if ((netN < 1) || (netN > MAXNUMNET)) { printf("ERROR in define of MAXNUMNET\n"); return 0; }

  net.nummod_A[net_num - 1] = nummod;
  net.ComPort_A[net_num - 1] = ComPort;
  net.BaudRate_A[net_num - 1] = BaudRate;
  net.SioError_A[net_num - 1] = SioError;
  net.IOBusy_A[net_num - 1] = IOBusy;

  switch (netN) {
    case 1: mod = net.mod1; max_nummod = MAXNUMMOD1; break;
#if MAXNUMNET > 1
    case 2: mod = net.mod2; max_nummod = MAXNUMMOD2; break;
#endif
#if MAXNUMNET > 2
    case 3: mod = net.mod3; max_nummod = MAXNUMMOD3; break;
#endif
#if MAXNUMNET > 3
    case 4: mod = net.mod4; max_nummod = MAXNUMMOD4; break;
#endif
  }
  net_num = netN;                    // Load new working values
  nummod   = net.nummod_A[net_num - 1];
  ComPort  = net.ComPort_A[net_num - 1];
  BaudRate = net.BaudRate_A[net_num - 1];
  SioError = net.SioError_A[net_num - 1];
  IOBusy   = net.IOBusy_A[net_num - 1];

  return 1;
}
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// S E R V O   M O D U L E  ---------------------------------------------------
//-----------------------------------------------------------------------------

//--------------------- Servo Module initialization ---------------------------

void ServoNewMod(byte addr) {
  mod[addr].kind.servo.pos     = 0;
  mod[addr].kind.servo.ad      = 0;
  mod[addr].kind.servo.vel     = 0;
  mod[addr].kind.servo.aux     = 0;
  mod[addr].kind.servo.home    = 0;
  mod[addr].kind.servo.perror  = 0;
  mod[addr].kind.servo.inport1 = 0;
  mod[addr].kind.servo.inport2 = 0;
  mod[addr].kind.servo.inport3 = 0;
  mod[addr].kind.servo.cmdpos  = 0;
  mod[addr].kind.servo.cmdvel  = 0;
  mod[addr].kind.servo.cmdacc  = 0;
  mod[addr].kind.servo.cmdpwm  = 0;
  mod[addr].kind.servo.cmdadc  = 0;

  mod[addr].kind.servo.gain.kp = 0;
  mod[addr].kind.servo.gain.kd = 0;
  mod[addr].kind.servo.gain.ki = 0;
  mod[addr].kind.servo.gain.il = 0;
  mod[addr].kind.servo.gain.ol = 0;
  mod[addr].kind.servo.gain.cl = 0;
  mod[addr].kind.servo.gain.el = 0;
  mod[addr].kind.servo.gain.sr = 1;
  mod[addr].kind.servo.gain.dc = 0;

  mod[addr].kind.servo.stoppos  = 0;
  mod[addr].kind.servo.ioctrl   = IO1_IN | IO2_IN;
  mod[addr].kind.servo.homectrl = 0;
  mod[addr].kind.servo.movectrl = 0;
  mod[addr].kind.servo.stopctrl = 0;
  mod[addr].kind.servo.servoinit= 0;
  mod[addr].kind.servo.stp_dir_mode = 0;
  mod[addr].kind.servo.advmode = 0;
  mod[addr].kind.servo.npoints = 0;
  mod[addr].kind.servo.last_ppoint = 0;
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetStat                  <Ldcn.lib>

SYNTAX: bool ServoGetStat (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Receives the status data of a specified Servo module according
             to the last defined by LdcnDefineStatus () status items.

RETURN VALUE:  true on success, false on failure
******************************************************************************/

bool ServoGetStat(byte addr) {
  int numbytes, numrcvd;
  int i, bytecount;
  byte cksum;
  byte inbuf[32];

// Find number of bytes to read:
  numbytes = 2;                   // start with stat & cksum
  if ( (mod[addr].statusitems) & SEND_POS  ) numbytes +=4;
  if ( (mod[addr].statusitems) & SEND_AD   ) numbytes +=1;
  if ( (mod[addr].statusitems) & SEND_VEL  ) numbytes +=2;
  if ( (mod[addr].statusitems) & SEND_AUX  ) numbytes +=1;
  if ( (mod[addr].statusitems) & SEND_HOME ) numbytes +=4;
  if ( (mod[addr].statusitems) & SEND_ID   ) numbytes +=2;

  if ((mod[addr].modver > 4) || (mod[addr].modtype == SERVOHYBTYPE)) {
    if ( (mod[addr].statusitems) & SEND_PERROR ) numbytes +=2;
    if ((mod[addr].modver >=60) && (mod[addr].modver < 70))
      if ( mod[addr].statusitems  & SEND_INPORTS) numbytes +=3;
    if (mod[addr].kind.servo.advmode && (mod[addr].statusitems & SEND_NPOINTS)) numbytes +=1;
  }

  numrcvd = SioGetChars(ComPort, (char *)inbuf, numbytes);

// Verify enough data was read
  if (numrcvd != numbytes)
     return false;

// Verify checksum:
  cksum = 0;
  for (i = 0; i < numbytes-1; i++) cksum = (byte)(cksum + inbuf[i]);
  if (cksum != inbuf[numbytes-1])
      return false;

// Verify command was received intact before updating status data
  mod[addr].stat = inbuf[0];
  if (mod[addr].stat & CKSUM_ERROR)
      return false;

// Finally, fill in status data
  bytecount = 1;
  if ( (mod[addr].statusitems) & SEND_POS ) {
     mod[addr].kind.servo.pos = *( (long *)(inbuf + bytecount) );
     bytecount +=4;
  }
  if ( (mod[addr].statusitems) & SEND_AD )  {
     mod[addr].kind.servo.ad = inbuf[bytecount];
     bytecount +=1;
  }
  if ( (mod[addr].statusitems) & SEND_VEL ) {
     mod[addr].kind.servo.vel = *( (int *)(inbuf + bytecount) );
     bytecount +=2;
  }
  if ( (mod[addr].statusitems) & SEND_AUX ) {
     mod[addr].kind.servo.aux = inbuf[bytecount];
     bytecount +=1;
  }
  if ( (mod[addr].statusitems) & SEND_HOME ) {
     mod[addr].kind.servo.home = *( (unsigned long *)(inbuf + bytecount) );
     bytecount +=4;
  }
  if ( (mod[addr].statusitems) & SEND_ID ) {
     mod[addr].modtype = inbuf[bytecount];
     mod[addr].modver = inbuf[bytecount+1];
     bytecount +=2;
  }
  if ( (mod[addr].statusitems) & SEND_PERROR )
   if ( (mod[addr].modver > 4) || (mod[addr].modtype == SERVOHYBTYPE))  {
     mod[addr].kind.servo.perror = *( (int *)(inbuf + bytecount) );
     bytecount +=2;
  }
  if ( (mod[addr].statusitems & SEND_NPOINTS) && mod[addr].kind.servo.advmode ) {
     mod[addr].kind.servo.npoints = inbuf[bytecount];
     bytecount +=1;
  }

  if ( (mod[addr].statusitems & SEND_INPORTS) &&               // check for v6x
       (mod[addr].modver >=60) && (mod[addr].modver < 70)) {
     mod[addr].kind.servo.inport1 = inbuf[bytecount++];
     mod[addr].kind.servo.inport2 = inbuf[bytecount++];
     mod[addr].kind.servo.inport3 = inbuf[bytecount++];
  }
  return true;
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetPos                  <Ldcn.lib>

SYNTAX: long ServoGetPos (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the current motor position of a SERVO module.
             Note: this data is only valid if the SEND_POS bit has been
             set in the most recently issued LdcnDefineStatus () command.

RETURN VALUE: current motor position of a SERVO module
******************************************************************************/

long ServoGetPos(byte addr) {  return mod[addr].kind.servo.pos; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetAD                  <Ldcn.lib>

SYNTAX: byte ServoGetAD (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the current A/D value of a SERVO module.
             Note: this data is only valid if the SEND_AD bit has been
             set in the most recently issued LdcnDefineStatus () command.

RETURN VALUE: current A/D value of a SERVO module
******************************************************************************/

byte ServoGetAD(byte addr) {  return mod[addr].kind.servo.ad; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetVel                 <Ldcn.lib>

SYNTAX: int ServoGetVel (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the current motor velocity of a SERVO module.
             Note: this data is only valid if the SEND_VEL bit has been
             set in the most recently issued LdcnDefineStatus () command.

RETURN VALUE: current motor velocity of a SERVO module.
******************************************************************************/

int ServoGetVel(byte addr) {  return mod[addr].kind.servo.vel; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetAux                  <Ldcn.lib>

SYNTAX: byte ServoGetAux (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the current auxiliary status byte of a SERVO module.
             Note: this data is only valid if the SEND_AUX bit has been
             set in the most recently issued LdcnDefineStatus () command.

RETURN VALUE: current auxiliary status byte of a SERVO module
******************************************************************************/

byte ServoGetAux(byte addr) {  return mod[addr].kind.servo.aux; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetHome                  <Ldcn.lib>

SYNTAX: long ServoGetHome (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the current motor home position of a SERVO module.
             Note: this data is only valid if the SEND_HOME bit has been
             set in the most recently issued LdcnDefineStatus () command.

RETURN VALUE: current motor home position of a SERVO module
******************************************************************************/

long ServoGetHome(byte addr) {  return mod[addr].kind.servo.home; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetPError                 <Ldcn.lib>

SYNTAX: int ServoGetPError (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the current position error of a SERVO module.
             Note: this data is only valid if the SEND_PERROR bit has
             been set in the most recently issued LdcnDefineStatus()
             command and the module version is greater than 5.

RETURN VALUE: current position error of a SERVO module
******************************************************************************/

int ServoGetPError(byte addr) {  return mod[addr].kind.servo.perror; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetInport1                  <Ldcn.lib>

SYNTAX: byte ServoGetInport1 (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Return the current value of input port 1.
             Note: This data is only valid if the SEND_INPORTS bit has been
             set in the most recently issued LdcnDefineStatus () command and
             the module version is greater or equal than 60 and less than 70.

RETURN VALUE: current value of input port 1
******************************************************************************/

byte ServoGetInport1(byte addr) {  return mod[addr].kind.servo.inport1; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetInport2                  <Ldcn.lib>

SYNTAX: byte ServoGetInport2 (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Return the current value of input port 2.
             Note: This data is only valid if the SEND_INPORTS bit has been
             set in the most recently issued LdcnDefineStatus () command and
             the module version is greater or equal than 60 and less than 70.

RETURN VALUE: current value of input port 2
******************************************************************************/

byte ServoGetInport2(byte addr) {  return mod[addr].kind.servo.inport2; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetInport3                  <Ldcn.lib>

SYNTAX: byte ServoGetInport3 (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Return the current value of input port 3.
             Note: This data is only valid if the SEND_INPORTS bit has been
             set in the most recently issued LdcnDefineStatus () command and
             the module version is greater or equal than 60 and less than 70.

RETURN VALUE: current value of input port 3
******************************************************************************/

byte ServoGetInport3(byte addr) {  return mod[addr].kind.servo.inport3; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetOutport1                  <Ldcn.lib>

SYNTAX: byte ServoGetOutport1 (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Return the current value of output port 1.
             Note: This data is only valid if the module version
                   is greater or equal than 60 and less than 70.

RETURN VALUE: current value of output port 1
******************************************************************************/

byte ServoGetOutport1(byte addr) {  return mod[addr].kind.servo.outport1; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetOutport2                  <Ldcn.lib>

SYNTAX: byte ServoGetOutport2 (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Return the current value of output port 2.
             Note: This data is only valid if the module version
                   is greater or equal than 60 and less than 70.

RETURN VALUE: current value of output port 2
******************************************************************************/

byte ServoGetOutport2(byte addr) {  return mod[addr].kind.servo.outport2; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetOutport3                  <Ldcn.lib>

SYNTAX: byte ServoGetOutport3 (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Return the current value of output port 3.
             Note: This data is only valid if the module version
                   is greater or equal than 60 and less than 70.

RETURN VALUE: current value of output port 3
******************************************************************************/

byte ServoGetOutport3(byte addr) {  return mod[addr].kind.servo.outport3; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetOutport4                  <Ldcn.lib>

SYNTAX: byte ServoGetOutport4 (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Return the current value of output port 4.
             Note: This data is only valid if the module version
                   is greater or equal than 60 and less than 70.

RETURN VALUE: current value of output port 4
******************************************************************************/

byte ServoGetOutport4(byte addr) {  return mod[addr].kind.servo.outport4; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetCmdPos                  <Ldcn.lib>

SYNTAX: long ServoGetCmdPos (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the last used command position for a SERVO module.

RETURN VALUE: the last command position for a SERVO module
******************************************************************************/

long ServoGetCmdPos(byte addr) {  return mod[addr].kind.servo.cmdpos; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetCmdVel                   <Ldcn.lib>

SYNTAX: long ServoGetCmdVel (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the last used command velocity for a SERVO module.

RETURN VALUE: the last command velocity for a SERVO module
******************************************************************************/

long ServoGetCmdVel(byte addr) {  return mod[addr].kind.servo.cmdvel; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetCmdAcc                  <Ldcn.lib>

SYNTAX: long ServoGetCmdAcc (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the last used command acceleration for a SERVO module.

RETURN VALUE: the last command acceleration for a SERVO module
******************************************************************************/

long ServoGetCmdAcc(byte addr) {  return mod[addr].kind.servo.cmdacc; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetCmdPwm                  <Ldcn.lib>

SYNTAX: byte ServoGetCmdPwm (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the PWM value of the most recently issued
             function ServoLoadTraj () with LOAD_PWM bit set in
             the mode byte for a SERVO module.

RETURN VALUE: the last commanded PWM value sent to a SERVO module
******************************************************************************/

byte ServoGetCmdPwm(byte addr) {  return mod[addr].kind.servo.cmdpwm; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetCmdAdc                  <Ldcn.lib>

SYNTAX: byte ServoGetCmdAdc (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the last commanded absolute position sent with
             function ServoLoadTraj (). This function works with
             LS-173AP servo controller only.

RETURN VALUE: the last commanded absolute position
******************************************************************************/

byte ServoGetCmdAdc(byte addr) { return mod[addr].kind.servo.cmdadc; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetIoCtrl                  <Ldcn.lib>

SYNTAX: byte ServoGetIoCtrl (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the most recently issued I/O
             command control byte for a SERVO module.

RETURN VALUE: the last issued I/O command control byte
******************************************************************************/

byte ServoGetIoCtrl(byte addr) {  return mod[addr].kind.servo.ioctrl; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetHomeCtrl                  <Ldcn.lib>

SYNTAX: byte ServoGetHomeCtrl (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the home mode control byte the most recently
             issued function ServoSetHoming () for a SERVO module.

RETURN VALUE: home mode control byte for a SERVO module.
******************************************************************************/

byte ServoGetHomeCtrl(byte addr) {  return mod[addr].kind.servo.homectrl; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetStopCtrl                  <Ldcn.lib>

SYNTAX:xmem byte ServoGetStopCtrl (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the stop mode control byte of the most recently
             issued function ServoStopMotor() for a SERVO module.

RETURN VALUE: stop mode control byte for a SERVO module
******************************************************************************/

byte ServoGetStopCtrl(byte addr) {  return mod[addr].kind.servo.stopctrl; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetMoveCtrl                  <Ldcn.lib>

SYNTAX: byte ServoGetMoveCtrl (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the move mode control byte of the most recently
             issued function ServoLoadTraj() for a SERVO module.

RETURN VALUE: move mode control byte for a SERVO module
******************************************************************************/

byte ServoGetMoveCtrl(byte addr) {  return mod[addr].kind.servo.movectrl; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetServoInit                  <Ldcn.lib>

SYNTAX: byte ServoGetServoInit (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns Servo init state (1 for servo on power-up)
             of SERVO device with version greater or equal
             than 60 and less than 70.

RETURN VALUE: Servo init state of SERVO device
******************************************************************************/

byte ServoGetServoInit(byte addr) {  return mod[addr].kind.servo.servoinit; }
//-----------------------------------------------------------------------------
/******************************************************************************
ServoGetSDMode                  <Ldcn.lib>

SYNTAX: byte ServoGetSDMode (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns 1 if the Servo module at address addr is in
             Step & Direction mode and 0 otherwise. The module version
             must be greater or equal than 60 and less than 70.

RETURN VALUE: 1 or 0
******************************************************************************/

byte ServoGetSDMode(byte addr) {  return mod[addr].kind.servo.stp_dir_mode; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetGain                  <Ldcn.lib>

SYNTAX: void ServoGetGain (byte addr, int * kp, int * kd, int * ki,
         int * il, byte * ol, byte * cl, int * el, byte * sr, byte * dc);

PARAMETER1:  <addr> - module address
PARAMETER2:  <kp>   - kp
PARAMETER3:  <kd>   - kd
PARAMETER4:  <ki>   - ki
PARAMETER5:  <il>   - il
PARAMETER6:  <ol>   - PWM limit
PARAMETER7:  <cl>   - current limit
PARAMETER8:  <el>   - position error limit
PARAMETER9:  <sr>   - servo rate
PARAMETER10: <dc>   - deadband compensation

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the most recently issued servo gain values for a SERVO module.

RETURN VALUE: none
******************************************************************************/

void ServoGetGain(byte addr, int * kp, int * kd, int * ki,
	 int * il, byte * ol, byte * cl, int * el, byte * sr, byte * dc) {
  *kp = mod[addr].kind.servo.gain.kp;
  *kd = mod[addr].kind.servo.gain.kd;
  *ki = mod[addr].kind.servo.gain.ki;
  *il = mod[addr].kind.servo.gain.il;
  *ol = mod[addr].kind.servo.gain.ol;
  *cl = mod[addr].kind.servo.gain.cl;
  *el = mod[addr].kind.servo.gain.el;
  *sr = mod[addr].kind.servo.gain.sr;
  *dc = mod[addr].kind.servo.gain.dc;
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoSetGain                  <Ldcn.lib>

SYNTAX: bool ServoSetGain (byte addr, int kp, int kd, int ki,
         int il, byte ol, byte cl, int el, byte sr, byte dc);

PARAMETER1:  <addr> - module address
PARAMETER2:  <kp>   - kp
PARAMETER3:  <kd>   - kd
PARAMETER4:  <ki>   - ki
PARAMETER5:  <il>   - il
PARAMETER6:  <ol>   - PWM limit
PARAMETER7:  <cl>   - current limit
PARAMETER8:  <el>   - position error limit
PARAMETER9:  <sr>   - servo rate
PARAMETER10: <dc>   - deadband compensation

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Sets the servo gains for a SERVO module.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoSetGain(byte addr, int kp, int kd, int ki,
         int il, byte ol, byte cl, int el, byte sr, byte dc) {
char cmdstr[16];

  mod[addr].kind.servo.gain.kp = kp;
  mod[addr].kind.servo.gain.kd = kd;
  mod[addr].kind.servo.gain.ki = ki;
  mod[addr].kind.servo.gain.il = il;
  mod[addr].kind.servo.gain.ol = ol;
  mod[addr].kind.servo.gain.cl = cl;
  mod[addr].kind.servo.gain.el = el;
  mod[addr].kind.servo.gain.sr = sr;
  mod[addr].kind.servo.gain.dc = dc;

  *( (int  *)(cmdstr   ) ) = kp;
  *( (int  *)(cmdstr+ 2) ) = kd;
  *( (int  *)(cmdstr+ 4) ) = ki;
  *( (int  *)(cmdstr+ 6) ) = il;
  *( (byte *)(cmdstr+ 8) ) = ol;
  *( (byte *)(cmdstr+ 9) ) = cl;
  *( (int  *)(cmdstr+10) ) = el;
  *( (byte *)(cmdstr+12) ) = sr;
  *( (byte *)(cmdstr+13) ) = dc;

  return LdcnSendCmd(addr, SET_GAIN, cmdstr, 14);
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoLoadTraj                  <Ldcn.lib>

SYNTAX: bool ServoLoadTraj (byte addr, byte mode,
                 long pos, long vel, long acc, byte pwm);

PARAMETER1:  <addr> - module address
PARAMETER2:  <mode> - trajectory control byte
PARAMETER3:  <pos>  - position value
PARAMETER4:  <vel>  - velosity value
PARAMETER5:  <acc>  - acceleration value
PARAMETER6:  <pwm>  - PWM value

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Loads motion trajectory information for a SERVO module. Mode is the
             load trajectory control byte, and should be set to the bitwise
             OR of the trajectory control byte bits. Note that if the START_NOW
             bit is set, the motion will begin immediately.
             If it is not set, the motion will be started  when
             ServoStartMotiont () is called with the module's address.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoLoadTraj(byte addr, byte mode, long pos, long vel, long acc, byte pwm) {
  char cmdstr[16];
  int count;

  mod[addr].kind.servo.movectrl = mode;

  count = 0;
  *( (byte *)(cmdstr + count) ) = mode;  count += 1;

  if (mode & LOAD_POS)  {
    mod[addr].kind.servo.cmdpos = pos;
    *( (long *)(cmdstr + count) ) = pos;
    count += 4;
  }
  if (mode & LOAD_VEL)  {
    mod[addr].kind.servo.cmdvel = vel;
    *( (long *)(cmdstr + count) ) = vel;
    count += 4;
  }
  if (mode & LOAD_ACC)  {
    mod[addr].kind.servo.cmdacc = acc;
    *( (long *)(cmdstr + count) ) = acc;
    count += 4;
  }
  if (mode & LOAD_PWM)  {
    if (mode & LOAD_VEL)
      mod[addr].kind.servo.cmdadc = pwm; // ADC pos
    else
      mod[addr].kind.servo.cmdpwm = pwm;
    *( (byte *)(cmdstr + count) ) = pwm;
    count += 1;
  }

  return LdcnSendCmd(addr, LOAD_TRAJ, cmdstr, (byte)count);
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoStartMotion                  <Ldcn.lib>

SYNTAX: bool ServoStartMotion (byte groupaddr);

PARAMETER1: <groupaddr> - group address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: This will cause a SERVO module to execute
             a previously loaded trajectory.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoStartMotion(byte groupaddr) { return LdcnSendCmd(groupaddr, START_MOVE, NULL, 0); }
//---------------------------------------------------------------------------

/******************************************************************************
ServoResetPos                   <Ldcn.lib>

SYNTAX: bool ServoResetPos (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Resets the 32-bit encoder counter to 0. Also resets the
             internal command position to 0 to prevent the motor from
             jumping abruptly if the position servo is enabled. Do not
             issue this command while executing a motion.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoResetPos(byte addr) {  return LdcnSendCmd(addr, RESET_POS, NULL, 0); }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoClearBits                  <Ldcn.lib>

SYNTAX: bool ServoClearBits (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Clears the sticky status bits (overcurrent and position
             error bits in the status byte and the position wrap and
             servo timer overrun bits in the auxiliary status byte)
             for a SERVO module. They will stay set unless cleared
             explicitly with this command.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoClearBits(byte addr) {  return LdcnSendCmd(addr, CLEAR_BITS, NULL, 0); }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoStopMotor                 <Ldcn.lib>

SYNTAX: bool ServoStopMotor (byte addr, byte mode);

PARAMETER1: <addr> - module address
PARAMETER2: <mode> - mode byte

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Stops a motor in the manner specified by mode.
             Mode is the stop control byte, and should be set
             to the bitwise OR of the stop control bits.

RETURN VALUE: true on success, false on failure
******************************************************************************/

void SetStopCtrl(byte addr, byte mode) {

  if ((mod[addr].modtype != SERVOMODTYPE) &&
      (mod[addr].modtype != SERVOHYBTYPE))
      return;

  mod[addr].kind.servo.stopctrl = mode;

// LS-174
  if (mode & ADV_MODE) if ((mod[addr].modver >=70) && (mod[addr].modver < 80))
     mod[addr].kind.servo.advmode = 1;
}


bool ServoStopMotor(byte addr, byte mode) {
int i;

  mode &= (byte)(~STOP_HERE);

  if (addr & 0x80) {                 // addr is a group address
    for (i = 1; i <= nummod; i++)
      if (mod[i].groupaddr == addr)
         SetStopCtrl(i, mode);
  } else SetStopCtrl(addr, mode);

  return LdcnSendCmd(addr, STOP_MOTOR, (char *)(&mode), 1);
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoSetHoming                  <Ldcn.lib>

SYNTAX: bool ServoSetHoming (byte addr, byte mode);

PARAMETER1: <addr> - module address
PARAMETER2: <mode> - mode byte

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Starts the homing for SERVO module. Mode is the
             homing control byte and should be set to the bitwise
             OR of the homing control bits. This command sets the
             homing conditions but does not start any motion. The
             HOME_IN_PROG bit of the status byte should be monitored
             to detect when the home position has been captured.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoSetHoming(byte addr, byte mode) {
  mod[addr].kind.servo.homectrl = mode;
  return LdcnSendCmd(addr, SET_HOMING, (char *)(&mode), 1);
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoSetOutputs                  <Ldcn.lib>

SYNTAX: bool ServoSetOutputs (byte addr, byte mode, byte out1,
                                  byte out2, byte out3, byte out4);

PARAMETER1: <addr> - module address
PARAMETER2: <mode> - mode byte
PARAMETER3: <out1> - data byte for outport 1
PARAMETER4: <out2> - data byte for outport 2
PARAMETER5: <out3> - data byte for outport 3
PARAMETER6: <out4> - data byte for outport 4

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Sets some of or all four values of auxiliary ports of a
             SERVO device (with version greater or equal than 60 and
             less than 70). Bits 0 and 1 of the mode byte are not used,
             bits 2 and 3 must be set to 1 (= input).  Bits 4 to 7
             define which of outx function parameters to be written to
             the corresponding out port.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoSetOutputs(byte addr, byte mode, byte out1, byte out2, byte out3, byte out4) {
  char cmdstr[8];
  int count;

  cmdstr[0] = mode;
  count = 1;
  if (mode & WR_OUT1)  { mod[addr].kind.servo.outport1 = out1; cmdstr[count++] = out1; }
  if (mode & WR_OUT2)  { mod[addr].kind.servo.outport2 = out2; cmdstr[count++] = out2; }
  if (mode & WR_OUT3)  { mod[addr].kind.servo.outport3 = out3; cmdstr[count++] = out3; }
  if (mode & WR_OUT4)  { mod[addr].kind.servo.outport4 = out4; cmdstr[count++] = out4; }

  mod[addr].kind.servo.ioctrl = mode;
  return LdcnSendCmd(addr, IO_CTRL, cmdstr, (byte)count);
}
//-----------------------------------------------------------------------------
// internal use only for ServoEEPROMCtrl

bool ServoReadEEPROM (byte addr, byte mode) {
// Vars for reading back eeprom data
  char eedata[30];
  byte outstr[5];
  int numrcvd;

  outstr[0] = (char) 0xAA;                  // header byte
  outstr[1] = addr;                         // address byte
  outstr[2] = (byte)(EEPROM_CTRL + 0x10);   // command byte
  outstr[3] = 0;                            // mode = 0
  outstr[4] = (byte)(outstr[1] + outstr[2] + outstr[3]);  //cksum

  SioClrInbuf(ComPort);                     // Get rid of any old input chars
  SioPutChars(ComPort, (char *)outstr, 5);  // Send the command string

// Read back 29 chars prior to receiving status packet:
  numrcvd = SioGetChars(ComPort, eedata, 29);

// Verify enough data was read
  if (numrcvd != 29)   return false;

// Store the data in the module's data structure
  if ((mode==0) || (mode & FETCH_GAINS))
      memcpy( &(mod[addr].kind.servo.gain), eedata, sizeof(GAINVECT));

  if ((mode==0) || (mode & FETCH_VA)) {
      mod[addr].kind.servo.cmdvel = *( (long int *)(eedata + 14) );
      mod[addr].kind.servo.cmdacc = *( (long int *)(eedata + 18) );
  }

  if (mode==0) {
     mod[addr].kind.servo.servoinit    = (byte)eedata[23];
     mod[addr].kind.servo.stp_dir_mode = (byte)eedata[28];
  }

  if ((mode==0) || (mode & FETCH_OUTPUTS)) {
      mod[addr].kind.servo.outport1 = (byte)eedata[24];
      mod[addr].kind.servo.outport2 = (byte)eedata[25];
      mod[addr].kind.servo.outport3 = (byte)eedata[26];
      mod[addr].kind.servo.outport4 = (byte)eedata[27];
  }

// read in the normal status packet
  return ServoGetStat(addr);
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoEEPROMCtrl                  <Ldcn.lib>

SYNTAX: bool ServoEEPROMCtrl (byte addr, byte mode, byte out1,
                                   byte out2, byte out3, byte out4);

PARAMETER1: <addr> - module address
PARAMETER2: <mode> - mode byte
PARAMETER3: <out1> - data byte for outport 1
PARAMETER4: <out2> - data byte for outport 2
PARAMETER5: <out3> - data byte for outport 3
PARAMETER6: <out4> - data byte for outport 4

KEYWORDS: ServoModuleFunctions

DESCRIPTION:Performs  EEPROM  read  or  write  operations  in  a  SERVO module with
            version greater or equal  than 60 and less  than 70. Mode is  the EEPROM
            control byte and should be set  to the bitwise OR of the  EEPROM control
            bits.
            Note: These commands  normally need more  time to be  executed than
            single execution  loop time  and causes  servo_overrun bit  in auxiliary
            status byte to be  set. This  bit can  be cleared with ServoClearBits ()
            function. Do not use this function when servo is on!

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoEEPROMCtrl(byte addr, byte mode, byte out1, byte out2, byte out3, byte out4) {
  char cmdstr[8];
  int count;

  if (mod[addr].modver < 60 || mod[addr].modver >= 70) return(false);   // punt if not EEPROM

  if (mode != 0) {              // normal execution if setting or restoring data
    if ((mode & FETCH_GAINS) || (mode & FETCH_VA) || (mode & FETCH_OUTPUTS))
      if (!ServoReadEEPROM(addr, mode))
         return false;

    cmdstr[0] = mode;
    count = 1;

    if (mode & STORE_SI_BIT)     // update value of servoinit
      mod[addr].kind.servo.servoinit = (mode & INIT_SERVO ? (byte) 1 : (byte) 0);

    if (mode & STORE_OUTPUTS) {
      cmdstr[1] = out1;
      cmdstr[2] = out2;
      cmdstr[3] = out3;
      cmdstr[4] = out4;
      count = 5;
    }
    return LdcnSendCmd(addr, EEPROM_CTRL, cmdstr, (byte)count);
  }
  else   // if mode = 0, read in 28 bytes then get status - USE  mode = 0 ONLY for INITIALIZATION!
    return ServoReadEEPROM(addr, mode);
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoGetNPoints                  <Ldcn.lib>

SYNTAX: byte ServoGetNPoints (byte addr);

PARAMETER1: <addr>  - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Returns the current number of points in the path buffer
             of a SERVO Drive.
             Note: this data is only valid if the SEND_NPOINTS bit of
             statusitems has been set in the most recently issued
             LdcnDefineStatus() function.

RETURN VALUE: current number of points in the path buffer of a SERVO Drive.
******************************************************************************/

byte ServoGetNPoints(byte addr) {  return mod[addr].kind.servo.npoints; }
//-----------------------------------------------------------------------------

/******************************************************************************
ServoSetFastPath                 <Ldcn.lib>

SYNTAX: bool ServoSetFastPath (byte addr, bool fast_path);

PARAMETER1: <addr>      - module address
PARAMETER2: <fast_path> - fast path mode bite

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Set a servo drive advanced mode.
             If parameter <fast_path> is false the drive is set in
             slow path mode (30 Hz / 60 Hz - bit 6 of I/O control
             byte is 0), otherwise the drive is set in fast
             path mode (60Hz / 120 Hz).

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoSetFastPath(byte addr, bool fast_path) {
byte mode;

  mode = fast_path ? FAST_PATH : 0;
  mode |= mod[addr].kind.servo.ioctrl;
  mod[addr].kind.servo.ioctrl = mode;
  return LdcnSendCmd(addr, IO_CTRL, (char *)(&mode), 1);
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoResetRelHome                  <Ldcn.lib>

SYNTAX: bool ServoResetRelHome (byte addr);

PARAMETER1: <addr>  - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Extension of the Reset Position (0x1) command.
             The current position will be set to the difference
             between old current position and the home position.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoResetRelHome(byte addr) {
byte mode;

  mode = REL_HOME;
  return LdcnSendCmd(addr, RESET_POS, (char *)(&mode), 1);
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoInitPath                  <Ldcn.lib>

SYNTAX: void ServoInitPath (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: Saves the current position into a variable that is
             used later by ServoAddPathPoints() function to calculate
             the distance between the last added and the new point and
             the relevant direction. Execute this function only once
             before starting to add points into the drive path buffer.

RETURN VALUE: none
******************************************************************************/

void ServoInitPath(byte addr) {

  LdcnReadStatus(addr, SEND_POS | SEND_PERROR);
  mod[addr].kind.servo.last_ppoint = mod[addr].kind.servo.pos + mod[addr].kind.servo.perror;
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoAddPathPoints                  <Ldcn.lib>

SYNTAX: bool ServoAddPathPoints (byte addr, int npoints,
                                     long *path, bool high_freq);

PARAMETER1: <addr>      - module address
PARAMETER2: <npoints>   - number of points
PARAMETER3: <path>      - points array
PARAMETER4: <high_freq> - frequency type /low, high/

KEYWORDS: ServoModuleFunctions

DESCRIPTION: This function allows the user to add path points into the internal
             path point buffer of the drive.  Up to 7 path points  may be added
             at with  a single command, and multiple commands may  be issued to
             load a total  of 96 path points. Normally,  just one timing mode
             is used in a  path, but the timing can  be mixed. The  timing mode
             depends  on two parameters -high_freq  and  the  fast  parameter
             of ServoSetFastPath() function. If high_freq is  true the  points
             will  be added  with the  higher possible frequency determined by
             the fast parameter - 60 Hz if fast is false  and 120 Hz if fast
             is true.

RETURN VALUE: true on success, false on failure
******************************************************************************/

// Add ABSOLUTE points
bool ServoAddPathPoints(byte addr, int npoints, long *path, bool high_freq) {
char cmdstr[16];
long diff;
int i, shift_bits;
bool rev;

  if (npoints > 7) return false;

  shift_bits = (mod[addr].kind.servo.ioctrl & FAST_PATH) ? 3 : 2;
  if (high_freq) shift_bits++;

  for (i = 0; i < npoints; i++) {
    diff = path[ i] - mod[addr].kind.servo.last_ppoint;

    if (diff < 0) { rev = 0x01;  diff = -diff;
    } else          rev = 0x00;

    if (diff >> ( 16 - shift_bits)) return false;
    diff <<= shift_bits;
    diff  |= rev;
    if (!high_freq) diff |= 0x02;

    *( (int *)(cmdstr + 2*i) ) = (int)diff;
    mod[addr].kind.servo.last_ppoint = path[i];
  }
  return LdcnSendCmd(addr, ADD_PATHPOINT, cmdstr, (byte)(npoints*2));
}
//-----------------------------------------------------------------------------

/******************************************************************************
ServoStartPathMode                  <Ldcn.lib>

SYNTAX: bool ServoStartPathMode (byte groupaddr);

PARAMETER1: <groupaddr> - group address

KEYWORDS: ServoModuleFunctions

DESCRIPTION: To actually start execution of the path in the path
             point buffer, an Add Path Points (0xD) command is issued
             with no path point data included. The groupaddr parameter
             could be a valid group address (to start several
             controllers simultaneously) or any individual address.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool ServoStartPathMode(byte groupaddr) {
   return LdcnSendCmd(groupaddr, ADD_PATHPOINT, NULL, 0);
}


//-----------------------------------------------------------------------------
// S T E P P E R  M O D U L E -------------------------------------------------
//-----------------------------------------------------------------------------

/******************************************************************************
StepsPerSec2StepTime                          <Ldcn.lib>

SYNTAX: unsigned int StepsPerSec2StepTime (double StepsPerSecond,
                                               int SpeedFactor);
PARAMETER1: <StepsPerSecond> - steps per second
PARAMETER2: <SpeedFactor>    - Speed Factor - step time parameter

KEYWORDS: StepperFunctions

DESCRIPTION: Converts a velocity value in steps per second into
             StepTime parameter required by LoadTrajectory command.
             This conversion depends on the step timer speed that is
             defined by <SpeedFactor> parameter:
                SF    Step_Timer_Speed
                0          625,000 Hz
                1        1,250,000 Hz
                2        2,500,000 Hz
                3        5,000,000 Hz

RETURN VALUE:  Step time
******************************************************************************/

unsigned int StepsPerSec2StepTime(double StepsPerSecond, int SpeedFactor) {
long res;

  if (StepsPerSecond == 0.) res = 65452; else
  switch (SpeedFactor) {
    case 0: res = (long)(65536 - ( 625000 / StepsPerSecond) +  2.5); break;
    case 1: res = (long)(65536 - (1250000 / StepsPerSecond) +  4.5); break;
    case 2: res = (long)(65536 - (2500000 / StepsPerSecond) +  8.5); break;
    case 3: res = (long)(65536 - (5000000 / StepsPerSecond) + 16.5); break;
  }
  if (res < 1) res = 1; else
  if (res > 65452) res = 65452;
  return (unsigned int) res;
}
//-----------------------------------------------------------------------------

/******************************************************************************
StepTime2StepsPerSec                    <Ldcn.lib>

SYNTAX: double StepTime2StepsPerSec (unsigned int InitialTimerCount,
                                     int SpeedFactor);

PARAMETER1: <InitialTimerCount> - Step time
PARAMETER2: <SpeedFactor>       - Speed Factor - step time parameter

KEYWORDS: StepperFunctions

DESCRIPTION: Converts a velocity value in StepTime parameter required
             by LoadTrajectory command into steps per second.

RETURN VALUE: steps per second
******************************************************************************/

double StepTime2StepsPerSec(unsigned int InitialTimerCount, int SpeedFactor) {
  switch (SpeedFactor) {
    case 0: return  625000. / (65536 +  2 - InitialTimerCount);
    case 1: return 1250000. / (65536 +  4 - InitialTimerCount);
    case 2: return 2500000. / (65536 +  8 - InitialTimerCount);
    case 3: return 5000000. / (65536 + 16 - InitialTimerCount);
  }
  return 0;
}
//-----------------------------------------------------------------------------

/******************************************************************************
StepsPerSec2mSecPerStep                    <Ldcn.lib>

SYNTAX: double StepsPerSec2mSecPerStep (double StepsPerSecond);

PARAMETER1: <StepsPerSecond> -  Steps Per Second

KEYWORDS: StepperFunctions

DESCRIPTION: Converts a velocity in steps per second into steps per microsecond.

RETURN VALUE: steps per microsecond
******************************************************************************/

double StepsPerSec2mSecPerStep(double StepsPerSecond) {
  return (1000000. / StepsPerSecond);
}
//-----------------------------------------------------------------------------

/******************************************************************************
mSecPerStep2StepsPerSec                    <Ldcn.lib>

SYNTAX: double mSecPerStep2StepsPerSec (double StepsPerSecond);

PARAMETER1: <StepsPerSecond> -  Steps Per Second

KEYWORDS: StepperFunctions

DESCRIPTION: Converts a velocity in steps per microsecond
             into steps per second.

RETURN VALUE: steps per microsecond
******************************************************************************/

double mSecPerStep2StepsPerSec(double mSecPerStep) {
  return  1000000. / mSecPerStep;
}
//-----------------------------------------------------------------------------

/******************************************************************************
MaxStepPeriod                    <Ldcn.lib>

SYNTAX: double MaxStepPeriod (int SpeedFactor);

PARAMETER1: <SpeedFactor> - Speed Factor

KEYWORDS: StepperFunctions

DESCRIPTION: Calculates the maximum Step Period in microseconds

RETURN VALUE: the maximum Step Period in microseconds
******************************************************************************/

double MaxStepPeriod(int SpeedFactor) {
  return StepsPerSec2mSecPerStep(StepTime2StepsPerSec(65452, SpeedFactor));
}
//-----------------------------------------------------------------------------

/******************************************************************************
MinStepPeriod                    <Ldcn.lib>

SYNTAX: double MinStepPeriod (int SpeedFactor);

PARAMETER1: <SpeedFactor> - Speed Factor

KEYWORDS: StepperFunctions

DESCRIPTION: Calculates the minimum Step Period in microseconds

RETURN VALUE: the minimum Step Period in microseconds
******************************************************************************/

double MinStepPeriod(int SpeedFactor) {
  return StepsPerSec2mSecPerStep(StepTime2StepsPerSec(1, SpeedFactor));
}
//-----------------------------------------------------------------------------


//------------------ Initializes STEPMOD structure ----------------------------
void StepNewMod(byte addr) {
  mod[addr].kind.step.pos = 0;
  mod[addr].kind.step.ad = 0;
  mod[addr].kind.step.st = 0;
  mod[addr].kind.step.inbyte = 0;
  mod[addr].kind.step.home = 0;
  mod[addr].kind.step.cmdpos = 0;
  mod[addr].kind.step.cmdspeed = 1;
  mod[addr].kind.step.cmdacc = 1;
  mod[addr].kind.step.cmdst = 1;
  mod[addr].kind.step.st_period = MinStepPeriod(0);
  mod[addr].kind.step.min_speed = 1;
  mod[addr].kind.step.outbyte = 0;
  mod[addr].kind.step.homectrl = HOME_STOP_ABRUPT;
  mod[addr].kind.step.ctrlmode = SPEED_1X | MOFF_STOP;
  mod[addr].kind.step.stopctrl = 0;
  mod[addr].kind.step.run_pwm = 0;
  mod[addr].kind.step.hold_pwm = 0;
  mod[addr].kind.step.therm_limit = 0;
  mod[addr].kind.step.move_mode = 0;
  mod[addr].kind.step.emergency_acc = 255;
  mod[addr].kind.step.stat_io = 0;
}

//-------------------------- STEPPER functions --------------------------------

/******************************************************************************
StepGetStat                  <Ldcn.lib>

SYNTAX: bool StepGetStat (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Receives the status data of a specified STEP device according
             to the last defined by LdcnDefineStatus() status items.

RETURN VALUE: true - on success, false on failure
******************************************************************************/

bool StepGetStat(byte addr) {
  int numbytes, numrcvd;
  int i, bytecount;
  byte cksum;
  byte inbuf[20];

  if (addr & 0x80) return false;              // punt if addr is a group address
  if (mod[addr].modtype != STEPMODTYPE) return false;

// Find number of bytes to read:
  numbytes = 2;       // start with stat & cksum
  if ( (mod[addr].statusitems) & SEND_POS )	numbytes +=4;
  if ( (mod[addr].statusitems) & SEND_AD ) 	numbytes +=1;
  if ( (mod[addr].statusitems) & SEND_ST ) 	numbytes +=2;
  if ( (mod[addr].statusitems) & SEND_INBYTE )  numbytes +=1;
  if ( (mod[addr].statusitems) & SEND_HOME )	numbytes +=4;
  if ( (mod[addr].statusitems) & SEND_ID ) 	numbytes +=2;
  if ( (mod[addr].statusitems) & SEND_OUT ) 	numbytes +=1;
  numrcvd = SioGetChars(ComPort, (char *)inbuf, numbytes);

// Verify enough data was read
  if (numrcvd != numbytes)  return false;

// Verify checksum:
  cksum = 0;
  for (i=0; i<numbytes-1; i++)     cksum = (byte)(cksum + inbuf[i]);
  if (cksum != inbuf[numbytes-1])  return false;

// Verify command was received intact before updating status data
  mod[addr].stat = inbuf[0];
  if (mod[addr].stat & CKSUM_ERROR)  return false;

// Finally, fill in status data
  bytecount = 1;
  if ( (mod[addr].statusitems) & SEND_POS ) {
    mod[addr].kind.step.pos = *( (long *)(inbuf + bytecount) );
    bytecount +=4;
  }
  if ( (mod[addr].statusitems) & SEND_AD ) {
    mod[addr].kind.step.ad = inbuf[bytecount];
    bytecount +=1;
  }
  if ( (mod[addr].statusitems) & SEND_ST ) {
    mod[addr].kind.step.st = *( (unsigned int *)(inbuf + bytecount) );
    bytecount +=2;
  }
  if ( (mod[addr].statusitems) & SEND_INBYTE ) {
    mod[addr].kind.step.inbyte = inbuf[bytecount];
    bytecount +=1;
  }
  if ( (mod[addr].statusitems) & SEND_HOME ) {
    mod[addr].kind.step.home = *( (unsigned long *)(inbuf + bytecount) );
    bytecount +=4;
  }
  if ( (mod[addr].statusitems) & SEND_ID ) {
    mod[addr].modtype = inbuf[bytecount];
    mod[addr].modver = inbuf[bytecount+1];
    bytecount +=2;
  }

  if ( (mod[addr].statusitems) & SEND_OUT ) {
    mod[addr].kind.step.stat_io = inbuf[bytecount];
//    bytecount +=1;
  }

  return true;
}
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetPos                  <Ldcn.lib>

SYNTAX: long StepGetPos (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the current motor position of a STEP module
             Note: this data is valid only if the SEND_POS bit has been
             set in the most recently issued LdcnDefineStatus () command

RETURN VALUE: current motor position of a STEP module

******************************************************************************/

long StepGetPos(byte addr) {  return mod[addr].kind.step.pos; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetAD                  <Ldcn.lib>

SYNTAX: byte StepGetAD (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the current A/D value of a STEP module.
             Note: this data is only valid if the SEND_AD bit has been
             set in the most recently issued LdcnDefineStatus() command.

RETURN VALUE: current A/D value of a STEP module
******************************************************************************/

byte StepGetAD(byte addr) { return mod[addr].kind.step.ad; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetStepTime                          <Ldcn.lib>

SYNTAX: unsigned int StepGetStepTime (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used step time for a STEP module.
             Note: this data is only valid if the SEND_ST bit has been
             set in the most recently issued LdcnDefineStat() command.

RETURN VALUE: used step time for a STEP module
******************************************************************************/

unsigned int StepGetStepTime(byte addr) {  return mod[addr].kind.step.st; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetInbyte                  <Ldcn.lib>

SYNTAX: byte StepGetInbyte (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the input byte value of a STEP module.
             Note: this data is only valid if the SEND_INBYTE bit has been
             set in the most recently issued LdcnDefineStat() command.

RETURN VALUE: input byte value of a STEP module
****************************************************************************/

byte StepGetInbyte(byte addr) {  return mod[addr].kind.step.inbyte; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetHome                  <Ldcn.lib>

SYNTAX: long StepGetHome (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the current motor home position of a STEP module.
             Note: this data is only valid if the SEND_HOME bit has been
             set in the most recently issued LdcnDefineStat() command.

RETURN VALUE: current home position of a STEP module
******************************************************************************/

long StepGetHome(byte addr) {  return mod[addr].kind.step.home; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetCmdPos                  <Ldcn.lib>

SYNTAX: long StepGetCmdPos (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used command position for a STEP module.

RETURN VALUE: command position for a STEP module
******************************************************************************/

long StepGetCmdPos(byte addr) {  return mod[addr].kind.step.cmdpos; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetCmdSpeed                  <Ldcn.lib>

SYNTAX: byte StepGetCmdSpeed (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used command speed for a STEP module.

RETURN VALUE: Last used speed for a STEP module
******************************************************************************/

byte StepGetCmdSpeed(byte addr) {  return mod[addr].kind.step.cmdspeed; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetCmdAcc                  <Ldcn.lib>

SYNTAX: byte StepGetCmdAcc (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used command acceleration for a STEP module.

RETURN VALUE: command acceleration for a STEP module
******************************************************************************/

byte StepGetCmdAcc(byte addr) {  return mod[addr].kind.step.cmdacc; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetCmdST                          <Ldcn.lib>

SYNTAX: unsigned int StepGetCmdST (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used command step time for a STEP module.

RETURN VALUE: command step time for a STEP module.
******************************************************************************/

unsigned int StepGetCmdST(byte addr) {  return mod[addr].kind.step.cmdst; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetStepPeriod                    <Ldcn.lib>

SYNTAX: double StepGetStepPeriod (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used step period in milliseconds
             by StepLoadUnprofiledTraj() function for a STEPPER drive.

RETURN VALUE: used step period
******************************************************************************/

double StepGetStepPeriod(byte addr) {  return mod[addr].kind.step.st_period; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetMinSpeed                  <Ldcn.lib>

SYNTAX: byte StepGetMinSpeed (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used commanded minimal
             velocity for a STEP module.

RETURN VALUE: minimal velocity for a STEP module
******************************************************************************/

byte StepGetMinSpeed(byte addr) {  return mod[addr].kind.step.min_speed; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetOutputs                  <Ldcn.lib>

SYNTAX: byte StepGetOutputs (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently set state of
             an output byte for a STEPPER drive.

RETURN VALUE: output byte for a STEPPER drive
******************************************************************************/

byte StepGetOutputs(byte addr) {  return mod[addr].kind.step.outbyte; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetCtrlMode                  <Ldcn.lib>

SYNTAX: byte StepGetCtrlMode (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the mode control byte of the most recently
             issued function StepSetParam() for a STEPPER drive.

RETURN VALUE: mode control byte for a STEPPER drive
******************************************************************************/

byte StepGetCtrlMode(byte addr) {  return mod[addr].kind.step.ctrlmode; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetMvMode                  <Ldcn.lib>

SYNTAX: byte StepGetMvMode (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used motion mode control byte
             by LoadTrajectory command for a STEPPER drive.

RETURN VALUE: last motion mode for a STEPPER drive
******************************************************************************/

byte StepGetMvMode(byte addr) {  return mod[addr].kind.step.move_mode; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetRunCurrent                  <Ldcn.lib>

SYNTAX: byte StepGetRunCurrent (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used commanded
             running current for a STEP module.

RETURN VALUE: last used commanded running current for a STEP module
******************************************************************************/

byte StepGetRunCurrent(byte addr) {  return mod[addr].kind.step.run_pwm; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetHoldCurrent                  <Ldcn.lib>

SYNTAX: byte StepGetHoldCurrent (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used commanded
             holding current for a STEP module.

RETURN VALUE: last used commanded holding current for a STEP module.
******************************************************************************/

byte StepGetHoldCurrent(byte addr) {  return mod[addr].kind.step.hold_pwm; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetThermLimit                  <Ldcn.lib>

SYNTAX: byte StepGetThermLimit (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the most recently used commanded thermal limit for a STEP module.

RETURN VALUE:  last used commanded thermal limit for a STEP module.
******************************************************************************/

byte StepGetThermLimit(byte addr) {  return mod[addr].kind.step.therm_limit; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetHomeCtrl                  <Ldcn.lib>

SYNTAX: byte StepGetHomeCtrl (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the mode control byte of the most recently
             issued function StepSetHoming() for a STEP module.

RETURN VALUE:  mode control byte for last function StepSetHoming()
******************************************************************************/

byte StepGetHomeCtrl(byte addr) {  return mod[addr].kind.step.homectrl; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetStopCtrl                  <Ldcn.lib>

SYNTAX: byte StepGetStopCtrl (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns the stop control byte of the most recently
             issued function StepStopMotor() for a STEP module.

RETURN VALUE: stop control byte for last function StepStopMotor()
******************************************************************************/

byte StepGetStopCtrl(byte addr) { return mod[addr].kind.step.stopctrl; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetEmAcc                  <Ldcn.lib>

SYNTAX: byte StepGetEmAcc(byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

RETURN VALUE: Returns the most recently used commanded
              emergency acceleratin for a STEP module.
******************************************************************************/

byte StepGetEmAcc(byte addr)  {return mod[addr].kind.step.emergency_acc; }
//-----------------------------------------------------------------------------

/******************************************************************************
StepSetParam                  <Ldcn.lib>

SYNTAX: bool StepSetParam (byte addr, byte mode, byte minspeed, byte runcur,
                           byte holdcur, byte thermlim, byte em_acc);

PARAMETER1: <addr>     - module address
PARAMETER2: <mode>     - control byte
PARAMETER3: <minspeed> - minimum velocity
PARAMETER4: <runcur>   - running current
PARAMETER5: <holdcur>  - holding current
PARAMETER6: <thermlim> - thermal limit
PARAMETER7: <em_acc>   - emergency acceleration

KEYWORDS: StepperFunctions

DESCRIPTION: Sets motion parameters for a STEP module. Mode is the set
             parameters control byte and should be set to the bitwise OR
             of the set parameters control bits. Also sets minimal velocity,
             running current, holding current, and thermal limit.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool StepSetParam(byte addr, byte mode, byte minspeed, byte runcur, byte holdcur, byte thermlim, byte em_acc) {
  char cmdstr[6];

  mod[addr].kind.step.ctrlmode = mode;
  mod[addr].kind.step.min_speed = minspeed;
  mod[addr].kind.step.run_pwm = runcur;
  mod[addr].kind.step.hold_pwm = holdcur;
  mod[addr].kind.step.therm_limit = thermlim;
  mod[addr].kind.step.emergency_acc = em_acc;

  cmdstr[0] = mode;
  cmdstr[1] = minspeed;
  cmdstr[2] = runcur;
  cmdstr[3] = holdcur;
  cmdstr[4] = thermlim;
  cmdstr[5] = em_acc;

  return LdcnSendCmd(addr, SET_PARAM, cmdstr, 6);
}
//-----------------------------------------------------------------------------

/******************************************************************************
StepLoadTraj                  <Ldcn.lib>

SYNTAX: bool StepLoadTraj (byte addr, byte mode, long pos,
                           byte vel, byte acc, unsigned int steptime);

PARAMETER1: <addr>     - module address
PARAMETER2: <mode>     - motion mode
PARAMETER3: <pos>      - position
PARAMETER4: <vel>      - velocity
PARAMETER5: <acc>      - acceleration
PARAMETER6: <steptime> - step time

KEYWORDS: StepperFunctions

DESCRIPTION: Loads motion trajectory information for a STEPPER drive by
             executing LoadTrajectory command. Mode is the load trajectory
             control byte, and should be set to bitwise OR of the trajectory
             control byte bits. Note that if the START_NOW bit is set, the
             motion will begin immediately; if it is not set, the motion will
             be started when StartMotiont command is executed.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool StepLoadTraj(byte addr, byte mode, long pos, byte vel, byte acc, unsigned int steptime) {
  char cmdstr[16];
  byte nearspeed;
  int count;

  if (vel < 1 ) vel = 1;
  if (acc < 1 ) acc = 1;

  switch (mod[addr].kind.step.ctrlmode & 0x03) {
    case SPEED_1X: nearspeed = (byte)( 625000 / (65536 +  2 - steptime) /  25); break;
    case SPEED_2X: nearspeed = (byte)(1250000 / (65536 +  4 - steptime) /  50); break;
    case SPEED_4X: nearspeed = (byte)(2500000 / (65536 +  8 - steptime) / 100); break;
    case SPEED_8X: nearspeed = (byte)(5000000 / (65536 + 16 - steptime) / 200); break;
  }

  nearspeed = (byte)(nearspeed + 0.5);

  count = 0;
  *( (byte *)(cmdstr + count) ) = mode;  count += 1;
  if (mode & LOAD_POS) {
    mod[addr].kind.step.cmdpos = pos;
    *( (long *)(cmdstr + count) ) = pos;
    count += 4;
  }
  if (mode & LOAD_SPEED) {
    mod[addr].kind.step.cmdspeed = vel;
    *( (byte *)(cmdstr + count) ) = vel;
    count += 1;
  }
  if (mode & LOAD_ACC) {
    mod[addr].kind.step.cmdacc = acc;
    *( (byte *)(cmdstr + count) ) = acc;
    count += 1;
  }
  if (mode & LOAD_ST) {
    mod[addr].kind.step.cmdst = steptime;
    *( (int *)(cmdstr + count) ) = steptime;
    count += 2;
    *( (byte *)(cmdstr + count) ) = nearspeed;
    count += 1;
  }

  mod[addr].kind.step.move_mode = mode;
  return LdcnSendCmd(addr, LOAD_TRAJ, cmdstr, (byte)count);
}
//-----------------------------------------------------------------------------

/******************************************************************************
StepLoadUnprofiledTraj                  <Ldcn.lib>

SYNTAX: bool StepLoadUnprofiledTraj (byte addr, byte mode,
                                     long pos, double step_period);

PARAMETER1: <addr>        - module address
PARAMETER2: <mode>        - motion mode control byte
PARAMETER3: <pos>         - position
PARAMETER4: <step_period> - step period

KEYWORDS: StepperFunctions

DESCRIPTION: This function works similarly to StepLoadTraj()  for loading
             only unprofiled motion parameters. To avoid the inconvenience
             steptime parameter StepLoadUnprofiledTraj() uses
             <step_period> - the time for a single step in microseconds.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool StepLoadUnprofiledTraj(byte addr, byte mode, long pos, double step_period) {
  byte SpeedFactor, param_mode;
  unsigned int steptime;

  param_mode = StepGetCtrlMode(addr);
  SpeedFactor = 0;
  switch (param_mode & 0x03) {
    case SPEED_8X: SpeedFactor = 3; break;
    case SPEED_4X: SpeedFactor = 2; break;
    case SPEED_2X: SpeedFactor = 1; break;
    case SPEED_1X: SpeedFactor = 0; break;
  }

  if ((step_period < MaxStepPeriod(SpeedFactor)) || (step_period > MinStepPeriod(SpeedFactor))) return 0;
  steptime = StepsPerSec2StepTime(mSecPerStep2StepsPerSec(step_period), SpeedFactor);
  mode = mode & ~(LOAD_SPEED | LOAD_ACC);
  mod[addr].kind.step.st_period = step_period;
  return StepLoadTraj(addr, mode, pos, 0, 0, steptime);
}
//-----------------------------------------------------------------------------

/******************************************************************************
StepResetPos                  <Ldcn.lib>

SYNTAX: bool StepResetPos (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Resets the current position to 0.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool StepResetPos(byte addr) {  return LdcnSendCmd(addr, RESET_POS, NULL, 0); }
//-----------------------------------------------------------------------------

/******************************************************************************
StepStopMotor                  <Ldcn.lib>

SYNTAX: bool StepStopMotor (byte addr, byte mode);

PARAMETER1: <addr> - module address
PARAMETER2: <mode> - stop mode control byte

KEYWORDS: StepperFunctions

DESCRIPTION: Stops a motor in the manner specified by mode.
             Mode is the stop control byte, and should be set
             to the bitwise OR of the stop control bits.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool StepStopMotor(byte addr, byte mode) {
  mod[addr].kind.step.stopctrl = mode;
  return LdcnSendCmd(addr, STOP_MOTOR, (char *)(&mode), 1);
}
//-----------------------------------------------------------------------------

/******************************************************************************
StepSetOutputs                  <Ldcn.lib>

SYNTAX: bool StepSetOutputs (byte addr, byte outbyte);

PARAMETER1: <addr>    - module address
PARAMETER2: <outbyte> - set value

KEYWORDS: StepperFunctions

DESCRIPTION: Immediately sets the values for the output bits.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool StepSetOutputs(byte addr, byte outbyte) {
  mod[addr].kind.step.outbyte = outbyte;
  return LdcnSendCmd(addr, SET_OUTPUTS, (char *)(&outbyte), 1);
}
//-----------------------------------------------------------------------------

/******************************************************************************
StepSetHoming                  <Ldcn.lib>

SYNTAX: bool StepSetHoming (byte addr, byte mode);

PARAMETER1: <addr> - module address
PARAMETER2: <mode> - homing control byte

KEYWORDS: StepperFunctions

DESCRIPTION: Starts the homing for STEP module. Mode is the homing
             control byte and should be set to the bitwise OR of the
             homing control bits. This command sets the homing conditions,
             but does not start any motion. The HOME_IN_PROG bit of the
             status byte should be monitored to detect when the home
             position has been captured.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool StepSetHoming(byte addr, byte mode) {
  mod[addr].kind.step.homectrl = mode;
  return LdcnSendCmd(addr, SET_HOMING, (char *)(&mode), 1);
}
//-----------------------------------------------------------------------------

/******************************************************************************
StepGetIObyte                  <Ldcn.lib>

SYNTAX: byte StepGetIObyte (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: StepperFunctions

DESCRIPTION: Returns a byte containing the current state of all
             IO ports of a STEPPER drive: Bit 0 - IN 0, bit 1 - IN 1,
             bit 2 - IN 2, bit 3 - OUT 0  bit 7 - OUT 4
             Note: this data is only valid if the SEND_OUT bit of statusitems
             has been set in the most recently issued LdcnDefineStat() function.

RETURN VALUE: byte containing the current state of all IO ports
******************************************************************************/

byte StepGetIObyte(byte addr)  {
  return mod[addr].kind.step.stat_io;
}


//-----------------------------------------------------------------------------
// I O  N O D E  F U N C T I O N S --------------------------------------------
//-----------------------------------------------------------------------------


//--------------------- IO Module initialization ------------------------------

void IoNewMod(byte addr) {
  mod[addr].kind.io.inbits= 0;
  mod[addr].kind.io.ad1   = 0;
  mod[addr].kind.io.ad2   = 0;
  mod[addr].kind.io.ad3   = 0;
  mod[addr].kind.io.timer = 0;
  mod[addr].kind.io.inbits_s = 0;
  mod[addr].kind.io.timer_s  = 0;
  mod[addr].kind.io.pwm1 = 0;
  mod[addr].kind.io.pwm2 = 0;
  mod[addr].kind.io.bitdir = 0x0FFF;
  mod[addr].kind.io.outbits = 0;
  mod[addr].kind.io.timermode = 0;
}
//-----------------------------------------------------------------------------

//-------------------------- IO node functions --------------------------------

/******************************************************************************
IoGetStat                  <Ldcn.lib>

SYNTAX: bool IoGetStat (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: IONodeFunctions

DESCRIPTION: Receives the status data of a specified I/O node according
             to the last defined by LdcnDefineStatus () status items.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool IoGetStat(byte addr) {
  int numbytes, numrcvd;
  int i, bytecount;
  byte cksum;
  byte inbuf[20];

// Find number of bytes to read:
  numbytes = 2;       //start with stat & cksum
  if ( (mod[addr].statusitems) & SEND_INPUTS )   numbytes +=2;
  if ( (mod[addr].statusitems) & SEND_AD1 )      numbytes +=1;
  if ( (mod[addr].statusitems) & SEND_AD2 )      numbytes +=1;
  if ( (mod[addr].statusitems) & SEND_AD3 )      numbytes +=1;
  if ( (mod[addr].statusitems) & SEND_TIMER )    numbytes +=4;
  if ( (mod[addr].statusitems) & SEND_ID )       numbytes +=2;
  if ( (mod[addr].statusitems) & SEND_SYNC_IN )  numbytes +=2;
  if ( (mod[addr].statusitems) & SEND_SYNC_TMR ) numbytes +=4;
  numrcvd = SioGetChars(ComPort, (char *)inbuf, numbytes);

// Verify enough data was read
  if (numrcvd != numbytes) return false;

// Verify checksum:
  cksum = 0;
  for (i=0; i<numbytes-1; i++) cksum = (byte)(cksum + inbuf[i]);
   if (cksum != inbuf[numbytes-1]) return false;

// Verify command was received intact before updating status data
  mod[addr].stat = inbuf[0];
  if (mod[addr].stat & CKSUM_ERROR) return false;

// Finally, fill in status data
  bytecount = 1;
  if ( (mod[addr].statusitems) & SEND_INPUTS )	{
    mod[addr].kind.io.inbits = *( (int *)(inbuf + bytecount) );
    bytecount +=2;
  }
  if ( (mod[addr].statusitems) & SEND_AD1 ) {
    mod[addr].kind.io.ad1 = inbuf[bytecount];
    bytecount +=1;
  }
  if ( (mod[addr].statusitems) & SEND_AD2 ) {
    mod[addr].kind.io.ad2 = inbuf[bytecount];
    bytecount +=1;
  }
  if ( (mod[addr].statusitems) & SEND_AD3 ) {
    mod[addr].kind.io.ad3 = inbuf[bytecount];
    bytecount +=1;
  }
  if ( (mod[addr].statusitems) & SEND_TIMER ) {
    mod[addr].kind.io.timer = *( (unsigned long *)(inbuf + bytecount) );
    bytecount +=4;
  }
  if ( (mod[addr].statusitems) & SEND_ID ) {
    mod[addr].modtype = inbuf[bytecount];
    mod[addr].modver = inbuf[bytecount+1];
    bytecount +=2;
  }
  if ( (mod[addr].statusitems) & SEND_SYNC_IN )	{
    mod[addr].kind.io.inbits_s = *( (int *)(inbuf + bytecount) );
    bytecount +=2;
  }
  if ( (mod[addr].statusitems) & SEND_SYNC_TMR ) {
    mod[addr].kind.io.timer_s = *( (unsigned long *)(inbuf + bytecount) );
    // bytecount +=4;
  }

  return true;
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoInBitVal                  <Ldcn.lib>

SYNTAX: bool IoInBitVal (byte addr, int bitnum);

PARAMETER1: <addr>   - module address
PARAMETER2: <bitnum> - number of input bit

KEYWORDS: IONodeFunctions

DESCRIPTION: Returns the value of input bit <bitnum> from an I/O node.
             Note: this data is only valid if the SEND_INPUTS bit has been
             set in the most recently issued LdcnDefineStat () command.

RETURN VALUE: value of input bit <bitnum>
******************************************************************************/

bool IoInBitVal(byte addr, int bitnum) { return ((mod[addr].kind.io.inbits >> bitnum) & 1); }
//-----------------------------------------------------------------------------

/******************************************************************************
IoInBitSVal                  <Ldcn.lib>

SYNTAX: bool IoInBitSVal (byte addr, int bitnum);

PARAMETER1: <addr>   - module address
PARAMETER2: <bitnum> - number of input bit

KEYWORDS: IONodeFunctions

DESCRIPTION: Returns the value of a synchronously captured input
             bit from an I/O node.
             Note: this data is only valid if the SEND_SYNCH_IN bit has been
             set in the most recently issued LdcnDefineStat () command.

RETURN VALUE: value of a synchronously captured input bit from an I/O node
******************************************************************************/

bool IoInBitSVal(byte addr, int bitnum) { return ((mod[addr].kind.io.inbits_s >> bitnum) & 1); }
//-----------------------------------------------------------------------------

/******************************************************************************
IoOutBitVal                  <Ldcn.lib>

SYNTAX: bool IoOutBitVal (byte addr, int bitnum);

PARAMETER1: <addr>   - module address
PARAMETER2: <bitnum> - number of output bit

KEYWORDS: IONodeFunctions

DESCRIPTION: Returns the most recently set state of an
             output bit <bitnum> of an I/O node.

RETURN VALUE: the last set state of an output bit <bitnum> of an I/O node.
******************************************************************************/

bool IoOutBitVal(byte addr, int bitnum) { return ((mod[addr].kind.io.outbits >> bitnum) & 1); }
//-----------------------------------------------------------------------------

/******************************************************************************
IoSetOutBit                  <Ldcn.lib>

SYNTAX: bool IoSetOutBit (byte addr, int bitnum);

PARAMETER1: <addr>   - module address
PARAMETER2: <bitnum> - number of output bit to be set

KEYWORDS: IONodeFunctions

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool IoSetOutBit(byte addr, int bitnum) {
  if (mod[addr].modver == 1) if (bitnum > 11) return false; else;
  else                       if (bitnum > 6)  return false;

  mod[addr].kind.io.outbits = mod[addr].kind.io.outbits | (int)(1 << bitnum);

  return LdcnSendCmd(addr, SET_OUTPUT, (char *)(&(mod[addr].kind.io.outbits)), 2);
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoGetOutputs                          <Ldcn.lib>

SYNTAX: unsigned int IoGetOutputs (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: IONodeFunctions

RETURN VALUE: Returns the most recently set state of an output byte of an I/O node.
******************************************************************************/

unsigned int IoGetOutputs(byte addr) {
  return ((unsigned int)mod[addr].kind.io.outbits);
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoSetOutputs                  <Ldcn.lib>

SYNTAX: bool IoSetOutputs (byte addr, unsigned int outval);

PARAMETER1: <addr>   - module address
PARAMETER2: <outval> - output value

KEYWORDS: IONodeFunctions

DESCRIPTION: Immediately sets the values for the output bits.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool IoSetOutputs(byte addr, unsigned int outval) {
  outval &= (mod[addr].modver == 1 ? 0x0FFF : 0x7F);
  mod[addr].kind.io.outbits = outval;

  return LdcnSendCmd(addr, SET_OUTPUT, (char *)(&(mod[addr].kind.io.outbits)), 2);
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoClrOutBit                  <Ldcn.lib>

SYNTAX: bool IoClrOutBit (byte addr, int bitnum);

PARAMETER1: <addr>   - module address
PARAMETER2: <bitnum> - bit number to be cleared

KEYWORDS: IONodeFunctions

DESCRIPTION: For module at <addr> clears output bit number <bitnum>

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool IoClrOutBit(byte addr, int bitnum) {
  mod[addr].kind.io.outbits = mod[addr].kind.io.outbits & (int)(~(1 << bitnum));

  return LdcnSendCmd(addr, SET_OUTPUT, (char *)(&(mod[addr].kind.io.outbits)), 2);
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoGetADCVal                  <Ldcn.lib>

SYNTAX: byte IoGetADCVal (byte addr, int channel);

PARAMETER1: <addr>    - module address
PARAMETER2: <channel> - 0,1,2 from an I/O node

KEYWORDS: IONodeFunctions

DESCRIPTION: Returns the A/D value from <channel> (0, 1 ,2 from an I/O node).
             Note: this data is only valid if the SEND_ADn (n = 1, 2 or 3)
                   bit has been set in the most recently issued
                   LdcnDefineStat () command.

RETURN VALUE: A/D value from <channel>
******************************************************************************/

byte IoGetADCVal(byte addr, int channel) {
  switch (channel) {
    case 0: return mod[addr].kind.io.ad1;
    case 1: return mod[addr].kind.io.ad2;
    case 2: return mod[addr].kind.io.ad3;
  }
  return 0;
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoSetPWMVal                  <Ldcn.lib>

SYNTAX: bool IoSetPWMVal (byte addr, byte pwm1, byte pwm2);

PARAMETER1: <addr> - module address
PARAMETER2: <pwm1> - PWM1 Data byte
PARAMETER3: <pwm2> - PWM2 Data byte

KEYWORDS: IONodeFunctions

DESCRIPTION: Sets the PWM values for two channels of an I/O node.
             <pwm1> and <pwm2> should be between 0 and 255.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool IoSetPWMVal(byte addr, byte pwm1, byte pwm2) {
  char cmdstr[2];

  mod[addr].kind.io.pwm1 = pwm1;
  mod[addr].kind.io.pwm2 = pwm2;
  cmdstr[0] = pwm1;
  cmdstr[1] = pwm2;

  return LdcnSendCmd(addr, SET_PWM, cmdstr, 2);
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoSetSynchOutput                  <Ldcn.lib>

SYNTAX: bool IoSetSynchOutput (byte addr, int outbits,
                                   byte pwm1, byte pwm2);

PARAMETER1: <addr>    - module address
PARAMETER2: <outbits> - output bit values
PARAMETER3: <pwm1>    - PWM1 Data byte
PARAMETER4: <pwm2>    - PWM2 Data byte

KEYWORDS: IONodeFunctions

DESCRIPTION: Specify the <output bit> values and the PWM values to be
             output at a later time when LdcnSynchOutput() is called.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool IoSetSynchOutput(byte addr, int outbits, byte pwm1, byte pwm2) {
  char cmdstr[4];

  mod[addr].kind.io.outbits =  outbits;
  mod[addr].kind.io.pwm1 = pwm1;
  mod[addr].kind.io.pwm2 = pwm2;
  cmdstr[0] = ((char *)(&outbits))[0];
  cmdstr[1] = ((char *)(&outbits))[1];
  cmdstr[2] = pwm1;
  cmdstr[3] = pwm2;
  return LdcnSendCmd(addr, SET_SYNCH_OUT, cmdstr, 4);
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoGetPWMVal                  <Ldcn.lib>

SYNTAX: byte IoGetPWMVal (byte addr, int channel);

PARAMETER1: <addr>    - module address
PARAMETER2: <channel> - 0, 1, 2 from an I/O node

KEYWORDS: IONodeFunctions

DESCRIPTION: Returns the most recently set PWM value for
             <channel> 0 or 1 of an I/O node.

RETURN VALUE: the last PWM value
******************************************************************************/

byte IoGetPWMVal(byte addr, int channel) {
  if (channel == 0) return(mod[addr].kind.io.pwm1);
  else return(mod[addr].kind.io.pwm2);
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoSetTimerMode                  <Ldcn.lib>

SYNTAX: bool IoSetTimerMode (byte addr, byte tmrmode);

PARAMETER1: <addr>    - module address
PARAMETER2: <tmrmode> - timer control byte

KEYWORDS: IONodeFunctions

DESCRIPTION: Sets the counter/timer mode for an I/O node.
             <tmrmode> is the timer control byte and should be
             set to the bitwise OR of the timer control bits.

RETURN VALUE: true on success, false on failure
******************************************************************************/

bool IoSetTimerMode(byte addr, byte tmrmode) {
  char cmdstr[2];

  mod[addr].kind.io.timermode = tmrmode;
  cmdstr[0] = tmrmode;

  return LdcnSendCmd(addr, SET_TMR_MODE, cmdstr, 1);
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoGetTimerMode                  <Ldcn.lib>

SYNTAX: byte IoGetTimerMode (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: IONodeFunctions

DESCRIPTION: Returns the most recently set timer control byte for an I/O node.

RETURN VALUE: the last timer control byte
******************************************************************************/

byte IoGetTimerMode(byte addr) {  return mod[addr].kind.io.timermode; }
//-----------------------------------------------------------------------------

/******************************************************************************
IoGetTimerVal                           <Ldcn.lib>

SYNTAX: unsigned long IoGetTimerVal (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: IONodeFunctions

DESCRIPTION: Returns the timer or counter value from an I/O node.
             Note: this data is only valid if the SEND_TIMER bit has been
             set in the most recently issued LdcnDefineStat () command.

RETURN VALUE: Timer or Counter value from an I/O node.
******************************************************************************/

unsigned long IoGetTimerVal(byte addr) {  return(mod[addr].kind.io.timer); }
//-----------------------------------------------------------------------------

/******************************************************************************
IoGetTimerSVal                           <Ldcn.lib>

SYNTAX: unsigned long IoGetTimerSVal (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: IONodeFunctions

DESCRIPTION: Returns the synchronously captured timer or
             counter value from an I/O node.
             Note: this data is only valid if the SEND_SYNC_TMR bit has been
             set in the most recently issued LdcnDefineStat () command.

RETURN VALUE: Synchronously captured timer or counter value from an I/O node
******************************************************************************/

unsigned long IoGetTimerSVal(byte addr) {  return(mod[addr].kind.io.timer_s); }
//-----------------------------------------------------------------------------

/******************************************************************************
IOSynchOutput                  <Ldcn.lib>

SYNTAX: bool IOSynchOutput (byte groupaddr);

PARAMETER1: <groupaddr> - group address

KEYWORDS: IONodeFunctions

DESCRIPTION: Synchronous output command issued to groupaddr. Causes a
             group of modules to synchronously output their previously
             buffered output commands. This will cause an I/O module
             to output previously stored output bit and PWM values.

RETURN VALUE: true on success, false on failure
******************************************************************************/

// Synchronous output command issued to groupaddr.
bool IOSynchOutput(byte groupaddr) { return LdcnSendCmd(groupaddr, SYNCH_OUT, NULL, 0); }
//-----------------------------------------------------------------------------

/******************************************************************************
IoGetBitDir                 <Ldcn.lib>

SYNTAX: int IoGetBitDir (byte addr, int bitnum);

PARAMETER1: <addr>   - module address
PARAMETER2: <bitnum> - bit number

KEYWORDS: IONodeFunctions

RETURN VALUE: returns 0 if a IO module I/O bit  is defined as an output,
                      1 if defined as an input.
******************************************************************************/

int IoGetBitDir(byte addr, int bitnum) { return ((mod[addr].kind.io.bitdir >> bitnum) & 1); }
//-----------------------------------------------------------------------------

/******************************************************************************
IoBitDirOut                 <Ldcn.lib>

SYNTAX: bool IoBitDirOut (byte addr, int bitnum);

PARAMETER1: <addr>   - module address
PARAMETER2: <bitnum> - number of the bit to be set as output

KEYWORDS: IONodeFunctions

DESCRIPTION: Sets the direction of an I/O bit to be an output bit.

RETURN VALUE: true on success, false on failure

******************************************************************************/

int IoBitDirOut(byte addr, int bitnum) {
  mod[addr].kind.io.bitdir = mod[addr].kind.io.bitdir & (int)(~(1 << bitnum));
  return LdcnSendCmd(addr, SET_IO_DIR, (char *)(&(mod[addr].kind.io.bitdir)), 2);
}
//-----------------------------------------------------------------------------

/******************************************************************************
IoBitDirIn                 <Ldcn.lib>

SYNTAX: int IoBitDirIn (byte addr, int bitnum);

PARAMETER1: <addr>   - module address
PARAMETER2: <bitnum> - number of the bit to be set as input

KEYWORDS: IONodeFunctions

DESCRIPTION: Sets the direction of an I/O bit to be an input bit.

RETURN VALUE: true on success, false on failure
******************************************************************************/

int IoBitDirIn(byte addr, int bitnum) {

  mod[addr].kind.io.bitdir = mod[addr].kind.io.bitdir | (int)(1 << bitnum);
  return LdcnSendCmd(addr, SET_IO_DIR, (char *)(&(mod[addr].kind.io.bitdir)), 2);
}

// ------------------------------------------------------------------------
// L D C N C O M  F U N C T I O N S ---------------------------------------
// ------------------------------------------------------------------------

// Initialize misc network variables

int InitLdcnVars(void) {
    int i;

    //segchain _GLOBAL_INIT {  //FIXME this segchain only works in Dynamic C
      ComPort = INVALID_HANDLE_VALUE;
      nummod = max_nummod = SioError = IOBusy = 0;
      BaudRate = 0;
      net_num = 1; 
      LdcnSetNet(1);
    //}

    for (i = 0; i <= max_nummod; i++) {
      mod[i].modtype = 0xFF;
      mod[i].modver = 0;
      mod[i].stat = 0;
      mod[i].statusitems = 0;
      mod[i].groupaddr   = 0xFF;
      mod[i].groupleader = false;
    }

    return 0;
}

//------------------------ LDCNetwork Functions -----------------------------

/******************************************************************************
GetComPort                    <Ldcn.lib>

SYNTAX: handle GetComPort ();

KEYWORDS: LdcNetworkFunctions

RETURN VALUE: The handle of ComPort, opened by LdcnInit()
******************************************************************************/

handle GetComPort() {  return ComPort; }
//-----------------------------------------------------------------------------

/******************************************************************************
GetSioError                 <Ldcn.lib>

SYNTAX: int GetSioError ();

KEYWORDS: LdcNetworkFunctions

RETURN VALUE: The number of serial IO errors
******************************************************************************/

int GetSioError() {  return SioError; }
//-----------------------------------------------------------------------------

/******************************************************************************
IsBusy                 <Ldcn.lib>

SYNTAX: int IsBusy ();

KEYWORDS: LdcNetworkFunctions

RETURN VALUE: returns IO status - busy /true/ or ready /false/
******************************************************************************/

int IsBusy() {  return IOBusy; }
//-----------------------------------------------------------------------------

/******************************************************************************
FixSioError                  <Ldcn.lib>

SYNTAX: void FixSioError ();

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Attempts to re-synch communications sending out to the
             COM port a null string and clears the input buffer.

RETURN VALUE: none
******************************************************************************/

void FixSioError() {
  char teststr[2];
  int i;

// If too many errors - prompt to reset

  if (SioError >= MAXSIOERROR)  return;

// Otherwise, send out a null string to re-synch
  teststr[0] = 0;
  for (i = 0; i < 30; i++) SioPutChars(ComPort, teststr, 1);
  //delay(100); //FIXME this does not exist in Linux
  usleep(100000);  //substitue for delay(100ms) for Linux
  SioClrInbuf(ComPort);
}
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnSendCmd                  <Ldcn.lib>

SYNTAX: bool LdcnSendCmd (byte addr, byte cmd, char *datastr, byte n);

PARAMETER1: <addr>    - receiver module address
PARAMETER2: <cmd>     - command byte
PARAMETER3: <datastr> - sended data structure
PARAMETER4: <n>       - bytes number to send

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Sends to module(s) at address <addr> the command byte <cmd> followed
             by <n> additional data bytes in <datastr> and calculates the number
             of bytes to be received. Shoves the returned status data in the
             internal structure mod []. This function does not make syntax
             analysis of the command and does not store sent values in the
             internal structure mod[]. That's why it is not recommended to use
             this function directly. Better practice is to invoke higher-level
             functions from Servo, Stepper and I/O groups.

RETURN VALUE: true - on success, false on failure
******************************************************************************/

bool LdcnSendCmd(byte addr, byte cmd, char *datastr, byte n ) {
  byte cksum;
  byte i;
  bool iostat;
  byte stataddr;            // address of the "answering" device
  char cstr[30];

  stataddr = 0;
  if (addr < 0x80)          // addr is a individual address
    stataddr = addr;
  else                      // addr is a group address
    for (i=1; i <= nummod; i++)
      if ((mod[i].groupaddr == addr) && (mod[i].groupleader)) // there is a group leader
         { stataddr = i; break; }                             // that will answer

// If too many SIO errors, punt
  if (SioError > MAXSIOERROR)
    return(false);

// Check if data is to be returned from to a known module type
  if ( !( ( mod[stataddr].modtype==STEPMODTYPE) ||
          ( mod[stataddr].modtype==SERVOHYBTYPE) ||
          ( mod[stataddr].modtype==SERVOMODTYPE) ||
          ( mod[stataddr].modtype==IOMODTYPE) ) ) {
      return (false);
  }

// Calculate the adjust command byte, calculate checksum and send the command
  cksum = 0;
  cstr[0] = (char)0xAA;	// start with header byte
  cstr[1] = addr;	// add on address byte
  cksum += cstr[1];
  cstr[2] = (byte)(((n<<4) & 0xF0) | cmd);
  cksum += cstr[2];
  for (i = 0; i < n; i++) {
    cstr[i+3] = datastr[i];
    cksum += cstr[i+3];
  }
  cstr[n+3] = cksum;

  IOBusy = true;

  SioClrInbuf(ComPort);              // Get rid of any old input chars
  SioPutChars(ComPort, cstr, n+4);   // Send the command string

  if (stataddr == 0) {   // If a group command w/ no leader, add delay then exit
    //delay(60); //FIXME this does not exist in Linux
    usleep(60000);  //substitue for delay(ms) for Linux
    IOBusy = false;
    return true;
  }

//  delay( 20);
  switch (mod[stataddr].modtype) {
    case STEPMODTYPE:  iostat = StepGetStat(stataddr);  break;
    case SERVOHYBTYPE:
    case SERVOMODTYPE: iostat = ServoGetStat(stataddr); break;
    case IOMODTYPE:    iostat = IoGetStat(stataddr);    break;
  }

  if (iostat == false) { SioError++;  FixSioError(); }
  else                   SioError = 0;

  IOBusy = false;
  return iostat;
}
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnChangeBaud                  <Ldcn.lib>

SYNTAX: bool LdcnChangeBaud (byte groupaddr, long baudrate);

PARAMETER1: <groupaddr> -  group address
PARAMETER2: <baudrate> -   new baund rate

KEYWORDS: LdcNetworkFunctions

DESCRIPTION:Changes the baud rate of all controllers with group address <groupaddr>
            and also changes host's baud rate. <groupaddr> should include all
            modules. A status packet returned from this command would be at the
            new baud rate, so typically (unless the host's baud rate can be accurately
            synchronized) there should be no group leader when this command is issued.

RETURN VALUE: true - on success, false on failure.
******************************************************************************/

bool LdcnChangeBaud(byte groupaddr, long baudrate) {
  char cstr[5];

  cstr[0] = (char) 0xAA;	                  // Header
  cstr[1] = groupaddr;
  cstr[2] = 0x10 | SET_BAUD;

  if (baudrate ==  9600)          cstr[3] = PB9600;
  else if (baudrate == 19200)     cstr[3] = PB19200;
  else if (baudrate == 57600)     cstr[3] = PB57600;
  else if (baudrate == 115200)    cstr[3] = PB115200;
  else {   cstr[3] = PB19200;     baudrate = 19200; }

  cstr[4] = (byte)(cstr[1] + cstr[2] + cstr[3]);  // checksum
  SioPutChars(ComPort, cstr, 5);                  // send command
  // wait for reset to execute
  //delay(100); //FIXME this does not exist in Linux
  usleep(100000);  //substitue for delay(ms) for Linux
    

  SioChangeBaud(ComPort, baudrate);  //Reset the baud rate to the default

  SioClrInbuf(ComPort);              //clear out any random crap left in buffer
  //delay(100); //FIXME this does not exist in Linux
  usleep(100000);  //substitue for delay(ms) for Linux

  BaudRate = baudrate;
  return true;
}
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnSynchInput                  <Ldcn.lib>

SYNTAX: bool LdcnSynchInput (byte groupaddr);

PARAMETER1: <groupaddr> - Group address

KEYWORDS: LdcNetworkFunctions

DESCRIPTION:Synchronous input command issued to groupaddr. Causes a group of modules
            to synchronously capture input values. This will cause a SERVO module to
            store its current position as the home position and it will cause an I/O
            node to store its input bits and timer values.

RETURN VALUE: true - on success, false on failure
******************************************************************************/
// Synchronous input command issued to groupaddr.

bool LdcnSynchInput(byte groupaddr) { return LdcnSendCmd(groupaddr, SYNCH_INPUT, NULL, 0); }
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnNoOp                  <Ldcn.lib>

SYNTAX: bool LdcnNoOp (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Issued a No Operation command to module(s) at address
             <addr>, simply causing its currently defined items of
             status data to be returned.

RETURN VALUE: true - on success, false on failure

******************************************************************************/

bool LdcnNoOp(byte addr) { return LdcnSendCmd(addr, NOP, NULL, 0); }
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnSetGroupAddr                  <Ldcn.lib>

SYNTAX: bool LdcnSetGroupAddr (byte addr, byte groupaddr, bool leader) ;

PARAMETER1: <addr>      - receiver module address
PARAMETER2: <groupaddr> - New group address /between 0x80 and 0xFF/
PARAMETER3: <leader>    - mark receiver module as leader or not

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Sends the  Set Address  command to  module(s) at  address <addr>. Sets the
             group  address  of  a  controller.  If  leader  is  true,  the specified
             controller becomes the group leader. Only one leader should be  specified
             for any group address.

RETURN VALUE: true - on success, false on failure
******************************************************************************/

bool LdcnSetGroupAddr(byte addr, byte groupaddr, bool leader) {
  char cstr[2];

  if (!(groupaddr & 0x80)) return false;  // punt if not valid group address

  cstr[0] = addr;
  cstr[1] = groupaddr;
  if (leader) cstr[1] &= 0x7F;            // clear upper bit if a group leader
  mod[addr].groupaddr = groupaddr;
  mod[addr].groupleader = leader;
  return LdcnSendCmd(addr, SET_ADDR, cstr, 2);
}
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnHardReset                  <Ldcn.lib>

SYNTAX: bool LdcnHardReset ();

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Reset all controllers - uses universal reset address 0xFF
             and set default values.

RETURN VALUE: true on success, false on failure
******************************************************************************/
// Reset all controllers - uses universal reset address 0xFF

bool LdcnHardReset() {
  char cstr[5];
  int i;

  nummod   = 0;
  SioError = 0;	               // reset the number of errors to 0
  IOBusy = false;

// Send string of 0's to flush input buffers
  cstr[0] = 0;
  for (i = 0; i < 20; i++) SioPutChars(ComPort, cstr, 1);

// Send out reset command string
  cstr[0] = (char) 0xAA;               // Header
  cstr[1] = (char) 0xFF;
  cstr[2] = HARD_RESET;
  cstr[3] = (byte)(cstr[1] + cstr[2]); // checksum
  SioPutChars(ComPort, cstr, 4);       // send reset
// wait for reset to execute
  //delay(100); //FIXME this does not exist in Linux
  usleep(100000);  //substitue for delay(ms) for Linux

  SioChangeBaud(ComPort, 19200);       // Reset the baud rate to the default
  SioClrInbuf(ComPort);                // clear out any random crap left in buffer

  return true;
}
//-------------------------------------------------------------------------

/******************************************************************************
LdcnReadStatus                  <Ldcn.lib>

SYNTAX: bool LdcnReadStatus (byte addr, byte statusitems);

PARAMETER1: <addr>        - module address
PARAMETER2: <statusitems> - status items to be read

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Reads status data from a module without changing the
             default status data. <Statusitems> should be set to
             the bitwise OR of the status items.

RETURN VALUE: true - on success, false on failure
******************************************************************************/

bool LdcnReadStatus(byte addr, byte statusitems) {
  byte oldstat;
  bool retval;
  char cstr[1];

// set bit 7 to 0 for stepper
  statusitems = (mod[addr].modtype == STEPMODTYPE ? statusitems & 0x7F : statusitems);
  cstr[0] = statusitems;
  oldstat = mod[addr].statusitems;
  mod[addr].statusitems = statusitems;
  retval = LdcnSendCmd(addr, READ_STAT, cstr, 1);
  mod[addr].statusitems = oldstat;

  return retval;
}
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnDefineStatus                   <Ldcn.lib>

SYNTAX: bool LdcnDefineStatus (byte addr, byte statusitems);

PARAMETER1: <addr>        - module address
PARAMETER2: <statusitems> - status items to be included in the status packet

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: For module(s) at address <addr>, defines which status
             data will always be sent back with each command by
             sending DefineStatus command. <Statusitems> should be set
             to the bitwise OR of the status items.

RETURN VALUE: true - on success, false on failure
******************************************************************************/

bool LdcnDefineStatus(byte addr, byte statusitems) {
  char cstr[1];

  cstr[0] = statusitems;
  mod[addr].statusitems = statusitems;
  return LdcnSendCmd(addr, DEF_STAT, cstr, 1);
}
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnGetStat                  <Ldcn.lib>

SYNTAX: byte LdcnGetStat (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Returns the last status byte of module at address <addr>.

RETURN VALUE: The last status byte
******************************************************************************/

byte LdcnGetStat(byte addr) {  return mod[addr].stat; }
//-----------------------------------------------------------------------------


/******************************************************************************
LdcnGetStatItems                  <Ldcn.lib>

SYNTAX: byte LdcnGetStatItems (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Returns the byte specifying the default status items
             to be returned in the status data packet - the last
             sent parameter of the DefineStatus command.

RETURN VALUE: Last status item  byte
******************************************************************************/

byte LdcnGetStatItems(byte addr) {  return mod[addr].statusitems; }
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnGetModType                  <Ldcn.lib>

SYNTAX: byte LdcnGetModType (byte addr);

PARAMETER1: <addr> - address

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Returns the module type of a particular module.
             This value will either be SERVOMODTYPE (0), IOMODTYPE (2),
             STEPMODTYPE (3) or SERVOHYBTYPE (90)

RETURN VALUE: module type byte.
******************************************************************************/

byte LdcnGetModType(byte addr) { return mod[addr].modtype; }
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnGetModVer                  <Ldcn.lib>

SYNTAX: byte LdcnGetModVer (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Returns the firmware version number of a particular module.

RETURN VALUE: firmware version number
******************************************************************************/

byte LdcnGetModVer(byte addr) { return mod[addr].modver; }
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnGetGroupAddr                  <Ldcn.lib>

SYNTAX: byte LdcnGetGroupAddr (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: LdcNetworkFunctions

RETURN VALUE: Returns the group address of a particular module.

******************************************************************************/

byte LdcnGetGroupAddr(byte addr) { return mod[addr].groupaddr; }
//-----------------------------------------------------------------------------


/******************************************************************************
LdcnGroupLeader                  <Ldcn.lib>

SYNTAX: bool LdcnGroupLeader (byte addr);

PARAMETER1: <addr> - module address

KEYWORDS: LdcNetworkFunctions

RETURN VALUE: Returns true if the specified module is a group leader, false if not.
******************************************************************************/

bool LdcnGroupLeader(byte addr) { return mod[addr].groupleader; }
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnShutdown                  <Ldcn.lib>

SYNTAX: void LdcnShutdown ();

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Cleans up the internal data structure mod [], resets
             all LDCN modules to their power-up state by sending
             HardReset command and closes previously opened COM port.

RETURN VALUE: none
******************************************************************************/

void LdcnShutdown(void) {
  if ((ComPort!=INVALID_HANDLE_VALUE) && (ComPort!=0)) LdcnHardReset();
  SioClose(ComPort);
  ComPort = INVALID_HANDLE_VALUE;
}
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnInit                 <Ldcn.lib>

SYNTAX: int LdcnInit (int portname, long baudrate);

PARAMETER1: <portname> -  COM port number - to open
PARAMETER2: <baudrate> -  baud rate for group address 0xFF (all devices).

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Initializes all modules on the LDCN network with unique
             sequential addresses starting at 1 and establishes their
             device types. All modules are assigned a group address
             of 0xFF. Device type and version information is stored
             in the internal data structure mod [].

RETURN VALUE: Returns the number of modules found on the network.
******************************************************************************/

int LdcnInit(char *portname, long baudrate) {
  char cstr[6];
  int numread;
  byte addr, i;

  BaudRate = baudrate;
  InitLdcnVars();

  ComPort = SioOpen(portname, 19200);    // Open with default rate of 19200
  if ( ComPort == INVALID_HANDLE_VALUE )  return 0;

  for (i = 0; i <= ABS_MAXNUMMOD; i++) {
    cstr[0] = 0xAA;	         // Header
    cstr[1] = i;
    cstr[2] = CLEAR_BITS;
    cstr[3] = (byte)(cstr[1] + cstr[2]); // checksum
    SioPutChars(ComPort, cstr, 4);       // Send command Clear Sticky Bits
  }

  LdcnHardReset();
  LdcnHardReset();

  addr = 1;
  while (1) {
// First set the address to a unique value:
    cstr[0] = 0xAA;       // Header
    cstr[1] = 0;                 // Send to default address of 0
    cstr[2] = 0x20 | SET_ADDR;
    cstr[3] = addr;              // Set new address sequentially
    cstr[4] = 0xFF;       // Set group address to 0xFF
    cstr[5] = (byte)(cstr[1] + cstr[2] + cstr[3] + cstr[4]);    // checksum
    SioPutChars(ComPort, cstr, 6);                              // Send command

//    delay( 20);
    usleep(20000);  //substitue for delay(20ms) for Linux

    numread = SioGetChars(ComPort, cstr, 2); // get back status, cksum
    if (numread != 2) break;                 // if no response, punt out of loop
    if (cstr[0] != cstr[1]) {
        SioClose(ComPort);
        return(0);
    }


// Read the device type
    cstr[0] = 0xAA;         // Header
    cstr[1] = addr;                // Send to addr
    cstr[2] = 0x10 | READ_STAT;
    cstr[3] = SEND_ID;             // send ID only
    cstr[4] = (byte)(cstr[1] + cstr[2] + cstr[3]);  // checksum
    SioPutChars(ComPort, cstr, 5);                  // Send command

//    delay( 20);
    usleep(20000);  //substitue for delay(20ms) for Linux

    numread = SioGetChars(ComPort, cstr, 4);  // get back stat, type, ver, cksum
    if (numread != 4) {
        SioClose(ComPort);
        return(0);
    }

    if (addr > max_nummod) { printf("ERROR in define of MAXNUMMODx\n"); return 0;}

    mod[addr].stat    = cstr[0];
    mod[addr].modtype = cstr[1];
    mod[addr].modver  = cstr[2];
    mod[addr].statusitems = 0;
    mod[addr].groupaddr   = 0xFF;
    mod[addr].groupleader = false;
    switch (mod[addr].modtype)	{
        case STEPMODTYPE:  StepNewMod(addr); break;
        case SERVOHYBTYPE:
        case SERVOMODTYPE: ServoNewMod(addr); break;
        case IOMODTYPE:    IoNewMod(addr);    break;
    }
    addr++;  // increment the address
  }

  nummod = addr-1;
  if (nummod > 0)
    LdcnChangeBaud(0xFF, baudrate);
  else
    SioClose(ComPort);

  return(nummod);  // Return number of modules
}
//-----------------------------------------------------------------------------

/******************************************************************************
LdcnFullInit                 <Ldcn.lib>

SYNTAX: int LdcnFullInit (int portname, long baudrate);

PARAMETER1: <portname> -  COM port number - to open
PARAMETER2: <baudrate> -  baud rate for group address 0xFF (all devices).

KEYWORDS: LdcNetworkFunctions

DESCRIPTION: Initializes all modules on the LDCN network with unique
             sequential addresses starting at 1 and establishes their
             device types. All modules are assigned a group address
             of 0xFF. Device type and version information is stored
             in the internal data structure mod [].

RETURN VALUE: Returns the number of modules found on the network.
******************************************************************************/

int LdcnFullInit(char *portname, long baudrate) {
  char cstr[6];
  char chbr[6];
  char zstr[20];
  int numread;
  byte addr, i;
  long BaudRates[4];
  int found, br;

  BaudRate = baudrate;

  BaudRates[0] =  19200;
  BaudRates[1] =   9600;
  BaudRates[2] =  57600;
  BaudRates[3] = 115200;

  InitLdcnVars();

  ComPort = SioOpen(portname, BaudRates[0]);  // Open with default rate of 19200
  if ( ComPort == INVALID_HANDLE_VALUE )  return 0;

  for (br = 0; br < 4; br++) {                // Send HardReset at all baudrates
    if (!SioChangeBaud(ComPort, BaudRates[br])) { SioClose(ComPort); return 0;}
    LdcnHardReset();
    for (i = 0; i < ABS_MAXNUMMOD; i++) {
        cstr[0] = (char) 0xAA;	              // Header
        cstr[1] = i;
        cstr[2] = CLEAR_BITS;
        cstr[3] = (byte)(cstr[1] + cstr[2]);  // checksum
       SioPutChars(ComPort, cstr, 4);         // Send command Clear Sticky Bits
    }
    LdcnHardReset();
  }

  for (i = 0; i < 20; i++) zstr[i] = 0;       // Prepare string of 20 zeros
  chbr[0] = (char) 0xAA;                      // Prepare Change Baud Rate command string
  chbr[2] = 0x10 | SET_BAUD;

  if (BaudRate ==  9600)       chbr[3] = PB9600;
  else if (BaudRate == 19200)  chbr[3] = PB19200;
  else if (BaudRate == 57600)  chbr[3] = PB57600;
  else if (BaudRate == 115200) chbr[3] = PB115200;
  else {   BaudRate = 19200;   chbr[3] = PB19200;   }

  found = addr = 1;
// If there are devices with default baud rate 9600: br < 2
  while (found) for (br = 0; br < 1; br++) {
    if (!SioChangeBaud(ComPort, BaudRates[br])) { SioClose(ComPort); return 0; }
// Clear Com port
    SioPutChars(ComPort, zstr, 20);
    SioClrInbuf(ComPort);
// Send Clear Sticky Bits to addr 0
    cstr[0] = (char) 0xAA;	              // Header
    cstr[1] = 0;
    cstr[2] = CLEAR_BITS;
    cstr[3] = (byte)(cstr[1] + cstr[2]);      // checksum
    SioPutChars(ComPort, cstr, 4);            // Send command Clear Sticky Bits
    numread = SioGetChars(ComPort, cstr, 2);

// First set the address to a unique value:
    cstr[0] = (char) 0xAA; 	              // Header
    cstr[1] = 0;  	                      // Send to default address of 0
    cstr[2] = 0x20 | SET_ADDR;
    cstr[3] = addr;  	                      // Set new address sequentially
    cstr[4] = (char) 0xFF;                    // Set group address to 0xFF
    cstr[5] = (byte)(cstr[1] + cstr[2] + cstr[3] + cstr[4]);  // checksum
    SioPutChars(ComPort, cstr, 6);            // Send command
    numread = SioGetChars(ComPort, cstr, 2);  // get back status, cksum
        // For Default 9600 br == 1
    if (numread != 2) { if (br == 0) found = 0;  continue; } // if no response try next baudrate
    if (cstr[0] != cstr[1]) { SioClose(ComPort); return(0); }

// Read the device type
    cstr[0] = (char) 0xAA;  	              // Header
    cstr[1] = addr;  		              // Send to addr
    cstr[2] = 0x10 | READ_STAT;
    cstr[3] = SEND_ID;                        // send ID only
    cstr[4] = (byte)(cstr[1] + cstr[2] + cstr[3]);
    SioPutChars(ComPort, cstr, 5);            // Send command
    numread = SioGetChars(ComPort, cstr, 4);  // get back stat, type, ver, cksum

    if (numread != 4) { SioClose(ComPort); return(0); }
    mod[addr].stat    = cstr[0];
    mod[addr].modtype = cstr[1];
    mod[addr].modver  = cstr[2];
    mod[addr].statusitems = 0;
    mod[addr].groupaddr   = 0xFF;
    mod[addr].groupleader = false;
    switch (mod[addr].modtype)	{
      case SERVOMODTYPE:
      case SERVOHYBTYPE: ServoNewMod(addr); break;
      case IOMODTYPE:    IoNewMod(addr);    break;
      case STEPMODTYPE:  StepNewMod(addr);  break;
    }

    chbr[1] = addr;
    chbr[4] = (byte)(chbr[1] + chbr[2] + chbr[3]);
    SioPutChars(ComPort, chbr, 5);            // send command Change Baud Rate
    // wait for reset to execute
    //delay(100); //FIXME this does not exist in Linux
    usleep(100000);  //substitue for delay(ms) for Linux

    SioClrInbuf(ComPort);

    addr++;		                      // increment the address
    br = 100;                                 // if found at 9600
  }

  nummod = addr-1;
  if (!nummod) { SioClose(ComPort); return 0; }

  SioChangeBaud(ComPort, BaudRate);
// Clear Com port
  SioPutChars(ComPort, zstr, 20);
  SioClrInbuf(ComPort);

  for (addr = 1; addr <= nummod; addr++)  {
      cstr[0] = (char) 0xAA;	              // Header
      cstr[1] = addr;
      cstr[2] = CLEAR_BITS;
      cstr[3] = (byte)(cstr[1] + cstr[2]);    // checksum
      SioPutChars(ComPort, cstr, 4);          // Send command Clear Sticky Bits
      i = SioGetChars(ComPort, cstr, 2);      // get back status, cksum
      //delay(100); //FIXME this does not exist in Linux
      usleep(100000);  //substitue for delay(ms) for Linux
      SioPutChars(ComPort, cstr, 4);          // Send command Clear Sticky Bits
      i = SioGetChars(ComPort, cstr, 2);      // get back status, cksum
  }

  return(nummod);                             // Return number of modules
}


//-----------------------------------------------------------------------------
// COORDINATED MOTION CONTROL -------------------------------------------------
//-----------------------------------------------------------------------------

#define DTOR 0.017453

// Segment types:
#define LINE 0
#define ARC 1

// Values for tangent tolerance
#define TAN_1DEGREE 0.99985
#define TAN_3DEGREE 0.99863
#define TAN_5DEGREE 0.99619
#define TAN_10DEGREE 0.98481
#define TAN_20DEGREE 0.93969
#define TAN_45DEGREE 0.70711

// Data types:

typedef float fp[3];     // floating point 3x1 vector
typedef long int ip[3];  // integer 3x1 vector

typedef struct {         // data type for line segments or arc segments
  int type;              // LINE or ARC
  fp p1;                 // Starting point
  fp p2;                 // Ending point
  fp c;                  // Center point (arcs only)
  fp norm;               // Normal vector (arcs only)
  float len;             // Segment length
  float r;               // Radius (arcs only)
} segment;

typedef struct {         // data type for a coordinate frame
  fp x;
  fp y;
  fp z;
  fp p;
} frame;

//---------------------------------------------------------------------
// Globals:

int pathsize;                 // local path size
segment seglist[MAXSEG];      // list of segments
bool seg_is_added;
int curppoint;                // current pathpoint
float tan_tolerance;          // minimum cos(th) for angle between tangents
frame cur_arcframe;           // coordinate fram for the current arc
float pathlen;                // total path length
float arclen;                 // total length of path already downloaded
float stublen;                // length of path in current segment
int pathfreq;                 // selected path frequency
int bufsize;                  // max num points to store in the PIC-SERVO buffer
float maxvel, vel, acc, decl; // max. velocity, cur velocity, accel & decel
byte x;                       // axes addresses
byte y;
byte z;
byte group;                   // group address for coordinated controllers
int at_end;                   // flags when we are at end of path
float xoff, yoff, zoff;       // origin offset
float UTOCX, UTOCY, UTOCZ;    // Units TO Counts conversion factors
float tolerance;              // small distance tolerance use for near-zero comparisons

int slhead, sltail;           // pointers to head and tail of the segment list
bool IsPathInitialized;
int path_net_num;
//---------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Coordinated motion control functions
//-----------------------------------------------------------------------------


// Pointer functions
// increment segment list head pointer
void inc_slhead()  {  slhead++;  if (slhead == MAXSEG) slhead = 0; }

//-----------------------------------------------------------------------------
// decrement segment list head pointer
void dec_slhead()  {  slhead--;  if (slhead == -1) slhead = MAXSEG - 1; }

//-----------------------------------------------------------------------------
// increment segment list tail pointer
void inc_sltail()  {  sltail++;  if (sltail == MAXSEG) sltail = 0; }

//-----------------------------------------------------------------------------
//return the segment ptr before slhead
int prev_seg()  {
int ptr;

  ptr = slhead - 1;
  if (ptr == -1) ptr = MAXSEG - 1;
  return(ptr);
}


//-----------------------------------------------------------------------------
//  Geometric functions

// Returns the magnitude of a floating point vector
float mag(fp p) { return(sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2])); }

//-----------------------------------------------------------------------------

// Returns the dot product of two floating point vectors
float dot(fp x, fp y) { return( x[0]*y[0] + x[1]*y[1] + x[2]*y[2] ); }

//-----------------------------------------------------------------------------
// Z is returned as the cross product of (x cross y)
void cross(fp x, fp y, fp z) {
  z[0] = x[1]*y[2] - x[2]*y[1];
  z[1] = x[2]*y[0] - x[0]*y[2];
  z[2] = x[0]*y[1] - x[1]*y[0];
}
//-----------------------------------------------------------------------------
// y = normalized x   ( normalize(x,x) works OK )
float normalize(fp x, fp y) {
float a;

  a = mag(x);
  if (a==0.0) return(a);

  y[0] /= a;  y[1] /= a;  y[2] /= a;

  return(a);
}
//-----------------------------------------------------------------------------
// Coordinate frame transformation: y =  F*x, ( y = F*y OK )
void fvmult(frame *F, fp x, fp y) {
fp xtemp;

  xtemp[0] = x[0];
  xtemp[1] = x[1];
  xtemp[2] = x[2];
  y[0] = F->x[0]*xtemp[0] + F->y[0]*xtemp[1] + F->z[0]*xtemp[2] + F->p[0];
  y[1] = F->x[1]*xtemp[0] + F->y[1]*xtemp[1] + F->z[1]*xtemp[2] + F->p[1];
  y[2] = F->x[2]*xtemp[0] + F->y[2]*xtemp[1] + F->z[2]*xtemp[2] + F->p[2];
}
//-----------------------------------------------------------------------------
// Coordinate frame inversion: *B = inverse(A)
void finvert(frame A, frame *B) {
  B->x[1] = A.y[0];
  B->x[2] = A.z[0];
  B->y[2] = A.z[1];
  B->y[0] = A.x[1];
  B->z[0] = A.x[2];
  B->z[1] = A.y[2];
  B->x[0] = A.x[0];
  B->y[1] = A.y[1];
  B->z[2] = A.z[2];
  B->p[0] = -A.x[0]*A.p[0] - A.y[0]*A.p[1] - A.z[0]*A.p[2];
  B->p[1] = -A.x[1]*A.p[0] - A.y[1]*A.p[1] - A.z[1]*A.p[2];
  B->p[2] = -A.x[2]*A.p[0] - A.y[2]*A.p[1] - A.z[2]*A.p[2];
}
//-----------------------------------------------------------------------------
// Get a normal vector tangent to a segment endpoint (=1 for p1, =2 for p2)
// returns -1 if segment length or radius < tolerance, 0 on success
int GetTanVect(segment *s, fp p, int endpoint) {
fp q;

  if (s->type == LINE) {
    p[0] = s->p2[0] - s->p1[0];
    p[1] = s->p2[1] - s->p1[1];
    p[2] = s->p2[2] - s->p1[2];
    if (normalize(p, p) < tolerance) return(-1);
    else return(0);
  } else if (s->type == ARC) {
    if (endpoint == 1) {
      q[0] = s->p1[0] - s->c[0];
      q[1] = s->p1[1] - s->c[1];
      q[2] = s->p1[2] - s->c[2];
    } else {
      q[0] = s->p2[0] - s->c[0];
      q[1] = s->p2[1] - s->c[1];
      q[2] = s->p2[2] - s->c[2];
    }
    if (normalize(q, q) < tolerance) return(-1);
    cross(s->norm, q, p);
    return(0);
  }
  return(0);
}
//-----------------------------------------------------------------------------
// Extract the reference frame for an arc
// Also fills in the radius and arclength

void GetArcFrame(segment *seg, frame *F) {
float q, theta;
fp p;

  if (seg->type != ARC) return;     // punt if not an arc

  F->p[0] = seg->c[0];              // origin is at the center
  F->p[1] = seg->c[1];
  F->p[2] = seg->c[2];
  F->x[0] = seg->p1[0] - seg->c[0]; // X vector points from center to P1
  F->x[1] = seg->p1[1] - seg->c[1];
  F->x[2] = seg->p1[2] - seg->c[2];
  seg->r = normalize(F->x, F->x);   // extract radius and normalize
  q = dot(F->x, seg->norm);         // make sure normal vector is perp. to X
  F->z[0] = seg->norm[0] - q*F->x[0];
  F->z[1] = seg->norm[1] - q*F->x[1];
  F->z[2] = seg->norm[2] - q*F->x[2];
  normalize(F->z, F->z);
  cross(F->z, F->x, F->y);

  p[0] =  seg->p2[0] - seg->c[0];   // get the arclength
  p[1] =  seg->p2[1] - seg->c[1];
  p[2] =  seg->p2[2] - seg->c[2];

  theta = atan2( dot(p,F->y), dot(p,F->x) );
  if (fabs(theta) < 0.001) theta = TWOPI - theta;
  seg->len = fabs(seg->r * theta);
  if (theta < 0) seg->len = (2*seg->r*PI - seg->len);
}
//-----------------------------------------------------------------------------
// Returns a point p which lies on a line segment, and is a distance s from
// the start of the line segment

void GetLineSegPoint(segment *seg, float s, fp p) {
float r;

  r = (seg->len != 0. ?  s/seg->len : 0.);
  p[0] = seg->p1[0] + r*(seg->p2[0] - seg->p1[0]);
  p[1] = seg->p1[1] + r*(seg->p2[1] - seg->p1[1]);
  p[2] = seg->p1[2] + r*(seg->p2[2] - seg->p1[2]);
}
//-----------------------------------------------------------------------------
// Gets the next point in the path.
// returns -1 if already at the end of the path
// returns 1 if the last point in the path
// returns 0 otherwise
int GetNextPathpoint(long int *xp, long int *yp, long int *zp) {
fp p;
int last_seg;

  if (at_end) return(-1);

// First check if decelerating to the endpoint

  if (vel*vel > 2*decl*(pathlen - arclen) ) {
    vel -= decl;
    if (vel<decl) vel = decl;  // use decl value as minimum velocity
  } else if (vel<maxvel) {     // check for acceleration to current maxvel
    vel += acc;
    if (vel>maxvel) vel = maxvel;
  } else if (vel>maxvel) {     // check for deceleration to current maxvel
    vel -= decl;
    if (vel<maxvel) vel = maxvel;
  }

  while (1) {                  // skip over a segment (or more) if necessary
    last_seg = sltail;
    if ( (stublen + vel) > seglist[sltail].len) {  // if past end of segment
      stublen -= seglist[sltail].len;              // subtract off the seg length
      inc_sltail();                                //inc. tail pointer to free up space
      if (sltail == slhead)
        { at_end = 1;                              //check for end of segment list
          break;
        }
      if (seglist[sltail].type == ARC)             // update arcframe if necessary
        GetArcFrame(  &(seglist[sltail]), &cur_arcframe );
    } else {
      stublen += vel;
      arclen  += vel;
      if (arclen > pathlen) at_end = 1;
      break;
    }
  }

  if (at_end) {                // return the final endpoint
    *xp = (long int)( (seglist[last_seg].p2[0] + xoff) * UTOCX );
    *yp = (long int)( (seglist[last_seg].p2[1] + yoff) * UTOCY );
    *zp = (long int)( (seglist[last_seg].p2[2] + zoff) * UTOCZ );
  } else {                     // find the point within the current segment
    if (seglist[sltail].type == LINE) {
      GetLineSegPoint( &(seglist[sltail]), stublen, p );
    } else if (seglist[sltail].type == ARC) {
      p[0] = (float) (seglist[sltail].r != 0. ? seglist[sltail].r * cos(stublen/seglist[sltail].r) : 0.);
      p[1] = (float) (seglist[sltail].r != 0. ? seglist[sltail].r * sin(stublen/seglist[sltail].r) : 0.);
      p[2] = 0;
      fvmult(&cur_arcframe, p, p);
    }
    *xp = (long int)( (p[0] + xoff) * UTOCX );
    *yp = (long int)( (p[1] + yoff) * UTOCY );
    *zp = (long int)( (p[2] + zoff) * UTOCZ );
  }
  return(at_end);
}
//-----------------------------------------------------------------------------

/******************************************************************************
SetTangentTolerance                  <Ldcn.lib>

SYNTAX: void SetTangentTolerance (float theta);

PARAMETER1: <theta> - degree angle

KEYWORDS: CoordinatedMotion

DESCRIPTION: Set allowable angle (in degrees) between continuous path segments

RETURN VALUE: none
******************************************************************************/
// Theta = allowable angle (in degrees) between continuous path segments

void SetTangentTolerance(float theta) {
  tan_tolerance = cos(theta*DTOR);
}
//-----------------------------------------------------------------------------

/******************************************************************************
SetFeedrate                  <Ldcn.lib>

SYNTAX: void SetFeedrate (float fr);

PARAMETER1: <fr> - feedrate value

KEYWORDS: CoordinatedMotion

DESCRIPTION: Set feedrate in units per second

RETURN VALUE: none
******************************************************************************/
// Set feedrate in units per second

void SetFeedrate(float fr) {
  switch (pathfreq) {                    // calculate velocity in units per tick
    case P_30HZ:  maxvel = fr / (1000./(64*0.512)); break;
    case P_60HZ:  maxvel = fr / (1000./(32*0.512)); break;
    case P_120HZ: maxvel = fr / (1000./(16*0.512)); break;
    default:      maxvel = fr / (1000./(64*0.512));
  }
}
//-----------------------------------------------------------------------------

/******************************************************************************
SetPathParams                 <Ldcn.lib>

SYNTAX: int SetPathParams (int freq, int nbuf, int xaxis, int yaxis, int zaxis,
                           int groupaddr,float xscale, float yscale, float zscale,
                           float accel, float decel);

PARAMETER1:  <freq>      - set using defined constants P_30HZ, P_60HZ or P_120HZ
PARAMETER2:  <nbuf>      - number of points to store in the path point buffer (max 87)
PARAMETER3:  <xaxis>     - individual module addresses for the X axes
PARAMETER4:  <yaxis>     - individual module addresses for the Y axes
PARAMETER5:  <zaxis>     - individual module addresses for the Z axes
PARAMETER6:  <groupaddr> - group address
PARAMETER7:  <xscale>    - scale factor for the X axes
PARAMETER8:  <yscale>    - scale factor for the Y axes
PARAMETER9:  <zscale>    - scale factor for the Z axes
PARAMETER10: <accel>     - acceleration in units per tick
PARAMETER11: <decel>     - deceleration in units per tick

KEYWORDS: CoordinatedMotion

DESCRIPTION: Initialize various parameters for this path generation module
             Returns : -1 if Status items are not set properly
                       -2 if scale facotrs are zero
                       -3 if ServoSetFastPath returned False
                        0 if OK

RETURN VALUE:  (-3, -2, -1 or 0)
******************************************************************************/
// Initialize various parameters for this path generation module
// Returns -1 if Status items are not set properly,
//         -2 if scale facotrs are zero


int SetPathParams(int freq, int nbuf,
                  int xaxis, int yaxis, int zaxis, int groupaddr,
                  float xscale, float yscale, float zscale,
                  float accel, float decel) {
byte statitems;
bool OK;
byte sr;
float ACCCOEF, AccCoef;
long dcl;
float real_freq;

  //segchain _GLOBAL_INIT {  //FIXME this segchain only works in Dynamic C
    ACCCOEF = 1000000000000.0/17179869184.; // 10^12/2^34
    tan_tolerance = TAN_3DEGREE;
    pathfreq = P_30HZ;
    x = 1;
    y = 2;
    z = 3;
    group = 0xFF;
    IsPathInitialized = false;
    path_net_num = -1;
  //}

  pathfreq = freq;        // set to 30 or 60 hz
  bufsize = nbuf;         // max num points to store in the PIC-SERVO buffer
  //axes addresses
  x = (byte) xaxis;
  y = (byte) yaxis;
  z = (byte) zaxis;
  if ((x == 0) || (z && !(y))) return -1;

  sr = mod[x].kind.servo.gain.sr;
  if (sr) {
     AccCoef = ACCCOEF / sr;
     dcl = (long) fabs((decel * xscale ) / AccCoef);
     LdcnReadStatus(x, SEND_POS);
     ServoLoadTraj(x, LOAD_POS | LOAD_VEL | LOAD_ACC | START_NOW | ENABLE_SERVO, ServoGetPos(x), dcl, dcl, 0);
  }

  if (y) {
     sr = mod[y].kind.servo.gain.sr;
     if (sr) {
       AccCoef = ACCCOEF / sr;
       dcl = (long) fabs((decel * xscale ) / AccCoef);
       LdcnReadStatus(y, SEND_POS);
       ServoLoadTraj(y, LOAD_POS | LOAD_VEL | LOAD_ACC | START_NOW | ENABLE_SERVO, ServoGetPos(y), dcl, dcl, 0);
     }
  }

  if (z) {
     sr = mod[z].kind.servo.gain.sr;
     if (sr) {
       AccCoef = ACCCOEF / sr;
       dcl = (long) fabs((decel * xscale ) / AccCoef);
       LdcnReadStatus(z, SEND_POS);
       ServoLoadTraj(z, LOAD_POS | LOAD_VEL | LOAD_ACC | START_NOW | ENABLE_SERVO, ServoGetPos(z), dcl, dcl, 0);
     }
  }

  group = (byte)groupaddr;

  LdcnSetGroupAddr(x, group, true);
  if (y) LdcnSetGroupAddr(y, group, false);
  if (z) LdcnSetGroupAddr(z, group, false);

  ServoStopMotor(x, ServoGetStopCtrl(x) | ADV_MODE);
  if (y) ServoStopMotor(y, ServoGetStopCtrl(y) | ADV_MODE);
  if (z) ServoStopMotor(z, ServoGetStopCtrl(z) | ADV_MODE);


  if (freq == P_120HZ) {  // set fast path mode if using 120 Hz path
    OK = ServoSetFastPath(x, true);
    if (y) OK = (OK && ServoSetFastPath(y, true));
    if (z) OK = (OK && ServoSetFastPath(z, true));
  } else {                // otherwise, use slow mode
    OK = ServoSetFastPath(x, false);
    if (y) OK = (OK && ServoSetFastPath(y, false));
    if (z) OK = (OK && ServoSetFastPath(z, false));
  }
  if (!OK) return -3;

  if (fabs(xscale) == 0.0) return(-2);
  if ( y && (fabs(yscale) == 0.0) ) return(-2);
  if ( z && (fabs(zscale) == 0.0) ) return(-2);

  UTOCX = xscale;         // Units To X counts
  UTOCY = yscale;
  UTOCZ = zscale;

// Set the tolerance equivalent to 40 counts of the lowest resolution axis
  tolerance = fabs(40.0/xscale);
  if (y)
    if (tolerance < fabs(40.0/yscale)) tolerance = fabs(40.0/yscale);
  if (z)
    if (tolerance < fabs(40.0/zscale)) tolerance = fabs(40.0/zscale);

//  switch (pathfreq) {     // calculate acceleration in units per tick^2
//    case P_30HZ:  acc  = accel/30.0/30.0;
//                  decl = decel/30.0/30.0;
//                  break;
//    case P_60HZ:  acc  = accel/60.0/60.0;
//                  decl = decel/60.0/60.0;
//                  break;
//    case P_120HZ: acc  = accel/120.0/120.0;
//                  decl = decel/120.0/120.0;
//                  break;
//    default:      acc  = accel/30.0/30.0;
//                  decl = decel/30.0/30.0;
//  }


  switch (pathfreq) {    //calculate acceleration in units per tick^2
    case P_30HZ : real_freq = (1000./(64*0.512)); break;
    case P_60HZ : real_freq = (1000./(32*0.512)); break;
    case P_120HZ: real_freq = (1000./(16*0.512)); break;
    default     : real_freq = (1000./(64*0.512)); break;
  }

  acc  = accel/real_freq/real_freq;
  decl = decel/real_freq/real_freq;

// Check that the required status data will be returned with each command:
  statitems = SEND_POS | SEND_NPOINTS | SEND_PERROR | SEND_AUX;

  LdcnDefineStatus(x, LdcnGetStatItems(x) | statitems);
  if (y) LdcnDefineStatus(y, LdcnGetStatItems(y) | statitems);
  if (z) LdcnDefineStatus(z, LdcnGetStatItems(z) | statitems);

  path_net_num = net_num;
  return(0);
}
//-----------------------------------------------------------------------------

/******************************************************************************
ClearSegList                  <Ldcn.lib>

SYNTAX: void ClearSegList (float xi, float yi, float zi);

PARAMETER1: <xi> - x starting value
PARAMETER2: <yi> - y starting value
PARAMETER3: <zi> - z starting value

KEYWORDS: CoordinatedMotion

DESCRIPTION: Clears the internal segment list and sets the starting
             point for a new set of segments. The new starting point
             (x, y, z) should be in whatever units you choose to use
             for the scale parameters set with SetPathParams ().

RETURN VALUE:  none
******************************************************************************/
// Clear the current segment list and initialize the starting point

void ClearSegList(float xi, float yi, float zi) {

  seg_is_added = 0;
  slhead = 0;
  sltail = 0;
  pathlen = 0;
  seglist[0].p1[0] = xi;
  seglist[0].p1[1] = yi;
  seglist[0].p1[2] = zi;
  IsPathInitialized = false;
}
//-----------------------------------------------------------------------------

/******************************************************************************
AddLineSeg                 <Ldcn.lib>

SYNTAX: int AddLineSeg (float x, float y, float z);

PARAMETER1: <x> - x endpoint of the line
PARAMETER2: <y> - y endpoint of the line
PARAMETER3: <z> - z endpoint of the line

KEYWORDS: CoordinatedMotion

DESCRIPTION: Adds a line segment to the internal segment list.

RETURN VALUE: index of the segment in the segment list if OK
              -1 if segment is not tangent
              -2 if segment list is full
******************************************************************************/

// Add a line segment to the segment list
// Returns: position in segment list if OK
//          -1 if segment is not tangent
//          -2 if segment list is full
// Function assumes the normal vector of any previous arc segment is accurate

int AddLineSeg(float x, float y, float z) {
fp pn, qn;

  inc_slhead();             // test increment the head pointer to check for room
  if (slhead == sltail)
    {
    dec_slhead();
    return(-2);
    }
  dec_slhead();  //  point back to the current position

  seglist[slhead].type = LINE;

  seglist[slhead].p2[0] = x;
  seglist[slhead].p2[1] = y;
  seglist[slhead].p2[2] = z;

  if (seg_is_added) {          // match start point to end point of prev segment
    seglist[slhead].p1[0] = seglist[prev_seg()].p2[0];
    seglist[slhead].p1[1] = seglist[prev_seg()].p2[1];
    seglist[slhead].p1[2] = seglist[prev_seg()].p2[2];
  }

//  Calculate normal vector and length for this segment
  pn[0] = x - seglist[slhead].p1[0];
  pn[1] = y - seglist[slhead].p1[1];
  pn[2] = z - seglist[slhead].p1[2];
  seglist[slhead].len = normalize(pn, pn);

// Check tangency with prev. segment for segments > tolerance:
  if ( (seglist[slhead].len > tolerance) && seg_is_added )
    if ( GetTanVect( &(seglist[prev_seg()]), qn, 2) == 0 )
      if (dot(pn,qn) < tan_tolerance) return(-1);

  pathlen += seglist[slhead].len;      //  increment the overall path length

  inc_slhead();
  seg_is_added = 1;
  return(prev_seg());
}
//-----------------------------------------------------------------------------

// Add an arc segment to the segment list
// Returns: position in segment list if OK
//          -1 if segment is not tangent
//          -2 if segment list is full
//          -3 if arc data invalid
// (Invalid arc data - zero len. normal, radius < tolerance, normal not perp.)
// Function assumes the normal vector of any previous arc segment is accurate

/******************************************************************************
AddArcSeg                 <Ldcn.lib>

SYNTAX: int AddArcSeg (float x, float y, float z, float cx, float cy, float cz,
                           float nx, float ny, float nz);
PARAMETER1: <x>  - x end point value
PARAMETER2: <y>  - y end point value
PARAMETER3: <z>  - z end point value
PARAMETER4: <cx> - x center poin value
PARAMETER5: <cy> - y center poin value
PARAMETER6: <cz> - z center poin value
PARAMETER7: <nx> - x vector normal value
PARAMETER8: <ny> - y vector normal value
PARAMETER9: <nz> - z vector normal value

KEYWORDS: CoordinatedMotion

DESCRIPTION:Adds an arc segment to the internal segment list. Coordinates (x, y,  z)
            are  the  endpoint  of  the  arc,  coordinates (cx,cy,cz) are the center
            point of  the arc,  and coordinates  (nx,ny,nz) form  a vector normal to
            the plane of the  arc. The direction of  the normal vector (positive  or
            negative)  dictates  the  direction  of  the  arc, moving from the start
            point  to  the  endpoint  in  a  right-handed  sense. The end and center
            points coordinates  should be  relative to  the origin,  and in whatever
            units  you   choose  to   use  for   the  scale   parameters  set   with
            SetPathParams().

RETURN VALUE: Index of the segment in the segment list if OK, -3 if arc data is invalid,
           -2 if segment list is full, -1 if segment is not tangent
           (Invalid arc data - zero length, normal or radius < tolerance, normal not perp.)
            Function assumes the normal vector of any previous arc segment is accurate.
******************************************************************************/

int AddArcSeg( float x, float y, float z,      // end point
               float cx, float cy, float cz,   // center point
               float nx, float ny, float nz) { // normal
fp pn, qn;
frame F;

  inc_slhead();            //  test increment the head pointer to check for room
  if (slhead == sltail)
    {
    dec_slhead();
    return(-2);
    }
  dec_slhead();            //  point back to the current position

  seglist[slhead].type = ARC;

  seglist[slhead].p2[0] = x;
  seglist[slhead].p2[1] = y;
  seglist[slhead].p2[2] = z;
  seglist[slhead].c[0] = cx;
  seglist[slhead].c[1] = cy;
  seglist[slhead].c[2] = cz;
  seglist[slhead].norm[0] = nx;
  seglist[slhead].norm[1] = ny;
  seglist[slhead].norm[2] = nz;

  if (seg_is_added) {      //  match start point to end point of prev segment
    seglist[slhead].p1[0] = seglist[prev_seg()].p2[0];
    seglist[slhead].p1[1] = seglist[prev_seg()].p2[1];
    seglist[slhead].p1[2] = seglist[prev_seg()].p2[2];
  }

// Normalize n and punt if too small
  if ( normalize(seglist[slhead].norm, seglist[slhead].norm) < 0.5 )
    return(-3);

// Find radius to p2, and punt if too small
  pn[0] = x - cx;
  pn[1] = y - cy;
  pn[2] = z - cz;
  seglist[slhead].r = normalize(pn, pn);
  if ( seglist[slhead].r < tolerance) return(-3);

// Check if normal is perp to c->p2 vector and punt if not
  if ( fabs(dot(seglist[slhead].norm, pn)) > 0.001 ) return(-3);

// Find radius to p1, and punt if not equal to radius to p2
  pn[0] = seglist[slhead].p1[0] - cx;
  pn[1] = seglist[slhead].p1[1] - cy;
  pn[2] = seglist[slhead].p1[2] - cz;
  if ( fabs(seglist[slhead].r - normalize(pn, pn)) > tolerance) return(-3);

// Check if normal is perp to c->p2 vector and punt if not
  if ( fabs(dot(seglist[slhead].norm, pn)) > 0.001 ) return(-3);

// Check for tangency with prev segment
  if (seg_is_added) {
    GetTanVect( &(seglist[slhead]), pn, 1);                //get current tangent
    if ( GetTanVect( &(seglist[prev_seg()]), qn, 2) == 0 ) // get prev tangent
      if (dot(pn,qn) < tan_tolerance) return(-1);
  }

  GetArcFrame(&(seglist[slhead]), &F);      //  fills in segment length

  pathlen += seglist[slhead].len;           // increment the overall path length

  inc_slhead();
  seg_is_added = 1;
  return(prev_seg());
}
//-----------------------------------------------------------------------------

// Initializes the coordinated path after all of the segments have been added.
// This function should be called just before the application starts calling
// the function AddPathPoints().
// Returns the overall path length for all of the segments.
// Returns 0.0 on communications error

/******************************************************************************
InitPath                   <Ldcn.lib>

SYNTAX: float InitPath ();

KEYWORDS: CoordinatedMotion

DESCRIPTION: Initializes the  coordinated path  after all  of the  segments have been
             added.  This  function  should  be  called  just  before the application
             starts calling the function  AddPathPoints (). Returns the  overall path
             length for all of the segments. Returns 0.0 on communications error

RETURN VALUE: Overall path length for all segments, 0 if found any error
******************************************************************************/

float InitPath() {

  curppoint = 0;
  arclen    = 0.0;
  stublen   = 0.0;
  vel       = 0.0;
  at_end    = 0;

  if (seglist[0].type == ARC) GetArcFrame( &(seglist[0]), &cur_arcframe );

// make sure we exit path mode first
  if        (!ServoStopMotor(x, ServoGetStopCtrl(x) | (byte)SRV_ENABLE_AMP)) return(0.0);
  if (y) if (!ServoStopMotor(y, ServoGetStopCtrl(y) | (byte)SRV_ENABLE_AMP)) return(0.0);
  if (z) if (!ServoStopMotor(z, ServoGetStopCtrl(z) | (byte)SRV_ENABLE_AMP)) return(0.0);

  ServoInitPath(x);     // set the beginning of the path to the current position
  if (y) ServoInitPath(y);
  if (z) ServoInitPath(z);

  xoff = x ? (float)( mod[x].kind.servo.last_ppoint) / UTOCX - seglist[sltail].p1[0] : 0;
  yoff = y ? (float)( mod[y].kind.servo.last_ppoint) / UTOCY - seglist[sltail].p1[1] : 0;
  zoff = z ? (float)( mod[z].kind.servo.last_ppoint) / UTOCZ - seglist[sltail].p1[2] : 0;

  return(pathlen);
}
//-----------------------------------------------------------------------------

// Adds points to path buffer - should be called at regular intervals which
// are shorter than the buffer time (bufsize/pathfreq).
//
// Returns: -1     if path download is done
//          curseg if in middle  of the path
//          -2     if communication error
//          -3

/******************************************************************************
AddPathPoints                 <Ldcn.lib>

SYNTAX: int AddPathPoints ();

KEYWORDS: CoordinatedMotion

DESCRIPTION: Adds path points  to the path  point buffers, and  begins path execution
             automatically after nbuf points have  been added. If the path  is longer
             that can be  held in the  buffers, additional calls  to AddPathPoints ()
             should be  made frequently  enough to  make sure  the buffers  are never
             emptied prematurely.
RETURN VALUE: current segment or
              0 if the path is not complete (ie, more points need to be added)
             -1 if all path points for the current segment list have been downloaded
             -2 on a communications error
******************************************************************************/

int AddPathPoints() {
long int xp[7], yp[7], zp[7];        // set of up to 7 pathpoints
int pcount, res;

  if (path_net_num != net_num) { printf( "SET CORRECT NET NUMBER!!!"); return -4; }
  
  if (!IsPathInitialized) { InitPath(); IsPathInitialized = true; }

// VEL = MAXVEL = 0.0 defines a feedhold condition
// Setting MAXVEL to a non-zero value will resume path execution
  if (maxvel==0.0 && vel == 0.0) return(sltail);

  while (!at_end) {                //  GetNextPathpoint() sets the global at_end
    if (!LdcnNoOp(x)) return(-2);  //  read num points from X

// punt when PIC-SERVO buffer is full
    if ((ServoGetNPoints(x)>bufsize) || (ServoGetNPoints(x)>87)) {
      if (y) if (!LdcnNoOp(y)) return(-2);  //make sure data is updated even if points are not added
      if (z) if (!LdcnNoOp(z)) return(-2);
      break;
    }

    for (pcount=0; pcount < 7; pcount++) {   //  get upto 7 new points
      res = GetNextPathpoint( xp+pcount, yp+pcount, zp+pcount);
      if (res) break;
    }
    if ((pcount < 7) && (res > 0)) pcount++;

    if        (!ServoAddPathPoints(x, pcount, xp, pathfreq != P_30HZ)) return(-3);
    if (y) if (!ServoAddPathPoints(y, pcount, yp, pathfreq != P_30HZ)) return(-3);
    if (z) if (!ServoAddPathPoints(z, pcount, zp, pathfreq != P_30HZ)) return(-3);
  }

  if ( !(ServoGetAux(x) & PATH_MODE))      // start path mode when buffer full
    if (!ServoStartPathMode(group)) return(-2);

  if (at_end) return(-1);
  return(sltail);
}
//-----------------------------------------------------------------------------

/******************************************************************************
SegmentsInBuffer                 <Ldcn.lib>

SYNTAX: int SegmentsInBuffer ();

KEYWORDS: CoordinatedMotion

DESCRIPTION: Returned number of segments in buffer

RETURN VALUE: number of segments in buffer
******************************************************************************/

int SegmentsInBuffer() {

  if (slhead == sltail) return 0;
  if (slhead >  sltail) return (slhead - sltail);
  return MAXSEG - (sltail - slhead) + 1;
}
//-----------------------------------------------------------------------------
