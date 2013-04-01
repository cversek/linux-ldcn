/******************************************************************************
LDCN.LIB version 1.52N

Linux Implementation
Based on Source From http://www.logosolinc.com/software.htm 

author: Craig Wm. Versek, Yankee Environmental Systems 
author_email: cwv@yesinc.com
******************************************************************************/
#ifndef ___Ldcn_h___
#define ___Ldcn_h___
// ----------------------------------------------------------------------------
// G L O B A L S --------------------------------------------------------------
// ----------------------------------------------------------------------------
#ifndef false
  #define false 0
#endif

#ifndef true
  #define true !false
#endif

#define INVALID_HANDLE_VALUE (int) 0

typedef int bool;
typedef int handle;
typedef unsigned char byte;

// ----------------------------------------------------------------------------
// S E R I A L  I O  ----------------------------------------------------------
// ----------------------------------------------------------------------------

// ------------------------ CONSTANTS -----------------------------------------
#define TIME_OUT 100

// ------------------------ FUNCTIONS -----------------------------------------
//handle SioOpen(int port, long baudrate);
handle SioOpen(char *port, long baudrate);
bool   SioChangeBaud( handle ComPort, long baudrate);
bool   SioPutChars( handle ComPort, char *stuff, int n);
int    SioGetChars( handle ComPort, char *stuff, int n);
int    SioTest( handle ComPort);
bool   SioClrInbuf( handle ComPort);
bool   SioClose( handle ComPort);
bool   SioPDmode(int mode);

//-----------------------------------------------------------------------------
// S E R V O   M O D U L E  ---------------------------------------------------
//-----------------------------------------------------------------------------

// ------------------------ CONSTANTS -----------------------------------------
// Servo Module Command set:
#define RESET_POS         0x00  // Reset encoder counter to 0 (0 bytes)
#define SET_ADDR          0x01  // Set address and group address (2 bytes)
#define DEF_STAT          0x02  // Define status items to return (1 byte)
#define READ_STAT         0x03  // Read value of current status items
#define LOAD_TRAJ         0x04  // Load trajectory date (1 - 14 bytes)
#define START_MOVE        0x05  // Start pre-loaded trajectory (0 bytes)
#define SET_GAIN          0x06  // Set servo gain and control parameters (13 or 14)
#define STOP_MOTOR        0x07  // Stop motor (1 byte)
#define IO_CTRL           0x08  // Define bit directions and set output (1 byte)
#define SET_HOMING        0x09  // Define homing mode (1 byte)
#define SET_BAUD          0x0A  // Set the baud rate (1 byte)
#define CLEAR_BITS        0x0B  // Save current pos. in home pos. register (0 bytes)
#define SAVE_AS_HOME      0x0C  // Store the input bytes and timer val (0 bytes)
#define EEPROM_CTRL       0x0D  // Store or retrieve values from EEPROM
#define ADD_PATHPOINT     0x0D  // Adds path popint in path mode
#define NOP               0x0E  // No operation - returns prev. defined status (0 bytes)
#define HARD_RESET        0x0F  // RESET - no status is returned

// Servo Module RESET_POS control byte bit definitions:
// (if no control byte is used, reset is absolute)
#define REL_HOME          0x01  // Reset position relative to current home position

// Servo Module STATUSITEMS bit definitions:
#define SEND_POS          0x01  // 4 bytes data
#define SEND_AD           0x02  // 1 byte
#define SEND_VEL          0x04  // 2 bytes
#define SEND_AUX          0x08  // 1 byte
#define SEND_HOME         0x10  // 4 bytes
#define SEND_ID           0x20  // 2 bytes
#define SEND_PERROR       0x40  // 2 bytes
#define SEND_INPORTS      0x80  // 3 bytes
#define SEND_NPOINTS      0x80  // 1 byte

// Servo Module LOAD_TRAJ control byte bit definitions:
#define LOAD_POS          0x01  // +4 bytes
#define LOAD_VEL          0x02  // +4 bytes
#define LOAD_ACC          0x04  // +4 bytes
#define LOAD_PWM          0x08  // +1 byte
#define ENABLE_SERVO      0x10  // 1 = servo mode, 0 = PWM mode
#define VEL_MODE          0x20  // 1 = velocity mode, 0 = trap. position mode
#define REVERSE           0x40  // 1 = command neg. PWM or vel, 0 = positive
#define MOVE_REL          0x40  // 1 = relative move, 0 = absolute move; works in advanced mode
#define START_NOW         0x80  // 1 = start now, 0 = wait for START_MOVE command

// Servo Module STOP_MOTOR control byte bit definitions:
#define SRV_ENABLE_AMP    0x01  // 1 = raise amp enable output, 0 = lower amp enable
#define MOTOR_OFF         0x02  // set to turn motor off
#define STOP_ABRUPT       0x04  // set to stop motor immediately
#define STOP_SMOOTH       0x08  // set to decellerate motor smoothly
#define STOP_HERE         0x10  // set to stop at position (4 add'l data bytes required)
#define ADV_MODE          0x20  // Enables advanced mode

// Servo Module IO_CTRL control byte bit definitions:
#define SET_OUT1          0x01  // 1 = set limit 1 output, 0 = clear limit 1 output
#define SET_OUT2          0x02  // 1 = set limit 2 output, 0 = clear limit 2 output
#define IO1_IN            0x04  // 1 = limit 1 is an input, 0 = limit 1 is an output
#define IO2_IN            0x08  // 1 = limit 2 is an input, 0 = limit 2 is an output
#define WR_OUT1           0x10  // Write to output port 1
#define WR_OUT2           0x20  // Write to output port 2
#define WR_OUT3           0x40  // Write to output port 3
#define FAST_PATH         0x40  // 0 = 30/60 Hz, 1 = 60/120 Hz
#define WR_OUT4           0x80  // Write to output port 4

// Servo Module SET_HOMING control byte bit definitions:
#define ON_LIMIT1         0x01  // home on change in limit 1
#define ON_LIMIT2         0x02  // home on change in limit 2
#define HOME_MOTOR_OFF    0x04  // turn motor off when homed
#define ON_INDEX          0x08  // home on change in index
#define HOME_STOP_ABRUPT  0x10  // stop abruptly when homed
#define HOME_STOP_SMOOTH  0x20  // stop smoothly when homed
#define ON_POS_ERR        0x40  // home on excessive position error
#define ON_CUR_ERR        0x80  // home on overcurrent error

// Servo Module EEPROM_CTRL control byte bit definitions:
#define STORE_GAINS       0x01  // Store servo gains
#define FETCH_GAINS       0x02  // Retrieve stored gains
#define STORE_VA          0x04  // Store velocity and acceleration
#define FETCH_VA          0x08  // Retrieve stored velocity and acceleration
#define STORE_OUTPUTS     0x10  // Store output port values
#define FETCH_OUTPUTS     0x20  // Retrieve stored output port values
#define STORE_SI_BIT      0x40  // Store "servo initialize" bit
#define INIT_SERVO        0x80  // Initializes servo on power-up

// Servo Module ADD_PATHPOINT frequency definitions
#define P_30HZ              30  //  30 hz path resolution
#define P_60HZ              60  //  60 hz path resolution
#define P_120HZ            120  // 120 hz path resolution

// Servo Module Status byte bit definitions:
#define MOVE_DONE         0x01  // set when move done (trap. pos mode), when goal
                                // vel. has been reached (vel mode) or when not servoing
#define CKSUM_ERROR       0x02  // checksum error in received command
#define OVERCURRENT       0x04  // set on overcurrent condition (sticky bit)
#define POWER_ON          0x08  // set when motor power is on
#define POS_ERR           0x10  // set on excess pos. error (sticky bit)
#define LIMIT1            0x20  // value of limit 1 input
#define LIMIT2            0x40  // value of limit 2 input
#define HOME_IN_PROG      0x80  // set while searching for home, cleared when home found

// Servo Module Auxilliary status byte bit definitions:
#define INDEX             0x01  // value of the encoder index signal
#define POS_WRAP          0x02  // set when 32 bit position counter wraps around
     	     			              // (sticky bit)
#define SERVO_ON          0x04  // set when position servo is operating
#define ACCEL_DONE        0x08  // set when acceleration portion of a move is done
#define SLEW_DONE         0x10  // set when slew portion of a move is done
#define SERVO_OVERRUN     0x20  // set if servo takes longer than the specified
     				                 // servo period to execute
#define PATH_MODE         0x40  // path mode is enabled

// ---------------------- MODULE STRUCTURE ------------------------------------

typedef struct _GAINVECT {
  int  kp;              // gain values
  int  kd;
  int  ki;
  int  il;
  byte ol;
  byte cl;
  int  el;
  byte sr;
  byte dc;
} GAINVECT;

typedef struct _SERVOMOD {
  long   pos;           // current position
  byte   ad;            // a-d value
  int    vel;           // current velocity
  byte   aux;           // auxilliary status byte
  long   home;          // home position
  int    perror;        // position error
  byte   inport1;       // input port 1
  byte   inport2;       // input port 2
  byte   inport3;       // input port 3
// The following data is stored locally for reference
  long   cmdpos;        // last commanded position
  long   cmdvel;        // last commanded velocity
  long   cmdacc;        // last commanded acceleration
  byte   cmdpwm;        // last commanded PWM value
  byte   cmdadc;        // last commanded External position
  GAINVECT   gain;
  long   stoppos;       // motor stop position (used by stop command)
  byte   outport1;      // output port val (used by I-O control)
  byte   outport2;      // output port val (used by I-O control)
  byte   outport3;      // output port val (used by I-O control)
  byte   outport4;      // output port val (used by I-O control)
  byte   stopctrl;      // stop control byte
  byte   movectrl;      // load_traj control byte
  byte   ioctrl;        // I-O control byte
  byte   homectrl;      // homing control byte
  byte   servoinit;     // set to 1 for servo on powerup, zero otherwise
  byte   stp_dir_mode;  // step-direction mode
  byte   advmode;       // 0 - LS-173 mode, 1 - advanced mode
  byte   npoints;       // number of points in path buffer
  long   last_ppoint;   // last path point specified
} SERVOMOD;

// ------------------------ FUNCTIONS -----------------------------------------
void ServoNewMod(byte addr);
//ServoModuleFunctions                 <Ldcn.lib>
bool ServoGetStat(byte addr);
long ServoGetPos(byte addr);
byte ServoGetAD(byte addr);
int  ServoGetVel(byte addr);
byte ServoGetAux(byte addr);
long ServoGetHome(byte addr);
int  ServoGetPError(byte addr);
byte ServoGetInport1(byte addr);
byte ServoGetInport2(byte addr);
byte ServoGetInport3(byte addr);
byte ServoGetOutport1(byte addr);
byte ServoGetOutport2(byte addr);
byte ServoGetOutport3(byte addr);
byte ServoGetOutport4(byte addr);
long ServoGetCmdPos(byte addr);
long ServoGetCmdVel(byte addr);
long ServoGetCmdAcc(byte addr);
byte ServoGetCmdPwm(byte addr);
byte ServoGetCmdAdc(byte addr);
byte ServoGetIoCtrl(byte addr);
byte ServoGetHomeCtrl(byte addr);
byte ServoGetStopCtrl(byte addr);
byte ServoGetMoveCtrl(byte addr);
byte ServoGetServoInit(byte addr);
byte ServoGetSDMode(byte addr);
void ServoGetGain(byte addr, int * kp, int * kd, int * ki, int * il, byte * ol,
                  byte * cl, int * el, byte * sr, byte * dc);
bool ServoSetGain(byte addr, int kp, int kd, int ki, int il, byte ol,
                  byte cl, int el, byte sr, byte dc);
bool ServoLoadTraj(byte addr, byte mode, long pos, long vel, long acc, byte pwm);
bool ServoStartMotion(byte groupaddr);
bool ServoResetPos(byte addr);
bool ServoClearBits(byte addr);
bool ServoStopMotor(byte addr, byte mode);
bool ServoSetHoming(byte addr, byte mode);
bool ServoSetOutputs(byte addr, byte mode, byte out1, byte out2, byte out3, byte out4);
bool ServoEEPROMCtrl(byte addr, byte mode, byte out1, byte out2, byte out3, byte out4);
byte ServoGetNPoints(byte addr);
bool ServoSetFastPath(byte addr, bool fast_path);
bool ServoResetRelHome(byte addr);
void ServoInitPath(byte addr);
bool ServoAddPathPoints(byte addr, int npoints, long *path, bool high_freq);
bool ServoStartPathMode(byte groupaddr);

//-----------------------------------------------------------------------------
// S T E P P E R  M O D U L E -------------------------------------------------
//-----------------------------------------------------------------------------

// ------------------------ CONSTANTS -----------------------------------------
// Step Module Command set:
// #define RESET_POS      0x00  // Reset encoder counter to 0 (0 bytes)
// #define SET_ADDR       0x01  // Set address and group address (2 bytes)
// #define DEF_STAT       0x02  // Define status items to return (1 byte)
// #define READ_STAT      0x03  // Read value of current status items
#define LOAD_TRAJ         0x04  // Load trajectory data
#define START_MOVE        0x05  // Start pre-loaded trajectory (0 bytes)
#define SET_PARAM         0x06  // Set operating parameters (6 bytes)
#define STOP_MOTOR        0x07  // Stop motor (1 byte)
#define SET_OUTPUTS       0x08  // Set output bits (1 byte)
#define SET_HOMING        0x09  // Define homing mode (1 byte)
// #define SET_BAUD       0x0A  // Set the baud rate (1 byte)
#define RESERVED          0x0B  //
#define SAVE_AS_HOME      0x0C  // Store the input bytes and timer val (0 bytes)
#define NOT_USED          0x0D  //
// #define NOP            0x0E  // No operation - returns prev. defined status (0 bytes)
// #define HARD_RESET     0x0F  // RESET - no status is returned

// Step Module STATUSITEMS bit definitions:
#define SEND_POS          0x01  // 4 bytes data
#define SEND_AD           0x02  // 1 byte
#define SEND_ST           0x04  // 2 bytes
#define SEND_INBYTE       0x08  // 1 byte
#define SEND_HOME         0x10  // 4 bytes
// #define SEND_ID        0x20  // 2 bytes
#define SEND_OUT          0x40  // 1 byte

// Step Module LOAD_TRAJ control byte bit definitions:
#define LOAD_POS          0x01  // +4 bytes
#define LOAD_SPEED        0x02  // +1 bytes
#define LOAD_ACC          0x04  // +1 bytes
#define LOAD_ST           0x08  // +3 bytes
#define STEP_REV          0x10  // reverse dir
#define START_NOW         0x80  // 1 = start now, 0 = wait for START_MOVE command

// Step Module SET_PARAM operating mode byte bit definitions:
#define SPEED_8X          0x00  // use 8x timing
#define SPEED_4X          0x01  // use 4x timing
#define SPEED_2X          0x02  // use 2x timing
#define SPEED_1X          0x03  // use 1x timing
#define IGNORE_LIMITS     0x04  // Do not stop automatically on limit switches
#define MOFF_LIMIT        0x08  // Turn Off Motor on Limit
#define MOFF_STOP         0x10  // Turn Off motor on Stop

// Step Module STOP_MOTOR control byte bit definitions:
#define STP_ENABLE_AMP    0x01  // 1 = raise amp enable output, 0 = lower amp enable
#define STOP_ABRUPT       0x04  // set to stop motor immediately
#define STOP_SMOOTH       0x08  // set to decellerate motor smoothly

// Step Module SET_HOMING control byte bit definitions:
#define ON_LIMIT1         0x01  // home on change in limit 1
#define ON_LIMIT2         0x02  // home on change in limit 2
#define HOME_MOTOR_OFF    0x04  // turn motor off when homed
#define ON_HOMESW         0x08  // home on change in index
#define HOME_STOP_ABRUPT  0x10  // stop abruptly when homed
#define HOME_STOP_SMOOTH  0x20  // stop smoothly when homed

// Step Module Status byte bit definitions:
#define MOTOR_MOVING      0x01  // Set when motor is moving
//      CKSUM_ERROR       0x02  // checksum error in received command
#define STP_AMP_ENABLED   0x04  // set if amplifier is enabled
#define POWER_ON          0x08  // set when motor power is on
#define AT_SPEED          0x10  // set if the commanded velocity is reached.
#define VEL_MODE          0x20  // set when in velocity profile mode
#define TRAP_MODE         0x40  // set when in trap. profile mode
#define HOME_IN_PROG      0x80  // set while searching for home, cleared when home found

// Step Module Input byte bit definitions:
#define ESTOP             0x01  // emergency stop input
#define AUX_IN1           0x02  // auxilliary input #1
#define AUX_IN2           0x04  // auxilliary input #2
#define FWD_LIMIT         0x08  // forward limit switch
#define REV_LIMIT         0x10  // reverse limit switch
#define HOME_SWITCH       0x20  // homing limit switch

// ---------------------- MODULE STRUCTURE ------------------------------------

typedef struct _STEPMOD {
  long     pos;         // current position
  byte     ad;          // a/d value
  unsigned int st;      // current step time
  byte     inbyte;      // input bits
  long     home;        // home position
// The following data is stored locally for reference
  long     cmdpos;      // last commanded position
  byte     cmdspeed;    // last commanded speed
  byte     cmdacc;      // last commanded acceleration
  int      cmdst;       // last commanded step time
  double   st_period;
  byte     move_mode;   // last commanded move mode
  byte     min_speed;   // minimum running speed
  byte     stopctrl;    // stop control byte
  byte     outbyte;     // output bits
  byte     homectrl;    // homing control byte
  byte     ctrlmode;    // operating control mode byte
  byte     run_pwm;     // pwm for running current limit
  byte     hold_pwm;    // pwm for holding current limit
  byte     therm_limit; // thermal limit
  byte     emergency_acc;
  byte     stat_io;     // IO byte, returned in status packet
} STEPMOD;

// ------------------------ FUNCTIONS -----------------------------------------
void StepNewMod(byte addr);
// StepperFunctions                 <Ldcn.lib>
unsigned int StepsPerSec2StepTime(double StepsPerSecond, int SpeedFactor);
double StepTime2StepsPerSec(unsigned int InitialTimerCount, int SpeedFactor);
double StepsPerSec2mSecPerStep(double StepsPerSecond);
double mSecPerStep2StepsPerSec(double mSecPerStep);
double MinStepPeriod(int SpeedFactor);
double MaxStepPeriod(int SpeedFactor);

bool StepGetStat(byte addr);
long StepGetPos(byte addr);
byte StepGetAD(byte addr);
unsigned int StepGetStepTime(byte addr);
byte StepGetInbyte(byte addr);
long StepGetHome(byte addr);
long StepGetCmdPos(byte addr);
byte StepGetCmdSpeed(byte addr);
byte StepGetCmdAcc(byte addr);
unsigned int StepGetCmdST(byte addr);
double StepGetStepPeriod(byte addr);
byte StepGetMinSpeed(byte addr);
byte StepGetOutputs(byte addr);
byte StepGetCtrlMode(byte addr);
byte StepGetMvMode(byte addr);
byte StepGetRunCurrent(byte addr);
byte StepGetHoldCurrent(byte addr);
byte StepGetEmAcc(byte addr);
byte StepGetThermLimit(byte addr);
byte StepGetHomeCtrl(byte addr);
byte StepGetStopCtrl(byte addr);
bool StepSetParam(byte addr, byte mode, byte minspeed, byte runcur, byte holdcur, byte thermlim, byte em_acc);
bool StepLoadTraj(byte addr, byte mode, long pos, byte vel, byte acc, unsigned int steptime);
bool StepLoadUnprofiledTraj(byte addr, byte mode, long pos, double step_period);
bool StepResetPos(byte addr);
bool StepStopMotor(byte addr, byte mode);
bool StepSetOutputs(byte addr, byte outbyte);
bool StepSetHoming(byte addr, byte mode);
byte StepGetIObyte(byte addr);

//-----------------------------------------------------------------------------
// I O  N O D E  M O D U L E --------------------------------------------------
//-----------------------------------------------------------------------------

// ------------------------ CONSTANTS -----------------------------------------
// IO Module Command set:
#define	SET_IO_DIR	  0x00	// Set direction of IO bits (2 data bytes)
// #define SET_ADDR       0x01  // Set address and group address (2 bytes)
// #define DEF_STAT       0x02  // Define status items to return (1 byte)
// #define READ_STAT      0x03  // Read value of current status items
#define SET_PWM           0x04  // Immediatley set PWM1 and PWM2 (2 bytes)
#define SYNCH_OUT         0x05  // Output prev. stored PWM & output bytes (0 bytes)
#define SET_OUTPUT        0x06  // Immediately set output bytes
#define SET_SYNCH_OUT     0x07  // Store PWM & outputs for synch'd output (4 bytes)
#define SET_TMR_MODE      0x08  // Set the counter/timer mode (1 byte)
// Not used               0x09
// #define SET_BAUD       0x0A  // Set the baud rate (1 byte)
// Not used               0x0B
#define SYNCH_INPUT       0x0C  // Store the input bytes and timer val (0 bytes)
// Not used               0x0D
// #define NOP            0x0E  // No operation - returns prev. defined status (0 bytes)
// #define HARD_RESET     0x0F  // RESET - no status is returned

// IO Module STATUSITEMS bit definitions
#define SEND_INPUTS       0x01  // 2 bytes data
#define SEND_AD1          0x02  // 1 byte
#define SEND_AD2          0x04  // 1 byte
#define SEND_AD3          0x08  // 1 byte
#define SEND_TIMER        0x10  // 4 bytes
// #define SEND_ID        0x20  // 2 bytes
#define SEND_SYNC_IN      0x40  // 2 bytes
#define SEND_SYNC_TMR     0x80  // 4 bytes

// IO Module Timer mode definitions
// Timer mode and resolution may be OR'd together
#define OFFMODE           0x00
#define COUNTERMODE       0x03
#define TIMERMODE         0x01
#define RESx1             0x00
#define RESx2             0x10
#define RESx4             0x20
#define RESx8             0x30

// ---------------------- MODULE STRUCTURE ------------------------------------

typedef struct _IOMOD {
  int  inbits;          // input bits
  byte ad1;             // A/D input bytes
  byte ad2;
  byte ad3;
  unsigned long timer;  // timer value
  int  inbits_s;        // synchronized input bytes
  unsigned long timer_s;// synchronized timer value
// The following data is stored locally for reference
  byte pwm1;            // current PWM output values
  byte pwm2;
  byte timermode;       // current timer mode
  int bitdir;	 	//current bit direction values
  int  outbits;         // current output byte values
} IOMOD;

// ------------------------ FUNCTIONS -----------------------------------------
void IoNewMod(byte addr);
// IONodeFunctions                 <Ldcn.lib>
bool IoGetStat(byte addr);
bool IoInBitVal(byte addr, int bitnum);
bool IoInBitSVal(byte addr, int bitnum);
bool IoOutBitVal(byte addr, int bitnum);
bool IoSetOutBit(byte addr, int bitnum);
unsigned int IoGetOutputs(byte addr);
bool IoSetOutputs(byte addr, unsigned int outval);
bool IoClrOutBit(byte addr, int bitnum);
byte IoGetADCVal(byte addr, int channel);
bool IoSetPWMVal(byte addr, byte pwm1, byte pwm2);
bool IoSetSynchOutput(byte addr, int outbits, byte pwm1, byte pwm2);
bool IoSetTimerMode(byte addr, byte tmrmode);
byte IoGetTimerMode(byte addr);
unsigned long IoGetTimerVal(byte addr);
byte IoGetPWMVal(byte addr, int channel);
unsigned long IoGetTimerSVal(byte addr);
bool IOSynchOutput(byte groupaddr);
int IoGetBitDir(byte addr, int bitnum);
int IoBitDirOut(byte addr, int bitnum);
int IoBitDirIn(byte addr, int bitnum);

// ----------------------------------------------------------------------------
// L D C N C O M   M O D U L E ------------------------------------------------
// ----------------------------------------------------------------------------

/* ****************************************************************************
 *                                                                            *
 *   The following must be created for each new module type:                  *
 *   - Data structure XXXMOD                                                  *
 *   - Initializer function NewXXXMod                                         *
 *   - Status reading function GetXXXStat                                     *
 *   LdcnInit and SendLdcnCmd must be modified to include calls               *
 *   to the two functions above                                               *
 *                                                                            *
 * ***************************************************************************/

// ------------------------ CONSTANTS -----------------------------------------
#define MAXSIOERROR 3

// Define PIC baud rate divisors
#define	PB9600	           129
#define PB19200             63
#define PB57600             20
#define PB115200            10

// Module type definitions:
#define SERVOMODTYPE         0
#define IOMODTYPE            2
#define STEPMODTYPE          3
#define SERVOHYBTYPE        90

#define ABS_MAXNUMMOD       32

#ifndef MAXNUMMOD
  #define MAXNUMMOD         32
#endif

#ifndef MAXNUMMOD1
  #define MAXNUMMOD1 MAXNUMMOD
#endif

#ifndef MAXNUMMOD2
  #define MAXNUMMOD2 MAXNUMMOD
#endif

#ifndef MAXNUMMOD3
  #define MAXNUMMOD3 MAXNUMMOD
#endif

#ifndef MAXNUMMOD4
  #define MAXNUMMOD4 MAXNUMMOD
#endif

#ifndef MAXNUMNET
  #define MAXNUMNET          1
#endif

typedef struct _LDCNMOD {
  byte modtype;         // module type
  byte modver;          // module version number
  byte statusitems;     // definition of items to be returned
  byte stat;            // status byte
  byte groupaddr;       // current group address
  bool groupleader;     // TRUE if group leader
  union {
    STEPMOD  step;
    SERVOMOD servo;
    IOMOD    io;
  } kind;
} LDCNMOD;

typedef struct _LDCNNET {
  LDCNMOD mod1[MAXNUMMOD1 + 1]; // Array of modules of net 1
#if MAXNUMNET > 1
  LDCNMOD mod2[MAXNUMMOD2 + 1]; // Array of modules of net 2
#endif
#if MAXNUMNET > 2
  LDCNMOD mod3[MAXNUMMOD3 + 1]; // Array of modules of net 3
#endif
#if MAXNUMNET > 3
  LDCNMOD mod4[MAXNUMMOD4 + 1]; // Array of modules of net 4
#endif
  int nummod_A[MAXNUMNET];      // number of modules
  handle ComPort_A[MAXNUMNET];
  long BaudRate_A[MAXNUMNET];
  int SioError_A[MAXNUMNET];
  int IOBusy_A[MAXNUMNET];
} LDCNNET;

// ------------------------ GLOBAL DATA ---------------------------------------

LDCNNET net;
int net_num;

LDCNMOD *mod; // Array of modules
int nummod, max_nummod;
handle ComPort;
char *ComPortDevice; //stores the name of the Linux device file for the current open ComPort
long BaudRate;
int SioError;
int IOBusy;
// ------------------------ FUNCTIONS -----------------------------------------
int  InitLdcnVars(void);
// LdcNetworkFunctions             <Ldcn.lib>
extern long BaudRate;
bool LdcnSetNet(int netN);
handle GetComPort();
int  GetSioError();
int  IsBusy();
void FixSioError();
bool LdcnHardReset();
bool LdcnChangeBaud(byte groupaddr, long baudrate);
int  LdcnInit(char *portname, long baudrate);
int  LdcnFullInit(char *portname, long baudrate);
bool LdcnSendCmd(byte addr, byte cmd, char *datastr, byte n );
bool LdcnSetGroupAddr(byte addr, byte groupaddr, bool leader);
bool LdcnSynchInput(byte groupaddr);
bool LdcnNoOp(byte addr);
bool LdcnReadStatus(byte addr, byte statusitems);
bool LdcnDefineStatus(byte addr, byte statusitems);
byte LdcnGetStat(byte addr);
byte LdcnGetStatItems(byte addr);
byte LdcnGetModType(byte addr);
byte LdcnGetModVer(byte addr);
byte LdcnGetGroupAddr(byte addr);
bool LdcnGroupLeader(byte addr);
void LdcnShutdown();

//-----------------------------------------------------------------------------
// COORDINATED MOTION CONTROL -------------------------------------------------
//-----------------------------------------------------------------------------
#define MAXSEG 100     // Maximum number of segments
#ifndef PI
  #define PI (double) 3.14159265358979323846
#endif
#define TWOPI 6.28319

// ------------------------ FUNCTIONS -----------------------------------------
// CoordinatedMotion                 <Ldcn.lib>
void SetTangentTolerance(float theta);
void SetFeedrate(float fr);
int  SetPathParams(int freq, int nbuf, int xaxis, int yaxis, int zaxis, int groupaddr,
                   float xscale, float yscale, float zscale, float accel, float decel);
void ClearSegList(float xi, float yi, float zi);
int  AddLineSeg(float x, float y, float z);
int  AddArcSeg( float x, float y, float z,     // end point
                float cx, float cy, float cz,  // center point
                float nx, float ny, float nz); // normal
float InitPath();
int   AddPathPoints();
int   SegmentsInBuffer();

#endif // __Ldcn_h__
