// WetaServo library.
// Simon Jansen
// November 2015
// Library to be used with the Weta Teensy servo driver.

#include "Arduino.h"
#include <Servo.h>                        // Servo library.
#include <Dynamixel_Serial_Teensy.h>      // Library needed to control Dynamixal servo.

#ifndef WetaServo_h
#define WetaServo_h
#define HWSERIAL Serial1                  	// Define the serial port used for the Dynamixels.

const int NUMSTANDARDSERVOS = 12;              // Maximum number of standard servos.
const int NUMDYNAMIXELSERVOS = 24;             // Maximum number of Dynamixel servos.

const int SERVO_ControlPin = 0x02;             // Control pin of buffer chip.
const int SERVO_SET_Baudrate = 57600;          // Baud rate speed which the Dynamixels will be set too (57600).
const int CW_LIMIT_ANGLE = 0x000;              // Lowest clockwise angle is 0.
const int CCW_LIMIT_ANGLE = 0xFFF;             // Highest anit-clockwise angle is 0XFFF (4095).
const int MIN_SPEED = 0x01;   				   // Min speed.	
const int MAX_SPEED = 0x3FF;                   // Max speed.   

// Time scaling min and max values.
const int TIME_SCALE_MIN = 1;
const int TIME_SCALE_NORM = 100;
const int TIME_SCALE_MAX = 100;

// Define Dynamixel commands.
const int DWHEEL = 0;						  	// Dynamixel wheel command.
const int DSERVO = 1;						  	// Dynamixel servo command.
const int DTURN = 2;							// Dynamixel turn command.

// Define Dynamixel direction.
const int DLEFT = 0;						  // Dynamixel direction left.
const int DRIGHT = 1;						  // Dynamixel direction right.	

// Define Dynamixel servicing modes.
const int MNONE = 0;						  // No mode.
const int MTIMED = 1;						  // Timed mode.
const int MSTEPS = 2;						  // Steps mode.
const int MSERVO = 3;						  // Servo mode.

const int MAX_STRING = 32;					  // Dynamixel variable string lengths.
	
////////////////////////////////////////////////////////////
// StandardServo class.
////////////////////////////////////////////////////////////  
class StandardServo 
{
  private:
  byte pin;
  Servo ser;
  byte minPos;
  byte maxPos;
 
  public:
  StandardServo( byte pin ); 
  void MoveTo( byte pos );
  void SetMinMax( byte min, byte max );
};

////////////////////////////////////////////////////////////
// DynamixelServo class.
////////////////////////////////////////////////////////////  
class DynamixelServo 
{
  private:
  // Private member variables.
  char name[MAX_STRING];
  char cw_string[MAX_STRING];
  char ccw_string[MAX_STRING];
  byte id;
  byte mode;
  byte direction;
  byte lookupTableIndex;
  bool needsService;  
  bool accelerating;
  bool decelerating;  
  bool atMaxSpeed;
  bool enabled;
  int cw_limit;
  int ccw_limit;
  int time_scale;
  int cw_speed;
  int ccw_speed;
  int acdc;
  int serviceMode;
  int initialPosition;
  int goalPosition;
  int newPosition;
  int oldPosition; 
  long stepsAccDcc;  
  long steps;
  long stepsToRun;
  long totalSteps;  
  unsigned long timeStart;
  unsigned long timeToRun;   
  unsigned long timeAccDcc;
  unsigned long decStart;  
  unsigned long increment;
  unsigned long acdcStopActual;
  unsigned long decStartActual;
  float turns;
  float turnsToRun;
  
  // Private functions.  
  unsigned long ScaleTime(unsigned long tm);
  unsigned long ScaleSpeed(int spd);
  bool Wrapped(int value, int oldValue);  
  void StartAcc();
  void StartDcc();
  void StopAccDcc();  
  int NextStep();
  int NextServoStep();
  void DoServo(int pos, bool preLoad);
  
  public:
  
  // Constructor.
  DynamixelServo(byte id, const char* name, const char* cw_string, const char* ccw_string);
  
  // Public functions.
  void Start(byte mode, int cw_speed, int ccw_speed, int acdc, int cw_limit, int ccw_limit);
  
  void Enable(bool enabled);
  bool SetDriveMode(byte drive_mode);
  void SetCWSpeed(int speed);
  void SetCCWSpeed(int speed);
  void SetDirection(byte dir);   
  bool SetCWLimit(int cw_limit);
  bool SetCCWLimit(int ccw_limit);
  void SetACDC(int acdc);
  void SetMode(int mode);  
  void SetHolding(bool hold);  
  void SetTimeScale(int scale);
  bool SetPID(byte P, byte I, byte D);
  
  byte GetID();
  char* GetName();
  char* GetCWString();
  char* GetCCWString();
  int GetMode();
  int GetCWSpeed();
  int GetCCWSpeed();
  int GetCurrentSpeed(); 
  int GetACDC(); 
  int GetCWLimit();
  int GetCCWLimit();
  int GetPosition();
  long GetTotalSteps();  
    
  void Wheel(unsigned long tm);
  void Step(long steps, long acdcStps);
  void Turns(float turns, float acdc);
  void Servo(int pos);
  void ServoPreload(int pos);    
  void Stop();
  void Action();
  void ActionAll();
  
  bool IsMoving();
  bool Ping();
  void Service();
  
};

#endif