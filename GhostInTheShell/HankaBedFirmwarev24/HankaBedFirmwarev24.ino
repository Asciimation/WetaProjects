// Author: Simon Jansen

// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"

#define speedRCPIN 0
#define directionRCPIN 7

// If set to true (1) the EEProm is written with 0 for
// all parms. Do this for setup. 
#define INITIALSTART 0

#define MAXPWMSPEED 511
#define M1MAXSPEED 511
#define M2MAXSPEED 370
#define M3MAXSPEED 511
#define M4MAXSPEED 511

#define OPENING 0
#define STOPPED 1
#define CLOSING 2
#define TENSIONING 3
#define WAITINGTOOPEN 4
#define WAITINGTOCLOSE 5

#define OPENRCVALUE 950
#define CLOSERCVALUE 2000

#define M1DELAY 0
#define M2DELAY 0
#define M3DELAY 0
#define M4DELAY 0

// To avoid having to deal with negative encoder values we
// add on a huge constant so the value is always positive.
#define ENCODEROFFSET 5000000

// Maximum open position.
#define MAXPOS 500000 
                
volatile uint16_t channelSpeed;
volatile uint16_t channelDirection;
volatile uint16_t channelSpeed_start;
volatile uint16_t channelDirection_start;

int flagSpeed;
int flagDirection;

elapsedMillis elapsedRC;
elapsedMillis motorStartedElapsed; 

// Create an IntervalTimer object 
IntervalTimer myTimer;

int speedRC;   
int directionRC;
int oldDirectionRC;

Encoder M1encoder (15, 14);
Encoder M2encoder (18, 19);
Encoder M3encoder (16, 17);
Encoder M4encoder (21, 20);

int M1Speed = 0;
int M2Speed = 0;
int M3Speed = 0;
int M4Speed = 0;

int M1pinAIN1 = 30;
int M1pinAIN2 = 29;

int M2pinAIN1 = 27;
int M2pinAIN2 = 28;

int M3pinAIN1 = 32;
int M3pinAIN2 = 31;

int M4pinAIN1 = 25;
int M4pinAIN2 = 26;

int M1pinPWM = 3;
int M2pinPWM = 5;
int M3pinPWM = 4;
int M4pinPWM = 6;

// This structure contains all the parameters that we will be
// writing to the EEProm.
struct PARAMETERS
{
  int M1lastEncoderPos = 0;
  int M2lastEncoderPos = 0;
  int M3lastEncoderPos = 0;
  int M4lastEncoderPos = 0; 

  int M1openPos = MAXPOS + ENCODEROFFSET;
  int M2openPos = MAXPOS + ENCODEROFFSET;
  int M3openPos = MAXPOS + ENCODEROFFSET;
  int M4openPos = MAXPOS + ENCODEROFFSET;

  int M1closePos = 0 + ENCODEROFFSET;
  int M2closePos = 0 + ENCODEROFFSET;
  int M3closePos = 0 + ENCODEROFFSET;
  int M4closePos = 0 + ENCODEROFFSET;

  int M1maxSpeed = M1MAXSPEED;
  int M2maxSpeed = M2MAXSPEED;
  int M3maxSpeed = M3MAXSPEED;
  int M4maxSpeed = M4MAXSPEED;

  int M1preload = 0;
  int M2preload = 0;
  int M3preload = 0;
  int M4preload = 0;  
};
PARAMETERS parms;

/*
M1 enabled: 1    open: 269629    close: 231819   preload: 2116   encoder: 234406   speed: 511
M2 enabled: 1   open: 263838    close: 231554   preload: 0    encoder: 230775   speed: 370
M3 enabled: 1   open: 267451    close: 231672   preload: 2198   encoder: 234203   speed: 511
M4 enabled: 1   open: 287722    close: 212495   preload: 0    encoder: 211170   speed: 511

 */

int M1direction = STOPPED;
int M2direction = STOPPED;
int M3direction = STOPPED;
int M4direction = STOPPED;

// Start with all motors enabled.
bool M1enabled = true;
bool M2enabled = true;
bool M3enabled = true;
bool M4enabled = true;

const int M1OpenDelay = 0;
const int M1CloseDelay = 1000;

const int M2OpenDelay = 1000;
const int M2CloseDelay = 0;

const int M3OpenDelay = 0;
const int M3CloseDelay = 1000;

const int M4OpenDelay = 0;
const int M4CloseDelay = 0;

elapsedMillis M1OpenTimer = 0;
elapsedMillis M1CloseTimer = 0;
elapsedMillis M2OpenTimer = 0;
elapsedMillis M2CloseTimer = 0;
elapsedMillis M3OpenTimer = 0;
elapsedMillis M3CloseTimer = 0;
elapsedMillis M4OpenTimer = 0;
elapsedMillis M4CloseTimer = 0;

byte byteRead;
unsigned int currentDirection;
boolean led = 0;

unsigned long directionChangeCount = 0;

elapsedMillis elapsed;
elapsedMillis elapsedRCInput;

bool startup = true;

////////////////////////////////////////////////////////////////////////////////////////
// Setup.
////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  Serial.begin(19200);
  delay(2000);                     // Wait for serial connection to the computer.
  
  Serial.println("Start");
  
  pinMode(LED_BUILTIN, OUTPUT);    // Built in LED.

  myTimer.begin(CheckPositions, 10000);    // Call every 0.01 seconds to check positions .

  // Motor control pins and parameters.
  analogWriteResolution(9);
  
  analogWriteFrequency(M1pinPWM, 93750);
  analogWriteFrequency(M2pinPWM, 93750);
  analogWriteFrequency(M3pinPWM, 93750);
  analogWriteFrequency(M4pinPWM, 93750);
  
  pinMode(M1pinAIN1, OUTPUT);
  pinMode(M1pinAIN2, OUTPUT);
  pinMode(M2pinAIN1, OUTPUT);
  pinMode(M2pinAIN2, OUTPUT);
  pinMode(M3pinAIN1, OUTPUT);
  pinMode(M3pinAIN2, OUTPUT);
  pinMode(M4pinAIN1, OUTPUT);
  pinMode(M4pinAIN2, OUTPUT);
 
  // RC control stuff.
  pinMode(speedRCPIN, INPUT);
  pinMode(directionRCPIN, INPUT);

  // Attach an interrupt handler to be called whenever
  // the pin changes from LOW to HIGH or vice versa
  attachInterrupt(speedRCPIN, RCchannelSpeed, CHANGE);
  attachInterrupt(directionRCPIN, RCchannelDirection, CHANGE);
 
  // On initial start write base position to EEPROM.
  if ( INITIALSTART )
  {
    parms.M1lastEncoderPos = 0 + ENCODEROFFSET + (MAXPOS/2);
    parms.M2lastEncoderPos = 0 + ENCODEROFFSET + (MAXPOS/2);
    parms.M3lastEncoderPos = 0 + ENCODEROFFSET + (MAXPOS/2);
    parms.M4lastEncoderPos = 0 + ENCODEROFFSET + (MAXPOS/2);
    EEPROM_writeAnything(0, parms);
  }
  
  // Read the last parms.
  EEPROM_readAnything(0, parms);

  parms.M1maxSpeed = M1MAXSPEED;
  parms.M2maxSpeed = M2MAXSPEED;
  parms.M3maxSpeed = M3MAXSPEED;
  parms.M4maxSpeed = M4MAXSPEED;
  
  // Reset the encoders.
  M1encoder.write(parms.M1lastEncoderPos);
  M2encoder.write(parms.M2lastEncoderPos);  
  M3encoder.write(parms.M3lastEncoderPos);
  M4encoder.write(parms.M4lastEncoderPos);
  
  PrintDebugInfo();
  Serial.print("M1OpenDelay: ");
  Serial.println(M1OpenDelay);
  Serial.print("M1CloseDelay: ");
  Serial.println(M1CloseDelay);
  Serial.print("M2OpenDelay: ");
  Serial.println(M2OpenDelay);
  Serial.print("M2CloseDelay: ");
  Serial.println(M2CloseDelay);
  Serial.print("M3OpenDelay: ");
  Serial.println(M3OpenDelay);
  Serial.print("M3CloseDelay: ");
  Serial.println(M3CloseDelay);
  Serial.print("M4OpenDelay: ");
  Serial.println(M4OpenDelay);
  Serial.print("M4CloseDelay: ");
  Serial.println(M4CloseDelay);  

  // Make sure nothing is moving when we start.
  StopM1();
  StopM2();
  StopM3();
  StopM4();
  currentDirection = STOPPED; 
  startup = true;
}

////////////////////////////////////////////////////////////////////////////////////////
// Open all flaps.
////////////////////////////////////////////////////////////////////////////////////////
void OpenAll()
{
  
  // Motor 1.
  if ( M1enabled && (parms.M1lastEncoderPos < parms.M1openPos) )
  {
    digitalWrite(M1pinAIN1, LOW); 
    digitalWrite(M1pinAIN2, HIGH); 
    analogWrite(M1pinPWM, 0);
    // Reset the timer.
    M1OpenTimer = 0;
    Serial.print("M1 wait to open time: ");
    Serial.println(M1OpenDelay);
    M1direction = WAITINGTOOPEN;
  }

  // Motor 2.
  if ( M2enabled && (parms.M2lastEncoderPos < parms.M2openPos) )
  {
    digitalWrite(M2pinAIN1, LOW); 
    digitalWrite(M2pinAIN2, HIGH); 
    analogWrite(M2pinPWM, 0);
    // Reset the timer.    
    M2OpenTimer = 0;
    Serial.print("M2 wait to open time: ");
    Serial.println(M2OpenDelay);
    M2direction = WAITINGTOOPEN;
  }

  // Motor 3.
  if ( M3enabled && (parms.M3lastEncoderPos < parms.M3openPos) )
  {
    digitalWrite(M3pinAIN1, LOW); 
    digitalWrite(M3pinAIN2, HIGH); 
    analogWrite(M3pinPWM, 0);
    // Reset the timer.
    M3OpenTimer = 0;
    Serial.print("M3 wait to open time: ");
    Serial.println(M3OpenDelay);
    M3direction = WAITINGTOOPEN;
  }

  // Motor 4.
  if ( M4enabled && (parms.M4lastEncoderPos < parms.M4openPos) )
  {
    digitalWrite(M4pinAIN1, LOW); 
    digitalWrite(M4pinAIN2, HIGH); 
    analogWrite(M4pinPWM, 0);
    // Reset the timer.
    M4OpenTimer = 0;
    Serial.print("M4 wait to open time: ");
    Serial.println(M4OpenDelay);
    M4direction = WAITINGTOOPEN;
  }
}

////////////////////////////////////////////////////////////////////////////////////////
// Close all flaps.
////////////////////////////////////////////////////////////////////////////////////////

void CloseAll()
{
   
  // Motor 1.
  if ( M1enabled && (parms.M1lastEncoderPos > parms.M1closePos) )
  {
    digitalWrite(M1pinAIN1, HIGH); 
    digitalWrite(M1pinAIN2, LOW); 
    analogWrite(M1pinPWM, 0);
    // Reset the timer.
    M1CloseTimer = 0;
    Serial.print("M1 wait to close time: ");
    Serial.println(M1CloseDelay);
    M1direction = WAITINGTOCLOSE;
  }

  // Motor 2.
  if ( M2enabled && (parms.M2lastEncoderPos > parms.M2closePos) )
  {

    digitalWrite(M2pinAIN1, HIGH); 
    digitalWrite(M2pinAIN2, LOW);   
    analogWrite(M2pinPWM, 0);
    // Reset the timer.
    M2CloseTimer = 0;
    Serial.print("M2 wait to close time: ");
    Serial.println(M2CloseDelay);
    M2direction = WAITINGTOCLOSE;
  }

  // Motor 3.
  if ( M3enabled && (parms.M3lastEncoderPos > parms.M3closePos) )
  {
    digitalWrite(M3pinAIN1, HIGH); 
    digitalWrite(M3pinAIN2, LOW);   
    analogWrite(M3pinPWM, 0);
    // Reset the timer.
    M3CloseTimer = 0;
    Serial.print("M3 wait to close time: ");
    Serial.println(M3CloseDelay);
    M3direction = WAITINGTOCLOSE;
  }

  // Motor 4.
  if ( M4enabled && (parms.M4lastEncoderPos > parms.M4closePos) )
  {
    digitalWrite(M4pinAIN1, HIGH); 
    digitalWrite(M4pinAIN2, LOW); 
    analogWrite(M4pinPWM, 0);
    // Reset the timer.
    M4CloseTimer = 0;
    Serial.print("M4 wait to close time: ");
    Serial.println(M4CloseDelay);
    M4direction = WAITINGTOCLOSE;
  }
}

////////////////////////////////////////////////////////////////////////////////////////
// Stop code.
////////////////////////////////////////////////////////////////////////////////////////
void StopAll()
{
  StopM1();
  StopM2();
  StopM3();
  StopM4();
  currentDirection = STOPPED;
}

void StopM1()
{
  Serial.println("Stop M1.");
  analogWrite(M1pinPWM, 0);
  digitalWrite(M1pinAIN1, LOW); 
  digitalWrite(M1pinAIN2, LOW); 
  M1direction = STOPPED;
  // Write to the EEProm.
  EEPROM_writeAnything(0, parms);
}

void StopM2()
{
  Serial.println("Stop M2.");
  analogWrite(M2pinPWM, 0);
  digitalWrite(M2pinAIN1, LOW); 
  digitalWrite(M2pinAIN2, LOW); 
  M2direction = STOPPED;
  // Write to the EEProm.
  EEPROM_writeAnything(0, parms);
}

void StopM3()
{
  Serial.println("Stop M3.");
  analogWrite(M3pinPWM, 0);
  digitalWrite(M3pinAIN1, LOW); 
  digitalWrite(M3pinAIN2, LOW); 
  M3direction = STOPPED;
  // Write to the EEProm.
  EEPROM_writeAnything(0, parms);
}

void StopM4()
{
  Serial.println("Stop M4.");
  analogWrite(M4pinPWM, 0);
  digitalWrite(M4pinAIN1, LOW); 
  digitalWrite(M4pinAIN2, LOW); 
  M4direction = STOPPED;
  // Write to the EEProm.
  EEPROM_writeAnything(0, parms);
}

////////////////////////////////////////////////////////////////////////////////////////
// Pretensioning code.
////////////////////////////////////////////////////////////////////////////////////////
void PretensionM1()
{
  Serial.println("Pretensioning M1.");  
  if ( M1direction == CLOSING)
  {
    digitalWrite(M1pinAIN1, LOW); 
    digitalWrite(M1pinAIN2, HIGH); 
  }
  else
  { 
    digitalWrite(M1pinAIN1, HIGH); 
    digitalWrite(M1pinAIN2, LOW); 
  }
  analogWrite(M1pinPWM, parms.M1maxSpeed);
  M1direction = TENSIONING;
}

void PretensionM2()
{
  Serial.println("Pretensioning M2.");  
  if ( M2direction == CLOSING)
  {
    digitalWrite(M2pinAIN1, LOW); 
    digitalWrite(M2pinAIN2, HIGH); 
  }
  else
  { 
    digitalWrite(M2pinAIN1, HIGH); 
    digitalWrite(M2pinAIN2, LOW); 
  }
  analogWrite(M2pinPWM, parms.M2maxSpeed);
  M2direction = TENSIONING;
}

void PretensionM3()
{
  Serial.println("Pretensioning M3.");  
  if ( M3direction == CLOSING)
  {
    digitalWrite(M3pinAIN1, LOW); 
    digitalWrite(M3pinAIN2, HIGH); 
  }
  else
  { 
    digitalWrite(M3pinAIN1, HIGH); 
    digitalWrite(M3pinAIN2, LOW); 
  }
  analogWrite(M3pinPWM, parms.M3maxSpeed);
  M3direction = TENSIONING;
}

void PretensionM4()
{
  Serial.println("Pretensioning M4.");  
  if ( M4direction == CLOSING)
  {
    digitalWrite(M4pinAIN1, LOW); 
    digitalWrite(M4pinAIN2, HIGH); 
  }
  else
  { 
    digitalWrite(M4pinAIN1, HIGH); 
    digitalWrite(M4pinAIN2, LOW); 
  }
  analogWrite(M4pinPWM, parms.M4maxSpeed);
  M4direction = TENSIONING;
}

////////////////////////////////////////////////////////////////////////////////////////
// Update speeds based on last controller position.
////////////////////////////////////////////////////////////////////////////////////////
void UpdateSpeeds()
{
   // Update the speed on all controllers proportionally.
  M1Speed = map (speedRC, 0, MAXPWMSPEED, 0, parms.M1maxSpeed);
  M2Speed = map (speedRC, 0, MAXPWMSPEED, 0, parms.M2maxSpeed);
  M3Speed = map (speedRC, 0, MAXPWMSPEED, 0, parms.M3maxSpeed);
  M4Speed = map (speedRC, 0, MAXPWMSPEED, 0, parms.M4maxSpeed);   
}

////////////////////////////////////////////////////////////////////////////////////////
// Check positions.
////////////////////////////////////////////////////////////////////////////////////////
void CheckPositions ()
{
  
  // Update speeds.
  UpdateSpeeds();

  // If waiting to move the speed should be zero.  
  if ( (M1direction == WAITINGTOOPEN) ||  (M1direction == WAITINGTOCLOSE) )
  {
    M1Speed = 0; 
  }
  if ( (M2direction == WAITINGTOOPEN) ||  (M2direction == WAITINGTOCLOSE) )
  {
    M2Speed = 0; 
  }
  if ( (M3direction == WAITINGTOOPEN) ||  (M3direction == WAITINGTOCLOSE) )
  {
    M3Speed = 0; 
  }
  if ( (M4direction == WAITINGTOOPEN) ||  (M4direction == WAITINGTOCLOSE) )
  {
    M4Speed = 0; 
  }
  
  // Start any motors based on their timers.
  // Motor 1.
  if ( (M1direction == WAITINGTOOPEN) && (M1OpenTimer > M1OpenDelay)) 
  {
    M1direction = OPENING;
  }
  if ( (M1direction == WAITINGTOCLOSE) && (M1CloseTimer > M1CloseDelay) )
  {    
    M1direction = CLOSING;
  }

  // Motor 2.
  if ( (M2direction == WAITINGTOOPEN) && (M2OpenTimer > M2OpenDelay)) 
  {
    M2direction = OPENING;
  }
  if ( (M2direction == WAITINGTOCLOSE) && (M2CloseTimer > M2CloseDelay) )
  {    
    M2direction = CLOSING;
  }

  // Motor 3.
  if ( (M3direction == WAITINGTOOPEN) && (M3OpenTimer > M3OpenDelay)) 
  {
    M3direction = OPENING;
  }
  if ( (M3direction == WAITINGTOCLOSE) && (M3CloseTimer > M3CloseDelay) )
  {    
    M3direction = CLOSING;
  }  
  // Motor 4.
  if ( (M4direction == WAITINGTOOPEN) && (M4OpenTimer > M4OpenDelay) ) 
  {
    M4direction = OPENING;
  }
  if ( (M4direction == WAITINGTOCLOSE) && (M4CloseTimer > M4CloseDelay) )
  {    
    M4direction = CLOSING;
  }  

  // Set the speeds.
  analogWrite(M1pinPWM, M1Speed);
  analogWrite(M2pinPWM, M2Speed);
  analogWrite(M3pinPWM, M3Speed);
  analogWrite(M4pinPWM, M4Speed);
  
  // Get the current parms.
  int newM1Pos = M1encoder.read();
  int newM2Pos = M2encoder.read();
  int newM3Pos = M3encoder.read();
  int newM4Pos = M4encoder.read();

  // First read and update current positions. 
  if ( parms.M1lastEncoderPos != newM1Pos )
  {
    Serial.print("M1: Close limit: ");
    Serial.print(parms.M1closePos - ENCODEROFFSET );
    Serial.print("\tPosition: ");
    Serial.print(newM1Pos - ENCODEROFFSET );
    Serial.print("\tOpen limit: ");
    Serial.print(parms.M1openPos - ENCODEROFFSET );    
    Serial.print("\tPreload: ");
    Serial.print(parms.M1preload );
    Serial.print("\tSpeed: ");
    Serial.println(M1Speed );
    parms.M1lastEncoderPos = newM1Pos;    
  }
  
  if ( parms.M2lastEncoderPos != newM2Pos )
  {
    Serial.print("M2: Close limit: ");
    Serial.print(parms.M2closePos - ENCODEROFFSET );
    Serial.print("\tPosition: ");
    Serial.print(newM2Pos - ENCODEROFFSET );
    Serial.print("\tOpen limit: ");
    Serial.print(parms.M2openPos - ENCODEROFFSET );
    Serial.print("\tPreload: ");
    Serial.print(parms.M2preload );
    Serial.print("\tSpeed: ");
    Serial.println(M2Speed );
    parms.M2lastEncoderPos = newM2Pos;
  }

  if ( parms.M3lastEncoderPos != newM3Pos )
  {
    Serial.print("M3: Close limit: ");
    Serial.print(parms.M3closePos - ENCODEROFFSET );    
    Serial.print("\tPosition: ");
    Serial.print(newM3Pos - ENCODEROFFSET );
    Serial.print("\tOpen limit: ");
    Serial.print(parms.M3openPos - ENCODEROFFSET );
    Serial.print("\tPreload: ");
    Serial.print(parms.M3preload );
    Serial.print("\tSpeed: ");
    Serial.println(M3Speed );
    parms.M3lastEncoderPos = newM3Pos;
  }

  if ( parms.M4lastEncoderPos != newM4Pos )
  {
    Serial.print("M4: Close limit: ");
    Serial.print(parms.M4closePos - ENCODEROFFSET );    
    Serial.print("\tPosition: ");
    Serial.print(newM4Pos - ENCODEROFFSET );
    Serial.print("\tOpen limit: ");
    Serial.print(parms.M4openPos - ENCODEROFFSET );
    Serial.print("\tPreload: ");
    Serial.print(parms.M4preload );
    Serial.print("\tSpeed: ");
    Serial.println(M4Speed );
    parms.M4lastEncoderPos = newM4Pos;
  }

  // If the motors have run a while we can check the limits. We need the delay
  // to allow the motors time to move.
  if ( motorStartedElapsed > 50 )
  { 
    // If the current position is outside range for any motor we stop it.  
    if ( currentDirection == OPENING )
    {

      // Motor 1.       
      if ( (parms.M1lastEncoderPos > parms.M1openPos)
        && (M1direction != STOPPED) && (M1direction != TENSIONING) && (M1enabled) )
      {  
        if ( parms.M1preload > 0 )
        {
          Serial.print("Pretensioning M1 by: ");        
          Serial.println(parms.M1preload );
          PretensionM1();
        }
        else
        {
          Serial.print("Stopping M1 in position: ");
          Serial.println(parms.M1lastEncoderPos - ENCODEROFFSET );
          StopM1();
        }
      }
      // When we finish pretensioning we stop.
      if ( (parms.M1lastEncoderPos < (parms.M1openPos - parms.M1preload) )         
        && (M1direction == TENSIONING) && (M1enabled) )
      {
        Serial.print("Stopping M1 in position: ");
        Serial.println(parms.M1lastEncoderPos - ENCODEROFFSET );
        StopM1();
      }                   

     // Motor 2.       
      if ( (parms.M2lastEncoderPos > parms.M2openPos)
        && (M2direction != STOPPED) && (M2direction != TENSIONING) && (M2enabled) )
      {  
        if ( parms.M2preload > 0 )
        {
          Serial.print("Pretensioning M2 by: ");        
          Serial.println(parms.M2preload );
          PretensionM2();
        }
        else
        {
          Serial.print("Stopping M2 in position: ");
          Serial.println(parms.M2lastEncoderPos - ENCODEROFFSET );
          StopM2();
        }
      }
      // When we finish pretensioning we stop.
      if ( (parms.M2lastEncoderPos < (parms.M2openPos - parms.M2preload) )         
        && (M2direction == TENSIONING) && (M2enabled) )
      {
        Serial.print("Stopping M2 in position: ");
        Serial.println(parms.M2lastEncoderPos - ENCODEROFFSET );
        StopM2();
      }  
                       
      // Motor 3.       
      if ( (parms.M3lastEncoderPos > parms.M3openPos)
        && (M3direction != STOPPED) && (M3direction != TENSIONING) && (M3enabled) )
      {  
        if ( parms.M3preload > 0 )
        {
          Serial.print("Pretensioning M3 by: ");        
          Serial.println(parms.M3preload );
          PretensionM3();
        }
        else
        {
          Serial.print("Stopping M3 in position: ");
          Serial.println(parms.M3lastEncoderPos - ENCODEROFFSET );
          StopM3();
        }
      }
      // When we finish pretensioning we stop.
      if ( (parms.M3lastEncoderPos < (parms.M3openPos - parms.M3preload) )         
        && (M3direction == TENSIONING) && (M3enabled) )
      {
        Serial.print("Stopping M3 in position: ");
        Serial.println(parms.M3lastEncoderPos - ENCODEROFFSET );
        StopM3();
      } 


      // Motor 4.       
      if ( (parms.M4lastEncoderPos > parms.M4openPos)
        && (M4direction != STOPPED) && (M4direction != TENSIONING) && (M4enabled) )
      {  
        if ( parms.M4preload > 0 )
        {
          Serial.print("Pretensioning M4 by: ");        
          Serial.println(parms.M4preload );
          PretensionM4();
        }
        else
        {
          Serial.print("Stopping M4 in position: ");
          Serial.println(parms.M4lastEncoderPos - ENCODEROFFSET );
          StopM4();
        }
      }
      // When we finish pretensioning we stop.
      if ( (parms.M4lastEncoderPos < (parms.M4openPos - parms.M4preload) )         
        && (M4direction == TENSIONING) && (M4enabled) )
      {
        Serial.print("Stopping M4 in position: ");
        Serial.println(parms.M4lastEncoderPos - ENCODEROFFSET );
        StopM4();
      } 
    }
    // If we go past the close point we then reverse and pretension if any set.
    if ( currentDirection == CLOSING )
    {
      // Motor 1.       
      if ( (parms.M1lastEncoderPos < parms.M1closePos)         
        && (M1direction != STOPPED) && (M1direction != TENSIONING) && (M1enabled) )
      {
        if ( parms.M1preload > 0 )
        {
          Serial.print("Pretensioning M1 by: ");        
          Serial.println(parms.M1preload);
          PretensionM1();
        }
        else
        {
          Serial.print("Stopping M1 in position: ");
          Serial.println(parms.M1lastEncoderPos - ENCODEROFFSET );
          StopM1();
        }
      }
      
      // When we finish pretensioning we stop.
      if ( (parms.M1lastEncoderPos > (parms.M1closePos + parms.M1preload) )         
        && (M1direction == TENSIONING) && (M1enabled) )
      {
        Serial.print("Stopping M1 in position: ");
        Serial.println(parms.M1lastEncoderPos - ENCODEROFFSET );
        StopM1();
      }        

      // Motor 2.  
      if ( (parms.M2lastEncoderPos < parms.M2closePos)         
        && (M2direction != STOPPED) && (M2direction != TENSIONING) && (M2enabled) )
      {
        if ( parms.M2preload > 0 )
        {
          Serial.print("Pretensioning M2 by: ");        
          Serial.println(parms.M2preload );
          PretensionM2();
        }
        else
        {
          Serial.print("Stopping M2 in position: ");
          Serial.println(parms.M2lastEncoderPos - ENCODEROFFSET );
          StopM2();
        }
      }
      // When we finish pretensioning we stop.
      if ( (parms.M2lastEncoderPos > (parms.M2closePos + parms.M2preload) )         
        && (M2direction == TENSIONING) && (M2enabled) )
      {
        Serial.print("Stopping M2 in position: ");
        Serial.println(parms.M2lastEncoderPos - ENCODEROFFSET );
        StopM2();
      }        

      // Motor 3.
      if ( (parms.M3lastEncoderPos < parms.M3closePos)         
        && (M3direction != STOPPED) && (M3direction != TENSIONING) && (M3enabled) )
      {
        if ( parms.M3preload > 0 )
        {
          Serial.print("Pretensioning M3 by: ");        
          Serial.println(parms.M3preload );
          PretensionM3();
        }
        else
        {
          Serial.print("Stopping M3 in position: ");
          Serial.println(parms.M3lastEncoderPos - ENCODEROFFSET );
          StopM3();
        }
      }
      // When we finish pretensioning we stop.
      if ( (parms.M3lastEncoderPos > (parms.M3closePos + parms.M3preload) )         
        && (M3direction == TENSIONING) && (M3enabled) )
      {
        Serial.print("Stopping M3 in position: ");
        Serial.println(parms.M3lastEncoderPos - ENCODEROFFSET );
        StopM3();
      }        

      // Motor 4.
      if ( (parms.M4lastEncoderPos < parms.M4closePos)         
        && (M4direction != STOPPED) && (M4direction != TENSIONING) && (M4enabled) )
      {
        if ( parms.M4preload > 0 )
        {
          Serial.print("Pretensioning M4 by: ");        
          Serial.println(parms.M4preload );
          PretensionM4();
        }
        else
        {
          Serial.print("Stopping M4 in position: ");
          Serial.println(parms.M4lastEncoderPos - ENCODEROFFSET );
          StopM4();
        }
      }
      // When we finish pretensioning we stop.
      if ( (parms.M4lastEncoderPos > (parms.M4closePos + parms.M4preload) )         
        && (M4direction == TENSIONING) && (M4enabled) )
      {
        Serial.print("Stopping M4 in position: ");
        Serial.println(parms.M4lastEncoderPos - ENCODEROFFSET );
        StopM4();
      }        
    }
  }
 
  // Update the speed on each motor if needed.
  if ( flagSpeed == true )
  {    
    if ( (channelSpeed >= 940) && (channelSpeed <= 1900) )
    {
      int oldSpeed = speedRC;          
      speedRC = map(channelSpeed, 940, 1900, 0, MAXPWMSPEED);       
      // If the speed has changed.
      if ( speedRC != oldSpeed )
      {
        // Only print the speed change if it is more than 2. 
        if ( abs(speedRC - oldSpeed) > 2 ) 
        {          
            Serial.print("RC Speed: ");
            Serial.println(speedRC);         
        }                  
      }
    }

    // Reset the flag. 
    flagSpeed = false;
    
  }

  // Update the direction if needed.
  if ( flagDirection == true )
  { 

    // Increment the count.
    directionChangeCount++;
    if ( directionChangeCount == 4294967295 )
    {
       directionChangeCount = 0;
    }

    //Serial.print("channelDirection: ");     
    //Serial.println(channelDirection); 

    // Work out direction based on the RC direction value.  
    if ( channelDirection < OPENRCVALUE )
    {
      directionRC = OPENING;           
    }
    else if ( channelDirection > CLOSERCVALUE )
    {
      directionRC = CLOSING;           
    }
    else
    {
      directionRC = STOPPED;           
    }
    //directionRC = map(channelDirection, 940, 1900, 0, 2); 
     
    // If the reading changed reset the count.
    if ( directionRC != oldDirectionRC )
    {
      directionChangeCount = 0;
      //Serial.println("directionChangeCount resetting.");     
    }

    oldDirectionRC = directionRC;   

    if ( directionChangeCount > 20 )
    {
        
      directionChangeCount = 0;
      
      //Serial.print("Mapped direction RC value: ");     
      //Serial.println(directionRC);     
  
      if (directionRC == STOPPED)  // 1.
      {

        // Reset the startup flag.
        startup = false;
        
        if (currentDirection != STOPPED)
        {
          // Stop everything.
          StopAll();
  
          // Wait for actual stop. Just delay a bit.
          delay(500);
          
          // Update EEPRom with current parms.
          EEPROM_writeAnything(0, parms);
        }
      }
        
      if ( (directionRC == OPENING) && (startup == false) ) // 0.
      {      
        if (currentDirection != OPENING)
        {
          Serial.println("Opening");
          currentDirection = OPENING;        
          OpenAll();               
          motorStartedElapsed = 0; 
        }
      }
      
      if ( (directionRC == CLOSING) && (startup == false) ) // 2.
      {
        if (currentDirection != CLOSING)
        {
          Serial.println("Closing");
          currentDirection = CLOSING;
          CloseAll();
          motorStartedElapsed = 0; 
        }
      }
    }    
       
    // Reset the direction change flag.
    flagDirection = false;
  }
}



////////////////////////////////////////////////////////////////////////////////////////
// Main loop.
////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{ 
  // Blink the onboard LED.
  unsigned int delayLED;
  delayLED = map(channelSpeed, 940, 1900, 1000, 10);      
  if (elapsed > delayLED) 
  {
    elapsed = 0;
    led = !led;
    digitalWrite(LED_BUILTIN, led);    
  }

  // If serial available.
  if (Serial.available()) 
  {
    // Get the data.     
    byteRead = Serial.read();
    // Echo it out. 
    Serial.write(byteRead);
    Serial.write(": ");

    // Switch based on the letter entered.
    switch (byteRead) 
    {
      case '1':
        M1ToggleEnable();
        break;
      case '2':
        M2ToggleEnable();
        break;
      case '3':
        M3ToggleEnable();
        break;
      case '4':
        M4ToggleEnable();
        break;
      case 'a':
        EnableDisableAll();
        break;
      case 'c':
        SetCloseLimits();
        break;
      case 'd':
        PrintDebugInfo();
        break;    
      case 'o':
        SetOpenLimits();
        break; 
      case 'p':
        SetPreload();
        break;  
      case 'r':
        ResetLimits();
        break;
        
      default: 
        // Do nothing.
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////
// Debugging.
////////////////////////////////////////////////////////////////////////////////////////
void PrintDebugInfo()
{
  Serial.println("Debug information."); 
    
  Serial.print("M1 enabled: "); 
  Serial.print(M1enabled);   
  Serial.print("\t\topen: "); 
  Serial.print(parms.M1openPos - ENCODEROFFSET);   
  Serial.print("\t\tclose: ");      
  Serial.print(parms.M1closePos - ENCODEROFFSET); 
  Serial.print("\t\tpreload: ");      
  Serial.print(parms.M1preload); 
  Serial.print("\t\tencoder: ");      
  Serial.print(parms.M1lastEncoderPos - ENCODEROFFSET); 
  Serial.print("\t\tspeed: ");      
  Serial.println(parms.M1maxSpeed); 

  Serial.print("M2 enabled: "); 
  Serial.print(M2enabled);      
  Serial.print("\t\topen: "); 
  Serial.print(parms.M2openPos - ENCODEROFFSET);   
  Serial.print("\t\tclose: ");            
  Serial.print(parms.M2closePos - ENCODEROFFSET); 
  Serial.print("\t\tpreload: ");      
  Serial.print(parms.M2preload); 
  Serial.print("\t\tencoder: ");      
  Serial.print(parms.M2lastEncoderPos - ENCODEROFFSET);
  Serial.print("\t\tspeed: ");      
  Serial.println(parms.M2maxSpeed); 

  Serial.print("M3 enabled: "); 
  Serial.print(M3enabled);
  Serial.print("\t\topen: "); 
  Serial.print(parms.M3openPos - ENCODEROFFSET);   
  Serial.print("\t\tclose: ");         
  Serial.print(parms.M3closePos - ENCODEROFFSET); 
  Serial.print("\t\tpreload: ");      
  Serial.print(parms.M3preload); 
  Serial.print("\t\tencoder: ");      
  Serial.print(parms.M3lastEncoderPos - ENCODEROFFSET);
  Serial.print("\t\tspeed: ");      
  Serial.println(parms.M3maxSpeed); 


  Serial.print("M4 enabled: "); 
  Serial.print(M4enabled);           
  Serial.print("\t\topen: "); 
  Serial.print(parms.M4openPos - ENCODEROFFSET);   
  Serial.print("\t\tclose: "); 
  Serial.print(parms.M4closePos - ENCODEROFFSET); 
  Serial.print("\t\tpreload: ");      
  Serial.print(parms.M4preload); 
  Serial.print("\t\tencoder: ");      
  Serial.print(parms.M4lastEncoderPos - ENCODEROFFSET);
  Serial.print("\t\tspeed: ");      
  Serial.println(parms.M4maxSpeed); 

}

////////////////////////////////////////////////////////////////////////////////////////
// RCchannelSpeed.
////////////////////////////////////////////////////////////////////////////////////////
void RCchannelSpeed() 
{
  // If the pin is HIGH, start a timer
  if (digitalRead(speedRCPIN) == HIGH) 
  {
      channelSpeed_start = micros();
  } 
  else 
  {
      // The pin is now LOW so output the difference
      // between when the timer was started and now
      channelSpeed = (uint16_t) (micros() - channelSpeed_start);
      flagSpeed = true;      
      //Serial.print("channelSpeed: ");
      //Serial.println(channelSpeed);
  }
}

////////////////////////////////////////////////////////////////////////////////////////
// RCchannelDirection.
////////////////////////////////////////////////////////////////////////////////////////
void RCchannelDirection() 
{
  
  // If the pin is HIGH, start a timer
  if (digitalRead(directionRCPIN) == HIGH) 
  {
      channelDirection_start = micros();
  } 
  else 
  {
      // The pin is now LOW so output the difference
      // between when the timer was started and now
      channelDirection = (uint16_t) (micros() - channelDirection_start);
      flagDirection = true;      
      //Serial.print("channelDirection: ");
      //Serial.println(channelDirection);
  }
}

////////////////////////////////////////////////////////////////////////////////////////
// Toggle enabling and disabling motors.
////////////////////////////////////////////////////////////////////////////////////////
void M1ToggleEnable()
{
  M1enabled = !M1enabled;
  if ( M1enabled )
  {
    Serial.println("M1 enabled.");
  }
  else
  {
    StopM1();
    Serial.println("M1 disabled.");
  }
  PrintDebugInfo();
}

void M2ToggleEnable()
{
  M2enabled = !M2enabled;
  if ( M2enabled )
  {
    Serial.println("M2 enabled.");
  }
  else
  {
    StopM2();
    Serial.println("M2 disabled.");
  }
  PrintDebugInfo();
}

void M3ToggleEnable()
{
  M3enabled = !M3enabled;
  if ( M3enabled )
  {
    Serial.println("M3 enabled.");
  }
  else
  {
    StopM3();
    Serial.println("M3 disabled.");
  }
  PrintDebugInfo();
}

void M4ToggleEnable()
{
  M4enabled = !M4enabled;
  if ( M4enabled )
  {
    Serial.println("M4 enabled.");
  }
  else
  {
    StopM4();
    Serial.println("M4 disabled.");
  }  
  PrintDebugInfo();
}

void EnableDisableAll()
{
  M1enabled = !M1enabled;
  if ( M1enabled )
  {
    M2enabled = true;
    M3enabled = true;
    M4enabled = true;
    Serial.println("All enabled.");
  }
  else
  {
    StopM1();
    StopM2();
    StopM3();
    StopM4();
    M2enabled = false;
    M3enabled = false;
    M4enabled = false;    
    Serial.println("All disabled.");
  }
  PrintDebugInfo();
}

////////////////////////////////////////////////////////////////////////////////////////
// Set close limits.
////////////////////////////////////////////////////////////////////////////////////////
void SetCloseLimits()
{
  Serial.println("Setting close limits...");
  
  if ( M1enabled )
  {
    // Read the encoder position.
    int newM1Pos = M1encoder.read();
    // Set the current position.
    parms.M1lastEncoderPos = newM1Pos; 
    // Set the new close position.
    parms.M1closePos = newM1Pos; 
    Serial.print("M1 open: "); 
    Serial.print(parms.M1openPos - ENCODEROFFSET);   
    Serial.print(" close: "); 
    Serial.println(parms.M1closePos - ENCODEROFFSET);   
  }
  if ( M2enabled )
  {
    // Read the encoder position.
    int newM2Pos = M2encoder.read();
    // Set the current position.
    parms.M2lastEncoderPos = newM2Pos; 
    // Set the new close position.
    parms.M2closePos = newM2Pos; 
    Serial.print("M2 open: "); 
    Serial.print(parms.M2openPos - ENCODEROFFSET);   
    Serial.print(" close: "); 
    Serial.println(parms.M2closePos - ENCODEROFFSET);   
  } 
  if ( M3enabled )
  {
    // Read the encoder position.
    int newM3Pos = M3encoder.read();
    // Set the current position.
    parms.M3lastEncoderPos = newM3Pos; 
    // Set the new close position.
    parms.M3closePos = newM3Pos;
    Serial.print("M3 open: "); 
    Serial.print(parms.M3openPos - ENCODEROFFSET);   
    Serial.print(" close: "); 
    Serial.println(parms.M3closePos - ENCODEROFFSET);    
  }
  if ( M4enabled )
  {
    // Read the encoder position.
    int newM4Pos = M4encoder.read();
    // Set the current position.
    parms.M4lastEncoderPos = newM4Pos; 
    // Set the new close position.
    parms.M4closePos = newM4Pos; 
    Serial.print("M4 open: "); 
    Serial.print(parms.M4openPos - ENCODEROFFSET);   
    Serial.print(" close: "); 
    Serial.println(parms.M4closePos - ENCODEROFFSET);   
  }
}

////////////////////////////////////////////////////////////////////////////////////////
// Set open limits.
////////////////////////////////////////////////////////////////////////////////////////
void SetOpenLimits()
{
  Serial.println("Setting open limits...");
      
  if ( M1enabled )
  {
    // Get the current parms.
    int newM1Pos = M1encoder.read();    
    // Set the current position.
    parms.M1lastEncoderPos = newM1Pos; 
    // Set the new open position.
    parms.M1openPos = newM1Pos;    
    Serial.print("M1 open: "); 
    Serial.print(parms.M1openPos - ENCODEROFFSET);   
    Serial.print(" close: "); 
    Serial.println(parms.M1closePos - ENCODEROFFSET);  
  }
  
  if ( M2enabled )
  {
    // Get the current parms.
    int newM2Pos = M2encoder.read();    
    // Set the current position.
    parms.M2lastEncoderPos = newM2Pos; 
    // Set the new open position.
    parms.M2openPos = newM2Pos;    
    Serial.print("M2 open: "); 
    Serial.print(parms.M2openPos - ENCODEROFFSET);   
    Serial.print(" close: "); 
    Serial.println(parms.M2closePos - ENCODEROFFSET);  
  }

  if ( M3enabled )
  {
    // Get the current parms.
    int newM3Pos = M3encoder.read();    
    // Set the current position.
    parms.M3lastEncoderPos = newM3Pos; 
    // Set the new open position.
    parms.M3openPos = newM3Pos;    
    Serial.print("M3 open: "); 
    Serial.print(parms.M3openPos - ENCODEROFFSET);   
    Serial.print(" close: "); 
    Serial.println(parms.M3closePos - ENCODEROFFSET);  
  }

  if ( M4enabled )
  {
    // Get the current parms.
    int newM4Pos = M4encoder.read();    
    // Set the current position.
    parms.M4lastEncoderPos = newM4Pos; 
    // Set the new open position.
    parms.M4openPos = newM4Pos;    
    Serial.print("M4 open: "); 
    Serial.print(parms.M4openPos - ENCODEROFFSET);   
    Serial.print(" close: "); 
    Serial.println(parms.M4closePos - ENCODEROFFSET);  
  }
}

////////////////////////////////////////////////////////////////////////////////////////
// Set preload.
////////////////////////////////////////////////////////////////////////////////////////
void SetPreload()
{
  Serial.println("Setting preload...");
      
  if ( M1enabled )
  {    
    // Get the current parms.
    int newM1Pos = M1encoder.read();    
    // Set the current position.
    parms.M1lastEncoderPos = newM1Pos; 
    // Set the new preload.
    parms.M1preload = min( abs(newM1Pos - parms.M1closePos), abs(newM1Pos - parms.M1openPos) );    
    Serial.print("M1 preload: "); 
    Serial.println(parms.M1preload);              
  }
  
  if ( M2enabled )
  {
    // Get the current parms.
    int newM2Pos = M2encoder.read();    
    // Set the current position.
    parms.M2lastEncoderPos = newM2Pos; 
    // Set the new preload.
    parms.M2preload = min( abs(newM2Pos - parms.M2closePos), abs(newM2Pos - parms.M2openPos) );    
    Serial.print("M2 preload: "); 
    Serial.println(parms.M2preload);   
  }
  
  if ( M3enabled )
  {
    // Get the current parms.
    int newM3Pos = M3encoder.read();    
    // Set the current position.
    parms.M3lastEncoderPos = newM3Pos; 
    // Set the new preload.
    parms.M3preload = min( abs(newM3Pos - parms.M3closePos), abs(newM3Pos - parms.M3openPos) );    
    Serial.print("M3 preload: "); 
    Serial.println(parms.M3preload);           
  }
  
  if ( M4enabled )
  {
    // Get the current parms.
    int newM4Pos = M4encoder.read();    
    // Set the current position.
    parms.M4lastEncoderPos = newM4Pos; 
    // Set the new preload.
    parms.M4preload = min( abs(newM4Pos - parms.M4closePos), abs(newM4Pos - parms.M4openPos) );    
    Serial.print("M4 preload: "); 
    Serial.println(parms.M4preload);           
  }
} 

////////////////////////////////////////////////////////////////////////////////////////
// Reset limits.
////////////////////////////////////////////////////////////////////////////////////////
void ResetLimits()
{
  Serial.println("Resetting limits to 0 close and MAXPOS open.");  
  // Set the max open position.
  if (M1enabled) parms.M1openPos = MAXPOS + ENCODEROFFSET;    
  if (M2enabled) parms.M2openPos = MAXPOS + ENCODEROFFSET;    
  if (M3enabled) parms.M3openPos = MAXPOS + ENCODEROFFSET;    
  if (M4enabled) parms.M4openPos = MAXPOS + ENCODEROFFSET;        
  
  // Set the new close position.
  if (M1enabled) parms.M1closePos = 0 + ENCODEROFFSET;    
  if (M2enabled) parms.M2closePos = 0 + ENCODEROFFSET;    
  if (M3enabled) parms.M3closePos = 0 + ENCODEROFFSET;        
  if (M4enabled) parms.M4closePos = 0 + ENCODEROFFSET;       
  
  // Set the current position to the mid point.
  if (M1enabled) parms.M1lastEncoderPos = 0 + ENCODEROFFSET + (MAXPOS/2);
  if (M2enabled) parms.M2lastEncoderPos = 0 + ENCODEROFFSET + (MAXPOS/2);
  if (M3enabled) parms.M3lastEncoderPos = 0 + ENCODEROFFSET + (MAXPOS/2);
  if (M4enabled) parms.M4lastEncoderPos = 0 + ENCODEROFFSET + (MAXPOS/2);

  // Reset the encoders.
  if (M1enabled) M1encoder.write(parms.M1lastEncoderPos);
  if (M2enabled) M2encoder.write(parms.M2lastEncoderPos);
  if (M3enabled) M3encoder.write(parms.M3lastEncoderPos);
  if (M4enabled) M4encoder.write(parms.M4lastEncoderPos);

  // Reset the preloads.
  if (M1enabled) parms.M1preload = 0;
  if (M2enabled) parms.M2preload = 0;
  if (M3enabled) parms.M3preload = 0;
  if (M4enabled) parms.M4preload = 0;

  // Write to the EEProm.
  EEPROM_writeAnything(0, parms);
}
