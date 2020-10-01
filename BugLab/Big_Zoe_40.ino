//Author: Simon Jansen

const String version_string = "40";

// Debounce library.
#include <Bounce2.h>

// Mappings between Arduino Mega pins to M-Duino 21.
// Analog/Digital in.
int I7 = A0;     
int I8 = A1;     
int I9 = A2;      
int I10 = A3;    
int I11 = A4;   
int I12 = A5;  

// Interrupt pins.
int I5 = 2;  
int I6 = 3;  
 
// Digital out. 
int Q00 = 36;  
int Q01 = 37;  
int Q02 = 38;  
int Q03 = 39;  
int Q04 = 40;  
int Q05 = 4;
int Q06 = 5;
int Q07 = 6;  

// Outputs.
const int dirPin = Q00;                   // Dir pin. 
const int stepPin = Q01;                  // Step pin.
const int strobePin = Q07;                // Strobe output trigger pin. 
const int SP1Pin = Q03;                   // Speed control pin 1.
                                          // Speed control pins 2 and 3 left low on the controller.     

const int auxOutput1 = Q04;               // Auxiliary output 1. Used for triggering external events, sounds, light, etc.
const int auxOutput2 = Q05;               // Auxiliary output 2.
const int auxOutput3 = Q06;               // Auxiliary output 3.
const int auxOutput4 = Q02;               // Auxiliary output 4.   

bool auxOutput1On = false;                // Flag 1. Flag to indicate and output is active. 
bool auxOutput2On = false;                // Flag 2.
bool auxOutput3On = false;                // Flag 3. 
bool auxOutput4On = false;                // Flag 4. 

// Inputs.
const int triggerPin = I12;                // Trigger signal input.
const int homeInductiveSensorPin = I11;    // Home sensor input.
const int strobeInductiveSensorPin = I5;   // Strobe sensor input on interrupt pin.
const int maintenanceSwitchPin = I10;      // Maintenance mode switch input.
const int testTriggerSwitchPin = I9;       // Test trigger switch input.

// Input state flags.
bool maintenanceMode = false;              // Flag to indicate we were are in maintenance mode.
bool triggered = false;                    // Flag to indicate we were triggered.

// PWM pins.
//Pins 4 and 13: controlled by timer 0
//Pins 11 and 12: controlled by timer 1
//Pins 9 and 10: controlled by timer 2
//Pin 2, 3 and 5: controlled by timer 3
//Pin 6, 7 and 8: controlled by timer 4
//Pin 46, 45 and 44: controlled by timer 5

// Debouncers.
Bounce triggerSignal = Bounce();
Bounce testTriggerSwitch = Bounce();

// State machine variables.

// States for the top dome.
enum UppyDowny_State { top_unknown,
                       top_opening,
                       top_opening_wait,
                       top_closing,
                       top_closing_wait,
                       top_open,
                       top_closed,
                       top_homing_up,                       
                       top_homing_down,
                       top_moving_up,
                       top_moving_down,
                       top_fault
};
UppyDowny_State uppydowny_state = top_unknown;

// States for the dome motor.
enum Motor_State { motor_unknown,
                   motor_stopped, 
                   motor_accelerating,
                   motor_decelerating,
                   motor_constant_speed,
                   motor_homing_speed,
                   motor_moving_speed,
                   motor_fault     
};
Motor_State motor_state = motor_unknown;

// States for the rotating disc.
enum RoundyRoundy_State { disc_unknown,
                          disc_start,
                          disc_start_wait,
                          disc_stop,
                          disc_stop_wait                                                   
};
RoundyRoundy_State roundyroundy_state = disc_unknown;

// States for the strobe lights.
enum Strobe_State { strobe_unknown,
                    strobe_on,
                    strobe_off,
                    strobe_pwm
};
Strobe_State strobe_state = strobe_unknown;

// Homing settings.
// Homing switch is in the positive direction.
// To home:
// 1. If the home switch is triggered go back down homingDownSteps to clear the switch.
// 2. Home upwards until the home switch is triggered.
// If homing doesn't happen within the homing timeout time we fault.
const int homingSpeed = 500;                                                  // Homing speed interval between steps in uS (sets the homing speed)
const int homingDownSteps = 5000;                                             // Steps to go negative before positive homing if switch already trigegred.
bool homedTop = false;                                                        // Flag to indicate we are homed to the top position.
bool homed = false;                                                           // Flag to indicate we are fully homed.
const unsigned long homingTimeout = 120000;                                   // Timeout for homing in mS.
unsigned long homingStartTime = 0;                                            // Time homing began.

// Cycle runnning flag.
bool cycleRunning = false;                                                    // Variable to tell if a cycle is running.
unsigned long cycleCount = 0;                                                 // Count of how many cycles we have done since restart.    

// Test triggering.
unsigned long lastTestTriggerTime = 0;                                       // Last time a test cycle was triggered.
const unsigned long testTriggerPeriod = 160000;                              // Time between test cycles.
const unsigned long nextTestPrintTime = 10000;                               // Time between printing when the next test will be.
bool testSwitchOn = false;                                                   // Flag to indicate the test switch is on.   

// Settings for moving via serial commands.
long movingSteps = 0;                                                         // Number of steps to move up or down via serial.
const int movingSpeed = 500;                                                  // Moving interval between steps in uS (sets the moving speed).

// Uppy downy settings.
const unsigned long homePosition = 53300; //53500;                            // Homing position. Home is in the positive direction.
                                                                              // Once homed the zoetrope moves down this many steps
                                                                              // back to the zero position.   
const unsigned long openingExtraSteps = 500;                                  // Go this many extra steps when opening the top to 
                                                                              // ensure we hit the home switch every time.
                                                                              
const unsigned long totalSteps = homePosition;                                // Total number of steps to move in one direction.
const unsigned long openCloseTime = 12000;                                    // Open/close time in mS. This is the time taken with NO acc/dcc. 
                                                                              // Any acc/dcc will increase this time.            
const int accdccPercentage = 10;                                              // Percentage time to spend accelerating and decelerating (half on each).
unsigned long stepsToAccDcc = (( totalSteps / 100 ) * accdccPercentage) / 2;  // Calculated steps spent accelerating and decelerating.

// Times.
unsigned long triggerTime = 0;                                                // Time we were triggered.
unsigned long openedTime = 0;                                                 // Time we were fully opened.
unsigned long strobeStartedTime = 0;                                          // Time we started the strobe.
unsigned long closingTime = 0;                                                // Time we started closing.
const unsigned long runTime = 60000;                                          // Run time in mS.
const unsigned long spinStartDelay = 0;                                       // Delay before we start spinning in mS.
const unsigned long spinStopDelay = 9000;// 11000;                            // Delay before we stop spinning in mS.
const unsigned long openDelay = 700;                                          // Delay before we start opening in mS.
const unsigned long strobeStartDelay = 0;                                     // Delay before we start the strobe after starting deceleration of the top.
const unsigned long strobeStopDelay = 1000;                                   // Delay before we turn off the strobe after we start closing.
const unsigned long firstFlashDelay = 100;                                    // Delay before firing first flash after being triggered.
bool firstFlashFired = false;                                                 // Boolean to say the first flash fired.  
bool lastFlashFired = false;                                                  // Boolean to say the last flash fired.  
const unsigned long lastFlashDelay = 15000;                                   // Delay before firing last flash after starting to close.
const unsigned long pulseTime = 1000;                                         // Length of pulse sent on auxilary outputs in mS.
unsigned long pulseTimer = 0;                                                 // Timer for pulses.

// Strobe and PWM variables.
volatile bool strobeTriggered = false;                                        // Strobe triggered flag.
bool lastStrobeTriggered = false;                                             // Variable to tell if the strobe state changed.
unsigned long strobeLastOnTime = 0;                                           // Last time strobe was turned on.
const unsigned long strobeOnPeriod = 200;                                     // Time the strobe is on for in uS.
const unsigned long strobeMaximumOnTime = 100;                                // Maximum strobe on time in mS. 
const unsigned long minPWMValue = 1;                                          // PWM minimum value.
const unsigned long maxPWMValue = 255;                                        // PWM maximum value.
const unsigned long PWMPulseTime1 = 3000;                                     // PWM pulse 1 length in mS. Opening flash.
const unsigned long PWMPulseTime2 = 1500;                                     // PWM pulse 2 length in mS. Closing flash.
unsigned long PWMPulseTime = 0;                                               // PWM pulse length in mS.
unsigned long PWMPulseTimer = 0;                                              // Timer for PWM pulses.
const int PWMLevel = 5;                                                       // PWM level.
bool PWMOn = false;                                                           // Flag to say PWM is active.

// Steps and positions variables.
volatile unsigned long stepCount = 0;                                         // Up/down step counter.
volatile bool didStep = false;                                                // Was a step done. 
float stepsPerMillisecond = (float)totalSteps / (float)openCloseTime;         // Calculated steps per mS.    
float lastStepTime = 0;                                                       // Last step time in uS.
float stepInterval = 0;                                                       // Interval between steps in uS.
unsigned long movementStartTime = 0;                                          // Time of last movement phase.
volatile long absolutePosition = 0;                                           // Absolute position maintained after homing.     
const int absolutePositionError = 25;                                         // Margin of how many steps our absolute position can be out by. 

// Acceleration.
int accdccFactor = 10;                                                        // Factor for how much accelerating and decelerating we do.
int accdccStep = 0;                                                           // Count of accelerating and decelerating steps.  
int accdccTableIndex = 0;                                                     // Current index into the accelerastion/deceleration table.    
unsigned long accdccInterval = 0;                                             // Maximum interval used when accelerating or decelerating.

// Variables for handling commands.
const int numberOfCommandEntries = 10;                                        // Number of command tokens.
bool commandReceived = false;                                                 // Flag to indicate a command was received.
const int commandBufferSize = 64;                                             // Maximum command buffer size.
int commandBufferIndex = 0;                                                   // Index into command buffer.
char commandBuffer[commandBufferSize];                                        // Buffer for incoming command.
const char commandDelimiter[2] = " ";                                         // Delimiting character.
char* commandTokenArray[numberOfCommandEntries];                              // Array of pointers to command tokens.

// Direction.
bool opening = true;                                                          // Which direction are we moving?

// RPM debugging display.
bool displayRPM = false;                                                      // Do we display RPM.      
unsigned long RPMTimer;                                                       // Timer for RPM calculation.    
volatile unsigned int pulseCounter = 0;                                       // Trigger pulse counter.        

# define ACDCTABLESTEPS  128                                                  // SIN based look up table for acceleration/deceleration.    
// SIN table.
const int ACDCLookupTable[] = 
{
  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,
  2,  3,  3,  3,  4,  4,  5,  5,  6,  6,
  7,  8,  8,  9,  10, 11, 11, 12, 13, 14,
  15, 16, 16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 28, 29, 30, 31, 32, 33, 34,
  35, 37, 38, 39, 40, 41, 43, 44, 45, 46,
  48, 49, 50, 50, 51, 52, 54, 55, 56, 57,
  59, 60, 61, 62, 63, 65, 66, 67, 68, 69,
  70, 71, 72, 74, 75, 76, 77, 78, 79, 80,
  81, 82, 83, 84, 84, 85, 86, 87, 88, 89,
  89, 90, 91, 92, 92, 93, 94, 94, 95, 95,
  96, 96, 97, 97, 97, 98, 98, 99, 99, 99,
  99, 99, 100, 100, 100, 100, 100, 100
};

/*
# define ACDCTABLESTEPS  100                                                  // Linear based look up table for acceleration/deceleration. 
// Linear table.
const int ACDCLookupTable[] = 
{
   1,  2,  3,  4,  5,  6,  7,  8,  9, 10,
  11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 
  21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 
  31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 
  41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 
  51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 
  61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 
  71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 
  81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 
  91, 92, 93, 94, 95, 96, 97, 98, 99, 100  
};
*/

////////////////////////////////////////////////////////////
// Setup.
////////////////////////////////////////////////////////////
void setup()   
{ 
  // Initialise the serial port.
  Serial.begin(250000);
  Serial.print("Zoetrope controller started.  Version: ");  
  Serial.println(version_string);  

  // Set up inputs. 
  
  // Debouncers. 
  triggerSignal.attach(triggerPin);
  triggerSignal.interval(25);
  testTriggerSwitch.attach(testTriggerSwitchPin);
  testTriggerSwitch.interval(25);
  
  // Inputs.  
  pinMode(triggerPin, INPUT);      
  pinMode(testTriggerSwitchPin, INPUT);     
  pinMode(strobeInductiveSensorPin, INPUT);    
  pinMode(homeInductiveSensorPin, INPUT);
  pinMode(maintenanceSwitchPin, INPUT);
  
  // Outputs.
  pinMode(dirPin, OUTPUT);     
  pinMode(SP1Pin, OUTPUT); 
  pinMode(strobePin, OUTPUT);       
  pinMode(stepPin, OUTPUT);
  pinMode(auxOutput1, OUTPUT);
  pinMode(auxOutput2, OUTPUT);
  pinMode(auxOutput3, OUTPUT);
  pinMode(auxOutput4, OUTPUT);

  // Not triggered.
  triggered = false;
 
  // Set direction.
  digitalWrite(dirPin, LOW);
  
  // Step pin low.
  digitalWrite(stepPin, LOW);

  // Speed pin low (off).
  digitalWrite(SP1Pin, LOW);
  
  // Work out the steps per mS.
  stepsPerMillisecond = (float)totalSteps / (float)openCloseTime;
  
  // Work out the interval between steps in uS.
  stepInterval = 1000 / stepsPerMillisecond;

  // No steps yet.
  didStep = false;

  // Set up the spinning disc state.
  roundyroundy_state = disc_unknown;

  // Set up the top state.
  uppydowny_state = top_unknown; 

  // Set up the strobe state.
  strobe_state = strobe_off; 
  
  // Set up the motor state.
  motor_state = motor_stopped; 

  // Homing state.
  homedTop = false;
  homed = false;

  // Work out the accleration and deceleration interval based on the scaling factor.
  accdccInterval = accdccFactor * stepInterval;

  // All outputs off.
  digitalWrite(auxOutput1, LOW);
  digitalWrite(auxOutput2, LOW);
  digitalWrite(auxOutput3, LOW);
  digitalWrite(auxOutput4, LOW);
  auxOutput1On = false;             
  auxOutput2On = false;             
  auxOutput3On = false;             
  auxOutput4On = false;             
  
  // Timers.
  uint8_t SaveSREG = SREG;          // Save interrupt flag.
  cli();                            // Disable all interrupts.    
  
  // Initialize timer 1. Used for motor speed control on uppy downy.
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  
  // Set up timer one with a 8x prescale to give 0.5uS interrupt time.
  OCR1A = 100;                  // Compare match register - 100 * O.5uS = 50uS.
  TCCR1B |= (1 << WGM12);       // CTC mode.
  TCCR1B |= (1 << CS11);        // 8 prescaler. 
  TIMSK1 &= (0 << OCIE1A);      // Disable timer compare interrupt.

  // Initialize timer 3. Used for timing to turn off the strobe.
  TCCR3A = 0; 
  TCCR3B = 0;
  TCNT3  = 0;
  TIMSK3 = 0;

  // Set up timer three with a 8x prescale to give 0.5uS interrupt time.
  OCR3A = strobeOnPeriod * 2;   // Compare match register - (strobeOnPeriod * O.5uS) * 2.
  TCCR3B |= (1 << WGM32);       // CTC mode.
  TCCR3B |= (1 << CS31);        // 8 prescaler. 
  TIMSK3 &= (0 << OCIE3A);      // Disable timer compare interrupt.

  // Set up timer four prescale.
  // CS42 CS41 CS40 
  // 0    0    1     - 1 (31 kHz )
  // 0    1    0     - 1/8 (4 kHz)
  // 0    1    1     - 1/64 (500 Hz)
  // 1    0    0     - 1/256  (125 Hz)
  // 1    0    1     - 1/1024 (30 Hz)
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4A |= (1 << WGM40);       // PWM mode - 8 bit. 
  TCCR4B |= (1 << CS40);     
  //TCCR4B |= (1 << CS41);        
  //TCCR4B |= (1 << CS42);      

  SREG = SaveSREG;                  // Restore interrupt flag.
  sei();                            // Enable all interrupts.
  
  Serial.print("Waiting 10 seconds");
  for ( int i = 9; i > 0; i-- )
  {
    delay(1000);    
    Serial.print("...");
    Serial.print(i);
  }
  Serial.println(""); 

  // Check the maintenance switch.
  if ( digitalRead(maintenanceSwitchPin) == HIGH )
  {
    // Switch triggered.
    Serial.println();
    Serial.println("Maintenance switch on. Entering maintenance mode."); 
    Serial.println();
    maintenanceMode = true;
  }

  // Set the last triggered time.
  // Set it to the period to ensure we can trigger in period 0;
  lastTestTriggerTime = testTriggerPeriod;
  
  // Home to known position.  
  Home();

  Serial.println();
  Serial.print("Total steps to move: ");
  Serial.println(totalSteps);
  Serial.print("Acceleration and deceleration percentage: ");
  Serial.println(accdccPercentage);
  Serial.print("Acc Dccsteps: ");
  Serial.println(stepsToAccDcc);
  Serial.print("Moving time in mS: ");
  Serial.println(openCloseTime);
  Serial.print("Steps per mS: ");
  Serial.println(stepsPerMillisecond);
  Serial.print("Step interval in uS: ");
  Serial.println(stepInterval);  
}

////////////////////////////////////////////////////////////
// Timer 1 compare interrupt service routine.
// This controls the step duration. We change this based on the 
// current acceleration or decleration rate.
// This interrupt handler is called once for every step we take.
////////////////////////////////////////////////////////////
ISR(TIMER1_COMPA_vect)          
{
  // Do a step.      
  digitalWrite(stepPin, HIGH);
  
  // Set the pin low again.
  digitalWrite(stepPin, LOW);

  // We did a step.
  didStep = true;
    
  // Increment the count.
  stepCount++;    

  // If we aren't homing we update the absolute position.
  if ( (homedTop == true) || (homed == true) )
  { 
    // Update absolute position.
    if ( opening ) 
    {
      // Opening so increase position.
      absolutePosition++;     

      /*
      // Extra check we haven't gone out of bounds.
      // We have a slight allowance here for errors.
      if ( absolutePosition - absolutePositionError > totalSteps )
      {
        Serial.println("Gone over upper limit. Stopping uppydowny!");
        Serial.print("Current stepCount: ");
        Serial.println(stepCount);
        StopUppyDowny();
      }
      */
    }
    else
    {
      // Closing so decrease position.
      absolutePosition--;

      /*
      // Extra check we haven't gone out of bounds.
      // We have a slight allowance here for errors.
      if ( absolutePosition + absolutePositionError < 0 )
      {        
        Serial.println("Gone over lower limit. Stopping uppydowny!");
        Serial.print("Current stepCount: ");
        Serial.println(stepCount);
        StopUppyDowny();
      }
      */
    }
  }
}

////////////////////////////////////////////////////////////
// Timer 3 compare interrupt service routine.
// This controls the turning off of the strobe pulse.
////////////////////////////////////////////////////////////
ISR(TIMER3_COMPA_vect)            // Timer 3 compare interrupt service routine.
{   
  digitalWrite(strobePin, LOW);   // LEDs off.
  strobeTriggered = false;        // Reset the strobe trigger.
  lastStrobeTriggered = false;    // Reset the last strobe trigger.  
  TIMSK3 &= (0 << OCIE3A);        // Disable timer compare interrupt.     
  //TIFR3 = bit (OCF3A);            // Clear any pending interrupt 
}


////////////////////////////////////////////////////////////
// Send output pulse.
// One channel can be active at a time.
// When activated we measure the time and automatically 
// turn the pulse off.
////////////////////////////////////////////////////////////
void SendOutputPulse(int output)
{
  // All outputs off.
  digitalWrite(auxOutput1, LOW);
  digitalWrite(auxOutput2, LOW);
  digitalWrite(auxOutput3, LOW);
  digitalWrite(auxOutput4, LOW);
  auxOutput1On = false;             
  auxOutput2On = false;             
  auxOutput3On = false;             
  auxOutput4On = false;   

  Serial.print("Sending output pulse on output: ");
  Serial.println(output);  
  
  // Handle each output.
  if ( output == 1 )
  {
    digitalWrite(auxOutput1, HIGH);
    auxOutput1On = true;
    pulseTimer = millis();
  }
  else if ( output == 2 )
  {
    digitalWrite(auxOutput2, HIGH);
    auxOutput2On = true;
    pulseTimer = millis();
  }
  else if ( output == 3 )
  {
    digitalWrite(auxOutput3, HIGH);
    auxOutput3On = true;
    pulseTimer = millis();
  }
  else if ( output == 4 )
  {
    digitalWrite(auxOutput4, HIGH);
    auxOutput4On = true;
    pulseTimer = millis();
  } 
  else
  {
    Serial.print("Unknown output channel: ");
    Serial.print(output);
  }
}

////////////////////////////////////////////////////////////
// Check output pulse.
// If any outputs are on and the timer has expired we 
// turn it off.
////////////////////////////////////////////////////////////
void CheckOutputPulse()
{
  // If any outputs are on.
  if ( auxOutput1On || auxOutput2On || auxOutput3On || auxOutput4On )
  {      
    // If the timer has expired.
    if ( (millis() - pulseTimer) > pulseTime )    
    {  
      // Handle each output.
      if ( auxOutput1On )
      {
        digitalWrite(auxOutput1, LOW);
        auxOutput1On = false;        
      }
      if ( auxOutput2On )
      {
        digitalWrite(auxOutput2, LOW);
        auxOutput2On = false;        
      }
      if ( auxOutput3On )
      {
        digitalWrite(auxOutput3, LOW);
        auxOutput3On = false;        
      }
      if ( auxOutput4On )
      {
        digitalWrite(auxOutput4, LOW);
        auxOutput4On = false;        
      }
    }
  }
}

////////////////////////////////////////////////////////////
// Set PWM.
// Set the LEDs to the given level.
////////////////////////////////////////////////////////////
void SetPWM(bool turnOnPWM)
{
  
  // Are we turning on PWM.
  if ( turnOnPWM == true )
  {        
    Serial.println("Setting strobe_state to strobe_pwm.");
    strobe_state = strobe_pwm; 
    // PWM the LEDs.    
    analogWrite(strobePin, PWMLevel);
    PWMPulseTimer = millis();
    // Set the state.
    PWMOn = true;
  }
  else
  {
    Serial.println("Setting strobe_state to strobe_off.");
    strobe_state = strobe_off;
    // LED output off.
    digitalWrite(strobePin, LOW);   // LEDs off. 
    // Set the state.
    PWMOn = false;  
  }
}

////////////////////////////////////////////////////////////
// Check PWM pulse timer.
// If the PWM timer has expired we turn off the LEDs.
////////////////////////////////////////////////////////////
void CheckPWMPulse()
{ 
  // If the timer has expired.
  if ( (millis() - PWMPulseTimer) > PWMPulseTime )    
  {  
    // LED output off.
    digitalWrite(strobePin, LOW);   // LEDs off.         
    PWMOn = false;
    // Set the state.
    Serial.println("Setting strobe_state to strobe_off.");
    strobe_state = strobe_off;
  }
}

////////////////////////////////////////////////////////////
// Home.
////////////////////////////////////////////////////////////
void Home()
{   
  Serial.println("Starting homing.");    

  // Reset the flag.
  homedTop = false;
  homed = false;
  
  // Reset absolute position.
  absolutePosition = 0;
  
  // Set the start time. If we don't finish homing in a certain time
  // we signal a fault.
  homingStartTime = millis();

  // Set the movement time.
  triggerTime = millis();

  // Check the homing switch. If it is triggered already we need
  // to move down off the switch then home back up.
  if ( digitalRead(homeInductiveSensorPin) == LOW )
  {
    // Switch triggered already so go down.
    Serial.println("Homing switch already triggered. Backing off switch..."); 
        
    // Set the direction to close;
    DirectionClose();

    // Set the state.    
    uppydowny_state = top_homing_down;
    Serial.println("Setting uppydowny_state to top_homing_down.");
  }
  else
  {
    // Switch not triggered so home up.
    Serial.println("Homing up. Looking for switch.");     
    // Set the direction to open;
    DirectionOpen();

    // Set the state.
    Serial.println("Setting uppydowny_state to top_homing_up.");     
    uppydowny_state = top_homing_up;
  }

  // Set the motor state to be homing speed.
  motor_state = motor_homing_speed;
  Serial.println("Setting motor_state to motor_homing_speed.");
  
  // Set the interrupt time for homing. We home at a slower speed
  // to normal movement.
  uint8_t SaveSREG = SREG;          // Save interrupt flag.
  cli();                            // Disable all interrupts.    
  TCNT1 = 0;                        // Reset the counter.
  OCR1A = homingSpeed * 2;          // Compare match register - O.5uS per interrupt.
  TIMSK1 |= (1 << OCIE1A);          // Enable timer compare interrupt
  SREG = SaveSREG;                  // Restore interrupt flag.
  sei();                            // Enable all interrupts.
}
      
////////////////////////////////////////////////////////////
// Check steps. 
// Check how many steps we have done and change state accordingly.
// This is called continually from the main loop.
////////////////////////////////////////////////////////////
void CheckSteps()
{   
  unsigned long timeTaken = 0;
     
  // If we've gone the right number of steps accelerating we go to constant speed.
  if ( (stepCount >= stepsToAccDcc) && (motor_state == motor_accelerating) )
  {
    Serial.println("Accelerating done. Going to constant speed.");
    
    timeTaken = (millis() - movementStartTime);       
    Serial.print("Total accelerating time in mS was: ");
    Serial.println(timeTaken);
    Serial.print("Steps done: ");
    Serial.println(stepCount);    
    
    // Update the state to constant speed.
    motor_state = motor_constant_speed;  
    Serial.println("Setting motor_state to motor_constant_speed.");  
    
    movementStartTime = millis();      
  }  

  // If we've gone the right number of steps in constant speed we start decelerating.
  if ( (stepCount >= (totalSteps - stepsToAccDcc)) && (motor_state == motor_constant_speed) )
  {
    Serial.println("Constant speed done. Starting deceleration.");

    /*
    timeTaken = (millis() - movementStartTime);    
    Serial.print("Total constant speed time in mS was: ");
    Serial.println(timeTaken);
    Serial.print("Steps done: ");
    Serial.println(stepCount);   
    Serial.print("Deceleration should start at step: ");
    Serial.println(totalSteps - stepsToAccDcc);   
    */
    
    // Update the state to deceleration.
    motor_state = motor_decelerating;
    Serial.println("Setting motor_state to motor_decelerating.");
    movementStartTime = millis(); 
    
    // Reset the counters.
    accdccStep = 0;    
    accdccTableIndex = 0;     
  }  
  
  // If we've gone the right number of steps or we are going up and/or we hit the 
  // home switch we can stop.  
  // If opening we add in extra steps to the total to ensure we hit the home switch every time.
  bool homeSwitchHit = !(digitalRead(homeInductiveSensorPin)); // Home switch triggers low so reverse logic.
  
  if ( ((uppydowny_state != top_opening) && (stepCount >= totalSteps) && (motor_state == motor_decelerating))
     || ((uppydowny_state == top_opening) && (stepCount >= totalSteps + openingExtraSteps) && (motor_state == motor_decelerating))
     || ((uppydowny_state == top_opening) && homeSwitchHit) )
  {
    Serial.println("Decelerating done for top opening. Stopping.");
    if ( (uppydowny_state == top_opening) && homeSwitchHit )
    {
      Serial.println("Home switch hit.");
      
      // Set the absolute position.
      Serial.println("Resetting home position.");
      absolutePosition = homePosition;
    }
      
    // Stop the motor.
    uint8_t SaveSREG = SREG;          // Save interrupt flag.
    cli();                            // Disable all interrupts.    
    TIMSK1 &= (0 << OCIE1A);          // Disable timer compare interrupt.        
    SREG = SaveSREG;                  // Restore interrupt flag.
    sei();                            // Enable all interrupts.
    
    timeTaken = (millis() - movementStartTime);    
    Serial.print("Total decelerating time in mS was: ");
    Serial.println(timeTaken);    
    Serial.println();
    Serial.print("Steps done: ");
    Serial.print(stepCount);
    Serial.print(" of total: ");
    Serial.println(totalSteps);
    Serial.print("Difference in steps (should be 0):");
    long difference = stepCount - totalSteps;
    Serial.println(difference);
   
    Serial.print("Current absolute position: ");
    Serial.println(absolutePosition);
    Serial.println();
    
    timeTaken = (millis() - triggerTime);    
    Serial.print("Total movement took: ");
    Serial.print(timeTaken);
    Serial.println(" mS.");
    
    // Update the state to stopped.
    motor_state = motor_stopped;
    Serial.println("Setting motor_state to motor_stopped.");

    // Set the top state.
    if ( uppydowny_state == top_opening )
    {               
       // Set the top state.
       Serial.println("Setting uppydowny_state to top_open.");
       uppydowny_state = top_open;       

       // Set the opened time. 
       openedTime = millis();

       // Send a pulse on channel 2 output.
       SendOutputPulse(2);                    
    }
    else if ( uppydowny_state == top_closing )
    {
       // Set the top state.
       uppydowny_state = top_closed;
       Serial.println("Setting uppydowny_state to top_closed.");

       // Turn off any LEDs of on.
       if ( strobe_state != strobe_off )
       {
          digitalWrite(strobePin, LOW);   // LEDs off.
          PWMOn = false;
        
          // Set the state.
          Serial.println("Setting strobe_state to strobe_off.");
          strobe_state = strobe_off;
       }
       
       // Send a pulse on channel 4 output.
       SendOutputPulse(4);    

       // The cycle is finished so we can re-trigger.
       Serial.println("Setting cycleRunning to false.");
       cycleRunning = false;  

       // Reset the flash flags.
       firstFlashFired = false;
       lastFlashFired = false;
       
       // If we were homing and the top homed we are now fully homed.
       if ( (homed == false) && (homedTop == true) )
       {
          // Now fully homed.
          homed = true;
          Serial.println("Fully homed.");
       }

    }
    else
    {
      // Do nothing here.
    }
  }  

  // If we are homing down check the steps.
  if ((uppydowny_state == top_homing_down) && (stepCount >= homingDownSteps) )
  {   
    Serial.println("Homed down. ");

    // Check the home switch again. If still active something is wrong.
    if ( digitalRead(homeInductiveSensorPin) == LOW )
    {
      // Switch triggered already so go down.
      Serial.println("Homing switch still triggered. Fault!");     
      uppydowny_state = top_fault;
      Serial.println("Setting uppydowny_state to top_fault.");
      // Update the state to stopped.
      motor_state = motor_stopped;
      Serial.println("Setting motor_state to motor_stopped.");
      Stop();      
    }

    // If we aren't in a fault state.
    if ( uppydowny_state != top_fault )
    {
      Serial.println("Homing up to find switch. ");
      // We have gone enough steps down. Now go back up to find the 
      // home position.
      Serial.println("Setting uppydowny state to: top_homing_up");     
      uppydowny_state = top_homing_up;
      
      // Change direction.
      DirectionOpen();   
  
      // Set the motor state to be homing speed.
      motor_state = motor_homing_speed;
      Serial.println("Setting motor_state to motor_homing_speed.");
  
      // Set the interrupt time for homing.
      uint8_t SaveSREG = SREG;          // Save interrupt flag.
      cli();                            // Disable all interrupts.    
      TCNT1 = 0;                        // Reset the counter.
      OCR1A = homingSpeed * 2;          // Compare match register - O.5uS per interrupt.
      TIMSK1 |= (1 << OCIE1A);          // Enable timer compare interrupt
      SREG = SaveSREG;                  // Restore interrupt flag.
      sei();                            // Enable all interrupts.
    }
  }

  // If we are homing up we stop as soon as we hit the home switch.
  if (uppydowny_state == top_homing_up) 
  {    
    
    // If the switch is triggered we are homed so we stop.    
    if ( digitalRead(homeInductiveSensorPin) == LOW )
    {
      Serial.println("Homing switch hit. Home position found."); 

      // Stop the top.
      StopUppyDowny();
      //Set the motor state.
      motor_state = motor_stopped;

      // Set the state.      
      uppydowny_state = top_open;
      Serial.println("Setting uppydowny_state to top_open.");  
      
      // We are homed to the top.       
      homedTop = true;  
      Serial.println("Setting homed top to true.");  
      
      // Set the absolute position.
      absolutePosition = homePosition;
      Serial.println("Setting absolute position to home position.");
      
      // If in maintenance mode we wait until the switch is turned off to continue.
      if ( maintenanceMode == true )
      {
        // Wait for the switch to go off. We do this multiple times with a 
        // delay between them to ensure it is really off (crude debounce!).
        while ( digitalRead(maintenanceSwitchPin) == HIGH )
        {
          // Delay for a bit.
          delay(1000);
          Serial.println("In maintenance mode..."); 
        }
        delay(100);
        while ( digitalRead(maintenanceSwitchPin) == HIGH )
        {
          // Delay for a bit.
          delay(1000);
          Serial.println("In maintenance mode..."); 
        }
        delay(100);
        while ( digitalRead(maintenanceSwitchPin) == HIGH )
        {
          // Delay for a bit.
          delay(1000);
          Serial.println("In maintenance mode..."); 
        }
        // We are now out of maintenance mode.
        maintenanceMode = false;
        Serial.println();
        Serial.println("Maintenance switch off. Entering normal mode."); 
        Serial.println();
        // Reset the homing time so we don't trigger a fault.
        homingStartTime = millis(); 
      }
      
      Serial.println("Setting uppydowny state to: top_closing");
      uppydowny_state = top_closing;
      // Close the top.
      TopClose();
    }
  }

  // If we are moving up stop when the right number of steps are done.
  if ( uppydowny_state == top_moving_up ) 
  {    
    
    // Say if the homing switch is triggered.    
    if ( digitalRead(homeInductiveSensorPin) == LOW )
    {
       Serial.println("Moving and homing switch triggered!"); 
    }

    if ( stepCount >= movingSteps )
    {
       Serial.print("Moving up ");
       Serial.print(movingSteps);
       Serial.println(" done. Stopping.");
       StopAll();
    }
  }

  // If we are moving down stop when the right number of steps are done.
  if ( uppydowny_state == top_moving_down ) 
  {    
    
    // // Say if the homing switch is triggered.    
    if ( digitalRead(homeInductiveSensorPin) == LOW )
    {
       Serial.println("Moving and homing switch triggered!"); 
    }

    if ( stepCount >= movingSteps )
    {
       Serial.print("Moving down ");
       Serial.print(movingSteps);
       Serial.println(" done. Stopping.");
       StopAll();
    }
  }
}
    
////////////////////////////////////////////////////////////
// Accelerate the uppy/downy.
////////////////////////////////////////////////////////////
void UppyDownyAccelerate()
{
  // Reset the step flag.
  didStep = false;
  
  // Use the table to modify the interval based on the SIN curve. 
  
  // Get the index.
  int index = (ACDCTABLESTEPS - 1) - accdccTableIndex;
  
  // Get the value from the table.
  int tableValue = ACDCLookupTable[index];
  
  // Map it to the minimum and maximum intervals.
  unsigned long nextInterval = map (tableValue, 1, 100, stepInterval, accdccInterval);
  
  // Increment the acceleration step counter. 
  accdccStep++;      

  /*  
  Serial.print("Step interval: ");
  Serial.print(stepInterval);
  Serial.print("   ACC DCC interval: ");
  Serial.print(accdccInterval);
  Serial.print("   accdccTableIndex: ");
  Serial.print(accdccTableIndex);
  Serial.print("    index: ");
  Serial.print( index );    
  Serial.print("    tableValue: ");
  Serial.print( tableValue);    
  Serial.print("   Next interval: ");    
  Serial.print(nextInterval);       
  Serial.print("   Acceleration step: ");
  Serial.println(accdccStep);    
  */
  
  // Is it time to go to the next table index?
  if ( accdccStep > (stepsToAccDcc / ACDCTABLESTEPS) )
  {
    // Reset the counter.
    accdccStep = 0;
    
    // Increment the table index.
    if ( accdccTableIndex < ACDCTABLESTEPS )
    {
      accdccTableIndex++;
      //Serial.print("   Acceleration table index: ");
      //Serial.print(accdccTableIndex);  
    } 
  }
    
  // Set the interrupt time.
  uint8_t SaveSREG = SREG;          // Save interrupt flag.
  cli();                            // Disable all interrupts.    
  TCNT1 = 0;                        // Reset the counter.
  OCR1A = nextInterval * 2;         // Compare match register - O.5uS per interrupt.
  TIMSK1 |= (1 << OCIE1A);          // Enable timer compare interrupt
  SREG = SaveSREG;                  // Restore interrupt flag.
  sei();                            // Enable all interrupts.
}            

////////////////////////////////////////////////////////////
// Decelerate the uppy/downy.
////////////////////////////////////////////////////////////
void UppyDownyDecelerate()
{
  // Reset the step flag.
  didStep = false;
    
  // Get the index.
  int index = accdccTableIndex;
  
  // Get the value from the table.
  int tableValue = ACDCLookupTable[index];
  
  // Map it to the minimum and maximum intervals.
  unsigned long nextInterval = map (tableValue, 1, 100, stepInterval, accdccInterval);
  
  // Increment the acceleration step counter. 
  accdccStep++;      
  
  /*
  Serial.print("Step interval: ");
  Serial.print(stepInterval);
  Serial.print("   ACC DCC interval: ");
  Serial.print(accdccInterval);
  Serial.print("   accdccTableIndex: ");
  Serial.print(accdccTableIndex);
  Serial.print("    index: ");
  Serial.print( index );    
  Serial.print("    tableValue: ");
  Serial.print( tableValue);    
  Serial.print("   Next interval: ");    
  Serial.print(nextInterval);       
  Serial.print("   Acceleration step: ");
  Serial.println(accdccStep);  
  */
  
  // Is it time to go to the next table index?
  if ( accdccStep > (stepsToAccDcc / ACDCTABLESTEPS) )
  {
    // Reset the counter.
    accdccStep = 0;
    
    // Increment the table index.
    if ( accdccTableIndex < ACDCTABLESTEPS )
    {
      accdccTableIndex++;
      //Serial.print("   Acceleration table index: ");
      //Serial.print(accdccTableIndex);  
    } 
  }
  
  // Set the interrupt time.
  uint8_t SaveSREG = SREG;          // Save interrupt flag.
  cli();                            // Disable all interrupts.    
  TCNT1 = 0;                        // Reset the counter.
  OCR1A = nextInterval * 2;         // Compare match register - O.5uS per interrupt.
  TIMSK1 |= (1 << OCIE1A);          // Enable timer compare interrupt
  SREG = SaveSREG;                  // Restore interrupt flag.
  sei();                            // Enable all interrupts.
}

////////////////////////////////////////////////////////////
// Run the UppyDowny at constant speed.
////////////////////////////////////////////////////////////
void UppyDownyConstantSpeed()
{
  // Reset the step flag.
  didStep = false;
    
  // Set the interrupt time.
  uint8_t SaveSREG = SREG;          // Save interrupt flag.
  cli();                            // Disable all interrupts.    
  TCNT1 = 0;                        // Reset the counter.
  OCR1A = stepInterval * 2;         // Compare match register - O.5uS per interrupt.  
  TIMSK1 |= (1 << OCIE1A);          // Enable timer compare interrupt  
  SREG = SaveSREG;                  // Restore interrupt flag.
  sei();                            // Enable all interrupts.  
}

////////////////////////////////////////////////////////////
// Activate the strobe.
////////////////////////////////////////////////////////////
void ActivateStrobe()
{
    Serial.println("Activating strobe interrupt.");
    
    digitalWrite(strobePin, LOW);   // LEDs off.
 
    // Turn off PWM mode if in it.
    if ( strobe_state == strobe_pwm )
    {
      DeactivateStrobePWM(); 
    }
    
    // Attach the interrupt to trigger on the falling edge of the trigger.
    attachInterrupt(digitalPinToInterrupt(strobeInductiveSensorPin), FireStrobe, FALLING);

    // Set the state.
    Serial.println("Setting strobe_state to strobe_on.");
    strobe_state = strobe_on;
}

////////////////////////////////////////////////////////////
// Deactivate the strobe.
////////////////////////////////////////////////////////////
void DeactivateStrobe()
{
    Serial.println("Deactivating strobe interrupt.");    
    digitalWrite(strobePin, LOW);   // LEDs off.
    
    // Detach the strobe trigger.
    detachInterrupt(digitalPinToInterrupt(strobeInductiveSensorPin));

    // Set the state.
    Serial.println("Setting strobe_state to strobe_off.");
    strobe_state = strobe_off;
}

////////////////////////////////////////////////////////////
// Fire the strobe interrupt handler.
// Fires on the falling edge of the trigger.
////////////////////////////////////////////////////////////
void FireStrobe()
{ 
  
  // Fire the strobe is we aren't already firing it.
  if ( !strobeTriggered )
  {
    digitalWrite(strobePin, HIGH);    // Fire the strobe.
    strobeTriggered = true;           // Say we are firing.  
    pulseCounter++;                   // Increment the count for RPM monitoring.

    //Serial.println("Strobe triggered!");
    
    uint8_t SaveSREG = SREG;          // Save interrupt flag.
    cli();                            // Disable all interrupts.    
    TCNT3 = 0;                        // Reset the counter.
    TIFR3 = bit (OCF3A);              // Clear any pending interrupt. 
    TIMSK3 |= (1 << OCIE3A);          // Enable timer compare interrupt.
    SREG = SaveSREG;                  // Restore interrupt flag.
    sei();                            // Enable all interrupts.
  }
}

////////////////////////////////////////////////////////////
// Activate strobe PWM mode.
////////////////////////////////////////////////////////////
void ActivateStrobePWM(unsigned int value)
{
  
  Serial.println("Activating strobe PWM mode.");
  digitalWrite(strobePin, LOW);   // LEDs off.
  
  // If the strobe is in strobe mode deactivate it.
  if ( strobe_state == strobe_on )
  {
    DeactivateStrobe();  
  }

  // If the percentage is under the minimum raise it to the minimum.
  if ( value < minPWMValue )
  {
    Serial.print("Value: ");
    Serial.print(value);
    Serial.print(" is less than minimum of: ");
    Serial.print(minPWMValue);
    Serial.println(" so raising.");
    // Limit the minimum value possible.
    value = minPWMValue;
  }
  
  // If the percentage is over the maximum limit it to the maximum.
  if ( value > maxPWMValue )
  {
    Serial.print("Value: ");
    Serial.print(value);
    Serial.print(" is more than maximum of: ");
    Serial.print(maxPWMValue);
    Serial.println(" so raising.");
    // Limit the maximum value possible.
    value = maxPWMValue;
  }

  Serial.print("Setting PWM to: ");
  Serial.println(value);

  // Set the state.
  Serial.println("Setting strobe_state to strobe_pwm.");
  strobe_state = strobe_pwm;

  // PWM the LEDs.
  analogWrite(strobePin, value);
}
  
////////////////////////////////////////////////////////////
// Strobe PWM off.
////////////////////////////////////////////////////////////
void DeactivateStrobePWM()
{
  Serial.println("Deactivating strobe PWM mode.");   
  digitalWrite(strobePin, LOW);   // LEDs off.
  
  strobeTriggered = false;        // Reset the strobe trigger.
  lastStrobeTriggered = false;       
  
  // Set the state.
  Serial.println("Setting strobe_state to strobe_off.");
  strobe_state = strobe_off;
}

////////////////////////////////////////////////////////////
// Start disc spinning.
////////////////////////////////////////////////////////////
void DiscStart()      
{               
  Serial.println("Roundyroundy internal Speed 1 (start).");
  digitalWrite(SP1Pin, HIGH);
}

////////////////////////////////////////////////////////////
// Stop disc spinning.
////////////////////////////////////////////////////////////
void DiscStop()      
{               
  Serial.println("Roundyroundy internal Speed 2 (stop).");
  digitalWrite(SP1Pin, LOW);
}

////////////////////////////////////////////////////////////
// Open top.
////////////////////////////////////////////////////////////
void TopOpen()      
{               
  Serial.println("Opening top.");

  // Set the direction.
  DirectionOpen();

  // Start accelerating.
  Serial.println("Starting acceleration.");
  UppyDownyAccelerate();
}

////////////////////////////////////////////////////////////
// Close top.
////////////////////////////////////////////////////////////
void TopClose()      
{               
  Serial.println("Closing top.");

  // Set the direction.
  DirectionClose();

  // Start accelerating.
  Serial.println("Starting acceleration.");
  UppyDownyAccelerate();

}

////////////////////////////////////////////////////////////
// Direction open.
////////////////////////////////////////////////////////////
void DirectionOpen()
{
  // Set the appropriate pin.
  digitalWrite(dirPin, LOW);

  // Set the direction flag;
  opening = true;

  // Set the state for the motor. 
  motor_state = motor_accelerating;
  
  // Update the movement start time.
  movementStartTime = millis(); 
  
  // Reset the counters.
  accdccStep = 0;    
  accdccTableIndex = 0;  
  
  // Reset the steps.
  stepCount = 0;
    
  Serial.println("Setting direction to open.");
}   

////////////////////////////////////////////////////////////
// Direction close.
////////////////////////////////////////////////////////////
void DirectionClose()
{
  // Set the appropriate pin.
  digitalWrite(dirPin, HIGH);

  // Set the direction flag;
  opening = false;
  
  // Set the state for the motor. 
  motor_state = motor_accelerating; 

  // Update the movement start time.
  movementStartTime = millis(); 
  
  // Reset the counters.
  accdccStep = 0;    
  accdccTableIndex = 0;   

  // Reset the steps.
  stepCount = 0;
   
  Serial.println("Setting direction to close.");
}   

////////////////////////////////////////////////////////////
// Stop. This is a normal stop.
////////////////////////////////////////////////////////////
void Stop()
{
  // Stop uppy downy!
  StopUppyDowny();
  
  // Stop roundy roundy!
  StopRoundyRoundy();
}

////////////////////////////////////////////////////////////
// Stop uppydowny.
////////////////////////////////////////////////////////////
void StopUppyDowny()
{
  // Stop uppydowny!
  uint8_t SaveSREG = SREG;          // Save interrupt flag.
  cli();                            // Disable all interrupts.      
  TIMSK1 &= (0 << OCIE1A);          // Disable timer compare interrupt.        
  SREG = SaveSREG;                  // Restore interrupt flag.
  sei();                            // Enable all interrupts.
}

////////////////////////////////////////////////////////////
// Stop roundyroundy.
////////////////////////////////////////////////////////////
void StopRoundyRoundy()
{
  // Stop roundy roundy!
  digitalWrite(SP1Pin, LOW);
}

////////////////////////////////////////////////////////////
// SerialDataAvailable.
// Read until we have a line then set the command
// received flag.
////////////////////////////////////////////////////////////
void SerialDataAvailable(void)
{
  // Get the data.
  byte byteRead = Serial.read();

  // Have we read the whole line?
  if (byteRead == '\n')
  {
    // Set the flag to say we have a command.
    commandReceived = true;
  }
  else
  {
    // Put it in the command buffer.
    if ( commandBufferIndex < commandBufferSize - 1 )
    {
      commandBuffer[commandBufferIndex] = byteRead;      
      // Null terminate.
      commandBuffer[commandBufferIndex + 1] = 0;
      commandBufferIndex++;
    }     
    else
    {
      Serial.println("Command buffer full.");
      // Set the flag to say we have a command.
      commandReceived = true;
    }
  }
}

////////////////////////////////////////////////////////////
// Tokenise the command buffer.
// Token pointer stored in commandTokenArray.
////////////////////////////////////////////////////////////
int TokeniseCommandBuffer()
{
  int commandIndex = 0;

  // Clear the array.
  for (int i = 0; i < numberOfCommandEntries; i++)
  {
    commandTokenArray[i] = 0;
  }

  // Get the first part.
  commandTokenArray[commandIndex] = strtok(commandBuffer, commandDelimiter);

  // Now get the rest.
  while ( commandTokenArray[commandIndex] != NULL )
  {
    // Increment the index.
    commandIndex++;
    commandTokenArray[commandIndex] = strtok(NULL, commandDelimiter);
  }

  return commandIndex;
}

////////////////////////////////////////////////////////////
// StopAll.
////////////////////////////////////////////////////////////
void StopAll(void)
{

  Serial.println("Stopping everything.");
  
  // Disc stop.
  DiscStop();
  
  // Up down stop.
  // Disable interrupts.
  uint8_t SaveSREG = SREG;          // Save interrupt flag.
  cli();                            // Disable all interrupts.     
  TIMSK1 &= (0 << OCIE1A);          // Disable timer compare interrupt.        
  SREG = SaveSREG;                  // Restore interrupt flag.
  sei();                            // Enable all interrupts.
    
  // Set up the motor state.
  motor_state = motor_stopped; 
  Serial.println("Setting motor_state to motor_stopped.");

  // Set uppy downy state.
  uppydowny_state = top_unknown;
  Serial.println("Setting uppydowny_state to top_unknown.");

  // Stop the strobe.
  if ( strobe_state == strobe_on )
  {
    DeactivateStrobe();  
  }
  if ( strobe_state == strobe_pwm )
  {
    DeactivateStrobePWM();  
  }
  
}

////////////////////////////////////////////////////////////
// Up.
////////////////////////////////////////////////////////////
void Up(void)
{
  // Stop everything.
  StopAll();  

  // Set the direction to open;
  DirectionOpen();

  // Set uppy downy state.
  uppydowny_state = top_moving_up;
  Serial.println("Setting uppydowny_state to top_moving_up."); 
  
  // Set the interrupt time for moving. 
  uint8_t SaveSREG = SREG;          // Save interrupt flag.
  cli();                            // Disable all interrupts.    
  TCNT1 = 0;                        // Reset the counter.
  OCR1A = movingSpeed * 2;          // Compare match register - O.5uS per interrupt.
  TIMSK1 |= (1 << OCIE1A);          // Enable timer compare interrupt
  SREG = SaveSREG;                  // Restore interrupt flag.
  sei();                            // Enable all interrupts.
}

////////////////////////////////////////////////////////////
// Down.
////////////////////////////////////////////////////////////
void Down(void)
{
  // Stop everything.
  StopAll();  

  // Set the direction to close;
  DirectionClose();

  // Set uppy downy state.
  uppydowny_state = top_moving_down;
  Serial.println("Setting uppydowny_state to top_moving_down."); 
  
  // Set the interrupt time for moving. 
  uint8_t SaveSREG = SREG;          // Save interrupt flag.
  cli();                            // Disable all interrupts.    
  TCNT1 = 0;                        // Reset the counter.
  OCR1A = movingSpeed * 2;          // Compare match register - O.5uS per interrupt.
  TIMSK1 |= (1 << OCIE1A);          // Enable timer compare interrupt
  SREG = SaveSREG;                  // Restore interrupt flag.
  sei();                            // Enable all interrupts.
}

////////////////////////////////////////////////////////////
// ProcessCommand.
////////////////////////////////////////////////////////////
void ProcessCommand(void)
{
  bool validCommand = false;
  
  // Handle the commands.
  Serial.print("Command received: ");
  Serial.println(commandBuffer);

  // Tokenise it.
  int commandCount = TokeniseCommandBuffer();

  while(1)
  {        
    // Stop.
    // Stop all movement.
    if (strncmp(commandTokenArray[0], "x", 1) == 0 )
    {
      StopAll();
      validCommand = true;
      break;
    }    

    // Simulate trigger.
    if (strncmp(commandTokenArray[0], "trigger", 7) == 0 )
    {
      triggered = true;
      validCommand = true;
      break;
    }

    // Move up the given number of steps.
    if (strncmp(commandTokenArray[0], "u", 1) == 0 )
    {
      // Homing state.
      homedTop = false;
      homed = false;
  
      if (commandCount == 1)
      {
        Serial.println("Number of steps not specified.");
      }
      else
      {              
        // Set the number of steps to move.
        movingSteps = atoi(commandTokenArray[1]);

        // Move up.
        Up();
        validCommand = true;
        break;
      }
    }

    // Move down the given number of steps.
    if (strncmp(commandTokenArray[0], "d", 1) == 0 )
    {
      // Homing state.
      homedTop = false;
      homed = false;
      
      if (commandCount == 1)
      {
        Serial.println("Number of steps not specified.");
      }
      else
      {              
        // Set the number of steps to move.
        movingSteps = atoi(commandTokenArray[1]);

        // Move down.
        Down();
        validCommand = true;
        break;
      }
    }

    // Spin.
    // Start the main disc spinning.
    if (strncmp(commandTokenArray[0], "spin", 4) == 0 )
    {
      DiscStart();
      validCommand = true;
      break;
    }

    // Do a homing cycle.
    if (strncmp(commandTokenArray[0], "home", 4) == 0 )
    {
      Home();
      validCommand = true;
      break;
    }    

    // Strobe on.
    if (strncmp(commandTokenArray[0], "strobeon", 8) == 0 )
    {      
      ActivateStrobe();
      validCommand = true;
      break;
    }

    // Strobe off.
    if (strncmp(commandTokenArray[0], "strobeoff", 9) == 0 )
    {
      // Stop the strobe.
      if ( strobe_state == strobe_on )
      {
        DeactivateStrobe();  
      }
      if ( strobe_state == strobe_pwm )
      {
        DeactivateStrobePWM();  
      }
      validCommand = true;
      break;
    }

    // Strobe pwm.
    if (strncmp(commandTokenArray[0], "strobepwm", 9) == 0 )
    {
      if (commandCount == 1)
      {
        Serial.println("PWM percentage not specified.");
      }
      else
      {              
        // Call the PWM function with the  correct percentage.
        ActivateStrobePWM(atoi(commandTokenArray[1]));
        validCommand = true;
      }
      break;
    }
    
    // Outputs status info.
    if (strncmp(commandTokenArray[0], "status", 5) == 0 )
    {
      Status();
      validCommand = true;
      break;
    }
    
    // Enable displaying the RPM.
    if (strncmp(commandTokenArray[0], "rpm", 3) == 0 )
    {
      // Invert the rpm display boolean.
      displayRPM = !displayRPM;
      // Reset the pulse counter.
      pulseCounter = 0;
      validCommand = true;
      break;
    }     

    // Flash the strobe.
    if (strncmp(commandTokenArray[0], "f", 1) == 0 )
    {
      // Flash the strobes.
      FireStrobe();
      validCommand = true;
      break;
    }  

    // Toggle outputs on or off.
    if (strncmp(commandTokenArray[0], "toggle", 6) == 0 )
    {           
      if (commandCount == 1)
      {
        Serial.println("Output to toggle not specified.");
      }
      else
      {              
        // Get the channel to toggle.
        int outputChannel = atoi(commandTokenArray[1]);
        Serial.print("Toggling output channel: ");
        Serial.print(outputChannel);
        Serial.print(" to: ");
          
        if ( outputChannel == 1 )
        {           
          if ( auxOutput1On )
          {            
            Serial.println("OFF");
            // Toggle the channel.
            digitalWrite(auxOutput1, LOW);
            // Set the state.
            auxOutput1On = false;            
          }
          else
          {
            Serial.println("ON");
            // Toggle the channel.
            digitalWrite(auxOutput1, HIGH);
            // Set the state.
            auxOutput1On = true;
          }
          pulseTimer = millis();
        }        
        else if ( outputChannel == 2 )
        {           
          if ( auxOutput2On )
          {            
            Serial.println("OFF");
            // Toggle the channel.
            digitalWrite(auxOutput2, LOW);
            // Set the state.
            auxOutput2On = false;
          }
          else
          {
            Serial.println("ON");
            // Toggle the channel.
            digitalWrite(auxOutput2, HIGH);
            // Set the state.
            auxOutput2On = true;
          }
          pulseTimer = millis();
        }    
        else if ( outputChannel == 3 )
        {           
          if ( auxOutput3On )
          {            
            Serial.println("OFF");
            // Toggle the channel.
            digitalWrite(auxOutput3, LOW);
            // Set the state.
            auxOutput3On = false;
          }
          else
          {
            Serial.println("ON");
            // Toggle the channel.
            digitalWrite(auxOutput3, HIGH);
            // Set the state.
            auxOutput3On = true;
          }
          pulseTimer = millis();
        }
        else if ( outputChannel == 4 )
        {           
          if ( auxOutput4On )
          {            
            Serial.println("OFF");
            // Toggle the channel.
            digitalWrite(auxOutput4, LOW);
            // Set the state.
            auxOutput4On = false;
          }
          else
          {
            Serial.println("ON");
            // Toggle the channel.
            digitalWrite(auxOutput4, HIGH);
            // Set the state.
            auxOutput4On = true;
          }
          pulseTimer = millis();
        }
        else
        {
          Serial.println("Channel not supported!");
        }
        
        validCommand = true;
        break;
      }
    }
    
    // Outputs help info.
    if (strncmp(commandTokenArray[0], "help", 4) == 0 )
    {
      Help();
      validCommand = true;
      break;
    }
  
    if ( !validCommand )
    {
      Serial.println("Command not known. Type \"help\" for a list of commands.");
      break;
    }
  }

  // Reset the command buffer.
  // Must do this last!
  commandBufferIndex = 0;
  commandBuffer[commandBufferIndex] = 0;
  commandReceived = false;
}

////////////////////////////////////////////////////////////
// Help.
////////////////////////////////////////////////////////////
void Help(void)
{
  Serial.println("\n - Commands available -");
  Serial.println("\"x\"\n\tStop all.");  
  Serial.println("\"trigger\"\n\tTrigger a cycle.");
  Serial.println("\"u <x>\"\n\tMove up x steps.");
  Serial.println("\"d <x>\"\n\tMove down x steps.");
  Serial.println("\"spin\"\n\tStart spinning.");
  Serial.println("\"home\"\n\tPerform a homing cycle.");  
  Serial.println("\"strobeon\"\n\tEnable strobes.");
  Serial.println("\"strobeoff\"\n\tDisable strobes.");
  Serial.println("\"strobepwm <x>\"\n\tStrobe PWM value 1 - 255.");
  Serial.println("\"status\"\n\tStatus information.");
  Serial.println("\"rpm\"\n\tShow RPM.");
  Serial.println("\"f\"\n\tFire strobe.");
  Serial.println("\"help\"\n\tThis information.");
  Serial.println("\"toggle <n>\"\n\tToggle the output on channel n <1 to 4>.");
  Serial.println("");
}

////////////////////////////////////////////////////////////
// Status.
////////////////////////////////////////////////////////////
void Status(void)
{
  Serial.println("\n - Status -");

  Serial.print("Zoetrope controller version: ");  
  Serial.println(version_string);  
  Serial.print("runTime: "); Serial.println(runTime);
  Serial.print("spinStartDelay: "); Serial.println(spinStartDelay);
  Serial.print("spinStopDelay: "); Serial.println(spinStopDelay);
  Serial.print("openDelay: "); Serial.println(openDelay);
  Serial.print("strobeStartDelay: "); Serial.println(strobeStartDelay);
  Serial.print("strobeStopDelay: "); Serial.println(strobeStopDelay);
  Serial.print("lastFlashDelay: "); Serial.println(lastFlashDelay);
  Serial.print("pulseTime: "); Serial.println(pulseTime);

  if ( homed )
  {
    Serial.print("Current position:");
    Serial.println(absolutePosition);  
    Serial.print("Home position: ");
    Serial.println(homePosition);  
  }  
  else
  {
    Serial.println("Not homed so position not currently known.");
  }

  Serial.print("uppydowny_state: ");
  if (uppydowny_state == top_unknown) Serial.println("top_unknown");
  if (uppydowny_state == top_opening) Serial.println("top_opening");
  if (uppydowny_state == top_opening_wait) Serial.println("top_opening_wait");
  if (uppydowny_state == top_closing) Serial.println("top_closing");
  if (uppydowny_state == top_homing_up) Serial.println("top_homing_up");
  if (uppydowny_state == top_homing_down) Serial.println("top_homing_down");
  if (uppydowny_state == top_moving_up) Serial.println("top_moving_up");
  if (uppydowny_state == top_moving_down) Serial.println("top_moving_down");
  if (uppydowny_state == top_fault) Serial.println("top_fault");

  Serial.print("motor_state: ");
  if (motor_state == motor_unknown) Serial.println("motor_unknown");
  if (motor_state == motor_stopped) Serial.println("motor_stopped");
  if (motor_state == motor_accelerating) Serial.println("motor_accelerating");
  if (motor_state == motor_decelerating) Serial.println("motor_decelerating");
  if (motor_state == motor_constant_speed) Serial.println("motor_constant_speed");
  if (motor_state == motor_homing_speed) Serial.println("motor_homing_speed");
  if (motor_state == motor_moving_speed) Serial.println("motor_moving_speed");
  if (motor_state == motor_fault) Serial.println("motor_fault");
  
  Serial.print("roundyroundy_state: ");
  if (roundyroundy_state == disc_unknown) Serial.println("disc_unknown");
  if (roundyroundy_state == disc_start) Serial.println("disc_start");
  if (roundyroundy_state == disc_start_wait) Serial.println("disc_start_wait");
  if (roundyroundy_state == disc_stop) Serial.println("disc_stop");
  if (roundyroundy_state == disc_stop_wait) Serial.println("disc_stop_wait");

  Serial.print("strobe_state: ");
  if (strobe_state == strobe_unknown) Serial.println("strobe_unknown");
  if (strobe_state == strobe_on) Serial.println("strobe_on");
  if (strobe_state == strobe_off) Serial.println("strobe_off");
  if (strobe_state == strobe_pwm) Serial.println("strobe_pwm");

  // Output 1.
  if ( auxOutput1On )
  {
    Serial.println("Output 1: ON");
  }
  else
  {
    Serial.println("Output 1: OFF");
  }

  // Auxiliary output 2.
  if ( auxOutput2On )
  {
    Serial.println("Output 2: ON");
  }
  else
  {
    Serial.println("Output 2: OFF");
  }

  // Output 3.
  if ( auxOutput3On )
  {
    Serial.println("Output 3: ON");
  }
  else
  {
    Serial.println("Output 3: OFF");
  }

  // Output 4.
  if ( auxOutput4On )
  {
    Serial.println("Output 4: ON");
  }
  else
  {
    Serial.println("Output 4: OFF");
  }
}

// ----------------------------------------------------------------------
// Handle Serial.
// ----------------------------------------------------------------------
void HandleSerial()        
{
  // Check for USB serial data.
  if ( (Serial.available()) && (commandReceived == false) )
  {
    // Handle the data.
    SerialDataAvailable();
  }

  // If the command was received handle it.
  if ( commandReceived == true )
  {
    ProcessCommand();
  }
}

// ----------------------------------------------------------------------
// Check strobe.
// We do this check to prevent overloading the strobes.
// ----------------------------------------------------------------------
void CheckStrobes()
{
  // If the strobe state changed.
  if ( lastStrobeTriggered != strobeTriggered )
  {
     // And the strobe was fired.
     if ( strobeTriggered )
     {
       // Remember the time.
       strobeLastOnTime = millis();
       lastStrobeTriggered = true;
     }
  }
  
  // Check the strobe. If it has been on for too long turn it off!
  if ( (strobeTriggered) && (millis() - strobeLastOnTime > strobeMaximumOnTime) ) 
  {
    // Turn the strobe off.
    Serial.println("Turning strobe off!");
    digitalWrite(strobePin, LOW);
    strobeTriggered = false;
    lastStrobeTriggered = false;
  }
}

// ----------------------------------------------------------------------
// Handle acceleration/deceleration
// ----------------------------------------------------------------------
void HandleAccelerationDeceleration()        
{
  // Are we accelerating?
  if ( (motor_state == motor_accelerating) && (didStep == true) )
  {  
    // Accelerate.
    UppyDownyAccelerate();
  }    

  // Are we at constant speed?
  if ( (motor_state == motor_constant_speed) && (didStep == true) )
  {
    // Constant speed.
    UppyDownyConstantSpeed();
  }

  // Are we decelerating?
  if ( (motor_state == motor_decelerating) && (didStep == true) )
  {  
    // Decelerate.
    UppyDownyDecelerate();
  }
}
        
// ----------------------------------------------------------------------
// Main loop.
// ----------------------------------------------------------------------
void loop()                     
{ 

  // Handle any serial data.
  HandleSerial();

  // Check PWM pulses.
  if ( PWMOn == true)
  {
    CheckPWMPulse();
  }

  // Check output pulses.
  CheckOutputPulse();

  // Check the steps.
  CheckSteps();  
  
  // Check the strobes.
  //CheckStrobes();  

  // If we aren't running a cycle look for triggers.
  if ( cycleRunning == false )
  {
    // Check for trigger.
    triggerSignal.update();
    if ( triggerSignal.fell() )
    {
      // We are triggered externally.
      Serial.println();
      Serial.println("External trigger received."); 
      Serial.println(); 
      triggered = true;
    }
  
    // Check for test trigger.  
    bool testTriggerSwitchOn;
    testTriggerSwitch.update();
    if ( testTriggerSwitch.read() == HIGH )
    {
      testTriggerSwitchOn = true;
    }
    else
    {
      testTriggerSwitchOn = false;
    }

    // If it was on.
    if ( (testTriggerSwitchOn == true) && (homed == true) )
    {
      // Set the flag.
      testSwitchOn = true;
        
      // Check the time to see if we can trigger again.
      if ( (millis() - lastTestTriggerTime) > testTriggerPeriod )
      { 
        // We are triggered from the test switch.
        Serial.println();
        Serial.println("Test trigger received.");                    
        triggered = true;
        lastTestTriggerTime = millis();
      }
      else
      {
        // If enough time has passed print out when we'll do the next test cycle.
        if ( ((millis() - lastTestTriggerTime) % nextTestPrintTime) == 0 )
        {
          Serial.println();
          Serial.print("Will retrigger in: ");  
          Serial.print(testTriggerPeriod - (millis() - lastTestTriggerTime) );  
          Serial.println(" mS");  
        }
      }
    }
    
    if ( (testTriggerSwitchOn == false) && (testSwitchOn == true) )
    {
      // If the switch isn't on we reset the time.
      lastTestTriggerTime = 0;
      
      // And reset the test state.
      testSwitchOn = false;

      // We aren't triggered.
      triggered = false;

      Serial.println("Resetting trigger period.");
    }
  }

  // If we were triggered.
  if ( triggered == true )
  {
    Serial.println("Cycle initiated.");
    cycleCount++;
    Serial.print("Cycle number: ");
    Serial.println(cycleCount);    
    // If we aren't homed we ignore the trigger.
    if ( !homed )
    {
      Serial.println("Not homed so ignoring trigger!");
      
      // Reset the trigger.    
      triggered = false;
    }           
  }

  // Display the RPM if that option is enabled.
  if ( displayRPM )
  {
    // Check the RPM every 24 frames.
    if ( pulseCounter == 24 )
    {   
     unsigned long rev_timer = (millis() - RPMTimer);
     float rps = (float)rev_timer / (float)1000;
     rps = 1/rps;
     float rpm = rps * 60;
     Serial.print("LED Hz: ");
     Serial.print(rps * 24);
     Serial.print("   RPS: ");
     Serial.print(rps);    
     Serial.print("   RPM: ");
     Serial.println(rpm);  
     RPMTimer = millis();
     pulseCounter = 0;
    }
  }
  
  // If we are homed.
  if (homedTop) 
  {
    // Handle any acceleration/deceleration.
    HandleAccelerationDeceleration();
    
    // If we aren't already running and we were triggered.
    if ( (cycleRunning == false) && (triggered == true) )  
    {  
      // Reset the trigger.    
      triggered = false;      

      // If we are closed we open. 
      // Set the states to waiting.
      if ( uppydowny_state == top_closed ) 
      {
        // Set up the top state to waiting to open.
        Serial.println("Setting uppydowny state to: top_opening_wait");
        uppydowny_state = top_opening_wait;

        // Set up the spin state to waiting to spin.
        Serial.println("Setting roundyroundy state to: disc_start_wait");
        roundyroundy_state = disc_start_wait;

        // Reset the time.
        triggerTime = millis();

        // Set the cycling flag.
        Serial.println("Setting cycleRunning to true.");
        cycleRunning = true;
      }   
      else
      {
        Serial.println("Not in correct state (uppydowny_state == top_closed) to start cycle.");    
      }
    }    

    // Are we waiting to start opening?
    if ( (uppydowny_state == top_opening_wait) && (millis() - triggerTime >= openDelay)  )
    {
        Serial.println("Opening top.");
        // Set up the top state to opening.
        Serial.print("Waited: ");
        Serial.print(millis() - triggerTime);
        Serial.print(" mS (");
        Serial.print(openDelay);
        Serial.println(")");
        Serial.println("Setting uppydowny state to: top_opening");
        uppydowny_state = top_opening;
        
        // Open the top.
        TopOpen();

        // Send a pulse on channel 1 output.
        SendOutputPulse(1);        
    }

    // Are we waiting to start spinning?
    if ( (roundyroundy_state == disc_start_wait) && (millis() - triggerTime >= spinStartDelay)  )
    {
        Serial.println("Starting rotation.");
        Serial.print("Waited: ");
        Serial.print(millis() - triggerTime);
        Serial.print(" mS (");
        Serial.print(spinStartDelay);
        Serial.println(")");
        Serial.println("Setting roundyroundy state to: disc_start");
        roundyroundy_state = disc_start;        
        // Disc spin up.        
        DiscStart();
    }

    // Are we waiting to start the strobe?
    // Trigger when the top is opening and the motor starts decelerating and the right delay has passed.
    if ( (uppydowny_state == top_opening) && (motor_state == motor_decelerating) && (millis() - openedTime >= strobeStartDelay) && (strobe_state == strobe_off)  )    
    {
       Serial.println("Starting strobe.");
       Serial.print("Waited: ");
       Serial.print(millis() - openedTime);
       Serial.print(" mS (");
       Serial.print(strobeStartDelay);
       Serial.println(")");
       // Activate strobes.
       ActivateStrobe();  
       // Set the strobe started time.
       strobeStartedTime = millis();     
    }

    // Are we ready to close?
    if ( (uppydowny_state == top_open) && (millis() - strobeStartedTime >= runTime)  )
    {   
        Serial.println("Closing top.");     
        Serial.print("Waited: ");
        Serial.print(millis() - strobeStartedTime);
        Serial.print(" mS (");
        Serial.print(runTime);
        Serial.println(")");
        Serial.println("Setting uppydowny state to: top_closing");
        uppydowny_state = top_closing;
        // Close the top.
        TopClose();
        Serial.println("Setting roundyroundy state to: disc_stop_wait");
        roundyroundy_state = disc_stop_wait;                
        // Set the closing time.
        closingTime = millis();
        // Send a pulse on channel 3 output.
        SendOutputPulse(3);
    }
    
    // Are we ready to stop spinning.
    if ( (roundyroundy_state == disc_stop_wait) && (millis() - closingTime >= spinStopDelay)  )
    {   
        Serial.println("Stopping rotation.");     
        Serial.print("Waited: ");
        Serial.print(millis() - closingTime);
        Serial.print(" mS (");
        Serial.print(spinStopDelay);
        Serial.println(")");        
        Serial.println("Setting roundyroundy state to: disc_stop");
        roundyroundy_state = disc_stop;
        // Disc spin down.
        DiscStop();
    }
            
    // Are we ready to turn off the strobe?
    if ( (uppydowny_state == top_closing) && (millis() - closingTime >= strobeStopDelay) && (strobe_state == strobe_on) )
    {   
        Serial.println("Deactivating strobe.");     
        Serial.print("Waited: ");
        Serial.print(millis() - closingTime);
        Serial.print(" mS (");
        Serial.print(strobeStopDelay);
        Serial.println(")");
        // Deactivate strobes.
        DeactivateStrobe();        
    }       

    // Are we ready to fire the first flash.
    if ( (cycleRunning == true) && (firstFlashFired == false) && (PWMOn == false) && (uppydowny_state == top_opening) && (millis() - triggerTime >= firstFlashDelay) )
    { 
      Serial.println("Firing first flash of light.");       
      Serial.print("Waited: ");
      Serial.print(millis() - triggerTime);
      Serial.print(" mS (");
      Serial.print(firstFlashDelay);
      Serial.println(")");
      // Deactivate strobes if on.
      if ( strobe_state == strobe_on ) 
      {
        DeactivateStrobe();
      }
      // PWM strobes if not already on.
      if ( strobe_state == strobe_off ) 
      {
        // Set the time for the pulse.
        PWMPulseTime = PWMPulseTime1;
        SetPWM(true);
      }
      // Set the flag so we don't retrigger the flash.
      firstFlashFired = true;
    }
    
    // Are we ready to fire the final flash.
    if ( (cycleRunning == true) && (lastFlashFired == false) && (PWMOn == false) && (uppydowny_state == top_closing) && (millis() - closingTime >= lastFlashDelay) )
    { 
      Serial.println("Firing last flash of light.");              
      Serial.print("Waited: ");
      Serial.print(millis() - closingTime);
      Serial.print(" mS (");
      Serial.print(lastFlashDelay);
      Serial.println(")");
      // Deactivate strobes if on.
      if ( strobe_state == strobe_on ) 
      {
        DeactivateStrobe();
      }
      // PWM strobes if not already on.
      if ( strobe_state == strobe_off ) 
      {
        // Set the time for the pulse.
        PWMPulseTime = PWMPulseTime2;
        SetPWM(true);
      }
      // Set the flag so we don't retrigger the flash.
      lastFlashFired = true;
    }
  }
  else // Not homed.
  {
    // If we are homing.
    if ( (uppydowny_state == top_homing_up) || (uppydowny_state == top_homing_down) )
    {
  
      // If not homed yet check the time. If over the allocated time we signal a fault.
      if ( (millis() - homingStartTime > homingTimeout) && (uppydowny_state != top_fault) )
      {
        // Homing hasn't happened in time so stop and signal a fault.
        Serial.print("Homing didn't happen within: ");
        Serial.print(homingTimeout);
        Serial.println(" mS. Fault!");
        
        uppydowny_state = top_fault;
        Serial.println("Setting uppydowny state to: top_fault");

        // Update the state to stopped.
        motor_state = motor_stopped;
        Serial.println("Setting motor_state to motor_stopped.");
        
        // And stop.
        Stop();
      }
    }
  }
}
