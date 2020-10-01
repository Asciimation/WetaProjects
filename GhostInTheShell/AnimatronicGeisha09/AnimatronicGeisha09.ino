// Author: Simon Jansen

// Includes. 
#include <WetaServo.h>                    // Weta servo object library.
#include <Servo.h>                        // Servo library.
#include <Dynamixel_Serial_Teensy.h>      // Library needed to control Dynamixal servo.
#include <Encoder.h>                      // Rotary encoder library.
#include <EEPROM.h>                       // EEprom library.
#include <Picaso_Serial_4DLib.h>          // LCD Library.      
#include <Picaso_const4D.h>               // LCD Library.     

// Change this to a 1 to use the hard coded values.
const int useHardcoded = 0;

// THESE SHOULD NOT BE CHANGED - HARDWARE SPECIFIC DEFINES!
// Defines for hardware pins for fixed I/O switches.
const int DEBOUNCE = 10;                  // Debounce time in mS.
const int NUMBUTTONS = 10;                // Number of buttons.
const int pb1 = 3;                        // 1. Push button 1. 
const int pb2 = 4;                        // 2. Push button 2. 
const int pbExit = 5;                     // 3. Exit push button. 
const int pbEnter = 6;                    // 4. Enter push button.
const int pbPanic = 22;                   // 5. Panic switch.
const int eb5 = 19;                       // 6. Extra button 1.
const int eb4 = 20;                       // 7. Extra button 2.
const int eb3 = 21;                       // 8. Extra button 3.
const int eb2 = 28;                       // 9. Extra button 4.
const int eb1 = 29;                       // 10. Extra button 5.

const int rcpin = 26 ;                    // RC Receiver input pin.

//const int minRC = 950;                  // Theoretical limits. 
//const int maxRC = 2100;

const int minRC = 950;                   // Values to make it work.
const int maxRC = 1930;
const int midRC = 1520;
const int rcDeadZone = 0;


// Variables for RC receiver.
volatile uint16_t lastValue;
volatile uint16_t channel;
volatile uint16_t channel_start;
volatile bool gotRCValue;
bool RCEnabled = false;

byte buttons[] = {pb1, pb2, pbExit, pbEnter, pbPanic, eb1, eb2, eb3, eb4, eb5}; 
byte pressed[NUMBUTTONS];
byte justpressed[NUMBUTTONS];
byte justreleased[NUMBUTTONS];              

// Defines and variables for the rotary encoder.
const int encoderA = 11;                  // Rotary encoder A.
const int encoderB = 12;                  // Rotary encoder B.
const int led1 = 18;                      // LED 1 - BLUE.
const int led2 = 14;                      // LED 2 - GREEN.
const int led3 = 15;                      // LED 3 - RED.
const int encoderSwitch = 16;             // Rotary encoder switch.

// LEDs.
const int ledButton5 = 31;                // LED button 5.
const int ledButton4 = 25;                // LED button 4.
const int ledButton3 = 32;                // LED button 3.
const int ledButton2 = 24;                // LED button 2.
const int ledButton1 = 33;                // LED button 1 . 

Encoder rotaryEncoder(encoderA, encoderB);  // Encoder object.
int encoderPosition = 0;                    // Encoder position.
int oldEncoderPosition = 0;                 // Encoder previous position.
int encoderButton = 0;                      // Rotary encoder button state.

// Defines and variables for the LCD screen.
const int screenResetPin = 17;            // Reset pin for LCD.
const int SCREENLINES = 25;               // Number of lines to show on the screen.

// Define the hardware serial port for Serial 2.
// Magic needed to get the screen UART to work.
HardwareSerial2 Uart = HardwareSerial2();
Picaso_Serial_4DLib Display(&Uart); 

// THESE SHOULD BE CHANGED TO MODIFY THE BEHAVIOUR OF THE CONTROLLER.
// Menus of operation.
// These define the menu options so we know what mode we are in.
const int OPTION_FIRST = 1;                   // First menu item.
const int MENU_MAIN = 1;                      // Select menu menu.
const int DYNAMIXEL_SELECT = 2;               // Select Dynamixel.
const int SETUP = 3;                          // Setup menu.
const int SEQUENCE1 = 4;                      // Sequence 1 menu.
const int SEQUENCE2 = 5;                      // Sequence 2 menu.
const int SEQUENCE3 = 6;                      // Sequence 3 menu.
const int RCCONTROL = 7;                      // RC Control.
const int RESET = 8;                          // Reset menu.
const int FLACCID = 9;                        // Turn on or off holding torque on all Dynamixels.
const int PRINTVARIABLES = 10;                // Print out all variables.
const int MENU_LAST = 10;                     // Last menu item.

// THESE SHOULD BE CHANGED TO ADD NEW SETUP VARIABLES.
// Dynamixel menu variables.
const int FIRST_DYNAMIXEL_VARIABLE = 1;
const int CW_SPEED = 1;                    // Speed clockwise.
const int CCW_SPEED = 2;                   // Speed counterclockwise.
const int ACDC = 3;                        // Acceleration/deceleration.
const int CW_LIMIT = 4;                    // CW limit.
const int CCW_LIMIT = 5;                   // CCW limit.
const int MOVE_CW_LIMIT = 6;               // Visually adjust CW limits.
const int MOVE_CCW_LIMIT = 7;              // Visually adjust CCW limits.
const int MANUAL_CW_LIMIT = 8;             // Manually adjust CW limits.
const int MANUAL_CCW_LIMIT = 9;            // Manually adjust CCW limits.
const int SET_HOLDING = 10;                // Turn on and off the holding torque.
const int SET_ENABLED = 11;                // Enable or disable.
const int TEST_SERVO_LIMITS = 12;          // Test moving the servo in SERVO mode.
const int TEST_SERVO_WHEEL = 13;           // Test moving the servo in WHEEL mode.
const int LAST_DYNAMIXEL_VARIABLE = 13;

// Generic variables.
// First we define constants for each variable we'll have that needs
// setting. 
const int FIRST_VARIABLE = 1;
const int TIME_SCALE = 1;                 // Time scaling factor.
const int PAUSE_1 = 2;                    // Pause 1.
const int PAUSE_2 = 3;                    // Pause 1.
const int PAUSE_3 = 4;                    // Pause 1.
const int BLIP_1 = 5;                     // Blip 1.
const int SLOW_SPEED = 6;                 // Speed scaling factor.
const int ITERATIONS = 7;                 // Iterations.
const int LAST_VARIABLE = 7;

// Next define the actual variables.
// Dynamixel 1 variables.
int d1_cw_speed = 0;
int d1_ccw_speed = 0;
int d1_acdc = 0;
int d1_cw_limit = 2048;
int d1_ccw_limit = 2048;
// Dynamixel 2 variables.
int d2_cw_speed = 2048;
int d2_ccw_speed = 2048;
int d2_acdc = 2048;
int d2_cw_limit = 2048;
int d2_ccw_limit = 2048;
// Dynamixel 3 variables.
int d3_cw_speed = 0;
int d3_ccw_speed = 0;
int d3_acdc = 0;
int d3_cw_limit = 2048;
int d3_ccw_limit = 2048;
// Dynamixel 4 variables.
int d4_cw_speed = 0;
int d4_ccw_speed = 0;
int d4_acdc = 0;
int d4_cw_limit = 2048;
int d4_ccw_limit = 2048;
// Dynamixel 5 variables.
int d5_cw_speed = 0;
int d5_ccw_speed = 0;
int d5_acdc = 0;
int d5_cw_limit = 2048;
int d5_ccw_limit = 2048;
// Dynamixel 6 variables.
int d6_cw_speed = 0;
int d6_ccw_speed = 0;
int d6_acdc = 0;
int d6_cw_limit = 2048;
int d6_ccw_limit = 2048;

// Other variables.
int time_scale = 0;
int pause_1 = 0;
int pause_2 = 0;
int pause_3 = 0;
int blip_1 = 0;
int slow_speed_factor = 0;
int iterations = 0;

// DEFINES FOR THE SERVOS IN USE.
// Array of pointers to standard Servos.
StandardServo* standardServoArray[NUMSTANDARDSERVOS];    

// Array of pointers to Dynamixel servos.
DynamixelServo* dynamixelServoArray[NUMDYNAMIXELSERVOS];   
// Generic Dynamixel servo pointer.
DynamixelServo* dynamixel;                 

// Configure normal servos being used.
//StandardServo servo1(16);                   // Standard servo 1.
//StandardServo servo2(17);                   // Standard servo 2.
//StandardServo servo3(18);                   // Standard servo 3.

// Configure Dynamixel servos being used.
// We pass in the Dynamixel ID and a name.
DynamixelServo dynamixel1(21, "Left cheek", "Open", "Close");                    
DynamixelServo dynamixel2(22, "Left eye", "Open", "Close");                                      
DynamixelServo dynamixel3(23, "Right cheek", "Close", "Open");                    
DynamixelServo dynamixel4(24, "Right eye", "Close", "Open");                    
DynamixelServo dynamixel5(25, "Chin", "Close", "Open");                    
DynamixelServo dynamixel6(26, "Chin sides", "Open", "Close");                               

// General veriables.
int menu = MENU_MAIN;
int oldmenu = MENU_MAIN;
bool editing_variable = false; 
int dynamixelIndex = 0;
bool selected_dynamixel = false;
bool editing_dynamixel_variable = false;
int screenLines = 0;
int buttonState = 0;
int exitButtonState = 0;
int setup_variable = 0;
int setup_dynamixel_variable = 0;
int oldValue = 0;
int goalPosition = 0;
bool eyesOpen = false;
bool cheeksOpen = false;
bool chinsidesOpen = false;
bool chinOpen = false;

// Variables used in the RUN sequence.
unsigned long sequenceStartTime = 0;
unsigned long sequenceTime1 = 0;
int sequenceDelay1 = 0;
int sequenceStep = 0;
int count = 0;
bool testRun = false;
unsigned long tm = 0; 

////////////////////////////////////////////////////////////
// RC channel interrupt handler routine.
////////////////////////////////////////////////////////////   
void RCchannel() 
{
  // If the pin is HIGH, start a timer.
  if ( digitalRead(rcpin) == HIGH ) 
  {
      channel_start = micros();
  } 
  else 
  {
      // The pin is now LOW so output the difference
      // between when the timer was started and now.
      channel = (uint16_t) (micros() - channel_start);
      //Serial.println(channel);
      gotRCValue = true;
  }
}

////////////////////////////////////////////////////////////  
// Setup function.
////////////////////////////////////////////////////////////
void setup() 
{

  // Start USB serial port.
  Serial.begin(115200);

  // Set up IO pins.
  // Outputs.
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(screenResetPin, OUTPUT);
  pinMode(ledButton1, OUTPUT);
  pinMode(ledButton2, OUTPUT);
  pinMode(ledButton3, OUTPUT);
  pinMode(ledButton4, OUTPUT);
  pinMode(ledButton5, OUTPUT);  
  
  // Inputs.
  // Set internal pull ups on I/O button pins.
  pinMode(pb1, INPUT_PULLUP);
  pinMode(pb2, INPUT_PULLUP);
  pinMode(pbExit, INPUT_PULLUP);
  pinMode(pbEnter, INPUT_PULLUP);
  pinMode(pbPanic, INPUT_PULLUP);
  pinMode(eb1, INPUT_PULLUP);
  pinMode(eb2, INPUT_PULLUP);
  pinMode(eb3, INPUT_PULLUP);
  pinMode(eb4, INPUT_PULLUP);
  pinMode(eb5, INPUT_PULLUP);
    
  // Rotaty encoder switch pulls high, no internal pull up.
  pinMode(encoderSwitch, INPUT); 

  // Set up RC receiver input pin.
  pinMode(rcpin, INPUT);

  // RC is not enabled.
  RCEnabled = false;

  // Sequencing variables.
  sequenceStartTime = 0;           
  sequenceTime1 = 0;
  sequenceDelay1 = 0;
  sequenceStep = 0;
  count = 0;
  testRun = 0;
  tm = 0;
  slow_speed_factor = 0;
  
  // Clear the servo arrays.
  for ( int i = 0; i < NUMSTANDARDSERVOS; i++ )
  {
    standardServoArray[i] = NULL;
  }  
  for ( int i = 0; i < NUMDYNAMIXELSERVOS; i++ )
  {
    dynamixelServoArray[i] = NULL;
  }
  
  // Configure any standard servos.
  //standardServoArray[0] = &servo1;
  //standardServoArray[1] = &servo2;
  //standardServoArray[2] = &servo3;

  // Configure Dynamixel servos.
  Dynamixel.begin(HWSERIAL, SERVO_SET_Baudrate);    // Pass in the serial port to use and set the baud rate.  
  Dynamixel.setDirectionPin(SERVO_ControlPin);      // Set direction control pin.   
  Dynamixel.setStatusPaket(254, 2);                 // Set return packet state.                        
  delay(3000);                                      // Give the bloody things time to start.
   
  // Start the LCD display.
  screenLines = 0;
  Serial.println("Resetting LCD.");
  digitalWrite(screenResetPin, LOW);
  delay(1);
  digitalWrite(screenResetPin, HIGH);  
  delay(1000);               
  // Start LCD serial port.
  Uart.begin(9600);                                 // Screen baud rate (default 9600).
  Serial.println("Setting screen error callback.");
  Display.Callback4D = screenErrorCallback;         // Error call back handler.
  Display.TimeLimit4D = 5000;                       // 5 second timeout on all commands.                      
  Serial.println("Cursor to 0,0.");
  Display.gfx_MoveTo(0, 0);   
  Serial.println("Clearing the screen.");
  Display.gfx_Cls();                                // Clear the screen.  
  Display.txt_FGcolour(LIGHTGREEN);                    
  Serial.println("Screen started.");
  
    // Flash so we know we started.
  PrintString("Reading EEProm...", 1);
  BlinkyLights(); 
  
  // Set the menu variables.
  menu = MENU_MAIN;
  oldmenu = menu;
  editing_variable = false;
  selected_dynamixel = false;
  editing_dynamixel_variable = false;

  // Read the initial encoder value.
  encoderPosition = rotaryEncoder.read();
  oldEncoderPosition = encoderPosition;
  encoderButton = 0;
  
  // Setup variables for editing.
  setup_variable = FIRST_VARIABLE;
  setup_dynamixel_variable = FIRST_DYNAMIXEL_VARIABLE;

  if ( useHardcoded )
  {   
    Serial.println("Using hardcoded values.");
    
    // HARDCODED VALUES
    // Use these to initially set up a Teensy with the correct EEprom values.
    //
    d1_cw_speed = 1023;
    d1_ccw_speed = 805;
    d1_acdc = 0;
    d1_cw_limit = 1919;
    d1_ccw_limit = 2262;
    d2_cw_speed = 1023;
    d2_ccw_speed = 1023;
    d2_acdc = 0;
    d2_cw_limit = 1926;
    d2_ccw_limit = 2283;
    d3_cw_speed = 802;
    d3_ccw_speed = 1023;
    d3_acdc = 0;
    d3_cw_limit = 2018;
    d3_ccw_limit = 2316;
    d4_cw_speed = 1023;
    d4_ccw_speed = 1023;
    d4_acdc = 0;
    d4_cw_limit = 341;
    d4_ccw_limit = 672;
    d5_cw_speed = 533;
    d5_ccw_speed = 1023;
    d5_acdc = 0;
    d5_cw_limit = 2061;
    d5_ccw_limit = 2315;
    d6_cw_speed = 1022;
    d6_ccw_speed = 453;
    d6_acdc = 0;
    d6_cw_limit = 927;
    d6_ccw_limit = 1161;
    time_scale = 100;
    slow_speed_factor = 2;
    pause_1 = 500;
    pause_2 = 500;
    pause_3 = 500;
    blip_1 = 100;
    iterations = 1;


    
    WriteEEprom();
  }

    // Read all the stored values.
  ReadEEprom();
    
  // Print them out.
  //PrintVariables();

  // Assign the Dynamixels into the array.
  dynamixelServoArray[0] = &dynamixel1;
  dynamixelServoArray[1] = &dynamixel2;  
  dynamixelServoArray[2] = &dynamixel3;
  dynamixelServoArray[3] = &dynamixel4;
  dynamixelServoArray[4] = &dynamixel5;
  dynamixelServoArray[5] = &dynamixel6;

  // Start the Dynamixels.
  dynamixel1.Start(SERVO, d1_cw_speed, d1_ccw_speed, d1_acdc, d1_cw_limit, d1_ccw_limit);
  dynamixel2.Start(SERVO, d2_cw_speed, d2_ccw_speed, d2_acdc, d2_cw_limit, d2_ccw_limit); 
  dynamixel3.Start(SERVO, d3_cw_speed, d3_ccw_speed, d3_acdc, d3_cw_limit, d3_ccw_limit); 
  dynamixel4.Start(SERVO, d4_cw_speed, d4_ccw_speed, d4_acdc, d4_cw_limit, d4_ccw_limit); 
  dynamixel5.Start(SERVO, d5_cw_speed, d5_ccw_speed, d5_acdc, d5_cw_limit, d5_ccw_limit); 
  dynamixel6.Start(SERVO, d6_cw_speed, d6_ccw_speed, d6_acdc, d6_cw_limit, d6_ccw_limit);     
  
  // Set the time scaling.
  SetDynamixelsTimeScale(time_scale);
     
  PrintString("Controller started.", 1);
  
  // Print the menu.
  PrintString("-----|", 0);
  PrintMenu(menu);
  PrintString("|-----", 1);
}

////////////////////////////////////////////////////////////
// Return scaled time value.
////////////////////////////////////////////////////////////  
unsigned long ScaleTime(unsigned long tm)
{
  unsigned long temp = (tm * 100) / time_scale;
  return temp;      
}

////////////////////////////////////////////////////////////
// Print a string to the LCD and serial port with LF.
////////////////////////////////////////////////////////////  
void PrintString(const String& stringValue, int LF)
{  
  // Check the screen lines. 
  if ( screenLines > SCREENLINES )
  {
    // Wrap the screen.
    Display.gfx_MoveTo(0, 0);   
    Display.gfx_Cls(); 
    screenLines = 0;
  } 
  // If no linefeed required.       
  if ( !LF )
  {
    Serial.print(stringValue);
    Display.print(stringValue);
  }
  else
  {     
    // Write the new text.
    Serial.println(stringValue);
    Display.println(stringValue);
    // Increment the number of lines.      
    screenLines++;
  }
}

////////////////////////////////////////////////////////////
// Print a value to the LCD and serial port.
////////////////////////////////////////////////////////////  
void PrintValue(const int value, int LF)
{
  char buf[64] = {'\0'};
  
  // Convert value.
  itoa(value, buf, 10);

  // Check the screen lines.  
  if ( screenLines > SCREENLINES )
  {
    // Wrap the screen.
    Display.gfx_MoveTo(0, 0);   
    Display.gfx_Cls(); 
    screenLines = 0;
  }
  // If a linefeed is required.
  if ( !LF )
  {
    Serial.print(buf);
    Display.print(buf);
  }
  else
  {    
    // Write the new text.
    Serial.println(buf);
    Display.println(buf);   
    // Increment the number of lines. 
    screenLines++;   
  }
}

////////////////////////////////////////////////////////////
// Blink the lights.
////////////////////////////////////////////////////////////  
void BlinkyLights(void)
{
  EncoderOff(); 
  digitalWrite(ledButton5, LOW);
  digitalWrite(ledButton4, LOW);
  digitalWrite(ledButton3, LOW);
  digitalWrite(ledButton2, LOW);
  digitalWrite(ledButton1, LOW);
  delay(200);
  EncoderOrange();
  digitalWrite(ledButton1, HIGH);
  digitalWrite(ledButton5, HIGH);
  delay(200);
  EncoderPurple();
  digitalWrite(ledButton2, HIGH);
  digitalWrite(ledButton4, HIGH);
  delay(200);
  EncoderOff();
  digitalWrite(ledButton3, HIGH);  
}

////////////////////////////////////////////////////////////
// Light the encoder red.
////////////////////////////////////////////////////////////  
void EncoderRed(void)
{
  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, LOW);
}

////////////////////////////////////////////////////////////
// Light the encoder green.
////////////////////////////////////////////////////////////  
void EncoderGreen(void)
{
  digitalWrite(led1, HIGH);
  digitalWrite(led2, LOW);
  digitalWrite(led3, HIGH);
}

////////////////////////////////////////////////////////////
// Light the encoder blue.
////////////////////////////////////////////////////////////  
void EncoderBlue(void)
{
  digitalWrite(led1, LOW);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
}

////////////////////////////////////////////////////////////
// Light the encoder cyan.
////////////////////////////////////////////////////////////  
void EncoderCyan(void)
{
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, HIGH);
}

////////////////////////////////////////////////////////////
// Light the encoder Purple.
////////////////////////////////////////////////////////////  
void EncoderPurple(void)
{
  digitalWrite(led1, LOW);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, LOW);  
}

////////////////////////////////////////////////////////////
// Light the encoder Orange.
////////////////////////////////////////////////////////////  
void EncoderOrange(void)
{
  digitalWrite(led1, HIGH);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
}

////////////////////////////////////////////////////////////
// Light the encoder White.
////////////////////////////////////////////////////////////  
void EncoderWhite(void)
{
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
}

////////////////////////////////////////////////////////////
// Turn off the encoder LED.
////////////////////////////////////////////////////////////  
void EncoderOff(void)
{
  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
}

////////////////////////////////////////////////////////////
// Display a binary number on the encoder LED.
////////////////////////////////////////////////////////////  
void BinaryToLED(byte value)
{
  
  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
  if (value & 1)
  {
    digitalWrite(led1, LOW);
  }
  value = value >> 1;
  
  if (value & 1)
  {
    digitalWrite(led2, LOW);
  }
  value = value >> 1;
    
  if (value & 1)
  {
    digitalWrite(led3, LOW);
  }
  value = value >> 1;
  
}

////////////////////////////////////////////////////////////
// Light up the given LED.
////////////////////////////////////////////////////////////  
void LightLED( int LED, bool on )
{
  if ( LED == 0 )
  { 
    if ( on )
    {
      // LEDs off.
      digitalWrite(ledButton1, LOW);
      digitalWrite(ledButton2, LOW);
      digitalWrite(ledButton3, LOW);
      digitalWrite(ledButton4, LOW);
      digitalWrite(ledButton5, LOW);
    }
    else
    {
      // LEDs off.
      digitalWrite(ledButton1, HIGH);
      digitalWrite(ledButton2, HIGH);
      digitalWrite(ledButton3, HIGH);
      digitalWrite(ledButton4, HIGH);
      digitalWrite(ledButton5, HIGH);
    }
  }
  else if ( LED == 1 )
  {
    if ( on ) 
    {
      digitalWrite(ledButton1, LOW);
    }
    else 
    {
      digitalWrite(ledButton1, HIGH);
    }
  }
  else if ( LED == 2 )
  {
    if ( on ) 
    {
      digitalWrite(ledButton2, LOW);
    }
    else 
    {
      digitalWrite(ledButton2, HIGH);
    }
  }
  else if ( LED == 3 )
  {
    if ( on ) 
    {
      digitalWrite(ledButton3, LOW);
    }
    else 
    {
      digitalWrite(ledButton3, HIGH);
    }
  }
  else if ( LED == 4 )
  {
    if ( on ) 
    {
      digitalWrite(ledButton4, LOW);
    }
    else 
    {
      digitalWrite(ledButton4, HIGH);
    }
  }
  else if ( LED == 5 )
  {
   if ( on ) 
    {
      digitalWrite(ledButton5, LOW);
    }
    else 
    {
      digitalWrite(ledButton5, HIGH);
    }
  }
}
   
////////////////////////////////////////////////////////////
// Write an int to the EEprom.
////////////////////////////////////////////////////////////  
void EEpromWriteInt(uint16_t address, int value)
{
   uint16_t originalAddress = address;
   int originalValue = value;

   //PrintString("EEpromWriteInt address: ", 0); 
   //PrintString(address, 0);
   //PrintString(" - Value: ", 0); 
   //PrintString(value, 1);
     
   // Check the current value in EEPROM.
   int currentValue = EEpromReadInt(originalAddress);

   // If different we have to write the new one.
   if ( currentValue != value )
   {
     byte lowByte = ((value >> 0) & 0xFF);
     byte highByte = ((value >> 8) & 0xFF);
       
     EEPROM.write(address, lowByte);
     address++;
     EEPROM.write(address, highByte);
     address++;

     PrintString("Writing to EEprom.", 1); 
     //PrintString("Values different. Wrote value: ", 0); 
     //PrintString(originalValue, 1);
     //PrintString("at address: ", 0); 
     //PrintString(originalAddress, 1);
     //PrintString("Reading back value from address: ", 0);     
     //PrintString(originalAddress, 1);
  
     int readValue = EEpromReadInt(originalAddress);
     if (readValue != originalValue) 
     {
       PrintString("Write/Read mismatch", 1);
       PrintString("Address: ", 0);
       PrintString(originalAddress, 1);
       PrintString("Read value: ", 0);     
       PrintString(readValue, 1);
       PrintString("Expected value: ", 0);
       PrintString(originalValue, 1);
     }
     else
     {
      PrintString("Value correctly written.", 1);
     }
   }  
}

////////////////////////////////////////////////////////////
// Read an int from the EEprom.
////////////////////////////////////////////////////////////  
unsigned int EEpromReadInt(uint16_t address)
{
  int returnValue = 0;

  //PrintString("Reading from address: ", 0); 
  //PrintString(address, 1);
  
  byte lowByte = EEPROM.read(address);
  address++;
  byte highByte = EEPROM.read(address);
  address++;

  returnValue = highByte;
  returnValue = returnValue << 8;
  returnValue = returnValue + lowByte;
  
  //PrintString("Value: ", 0);     
  //PrintString(returnValue, 1);

  return returnValue;
}

////////////////////////////////////////////////////////////  
// Read from EEProm.
// Any new variables added need to be added here too.
////////////////////////////////////////////////////////////
void ReadEEprom() 
{
  uint16_t address = 0;
   
  Serial.println("Reading EEProm.");
   
  // Dynamixel 1.
  d1_cw_speed = EEpromReadInt(address); 
  address = address + 2; // 2
  d1_ccw_speed = EEpromReadInt(address); 
  address = address + 2; // 4
  d1_acdc = EEpromReadInt(address);  
  address = address + 2; // 6
  d1_cw_limit = EEpromReadInt(address);  
  address = address + 2; // 8
  d1_ccw_limit = EEpromReadInt(address);  
  address = address + 2; // 10
  
  // Dynamixel 2.  
  d2_cw_speed = EEpromReadInt(address);  
  address = address + 2; // 12
  d2_ccw_speed = EEpromReadInt(address);  
  address = address + 2; // 14
  d2_acdc = EEpromReadInt(address);  
  address = address + 2; // 16
  d2_cw_limit = EEpromReadInt(address);  
  address = address + 2; // 18
  d2_ccw_limit = EEpromReadInt(address);  
  address = address + 2; // 20
  
  // Dynamixel 3.
  d3_cw_speed = EEpromReadInt(address);  
  address = address + 2; // 22
  d3_ccw_speed = EEpromReadInt(address);  
  address = address + 2; // 24
  d3_acdc = EEpromReadInt(address);  
  address = address + 2; // 26
  d3_cw_limit = EEpromReadInt(address);  
  address = address + 2; // 28
  d3_ccw_limit = EEpromReadInt(address);  
  address = address + 2; // 30

  // Dynamixel 4.
  d4_cw_speed = EEpromReadInt(address);  
  address = address + 2; // 32
  d4_ccw_speed = EEpromReadInt(address);  
  address = address + 2; // 34
  d4_acdc = EEpromReadInt(address);  
  address = address + 2; // 36
  d4_cw_limit = EEpromReadInt(address);  
  address = address + 2; // 38
  d4_ccw_limit = EEpromReadInt(address);  
  address = address + 2; // 40

  // Dynamixel 5.
  d5_cw_speed = EEpromReadInt(address);  
  address = address + 2; // 42
  d5_ccw_speed = EEpromReadInt(address);  
  address = address + 2; // 44
  d5_acdc = EEpromReadInt(address);  
  address = address + 2; // 46
  d5_cw_limit = EEpromReadInt(address);  
  address = address + 2; // 48
  d5_ccw_limit = EEpromReadInt(address);  
  address = address + 2; // 50

  // Dynamixel 6.
  d6_cw_speed = EEpromReadInt(address);  
  address = address + 2; // 52
  d6_ccw_speed = EEpromReadInt(address);  
  address = address + 2; // 54
  d6_acdc = EEpromReadInt(address);  
  address = address + 2; // 56
  d6_cw_limit = EEpromReadInt(address);  
  address = address + 2; // 58
  d6_ccw_limit = EEpromReadInt(address);  
  address = address + 2; // 60


  // Other variables.
  time_scale = EEpromReadInt(address);  
  address = address + 2; // 62
  slow_speed_factor = EEpromReadInt(address);  
  address = address + 2; // 64
  pause_1 = EEpromReadInt(address);  
  address = address + 2; // 66
  pause_2 = EEpromReadInt(address);  
  address = address + 2; // 68
  pause_3 = EEpromReadInt(address);  
  address = address + 2; // 70
  blip_1 = EEpromReadInt(address);  
  address = address + 2; // 72
  iterations = EEpromReadInt(address); 
}

////////////////////////////////////////////////////////////  
// Write to the EEProm.
// Any new variables added need to be added here too.
////////////////////////////////////////////////////////////
void WriteEEprom() 
{  
  uint16_t writeAddress = 0;  
  
  //Serial.println("Writing EEprom.");
  
  // Dynamixel 1.
  EEpromWriteInt(writeAddress, d1_cw_speed); 
  writeAddress = writeAddress + 2; // 2
  EEpromWriteInt(writeAddress, d1_ccw_speed); 
  writeAddress = writeAddress + 2; // 4
  EEpromWriteInt(writeAddress, d1_acdc);
  writeAddress = writeAddress + 2; // 6
  EEpromWriteInt(writeAddress, d1_cw_limit);
  writeAddress = writeAddress + 2; // 8
  EEpromWriteInt(writeAddress, d1_ccw_limit);
  writeAddress = writeAddress + 2; // 10

  // Dynamixel 2.
  EEpromWriteInt(writeAddress, d2_cw_speed);
  writeAddress = writeAddress + 2; // 12
  EEpromWriteInt(writeAddress, d2_ccw_speed);
  writeAddress = writeAddress + 2; // 14
  EEpromWriteInt(writeAddress, d2_acdc);
  writeAddress = writeAddress + 2; // 16
  EEpromWriteInt(writeAddress, d2_cw_limit);
  writeAddress = writeAddress + 2; // 18
  EEpromWriteInt(writeAddress, d2_ccw_limit);
  writeAddress = writeAddress + 2; //20

  // Dynamixel 3.
  EEpromWriteInt(writeAddress, d3_cw_speed);
  writeAddress = writeAddress + 2; // 22
  EEpromWriteInt(writeAddress, d3_ccw_speed);
  writeAddress = writeAddress + 2; // 24
  EEpromWriteInt(writeAddress, d3_acdc);
  writeAddress = writeAddress + 2; // 26 
  EEpromWriteInt(writeAddress, d3_cw_limit);
  writeAddress = writeAddress + 2; // 28
  EEpromWriteInt(writeAddress, d3_ccw_limit);
  writeAddress = writeAddress + 2; // 30

  // Dynamixel 4.
  EEpromWriteInt(writeAddress, d4_cw_speed);
  writeAddress = writeAddress + 2; // 32
  EEpromWriteInt(writeAddress, d4_ccw_speed);
  writeAddress = writeAddress + 2; // 34 
  EEpromWriteInt(writeAddress, d4_acdc);
  writeAddress = writeAddress + 2; // 36
  EEpromWriteInt(writeAddress, d4_cw_limit);
  writeAddress = writeAddress + 2; // 38
  EEpromWriteInt(writeAddress, d4_ccw_limit);
  writeAddress = writeAddress + 2; // 40

  // Dynamixel 5.
  EEpromWriteInt(writeAddress, d5_cw_speed);
  writeAddress = writeAddress + 2; // 42
  EEpromWriteInt(writeAddress, d5_ccw_speed);
  writeAddress = writeAddress + 2; // 44 
  EEpromWriteInt(writeAddress, d5_acdc);
  writeAddress = writeAddress + 2; // 46
  EEpromWriteInt(writeAddress, d5_cw_limit);
  writeAddress = writeAddress + 2; // 48
  EEpromWriteInt(writeAddress, d5_ccw_limit);
  writeAddress = writeAddress + 2; // 50

  // Dynamixel 6.
  EEpromWriteInt(writeAddress, d6_cw_speed);
  writeAddress = writeAddress + 2; // 52
    EEpromWriteInt(writeAddress, d6_ccw_speed);
  writeAddress = writeAddress + 2; // 54 
  EEpromWriteInt(writeAddress, d6_acdc);
  writeAddress = writeAddress + 2; // 56
  EEpromWriteInt(writeAddress, d6_cw_limit);
  writeAddress = writeAddress + 2; // 58
  EEpromWriteInt(writeAddress, d6_ccw_limit);
  writeAddress = writeAddress + 2; // 60
  
  EEpromWriteInt(writeAddress, time_scale);
  writeAddress = writeAddress + 2; // 62
  EEpromWriteInt(writeAddress, slow_speed_factor);
  writeAddress = writeAddress + 2; // 64
  EEpromWriteInt(writeAddress, pause_1);
  writeAddress = writeAddress + 2; // 66
  EEpromWriteInt(writeAddress, pause_2);
  writeAddress = writeAddress + 2; // 68
  EEpromWriteInt(writeAddress, pause_3);
  writeAddress = writeAddress + 2; // 70
  EEpromWriteInt(writeAddress, blip_1);
  writeAddress = writeAddress + 2; // 72
  EEpromWriteInt(writeAddress, iterations);      
}

////////////////////////////////////////////////////////////  
// Print EEPROM variables.
// Any new variables added need to be added here too.
////////////////////////////////////////////////////////////
void PrintVariables()
{
   
  PrintString("Variables:", 1);
   
  // Dynamixel 1.
  PrintString("d1_cw_speed = ", 0); PrintString(d1_cw_speed, 0); PrintString(";", 1);
  PrintString("d1_ccw_speed = ", 0); PrintString(d1_ccw_speed, 0); PrintString(";", 1);
  PrintString("d1_acdc = ", 0); PrintString(d1_acdc, 0); PrintString(";", 1);
  PrintString("d1_cw_limit = ", 0); PrintString(d1_cw_limit, 0); PrintString(";", 1);  
  PrintString("d1_ccw_limit = ", 0); PrintString(d1_ccw_limit, 0); PrintString(";", 1);

  // Dynamixel 2.
  PrintString("d2_cw_speed = ", 0); PrintString(d2_cw_speed, 0); PrintString(";", 1);
  PrintString("d2_ccw_speed = ", 0); PrintString(d2_ccw_speed, 0); PrintString(";", 1);
  PrintString("d2_acdc = ", 0); PrintString(d2_acdc, 0); PrintString(";", 1);
  PrintString("d2_cw_limit = ", 0); PrintString(d2_cw_limit, 0); PrintString(";", 1);  
  PrintString("d2_ccw_limit = ", 0); PrintString(d2_ccw_limit, 0); PrintString(";", 1);

  // Dynamixel 3.
  PrintString("d3_cw_speed = ", 0); PrintString(d3_cw_speed, 0); PrintString(";", 1);
  PrintString("d3_ccw_speed = ", 0); PrintString(d3_ccw_speed, 0); PrintString(";", 1);
  PrintString("d3_acdc = ", 0); PrintString(d3_acdc, 0); PrintString(";", 1);
  PrintString("d3_cw_limit = ", 0); PrintString(d3_cw_limit, 0); PrintString(";", 1);  
  PrintString("d3_ccw_limit = ", 0); PrintString(d3_ccw_limit, 0); PrintString(";", 1);

  // Dynamixel 4.
  PrintString("d4_cw_speed = ", 0); PrintString(d4_cw_speed, 0); PrintString(";", 1);
  PrintString("d4_ccw_speed = ", 0); PrintString(d4_ccw_speed, 0); PrintString(";", 1);
  PrintString("d4_acdc = ", 0); PrintString(d4_acdc, 0); PrintString(";", 1);
  PrintString("d4_cw_limit = ", 0); PrintString(d4_cw_limit, 0); PrintString(";", 1);  
  PrintString("d4_ccw_limit = ", 0); PrintString(d4_ccw_limit, 0); PrintString(";", 1);

  // Dynamixel 5.
  PrintString("d5_cw_speed = ", 0); PrintString(d5_cw_speed, 0); PrintString(";", 1);
  PrintString("d5_ccw_speed = ", 0); PrintString(d5_ccw_speed, 0); PrintString(";", 1);
  PrintString("d5_acdc = ", 0); PrintString(d5_acdc, 0); PrintString(";", 1);
  PrintString("d5_cw_limit = ", 0); PrintString(d5_cw_limit, 0); PrintString(";", 1);  
  PrintString("d5_ccw_limit = ", 0); PrintString(d5_ccw_limit, 0); PrintString(";", 1);

  // Dynamixel 6.
  PrintString("d6_cw_speed = ", 0); PrintString(d6_cw_speed, 0); PrintString(";", 1);
  PrintString("d6_ccw_speed = ", 0); PrintString(d6_ccw_speed, 0); PrintString(";", 1);
  PrintString("d6_acdc = ", 0); PrintString(d6_acdc, 0); PrintString(";", 1);
  PrintString("d6_cw_limit = ", 0); PrintString(d6_cw_limit, 0); PrintString(";", 1);  
  PrintString("d6_ccw_limit = ", 0); PrintString(d6_ccw_limit, 0); PrintString(";", 1);

  // Other variables.
  PrintString("time_scale = ", 0); PrintString(time_scale, 0); PrintString(";", 1);
  PrintString("slow_speed_factor = ", 0); PrintString(slow_speed_factor, 0); PrintString(";", 1);
  PrintString("pause_1 = ", 0); PrintString(pause_1, 0); PrintString(";", 1);
  PrintString("pause_2 = ", 0); PrintString(pause_2, 0); PrintString(";", 1);
  PrintString("pause_3 = ", 0); PrintString(pause_3, 0); PrintString(";", 1);
  PrintString("blip_1 = ", 0); PrintString(blip_1, 0); PrintString(";", 1);
  PrintString("iterations = ", 0); PrintString(iterations, 0); PrintString(";", 1);
  
}

////////////////////////////////////////////////////////////
// Call to return a pointer to a Dynamixel servo based on
// the servo ID.
////////////////////////////////////////////////////////////
DynamixelServo* FindDynamixalByID (byte id)

{
  // Loop through all Dynamixel servos looking for the one with
  // the correct ID.
  for (int i = 0; i < NUMDYNAMIXELSERVOS; i++)
  {
    // If we find it return the pointer to it.
    if ( dynamixelServoArray[i]->GetID() == id )
    {
      return dynamixelServoArray[i];
    }    
  }
  // If not found return null.
  return NULL;   
}


////////////////////////////////////////////////////////////
// Call service all Dynamixels.
// This routine needs to be called in a loop continually
// to update the Dynamixels. Call it from the loop()
// function.
////////////////////////////////////////////////////////////
void ServiceDynamixels (void)

{
  // Loop through all Dynamixel servos looking for those
  // that need servicing.
  for (int i = 0; i < NUMDYNAMIXELSERVOS; i++)
  {
    // If we find it return the pointer to it.
    if ( dynamixelServoArray[i] )
    {
      //Serial.print("Servicing Dynamixel ID: ");
      //Serial.println(dynamixelServoArray[i]->GetID());
      dynamixelServoArray[i]->Service();
    }    
  }
}

////////////////////////////////////////////////////////////
// Set time scaling on all Dynamixels.
////////////////////////////////////////////////////////////
void SetDynamixelsTimeScale (int scale)

{
  // Loop through all Dynamixel servos and set the time
  // scaling on them.
  for (int i = 0; i < NUMDYNAMIXELSERVOS; i++)
  {
    // If we find it return the pointer to it.
    if ( dynamixelServoArray[i] )
    {      
      dynamixelServoArray[i]->SetTimeScale(scale);
    }    
  }
}

////////////////////////////////////////////////////////////
// Stop all Dynamixels.
////////////////////////////////////////////////////////////
void StopDynamixels (void)

{

  PrintString("Stopping all Dynamixels: ", 1);
  // Loop through all Dynamixel servos looking for those
  // that need servicing.
  for (int i = 0; i < NUMDYNAMIXELSERVOS; i++)
  {
    // If we find it return the pointer to it.
    if ( dynamixelServoArray[i] )
    {
      // And stop it.      
      dynamixelServoArray[i]->Stop();           
      //PrintString("Stopping Dynamixel: ", 0);
      //PrintString(dynamixelServoArray[i]->GetID(), 1);           
    }    
  }
}

////////////////////////////////////////////////////////////
// Return true if any Dynamixel is currently moving.
////////////////////////////////////////////////////////////
bool AnyMovingDynamixels (void)
{
  bool isMoving = false;
  
  // Loop through all Dynamixel servos and return true
  // if any are moving.
  for (int i = 0; i < NUMDYNAMIXELSERVOS; i++)
  {
    // If we find it return the pointer to it.
    if ( dynamixelServoArray[i] )
    {
      // Check if it is actually there.
      // And check if it is moving.
      if ( (dynamixelServoArray[i]->Ping() == true) && (dynamixelServoArray[i]->IsMoving() == true) )
      {
        // If any are moving return true.
        //PrintString("Dynamixed ID: ", 0);
        //PrintString(dynamixelServoArray[i]->GetID(), 0);
        //PrintString(" is moving.", 1);
        isMoving = true;        
        // Break on the first moving one. No need to check all.
        break;  
      }
    }    
  }
  return isMoving;
}
////////////////////////////////////////////////////////////
// Print current menu.
// Add new menu items here.
////////////////////////////////////////////////////////////  
void PrintMenu(int menuToPrint)
{
  
  if ( menuToPrint == MENU_MAIN )
  { 
    PrintString("MENU MAIN", 0);
  }
  else if ( menuToPrint == DYNAMIXEL_SELECT )
  { 
    PrintString("DYNAMIXEL SELECT", 0);    
  }
  else if ( menuToPrint == SETUP )
  { 
    PrintString("SETUP", 0);
  }
  else if ( menuToPrint == SEQUENCE1 )
  { 
    PrintString("SEQUENCE 1", 0);
  }
  else if ( menuToPrint == SEQUENCE2 )
  { 
    PrintString("SEQUENCE 2", 0);
  }
  else if ( menuToPrint == SEQUENCE3 )
  { 
    PrintString("SEQUENCE 3", 0);
  }
  else if ( menuToPrint == RCCONTROL )
  { 
    PrintString("RC CONTROL", 0);
  }
  else if ( menuToPrint == RESET )
  { 
    PrintString("RESET", 0);
  }
  else if ( menuToPrint == FLACCID )
  { 
    PrintString("FLACCID MODE", 0);
  }
  else if ( menuToPrint == PRINTVARIABLES )
  { 
    PrintString("PRINT VARIABLES", 0);
  }
  
}

////////////////////////////////////////////////////////////
// Print current variable.
////////////////////////////////////////////////////////////  
void PrintVariable(int variableToPrint)
{
  if ( variableToPrint == TIME_SCALE )
  { 
    PrintString("TIME_SCALE % - ", 0);   
    PrintValue(time_scale, 1);
  }
  else if ( variableToPrint == PAUSE_1 )
  { 
    PrintString("PAUSE_1 - ", 0);
    PrintValue(pause_1, 1);
  }
  else if ( variableToPrint == PAUSE_2 )
  { 
    PrintString("PAUSE_2 - ", 0);
    PrintValue(pause_2, 1);
  }
  else if ( variableToPrint == PAUSE_3 )
  { 
    PrintString("PAUSE_3 - ", 0);
    PrintValue(pause_3, 1);
  }
  else if ( variableToPrint == BLIP_1 )
  { 
    PrintString("BLIP_1 - ", 0);
    PrintValue(blip_1, 1);
  }
  else if ( variableToPrint == SLOW_SPEED)
  { 
    PrintString("SLOW_SPEED - ", 0);
    PrintValue(slow_speed_factor, 1);
  }
  else if ( variableToPrint == ITERATIONS )
  { 
    PrintString("ITERATIONS - ", 0);
    PrintValue(iterations, 1);
  }
}

////////////////////////////////////////////////////////////
// Print current Dynamixel variable.
////////////////////////////////////////////////////////////  
void PrintDynamixelVariable(int variableToPrint, DynamixelServo *dynamixel)
{

  String text;

  if ( variableToPrint == CW_SPEED )
  { 
    text = dynamixel->GetCWString();
    PrintString(text, 0);
    PrintString(" SPEED - ", 0);
    PrintValue(dynamixel->GetCWSpeed(), 1);
  }
  if ( variableToPrint == CCW_SPEED )
  { 
    text = dynamixel->GetCCWString();
    PrintString(text, 0);
    PrintString(" SPEED - ", 0);
    PrintValue(dynamixel->GetCCWSpeed(), 1);
  }
  else if ( variableToPrint == ACDC )
  { 
    PrintString("ACDC - ", 0);
    PrintValue(dynamixel->GetACDC(), 1);
  }
  else if ( variableToPrint == CW_LIMIT )
  { 
    text = dynamixel->GetCWString();
    PrintString(text, 0);
    PrintString(" LIMIT - ", 0);
    PrintValue(dynamixel->GetCWLimit(), 1);
  }
  else if ( variableToPrint == CCW_LIMIT )
  { 
    text = dynamixel->GetCCWString();
    PrintString(text, 0);
    PrintString(" LIMIT - ", 0);
    PrintValue(dynamixel->GetCCWLimit(), 1);
  }
  else if ( variableToPrint == MOVE_CW_LIMIT )
  { 
    text = dynamixel->GetCWString();
    PrintString("MOVE ", 0);
    PrintString(text, 0);
    PrintString(" LIMIT", 1);
  }
  else if ( variableToPrint == MOVE_CCW_LIMIT )
  { 
    text = dynamixel->GetCCWString();
    PrintString("MOVE ", 0);
    PrintString(text, 0);
    PrintString(" LIMIT", 1);
  }
  else if ( variableToPrint == MANUAL_CW_LIMIT )
  { 
    text = dynamixel->GetCWString();
    PrintString("HAND MOVE ", 0);
    PrintString(text, 0);
    PrintString(" LIMIT", 1);
  }
  else if ( variableToPrint == MANUAL_CCW_LIMIT )
  { 
    text = dynamixel->GetCCWString();
    PrintString("HAND MOVE ", 0);
    PrintString(text, 0);
    PrintString(" LIMIT", 1);
  }
  else if ( variableToPrint == SET_HOLDING )
  { 
    PrintString("SET_HOLDING", 1);
  }
  else if ( variableToPrint == SET_ENABLED )
  { 
    PrintString("SET_ENABLED", 1);
  }
  else if ( variableToPrint == TEST_SERVO_LIMITS )
  { 
    PrintString("TEST SERVO LIMITS", 1);
  }
  else if ( variableToPrint == TEST_SERVO_WHEEL )
  { 
    PrintString("TEST SERVO WHEEL", 1);
  }
}

////////////////////////////////////////////////////////////
// Read the encoder.
// Pass in the initial value to which we add/subtract.
// Pass in upper and lower limits for wrapping.
// The keypad buttons control the amount to increment.
// 1x with no buttons.
// 2x with button 1.
// 10x with button 2.
// 100X with both buttons.
// Wrap specifies if the value should wrap at minimum and
// maximum values. If not it remains at min/max values.
// Return selected value when the button is pressed.
////////////////////////////////////////////////////////////  
int ReadEncoder(int *buttonPressed, int initialValue, int lowerLimit, int upperLimit, int wrap)
{
  // New value to be returned.
  int value = initialValue;  

  // Modified incremtent.
  int increment = 1;

  // If both pressed
  if (Button1Pressed() && Button2Pressed())
  {
    increment = 100;
  }
  // If button 1 pressed
  else  if (Button1Pressed() && !Button2Pressed())
  {
    increment = 2;
  }
  // If button 2 pressed.
  else  if (!Button1Pressed() && Button2Pressed())
  {
    increment = 10;
  }
  // No buttons.
  else
  {
    increment = 1;
  }
  
  // Read the value.
  encoderPosition = rotaryEncoder.read();

  // If the encoder has changed position update the value.
  if ( (encoderPosition != oldEncoderPosition) && ( (abs(encoderPosition - oldEncoderPosition) % 4) == 0) )  
  {
    // If it went up increment the value.
    if ( encoderPosition > oldEncoderPosition )
    {
       value = value + increment;
       
       // Check the bounds.
       if ( value > upperLimit )
       {
          // Handle the wrapping.
          if (wrap) 
          {
            value = lowerLimit;
          }
          else
          {
            value = upperLimit;
          }
       }
    }
    // If it went down decrement the value.
    if ( encoderPosition < oldEncoderPosition )
    {
       value = value - increment;
       // Check the bounds.
       if ( value < lowerLimit )
       {
          // Handle the wrapping.
          if (wrap)
          {
            value = upperLimit;
          }
          else
          {
            value = lowerLimit;
          }          
       }
    } 

    // Remember the old position.
    oldEncoderPosition = encoderPosition;           
  }

  // Get the button state.
  *buttonPressed = digitalRead(encoderSwitch);

  // Return the value.
  return value;
}

////////////////////////////////////////////////////////////
// Called to service any button presses that might happen.
////////////////////////////////////////////////////////////  
void ServiceButtons()
{

  // Check the exit button.
  if (ExitButtonJustPressed())
  {                
    if ( menu == DYNAMIXEL_SELECT )
    {
      if ( (selected_dynamixel == true) && (editing_dynamixel_variable == true) )
      {
        menu = DYNAMIXEL_SELECT; 
        editing_variable = false;
        selected_dynamixel = true;      
        editing_dynamixel_variable = false;           
      }
      else if ( (selected_dynamixel == true) && (editing_dynamixel_variable == false) )
      {
        menu = DYNAMIXEL_SELECT;
        editing_variable = false;
        selected_dynamixel = true;            
        editing_dynamixel_variable = false;              
      }
      else
      {
        menu = MENU_MAIN; 
        editing_variable = false; 
        selected_dynamixel = false;            
        editing_dynamixel_variable = false;  
      }
    } 
    else
    {
      menu = MENU_MAIN;  
      editing_variable = false;
      selected_dynamixel = false;      
      editing_dynamixel_variable = false;   
    }
  }  
    
  // Check for button presses.
  if ( menu != SEQUENCE3)
  {
    if ( EB1JustPressed() )
    {
       PrintString("Extra button 1 pressed.", 1);      
       PrintString("Setting sequence 1", 1);
        // Stop all servos.
      StopDynamixels();
      // Go back to the base menu.
      menu = SEQUENCE1;   
      PrintString("-----|", 0); 
      PrintMenu(menu);
      PrintString("|-----", 1);
      // Update the menu.
      editing_variable = false;
      selected_dynamixel = false;      
      editing_dynamixel_variable = false;
      // Stop any sequencing.
      sequenceStep = 0;
    }
  
    if ( EB2JustPressed() )
    {
       PrintString("Extra button 2 pressed.", 1);      
       PrintString("Setting sequence 2", 1);
        // Stop all servos.
      StopDynamixels();
      // Go back to the base menu.
      menu = SEQUENCE2;   
      PrintString("-----|", 0); 
      PrintMenu(menu);
      PrintString("|-----", 1);
      // Update the menu.
      oldmenu = menu;    
      editing_variable = false;
      selected_dynamixel = false;      
      editing_dynamixel_variable = false;
      // Stop any sequencing.
      sequenceStep = 0;
    }
    
    /*
    if ( EB3JustPressed() )
    {
      PrintString("Extra button 3 pressed.", 1);      
      PrintString("PANIC!!!", 1);
      LightLED(3, true);    
       
        // Stop all servos.
      StopDynamixels();
      // Go back to the base menu.
      menu = MENU_MAIN;   
      PrintString("-----|", 0); 
      PrintMenu(menu);
      PrintString("|-----", 1);
      // Update the menu.
      oldmenu = menu;    
      selected_dynamixel = false;
      editing_dynamixel_variable = false;
      // Stop any sequencing.
      sequenceStep = 0;
    }
    */
         
    if ( EB4JustPressed() || EB5JustPressed())
    {          
       PrintString("Setting Flaccid mode: ON", 1);
       // Set the holding current off for all Dynamixels.
       SetAllDynamixelHolding(0);            
    }
  }

  // Check the panic switch. 
 if ( PanicButtonJustPressed() ) 
 {    
    // Stop all servos.
    StopDynamixels();
    // Go back to the base menu.
    menu = MENU_MAIN;   
    PrintString("-----|", 0); 
    PrintMenu(menu);
    PrintString("|-----", 1);
    // Update the menu.
    oldmenu = menu;    
    selected_dynamixel = false;
    editing_dynamixel_variable = false;
    // Stop any sequencing.
    sequenceStep = 0;
  }  
  
}

////////////////////////////////////////////////////////////
// Called to check if the exit button was pressed.
// Return true if it was.
////////////////////////////////////////////////////////////  
bool CheckExitButton()
{

  // Check the exit button.
  if (ExitButtonJustPressed())
  {                
    // Go back one MENU.
    if ( menu == SETUP )
    {
      menu = MENU_MAIN;  
      editing_variable = false;
      selected_dynamixel = false;      
      editing_dynamixel_variable = false;   
    }
    else if ( menu == DYNAMIXEL_SELECT )
    {
      if ( (selected_dynamixel == true) && (editing_dynamixel_variable == true) )
      {
        menu = DYNAMIXEL_SELECT; 
        editing_variable = false;
        selected_dynamixel = true;      
        editing_dynamixel_variable = false;   
      }
      else if ( (selected_dynamixel == true) && (editing_dynamixel_variable == false) )
      {
        menu = DYNAMIXEL_SELECT;
        editing_variable = false;
        selected_dynamixel = false;            
        editing_dynamixel_variable = false;       
      }
      else
      {
        menu = MENU_MAIN; 
        editing_variable = false; 
        selected_dynamixel = false;            
        editing_dynamixel_variable = false;  
      }
    } 
    else if ( menu == RCCONTROL )
    {

        PrintString("Detaching RC interrupt.", 1); 
        
        // Detach the interrupt pin.
        detachInterrupt(rcpin);       

        // RC is now disabled.
        RCEnabled = false;

        // Reset speeds back to proper values.
        SetAllDynamixelsToNormalSpeed(); 

        // Return to the main menu.
        menu = MENU_MAIN; 
        editing_variable = false; 
        selected_dynamixel = false;            
        editing_dynamixel_variable = false; 
    }
    else
    {
      menu = MENU_MAIN;  
      editing_variable = false;
      selected_dynamixel = false;      
      editing_dynamixel_variable = false;   
    }
    
    PrintString("-----|", 0); 
    PrintMenu(menu);   
    PrintString("|-----", 1); 

    // Return true.
    return true;
  }
  else 
  {
    // Return false.
    return false;
  }
}
////////////////////////////////////////////////////////////
// Read the encoder to change menus.
////////////////////////////////////////////////////////////  
int ReadEncoderForMenu(int initialMenu)
{
  int newMenu = initialMenu;
  int lastMenu = -1;
  int buttonState = LOW;
  
  // While the button isn't pressed select the next menu with the
  // rotary encoder.
  while ( buttonState == LOW )
  {
    
    // Read the encoder.
    newMenu = ReadEncoder( &buttonState, newMenu, OPTION_FIRST, MENU_LAST, 1 );    

    // If the setting has changed.
    if ( newMenu != lastMenu )
    {
      lastMenu = newMenu;
      PrintString("Select: ", 0);
      PrintMenu(newMenu);
      PrintString("", 1);      
    }
    
    // Check for any special button presses.
    ServiceButtons();   
    if (menu != MENU_MAIN) 
    { 
      return menu;
    }
        
  }

  // Debounce delay.
  delay(50);
    
  // Wait till the button comes back up.
  while ( buttonState == HIGH )
  {
    ReadEncoder( &buttonState, 0, 0, 0, 0 );
  }
 
  return newMenu;
}

////////////////////////////////////////////////////////////
// Read the encoder to select a Dynamixel.
////////////////////////////////////////////////////////////  
int ReadEncoderForDynamixel(int index)
{
  
  int numDynamixels = 0;
  int newIndex = index;
  int lastIndex = -1;
  int buttonState = LOW;

  // Loop through all Dynamixel servos to count them.
  for (int i = 0; i < NUMDYNAMIXELSERVOS; i++)
  {
    if ( dynamixelServoArray[i]  )    
    {
      numDynamixels++;        
    }    
  }

  // If none are connected we have none to set.
  if ( numDynamixels == 0 )
  {
    PrintString("No Dynamixels detected.", 1);
    menu = MENU_MAIN;       
    PrintString("-----|", 0);
    PrintMenu(menu); 
    PrintString("|-----", 1); 
    return -1; 
  }

  // While the button isn't pressed select the next Dynamixel with the
  // rotary encoder.
  while ( buttonState == LOW )
  {
    // Check exit button.
    if ( CheckExitButton() )
    {
      return -1;
    }
    
    // Check for any special button presses.
    ServiceButtons();    
    if ( menu != DYNAMIXEL_SELECT ) 
    {
      return -1; 
    }
    
    // Read the encoder.
    newIndex = ReadEncoder( &buttonState, newIndex, 0, numDynamixels - 1, 1 );    

    // If the setting has changed.
    if ( newIndex != lastIndex )
    {
      lastIndex = newIndex;
      PrintString("Select: ", 0);
      if ( !(dynamixelServoArray[newIndex]->Ping()) )
      {
        PrintString( "(NC) ", 0 );    
      }
      else
      {
        PrintString( dynamixelServoArray[newIndex]->GetID(), 0 );    
        PrintString( " ", 0 );    
      }
      PrintString( dynamixelServoArray[newIndex]->GetName(), 1 );    
    }
  }

  // Debounce delay.
  delay(50);
    
  // Wait till the button comes back up.
  while ( buttonState == HIGH )
  {
    ReadEncoder( &buttonState, 0, 0, 0, 0 );
  }
 
  return newIndex;
}

////////////////////////////////////////////////////////////
// Read the encoder to change variables.
////////////////////////////////////////////////////////////  
int ReadEncoderForVariableToEdit(int variable)
{  

  int newVariable = variable;
  int lastVariable = -1;
  int buttonState = LOW;
  
  // While the button isn't pressed select the next variable with the
  // rotary encoder.
  while ( buttonState == LOW )
  {
    // Check exit button.
    if ( CheckExitButton() )
    {
      return -1;
    }
    
    // Check for any special button presses.
    ServiceButtons();    
    // Check if the exit button was pressed to go back to the main menu.
    if ( menu == MENU_MAIN) return -1; 
      
    // Read the encoder.
    newVariable = ReadEncoder( &buttonState, newVariable, FIRST_VARIABLE, LAST_VARIABLE, 1 );    

    // If the variable has changed.
    if ( newVariable != lastVariable )
    {
      lastVariable = newVariable;
      PrintString("Edit: ", 0);
      PrintVariable(newVariable);    
    }     
  }
  
  // Debounce delay.
  delay(50);
  
  // Wait till the button comes back up.
  while ( buttonState == HIGH )
  {
    ReadEncoder( &buttonState, 0, 0, 0, 0);
  }

  // Debounce delay.
  delay(50);
  
  return newVariable; 
}

////////////////////////////////////////////////////////////
// Read the encoder to change the given variables value.
////////////////////////////////////////////////////////////  
int ReadEncoderForVariableValue(int value,  int lowerLimit, int upperLimit)
{  

  int newValue = value;
  int lastValue = -1;
  int buttonState = LOW;
  
  // While the enter or exit button isn't pressed select the next variable with the
  // rotary encoder.
  while ( 1 )
  {
    // Check exit button.
    if ( CheckExitButton() )
    {
      return -1;
    }
    
    // Check for any special button presses.
    ServiceButtons();        
    
    // Check the enter button.
    if (EnterButtonJustPressed())
    {
      // We break out and save the value.
      break;
    }
   
    // Read the encoder.
    newValue = ReadEncoder( &buttonState, newValue, lowerLimit, upperLimit, 1 );    

    // If the value has changed.
    if ( newValue != lastValue )
    {
      lastValue = newValue;
      PrintString("New value: ", 0);
      PrintValue(newValue, 1);    
    }     
  } 
  return newValue; 
}

////////////////////////////////////////////////////////////
// Read the encoder to change Dynamixel variables.
////////////////////////////////////////////////////////////  
int ReadEncoderForDynamixelVariableToEdit(int variable, DynamixelServo *dynamixel)
{  

  int newVariable = variable;
  int lastVariable = -1;
  int buttonState = LOW;
  
  // While the button isn't pressed select the next variable with the
  // rotary encoder.
  while ( buttonState == LOW )
  {
    // Check exit button.
    if ( CheckExitButton() )
    {
      return -1;
    }
    
    // Check for any special button presses.
    ServiceButtons();    
          
    // Read the encoder.
    newVariable = ReadEncoder( &buttonState, newVariable, FIRST_DYNAMIXEL_VARIABLE, LAST_DYNAMIXEL_VARIABLE, 1 );    

    // If the variable has changed.
    if ( newVariable != lastVariable )
    {
      lastVariable = newVariable;
      PrintString("Value: ", 0);
      PrintDynamixelVariable(newVariable, dynamixel);    
    }     
  }
  
  // Debounce delay.
  delay(50);
  
  // Wait till the button comes back up.
  while ( buttonState == HIGH )
  {
    ReadEncoder( &buttonState, 0, 0, 0, 0);
  }

  // Debounce delay.
  delay(50);
  
  return newVariable; 
}

////////////////////////////////////////////////////////////
// Edit the given variable.
////////////////////////////////////////////////////////////  
void EditVariable(String name, int *value, int lowerLimit, int upperLimit)
{
  // Pass in the old value to the encoder.
  int newValue = ReadEncoderForVariableValue(*value, lowerLimit, upperLimit);

  // If the value that comes back is valid and different store it.
  if ( (newValue >= 0) && (newValue != *value) )
  {
    // Store the new value.
    *value = newValue;
    
    // Print it.
    PrintString("Selected: ", 0);
    PrintValue(newValue, 1);                                                                                                                                                                                                                                                                                                                                                                                                                  
  }  
  editing_variable = false;
  editing_dynamixel_variable = false;             
}

////////////////////////////////////////////////////////////
// Test Dynamixel in WHEEL mode.
////////////////////////////////////////////////////////////  
void TestDynamixelWheel(DynamixelServo *dynamixel)
{
  if ( dynamixel->GetMode() != WHEEL )
  {
    PrintString("Not in WHEEL mode.", 1);
    // Go back one MENU.
    selected_dynamixel = true;
    editing_dynamixel_variable = false;
    menu = DYNAMIXEL_SELECT;
    PrintString("-----|", 0); 
    PrintMenu(menu);    
    PrintString("|-----", 1);
    return;
  }

  // Check the push button states.  
  if (Button1JustPressed() && (!Button2Pressed()))
  {
    PrintString("Button 1 pressed.", 1);
    PrintString("Moving RIGHT.", 1);  
    dynamixel->SetDirection(DRIGHT);
    dynamixel->Wheel(100000);                 
  }  
  else if ((!Button1Pressed()) && Button2JustPressed())
  {
    PrintString("Button 2 pressed.", 1);
    PrintString("Moving LEFT.", 1);  
    dynamixel->SetDirection(DLEFT);
    dynamixel->Wheel(100000);                
  }

  if  ((Button1Released()) || Button2Released())
  {
    PrintString("Stopping Dynamixel.", 1);
    dynamixel->Stop();
  }

  // Check exit button.
  CheckExitButton();      
}

////////////////////////////////////////////////////////////
// Test Dynamixel in SERVO mode.
////////////////////////////////////////////////////////////  
void TestDynamixelServo(DynamixelServo *dynamixel)
{

  int position = 0;
  
  if ( dynamixel->GetMode() != SERVO )
  {
    PrintString("Not in SERVO mode.", 1);
    // Go back one MENU.
    selected_dynamixel = true;
    editing_dynamixel_variable = false;
    menu = DYNAMIXEL_SELECT;
    PrintString("-----|", 0);
    PrintMenu(menu);  
    PrintString("|-----", 1);  
    return;
  }

  // Get the position.
  position = dynamixel->GetPosition();
    
  // Check the push button states.  
  if (Button1Pressed())
  {
    PrintString("Button 1 pressed.", 1);
    PrintString("CW limit: ", 0);
    PrintString(dynamixel->GetCWLimit(), 1);
    dynamixel->Servo(dynamixel->GetCWLimit());                 
    goalPosition = dynamixel->GetCWLimit();    
  }
  
  // Check the push button states.  
  if (Button2Pressed())  
  {
    PrintString("Button 2 pressed.", 1);
    PrintString("CCW limit: ", 0);
    PrintString(dynamixel->GetCCWLimit(), 1);        
    dynamixel->Servo(dynamixel->GetCCWLimit()); 
    goalPosition = dynamixel->GetCCWLimit();
  }

  /* 
  // If we aren't in position and the position has changed from the last time we printed it
  // we output the position.  
  if ( oldValue != position )  
  {
    if ( position != goalPosition )
    {    
      Display.txt_FGcolour(RED); 
      PrintString("Position: ", 0);
      PrintString(position, 0);          
      Display.txt_FGcolour(LIGHTGREEN);
      PrintString(" Goal: ", 0);
      PrintString(goalPosition, 1);
    }
    else
    {       
      PrintString("In position: ", 0);
      PrintString(oldValue, 1);             
    }
  } 
  */
  
  // Update the old position.
  oldValue = position;
    
  // Check exit button.
  CheckExitButton();       
}

////////////////////////////////////////////////////////////
// Adjust Dynamixal CW positional limit.
// Allows the limits to be adjusted visually by actually
// moving the servo.
////////////////////////////////////////////////////////////  
void AdjustDynamixelCWLimit(DynamixelServo *dynamixel)
{
  
  int new_cw = 0; 
  int actualPosition = 0; 

  while (1)
  {  
    new_cw = ReadEncoder( &buttonState, dynamixel->GetCWLimit(), CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE, 0 );
    
    if (new_cw != dynamixel->GetCWLimit())
    {
      actualPosition = dynamixel->GetPosition();
      PrintString("Set: ", 0); PrintValue(new_cw, 0);
      PrintString("  Current: ", 0); PrintValue(actualPosition, 1);    
    }
    dynamixel->SetCWLimit(new_cw);
    dynamixel->Servo(new_cw);                                 
    
    
    // Check if the enter button was just pressed.
    if (EnterButtonJustPressed())  
    {     
      UpdateDynamixelCWLimitVariable(dynamixelIndex, new_cw);
  
      // Store the values.
      WriteEEprom();
          
      // Go back one MENU.
      selected_dynamixel = true;
      editing_dynamixel_variable = false;
      menu = DYNAMIXEL_SELECT; 
      PrintString("-----|", 0);
      PrintMenu(menu);
      PrintString("|-----", 1);    
      break;
    } 
    
    // Check if the exit button was just pressed.
    if (CheckExitButton())  
    {    
        // If the exit button was pressed return without
        // updating the limits.  
        PrintString("Exit button pressed." , 1); 
        PrintString("Not updating limits.", 1);   
        break;      
    }
  }  
}

////////////////////////////////////////////////////////////
// Adjust Dynamixal CCW positional limit.
// Allows the limits to be adjusted visually by actually
// moving the servo.
////////////////////////////////////////////////////////////  
void AdjustDynamixelCCWLimit(DynamixelServo *dynamixel)
{
 
  int new_ccw = 0;  
  int actualPosition = 0;

  // Do this in a tight loop to get the speed we need.
  while (1)  
  {
    new_ccw = ReadEncoder( &buttonState, dynamixel->GetCCWLimit(), CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE, 0 );
    
    if (new_ccw != dynamixel->GetCCWLimit())
    {
      actualPosition = dynamixel->GetPosition();
      PrintString("Set: ", 0); PrintValue(new_ccw, 0);
      PrintString("  Current: ", 0); PrintValue(actualPosition, 1);
    }
    dynamixel->SetCCWLimit(new_ccw);
    dynamixel->Servo(new_ccw);                                 
  
    // Check if the enter button was just pressed.
    if (EnterButtonJustPressed())
    {   
      UpdateDynamixelCCWLimitVariable(dynamixelIndex, new_ccw);
      
      // Store the values.
      WriteEEprom();
          
      // Go back one MENU.
      selected_dynamixel = true;
      editing_dynamixel_variable = false;
      menu = DYNAMIXEL_SELECT; 
      PrintString("-----|", 0);
      PrintMenu(menu);
      PrintString("|-----", 1);
      break;    
    } 
  
    // Check if the exit button was just pressed.
    if (CheckExitButton())  
    {    
        // If the exit button was pressed return without
        // updating the limits.  
        PrintString("Exit button pressed." , 1); 
        PrintString("Not updating limits.", 1);         
        break;
    }
  }
}

////////////////////////////////////////////////////////////
// Manually Adjust Dynamixal CW positional limit.
// Allows the limits to be adjusted manually by actually
// moving the servo.
////////////////////////////////////////////////////////////  
void ManuallyAdjustDynamixelCWLimit(DynamixelServo *dynamixel)
{
  
  bool writeValue = true;
  
  // Positions.
  int currentPosition = 0;
  int lastPosition = 0;
  
  // Turn off holding torque so we can move the servos.
  dynamixel->SetHolding(false);

  // Now update the position until the enter button is pressed.
  while (!(EnterButtonJustPressed()))
  {
      // Get the current position.
      currentPosition = dynamixel->GetPosition();

      // If different print it out.
      if (currentPosition != lastPosition)
      {
        PrintValue(currentPosition, 1);
      }

      // Update the position.
      lastPosition = currentPosition;    
       
      // Check if the exit button was just pressed.
      if (ExitButtonJustPressed())  
      {    
          // If the exit button was pressed return without
          // updating the limits.
          PrintString("Exit button pressed." , 1); 
          PrintString("Not updating limits.", 1);
          writeValue = false;   
          break;    
      }  
  }

  // Enter button was pressed so save the new limit and reset the 
  // holding torque.
  
  if ( writeValue )
  {
    PrintString("Setting CW limit to: ", 0);
    PrintValue(currentPosition, 1);
    dynamixel->SetCWLimit(currentPosition);    
    UpdateDynamixelCWLimitVariable(dynamixelIndex, currentPosition);
  
    // Store the values.
    WriteEEprom();
  }

  dynamixel->SetHolding(true);
        
  // Go back one MENU.
  selected_dynamixel = true;
  editing_dynamixel_variable = false;
  menu = DYNAMIXEL_SELECT; 
  PrintString("-----|", 0);
  PrintMenu(menu);
  PrintString("|-----", 1);    
}

////////////////////////////////////////////////////////////
// Manual Adjust Dynamixal CCW positional limit.
// Allows the limits to be adjusted manually by actually
// moving the servo.
////////////////////////////////////////////////////////////  
void ManuallyAdjustDynamixelCCWLimit(DynamixelServo *dynamixel)
{

  bool writeValue = true;
  
  // Positions.
  int currentPosition = 0;
  int lastPosition = 0;
  
  // Turn off holding torque so we can move the servos.
  dynamixel->SetHolding(false);

  // Now update the position until the enter button is pressed.
  while (!(EnterButtonJustPressed()))
  {
      // Get the current position.
      currentPosition = dynamixel->GetPosition();

      // If different print it out.
      if (currentPosition != lastPosition)
      {
        PrintValue(currentPosition, 1);
      }

      // Update the position.
      lastPosition = currentPosition;    
       
      // Check if the exit button was just pressed.
      if (ExitButtonJustPressed())  
      {    
          // If the exit button was pressed return without
          // updating the limits.  
          PrintString("Exit button pressed." , 1); 
          PrintString("Not updating limits.", 1);
          writeValue = false;      
          break;    
      }  
  }

  if ( writeValue )
  {
    // Enter button was pressed so save the new limit and reset the 
    // holding torque.
    PrintString("Setting CCW limit to: ", 0);
    PrintValue(currentPosition, 1);
    dynamixel->SetCCWLimit(currentPosition);
    UpdateDynamixelCCWLimitVariable(dynamixelIndex, currentPosition);
  
    // Store the values.
    WriteEEprom();
  }

  dynamixel->SetHolding(true);
  
  // Go back one MENU.
  selected_dynamixel = true;
  editing_dynamixel_variable = false;
  menu = DYNAMIXEL_SELECT; 
  PrintString("-----|", 0);
  PrintMenu(menu);
  PrintString("|-----", 1);    
}


////////////////////////////////////////////////////////////
// Allow setting or unsetting of the holding torque on
// all Dynamixels.
////////////////////////////////////////////////////////////  
void SetAllDynamixelHolding(bool holding)
{
  // Loop through all Dynamixel servos and set the holding 
  // torque flag on them.
  for (int i = 0; i < NUMDYNAMIXELSERVOS; i++)
  {
    // If we find it return the pointer to it.
    if ( dynamixelServoArray[i] )
    {
      // And set the holding torque on it.
      dynamixelServoArray[i]->SetHolding(holding);               
    }    
  }  
}

////////////////////////////////////////////////////////////
// Allow enabling or disabling of all Dynamixels.
////////////////////////////////////////////////////////////  
void SetAllDynamixelEnabled(bool enabled)
{
  // Loop through all Dynamixel servos and set the enabled 
  // flag on them.
  for (int i = 0; i < NUMDYNAMIXELSERVOS; i++)
  {
    // If we find it return the pointer to it.
    if ( dynamixelServoArray[i] )
    {
      // And set the enabled flag on them.
      dynamixelServoArray[i]->Enable(enabled);               
    }    
  }  
}

////////////////////////////////////////////////////////////
// Callback for LCD screen.
////////////////////////////////////////////////////////////  
void screenErrorCallback(int ErrCode, unsigned char Errorbyte)
{
  Serial.print("Error code: ");
  Serial.println(ErrCode);
  Serial.print("Error byte: ");
  Serial.println(Errorbyte);
}

////////////////////////////////////////////////////////////
// Update the given Dynamixels CW speed value.
////////////////////////////////////////////////////////////  
void UpdateDynamixelCWSpeedVariable(int index, int speed)
{
  Serial.print("Index: "); Serial.println( index );
  
  switch (index)
  {
    case 0: d1_cw_speed = speed; break;      
    case 1: d2_cw_speed = speed; break;      
    case 2: d3_cw_speed = speed; break;      
    case 3: d4_cw_speed = speed; break;      
    case 4: d5_cw_speed = speed; break;
    case 5: d6_cw_speed = speed; break;        
  }  
}

////////////////////////////////////////////////////////////
// Update the given Dynamixels CCW speed value.
////////////////////////////////////////////////////////////  
void UpdateDynamixelCCWSpeedVariable(int index, int speed)
{
  Serial.print("Index: "); Serial.println( index );

  switch (index)
  {
    case 0: d1_ccw_speed = speed; break;      
    case 1: d2_ccw_speed = speed; break;      
    case 2: d3_ccw_speed = speed; break;      
    case 3: d4_ccw_speed = speed; break;      
    case 4: d5_ccw_speed = speed; break;
    case 5: d6_ccw_speed = speed; break;        
  }  
}

////////////////////////////////////////////////////////////
// Update the given Dynamixels acdc value.
////////////////////////////////////////////////////////////  
void UpdateDynamixelACDCVariable(int index, int acdc)
{
  Serial.print("Index: "); Serial.println( index );
  
  switch (index)
  {
    case 0: d1_acdc = acdc; break;      
    case 1: d2_acdc = acdc; break;
    case 2: d3_acdc = acdc; break;      
    case 3: d4_acdc = acdc; break;      
    case 4: d5_acdc = acdc; break;      
    case 5: d6_acdc = acdc; break;                
  }  
}

////////////////////////////////////////////////////////////
// Update the given Dynamixels CW limit value.
////////////////////////////////////////////////////////////  
void UpdateDynamixelCWLimitVariable(int index, int cw_limit)
{
  Serial.print("Index: "); Serial.println( index );
  
  switch (index)
  {
    case 0: d1_cw_limit = cw_limit; break;      
    case 1: d2_cw_limit = cw_limit; break;      
    case 2: d3_cw_limit = cw_limit; break;      
    case 3: d4_cw_limit = cw_limit; break;      
    case 4: d5_cw_limit = cw_limit; break;      
    case 5: d6_cw_limit = cw_limit; break;           
  }  
}

////////////////////////////////////////////////////////////
// Update the given Dynamixels CCW limit value.
////////////////////////////////////////////////////////////  
void UpdateDynamixelCCWLimitVariable(int index, int ccw_limit)
{
  Serial.print("Index: "); Serial.println( index );
  
  switch (index)
  {
    case 0: d1_ccw_limit = ccw_limit; break;      
    case 1: d2_ccw_limit = ccw_limit; break;      
    case 2: d3_ccw_limit = ccw_limit; break;      
    case 3: d4_ccw_limit = ccw_limit; break;      
    case 4: d5_ccw_limit = ccw_limit; break;      
    case 5: d6_ccw_limit = ccw_limit; break;           
  }  
}

////////////////////////////////////////////////////////////
// Flaccid mode.
////////////////////////////////////////////////////////////  
void FlaccidMode()
{
  // Get the new value.
  int variable = 1;
  EditVariable("Set holding", &variable, 0, 1);
  dynamixel->SetHolding(variable);
  if ( variable )
  {
    PrintString("Holding torque ON for all.", 1);
  }
  else
  {
    PrintString("Holding torque OFF for all.", 1);
  } 
  // Set the appropriate value on all Dynamixels.
  SetAllDynamixelHolding(variable);            
}

////////////////////////////////////////////////////////////
// Check the push buttons.
////////////////////////////////////////////////////////////  
void CheckButtons()
{
  static byte previousstate[NUMBUTTONS];
  static byte currentstate[NUMBUTTONS];
  static unsigned long lasttime;  

  // Handle wrap around.
  if ( millis() < lasttime )
  {
     lasttime = millis();  
  }

  // Check if enough time has passed.
  if ( (lasttime + DEBOUNCE) > millis() ) 
  {    
    return; 
  }
  
  // Enough time passed. Reset the timer.
  lasttime = millis();

  // Loop through checking the buttons.
  for (byte index = 0; index < NUMBUTTONS; index++ )
  {
    justreleased[index] = 0;
    justpressed[index] = 0;     
    currentstate[index] = digitalRead(buttons[index]);   // Read the button.

    if (currentstate[index] == previousstate[index]) 
    {
      if ((pressed[index] == LOW) && (currentstate[index] == LOW)) 
      {
          // just pressed
          justpressed[index] = 1;
      }
      else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)) 
      {
          // just released
          justreleased[index] = 1;
      }
      pressed[index] = !currentstate[index];  // remember, digital HIGH means NOT pressed
    }
    previousstate[index] = currentstate[index];   // keep a running tally of the buttons
  }
}

////////////////////////////////////////////////////////////
// Button 1 handling code.
////////////////////////////////////////////////////////////  
bool Button1JustPressed()
{
  CheckButtons();  // Check the buttons.    
  return justpressed[0];
}
bool Button1Pressed()
{
  CheckButtons();  // Check the buttons.  
  return pressed[0];
}
bool Button1Released()
{
  CheckButtons();  // Check the buttons.  
  return justreleased[0];
}
////////////////////////////////////////////////////////////
// Button 2 handling code.
////////////////////////////////////////////////////////////  
bool Button2JustPressed()
{
  CheckButtons();  // Check the buttons.  
  return justpressed[1];
}
bool Button2Pressed()
{
  CheckButtons();  // Check the buttons.  
  return pressed[1];
}
bool Button2Released()
{
  CheckButtons();  // Check the buttons.  
  return justreleased[1];
}
////////////////////////////////////////////////////////////
// Exit button handling code.
////////////////////////////////////////////////////////////  
bool ExitButtonJustPressed()
{
  CheckButtons();  // Check the buttons.
  return justpressed[2];
}
bool ExitButtonPressed()
{
  CheckButtons();  // Check the buttons.  
  return pressed[2];
}
bool ExitButtonReleased()
{
  CheckButtons();  // Check the buttons.  
  return justreleased[2];
}
////////////////////////////////////////////////////////////
// Enter button handling code.
////////////////////////////////////////////////////////////  
bool EnterButtonJustPressed()
{
  CheckButtons();  // Check the buttons.  
  return justpressed[3];
}
bool EnterButtonPressed()
{
  CheckButtons();  // Check the buttons.  
  return pressed[3];
}
bool EnterButtonReleased()
{
  CheckButtons();  // Check the buttons.  
  return justreleased[3];
}
////////////////////////////////////////////////////////////
// Panic button handling code.
////////////////////////////////////////////////////////////  
bool PanicButtonJustPressed()
{
  CheckButtons();  // Check the buttons.  
  return justpressed[4];
}
bool PanicButtonPressed()
{
  CheckButtons();  // Check the buttons.  
  return pressed[4];
}
bool PanicButtonReleased()
{
  CheckButtons();  // Check the buttons.  
  return justreleased[4];
}
////////////////////////////////////////////////////////////
// Extra button 1 handling code.
////////////////////////////////////////////////////////////  
bool EB1JustPressed()
{
  CheckButtons();  // Check the buttons.  
  return justpressed[5];
}
bool EB1Pressed()
{
  CheckButtons();  // Check the buttons. 
  return pressed[5];
}
bool EB1Released()
{
  CheckButtons();  // Check the buttons.  
  return justreleased[5];
}
////////////////////////////////////////////////////////////
// Extra button 2 handling code.
////////////////////////////////////////////////////////////  
bool EB2JustPressed()
{
  CheckButtons();  // Check the buttons.  
  return justpressed[6];
}
bool EB2Pressed()
{
  CheckButtons();  // Check the buttons.  
  return pressed[6];
}
bool EB2Released()
{
  CheckButtons();  // Check the buttons.  
  return justreleased[6];
}
////////////////////////////////////////////////////////////
// Extra button 3 handling code.
////////////////////////////////////////////////////////////  
bool EB3JustPressed()
{
  CheckButtons();  // Check the buttons.  
  return justpressed[7];
}
bool EB3Pressed()
{
  CheckButtons();  // Check the buttons.  
  return pressed[7];
}
bool EB3Released()
{
  CheckButtons();  // Check the buttons.  
  return justreleased[7];
}
////////////////////////////////////////////////////////////
// Extra button 4 handling code.
////////////////////////////////////////////////////////////  
bool EB4JustPressed()
{
  CheckButtons();  // Check the buttons.  
  return justpressed[8];
}
bool EB4Pressed()
{
  CheckButtons();  // Check the buttons.  
  return pressed[8];
}
bool EB4Released()
{
  CheckButtons();  // Check the buttons.  
  return justreleased[8];
}
////////////////////////////////////////////////////////////
// Extra button 5 handling code.
////////////////////////////////////////////////////////////  
bool EB5JustPressed()
{
  CheckButtons();  // Check the buttons.  
  return justpressed[9];
}
bool EB5Pressed()
{
  CheckButtons();  // Check the buttons.  
  return pressed[9];
}
bool EB5Released()
{
  CheckButtons();  // Check the buttons.  
  return justreleased[9];
}

////////////////////////////////////////////////////////////
// Handle receiving an RC command here.
////////////////////////////////////////////////////////////  
void HandleRCCommand()
{
    int dynamixel1position = 0;
    int dynamixel2position = 0;
    int dynamixel3position = 0; 
    int dynamixel4position = 0;
    int dynamixel5position = 0;
    int dynamixel6position = 0;

    // In RC mode since we don't know how open
    // the panels are we just say they are all open.
    eyesOpen = true;
    cheeksOpen = true;
    chinsidesOpen = true;
    chinOpen = true; 

    // If the value has changed.
    //Serial.print(channel);
    //Serial.print("  ");
    //Serial.println(lastValue);
    if ( (channel < lastValue - 2) || (channel > lastValue + 2) )
    {
      // If the stick is forward close at a scaled speed.
      if ( channel < (midRC - rcDeadZone)) 
      {
        lastValue = channel;
        int orig_slow = slow_speed_factor;
        slow_speed_factor = map(channel, minRC, midRC - (rcDeadZone/2), 50, 1);
  
        //Serial.println(slow_speed_factor);
        
        SetAllDynamixelsToSlowSpeed();
        
        dynamixel5.Servo(d5_ccw_limit);  
        dynamixel1.Servo(d1_cw_limit);       
        dynamixel2.Servo(d2_cw_limit);                           
        dynamixel3.Servo(d3_ccw_limit); 
        dynamixel4.Servo(d4_ccw_limit);                                   
        dynamixel6.Servo(d6_cw_limit);
  
        slow_speed_factor = orig_slow;
        SetAllDynamixelsToNormalSpeed();  
       
        eyesOpen = true;
        cheeksOpen = true;
        chinsidesOpen = true;
        chinOpen = true;          
        
      }
      // Else if stick is backwards open at scaled speed.
      else if ( channel > (midRC + rcDeadZone)) 
      {  
        lastValue = channel;
        int orig_slow = slow_speed_factor;
        slow_speed_factor = map(channel, midRC + (rcDeadZone/2), maxRC, 1, 50);
  
        //Serial.println(slow_speed_factor);
        
        SetAllDynamixelsToSlowSpeed();
        
        dynamixel5.Servo(d5_cw_limit);  
        dynamixel1.Servo(d1_ccw_limit);       
        dynamixel2.Servo(d2_ccw_limit);                           
        dynamixel3.Servo(d3_cw_limit); 
        dynamixel4.Servo(d4_cw_limit);                                   
        dynamixel6.Servo(d6_ccw_limit);
  
        slow_speed_factor = orig_slow;
        SetAllDynamixelsToNormalSpeed();  
        eyesOpen = false;
        cheeksOpen = false;
        chinsidesOpen = false;
        chinOpen = false;          
      }
    }
    
    else
    {
      lastValue = channel;

      int orig_slow = slow_speed_factor;
      slow_speed_factor = 0;
      SetAllDynamixelsToSlowSpeed();      
      //dynamixel5.Servo(d5_cw_limit);  
      //dynamixel1.Servo(d1_ccw_limit);       
      //dynamixel2.Servo(d2_ccw_limit);                           
      //dynamixel3.Servo(d3_cw_limit); 
      //dynamixel4.Servo(d4_cw_limit);                                   
      //dynamixel6.Servo(d6_ccw_limit);
      dynamixel5.Servo(dynamixel5.GetPosition());  
      dynamixel1.Servo(dynamixel1.GetPosition());       
      dynamixel2.Servo(dynamixel2.GetPosition());                           
      dynamixel3.Servo(dynamixel3.GetPosition()); 
      dynamixel4.Servo(dynamixel4.GetPosition());                                   
      dynamixel6.Servo(dynamixel6.GetPosition());
      slow_speed_factor = orig_slow;
      SetAllDynamixelsToNormalSpeed();  
    }
   
    
    /*
    // Map the channel signal to a position for each Dynamixel.
    // To reverse direction swap the CW and CCW limits around.
    dynamixel1position = map(channel, minRC, maxRC, d1_ccw_limit, d1_cw_limit); // 21 Left cheek.      
    dynamixel2position = map(channel, minRC, maxRC, d2_ccw_limit, d2_cw_limit); // 22 Left eye.      
    dynamixel3position = map(channel, minRC, maxRC, d3_cw_limit, d3_ccw_limit); // 23 Right cheek.      
    dynamixel4position = map(channel, minRC, maxRC, d4_cw_limit, d4_ccw_limit); // 24 Chin.      
    dynamixel5position = map(channel, minRC, maxRC, d5_cw_limit, d5_ccw_limit); // 25 Right eye.      
    dynamixel6position = map(channel, minRC, maxRC, d6_ccw_limit, d6_cw_limit); // 26 Chin sides.

    //PrintString(channel, 1);
        
    // Move the Dynamixels to the given positions.
    if ( (dynamixel1position > d1_cw_limit) && (dynamixel1position < d1_ccw_limit) )
    {
      dynamixel1.Servo(dynamixel1position);
    }
    if ( (dynamixel2position > d2_cw_limit) && (dynamixel2position < d2_ccw_limit) )
    {
      dynamixel2.Servo(dynamixel2position);
    }
    if ( (dynamixel3position > d3_cw_limit) && (dynamixel3position < d3_ccw_limit) )
    {
      dynamixel3.Servo(dynamixel3position);
    }
    if ( (dynamixel4position > d4_cw_limit) && (dynamixel4position < d4_ccw_limit) )
    {
      dynamixel4.Servo(dynamixel4position);
    }
    if ( (dynamixel5position > d5_cw_limit) && (dynamixel5position < d5_ccw_limit) )
    {
      dynamixel5.Servo(dynamixel5position);
    }
    if ( (dynamixel6position > d6_cw_limit) && (dynamixel6position < d6_ccw_limit) )
    {
      dynamixel6.Servo(dynamixel6position);
    }

    */

    // Update the flag to say we handled it.
    gotRCValue = false;    

      
  
}

////////////////////////////////////////////////////////////
// Set all speeds to the given speed.
////////////////////////////////////////////////////////////  
void SetAllDynamixelsToSpeed(int speed)
{
  //PrintString("Setting speeds to: ", 0);
  //PrintValue(speed, 1);
  dynamixel1.SetCWSpeed(speed);
  dynamixel1.SetCCWSpeed(speed);
  dynamixel2.SetCWSpeed(speed);
  dynamixel2.SetCCWSpeed(speed);
  dynamixel3.SetCWSpeed(speed);
  dynamixel3.SetCCWSpeed(speed);
  dynamixel4.SetCWSpeed(speed);
  dynamixel4.SetCCWSpeed(speed);
  dynamixel5.SetCWSpeed(speed);
  dynamixel5.SetCCWSpeed(speed);
  dynamixel6.SetCWSpeed(speed);
  dynamixel6.SetCCWSpeed(speed);
}

////////////////////////////////////////////////////////////
// Set all Dynamixels to the scaled slow speed value.
////////////////////////////////////////////////////////////  
void SetAllDynamixelsToSlowSpeed()
{
  int speed = 0;
  
  speed = d1_cw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel1.SetCWSpeed(speed);
  speed = d1_ccw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel1.SetCCWSpeed(speed);
  speed = d2_cw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel2.SetCWSpeed(speed);
  speed = d2_ccw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel2.SetCCWSpeed(speed);
  speed = d3_cw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel3.SetCWSpeed(speed);
  speed = d3_ccw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel3.SetCCWSpeed(speed);  
  speed = d4_cw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel4.SetCWSpeed(speed);
  speed = d4_ccw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel4.SetCCWSpeed(speed);
  speed = d5_cw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel5.SetCWSpeed(speed);
  speed = d5_ccw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel5.SetCCWSpeed(speed);
  speed = d6_cw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel6.SetCWSpeed(speed);
  speed = d6_ccw_speed * slow_speed_factor;
  speed = speed / 100;
  dynamixel6.SetCCWSpeed(speed);
  
}

////////////////////////////////////////////////////////////
// Reset all speeds back to normal.
////////////////////////////////////////////////////////////  
void SetAllDynamixelsToNormalSpeed()
{
  // PrintString("Resetting speeds to normal.", 1);
  // Reset speeds back to proper values.
  dynamixel1.SetCWSpeed(d1_cw_speed);
  dynamixel1.SetCCWSpeed(d1_ccw_speed);
  dynamixel2.SetCWSpeed(d2_cw_speed);
  dynamixel2.SetCCWSpeed(d2_ccw_speed);
  dynamixel3.SetCWSpeed(d3_cw_speed);
  dynamixel3.SetCCWSpeed(d3_ccw_speed);
  dynamixel4.SetCWSpeed(d4_cw_speed);
  dynamixel4.SetCCWSpeed(d5_ccw_speed);
  dynamixel5.SetCWSpeed(d5_cw_speed);
  dynamixel5.SetCCWSpeed(d5_ccw_speed);
  dynamixel6.SetCWSpeed(d6_cw_speed);
  dynamixel6.SetCCWSpeed(d6_ccw_speed);
}

////////////////////////////////////////////////////////////
// Main loop.
////////////////////////////////////////////////////////////  
void loop()
{
      
  // Service any Dynamixels that need it.    
  ServiceDynamixels(); 
  
  // Check the currently selected menu and change bahaviour based on it.
  switch (menu)
  {
    
    // In menu select menu we just allow changing the menu.
    case MENU_MAIN:
    {
      LightLED(0, false);
      EncoderGreen();
      
      // Pass in the old menu to the encoder.
      int newMenu = ReadEncoderForMenu(menu);
      
      // If it has changed when it comes back we
      // print it and update the variable.
      if ( newMenu != menu )
      {        
        menu = newMenu;
        PrintString("-----|", 0);
        PrintMenu(menu);
        PrintString("|-----", 1);
      }
      break;
    }

    // In the Dynamixel select menu we can adjust and test Dynamixel settings.
    case DYNAMIXEL_SELECT:
    {
      LightLED(0, false);
      EncoderRed();
      
      // Check for any special button presses.
      ServiceButtons();    
      
      if ( selected_dynamixel == false )
      {
        // Pass in the old variable to the encoder.
        int newIndex = ReadEncoderForDynamixel(dynamixelIndex);
        // If a dynamixel was selected.
        if ( newIndex != -1 )
        {
          dynamixel = dynamixelServoArray[newIndex];
          dynamixelIndex = newIndex;
          selected_dynamixel = true;       
          // Ping it. We say if it is not connected but allow
          // editing of it anyway.
          if ( dynamixel->Ping() == true)
          {
            PrintString("Editing: ", 0);
            PrintString(dynamixel->GetName(), 1);            
          }
          else
          {
            PrintString(dynamixel->GetName(), 0);
            PrintString(" not connected.", 1);                        
          }          
        }       
      }
      // Every variable we need to change goes here.
      if ( selected_dynamixel == true )
      {
        // Selected a servo. Now select the variable to edit on it.        
        if ( editing_dynamixel_variable == false )
        {
          // Pass in the old variable to the encoder.          
          int newVariable = ReadEncoderForDynamixelVariableToEdit(setup_dynamixel_variable, dynamixel);
          // If a variable was selected.
          if ( newVariable != -1 )
          {
            setup_dynamixel_variable = newVariable;
            if (  !(setup_dynamixel_variable == MOVE_CW_LIMIT)
               && !(setup_dynamixel_variable == MOVE_CCW_LIMIT) 
               && !(setup_dynamixel_variable == MANUAL_CCW_LIMIT) 
               && !(setup_dynamixel_variable == MANUAL_CCW_LIMIT)                
               && !(setup_dynamixel_variable == TEST_SERVO_LIMITS) 
               && !(setup_dynamixel_variable == TEST_SERVO_WHEEL))
            PrintString("Editing: ", 0);
            PrintDynamixelVariable(setup_dynamixel_variable, dynamixel);
            editing_dynamixel_variable = true;       
          }
        }
        else //editing_dynamixel_variable == true
        {          
          int variable = 0;          
          String editString;
          
          if (setup_dynamixel_variable == CW_SPEED)
          {
            variable = dynamixel->GetCWSpeed();
            editString = dynamixel->GetCWString();
            EditVariable(editString, &variable, 0, 1023); 
            dynamixel->SetCWSpeed(variable);
            UpdateDynamixelCWSpeedVariable(dynamixelIndex, variable);
            // Store the values.
            WriteEEprom();
          }
          if (setup_dynamixel_variable == CCW_SPEED)
          {
            variable = dynamixel->GetCCWSpeed();
            editString = dynamixel->GetCCWString();
            EditVariable(editString, &variable, 0, 1023); 
            dynamixel->SetCCWSpeed(variable);
            UpdateDynamixelCCWSpeedVariable(dynamixelIndex, variable);
            // Store the values.
            WriteEEprom();
          }
          if (setup_dynamixel_variable == ACDC)
          {
            variable = dynamixel->GetACDC();
            EditVariable("ACDC", &variable, 0, 5000);      
            dynamixel->SetACDC(variable);
            UpdateDynamixelACDCVariable(dynamixelIndex, variable);
            // Store the values.
            WriteEEprom();
          }
          if (setup_dynamixel_variable == CW_LIMIT)
          {
            variable = dynamixel->GetCWLimit();
            editString = dynamixel->GetCWString();
            EditVariable(editString, &variable, CW_LIMIT, CCW_LIMIT_ANGLE);
            dynamixel->SetCWLimit(variable);
            UpdateDynamixelCWLimitVariable(dynamixelIndex, variable);
            // Store the values.
            WriteEEprom();
          }
          if (setup_dynamixel_variable == CCW_LIMIT)
          {
            variable = dynamixel->GetCCWLimit();
            editString = dynamixel->GetCCWString();
            EditVariable(editString, &variable, CW_LIMIT, CCW_LIMIT_ANGLE);
            dynamixel->SetCCWLimit(variable);
            UpdateDynamixelCCWLimitVariable(dynamixelIndex, variable);
            // Store the values.
            WriteEEprom();
          }
          if (setup_dynamixel_variable == MOVE_CW_LIMIT) 
          {
            AdjustDynamixelCWLimit(dynamixel);
          }
          if (setup_dynamixel_variable == MOVE_CCW_LIMIT) 
          {
            AdjustDynamixelCCWLimit(dynamixel);
          }
          if (setup_dynamixel_variable == MANUAL_CW_LIMIT) 
          {
            ManuallyAdjustDynamixelCWLimit(dynamixel);
          }
          if (setup_dynamixel_variable == MANUAL_CCW_LIMIT) 
          {
            ManuallyAdjustDynamixelCCWLimit(dynamixel);
          }
          if (setup_dynamixel_variable == SET_HOLDING)
          {
            variable = 1;
            EditVariable("Set holding", &variable, 0, 1);
            dynamixel->SetHolding(variable);
            if ( variable )
            {
              PrintString("Holding torque ON.", 1);
            }
            else
            {
              PrintString("Holding torque OFF.", 1);
            }            
          }
          if (setup_dynamixel_variable == SET_ENABLED)
          {
            variable = 1;
            EditVariable("Set enabled", &variable, 0, 1);
            dynamixel->Enable(variable);
            if ( variable )
            {
              PrintString("Dynamixel ENABLED.", 1);
            }
            else
            {
              PrintString("Dynamixel DISABLED.", 1);
            }            
          }
          if (setup_dynamixel_variable == TEST_SERVO_LIMITS) 
          {
            TestDynamixelServo(dynamixel);    
          }
          if (setup_dynamixel_variable == TEST_SERVO_WHEEL) 
          {
            TestDynamixelWheel(dynamixel);    
          }        
        }            
      }
      break;
    }

    // In setup menu we cycle through variables so we can change them.
    case SETUP:
    {       
      LightLED(0, false);
      EncoderRed();         

      // Check for any special button presses.
      ServiceButtons();    
      
      // If not editing a variable we need to choose one to edit.
      if ( editing_variable == false )
      {
        // Pass in the old variable to the encoder.
        int newVariable = ReadEncoderForVariableToEdit(setup_variable);
        // If a variable was selected.
        if ( newVariable != -1 )
        {
          setup_variable = newVariable;
          PrintString("Editing: ", 0);
          PrintVariable(setup_variable);
          editing_variable = true;       
        }       
      }
      // Every variable we need to change goes here.
      if ( editing_variable == true )
      {
        if (setup_variable == TIME_SCALE) EditVariable("time_scale", &time_scale, TIME_SCALE_MIN, TIME_SCALE_MAX);                    
        if (setup_variable == PAUSE_1) EditVariable("pause_1", &pause_1, 0, 5000);
        if (setup_variable == PAUSE_2) EditVariable("pause_2", &pause_2, 0, 5000);
        if (setup_variable == PAUSE_3) EditVariable("pause_3", &pause_3, 0, 5000);
        if (setup_variable == BLIP_1) EditVariable("blip_1", &blip_1, 0, 5000);       
        if (setup_variable == SLOW_SPEED) EditVariable("slow_speed_factor", &slow_speed_factor, 0, 10000);       
        if (setup_variable == ITERATIONS) EditVariable("iterations", &iterations, 0, 10000); 
        SetDynamixelsTimeScale(time_scale);
        // Store the values.
        WriteEEprom();      
      }
      break;
    }
    
    
    case SEQUENCE1:
    {       

      LightLED(0, false);
      LightLED(1, true);      
      EncoderPurple();       
     
      // Check for any special button presses.
      ServiceButtons();

      int lastValue = slow_speed_factor;
       // Read the encoder.       
      int newValue = ReadEncoder( &buttonState, lastValue, TIME_SCALE_MIN, TIME_SCALE_MAX, 1 );    

      // If the value has changed.
      if ( newValue != lastValue )
      {
        lastValue = newValue;
        PrintString("slow_speed_factor: ", 0);
        PrintValue(newValue, 1); 
        slow_speed_factor = newValue; 
      }      
  
      // Wait for button press.
      if (Button1JustPressed())
      {
        // Sequence step 1.
        PrintString("Open button is pressed.", 1);     
        dynamixel5.Servo(d5_ccw_limit); 
        dynamixel1.Servo(d1_cw_limit);
        dynamixel2.Servo(d2_cw_limit);                          
        dynamixel3.Servo(d3_ccw_limit);
        dynamixel4.Servo(d4_ccw_limit);                  
        dynamixel6.Servo(d6_cw_limit);     
        eyesOpen = true;
        cheeksOpen = true;
        chinsidesOpen = true;
        chinOpen = true;                  
      } 

      // Wait for button press.
      if (Button2JustPressed())
      {
        // Sequence step 1.
        PrintString("Close button is pressed.", 1);
        SetAllDynamixelsToSlowSpeed();
        dynamixel5.Servo(d5_cw_limit);  
        dynamixel1.Servo(d1_ccw_limit);       
        dynamixel2.Servo(d2_ccw_limit);                           
        dynamixel3.Servo(d3_cw_limit); 
        dynamixel4.Servo(d4_cw_limit);                                   
        dynamixel6.Servo(d6_ccw_limit);
        SetAllDynamixelsToNormalSpeed();  
        eyesOpen = false;
        cheeksOpen = false;
        chinsidesOpen = false;
        chinOpen = false;                         
      } 
      break;
    }

    case SEQUENCE2:
    {  
      LightLED(0, false);
      LightLED(2, true);
      EncoderOrange();       
  
      // Check for any special button presses.
      ServiceButtons();

      int lastValue = slow_speed_factor;
       // Read the encoder.       
      int newValue = ReadEncoder( &buttonState, lastValue, TIME_SCALE_MIN, TIME_SCALE_MAX, 1 );    

      // If the value has changed.
      if ( newValue != lastValue )
      {
        lastValue = newValue;
        PrintString("slow_speed_factor: ", 0);
        PrintValue(newValue, 1); 
        slow_speed_factor = newValue; 
      }     
               
      // Wait for button press.
      if (Button1JustPressed())
      {        
        
        // Sequence step 1.
        PrintString("Open button is pressed.", 1);     
        dynamixel1.Servo(d1_cw_limit);
        dynamixel3.Servo(d3_ccw_limit);
        dynamixel5.Servo(d5_ccw_limit);
        dynamixel6.Servo(d6_ccw_limit - 50 );   
        eyesOpen = false;
        cheeksOpen = true;
        chinsidesOpen = true;
        chinOpen = true;                                                                   
      } 

      // Wait for button press.
      if (Button2JustPressed())
      {
        // Sequence step 1.
        PrintString("Close button is pressed.", 1);
        SetAllDynamixelsToSlowSpeed();                             
        dynamixel1.Servo(d1_ccw_limit);
        dynamixel3.Servo(d3_cw_limit);
        dynamixel5.Servo(d5_cw_limit);        
        SetAllDynamixelsToNormalSpeed();
        while ( AnyMovingDynamixels() )
        {
          // Wait...
        }
        dynamixel6.Servo(d6_ccw_limit);  
        eyesOpen = false;
        cheeksOpen = false;
        chinsidesOpen = false;
        chinOpen = false;                     
           
      } 

      // Extra button 3 toggles eyes.
      if ( EB3JustPressed() )
      {
        PrintString("Button 3 pressed.", 1);
        if (eyesOpen)
        { 
          // Cheeks must be open for eyes to open.
          if (cheeksOpen)        
          {
            PrintString("Eyes closing.", 1);          
            SetAllDynamixelsToSlowSpeed();
            dynamixel2.Servo(d2_ccw_limit);
            dynamixel4.Servo(d4_cw_limit);
            SetAllDynamixelsToNormalSpeed();
            eyesOpen = false;          
          }
          else
          {
            PrintString("Cheeks not open!", 1);          
          }
        }        
        else
        {
          // Cheeks must be open for the eyes to close.
          if (cheeksOpen)
          {
            PrintString("Eyes opening.", 1);          
            dynamixel2.Servo(d2_cw_limit);
            dynamixel4.Servo(d4_ccw_limit);
            eyesOpen = true;                
          }
          else
          {
            PrintString("Cheeks not open!", 1);          
          }
        }
      }
      
      break;
    }

    case SEQUENCE3:
    {  
      EncoderCyan();         
      // Check for any special button presses.
      ServiceButtons();

      // Extra button 3 toggles eyes.
      if ( EB3JustPressed() )
      {
        PrintString("Button 3 pressed.", 1);
        if (eyesOpen)
        { 
          // Cheeks must be open for eyes to open.
          if (cheeksOpen)        
          {
            PrintString("Eyes closing.", 1);          
            SetAllDynamixelsToSlowSpeed();
            dynamixel2.Servo(d2_ccw_limit);
            dynamixel4.Servo(d4_cw_limit);
            SetAllDynamixelsToNormalSpeed();
            eyesOpen = false;          
            LightLED(3, false);
          }
          else
          {
            PrintString("Cheeks not open!", 1);          
          }
        }        
        else
        {
          // Cheeks must be open for the eyes to close.
          if (cheeksOpen)
          {
            PrintString("Eyes opening.", 1);          
            dynamixel2.Servo(d2_cw_limit);
            dynamixel4.Servo(d4_ccw_limit);
            eyesOpen = true;          
            LightLED(3, true);
          }
          else
          {
            PrintString("Cheeks not open!", 1);          
          }
        }
      }
      
      // Extra button 2 toggles cheeks
      if ( EB2JustPressed() )
      {
        PrintString("Button 2 pressed.", 1);
        if (cheeksOpen)
        {   
          if (chinOpen)       
          {
            PrintString("Cheeks closing.", 1);
            SetAllDynamixelsToSlowSpeed();
            dynamixel1.Servo(d1_ccw_limit);
            dynamixel3.Servo(d3_cw_limit);
            SetAllDynamixelsToNormalSpeed();
            cheeksOpen = false;
            LightLED(2, false);
          }
          else 
          {
            PrintString("Chin not open!", 1);          
          }
        }
        else
        {
          if (chinOpen)
          {
            PrintString("Cheeks opening.", 1);
            dynamixel1.Servo(d1_cw_limit);
            dynamixel3.Servo(d3_ccw_limit);
            cheeksOpen = true;
            LightLED(2, true);
          }
          else
          {
            PrintString("Chin not open!", 1);
          }          
        }
      }
      
      // Extra button 4 toggles cheek sides.
      if ( EB4JustPressed() )
      {
        PrintString("Button 4 pressed.", 1);
        if (chinsidesOpen)
        {          
          PrintString("Chin sides closing.", 1);
          SetAllDynamixelsToSlowSpeed();
          dynamixel6.Servo(d6_ccw_limit);
          SetAllDynamixelsToNormalSpeed();
          chinsidesOpen = false;
          LightLED(4, false);
        }
        else
        {
          PrintString("Chin sides opening.", 1);
          dynamixel6.Servo(d6_cw_limit);
          chinsidesOpen = true;
          LightLED(4, true);
        }
      }
      
      // Extra button 1 toggles chin.
      if ( EB1JustPressed() )
      {
        PrintString("Button 1 pressed.", 1);
        if (chinOpen)
        {          
          PrintString("Chin closing.", 1);
          SetAllDynamixelsToSlowSpeed();
          dynamixel5.Servo(d5_cw_limit);
          SetAllDynamixelsToNormalSpeed();
          chinOpen = false;
          LightLED(1, false);
        }
        else
        {
          PrintString("Chin opening.", 1);
          dynamixel5.Servo(d5_ccw_limit);
          chinOpen = true;
          LightLED(1, true);
        }
      }      
      break;
    }

    case RCCONTROL:
    {  

      LightLED(0, false);
      EncoderOrange(); 
     
      // Check for any special button presses.
      ServiceButtons();    

      // Check if RC is enabled or not.
      if ( RCEnabled == false )
      {
        PrintString("Attaching RC interrupt.", 1); 
        
        // Attach an interrupt handler to be called whenever
        // the RC pin changes from LOW to HIGH or vice versa.
        attachInterrupt(rcpin, RCchannel, CHANGE);       

        // Temporarily set speed to maximum.
        // SetAllDynamixelsToSpeed(1023);

        // RC is now enabled.
        RCEnabled = true;
      }

      // See if we got an RC signal.        
      if ( gotRCValue == true )
      {
        HandleRCCommand();
      }
      
      break;
    }
    
    case RESET:
    {
      LightLED(0, false);
      EncoderWhite();     

      // Check for any special button presses.
      ServiceButtons();

      // Rest all Dynamixels.
      SetAllDynamixelHolding(true);
      SetAllDynamixelEnabled(true);
      PrintString("Enabling all Dynamixels.", 1);
      PrintString("Setting holding current on.", 1);

      break; 
    }    

    // Set the holding torque on of off for ALL Dynamixels.
    case FLACCID:
    {
      LightLED(0, false);
      EncoderOrange();
  
      FlaccidMode();           
      
      // Go back to the main mennu.
      menu = MENU_MAIN;       
      PrintValue(menu, 1);
      PrintString("-----|", 0);
      PrintMenu(menu);  
      PrintString("|-----", 1); 
      // Update the menu.
      oldmenu = menu;
      break;
    }

     // Print out the variables.
    case PRINTVARIABLES:
    {
      LightLED(0, false);
      // Print out all the variabels.
      PrintVariables();
      // Go back to the main mennu.
      menu = MENU_MAIN;       
      PrintString("-----|", 0);
      PrintMenu(menu);  
      PrintString("|-----", 1); 
      // Update the menu.
      oldmenu = menu;
      break;
    }
    
    default:
    {
      break;
    }
  }    
} 
