 // Includes. 
#include <WetaServo.h>                    // Weta servo object library.
#include <Servo.h>                        // Servo library.
#include <Dynamixel_Serial_Teensy.h>      // Library needed to control Dynamixal servo.
#include <Encoder.h>                      // Rotary encoder library.
#include <EEPROM.h>                       // EEprom library.
#include <Picaso_Serial_4DLib.h>          // LCD Library.      
#include <Picaso_const4D.h>               // LCD Library.     

// Author: Simon Jansen

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

const int rcpin = 26 ;                     // RC Receiver input pin.
const int minRC = 950;
const int maxRC = 2100;
// Variables for RC receiver.
volatile uint16_t channel;
volatile uint16_t channel_start;
volatile bool gotRCValue;
bool RCEnabled = false;

byte buttons[] = {pb1, pb2, pbExit, pbEnter, pbPanic, eb1, eb2, eb3, eb4, eb5}; // 
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
const int ledButton1 = 31;                // LED button 1.
const int ledButton2 = 25;                // LED button 2.
const int ledButton3 = 32;                // LED button 3.
const int ledButton4 = 24;                // LED button 4.
const int ledButton5 = 33;                // LED button 5. 

// Rotary encoder.
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
const int SHELL = 4;                          // Shelling sequence menu.
const int FLAPS_ONLY = 5;                     // Flaps only menu.
const int EYES_ONLY = 6;                      // Eyes only menu.
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
const int SET_HOLDING = 10;                // Turn on and off the holding tocque.
const int SET_ENABLED = 11;                // Enable or disable.
const int TEST_SERVO_LIMITS = 12;          // Test moving the servo in SERVO mode.
const int TEST_SERVO_WHEEL = 13;           // Test moving the servo in WHEEL mode.
const int LAST_DYNAMIXEL_VARIABLE = 13;

// Generic variables.
// First we define constants for each variable we'll have that needs
// setting. 
const int FIRST_VARIABLE = 1;
const int TIME_SCALE = 1;                 // Time scaling factor.
const int EYES_CLICK_PAUSE = 2;           // Delay before clicking eyes.
const int EYES_CLICK_TIME = 3;            // Time to click eyes.
const int EYES_IN_TIME = 4;               // Eyes in time.
const int EYES_DOWN_TIME = 5;             // Eyes down time.
const int SIDE_BIG_DELAY = 6;             // Big side flap delay.
const int SIDE_SMALL_DELAY = 7;           // Small side flap delay. 
const int TOP_DELAY = 8;                  // Top flap delay.
const int BASE_DELAY = 9;                 // Base flap delay. 
const int BASE_SIDE_DELAY = 10;           // Base sides delay.
const int BASE_REAR_DELAY = 11;           // Base rear flap delay.
const int SPEED_SCALE = 12;               // Speed scaling factor.
const int ITERATIONS = 13;                // Iterations.
const int PT_BACK_LEFT = 14;              // Pre-tension on back left.
const int PT_BACK_RIGHT = 15;             // Pre-tension on back right.
const int PT_BACK_SMALL = 16;             // Pre-tension on back small.
const int PT_SIDE_LEFT = 17;              // Pre-tension on side left.
const int PT_SIDE_RIGHT = 18;             // Pre-tension on side right.
const int LAST_VARIABLE = 18;

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
// Dynamixel 7 variables.
int d7_cw_speed = 0;
int d7_ccw_speed = 0;
int d7_acdc = 0;
int d7_cw_limit = 2048;
int d7_ccw_limit = 2048;
// Dynamixel 8 variables.
int d8_cw_speed = 2048;
int d8_ccw_speed = 2048;
int d8_acdc = 2048;
int d8_cw_limit = 2048;
int d8_ccw_limit = 2048;
// Dynamixel 9 variables.
int d9_cw_speed = 0;
int d9_ccw_speed = 0;
int d9_acdc = 0;
int d9_cw_limit = 2048;
int d9_ccw_limit = 2048;
// Dynamixel 10 variables.
int d10_cw_speed = 0;
int d10_ccw_speed = 0;
int d10_acdc = 0;
int d10_cw_limit = 2048;
int d10_ccw_limit = 2048;
// Dynamixel 11 variables.
int d11_cw_speed = 0;
int d11_ccw_speed = 0;
int d11_acdc = 0;
int d11_cw_limit = 2048;
int d11_ccw_limit = 2048;
// Dynamixel 12 variables.
int d12_cw_speed = 0;
int d12_ccw_speed = 0;
int d12_acdc = 0;
int d12_cw_limit = 2048;
int d12_ccw_limit = 2048;

// Other variables.
int time_scale = 0;
int eyes_click_pause = 0;
int eyes_click_time = 0;
int eyes_in_time = 0;
int eyes_down_time = 0;
int side_big_delay = 0;             
int side_small_delay = 0; 
int top_delay = 0;        
int base_delay = 0;               
int base_side_delay = 0;         
int base_rear_delay = 0;    
int iterations = 0;
int speed_scale = 0;
int pt_back_left = 0;
int pt_back_right = 0;
int pt_back_small = 0;
int pt_side_left = 0;
int pt_side_right = 0;

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
DynamixelServo dynamixel1(1, "Eyes in", "In", "Out");             
DynamixelServo dynamixel2(2, "Eyes down", "Up", "Down");           
DynamixelServo dynamixel3(3, "Right side big", "Close", "Open");         
DynamixelServo dynamixel4(4, "Left side big", "Open", "Close");        
DynamixelServo dynamixel5(5, "Top", "Close", "Open");        
DynamixelServo dynamixel6(6, "Back", "Close", "Open");        
DynamixelServo dynamixel7(7, "Back right", "Close", "Open");        
DynamixelServo dynamixel8(8, "Back left", "Open", "Close");                  
DynamixelServo dynamixel9(9, "Right side small", "Open", "Close");                 
DynamixelServo dynamixel10(10, "Left side small", "Close", "Open"); 
DynamixelServo dynamixel11(11, "Back small", "Close", "Open"); 
DynamixelServo dynamixel12(12, "Dynamixel 12", "Open", "Close"); 

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
int slowSpeed = 0;

elapsedMillis elapsed1;
elapsedMillis elapsed2;
elapsedMillis elapsed3;

bool topOpen = false;
bool backOpen = false;
bool bigSidesOpen = false;
bool smallSidesOpen = false;
bool backSidesOpen = false;
bool smallBackOpen = false;

// Variables used.
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
  slowSpeed = 0;
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
    d1_ccw_speed = 1023;
    d1_acdc = 3000;
    d1_cw_limit = 0;
    d1_ccw_limit = 0;
    d2_cw_speed = 991;
    d2_ccw_speed = 991;
    d2_acdc = 1339;
    d2_cw_limit = 0;
    d2_ccw_limit = 0;
    d3_cw_speed = 15;
    d3_ccw_speed = 40;
    d3_acdc = 0;
    d3_cw_limit = 1851;
    d3_ccw_limit = 2651;
    d4_cw_speed = 40;
    d4_ccw_speed = 15;
    d4_acdc = 0;
    d4_cw_limit = 1558;
    d4_ccw_limit = 2315;
    d5_cw_speed = 11;
    d5_ccw_speed = 40;
    d5_acdc = 0;
    d5_cw_limit = 1521;
    d5_ccw_limit = 2041;
    d6_cw_speed = 10;
    d6_ccw_speed = 40;
    d6_acdc = 0;
    d6_cw_limit = 1776;
    d6_ccw_limit = 2343;
    d7_cw_speed = 7;
    d7_ccw_speed = 50;
    d7_acdc = 0;
    d7_cw_limit = 1524;
    d7_ccw_limit = 2067;
    d8_cw_speed = 50;
    d8_ccw_speed = 7;
    d8_acdc = 0;
    d8_cw_limit = 1909;
    d8_ccw_limit = 2603;
    d9_cw_speed = 50;
    d9_ccw_speed = 7;
    d9_acdc = 0;
    d9_cw_limit = 1938;
    d9_ccw_limit = 2583;
    d10_cw_speed = 7;
    d10_ccw_speed = 50;
    d10_acdc = 0;
    d10_cw_limit = 1428;
    d10_ccw_limit = 2092;
    d11_cw_speed = 15;
    d11_ccw_speed = 50;
    d11_acdc = 0;
    d11_cw_limit = 1334;
    d11_ccw_limit = 2230;
    d12_cw_speed = 1024;
    d12_ccw_speed = 1024;
    d12_acdc = 1339;
    d12_cw_limit = 1848;
    d12_ccw_limit = 2248;
    time_scale = 100;
    speed_scale = 0;
    eyes_in_time = 4750;
    eyes_down_time = 5500;
    eyes_click_pause = 1;
    eyes_click_time = 1;
    side_big_delay = 0;
    side_small_delay = 0;
    top_delay = 0;
    base_delay = 0;
    base_side_delay = 0;
    base_rear_delay = 0;
    iterations = 1;
    pt_back_left = 200;
    pt_back_right = 100;
    pt_back_small = 0;
    pt_side_left = 0;
    pt_side_right = 0;
  

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
  dynamixelServoArray[6] = &dynamixel7;
  dynamixelServoArray[7] = &dynamixel8;
  dynamixelServoArray[8] = &dynamixel9;
  dynamixelServoArray[9] = &dynamixel10;
  dynamixelServoArray[10] = &dynamixel11;
  dynamixelServoArray[11] = &dynamixel12;
            
  dynamixel1.Start(WHEEL, d1_cw_speed, d1_ccw_speed, d1_acdc, d1_cw_limit, d1_ccw_limit);
  dynamixel2.Start(WHEEL, d2_cw_speed, d2_ccw_speed, d2_acdc, d2_cw_limit, d2_ccw_limit); 
  dynamixel3.Start(SERVO, d3_cw_speed, d3_ccw_speed, d3_acdc, d3_cw_limit, d3_ccw_limit); 
  dynamixel4.Start(SERVO, d4_cw_speed, d4_ccw_speed, d4_acdc, d4_cw_limit, d4_ccw_limit); 
  dynamixel5.Start(SERVO, d5_cw_speed, d5_ccw_speed, d5_acdc, d5_cw_limit, d5_ccw_limit); 
  dynamixel6.Start(SERVO, d6_cw_speed, d6_ccw_speed, d6_acdc, d6_cw_limit, d6_ccw_limit);      
  dynamixel7.Start(SERVO, d7_cw_speed, d7_ccw_speed, d7_acdc, d7_cw_limit, d7_ccw_limit);
  dynamixel8.Start(SERVO, d8_cw_speed, d8_ccw_speed, d8_acdc, d8_cw_limit, d8_ccw_limit); 
  dynamixel9.Start(SERVO, d9_cw_speed, d9_ccw_speed, d9_acdc, d9_cw_limit, d9_ccw_limit); 
  dynamixel10.Start(SERVO, d10_cw_speed, d10_ccw_speed, d10_acdc, d10_cw_limit, d10_ccw_limit); 
  dynamixel11.Start(SERVO, d11_cw_speed, d11_ccw_speed, d11_acdc, d11_cw_limit, d11_ccw_limit); 
  dynamixel12.Start(SERVO, d12_cw_speed, d12_ccw_speed, d12_acdc, d12_cw_limit, d12_ccw_limit);       
  
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
  digitalWrite(led1, HIGH);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
}

////////////////////////////////////////////////////////////
// Light the encoder Orange.
////////////////////////////////////////////////////////////  
void EncoderOrange(void)
{
  digitalWrite(led1, LOW);
  digitalWrite(led2, HIGH);
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
// Display a binary number on the LEDs.
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
  address = address + 2; 
  d1_ccw_speed = EEpromReadInt(address); 
  address = address + 2; 
  d1_acdc = EEpromReadInt(address);  
  address = address + 2;
  d1_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d1_ccw_limit = EEpromReadInt(address);  
  address = address + 2;
  
  // Dynamixel 2.  
  d2_cw_speed = EEpromReadInt(address);  
  address = address + 2;
  d2_ccw_speed = EEpromReadInt(address);  
  address = address + 2;
  d2_acdc = EEpromReadInt(address);  
  address = address + 2;
  d2_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d2_ccw_limit = EEpromReadInt(address);  
  address = address + 2;
  
  // Dynamixel 3.
  d3_cw_speed = EEpromReadInt(address);  
  address = address + 2;
  d3_ccw_speed = EEpromReadInt(address);  
  address = address + 2;
  d3_acdc = EEpromReadInt(address);  
  address = address + 2;
  d3_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d3_ccw_limit = EEpromReadInt(address);  
  address = address + 2;

  // Dynamixel 4.
  d4_cw_speed = EEpromReadInt(address);  
  address = address + 2;
  d4_ccw_speed = EEpromReadInt(address);  
  address = address + 2;
  d4_acdc = EEpromReadInt(address);  
  address = address + 2;
  d4_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d4_ccw_limit = EEpromReadInt(address);  
  address = address + 2;

  // Dynamixel 5.
  d5_cw_speed = EEpromReadInt(address);  
  address = address + 2;
  d5_ccw_speed = EEpromReadInt(address);  
  address = address + 2;
  d5_acdc = EEpromReadInt(address);  
  address = address + 2;
  d5_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d5_ccw_limit = EEpromReadInt(address);  
  address = address + 2;

  // Dynamixel 6.
  d6_cw_speed = EEpromReadInt(address);  
  address = address + 2;
  d6_ccw_speed = EEpromReadInt(address);  
  address = address + 2;
  d6_acdc = EEpromReadInt(address);  
  address = address + 2;
  d6_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d6_ccw_limit = EEpromReadInt(address);  
  address = address + 2;
  
  // Dynamixel 7.
  d7_cw_speed = EEpromReadInt(address); 
  address = address + 2; 
  d7_ccw_speed = EEpromReadInt(address); 
  address = address + 2; 
  d7_acdc = EEpromReadInt(address);  
  address = address + 2;
  d7_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d7_ccw_limit = EEpromReadInt(address);  
  address = address + 2;
  
  // Dynamixel 8.  
  d8_cw_speed = EEpromReadInt(address);  
  address = address + 2;
  d8_ccw_speed = EEpromReadInt(address);  
  address = address + 2;
  d8_acdc = EEpromReadInt(address);  
  address = address + 2;
  d8_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d8_ccw_limit = EEpromReadInt(address);  
  address = address + 2;
  
  // Dynamixel 9.
  d9_cw_speed = EEpromReadInt(address);  
  address = address + 2;
  d9_ccw_speed = EEpromReadInt(address);  
  address = address + 2;
  d9_acdc = EEpromReadInt(address);  
  address = address + 2;
  d9_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d9_ccw_limit = EEpromReadInt(address);  
  address = address + 2;

  // Dynamixel 10.
  d10_cw_speed = EEpromReadInt(address);  
  address = address + 2;
  d10_ccw_speed = EEpromReadInt(address);  
  address = address + 2;
  d10_acdc = EEpromReadInt(address);  
  address = address + 2;
  d10_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d10_ccw_limit = EEpromReadInt(address);  
  address = address + 2;

  // Dynamixel 11.
  d11_cw_speed = EEpromReadInt(address);  
  address = address + 2;
  d11_ccw_speed = EEpromReadInt(address);  
  address = address + 2;
  d11_acdc = EEpromReadInt(address);  
  address = address + 2;
  d11_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d11_ccw_limit = EEpromReadInt(address);  
  address = address + 2;

  // Dynamixel 12.
  d12_cw_speed = EEpromReadInt(address);  
  address = address + 2;
  d12_ccw_speed = EEpromReadInt(address);  
  address = address + 2;
  d12_acdc = EEpromReadInt(address);  
  address = address + 2;
  d12_cw_limit = EEpromReadInt(address);  
  address = address + 2;
  d12_ccw_limit = EEpromReadInt(address);  
  address = address + 2;
  
  // Other variables.
  time_scale = EEpromReadInt(address);  
  address = address + 2;
  speed_scale = EEpromReadInt(address);  
  address = address + 2; // 64
  eyes_in_time = EEpromReadInt(address);  
  address = address + 2;
  eyes_down_time = EEpromReadInt(address);  
  address = address + 2;
  eyes_click_pause = EEpromReadInt(address);  
  address = address + 2;
  eyes_click_time = EEpromReadInt(address);  
  address = address + 2;
  side_big_delay = EEpromReadInt(address);  
  address = address + 2; 
  side_small_delay = EEpromReadInt(address);  
  address = address + 2;
  top_delay = EEpromReadInt(address);  
  address = address + 2;
  base_delay = EEpromReadInt(address);  
  address = address + 2;
  base_side_delay = EEpromReadInt(address);  
  address = address + 2;
  base_rear_delay = EEpromReadInt(address);  
  address = address + 2;
  iterations = EEpromReadInt(address);
  address = address + 2;
  pt_back_left = EEpromReadInt(address); 
  address = address + 2;
  pt_back_right = EEpromReadInt(address); 
  address = address + 2;
  pt_back_small = EEpromReadInt(address); 
  address = address + 2;
  pt_side_left = EEpromReadInt(address); 
  address = address + 2;
  pt_side_right = EEpromReadInt(address); 
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
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d1_ccw_speed); 
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d1_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d1_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d1_ccw_limit);
  writeAddress = writeAddress + 2;

  // Dynamixel 2.
  EEpromWriteInt(writeAddress, d2_cw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d2_ccw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d2_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d2_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d2_ccw_limit);
  writeAddress = writeAddress + 2;

  // Dynamixel 3.
  EEpromWriteInt(writeAddress, d3_cw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d3_ccw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d3_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d3_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d3_ccw_limit);
  writeAddress = writeAddress + 2;

  // Dynamixel 4.
  EEpromWriteInt(writeAddress, d4_cw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d4_ccw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d4_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d4_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d4_ccw_limit);
  writeAddress = writeAddress + 2;

  // Dynamixel 5.
  EEpromWriteInt(writeAddress, d5_cw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d5_ccw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d5_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d5_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d5_ccw_limit);
  writeAddress = writeAddress + 2;

  // Dynamixel 6.
  EEpromWriteInt(writeAddress, d6_cw_speed);
  writeAddress = writeAddress + 2;
    EEpromWriteInt(writeAddress, d6_ccw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d6_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d6_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d6_ccw_limit);
  writeAddress = writeAddress + 2;
  
  // Dynamixel 7.
  EEpromWriteInt(writeAddress, d7_cw_speed); 
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d7_ccw_speed); 
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d7_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d7_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d7_ccw_limit);
  writeAddress = writeAddress + 2;

  // Dynamixel 8.
  EEpromWriteInt(writeAddress, d8_cw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d8_ccw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d8_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d8_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d8_ccw_limit);
  writeAddress = writeAddress + 2;

  // Dynamixel 9.
  EEpromWriteInt(writeAddress, d9_cw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d9_ccw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d9_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d9_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d9_ccw_limit);
  writeAddress = writeAddress + 2;

  // Dynamixel 10.
  EEpromWriteInt(writeAddress, d10_cw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d10_ccw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d10_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d10_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d10_ccw_limit);
  writeAddress = writeAddress + 2;

  // Dynamixel 11.
  EEpromWriteInt(writeAddress, d11_cw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d11_ccw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d11_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d11_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d11_ccw_limit);
  writeAddress = writeAddress + 2;

  // Dynamixel 12.
  EEpromWriteInt(writeAddress, d12_cw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d12_ccw_speed);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d12_acdc);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d12_cw_limit);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, d12_ccw_limit);
  writeAddress = writeAddress + 2;
  
  EEpromWriteInt(writeAddress, time_scale);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, speed_scale);
  writeAddress = writeAddress + 2; // 64
  EEpromWriteInt(writeAddress, eyes_in_time);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, eyes_down_time);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, eyes_click_pause);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, eyes_click_time);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, side_big_delay);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, side_small_delay);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, top_delay);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, base_delay);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, base_side_delay);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, base_rear_delay);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, iterations);
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, pt_back_left);      
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, pt_back_right);      
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, pt_back_small);      
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, pt_side_left);      
  writeAddress = writeAddress + 2;
  EEpromWriteInt(writeAddress, pt_side_left);      
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

 // Dynamixel 7.
  PrintString("d7_cw_speed = ", 0); PrintString(d7_cw_speed, 0); PrintString(";", 1);
  PrintString("d7_ccw_speed = ", 0); PrintString(d7_ccw_speed, 0); PrintString(";", 1);
  PrintString("d7_acdc = ", 0); PrintString(d7_acdc, 0); PrintString(";", 1);
  PrintString("d7_cw_limit = ", 0); PrintString(d7_cw_limit, 0); PrintString(";", 1);  
  PrintString("d7_ccw_limit = ", 0); PrintString(d7_ccw_limit, 0); PrintString(";", 1);

  // Dynamixel 8.
  PrintString("d8_cw_speed = ", 0); PrintString(d8_cw_speed, 0); PrintString(";", 1);
  PrintString("d8_ccw_speed = ", 0); PrintString(d8_ccw_speed, 0); PrintString(";", 1);
  PrintString("d8_acdc = ", 0); PrintString(d8_acdc, 0); PrintString(";", 1);
  PrintString("d8_cw_limit = ", 0); PrintString(d8_cw_limit, 0); PrintString(";", 1);  
  PrintString("d8_ccw_limit = ", 0); PrintString(d8_ccw_limit, 0); PrintString(";", 1);

  // Dynamixel 9.
  PrintString("d9_cw_speed = ", 0); PrintString(d9_cw_speed, 0); PrintString(";", 1);
  PrintString("d9_ccw_speed = ", 0); PrintString(d9_ccw_speed, 0); PrintString(";", 1);
  PrintString("d9_acdc = ", 0); PrintString(d9_acdc, 0); PrintString(";", 1);
  PrintString("d9_cw_limit = ", 0); PrintString(d9_cw_limit, 0); PrintString(";", 1);  
  PrintString("d9_ccw_limit = ", 0); PrintString(d9_ccw_limit, 0); PrintString(";", 1);

  // Dynamixel 10.
  PrintString("d10_cw_speed = ", 0); PrintString(d10_cw_speed, 0); PrintString(";", 1);
  PrintString("d10_ccw_speed = ", 0); PrintString(d10_ccw_speed, 0); PrintString(";", 1);
  PrintString("d10_acdc = ", 0); PrintString(d10_acdc, 0); PrintString(";", 1);
  PrintString("d10_cw_limit = ", 0); PrintString(d10_cw_limit, 0); PrintString(";", 1);  
  PrintString("d10_ccw_limit = ", 0); PrintString(d10_ccw_limit, 0); PrintString(";", 1);

  // Dynamixel 11.
  PrintString("d11_cw_speed = ", 0); PrintString(d11_cw_speed, 0); PrintString(";", 1);
  PrintString("d11_ccw_speed = ", 0); PrintString(d11_ccw_speed, 0); PrintString(";", 1);
  PrintString("d11_acdc = ", 0); PrintString(d11_acdc, 0); PrintString(";", 1);
  PrintString("d11_cw_limit = ", 0); PrintString(d11_cw_limit, 0); PrintString(";", 1);  
  PrintString("d11_ccw_limit = ", 0); PrintString(d11_ccw_limit, 0); PrintString(";", 1);

  // Dynamixel 12.
  PrintString("d12_cw_speed = ", 0); PrintString(d12_cw_speed, 0); PrintString(";", 1);
  PrintString("d12_ccw_speed = ", 0); PrintString(d12_ccw_speed, 0); PrintString(";", 1);
  PrintString("d12_acdc = ", 0); PrintString(d12_acdc, 0); PrintString(";", 1);
  PrintString("d12_cw_limit = ", 0); PrintString(d12_cw_limit, 0); PrintString(";", 1);  
  PrintString("d12_ccw_limit = ", 0); PrintString(d12_ccw_limit, 0); PrintString(";", 1);
  
  // Other variables.
  PrintString("time_scale = ", 0); PrintString(time_scale, 0); PrintString(";", 1);
  PrintString("speed_scale = ", 0); PrintString(speed_scale, 0); PrintString(";", 1);
  PrintString("eyes_in_time = ", 0); PrintString(eyes_in_time, 0); PrintString(";", 1);
  PrintString("eyes_down_time = ", 0); PrintString(eyes_down_time, 0); PrintString(";", 1);
  PrintString("eyes_click_pause = ", 0); PrintString(eyes_click_pause, 0); PrintString(";", 1);
  PrintString("eyes_click_time = ", 0); PrintString(eyes_click_time, 0); PrintString(";", 1);
  PrintString("side_big_delay = ", 0); PrintString(side_big_delay, 0); PrintString(";", 1);
  PrintString("side_small_delay = ", 0); PrintString(side_small_delay, 0); PrintString(";", 1);
  PrintString("top_delay = ", 0); PrintString(top_delay, 0); PrintString(";", 1);
  PrintString("base_delay = ", 0); PrintString(base_delay, 0); PrintString(";", 1);
  PrintString("base_side_delay = ", 0); PrintString(base_side_delay, 0); PrintString(";", 1);
  PrintString("base_rear_delay = ", 0); PrintString(base_rear_delay, 0); PrintString(";", 1);
  PrintString("iterations = ", 0); PrintString(iterations, 0); PrintString(";", 1);
  PrintString("pt_back_left = ", 0); PrintString(pt_back_left, 0); PrintString(";", 1);
  PrintString("pt_back_right = ", 0); PrintString(pt_back_right, 0); PrintString(";", 1);
  PrintString("pt_back_small = ", 0); PrintString(pt_back_small, 0); PrintString(";", 1);
  PrintString("pt_side_left = ", 0); PrintString(pt_side_left, 0); PrintString(";", 1);
  PrintString("pt_side_right = ", 0); PrintString(pt_side_right, 0); PrintString(";", 1); 
  
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
// Send action to a given Dynamixel.
////////////////////////////////////////////////////////////
void Action (int id)

{
  // Loop through all Dynamixel and send action command.
  for (int i = 0; i < NUMDYNAMIXELSERVOS; i++)
  {
    // If we find it return the pointer to it.
    if ( dynamixelServoArray[i] )
    {
      // If it is the one with a matching ID.
      if (dynamixelServoArray[i]->GetID() == id )
      {
        // Send action command.
        dynamixelServoArray[i]->Action();
      }  
    }    
  }
}

////////////////////////////////////////////////////////////
// Send action to all Dynamixels.
////////////////////////////////////////////////////////////
void ActionAll (void)

{
  // Loop through all Dynamixel and sent the action all command on the
  // first one found.
  for (int i = 0; i < NUMDYNAMIXELSERVOS; i++)
  {
    // If we find it return the pointer to it.
    if ( dynamixelServoArray[i] )
    {
      // And send action command.
      dynamixelServoArray[i]->ActionAll();      
      PrintString("Sending action all command.", 1); 
      return;
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
    PrintString("MENU_MAIN", 0);
  }
  else if ( menuToPrint == DYNAMIXEL_SELECT )
  { 
    PrintString("DYNAMIXEL_SELECT", 0);    
  }
  else if ( menuToPrint == SETUP )
  { 
    PrintString("SETUP", 0);
  }
  else if ( menuToPrint == SHELL )
  { 
    PrintString("SHELL", 0);
  }
  else if ( menuToPrint == FLAPS_ONLY )
  { 
    PrintString("FLAPS_ONLY", 0);
  }
  else if ( menuToPrint == EYES_ONLY )
  { 
    PrintString("EYES_ONLY", 0);
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
    PrintString("time_scale % - ", 0);   
    PrintValue(time_scale, 1);
  }
  else if ( variableToPrint == SPEED_SCALE )
  { 
    PrintString("speed_scale - ", 0);
    PrintValue(speed_scale, 1);
  }
  else if ( variableToPrint == EYES_CLICK_PAUSE )
  { 
    PrintString("eyes_click_pause - ", 0);
    PrintValue(eyes_click_pause, 1);
  }
  else if ( variableToPrint == EYES_CLICK_TIME )
  { 
    PrintString("eyes_click_time - ", 0);
    PrintValue(eyes_click_time, 1);
  }
  else if ( variableToPrint == EYES_IN_TIME )
  { 
    PrintString("eyes_in_time - ", 0);
    PrintValue(eyes_in_time, 1);
  }
  else if ( variableToPrint == EYES_DOWN_TIME )
  { 
    PrintString("eyes_down_time - ", 0);
    PrintValue(eyes_down_time, 1);
  }
  else if ( variableToPrint == EYES_DOWN_TIME )
  { 
    PrintString("eyes_down_time - ", 0);
    PrintValue(eyes_down_time, 1);
  }
  else if ( variableToPrint == SIDE_BIG_DELAY )
  { 
    PrintString("side_big_delay - ", 0);
    PrintValue(side_big_delay, 1);
  }
  else if ( variableToPrint == SIDE_SMALL_DELAY )
  { 
    PrintString("side_small_delay - ", 0);
    PrintValue(side_small_delay, 1);
  }
  else if ( variableToPrint == TOP_DELAY )
  { 
    PrintString("top_delay - ", 0);
    PrintValue(top_delay, 1);
  }
  else if ( variableToPrint == BASE_DELAY )
  { 
    PrintString("base_delay - ", 0);
    PrintValue(base_delay, 1);
  }
  else if ( variableToPrint == BASE_SIDE_DELAY )
  { 
    PrintString("base_side_delay - ", 0);
    PrintValue(base_side_delay, 1);
  }
  else if ( variableToPrint == BASE_REAR_DELAY )
  { 
    PrintString("base_rear_delay - ", 0);
    PrintValue(base_rear_delay, 1);
  }      
  else if ( variableToPrint == ITERATIONS )
  { 
    PrintString("Iterations - ", 0);
    PrintValue(iterations, 1);
  }
  else if ( variableToPrint == PT_BACK_LEFT )
  { 
    PrintString("pt_back_left - ", 0);
    PrintValue(pt_back_left, 1);
  }
  else if ( variableToPrint == PT_BACK_RIGHT )
  { 
    PrintString("pt_back_right - ", 0);
    PrintValue(pt_back_right, 1);
  }
  else if ( variableToPrint == PT_BACK_SMALL )
  { 
    PrintString("pt_back_small - ", 0);
    PrintValue(pt_back_small, 1);
  }
  else if ( variableToPrint == PT_SIDE_LEFT )
  { 
    PrintString("pt_side_left - ", 0);
    PrintValue(pt_side_left, 1);
  }  
  else if ( variableToPrint == PT_SIDE_RIGHT )
  { 
    PrintString("pt_side_right - ", 0);
    PrintValue(pt_side_right, 1);
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
  // If button 1 pressed.
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
    // Stop dynamixels. 
    StopDynamixels();
    
    // Stop any sequencing.
    sequenceStep = 0;
                   
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
  if ( EB1JustPressed() )
  {
     PrintString("Extra button 1 pressed.", 1);
      // Stop all servos.
     StopDynamixels();     
      // Go into the Shelling sequence.
     oldmenu = menu;
     menu = SHELL;        
     PrintString("-----|", 0); 
     PrintMenu(menu);
     PrintString("|-----", 1);
     editing_variable = false;
     selected_dynamixel = false;      
     editing_dynamixel_variable = false;  
     // Stop any sequencing.
     sequenceStep = 0;
  }
  
  if ( EB2JustPressed() )
  {
     PrintString("Extra button 2 pressed.", 1);
      // Stop all servos.
     StopDynamixels();
     // Stop any sequencing.
     sequenceStep = 0;
      // Do a reset.     
     oldmenu = menu; 
     menu = RESET; 
     PrintString("-----|", 0); 
     PrintMenu(menu);
     PrintString("|-----", 1);
     editing_variable = false;
     selected_dynamixel = false;      
     editing_dynamixel_variable = false; 
  }
  
  
  if ( EB3JustPressed() )
  {
     PrintString("Extra button 3 pressed.", 1);
     PrintString("PANIC!", 1);
     // Stop all servos.
     StopDynamixels();
     // Stop any sequencing.
     sequenceStep = 0;
    
     // Go back to the base menu.
     menu = MENU_MAIN;   
     PrintString("-----|", 0); 
     PrintMenu(menu);
     PrintString("|-----", 1);
     // Update the menu.
     oldmenu = menu;
     selected_dynamixel = true;
     editing_dynamixel_variable = false;     
  }
  
  if ( EB4JustPressed() )
  {
     PrintString("Extra button 4 pressed.", 1);      
     PrintString("Setting Flaccid mode: ON", 1);
     // Set the holding current off for all Dynamixels.
     SetAllDynamixelHolding(0);            
  }
    if ( EB5JustPressed() )
  {
     PrintString("Extra button 5 pressed.", 1);
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

    // Stop any moving Dynamixels.
    StopDynamixels();

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
    dynamixel->Wheel(0);              
  }  
  else if ((!Button1Pressed()) && Button2JustPressed())
  {
    PrintString("Button 2 pressed.", 1);
    PrintString("Moving LEFT.", 1);  
    dynamixel->SetDirection(DLEFT);
    dynamixel->Wheel(0);                
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
// Manual Adjust Dynamixal CW positional limit.
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
    case 6: d7_cw_speed = speed; break;
    case 7: d8_cw_speed = speed; break;        
    case 8: d9_cw_speed = speed; break;      
    case 9: d10_cw_speed = speed; break;      
    case 10: d11_cw_speed = speed; break;      
    case 11: d12_cw_speed = speed; break;        
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
    case 6: d7_ccw_speed = speed; break;
    case 7: d8_ccw_speed = speed; break;        
    case 8: d9_ccw_speed = speed; break;      
    case 9: d10_ccw_speed = speed; break;      
    case 10: d11_ccw_speed = speed; break;      
    case 11: d12_ccw_speed = speed; break;           
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
    case 6: d7_acdc = acdc; break;      
    case 7: d8_acdc = acdc; break;      
    case 8: d9_acdc = acdc; break;      
    case 9: d10_acdc = acdc; break;      
    case 10: d11_acdc = acdc; break;      
    case 11: d12_acdc = acdc; break;             
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
    case 6: d7_cw_limit = cw_limit; break;      
    case 7: d8_cw_limit = cw_limit; break;      
    case 8: d9_cw_limit = cw_limit; break;      
    case 9: d10_cw_limit = cw_limit; break;      
    case 10: d11_cw_limit = cw_limit; break;      
    case 11: d12_cw_limit = cw_limit; break;           
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
    case 6: d7_ccw_limit = ccw_limit; break;      
    case 7: d8_ccw_limit = ccw_limit; break;      
    case 8: d9_ccw_limit = ccw_limit; break;      
    case 9: d10_ccw_limit = ccw_limit; break;      
    case 10: d11_ccw_limit = ccw_limit; break;      
    case 11: d12_ccw_limit = ccw_limit; break;                 
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
    PrintString(channel, 1);
    
    // Update the flag to say we handled it.
    gotRCValue = false;      
}	
  
////////////////////////////////////////////////////////////
// Set all speeds to the given speed.
////////////////////////////////////////////////////////////  
void SetAllDynamixelsToSpeed(int speed)
{
  PrintString("Setting speeds to: ", 0);
  PrintValue(speed, 1);
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
// Main loop.
////////////////////////////////////////////////////////////  
void loop()
{
      
  ServiceDynamixels(); // Service any Dynamixels that need it.
        
  // Check the currently selected menu and change bahaviour based on it.
  switch (menu)
  {
    
    // In menu select menu we just allow changing the menu.
    case MENU_MAIN:
    {
      LightLED(0, false);
      EncoderBlue();

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
            selected_dynamixel = true;       
          }
          else
          {
            PrintString(dynamixel->GetName(), 0);
            PrintString(" not connected.", 1);
            selected_dynamixel = true;       
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
      EncoderCyan();         

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
        if (setup_variable == EYES_CLICK_PAUSE) EditVariable("eyes_click_pause", &eyes_click_pause, 0, 5000);
        if (setup_variable == EYES_CLICK_TIME) EditVariable("eyes_click_time", &eyes_click_time, 0, 5000);       
        if (setup_variable == SPEED_SCALE) EditVariable("speed_scale", &speed_scale, 0, 10000);  
		    if (setup_variable == EYES_IN_TIME) EditVariable("eyes_in_time", &eyes_in_time, 0, 10000);       
        if (setup_variable == EYES_DOWN_TIME) EditVariable("eyes_down_time", &eyes_down_time, 0, 10000);
        if (setup_variable == ITERATIONS) EditVariable("iterations", &iterations, 0, 10000); 
        if (setup_variable == PT_BACK_RIGHT) EditVariable("pt_back_right", &pt_back_right, 0, 2000); 
        if (setup_variable == PT_BACK_LEFT) EditVariable("pt_back_left", &pt_back_left, 0, 2000); 
        if (setup_variable == PT_BACK_SMALL) EditVariable("pt_back_small", &pt_back_small, 0, 2000); 
        if (setup_variable == PT_SIDE_RIGHT) EditVariable("pt_side_right", &pt_side_right, 0, 2000); 
        if (setup_variable == PT_SIDE_LEFT) EditVariable("pt_side_left", &pt_side_left, 0, 2000); 
        SetDynamixelsTimeScale(time_scale);
        // Store the values.
        WriteEEprom();      
      }
      break;
    }
    
    case SHELL:
    {
      LightLED(0, false);
      LightLED(5, true);  
      EncoderGreen();

      // Wait for button press to start sequence.
      if ( Button2JustPressed() && (sequenceStep == 0) )
      {
        // Sequence step 1.
        // Eyes down.        
        PrintString("Close button pressed.", 1);
        PrintString("Starting sequence...", 1);
        PrintString("Step 1. Eyes down, close.", 1);
        SetDynamixelsTimeScale(time_scale);
        sequenceStep = 1;
        sequenceStartTime = micros();
        tm = ScaleTime(eyes_down_time);           
        dynamixel2.SetDirection(DLEFT);        
        dynamixel2.Wheel(tm); // Wheel time in millis.    

        // Top.
        dynamixel5.ServoPreload(d5_cw_limit);
        topOpen = false;
        
        // Back.
        dynamixel6.ServoPreload(d6_cw_limit);
        backOpen = false;
        
        // Big sides.
        dynamixel3.ServoPreload(d3_cw_limit);   
        dynamixel4.ServoPreload(d4_ccw_limit); 
        bigSidesOpen = false;

        // Back sides.
        dynamixel7.ServoPreload(d7_cw_limit);    
        dynamixel8.ServoPreload(d8_ccw_limit);                 
        backSidesOpen = false;
        
        // Small sides.
        dynamixel9.ServoPreload(d9_ccw_limit);                 
        dynamixel10.ServoPreload(d10_cw_limit);                        
        smallSidesOpen = false;

        // Small back.
        dynamixel11.ServoPreload(d11_cw_limit); 
        smallBackOpen = false;

        // Set them all off.
        ActionAll();                       
        tm = tm * 1000; // Delay timings done in micros.        
      } 

      // Sequence step 2.
      // Eyes in.
      // Check the sequence step and time.        
      if ( (sequenceStep == 1) && (micros() >= sequenceStartTime + tm) )
      {
        // Next step in sequence.
        PrintString("Step 2. Eyes in.", 1);
        sequenceStep = 2;
        sequenceTime1 = micros();
        sequenceDelay1 = ScaleTime(eyes_click_pause);
        sequenceDelay1 = sequenceDelay1 * 1000;
        tm = ScaleTime(eyes_in_time);        
        dynamixel1.SetDirection(DRIGHT);
        dynamixel1.Wheel(tm); // Wheel time in millis.  
        tm = tm * 1000; // Delay timings done in micros.  
      }

      // Sequence step 3.
      // Eyes click.
      // Check the sequence step and time.
      if ( (sequenceStep == 2) && (micros() >= sequenceTime1 + tm + sequenceDelay1) )
      {
        // Next step in sequence.
        PrintString("Step 3. Eyes click.", 1);
        sequenceStep = 3;
        sequenceTime1 = micros();
        sequenceDelay1 = ScaleTime(1000);       
        sequenceDelay1 = sequenceDelay1 * 1000;        
        tm = ScaleTime(eyes_click_time);        
        dynamixel1.SetACDC(0);
        dynamixel1.SetDirection(DRIGHT);
        dynamixel1.Wheel(tm); // Wheel time in millis.  
        tm = tm * 1000; // Delay timings done in micros.
      }
      
      // Sequence step 4.
      // Sequence complete.
      if ( (sequenceStep == 3) && (micros() >= sequenceTime1 + tm + sequenceDelay1 ) && (!AnyMovingDynamixels()) ) 
      {
        PrintString("Run complete!", 1);
        SetDynamixelsTimeScale(TIME_SCALE_NORM);
        dynamixel1.SetACDC(d1_acdc);

        // Stop any sequencing.
        sequenceStep = 4;
      }       
      
      // Wait for button press for reset.
      if ( Button1JustPressed() )
      {
        // Reset the time scale.
        SetDynamixelsTimeScale(time_scale);
        dynamixel1.SetACDC(d1_acdc);
        
        // Stop all servos.
        StopDynamixels();    
        
        PrintString("Open button pressed.", 1);
        PrintString("Resetting...", 1);
                      
        // Reset positions. 
        
        // Small back.                 
        dynamixel11.Servo(d11_ccw_limit);
        smallBackOpen = true;    
        
        // Small sides.
        dynamixel9.Servo(d9_cw_limit);                 
        dynamixel10.Servo(d10_ccw_limit);                        
        smallSidesOpen = true;

        delay(500);

        // Pre tension back small
        PrintString("Pre-tensioning back small: ", 1);  
        dynamixel11.Servo(d11_ccw_limit - pt_back_small);    

        // Pre tension back sides.
        PrintString("Pre-tensioning small sides.", 1); 
        dynamixel9.Servo(d9_cw_limit + pt_side_right);   
        dynamixel10.Servo(d10_ccw_limit - pt_side_left);   
        
        // Back sides
        dynamixel7.Servo(d7_ccw_limit);    
        dynamixel8.Servo(d8_cw_limit);          
        backSidesOpen = true;

        delay(500);
        
        // Pre tension back sides.
        PrintString("Pre-tensioning small back.", 1); 
        dynamixel7.Servo(d7_ccw_limit - pt_back_right);    
        dynamixel8.Servo(d8_cw_limit + pt_back_left);   

        // Back.
        dynamixel6.Servo(d6_ccw_limit);
        backOpen = true;        

        delay(500);
        
        // Sides.
        dynamixel3.Servo(d3_ccw_limit);    
        dynamixel4.Servo(d4_cw_limit);
        bigSidesOpen = true;

        delay(500);
        
        // Top.
        dynamixel5.Servo(d5_ccw_limit);
        topOpen = true;

        PrintString("Moving D1.", 1); 
        tm = ScaleTime(eyes_in_time);         
        dynamixel1.SetDirection(DLEFT);        
        dynamixel1.Wheel(tm); // Wheel time in millis.    
                
        // Stop sequencing.
        sequenceStep = 5;

        // Set elapsed time.
        elapsed1 = 0;
      }

      // Reset sequence.
      if ( (sequenceStep == 5) && (elapsed1 > 1000) )
      {       
        PrintString("Moving D2.", 1); 
        tm = ScaleTime(eyes_down_time);           
        dynamixel2.SetDirection(DRIGHT);        
        dynamixel2.Wheel(tm); // Wheel time in millis.  

        // Next step in sequencing.
        sequenceStep = 6;

        // Set elapsed time.
        elapsed2 = 0;
      }

      if ( (sequenceStep == 6) && (elapsed2 > eyes_down_time + 100) )
      {       
        PrintString("Reset complete.", 1);
        SetDynamixelsTimeScale(TIME_SCALE_NORM);
        
        // Stop sequencing.
        sequenceStep = 0;
      }
      
      // Check for any special button presses.
      ServiceButtons();    
      
      break;
    }    

    case FLAPS_ONLY:
    {
      LightLED(0, false);
      LightLED(1, true);  
      EncoderBlue();
      
      // Wait for button press to start sequence.
      if ( Button2JustPressed() && (sequenceStep == 0) )
      {
        // Sequence step 1.    
        PrintString("Close button pressed.", 1);
        PrintString("Starting sequence...", 1);
        SetDynamixelsTimeScale(time_scale);
       
        // Top.
        dynamixel5.ServoPreload(d5_cw_limit);
        topOpen = false;
        
        // Back.
        dynamixel6.ServoPreload(d6_cw_limit);
        backOpen = false;
        
        // Big sides.
        dynamixel3.ServoPreload(d3_cw_limit);   
        dynamixel4.ServoPreload(d4_ccw_limit); 
        bigSidesOpen = false;

        // Back sides.
        dynamixel7.ServoPreload(d7_cw_limit);    
        dynamixel8.ServoPreload(d8_ccw_limit);                 
        backSidesOpen = false;
        
        // Small sides.
        dynamixel9.ServoPreload(d9_ccw_limit);                 
        dynamixel10.ServoPreload(d10_cw_limit);                        
        smallSidesOpen = false;

        // Small back.
        dynamixel11.ServoPreload(d11_cw_limit); 
        smallBackOpen = false;

        // Set them all off.
        ActionAll();                       
        tm = tm * 1000; // Delay timings done in micros.

        // Sequence step 1.
        sequenceStep = 1;
        sequenceStartTime = micros();
        
      } 

      // Sequence step 2. Sequence complete.
      if ( (sequenceStep == 1) && (!AnyMovingDynamixels()) ) 
      {
        PrintString("Run complete!", 1);
        SetDynamixelsTimeScale(TIME_SCALE_NORM);
        // Next sequence.
        sequenceStep = 0;
      }       

      // Wait for button press for reset.
      if (Button1JustPressed() && (sequenceStep != 2))
      {
         // Reset the time scale.
        SetDynamixelsTimeScale(time_scale);
        
        // Stop all servos.
        StopDynamixels();    
        
        PrintString("Open button pressed.", 1);
        PrintString("Resetting...", 1);
                      
        // Reset positions. 
        
        // Small back.                 
        dynamixel11.Servo(d11_ccw_limit);
        smallBackOpen = true;    
        
        // Small sides.
        dynamixel9.Servo(d9_cw_limit);                 
        dynamixel10.Servo(d10_ccw_limit);                        
        smallSidesOpen = true;

        delay(500);

        // Pre tension back small
        PrintString("Pre-tensioning back small: ", 1);  
        dynamixel11.Servo(d11_ccw_limit - pt_back_small);    

        // Pre tension back sides.
        PrintString("Pre-tensioning small sides.", 1); 
        dynamixel9.Servo(d9_cw_limit + pt_side_right);   
        dynamixel10.Servo(d10_ccw_limit - pt_side_left);   
        
        // Back sides
        dynamixel7.Servo(d7_ccw_limit);    
        dynamixel8.Servo(d8_cw_limit);          
        backSidesOpen = true;

        delay(500);
        
        // Pre tension back sides.
        PrintString("Pre-tensioning small back.", 1); 
        dynamixel7.Servo(d7_ccw_limit - pt_back_right);    
        dynamixel8.Servo(d8_cw_limit + pt_back_left);   

        // Back.
        dynamixel6.Servo(d6_ccw_limit);
        backOpen = true;        

        delay(500);
        
        // Sides.
        dynamixel3.Servo(d3_ccw_limit);    
        dynamixel4.Servo(d4_cw_limit);
        bigSidesOpen = true;

        delay(500);
        
        // Top.
        dynamixel5.Servo(d5_ccw_limit);
        topOpen = true;

        // Next step in sequence.
        sequenceStep = 2;
      } 

      // Sequence step 2. Sequence complete.
      if ( (sequenceStep == 2) && (!AnyMovingDynamixels()) ) 
      {
        PrintString("Run complete!", 1);
        SetDynamixelsTimeScale(TIME_SCALE_NORM);
        // Next sequence.
        sequenceStep = 0;
      } 
      
      // Check for any special button presses.
      ServiceButtons();    
      
      break;
    }   
    
    case EYES_ONLY:
    {      
      LightLED(0, false);    
      EncoderPurple();
      
      // Wait for button press to start sequence.
      if ( Button2JustPressed() && (sequenceStep == 0) )
      {
        // Sequence step 1.
        // Eyes down.        
        PrintString("Close button pressed.", 1);
        PrintString("Starting sequence...", 1);
        PrintString("Step 1. Eyes down, close.", 11);
        SetDynamixelsTimeScale(time_scale);
        sequenceStep = 1;
        sequenceStartTime = micros();
        tm = ScaleTime(eyes_down_time);           
        dynamixel2.SetDirection(DLEFT);        
        dynamixel2.Wheel(tm); // Wheel time in millis.             
        tm = tm * 1000; // Delay timings done in micros.
        
      } 

      // Sequence step 2.
      // Eyes in.
      // Check the sequence step and time.        
      if ( (sequenceStep == 1) && (micros() >= sequenceStartTime + tm) )
      {
        // Next step in sequence.
        PrintString("Step 2. Eyes in.", 1);
        sequenceStep = 2;
        tm = ScaleTime(eyes_in_time);        
        dynamixel1.SetDirection(DRIGHT);
        dynamixel1.Wheel(tm); // Wheel time in millis.  
        sequenceTime1 = micros(); 
      }

      // Sequence step 2. Sequence complete.
      if ( (sequenceStep == 2) && (micros() >= sequenceTime1 + ScaleTime(eyes_in_time + 100)) && (!AnyMovingDynamixels()) ) 
      {
        PrintString("Run complete!", 1);
        SetDynamixelsTimeScale(TIME_SCALE_NORM);
        // Next sequence.
        sequenceStep = 0;
      }       

      // Wait for button press for reset.
      if (Button1JustPressed() && (sequenceStep != 3))
      {
         // Reset the time scale.
        SetDynamixelsTimeScale(time_scale);
        
        // Stop all servos.
        StopDynamixels();    
        
        PrintString("Open button pressed.", 1);
        PrintString("Resetting...", 1);
                      
        // Reset positions.       
        PrintString("Moving D1.", 1); 
        tm = ScaleTime(eyes_in_time);         
        dynamixel1.SetDirection(DLEFT);        
        dynamixel1.Wheel(tm); // Wheel time in millis.    
                
        // Stop sequencing.
        sequenceStep = 3;

        // Set elapsed time.
        elapsed1 = 0;
      }

      // Reset sequence.
      if ( (sequenceStep == 3) && (elapsed1 > 1000) )
      {       
        PrintString("Moving D2.", 1); 
        tm = ScaleTime(eyes_down_time);           
        dynamixel2.SetDirection(DRIGHT);        
        dynamixel2.Wheel(tm); // Wheel time in millis.  

        // Next step in sequencing.
        sequenceStep = 4;

        // Set elapsed time.
        elapsed2 = 0;
      }

      if ( (sequenceStep == 4) && (elapsed2 > eyes_down_time + 100) )
      {       
        PrintString("Reset complete.", 1);
        SetDynamixelsTimeScale(TIME_SCALE_NORM);
        
        // Stop sequencing.
        sequenceStep = 0;
      }
      
      // Check for any special button presses.
      ServiceButtons();    
      
      break;
    }    

    case RESET:
    {
      LightLED(0, false);
      LightLED(4, true);  
      EncoderWhite();

      // Reset sequence.
      if ( sequenceStep == 0 )
      {
	      // Reset the time scale.
        SetDynamixelsTimeScale(time_scale);
        
        // Stop all servos.
        StopDynamixels();    
        
        PrintString("Open button pressed.", 1);
        PrintString("Resetting...", 1);
                      
        // Reset positions. 
        
        // Small back.                 
        dynamixel11.Servo(d11_ccw_limit);
        smallBackOpen = true;    
        
        // Small sides.
        dynamixel9.Servo(d9_cw_limit);                 
        dynamixel10.Servo(d10_ccw_limit);                        
        smallSidesOpen = true;

        delay(500);

        // Pre tension back small
        PrintString("Pre-tensioning back small: ", 1);  
        dynamixel11.Servo(d11_ccw_limit - pt_back_small);    

        // Pre tension back sides.
        PrintString("Pre-tensioning small sides.", 1); 
        dynamixel9.Servo(d9_cw_limit + pt_side_right);   
        dynamixel10.Servo(d10_ccw_limit - pt_side_left);   
        
        // Back sides
        dynamixel7.Servo(d7_ccw_limit);    
        dynamixel8.Servo(d8_cw_limit);          
        backSidesOpen = true;

        delay(500);
        
        // Pre tension back sides.
        PrintString("Pre-tensioning small back.", 1); 
        dynamixel7.Servo(d7_ccw_limit - pt_back_right);    
        dynamixel8.Servo(d8_cw_limit + pt_back_left);   

        // Back.
        dynamixel6.Servo(d6_ccw_limit);
        backOpen = true;        

        delay(500);
        
        // Sides.
        dynamixel3.Servo(d3_ccw_limit);    
        dynamixel4.Servo(d4_cw_limit);
        bigSidesOpen = true;

        delay(500);
        
        // Top.
        dynamixel5.Servo(d5_ccw_limit);
        topOpen = true;

        PrintString("Moving D1.", 1); 
        tm = ScaleTime(eyes_in_time);         
        dynamixel1.SetDirection(DLEFT);        
        dynamixel1.Wheel(tm); // Wheel time in millis.    
                       
        sequenceStep = 1;

        // Set elapsed time.
        elapsed1 = 0;
      }

      // Move eyes up.
      if ( (sequenceStep == 1) && (elapsed1 > 1000) )
      {       
        PrintString("Moving D2.", 1); 
        tm = ScaleTime(eyes_down_time);           
        dynamixel2.SetDirection(DRIGHT);        
        dynamixel2.Wheel(tm); // Wheel time in millis.  

        // Next step in sequence.           
        sequenceStep = 2;

        // Set elapsed time.
        elapsed2 = 0;
      }

      // We are done after the eyes finish moving.
      if ( (sequenceStep == 2) && (elapsed2 > eyes_down_time + 100) )
      {       
      
        PrintString("Reset complete.", 1);
        SetDynamixelsTimeScale(TIME_SCALE_NORM);

        // Stop sequencing.
        sequenceStep = 2;
        
        // Go back to the main mennu.
        menu = MENU_MAIN;       
        PrintString("-----|", 0);
        PrintMenu(menu);  
        PrintString("|-----", 1); 
        // Update the menu.
        oldmenu = menu;
      }

      // Check for any special button presses.
      ServiceButtons();    
                    
      break;
    }    

    // Set the holding torque on of off for ALL Dynamixels.
    case FLACCID:
    {

      EncoderOrange();   
         
      FlaccidMode(); 
      
      // Go back to the main mennu.
      menu = MENU_MAIN;       
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


     // Print out the variables.
    case RCCONTROL:
    {
      // Check for any special button presses.
      ServiceButtons();   
      break;
    }
    
    
    default:
    {
      break;
    }
  }
 
}
