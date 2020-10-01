const String version_string = "29";
const int dmx_page_size = 32;

#include "LEDGroup.h"
#include "DmxReceiver.h"
#include "RunningAverage.h"

#define SPINNINGCOG

// Running average for battery voltage.
const int batteryVoltagePin = A0;                   // Pin to read voltage on.
const int batteryRASize = 10;                       // Number of readings to average over.
RunningAverage batteryRA(batteryRASize);            // Running average object.
elapsedMillis batteryCheckTimer = 0;                // Timer for battery voltage readings.
const unsigned long batteryCheckInterval = 5000;    // Time between battery voltage readings.
const int batteryHighVoltage = 14000;               // Battery max level.
const int batteryLowVoltage = 10500;                // Battery low warning level.
const int batteryDeadVoltage = 10100;               // Battery absolutely flat level.
enum batteryLevelEnum { BATT_OK, 
                        BATT_LOW, 
                        BATT_DEAD };                // Battery level states.
batteryLevelEnum batteryLevel = BATT_OK;            // Battery current level.                   

// DMX receiver.
DmxReceiver dmx;
IntervalTimer dmxTimer;
const int dmx_updateTimer = 25;
const int DMXIntPin = 13;                           // Pin to indicate DMX traffic.
bool DMXIntPinOn = false;

// DMX channel offset.
int DMXOffset = 0;
int DMXChannelOffset = 0;
const int spinningcogDMXOffset = 14;                // Hard coded DMX offset for the spinning cog suit.

// Defines for the FastLED library.
// We support four strips on pins 2, 3, 4 and 5.
#define led_type            SK6822
#define color_order         GRB
#define data_pin_1          2
#define data_pin_2          3
#define data_pin_3          4
#define data_pin_4          5
#define num_leds_1          90                      // Set the number of LEDs on each strip here.
#define num_leds_2          20
#define num_leds_3          40
#define num_leds_4          45

// Define the arrays of LEDs for Fast LED.
CRGB leds_1[num_leds_1];
CRGB leds_2[num_leds_2];
CRGB leds_3[num_leds_3];
CRGB leds_4[num_leds_4];

const int default_brightness = 128;
const int maxTorchBrightness = 40;                  // Maximum torch brightness PWM value. 
const int maxFanSpeed = 192;                        // Maximum fan speed PWM value.

// Pins for DMX address.
const int dmxAddressBit1 = 15;                      // Pins to use for DMX address setting.
const int dmxAddressBit2 = 16;
const int dmxAddressBit3 = 17;
const int dmxAddressBit4 = 18;

// LED blinking used for battery low indication.
bool lowBattBlink = false;
bool lowBattBlinkOn = false;
elapsedMillis lowBattBlinkTimer = 0;
unsigned long lowBattBlinkInterval = 250;               // Speed of blinking in mS.

// Camera flashing LED.
bool cameraLEDOn = false;
bool cameraLEDFullOn = false;
bool cameraLEDFullOff = false;
const int minBlinkTime = 20;                        // Minimum blink time.
const int maxBlinkTime = 3000;                      // Maximum blink time.
elapsedMillis blinkTimer = 0;
unsigned long blinkTime = maxBlinkTime;

// Pins for torch and Fans.
const int torchPWMPin = 9;
const int fanPWMPin = 10;
const int torchControlPin = 11;
const int fanControlPin = 12;

// Pin for comms input switch.
const int commsSwitch = 21;
bool commsLEDsOn = false;

// Variables for the spinning cogs.
const int minFlickerTime = 1;                         // Minimum flicker time in mS.
const int maxFlickerTime = 500;                       // Maximum flicker time in mS.
const int minFlickerBrightness = 0;                   // Minimum flicker brightness.
const int maxFlickerBrightness1 = 255;                // Maximum flicker brightness LED group 1.
const int maxFlickerBrightness2 = 255;                // Maximum flicker brightness LED group 2.
elapsedMillis flickerTimer1 = 0;
unsigned long flickerTime1 = maxFlickerTime;
elapsedMillis flickerTimer2 = 0;
unsigned long flickerTime2 = maxFlickerTime;
unsigned long mappedFlickerTime = maxFlickerTime;
const int colourChangeChance = 10;                    // Colour change chance.
const int cogMotor1PWM = 22;                          // Motor 1.
const int cogMotor1Dir1 = 16;
const int cogMotor1Dir2 = 15;
const int cogMotor2PWM = 23;                          // Motor 2.
const int cogMotor2Dir1 = 17;
const int cogMotor2Dir2 = 18;
enum cogDirectionEnum { COG_OFF, 
                        COG_CW, 
                        COG_CCW };                  // Cog direction states.
cogDirectionEnum cog1Dir = COG_OFF;                 // Cog 1 direction.   
cogDirectionEnum cog2Dir = COG_OFF;                 // Cog 2 direction.   
const int minCogSpeed = 50;                         // Minimum cog motor speed.
int maxCogSpeed = 255;                              // Maximum cog motor speed.
const int minCogJitter = 1;                         // Minimum cog jitter time in mS.
const int maxCogJitter = 1000;                      // Maximum cog jitter time in mS.
elapsedMillis cog1Timer = 0;                        // Cog 1 movement timer.
unsigned long cog1Time = minCogJitter;              // Cog 1 movement time.
elapsedMillis cog2Timer = 0;                        // Cog 2 movement timer.
unsigned long cog2Time = minCogJitter;              // Cog 2 movement time.


// Define groups of LEDs.

// All LEDs strip 1.
const int strip1AllMin = 0;
const int strip1AllMax = num_leds_1;
LEDGroup allLEDsStrip1 ("AllStrip1", leds_1, strip1AllMin, strip1AllMax);

// All LEDs strip 2.
const int strip2AllMin = 0;
const int strip2AllMax = num_leds_2;
LEDGroup allLEDsStrip2 ("AllStrip2", leds_2, strip2AllMin, strip2AllMax);

// All LEDs strip 3.
const int strip3AllMin = 0;
const int strip3AllMax = num_leds_3;
LEDGroup allLEDsStrip3 ("AllStrip3", leds_3, strip3AllMin, strip3AllMax);

// All LEDs strip 4.
const int strip4AllMin = 0;
const int strip4AllMax = num_leds_4;
LEDGroup allLEDsStrip4 ("AllStrip4", leds_4, strip4AllMin, strip4AllMax);


// Strip 1. Neck, camera and comms.

// Blinking camera LED.
const int cameraLEDMin = 0;
const int cameraLEDMax = 0;
LEDGroup camLEDs ("Camera LEDs", leds_1, cameraLEDMin, cameraLEDMax);

// Neck 1.
const int neck1Min = 1;
const int neck1Max = 82;
LEDGroup neck1LEDs ("Neck1", leds_1, neck1Min, neck1Max);

// Comms LED.
const int commsLEDMin = 83;
const int commsLEDMax = 83;
LEDGroup commsLEDs ("Comms LEDs", leds_1, commsLEDMin, commsLEDMax);


// Strip 2. Battery level, backpack 1 and backpack 2.

// Backpack 1.
const int backpack1Min = 0;
const int backpack1Max = 1;
LEDGroup backpack1LEDs ("Backpack 1", leds_2, backpack1Min, backpack1Max);

// Battery level 1.
const int batteryLevel1Min = 2;
const int batteryLevel1Max = 6;
LEDGroup batteryLevel1LEDs ("Battery Level 1 LEDs", leds_2, batteryLevel1Min, batteryLevel1Max);

// Battery level 2.
const int batteryLevel2Min = 7;
const int batteryLevel2Max = 11;
LEDGroup batteryLevel2LEDs ("Battery Level 2 LEDs", leds_2, batteryLevel2Min, batteryLevel2Max);

// Backpack 2.
const int backpack2Min = 12;
const int backpack2Max = 12;
LEDGroup backpack2LEDs ("Backpack 2", leds_2, backpack2Min, backpack2Max);

// Exosuit rings.
const int exoRingsMin = 0;
const int exoRingsMax = 29;
LEDGroup exoLED1s ("Exo Suit LEDs 1", leds_3, exoRingsMin, exoRingsMax);

#ifdef SPINNINGCOG
  LEDGroup exoLED2s ("Exo Suit LEDs 2", leds_3, exoRingsMin + exoRingsMax + 1, exoRingsMax + exoRingsMax + 1); 
#else
  LEDGroup exoLED2s ("Exo Suit LEDs 2", leds_4, exoRingsMin, exoRingsMax);
#endif

// Spinning cog lights.
const int spinningCog1Min = 0;
const int spinningCog1Max = 21;
const int spinningCog2Min = 22;
const int spinningCog2Max = 42;
LEDGroup spinningCog1LEDs ("Spinning cog 1 LEDs", leds_4, spinningCog1Min, spinningCog1Max);
LEDGroup spinningCog2LEDs ("Spinning cog 2 LEDs", leds_4, spinningCog2Min, spinningCog2Max);

// Variables that are changed by DMX updates.

// Global brightness.
int globalBrightness = -1;  
int old_globalBrightness = -1;  

// Torch brightness.
int torchBrightness = -1;  
int old_torchBrightness = -1;    

// Fan speed.
int fanSpeed = -1;  
int old_fanSpeed = -1;    

// Camera LED.
int camLED_h = -1;  
int camLED_s = -1;  
int camLED_v = -1;  
int old_camLED_h = -1;
int old_camLED_s = -1;
int old_camLED_v = -1;
int cameraLEDBlinkRate = -1;
int old_cameraLEDBlinkRate = -1;

// Battery level indicators.
int batteryLevelLED_h = -1;
int batteryLevelLED_s = -1;
int batteryLevelLED_v = -1;
int old_batteryLevelLED_h = -1;
int old_batteryLevelLED_s = -1;
int old_batteryLevelLED_v = -1;
int batteryIndicatorLevel = -1;
int old_batteryIndicatorLevel = -1;

// Neck LEDs.
int neckLEDs_h = -1;
int neckLEDs_s = -1;
int neckLEDs_v = -1;
int old_neckLEDs_h = -1;
int old_neckLEDs_s = -1;
int old_neckLEDs_v = -1;

// Comms LEDs.
int commsLEDs_h = -1;
int commsLEDs_s = -1;
int commsLEDs_v = -1;
int old_commsLEDs_h = -1;
int old_commsLEDs_s = -1;
int old_commsLEDs_v = -1;

// Backpack1 LEDs.
int backpack1LEDs_h = -1;
int backpack1LEDs_s = -1;
int backpack1LEDs_v = -1;
int old_backpack1LEDs_h = -1;
int old_backpack1LEDs_s = -1;
int old_backpack1LEDs_v = -1;

// Backpack2 LEDs.
int backpack2LEDs_h = -1;
int backpack2LEDs_s = -1;
int backpack2LEDs_v = -1;
int old_backpack2LEDs_h = -1;
int old_backpack2LEDs_s = -1;
int old_backpack2LEDs_v = -1;

// Exo suit LEDs.
int exoLEDs_h = -1;
int exoLEDs_s = -1;
int exoLEDs_v = -1;
int old_exoLEDs_h = -1;
int old_exoLEDs_s = -1;
int old_exoLEDs_v = -1;

// Spinning cog LEDs.
int spinningCogLEDs_h = -1;
int spinningCogLEDs_s = -1;
int spinningCogLEDs_v = -1;
int old_spinningCogLEDs_h = -1;
int old_spinningCogLEDs_s = -1;
int old_spinningCogLEDs_v = -1;
int spinningCogLEDs_flicker = -1;
int old_spinningCogLEDs_flicker = -1;
int cogSpeed = -1;  
int old_cogSpeed = -1;   
int cogJitter = -1;  
int old_cogJitter = -1;   

// ----------------------------------------------------------------------
// DMX interupt service routine.
// ----------------------------------------------------------------------
void dmxTimerISR(void)
{
  dmx.bufferService();                            
  digitalWrite(DMXIntPin, DMXIntPinOn);
  DMXIntPinOn = !DMXIntPinOn;  
}

// ----------------------------------------------------------------------
// Print time in milliseconds.
// ----------------------------------------------------------------------
void PrintTime()
{
  Serial.print(millis());
  Serial.print(":\t");
}

// ----------------------------------------------------------------------
// Set Global Brightness.
// ----------------------------------------------------------------------
void SetGlobalBrightness(int globalBrightness)
{
  if ( (globalBrightness >= 0) && (globalBrightness < 256) )
  {
    LEDS.setBrightness(globalBrightness);
    FastLED.show(); 
    PrintTime();
    Serial.print("Setting global brightness: ");
    Serial.println(globalBrightness);
  }
}

// ----------------------------------------------------------------------
// Cog 1 direction.
// ----------------------------------------------------------------------
void SetCog1Direction ( cogDirectionEnum dir )
{
  PrintTime();
  Serial.print("Setting cog 1 direction to: ");  

  if ( dir == COG_OFF )
  {
    Serial.println("COG_OFF");
    digitalWrite (cogMotor1Dir1, LOW);
    digitalWrite (cogMotor1Dir2, LOW);    
  }
  else if ( dir == COG_CW )
  {
    Serial.println("COG_CW");
    digitalWrite (cogMotor1Dir1, LOW);
    digitalWrite (cogMotor1Dir2, HIGH);    
  }
  else if ( dir == COG_CCW )
  {
    Serial.println("COG_CCW");
    digitalWrite (cogMotor1Dir1, HIGH);
    digitalWrite (cogMotor1Dir2, LOW);    
  }
  else
  {
    Serial.println("direction unknown");
  }
}

// ----------------------------------------------------------------------
// Cog 1 speed.
// ----------------------------------------------------------------------
void SetCog1Speed ( int pwmValue )
{
  PrintTime();
  if ( (pwmValue >= 0 ) && (pwmValue < 256) )
  { 
       
    Serial.print("Cog 1 speed value is: "); 
    Serial.println(pwmValue);   

    // Map it to the maximum we have set.
    int mappedValue = map(pwmValue, 0, 255, minCogSpeed, maxCogSpeed);

    Serial.print("Setting cog 1 mapped speed value to: "); 
    Serial.println(mappedValue);  
    
    analogWrite(cogMotor1PWM, mappedValue);
  }
  else
  {   
    Serial.print("Speed value out of range: "); 
    Serial.println(pwmValue);  
  }
}

// ----------------------------------------------------------------------
// Cog 2 direction.
// ----------------------------------------------------------------------
void SetCog2Direction ( cogDirectionEnum dir )
{
  PrintTime();
  Serial.print("Setting cog 2 direction to: ");  

  if ( dir == COG_OFF )
  {
    Serial.println("COG_OFF");
    digitalWrite (cogMotor2Dir1, LOW);
    digitalWrite (cogMotor2Dir2, LOW);    
  }
  else if ( dir == COG_CW )
  {
    Serial.println("COG_CW");
    digitalWrite (cogMotor2Dir1, LOW);
    digitalWrite (cogMotor2Dir2, HIGH);    
  }
  else if ( dir == COG_CCW )
  {
    Serial.println("COG_CCW");
    digitalWrite (cogMotor2Dir1, HIGH);
    digitalWrite (cogMotor2Dir2, LOW);    
  }
  else
  {
    Serial.println("direction unknown");
  }
}

// ----------------------------------------------------------------------
// Cog 2 speed.
// ----------------------------------------------------------------------
void SetCog2Speed ( int pwmValue )
{
  PrintTime();
  if ( (pwmValue >= 0 ) && (pwmValue < 256) )
  { 
       
    Serial.print("Cog 2 speed value is: "); 
    Serial.println(pwmValue);   

    // Map it to the maximum we have set.
    int mappedValue = map(pwmValue, 0, 255, minCogSpeed, maxCogSpeed);

    Serial.print("Setting cog 2 mapped speed value to: "); 
    Serial.println(mappedValue);  
    
    analogWrite(cogMotor2PWM, mappedValue);
  }
  else
  {   
    Serial.print("Speed value out of range: "); 
    Serial.println(pwmValue);  
  }
}
// ----------------------------------------------------------------------
// Torch on.
// ----------------------------------------------------------------------
void TorchOn()
{
  PrintTime();
  Serial.println("Turning on torch.");  
  digitalWrite(torchControlPin, HIGH);
}

// ----------------------------------------------------------------------
// Torch off.
// ----------------------------------------------------------------------
void TorchOff()
{
  PrintTime();
  Serial.println("Turning off torch.");  
  digitalWrite(torchControlPin, LOW);
}

// ----------------------------------------------------------------------
// Torch set PWM.
// ----------------------------------------------------------------------
void TorchSetPWM(int pwmValue)
{
  PrintTime();
  if ( (pwmValue >= 0 ) && (pwmValue < 256) )
  { 
       
    Serial.print("Torch PWM value is: "); 
    Serial.println(pwmValue);   

    // Map it to the maximum we have set.
    int mappedValue = map(pwmValue, 0, 255, 0, maxTorchBrightness);

    Serial.print("Setting torch mapped PWM value to: "); 
    Serial.println(mappedValue);  
    
    analogWrite(torchPWMPin, mappedValue);
  }
  else
  {   
    Serial.print("PWM value out of range: "); 
    Serial.println(pwmValue);  
  }
}

// ----------------------------------------------------------------------
// Fan on.
// ----------------------------------------------------------------------
void FanOn()
{
  PrintTime();
  Serial.println("Turning on Fan.");  
  digitalWrite(fanControlPin, HIGH);
}

// ----------------------------------------------------------------------
// Fan off.
// ----------------------------------------------------------------------
void FanOff()
{
  PrintTime();
  Serial.println("Turning off Fan.");  
  digitalWrite(fanControlPin, LOW);
}

// ----------------------------------------------------------------------
// Fan set PWM.
// ----------------------------------------------------------------------
void FanSetPWM(int pwmValue)
{
  PrintTime();
  if ( (pwmValue > 0 ) && (pwmValue < 256) )
  {
    Serial.print("Fan PWM value is: "); 
    Serial.println(pwmValue);   

    // Map it to the maximum we have set.
    int mappedValue = map(pwmValue, 0, 255, 0, maxFanSpeed);

    Serial.print("Setting fan speed mapped PWM value to: "); 
    Serial.println(mappedValue);  
    
    analogWrite(fanPWMPin, mappedValue);
  }
  else
  {
    Serial.print("PWM value out of range: "); 
    Serial.println(pwmValue);  
  }
}

// ----------------------------------------------------------------------
// Camera LED set rate.
// ----------------------------------------------------------------------
void CameraLEDSetRate(int value)
{
  PrintTime();
  if ( (value >= 0 ) && (value <= 255) )
  {
    // If at zero we are fully off. 
    if ( value == 0 )
    {
      cameraLEDFullOn = false;
      cameraLEDFullOff = true;
      Serial.println("Setting camera LED to full OFF."); 
      blinkTime = minBlinkTime;
    }
    else if ( value == 255 )
    {
      cameraLEDFullOn = true;
      cameraLEDFullOff = false;
      Serial.println("Setting camera LED to full ON."); 
      blinkTime = minBlinkTime;
    }
    else
    {
      cameraLEDFullOn = false;
      cameraLEDFullOff = false;
      // Map the value to the time.
      blinkTime = map (value, 0, 255, maxBlinkTime, minBlinkTime);  
      Serial.print("Setting camera LED blink rate to: "); 
      Serial.println(blinkTime);   
      // Reset the timer.
      blinkTimer = 0;
      // Turn it on.
      camLEDs.On();            
      // Set the state.
      cameraLEDOn = true;
    }
  }
  else
  {
    Serial.print("Value out of range: "); 
    Serial.println(value);  
  }
}

// ----------------------------------------------------------------------
// Handle random flickering for the spinning cogs 1.
// ----------------------------------------------------------------------
void HandleCog1Light()
{
  PrintTime();
  Serial.println("Handle spinning cog 1 lights.");
  
  // Reset the flickering timer.
  flickerTimer1 = 0;

  // Generate a new random flickering time.
  long randomTime = random( minFlickerTime, mappedFlickerTime + 1 );

  // Set the new time.
  flickerTime1 = randomTime;

  // Generate a new random brightness level.
  int mappedBrightness = map (spinningCogLEDs_v, 0, 255, 0, maxFlickerBrightness1);
  PrintTime();
  Serial.print("Mapped cog 1 brightness: ");
  Serial.println(mappedBrightness);
  long randomBrightness = random( 0, mappedBrightness );
  PrintTime();
  Serial.print("Random cog 1 brightness: ");
  Serial.println(randomBrightness);

  // Generate a new random colour change.
  long randomColour = random ( 0, colourChangeChance );

  // Set the new brightness and colour.
  int saturation = 0;
  if ( randomColour == 0 )
  {   
      // Set the saturation to 0 (white).   
      saturation = 0;
  }
  else
  {
      // Set the saturation to normal.
      saturation = spinningCogLEDs_s;
  }

  // Set the new colour and brightness.
  spinningCog1LEDs.SetHSV( spinningCogLEDs_h, saturation, randomBrightness );
  
  // Show the leds.
  FastLED.show();
}

// ----------------------------------------------------------------------
// Handle random flickering for the spinning cogs 2.
// ----------------------------------------------------------------------
void HandleCog2Light()
{
  PrintTime();
  Serial.println("Handle spinning cog 2 lights.");
  
  // Reset the flickering timer.
  flickerTimer2 = 0;

  // Generate a new random flickering time.
  long randomTime = random( minFlickerTime, mappedFlickerTime + 1 );

  // Set the new time.
  flickerTime2 = randomTime;

  // Generate a new random brightness level.
  int mappedBrightness = map (spinningCogLEDs_v, 0, 255, 0, maxFlickerBrightness2);
  PrintTime();
  Serial.print("Mapped cog 2 brightness: ");
  Serial.println(mappedBrightness);
  long randomBrightness = random( 0, mappedBrightness );
  PrintTime();
  Serial.print("Random cog 2 brightness: ");
  Serial.println(randomBrightness);

  // Generate a new random colour change.
  long randomColour = random ( 0, colourChangeChance );

  // Set the new brightness and colour.
  int saturation = 0;
  if ( randomColour == 0 )
  {   
      // Set the saturation to 0 (white).   
      saturation = 0;
  }
  else
  {
      // Set the saturation to normal.
      saturation = spinningCogLEDs_s;
  }

  // Set the new colour and brightness.
  spinningCog2LEDs.SetHSV( spinningCogLEDs_h, saturation, randomBrightness );
  
  // Show the leds.
  FastLED.show();
}

// ----------------------------------------------------------------------
// Handle cog 1 random movement.
// ----------------------------------------------------------------------
void HandleCog1()
{
  PrintTime();
  Serial.println("Handle cog 1 speed and direction.");
  
  // Reset the  timer.
  cog1Timer = 0;

  // Generate a new random  time.
  int maxJitter = map (cogJitter, 0, 255, minCogJitter, maxCogJitter);
  PrintTime();
  Serial.print("Setting maximum cog jitter to: ");
  Serial.println(maxJitter);
  long randomTime = random( minCogJitter, maxJitter );

  // Set the new time.
  cog1Time = randomTime;
  PrintTime();
  Serial.print("Setting cog 1 time to: ");
  Serial.println(cog1Time);

  // Generate a new random speed.
  long randomSpeed = random( minCogSpeed, maxCogSpeed + 1 );

  // Generate a new random direction.
  long randomDirection = random( 0, 100 );
  cogDirectionEnum dir;

  // 10% chance we stop.
  if (randomDirection < 10) 
  {
    dir = COG_OFF;
  }
  // 80% chance we go one direction.
  else if (randomDirection > 90) 
  {
    dir = COG_CW;
  }
  else // 10% chance we go the opposite direction.
  {
    dir = COG_CCW;
  }
  
  // Set the speed and direction.
  SetCog1Speed(randomSpeed);
  SetCog1Direction(dir);
}

// ----------------------------------------------------------------------
// Handle cog 2 random movement.
// ----------------------------------------------------------------------
void HandleCog2()
{
  PrintTime();
  Serial.println("Handle cog 2 speed and direction.");
  
  // Reset the  timer.
  cog2Timer = 0;

  // Generate a new random  time.
  int maxJitter = map (cogJitter, 0, 255, minCogJitter, maxCogJitter);
  PrintTime();
  Serial.print("Setting maximum cog jitter to: ");
  Serial.println(maxJitter);
  long randomTime = random( minCogJitter, maxJitter );

  // Set the new time.
  cog2Time = randomTime;
  PrintTime();
  Serial.print("Setting cog 2 time to: ");
  Serial.println(cog2Time);

  // Generate a new random speed.
  long randomSpeed = random( minCogSpeed, maxCogSpeed + 1 );

  // Generate a new random direction.
  long randomDirection = random( 0, 100 );
  cogDirectionEnum dir;

  // 10% chance we stop.
  if (randomDirection < 10) 
  {
    dir = COG_OFF;
  }
  // 15% chance we go one direction.
  else if (randomDirection >= 85) 
  {
    dir = COG_CW;
  }
  else // 75% chance we go the opposite direction.
  {
    dir = COG_CCW;
  }
  
  // Set the speed and direction.
  SetCog2Speed(randomSpeed);
  SetCog2Direction(dir);
}
// ----------------------------------------------------------------------
// Start up LED test sequence.
// ----------------------------------------------------------------------
void StartUpLEDTest()
{
  // Startup routine.
  PrintTime();
  Serial.println("LED startup test. Setting Red/Green pattern.");
  bool ledStrip1Running = true;
  bool ledStrip2Running = true;
  bool ledStrip3Running = true;
  bool ledStrip4Running = true;
  int index = 0;
  
  while ( (ledStrip1Running == true) || (ledStrip2Running == true) || (ledStrip3Running == true) || (ledStrip4Running == true) )
  {
    // Check if there are more to turn on on each strip.
    if ( index == num_leds_1 ) ledStrip1Running = false;
    if ( index == num_leds_2 ) ledStrip2Running = false;
    if ( index == num_leds_3 ) ledStrip3Running = false;
    if ( index == num_leds_4 ) ledStrip4Running = false;
    
    // If so turn on the next LED.
    if ( ledStrip1Running )
    {      
      if ( (index % 2) == 0 )
      {
        leds_1[index].setRGB(255, 0, 0);
      }
      else
      {
        leds_1[index].setRGB(0, 255, 0);
      }   
    }
    if ( ledStrip2Running )
    {
      if ( (index % 2) == 0 )
      {
        leds_2[index].setRGB(255, 0, 0);
      }
      else
      {
        leds_2[index].setRGB(0, 255, 0);
      }
    }
    if ( ledStrip3Running )
    {
      if ( (index % 2) == 0 )
      {
        leds_3[index].setRGB(255, 0, 0);
      }
      else
      {
        leds_3[index].setRGB(0, 255, 0);
      }
    }
    if ( ledStrip4Running )
    {
      if ( (index % 2) == 0 )
      {
        leds_4[index].setRGB(255, 0, 0);
      }
      else
      {
        leds_4[index].setRGB(0, 255, 0);
      }
    }

    // Increment the index.
    index++;

    // Show the leds.
    FastLED.show();
  }

  // Delay 3 seconds.
  delay(3000);

  // Reset.
  Serial.println("Setting white pattern.");
  ledStrip1Running = true;
  ledStrip2Running = true;
  ledStrip3Running = true;
  ledStrip4Running = true;
  index = 0;
  
  while ( (ledStrip1Running == true) || (ledStrip2Running == true) || (ledStrip3Running == true) || (ledStrip4Running == true) )
  {
    // Check if there are more to turn on on each strip.
    if ( index == num_leds_1 ) ledStrip1Running = false;
    if ( index == num_leds_2 ) ledStrip2Running = false;
    if ( index == num_leds_3 ) ledStrip3Running = false;
    if ( index == num_leds_4 ) ledStrip4Running = false;
    
    // If so turn on the next LED.
    if ( ledStrip1Running )
    {      
      leds_1[index].setRGB(128, 128, 128);   
    }
    if ( ledStrip2Running )
    {
      leds_2[index].setRGB(128, 128, 128);
    }
    if ( ledStrip3Running )
    {
      leds_3[index].setRGB(128, 128, 128);
    }
    if ( ledStrip4Running )
    {
      leds_4[index].setRGB(128, 128, 128);
    }

    // Increment the index.
    index++;
    
    // Show the leds.
    FastLED.show();
  }
  
  // Delay 3 seconds.
  delay(3000);
}

// ----------------------------------------------------------------------
// Setup.
// ----------------------------------------------------------------------
void setup()
{
  Serial.begin(250000);
  delay(1000);

  PrintTime();
  Serial.print("Wandering Earth Controller. Version: ");
  Serial.println(version_string);

  #ifdef SPINNINGCOG
    Serial.println("Spinning cogs defined. Using modified code.");
  #else
    Serial.println("Spinning cogs not defined. Using normal code.");
  #endif  
    
  // Set up input pins.
  pinMode(commsSwitch, INPUT_PULLUP);

  #ifdef SPINNINGCOG
    pinMode(cogMotor1PWM, OUTPUT);
    pinMode(cogMotor1Dir1, OUTPUT);
    pinMode(cogMotor1Dir2, OUTPUT);
    pinMode(cogMotor2PWM, OUTPUT);
    pinMode(cogMotor2Dir1, OUTPUT);
    pinMode(cogMotor2Dir2, OUTPUT);
  #else
    pinMode(dmxAddressBit1, INPUT_PULLUP);
    pinMode(dmxAddressBit2, INPUT_PULLUP);
    pinMode(dmxAddressBit3, INPUT_PULLUP);
    pinMode(dmxAddressBit4, INPUT_PULLUP);
    
  #endif
  
  // Set up output pins.
  pinMode(torchPWMPin, OUTPUT);
  pinMode(fanPWMPin, OUTPUT);
  pinMode(torchControlPin, OUTPUT);
  pinMode(fanControlPin, OUTPUT);
  pinMode(DMXIntPin, OUTPUT);
 

  // Get the battery level.
  CheckBatteryLevel();

  #ifdef SPINNINGCOG
    // Use the preset offset.
    Serial.print("Using hardcoded spinningcog DMXOffset: ");
    Serial.println(spinningcogDMXOffset);
    DMXOffset = spinningcogDMXOffset;
    DMXChannelOffset = DMXOffset * dmx_page_size;
    Serial.print("DMX Channel Offset: ");
    Serial.println(DMXChannelOffset);
  #else
    // Get the DMX channel offset.
    Serial.println("Calling GetDMXOffset.");
    GetDMXOffset();
  #endif

  Serial.print("Using DMX channels: ");
  Serial.print(DMXChannelOffset + 1);
  Serial.print(" to: ");
  Serial.println(DMXChannelOffset + 32);
  
  dmx.begin();                                      // Initialise DMX.
  dmxTimer.begin(dmxTimerISR, dmx_updateTimer);     // Use a timer to service DMX buffers.
  
  // Set up the four fast LED objects.
  FastLED.addLeds< led_type, data_pin_1, color_order> (leds_1, num_leds_1);
  FastLED.addLeds< led_type, data_pin_2, color_order> (leds_2, num_leds_2);
  FastLED.addLeds< led_type, data_pin_3, color_order> (leds_3, num_leds_3);
  FastLED.addLeds< led_type, data_pin_4, color_order> (leds_4, num_leds_4);

  // Clear all LED strips.
  FastLED.clear();

  // Set global brightness.
  FastLED.setBrightness(default_brightness);

  // No blinking for all LEDs.
  lowBattBlink = false;
  lowBattBlinkOn = false;
  
  // Start up test.
  //StartUpLEDTest();

  // Initial settings.
  camLEDs.SetHSV(0, 255, 255);                // Camera LED is Red.
  batteryLevel1LEDs.SetHSV(0, 255, 255);      // Battery level LEDs are red.
  batteryLevel2LEDs.SetHSV(0, 255, 255);      // Battery level LEDs are red.
  batteryIndicatorLevel = 195;                // Battery level LEDs are red.
  neck1LEDs.SetHSV(0, 0, 255);                // Neck LEDs are white.
  commsLEDs.SetHSV(96, 255, 255);             // Comms LED is green.
  commsLEDs.Off();                            // Comms off.
  commsLEDsOn = false;  
  backpack1LEDs.SetHSV(0, 255, 255);          // Backpack 1 LEDs are blue.
  backpack2LEDs.SetHSV(171, 255, 255);        // Backpack 2 LEDs are red.
  exoLED1s.SetHSV(16, 255, 255);              // Exo suit LEDs are orange.
  exoLED2s.SetHSV(16, 255, 255);              // Exo suit LEDs are orange.
  spinningCogLEDs_h = 16;                     // Set the default hue.
  spinningCogLEDs_s = 255;
  spinningCogLEDs_v = 255;
  spinningCog1LEDs.SetHSV(spinningCogLEDs_h, 255, maxFlickerBrightness1);  // Spinning cog LEDs are orange.
  spinningCog2LEDs.SetHSV(spinningCogLEDs_h, 255, maxFlickerBrightness2);  // Spinning cog LEDs are orange.
  spinningCogLEDs_flicker = 10;  
  cogJitter = 50;
  mappedFlickerTime = map( spinningCogLEDs_flicker, 0, 255, minFlickerTime, maxFlickerTime );
  
  // Torch on at low brightness.
  TorchSetPWM(10);
  TorchOn();

  // Fan at highest speed.
  FanSetPWM(255);
  FanOn();

  // Show all changes.
  FastLED.show();
}

// ----------------------------------------------------------------------
// Handle the flashing camera LED.
// ----------------------------------------------------------------------
void HandleCameraLEDFlashing()
{ 
  // Reset the timer.
  blinkTimer = 0;
  
  // Are we full on?
  if ( (cameraLEDFullOn == true) && ( cameraLEDOn == false) )
  {
    // Turn it on.
    camLEDs.On();
    // Set the state.
    cameraLEDOn = true;    
  }
  // Are we full off?
  else if ( (cameraLEDFullOff == true) && ( cameraLEDOn == true) )
  {
    // Turn it off.
    camLEDs.Off();
    // Set the state.
    cameraLEDOn = false;    
  }
  // Do we blink?
  else if ( (cameraLEDFullOn == false) && (cameraLEDFullOff == false) )
  {
    // Should we be on or off.
    if ( cameraLEDOn == true )
    {
      // Turn it off.
      Serial.println("Setting camera LEDs off.");
      camLEDs.Off();        
    }
    else
    {
      // Turn it on.
      Serial.println("Setting camera LEDs on.");
      camLEDs.On();
    }
    // Invert the state.
    cameraLEDOn = !cameraLEDOn;
  }
} 

// ----------------------------------------------------------------------
// Get DMX offset.
// ----------------------------------------------------------------------
void GetDMXOffset()
{
  DMXOffset = 0;

  PrintTime();
  
  // Check the dmx address pins.
  if ( digitalRead(dmxAddressBit1) == LOW )
  {
     Serial.println("DMX address bit 1 set.");
     DMXOffset = DMXOffset + 1;
  }
  if ( digitalRead(dmxAddressBit2) == LOW )
  {
     Serial.println("DMX address bit 2 set.");
     DMXOffset = DMXOffset + 2;
  }
  if ( digitalRead(dmxAddressBit3) == LOW )
  {
     Serial.println("DMX address bit 3 set.");
     DMXOffset = DMXOffset + 4;
  }
  if ( digitalRead(dmxAddressBit4) == LOW )
  {
     Serial.println("DMX address bit 4 set.");
     DMXOffset = DMXOffset + 8;
  }
  Serial.print("DMX offset is: ");
  Serial.println(DMXOffset);

  DMXChannelOffset = DMXOffset * dmx_page_size;
  
  Serial.print("DMX channel offset is: ");
  Serial.println(DMXChannelOffset);
}


// ----------------------------------------------------------------------
// Check the comms switch.
// ----------------------------------------------------------------------
void CheckCommsSwitch()
{
  // Check the comms switch.
  if ( (digitalRead(commsSwitch) == LOW) && (commsLEDsOn == false) )
  {
     PrintTime();
     Serial.println("Comms switch on.");
     commsLEDs.On(); // Comms LED on.
     commsLEDsOn = true;
  }
  else if ( (digitalRead(commsSwitch) == HIGH) && (commsLEDsOn == true) )
  {
    PrintTime();
    Serial.println("Comms switch off.");
    commsLEDs.Off(); // Comms LED off.
    commsLEDsOn = false;
  }
}

// ----------------------------------------------------------------------
// Check the battery level is ok. 
// Set batteryLevel to OK if above minimum level.
// Set batteryLevel to low when between minimum and dead.
// Set batteryLevel to dead when below absolute minimum. 
// ----------------------------------------------------------------------
void CheckBatteryLevel()
{
  int voltage = 0;
  int voltageInMillivolts = 0;
    
  // Get the battery voltage.
  voltage = analogRead(batteryVoltagePin);
  //Serial.print("Battery voltage analog read: ");
  //Serial.println(voltage);

  // Map it.
  voltageInMillivolts = map ( voltage, 0, 1023, 0, batteryHighVoltage );
  //Serial.print("Battery voltage mapped: ");
  //Serial.println(volatgeInMillivolts);

  batteryRA.addValue(voltageInMillivolts);
  //Serial.print("Battery voltage average: ");
  //Serial.println(batteryRA.getAverage());

  PrintTime();
  // Check the level.
  // Is it above minimum.
  if ( voltageInMillivolts > batteryLowVoltage )
  {
    // Battery level ok.
    Serial.print("Battery level is at: ");
    Serial.print(voltageInMillivolts);
    Serial.println(" millivolts.");
    batteryLevel = BATT_OK;
  }
  // Is it below the low level warning but above flat.
  else if ( voltageInMillivolts > batteryDeadVoltage )
  {
    // Battery level is at or below minimum.
    Serial.print("Battery level: ");
    Serial.print(voltageInMillivolts);
    Serial.print(" is below minimum of: ");
    Serial.print(batteryLowVoltage);
    Serial.println(" millivolts.");
    batteryLevel = BATT_LOW;
  }
  // Else it is below the absolute minimum.
  else 
  {
    // Battery level is at or below absolute minimum.
    Serial.print("Battery level: ");
    Serial.print(voltageInMillivolts);
    Serial.print(" is below absolute minimum of: ");
    Serial.print(batteryDeadVoltage);
    Serial.println(" millivolts.");
    batteryLevel = BATT_DEAD;
  }
}

// ----------------------------------------------------------------------
// Check for any changes in values and implement them.
// This is called after getting new DMX data. We check what has changed
// and do the appropriate things.
// ----------------------------------------------------------------------
void CheckForChanges()
{
  // Global brightness.
  if ( globalBrightness != old_globalBrightness )
  {  
    PrintTime();
    Serial.print("Global brightness changed to: ");          
    Serial.println(globalBrightness);        
    SetGlobalBrightness(globalBrightness);
    old_globalBrightness = globalBrightness;    
  }

  // Torch brightness.
  if ( torchBrightness != old_torchBrightness )
  {
    PrintTime();
    Serial.print("Torch brightness changed to: ");
    Serial.println(torchBrightness);
    TorchSetPWM(torchBrightness);     
    old_torchBrightness = torchBrightness;
  }

  // Fan speed.
  if ( fanSpeed != old_fanSpeed )
  {
    PrintTime();
    Serial.print("Fan speed changed to: ");
    Serial.println(fanSpeed);       
    if ( fanSpeed == 0 )
    {
      FanOff();
    }
    else
    {
      FanOn();
      FanSetPWM(fanSpeed);     
    }
    old_fanSpeed = fanSpeed;  
  }

  // Camera LED.
  bool updateCameraLED = false;
  
  if ( camLED_h != old_camLED_h) 
  { 
    PrintTime();
    Serial.print("Camera LEDs H changed to: ");
    Serial.println(camLED_h);     
    old_camLED_h = camLED_h; 
    updateCameraLED = true;        
  }
  
  if ( camLED_s != old_camLED_s) 
  { 
    PrintTime();
    Serial.print("Camera LEDs S changed to: ");
    Serial.println(camLED_s);     
    old_camLED_s = camLED_s; 
    updateCameraLED = true;        
  }
  
  if ( camLED_v != old_camLED_v) 
  { 
    PrintTime();
    Serial.print("Camera LEDs V changed to: ");
    Serial.println(camLED_v);     
    old_camLED_v = camLED_v; 
    updateCameraLED = true;        
  }
  
  if ( updateCameraLED == true )
  {
    camLEDs.SetHSV(camLED_h, camLED_s, camLED_v);  
  }
  
  // Camera LED blink rate. 
  if ( cameraLEDBlinkRate != old_cameraLEDBlinkRate )
  {
    PrintTime();
    Serial.print("Camera LEDs blink rate changed to: ");
    Serial.println(cameraLEDBlinkRate);
    CameraLEDSetRate(cameraLEDBlinkRate);     
    old_cameraLEDBlinkRate = cameraLEDBlinkRate;    
  }

  // Batery level.
  bool updateBatteryLevel = false;
  
  if ( batteryLevelLED_h != old_batteryLevelLED_h )
  {  
    PrintTime();
    Serial.print("Battery level H changed to: ");          
    Serial.println(batteryLevelLED_h);       
    old_batteryLevelLED_h = batteryLevelLED_h;         
    updateBatteryLevel = true;
  }
  
  if ( batteryLevelLED_s != old_batteryLevelLED_s )
  {  
    PrintTime();
    Serial.print("Battery level S changed to: ");          
    Serial.println(batteryLevelLED_s);   
    old_batteryLevelLED_s = batteryLevelLED_s;         
    updateBatteryLevel = true;
  }
  
  if ( batteryLevelLED_v != old_batteryLevelLED_v )
  {  
    PrintTime();
    Serial.print("Battery level V changed to: ");          
    Serial.println(batteryLevelLED_v);    
    old_batteryLevelLED_v = batteryLevelLED_v;         
    updateBatteryLevel = true;
  }  
  
  if (updateBatteryLevel == true )
  {
    batteryLevel1LEDs.SetHSV(batteryLevelLED_h, batteryLevelLED_s, batteryLevelLED_v);  
    batteryLevel2LEDs.SetHSV(batteryLevelLED_h, batteryLevelLED_s, batteryLevelLED_v);          
  }
  
  // Battery level indicator.
  if ( batteryIndicatorLevel != old_batteryIndicatorLevel )
  {
    int percentage = map(batteryIndicatorLevel, 0, 255, 0, 100);
    PrintTime();
    Serial.print("Battery level percentage changed to: ");
    Serial.println(percentage);
    batteryLevel1LEDs.SetBargraphUp(percentage);
    batteryLevel2LEDs.SetBargraphDown(percentage);         
    old_batteryIndicatorLevel = batteryIndicatorLevel;       
  }    

  // Neck LEDs.
  bool updateNeckLEDs = false;
  if ( neckLEDs_h != old_neckLEDs_h )
  { 
    PrintTime();
    Serial.print("Neck LEDs H changed to: ");
    Serial.println(neckLEDs_h);     
    old_neckLEDs_h = neckLEDs_h;       
    updateNeckLEDs = true;      
  }
  
  if ( neckLEDs_s != old_neckLEDs_s )
  { 
    PrintTime();
    Serial.print("Neck LEDs S changed to: ");
    Serial.println(neckLEDs_s);         
    old_neckLEDs_s = neckLEDs_s;    
    updateNeckLEDs = true;      
  }
  
  if ( neckLEDs_v != old_neckLEDs_v )
  { 
    PrintTime();
    Serial.print("Neck LEDs V changed to: ");
    Serial.println(neckLEDs_v);       
    old_neckLEDs_v = neckLEDs_v;   
    updateNeckLEDs = true;      
  }

  if ( updateNeckLEDs == true )
  {
    neck1LEDs.SetHSV(neckLEDs_h, neckLEDs_s, neckLEDs_v); 
  }

  // Comms LED.
  bool updateCommsLEDs = false;
  
  if ( commsLEDs_h != old_commsLEDs_h )
  {  
    PrintTime();
    Serial.print("Comms LEDs H changed to: ");
    Serial.println(commsLEDs_h);
    old_commsLEDs_h = commsLEDs_h;  
    updateCommsLEDs = true;          
  } 

  if ( commsLEDs_s != old_commsLEDs_s )
  {  
    PrintTime();
    Serial.print("Comms LEDs S changed to: ");
    Serial.println(commsLEDs_s);
    old_commsLEDs_s = commsLEDs_s;            
    updateCommsLEDs = true;
  } 
  
  if ( commsLEDs_v != old_commsLEDs_v )
  {  
    PrintTime();
    Serial.print("Comms LEDs V changed to: ");
    Serial.println(commsLEDs_v);
    old_commsLEDs_v = commsLEDs_v;            
    updateCommsLEDs = true;
  } 
  
  if ( updateCommsLEDs == true )
  {
    commsLEDs.SetHSV(commsLEDs_h, commsLEDs_s, commsLEDs_v);  
    commsLEDsOn = true;    
  }

  // Backpack 1 LEDs.
  bool updateBackpack1LEDs = false;
  
  if ( backpack1LEDs_h != old_backpack1LEDs_h )
  { 
    PrintTime();
    Serial.print("Backpack 1 LEDs H changed to: ");
    Serial.println(backpack1LEDs_h);     
    old_backpack1LEDs_h = backpack1LEDs_h; 
    updateBackpack1LEDs = true;           
  }

  if ( backpack1LEDs_s != old_backpack1LEDs_s )
  { 
    PrintTime();
    Serial.print("Backpack 1 LEDs S changed to: ");
    Serial.println(backpack1LEDs_s);     
    old_backpack1LEDs_s = backpack1LEDs_s; 
    updateBackpack1LEDs = true;           
  }

  if ( backpack1LEDs_v != old_backpack1LEDs_v )
  { 
    PrintTime();
    Serial.print("Backpack 1 LEDs V changed to: ");
    Serial.println(backpack1LEDs_v);     
    old_backpack1LEDs_v = backpack1LEDs_v; 
    updateBackpack1LEDs = true;           
  }

  if ( updateBackpack1LEDs == true )
  { 
    backpack1LEDs.SetHSV(backpack1LEDs_h, backpack1LEDs_s, backpack1LEDs_v);  
  }

  // Backpack 2 LEDs.
  bool updateBackpack2LEDs = false;
  
  if ( (backpack2LEDs_h != old_backpack2LEDs_h) )
  { 
    PrintTime();
    Serial.print("Backpack 2 LEDs H changed to: ");
    Serial.println(backpack2LEDs_h);    
    old_backpack2LEDs_h = backpack2LEDs_h;  
    updateBackpack2LEDs = true;            
  }

  if ( (backpack2LEDs_s != old_backpack2LEDs_s) )
  { 
    PrintTime();
    Serial.print("Backpack 2 LEDs S changed to: ");
    Serial.println(backpack2LEDs_s);    
    old_backpack2LEDs_s = backpack2LEDs_s;  
    updateBackpack2LEDs = true;            
  }
  
  if ( (backpack2LEDs_v != old_backpack2LEDs_v) )
  { 
    PrintTime();
    Serial.print("Backpack 2 LEDs V changed to: ");
    Serial.println(backpack2LEDs_v);    
    old_backpack2LEDs_v = backpack2LEDs_v;  
    updateBackpack2LEDs = true;            
  }
  
  if ( updateBackpack2LEDs == true )
  {
    backpack2LEDs.SetHSV(backpack2LEDs_h, backpack2LEDs_s, backpack2LEDs_v);  
  }
  
  // Exo suit LEDs.
  bool updateExoSuitLEDs = false;
  
  if ( exoLEDs_h != old_exoLEDs_h )
  { 
    PrintTime();
    Serial.print("Exo suit LEDs H changed to: ");
    Serial.println(exoLEDs_h);    
    old_exoLEDs_h = exoLEDs_h;       
    updateExoSuitLEDs = true;
  }

  if ( exoLEDs_s != old_exoLEDs_s )
  { 
    PrintTime();
    Serial.print("Exo suit LEDs S changed to: ");
    Serial.println(exoLEDs_s);    
    old_exoLEDs_s = exoLEDs_s;       
    updateExoSuitLEDs = true;
  }

  if ( exoLEDs_v != old_exoLEDs_v )
  { 
    PrintTime();
    Serial.print("Exo suit LEDs V changed to: ");
    Serial.println(exoLEDs_v);    
    old_exoLEDs_v = exoLEDs_v;       
    updateExoSuitLEDs = true;
  }

  if ( updateExoSuitLEDs == true )
  {
     exoLED1s.SetHSV(exoLEDs_h, exoLEDs_s, exoLEDs_v);  
     exoLED2s.SetHSV(exoLEDs_h, exoLEDs_s, exoLEDs_v);  
  }

  // Spinningcog LEDs.
  bool updateSpinningCogLEDs = false;
  
  if ( spinningCogLEDs_h != old_spinningCogLEDs_h )
  { 
    PrintTime();
    Serial.print("SpinningCog LEDs H changed to: ");
    Serial.println(spinningCogLEDs_h);    
    old_spinningCogLEDs_h = spinningCogLEDs_h;       
    updateSpinningCogLEDs = true;
  }

  if ( spinningCogLEDs_s != old_spinningCogLEDs_s )
  { 
    PrintTime();
    Serial.print("SpinningCog LEDs S changed to: ");
    Serial.println(spinningCogLEDs_s);    
    old_spinningCogLEDs_s = spinningCogLEDs_s;       
    updateSpinningCogLEDs = true;
  }

  if ( spinningCogLEDs_v != old_spinningCogLEDs_v )
  { 
    PrintTime();
    Serial.print("SpinningCog LEDs V changed to: ");
    Serial.println(spinningCogLEDs_v);    
    old_spinningCogLEDs_v = spinningCogLEDs_v;       
    updateSpinningCogLEDs = true;
  }

  if ( spinningCogLEDs_flicker != old_spinningCogLEDs_flicker )
  { 
    PrintTime();
    Serial.print("SpinningCog LEDs flicker changed to: ");
    Serial.println(spinningCogLEDs_flicker);    
    old_spinningCogLEDs_flicker = spinningCogLEDs_flicker;       
    // Map the new time.
    mappedFlickerTime = map( spinningCogLEDs_flicker, 0, 255, minFlickerTime, maxFlickerTime );
    Serial.print("mappedFlickerTime: ");
    Serial.println(mappedFlickerTime);    
  }

  if ( cogSpeed != old_cogSpeed )
  { 
    PrintTime();
    Serial.print("SpinningCog max speed changed to: ");
    Serial.println(cogSpeed);    
    old_cogSpeed = cogSpeed;      
    maxCogSpeed = cogSpeed;
  }
  
  if ( cogJitter != old_cogJitter )
  { 
    PrintTime();     
    Serial.print("SpinningCog max jitter changed to: ");
    Serial.println(cogJitter);    
    old_cogJitter = cogJitter;      
  }
  
  if ( updateSpinningCogLEDs == true )
  {
     spinningCog1LEDs.SetHSV(spinningCogLEDs_h, spinningCogLEDs_s, spinningCogLEDs_v);  
     spinningCog2LEDs.SetHSV(spinningCogLEDs_h, spinningCogLEDs_s, spinningCogLEDs_v);  
  }
}

// ----------------------------------------------------------------------
// Main loop.
// ----------------------------------------------------------------------
void loop()
{    
  // Check if we have any new DMX info.
  if (dmx.newFrame())
  {
    // Loop through all DMX channels.
    for ( int i = 1; i <= 512; i++)
    {
      // Get the right channel by removing the offset.
      int channel = i - DMXChannelOffset;

      // Get the value for the channel.
      uint8_t value = dmx.getDimmer(i);        
      // Serial.print("Channel: ");
      // Serial.print(channel);
      // Serial.print("   Value: ");
      // Serial.println(value);
        
      // Main switch statement to handle DMX value changes based on channel.      
      switch ( channel )
      {      
        // Global brightness.
        case 1:
        {                    
          globalBrightness = value;                
          break;
        } 
  
        // Torch brightness.            
        case 2:
        {
          torchBrightness = value;                 
          break;
        }

        // Fan speed.
        case 3:
        {
          fanSpeed = value;       
          break;
        }
  
        // Camera LED hue.
        case 4:
        {          
          camLED_h = value;  
          break;
        }

        // Camera LED saturation.
        case 5:
        {          
          camLED_s = value;   
          break;
        }

        // Camera LED value.
        case 6:
        {          
          camLED_v = value;    
          break;
        }
           
        // Camera LED flash rate.
        case 7:
        {
          cameraLEDBlinkRate = value;        
          break;
        }

        // Battery level LED hue.
        case 8:
        {
          batteryLevelLED_h = value;
          break;
        }

        // Battery level LED saturation.
        case 9:
        {
          batteryLevelLED_s = value;           
          break;
        }

        // Battery level LED value.
        case 10:
        {
          batteryLevelLED_v = value;
          break;
        }
           
        // Battery level percentage.
        case 11:
        {          
          batteryIndicatorLevel = value;              
          break;
        }

        // Neck LEDs hue.
        case 12:
        {
          neckLEDs_h = value;
          break;
        }

        // Neck LEDs saturation.
        case 13:
        {
          neckLEDs_s = value; 
          if ( neckLEDs_s != old_neckLEDs_s )
          break;
        }

        // Neck LEDs value.
        case 14:
        {
          neckLEDs_v = value;
          break;
        }

        // Comms LEDs hue.
        case 15:
        {
          commsLEDs_h = value;
          break;
        }

        // Comms LEDs saturation.
        case 16:
        {
          commsLEDs_s = value; 
          break;
        }

        // Comms LEDs value.
        case 17:
        {
          commsLEDs_v = value;
          break;
        }

        // Backpack 1 LEDs hue.
        case 18:
        {
          backpack1LEDs_h = value;
          break;
        }

        // Backpack 1 LEDs saturation.
        case 19:
        {
          backpack1LEDs_s = value; 
          break;
        }

        // Backpack 1 LEDs value.
        case 20:
        {
          backpack1LEDs_v = value;
          break;
        }

        // Backpack 2 LEDs hue.
        case 21:
        {
          backpack2LEDs_h = value;
          break;
        }

        // Backpack 2 LEDs saturation.
        case 22:
        {
          backpack2LEDs_s = value;
          break;
        }

        // Backpack 2 LEDs value.
        case 23:
        {
          backpack2LEDs_v = value;
          break;
        }
        
        // Exo suit LEDs hue.
        case 24:
        {
          exoLEDs_h = value;
          break;
        }

        // Exo suit LEDs saturation.
        case 25:
        {
          exoLEDs_s = value; 
          break;
        }

        // Exo suit LEDs value.
        case 26:
        {
          exoLEDs_v = value;
          break;
        }

        // SpinningCog LEDs hue.
        case 27:
        {
          spinningCogLEDs_h = value;
          break;
        }

        // SpinningCog LEDs saturation.
        case 28:
        {
          spinningCogLEDs_s = value; 
          break;
        }

        // SpinningCog LEDs value.
        case 29:
        {
          spinningCogLEDs_v = value;
          break;
        }

        // SpinningCog LEDs flicker.
        case 30:
        {
          spinningCogLEDs_flicker = value;
          break;
        }

        // SpinningCog speed.
        case 31:
        {
          cogSpeed = value;
          break;
        }

        // SpinningCog jitter.
        case 32:
        {
          cogJitter = value;         
          break;
        }
        
        default:
        {
          // Nothing.
          break;
        }
      } // End switch.
    }
    
    // Check for any value changes.
    CheckForChanges();
  }
  
  // Check the flashing camera LED timer. 
  if ( blinkTimer > blinkTime )
  {
    HandleCameraLEDFlashing();  
  }

  // Check the comms switch.
  CheckCommsSwitch();

  // Check the spinning cogs if defined.
  #ifdef SPINNINGCOG
  {
    // Flickering light cog 1.
    if ( flickerTimer1 > flickerTime1 )
    {
      HandleCog1Light();
    }

    // Flickering light cog 2.
    if ( flickerTimer2 > flickerTime2 )
    {
      HandleCog2Light();
    }

    // Cog 1 motor.
    if ( cog1Timer > cog1Time )
    {
      HandleCog1();
    }

    // Cog 2 motor.
    if ( cog2Timer > cog2Time )
    {
      HandleCog2();
    }
  }
  #endif
  
  // Check the battery voltage reading.
  if ( batteryCheckTimer > batteryCheckInterval )
  {
    // Remember the old level.
    batteryLevelEnum oldBatteryLevel = batteryLevel;
    
    // Reset the interval timer. 
    batteryCheckTimer = 0;

    // If we are dead don't bother checking,
    // everything should already be off.
    if ( batteryLevel != BATT_DEAD )
    {
        CheckBatteryLevel();  
    }

    // If the level changed we do something.
    if ( oldBatteryLevel != batteryLevel )
    {     
      oldBatteryLevel = batteryLevel;
       
      // If the battery level is OK we don't blink the LEDs.
      if ( batteryLevel == BATT_OK ) 
      {
        // Stop blinking.
        lowBattBlink = false;                 
        lowBattBlinkOn = true;
        
        // Back to normal battery level.
        Serial.println("Setting battery level back to normal.");
        int percentage = map(batteryIndicatorLevel, 0, 255, 0, 100);      
        batteryLevel1LEDs.SetBargraphUp(percentage);
        batteryLevel2LEDs.SetBargraphDown(percentage);             
      }
      // If the battery level is too low we start blinking all the LEDs.    
      else if ( batteryLevel == BATT_LOW )
      {
        Serial.println("Setting battery level to LOW.");
        // Blink to indicate low voltage.
        lowBattBlink = true;   
      }
      // Else the battery is dead so switch everything off.
      else // if (batteryLevel == BATT_DEAD ) 
      {
        Serial.println("Setting battery level to DEAD.");
        // Torch off.
        TorchOff();
        // Fans off.
        FanOff();
        // Brightness to minimum.
        LEDS.setBrightness(0);
      }
    }
  }

  // Check if we need to blink all LEDs.
  if ( (lowBattBlink == true) && (lowBattBlinkTimer > lowBattBlinkInterval) )
  {
    // Reset the timer.
    lowBattBlinkTimer = 0;

    // Blink all LEDs.
    if ( lowBattBlinkOn == true )
    {
      batteryLevel1LEDs.SetBargraphUp(0);
      batteryLevel2LEDs.SetBargraphDown(0);   
      lowBattBlinkOn = false;
    }
    else
    {
      batteryLevel1LEDs.SetBargraphUp(100);
      batteryLevel2LEDs.SetBargraphDown(100);   
      lowBattBlinkOn = true;
    }      
  }  
}



