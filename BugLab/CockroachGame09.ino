//Author: Simon Jansen

const String version_string = "9";

// Debounce library.
#include <Bounce2.h>

// LED pin.
const int LED = 1; //PD1

// Sensor inputs.
const int head1GoodInput = 7; // PD7
const int head1BadInput1 = 4; // PD4
const int head1BadInput2 = 2; // PD2
const int head1BadInput3 = 0; // PD0

const int head2GoodInput = A0; // PC0
const int head2BadInput1 = A1; // PC1
const int head2BadInput2 = A2; // PC2
const int head2BadInput3 = A3; // PC3

// Debouncers.
Bounce head1Good =  Bounce();
Bounce head1Bad1 =  Bounce();
Bounce head1Bad2 =  Bounce();
Bounce head1Bad3 =  Bounce();

Bounce head2Good =  Bounce();
Bounce head2Bad1 =  Bounce();
Bounce head2Bad2 =  Bounce();
Bounce head2Bad3 =  Bounce();

const int debounceTime = 25; // Debounce time in mS.

// Trigger outputs.
const int head1BadSound = 10; // PB2
const int head1GoodSound = 12; // PB4

const int head2BadSound = 8; // PB0
const int head2GoodSound = 9; // PB1

// Triger pulse time in mS.
const int soundTriggerTime = 100;  

// LED                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   outputs.
const int head1LED1 = 5; // PD5
const int head1LED2 = 6; // PD6
const int head2LED1 = 3; // PD3
const int head2LED2 = 11; // PB3

// LED states.
int head1LED1State = 0;
int head1LED2State = 0;
int head2LED1State = 0;
int head2LED2State = 0;

// LED colour enumeration.
enum eyeColour {  unknown,
                  red,
                  green,
                  off };

// Sensor states.
bool head1BadState = false;
bool head1GoodState = false;
bool head2BadState = false;
bool head2GoodState = false;

// Delays between playing sounds.
const unsigned long soundResetDelay = 1000;
unsigned long head1LastSoundPlayed = 0;
unsigned long head2LastSoundPlayed = 0;

// Reset flag so we can trigger a new flag.
bool head1TriggerReset = true;
bool head2TriggerReset = true;

// Delays for eye colour.
const unsigned long headGoodFlickerTime = 2000;
unsigned long lastHead1ColourChange = 0;
unsigned long lastHead2ColourChange = 0;
bool head1ColourChanged = false;
bool head2ColourChanged = false;
const unsigned long headBadFlickerTime = 500;
const int headColourFlickerPercentage = 80;
unsigned long lastHead1FlickerStart = 0;
unsigned long lastHead2FlickerStart = 0;
bool head1Flickering = false;
bool head2Flickering = false;
unsigned long flickerTime = 0;

// ----------------------------------------------------------------------
// Flash LED.
// ----------------------------------------------------------------------
void FlashLED(int flashTimes, int delayTime)   
{
  for ( int i = 0; i < flashTimes; i++ )
  {
    digitalWrite (LED, HIGH);
    delay(delayTime);
    digitalWrite (LED, LOW);
    delay(delayTime);
  }
}

// ----------------------------------------------------------------------
// Flash Eyes.
// ----------------------------------------------------------------------
void FlashEyes(int flashTimes, int delayTime)   
{
  for ( int i = 0; i < flashTimes; i++ )
  {
    head1LED1State = 0;
    head1LED2State = 1;
    head2LED1State = 1;
    head2LED2State = 0;
    SetHead1LEDs();
    SetHead2LEDs();
    LEDOn();
    delay(delayTime);    
    head1LED1State = 1;
    head1LED2State = 0;
    head2LED1State = 0;
    head2LED2State = 1;
    SetHead1LEDs();
    SetHead2LEDs();
    LEDOff();
    delay(delayTime);
  }
}

// ----------------------------------------------------------------------
// LED on.
// ----------------------------------------------------------------------
void LEDOn()
{
  digitalWrite (LED, HIGH);
}

// ----------------------------------------------------------------------
// LED off.
// ----------------------------------------------------------------------
void LEDOff()
{
  digitalWrite (LED, LOW);
}

// ----------------------------------------------------------------------
// SetHead1LEDs
// ----------------------------------------------------------------------
void SetHead1LEDs()
{
  digitalWrite(head1LED1, head1LED1State);
  digitalWrite(head1LED2, head1LED2State);
}

// ----------------------------------------------------------------------
// SetHead2LEDs
// ----------------------------------------------------------------------
void SetHead2LEDs()
{
  digitalWrite(head2LED1, head2LED1State);
  digitalWrite(head2LED2, head2LED2State);
}

  // ----------------------------------------------------------------------
// SetHead1LEDsOff
// ----------------------------------------------------------------------
void SetHead1LEDsOff()
{
  digitalWrite(head1LED1, 0);
  digitalWrite(head1LED2, 0);
}

// ----------------------------------------------------------------------
// SetHead2LEDsOff
// ----------------------------------------------------------------------
void SetHead2LEDsOff()
{
  digitalWrite(head2LED1, 0);
  digitalWrite(head2LED2, 0);
}

// ----------------------------------------------------------------------
// Setup.
// ----------------------------------------------------------------------
void setup()   
{
  
  // Set inputs.
  pinMode(head1GoodInput, INPUT_PULLUP);
  pinMode(head1BadInput1, INPUT_PULLUP);
  pinMode(head1BadInput2, INPUT_PULLUP);
  pinMode(head1BadInput3, INPUT_PULLUP);
  
  pinMode(head2GoodInput, INPUT_PULLUP);
  pinMode(head2BadInput1, INPUT_PULLUP);
  pinMode(head2BadInput2, INPUT_PULLUP);
  pinMode(head2BadInput3, INPUT_PULLUP);

    // Debouncers. 
  head1Good.attach(head1GoodInput);
  head1Good.interval(debounceTime);
  head1Bad1.attach(head1BadInput1);
  head1Bad1.interval(debounceTime);
  head1Bad2.attach(head1BadInput2);
  head1Bad2.interval(debounceTime);
  head1Bad3.attach(head1BadInput3);
  head1Bad3.interval(debounceTime);
  
  head2Good.attach(head2GoodInput);
  head2Good.interval(debounceTime);
  head2Bad1.attach(head2BadInput1);
  head2Bad1.interval(debounceTime);
  head2Bad2.attach(head2BadInput2);
  head2Bad2.interval(debounceTime);
  head2Bad3.attach(head2BadInput3);
  head2Bad3.interval(debounceTime);
  
  // Set outputs.
  pinMode(LED, OUTPUT);
  pinMode(head1BadSound, OUTPUT);
  pinMode(head1GoodSound, OUTPUT);
  pinMode(head2BadSound, OUTPUT);
  pinMode(head2GoodSound, OUTPUT);
  pinMode(head1LED1, OUTPUT);
  pinMode(head1LED2, OUTPUT);
  pinMode(head2LED1, OUTPUT);
  pinMode(head2LED2, OUTPUT);
 
  digitalWrite(LED, LOW);  
  digitalWrite(head1BadSound, LOW);
  digitalWrite(head1GoodSound, LOW);
  digitalWrite(head2BadSound, LOW);
  digitalWrite(head2GoodSound, LOW);

  // Set the LED initial states. 
  head1LED1State = 0;
  head1LED2State = 0;
  head2LED1State = 0;
  head2LED2State = 0;

  // Set the actual LED outputs.
  SetHead1LEDs();
  SetHead2LEDs();

  // Flash the eyes to show we are started.
  FlashEyes(5, 250);

  // Set the actual LED outputs.
  SetHead1LEDs();
  SetHead2LEDs();

  // Set the states.
  head1BadState = false;
  head1GoodState = false;
  head2BadState = false;
  head2GoodState = false;

  // Reset the sound trigger time.
  head1LastSoundPlayed = 0;
  head2LastSoundPlayed = 0;

  // Set up eye colours.
  Head1Colour(green);
  Head2Colour(green);

  // Reset the eye colour change time.
  lastHead1ColourChange = 0;
  lastHead2ColourChange = 0;
  head1ColourChanged = false;
  head2ColourChanged = false;

  // Reset the flicker times.
  lastHead1FlickerStart = 0;
  lastHead2FlickerStart = 0;
  head1Flickering = false;
  head2Flickering = false;
}

// ----------------------------------------------------------------------
// Play the good sound for head 1.
// ----------------------------------------------------------------------
void PlayHead1GoodSound()
{
   // Reset the time.
   head1LastSoundPlayed = millis();
   digitalWrite(head1GoodSound, HIGH);
   delay(soundTriggerTime);
   digitalWrite(head1GoodSound, LOW);
}

// ----------------------------------------------------------------------
// Play the bad sound for head 1.
// ----------------------------------------------------------------------
void PlayHead1BadSound()
{  
   // Reset the time.
   head1LastSoundPlayed = millis();
   digitalWrite(head1BadSound, HIGH);
   delay(soundTriggerTime);
   digitalWrite(head1BadSound, LOW);
}

// ----------------------------------------------------------------------
// Play the good sound for head 2.
// ----------------------------------------------------------------------
void PlayHead2GoodSound()
{
   // Reset the time.
   head2LastSoundPlayed = millis();
   digitalWrite(head2GoodSound, HIGH);
   delay(soundTriggerTime);
   digitalWrite(head2GoodSound, LOW);
}

// ----------------------------------------------------------------------
// Play the bad sound for head 2.
// ----------------------------------------------------------------------
void PlayHead2BadSound()
{  
   // Reset the time.
   head2LastSoundPlayed = millis();
   digitalWrite(head2BadSound, HIGH);
   delay(soundTriggerTime);
   digitalWrite(head2BadSound, LOW);
}

// ----------------------------------------------------------------------
// Read head one sensors.
// ----------------------------------------------------------------------
void ReadHead1() 
{
   // Read all the debouncers.
   head1Good.update();
   head1Bad1.update();
   head1Bad2.update();
   head1Bad3.update();
   
   // Triggered when low.
   if ( head1Good.fell() )
   {
      head1GoodState = true;
   }
   else
   {
      head1GoodState = false;
   }

   // Triggered when low.
   if ( (head1Bad1.fell()) || (head1Bad2.fell()) || (head1Bad3.fell()) )
   {
      head1BadState = true;
   }
   else
   {
      head1BadState = false;
   }
   
   // Set the reset trigger appropriately.
   // When no sensors are triggered we can reset.
   if ( (head1GoodState == false) && (head1BadState == false) )
   {
      // No sensors triggered so we are reset.
      head1TriggerReset = true;
   }   
}

// ----------------------------------------------------------------------
// Read head two sensors.
// ----------------------------------------------------------------------
void ReadHead2() 
{    
   // Read all the debouncers.
   head2Good.update();
   head2Bad1.update();
   head2Bad2.update();
   head2Bad3.update();
   
   // Triggered when low.
   if ( head2Good.fell() )
   {
      head2GoodState = true;      
   }
   else
   {
      head2GoodState = false;
   }

   // Triggered when low.
   if ( (head2Bad1.fell()) || (head2Bad2.fell()) || (head2Bad3.fell()) )
   {
      head2BadState = true;
   }
   else
   {
      head2BadState = false;
   }
   // Set the reset trigger appropriately.
   // When no sensors are triggered we can reset.
   if ( (head2GoodState == false) && (head2BadState == false) )
   {
      // No sensors triggered so we are reset.
      head2TriggerReset = true;
   }   
}

// ----------------------------------------------------------------------
// Head one eye colour.
// ----------------------------------------------------------------------
void Head1Colour(eyeColour colour) 
{
    // Change colour based on input.
    if ( colour == green )
    {
      head1LED1State = 1;
      head1LED2State = 0;
    }
    else if ( colour == red )
    {
      head1LED1State = 0;
      head1LED2State = 1;
    }
    else
    {
      head1LED1State = 0;
      head1LED2State = 0;
    }
    // Set the LEDs.
    SetHead1LEDs();
    
    // Set the timer.
    lastHead1ColourChange = millis();
    
    // And the flag.
    head1ColourChanged = true;
}

// ----------------------------------------------------------------------
// Head1Flicker
// ----------------------------------------------------------------------
void Head1Flicker()   
{
  // Get a random number.
  int randomNumber = random( 1, 101 );
  
  // Compare to our random factor amount.
  if ( randomNumber <= headColourFlickerPercentage )
  {       
    // Set the LEDs to off.
    SetHead1LEDsOff();
  }
  else
  {
    SetHead1LEDs();    
  }

  // Do we reset the flickering flag for head 1.
  if ( (millis() - lastHead1FlickerStart) > flickerTime )
  {
    SetHead1LEDs();
    head1Flickering = false;
    LEDOff();
  }
}

// ----------------------------------------------------------------------
// Head2Flicker
// ----------------------------------------------------------------------
void Head2Flicker()   
{
  // Get a random number.
  int randomNumber = random( 1, 101 );
  
  // Compare to our random factor amount.
  if ( randomNumber <= headColourFlickerPercentage )
  {       
    // Set the LEDs to off.
    SetHead2LEDsOff();    
  }
  else
  {
    SetHead2LEDs();    
  }
  
  // Do we reset the flickering flag for head 2.
  if ( (millis() - lastHead2FlickerStart) > flickerTime )
  {
    SetHead2LEDs();
    head2Flickering = false;
    LEDOff();
  }
}

// ----------------------------------------------------------------------
// Head two eye colour.
// ----------------------------------------------------------------------
void Head2Colour(eyeColour colour) 
{
    // Change colour based on input.
    if ( colour == green )
    {
      head2LED1State = 1;
      head2LED2State = 0;
    }
    else if ( colour == red )
    {
      head2LED1State = 0;
      head2LED2State = 1;
    }
    else
    {
      head2LED1State = 0;
      head2LED2State = 0;
    }
    // Set the LEDs.
    SetHead2LEDs();
    
    // Set the timer.
    lastHead2ColourChange = millis();
    
    // And the flag.
    head2ColourChanged = true;
}

// ----------------------------------------------------------------------
// Main loop.
// ----------------------------------------------------------------------
void loop() 
{  
   // Read the first head.
   ReadHead1();

   // Based on the result play the appropriate sounds.
   // Was the good sensor triggered.
   if ( head1GoodState == true )
   {
     // Play the good sound.
     // If enough time has passed and the trigger is reset.
     if ( head1TriggerReset && ((millis() - head1LastSoundPlayed) > soundResetDelay) )
     { 
       // We've been triggered.
       head1TriggerReset = false;
       PlayHead1GoodSound();
       Head1Colour(red);
       // Flicker the LEDs.
       head1Flickering = true;
       flickerTime = headGoodFlickerTime;
       lastHead1FlickerStart = millis();
     }
   }
   // Were any of the bad sensors triggered?
   if ( head1BadState == true )
   {
     // Play the bad sound.
     // If enough time has passed and the trigger is reset.
     if ( head1TriggerReset && ((millis() - head1LastSoundPlayed) > soundResetDelay) )
     { 
       // We've been triggered.
       head1TriggerReset = false;
       PlayHead1BadSound();
       // Flicker the LEDs.
       head1Flickering = true; 
       flickerTime = headBadFlickerTime;
       lastHead1FlickerStart = millis();
     }
   } 

   // Do we reset the eye colour if it had changed for head 1.
   if ( (head1ColourChanged == true) && (millis() - lastHead1ColourChange > headGoodFlickerTime) )
   {
     Head1Colour(green);
     head1ColourChanged = false;
   }

   // Handle head 1 flickering.
   if ( head1Flickering == true )
   {      
      Head1Flicker();
      LEDOn();
   }
     
   // Read the second head.
   ReadHead2();

   // Based on the result play the appropriate sounds.
   // Was the good sensor triggered.
   if ( head2GoodState == true )
   {   
     // Play the good sound.
     // If enough time has passed and the trigger is reset.
     if ( head2TriggerReset && ((millis() - head2LastSoundPlayed) > soundResetDelay) )
     { 
      
       // We've been triggered.
       head2TriggerReset = false;       
       PlayHead2GoodSound();
       Head2Colour(red);
       // Flicker the LEDs.
       head2Flickering = true;
       flickerTime = headGoodFlickerTime;
       lastHead2FlickerStart = millis();
     }
   }
   // Were any of the bad sensors triggered?
   if ( head2BadState == true )
   {
     // Play the bad sound.
     // If enough time has passed and the trigger is reset.
     if ( head2TriggerReset && ((millis() - head2LastSoundPlayed) > soundResetDelay) )
     { 
       // We've been triggered.
       head2TriggerReset = false;       
       PlayHead2BadSound();
       // Flicker the LEDs.
       head2Flickering = true;
       flickerTime = headBadFlickerTime;
       lastHead2FlickerStart = millis();
     }
   } 

   // Do we reset the eye colour if it had changed for head 2.
   if ( (head2ColourChanged == true) && (millis() - lastHead2ColourChange > headGoodFlickerTime) )
   {
     Head2Colour(green);
     head2ColourChanged = false;
   }

   // Handle head 2 flickering.
   if ( head2Flickering == true )
   {      
      Head2Flicker();
      LEDOn();
   }
}




