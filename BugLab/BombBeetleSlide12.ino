//Author: Simon Jansen

const String version_string = "12";

// DMX library.
#include <DmxSimple.h>

// Debounce library.
#include <Bounce2.h>

// Debouncers.
Bounce bottomTrigger = Bounce();
Bounce middleTrigger = Bounce();
Bounce topTrigger = Bounce();

// DMX output pin.
const int DMXOutputPin = 1;

// Wave trigger output pins.
const int trigger1 = 3;
const int trigger2 = 2;

// Sensor input pins.
const int bottomSensor = 14;
const int middleSensor = 15;
const int topSensor = 16;

bool triggerPrimed = true;
const unsigned long triggerPeriod = 100;    // Period between triggers in mS.

elapsedMillis lastFart = 0;                 // Last fart time.  
const unsigned long fartPeriod = 30000;     // Period between fart sounds in mS.

elapsedMillis lastFlash = 0;                // Last flash time.  
const unsigned long flashDuration = 1000;   // Flash duration in mS.
bool flashOn = false;

// DMX channels.
const int DMXChannel1 = 1;
const int DMXChannel2 = 2;
const int DMXChannel3 = 3;
const int DMXChannel4 = 4;
const int DMXChannel5 = 5;
const int DMXFlashChannel = 6;

// Brightness levels for sequence.
const int brightness1 = 255;
const int brightness2 = 64;
const int brightness3 = 48;

// Constants for the number of lights and sequences.
const int maxLights = 5;
const int maxSteps = 7;

// Brightness levels for each LED at each step.
const int sequenceSteps[maxSteps][maxLights] =  {{ brightness1, 0, 0, 0, 0 },
                                                 { brightness2, brightness1, 0, 0, 0 },
                                                 { brightness3, brightness2, brightness1, 0, 0 },
                                                 { 0, brightness3, brightness2, brightness1, 0 },
                                                 { 0, 0, brightness3, brightness2, brightness1 },
                                                 { 0, 0, 0, brightness3, brightness2 },
                                                 { 0, 0, 0, 0, brightness3} };

int currentStep = 0;                              // Current step.
bool changedStep = false;                         // Flag to say we changed step.
bool runningLightSequence = false;                // Lights running flag.
elapsedMillis lastStep = 0;                       // Last step time.  
const unsigned long stepDuration = 100;           // Step duration in mS.   
elapsedMillis lastSequence = 0;                   // Last sequence time.
const unsigned long sequenceDelay = 2000;         // Deplay between sequences in mS. 
elapsedMillis backgroundSoundTriggered = 0;       // Last time the background sound was triggered.
const unsigned long backgroundSoundTime = 180000; // How long the background sound plays for.
bool backgroundSoundRunning = false;              // Is the background sound playing.                             

// DMX value to send.
int DMXValue = 0;

// Countup timer.
elapsedMillis countUp = 0;
long counter = 0;

// ----------------------------------------------------------------------
// Watchdog timer.
// ----------------------------------------------------------------------
#ifdef __cplusplus
extern "C" 
{
  #endif
  void startup_early_hook() 
  {
    WDOG_TOVALL = 10000; // This is the value that the watchdog timer compares itself to.
    WDOG_TOVALH = 0;
    WDOG_PRESC = 0; // prescaler
    WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN); // Enable WDG
  }
#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------
// WatchdogReset.
// ----------------------------------------------------------------------
void WatchdogReset()
{
  static elapsedMillis watchdogTimer;  
  if (watchdogTimer > 5)
  {
    watchdogTimer = 0;
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
  }
}

// ----------------------------------------------------------------------
// Setup.
// ----------------------------------------------------------------------
void setup()   
{ 
  //Serial.begin(250000);
  
  // Input.
  pinMode(bottomSensor, INPUT_PULLUP);
  pinMode(middleSensor, INPUT_PULLUP);
  pinMode(topSensor, INPUT_PULLUP);

    // Debouncers. 
  bottomTrigger.attach(bottomSensor);
  bottomTrigger.interval(10);
  
  middleTrigger.attach(middleSensor);
  middleTrigger.interval(10);
  
  topTrigger.attach(topSensor);
  topTrigger.interval(10);
  
  // Output.
  pinMode(trigger1, OUTPUT);
  pinMode(trigger2, OUTPUT);

  // Pin used for DMX output.
  DmxSimple.usePin(DMXOutputPin);
  
  // Max number of channels.
  DmxSimple.maxChannel(6);

  // Clear the DMX value.
  DMXValue = 0;
  
  // Ensure triggers are low.
  digitalWrite(trigger1, LOW); 
  digitalWrite(trigger2, LOW); 

  // Output some DMX.
  DMXValue = 0;    
  DmxSimple.write(DMXChannel1, DMXValue);
  DmxSimple.write(DMXChannel2, DMXValue);
  DmxSimple.write(DMXChannel3, DMXValue);
  DmxSimple.write(DMXChannel4, DMXValue);
  DmxSimple.write(DMXChannel5, DMXValue);
  DmxSimple.write(DMXFlashChannel, DMXValue);

  // The flash isn't on.
  flashOn = false;

  // Lights aren't on.  
  currentStep = 0; 
  changedStep = false;                  
  runningLightSequence = false; 

  // Background sound not running.
  backgroundSoundTriggered = 0;
  backgroundSoundRunning = false;

  // Trigger is primed. 
  triggerPrimed = true;   

  // Reset the countUp timer.
  countUp = 0;
  counter = 0;
  
  // Run the start up sequence.
  StartUp();
}

// ----------------------------------------------------------------------
// Startup sequence.
// ----------------------------------------------------------------------
void StartUp()   
{
  Serial.print("Bomb beetle slide. Version: ");   
  Serial.println(version_string);  

  TriggerFlash();
  
  for ( int i = 0; i < maxSteps; i++ )
  { 
    DmxSimple.write( DMXChannel1, sequenceSteps[i][0] );
    DmxSimple.write( DMXChannel2, sequenceSteps[i][1] );
    DmxSimple.write( DMXChannel3, sequenceSteps[i][2] );
    DmxSimple.write( DMXChannel4, sequenceSteps[i][3] );
    DmxSimple.write( DMXChannel5, sequenceSteps[i][4] );  
    delay(200);
  }
  for ( int i = maxSteps - 1; i >= 0; i-- )
  { 
    DmxSimple.write( DMXChannel1, sequenceSteps[i][0] );
    DmxSimple.write( DMXChannel2, sequenceSteps[i][1] );
    DmxSimple.write( DMXChannel3, sequenceSteps[i][2] );
    DmxSimple.write( DMXChannel4, sequenceSteps[i][3] );
    DmxSimple.write( DMXChannel5, sequenceSteps[i][4] );  
    delay(200);
  }
  for ( int i = 0; i < maxSteps; i++ )
  { 
    DmxSimple.write( DMXChannel1, sequenceSteps[i][0] );
    DmxSimple.write( DMXChannel2, sequenceSteps[i][1] );
    DmxSimple.write( DMXChannel3, sequenceSteps[i][2] );
    DmxSimple.write( DMXChannel4, sequenceSteps[i][3] );
    DmxSimple.write( DMXChannel5, sequenceSteps[i][4] );  
    delay(200);
  }
  TriggerFart();
  TriggerGurgle();
}
  
// ----------------------------------------------------------------------
// Trigger light sequence.
// ----------------------------------------------------------------------
void TriggerLightSequence() 
{
  Serial.println("Starting light sequence.");
  
  // Set the running flag.
  runningLightSequence = true;

  // Reset to step 1 to start the sequence.
  currentStep = 1;

  // Set the flag to say we changed steps.
  changedStep = true;

  // Reset the last step time.
  lastStep = 0;

  // Reset the sequence start time.
  lastSequence = 0;
}

// ----------------------------------------------------------------------
// Set light levels based on passed in currentStep.
// Steps index from 1!
// ----------------------------------------------------------------------
void RunLightSequence() 
{ 

  // If we changed steps and the step is valid.
  if ( (changedStep == true ) && ((currentStep > 0) && (currentStep <= maxSteps)) )
  {
    //Serial.print("Doing light sequence step: ");
    //Serial.println(currentStep);
  
    // If so set the lights based on the current step.
    // Steps index from 1 but the array indexes from 0.
    DmxSimple.write( DMXChannel1, sequenceSteps[currentStep - 1][0] );
    DmxSimple.write( DMXChannel2, sequenceSteps[currentStep - 1][1] );
    DmxSimple.write( DMXChannel3, sequenceSteps[currentStep - 1][2] );
    DmxSimple.write( DMXChannel4, sequenceSteps[currentStep - 1][3] );
    DmxSimple.write( DMXChannel5, sequenceSteps[currentStep - 1][4] );  
    //Serial.print("Sending DMX values: ");
    //Serial.print(sequenceSteps[currentStep - 1][0]);
    //Serial.print(", ");
    //Serial.print(sequenceSteps[currentStep - 1][1]);
    //Serial.print(", ");
    //Serial.print(sequenceSteps[currentStep - 1][2]);
    //Serial.print(", ");
    //Serial.print(sequenceSteps[currentStep - 1][3]);
    //Serial.print(", ");
    //Serial.println(sequenceSteps[currentStep - 1][4]);

    // Clear the changed flag.
    changedStep = false;
  }
  
  // If enough time has passed go to the next step.
  if ( lastStep > stepDuration )
  {
    // Increment the current step.
    currentStep++;

    // Set the changed flag.
    changedStep = true;
    
    // Have we done all the steps now?
    if ( currentStep > maxSteps )
    {
      // We've done all the steps.
      // Reset the current step.
      currentStep = 0; 
      
      // Clear the running flag.
      runningLightSequence = false;

      // Clear the changed flag.
      changedStep = false;

      Serial.println("Sequence complete.");
      Serial.println("Sending DMX values: 0, 0, 0, 0, 0");
      
      // Set all lights off.
      DmxSimple.write(DMXChannel1, 0);
      DmxSimple.write(DMXChannel2, 0);
      DmxSimple.write(DMXChannel3, 0);
      DmxSimple.write(DMXChannel4, 0);
      DmxSimple.write(DMXChannel5, 0);

    }
    else
    { 
      // Still have steps to go so reset the timer.
      lastStep = 0;      
    }
  }    
}

// ----------------------------------------------------------------------
// Trigger background gurgle.
// ----------------------------------------------------------------------
void TriggerGurgle() 
{
  Serial.println("Playing gurgle.");
  
  // Turn on sound 1 (gurgle noises).
  digitalWrite(trigger1, LOW);

  // Reset the time.
  backgroundSoundTriggered = 0;

  // Set the flag.
  backgroundSoundRunning = true;
}

// ----------------------------------------------------------------------
// Stop background gurgle.
// ----------------------------------------------------------------------
void CF() 
{
  Serial.println("Stopping gurgle.");
  
  // Turn off sound 1 (gurgle noises).
  digitalWrite(trigger1, HIGH); 

  // Set the flag.
  backgroundSoundRunning = false;
}

// ----------------------------------------------------------------------
// Trigger fart sound.
// ----------------------------------------------------------------------
void TriggerFart() 
{
  Serial.println("Playing fart.");
  
  // Turn on sound 2 (fart noises).
  digitalWrite(trigger2, HIGH); 
  delay(25);
  digitalWrite(trigger2, LOW); 
}

// ----------------------------------------------------------------------
// Trigger flash.
// ----------------------------------------------------------------------
void TriggerFlash() 
{
  Serial.println("Firing flash.");
  
  // Fire the flash.
  DmxSimple.write(DMXFlashChannel, 250);
  
  // Reset the elapsed millis timer.
  lastFlash = 0;
  
  // Set the flag.
  flashOn = true;
}

// ----------------------------------------------------------------------
// Main loop.
// ----------------------------------------------------------------------
void loop() 
{
  //if ( countUp > 1000 )
  //{
  //  Serial.println(++counter);
  //  countUp = 0;
  // }

  // Turn on sound 1 (gurgle noises).
  digitalWrite(trigger1, LOW);
  
  // Kick the dog.
  WatchdogReset();
  
  // Read the sensors.
  bottomTrigger.update();
  middleTrigger.update();
  topTrigger.update();
  
  // Handle running the light sequence.
  if ( runningLightSequence == true )
  { 
    RunLightSequence();
  }
  
  // Should we turn off the flash?
  if ( (flashOn == true) && (lastFlash > flashDuration) )
  {
    // Turn off the flash.
    Serial.println("Turning off flash."); 
    DmxSimple.write(DMXFlashChannel, 0);
    // Reset the flag.
    flashOn = false;
  }
    
  // If the right time has passed and the top is triggered we start the sequence.
  if ( (lastSequence > sequenceDelay) && (topTrigger.read() == 0) )
  {        
    // Top sensor triggered.
    Serial.println("Top trigger reset. Starting sequence.");
    
    // Trigger the light sequence.
    TriggerLightSequence();
  }

   // If the middle sensor is low and we are primed we play the fart noise.
   if ( middleTrigger.rose() )// && (triggerPrimed == true) )
   {
     Serial.print("Middle triggered at: ");
     Serial.println(millis());
               
     // Trigger the playing.
     TriggerFart();
     
     // Set the timers.
     lastFart = 0;
     
     // Say we need a trigger reset.
     triggerPrimed = false;
   }   
  
   // If the bottom sensor went from high to low we flash and reset.
   if ( (bottomTrigger.fell()) && (triggerPrimed == false) )
   {
     Serial.print("Bottom triggered at: ");  
     Serial.println(millis());   

     // Fire the bum flash.
     TriggerFlash();
   
     // Reset the trigger.
     Serial.println("Resetting trigger.");  
     triggerPrimed = true;     
   }

   // Has enough time passed to reset the trigger if not already reset?
   if ( (lastFart > fartPeriod) && (triggerPrimed == false) )
   {
     Serial.print("Resetting trigger due to timeout at: ");
     Serial.println(millis());    
     
     // Reset the trigger.
     triggerPrimed = true;  
     
     // Set the timer.
     lastFart = 0; 
   }
}




