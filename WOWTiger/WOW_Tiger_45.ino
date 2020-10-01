// Author: Simon Jansen 2016

/*

  Usage instructions.

  To record a file or pattern:
  ----------------------------

  Record the longest file first. Then later add additional channels to it.
  For example, if it is a talking/singing puppet record the jaw movement file first
  as this will be the longest one and all other movements will be synced to it.

  record <filename> <channel1> <channel2> <channel3> <channel4> <p>

  The filename is the name to save the file under. This is in 8.3 format and can just
  be a simple file name, e.g. "1", "A", "file1.txt", etc.

  The channels map to the RC input channels. These are defined below as rc_1_pin,
  rc_2_pin, rc_3_pin and rc_4_pin. These pins can map to any channel on the RC receiver.
  Each channel argument is to specify which servo you wish that channel to map to for
  this recording. Servos are mapped from 0 - (numberOfChannels - 1).

  If we wish to make a recording where we want rc_1_pin to map to servo 1, rc_2_pin to
  map to servo 2, rc_3_pin to map to servo 6 and rc_4_pin to map to channel 7 saved
  into a file called "1" you use
  the command:

  record 1 1 2 6 7

  If you only wish to record 2 channels, say rc_2_pin to servo 2 and rc_4_pin to servo 7
  into a file called "2" you can use the command:

  record 2 -1 2 -1 7

  Channels that have a -1 specified will not be written. Any channel not being recorded will
  to a value of 1500, the neutral servo position.

  When making a recording the file specified is created new each time. So an existing file
  will be deleted and recreated.

  Recording patterns
  ------------------

  The optional p is if you wish to record a pattern. A pattern is a small file that is later mapped
  to rc_1_pin to allow triggering of complex movements. Patterns can be up to 15 seconds long
  (defined by patternSeconds in the code). This size is dependent on how much memory is free.

  The difference between a pattern file and a normal recording is that instead of 1500 values
  being written into the file -1 is used.

  When a pattern is set up (as described below) the appropriate trigger on rc_1_pin will cause
  the pattern file to be played and inserted into the current recording. The -1 values are so
  the software knows not to overwrite those channels.

  An example of pattern use might be for doing a blink. The blink might be controlled by 4 servos,
  10, 11, 12 and 13 say. The blink movement is recorded with the following command:

  record blink 10 11 12 13 p

  The pattern recording can be over 15 seconds long but only a maximum of 15 seconds will be
  inserted into a recording. If less than 15 seconds then only the actual recorded pattern is
  inserted.

  You can record a pattern with another pattern enabled but only one actual pattern can be active
  at a given time.

  Enabling a pattern.
  -------------------

  Once a pattern file is recorded it needs to be setup to be triggered by rc_1_pin.

  The command is:

  pattern <filename> < + or - > <trigger>

  The filename is the recorded pattern file.
  The + or - is a modifier on the trigger, - means less than the trigger and + means greater than.
  The trigger is the RC value to trigger at defined in the code as between minRC and maxRC.

  So to trigger the blink pattern to start playing when rc_1_pin goes less than 1000 you would
  use the command:

  pattern blink - 1000

  Or if you want it to trigger when greater than 2000 you use:

  pattern blink + 2000

  Once a pattern is enabled (only one can be enabled at a time) any time you are recording if
  the trigger happens then the pattern will be inserted into the recording.

  The pattern is only triggered once so the whole pattern will play before being retriggered. If
  the trigger condition is still met when the pattern finishes playing it will simply retrigger.
  This lets you have repeating patterns by simply leaving the trigger enabled.

  To clear a pattern (for when you want rc_1_pin mapped to any channel again) use the command
  with no arguments:

  pattern

  Adding files.
  --------------

  To add to recording to add more channels you use the add command. This behaves exactly like the record
  command except you specify a source and a destination file. The source file is played (in real time)
  and simultaneously recorded to the new destination file with the specified channels being added in real time.

  So to add channels 10, 11, 12, 13 say to file "1" recorded above (to channels 1, 2, 6 and 7) and
  save in a new file called "2" you use the command:

  add 1 2 10 11 12 13

  If you are adding and only wish to add 2 channel use -1 as the channel as for recording. So to add only
  channels 10 and 12 in the example use the command:

  add 1 2 10 -1 12 -1

  Adding will overwrite the destination file.

  When adding you can use patterns in the same way as for recording.

  Twerking files.
  --------------

  Twerking allows you to play back a file and massage the settings in it using the sticks. This behaves like 
  the add function except the sticks work in an add/subtract mode. The source file is played (in real time)
  and simultaneously recorded to the new destination file with the specified channels being added in real time.

  The sticks are used to add or subtract from existing movements based on how far the stick is moved from the 
  neutral, middle position. With the stick in the mid position nothing is added to the existing recorded 
  movement. If the stick is pushed forward movement is subtracted from the existing movement. If moved backwards
  movement is added to the existing movement.

  If you twerk a file and don't touch the sticks the resultant file will be the same as the original file. 

  So to twerk channel 10 using RC stick 2 in the file called "1" you use the command.

  twerk 1 2 -1 10 -1 -1

  Twerking will overwrite the destination file.

  When twerking you can use patterns in the same way as for recording.

  Playing files.
  --------------

  To play a recorded file use the command:

  play 1

  You can also play pattern files. When playing these the -1 values will be converted to 1500 (servo neutral)
  when playing as -1 is an invalid servo position.

*/

// SD Card libraries.
#include <SPI.h>
#include <SD.h>

// Debounce library.
#include <Bounce2.h>

const int versionNumber = 45;

// Hardware pin constants.

// Midi receiver.
// Using RX2 - Pin 9.

// SSC32 serial.
// Using TX3 - Pin 8.

// SD card pins.
// CLK - 13
// D0 - 12
// D1 - 11
// CS - 14
// Chip select pin for SD card.
const int chipSelect = 14;

// Input buttons.
const int startButton = 3;
const int testButton = 4;
const int stopButton =  5;

// RC input pins.
const int rc_1_pin = 24;
const int rc_2_pin = 33;
const int rc_3_pin = 25;
const int rc_4_pin = 32;

// Output pins.
const int LEDPin = 6;
const int scopePin = 18;

// Recording/Playback interval in mS.
const unsigned long recPlayInterval = 50;

// Baud rates.
const int servo_driver_baud = 115200;
const int midi_baud = 115200;

// Timers.
elapsedMillis recPlayElapsed = 0;
elapsedMillis recPlayTotalTime = 0;
unsigned long nextInterval = 0;
elapsedMillis writeTimer;

// Constants for SSC32.
const int multipleForSSC32Send = 1;
int counterForSSC32Send = 1;

// state enumeration.
enum stateEnum {
  idling,
  play,
  play_wait,
  record,
  record_wait,
  add,
  add_wait,
  twerk,
  twerk_wait,
  list,
  deleteFileConfirm,
};

// Test mode.
bool testMode = false;

// Time mode.
bool timeMode = false;

// Performance mode.
bool performanceMode = false;

// Initial mode variable.
stateEnum state = idling;


// Files.
File playFile;
File recordFile;
File addFile;
File patternFile;
String fileToPlay;
String fileToRecord;
String fileToAdd;
String fileToDelete;

// Variables file handling.
const int numberOfFiles = 6;
// Array of active files.
bool fileActive[numberOfFiles];
int fileCounter = 0;

// Data string written to file.
String dataString = "";

// Channels being edited.
int channel1Edit = -1;
int channel2Edit = -1;
int channel3Edit = -1;
int channel4Edit = -1;

// Debouncers.
Bounce debouncerStop = Bounce();
Bounce debouncerTest = Bounce();
Bounce debouncerStart = Bounce();

// Max and minimum analog output values in milliVolts.
//const int minAnalogValue = 0;
//const int maxAnalogValue = 4095;
const int minAnalogVoltage = 595;
const int maxAnalogVoltage = 2600;
    
// RC values.
//const int minRC = 950;                  // Theoretical limits.
//const int maxRC = 2100;
const int minRC = 930;                    // Values to make it work.
const int maxRC = 2120;
const int servoNeutral = 1500;            // Standard servo neutral.
const int dynamixelNeutral = minRC;       // Dynamixel servo neutral.

// Variables for RC receiver.
volatile bool gotRCValue;

// Constants/variables for RC control.
volatile uint16_t channel_1;
volatile uint16_t channel_1_start;
volatile uint16_t channel_2;
volatile uint16_t channel_2_start;
volatile uint16_t channel_3;
volatile uint16_t channel_3_start;
volatile uint16_t channel_4;
volatile uint16_t channel_4_start;

const int numberOfCommandEntries = 33;            // Number of command tokens.
bool commandReceived = false;
const int commandBufferSize = 64;                 // Maximum command size.
int commandBufferIndex = 0;                       // Index into command buffer.
char commandBuffer[commandBufferSize];            // Buffer for incoming command.
const char commandDelimiter[2] = " ";             // Delimiting character.
char* commandTokenArray[numberOfCommandEntries];  // Array of pointers to command tokens.

const int timeCodeBufferSize = 32;                 // Maximum timecode size.
int timeCodeBufferIndex = 0;                       // Index into time code buffer.
char timeCodeBuffer[timeCodeBufferSize];           // Buffer for incoming time code.

// Array of channels and position values.
const int numberOfChannels = 32;
int channelPositions[numberOfChannels];
bool channelEnabled[numberOfChannels];

// Buffer for lines read from play file.
const int lineBufferSize = 256;         // Line buffer size.
char lineBuffer[lineBufferSize];        // Buffer for lines from playback file.
const char delimiter[2] = ",";          // Delimiting character.
char* tokenArray[numberOfChannels + 1];

// Lines written.
unsigned long linesWritten = 0;
unsigned long linesPlayed = 0;

// Midi decode variables.
enum { F24 = 0, F25 = 2, F30DF = 4, F30 = 6 }; // Frame types.

int h = 0;    // Hours.
int m = 0;    // Minutes.
int s = 0;    // Seconds.
int f = 0;    // Frames.

byte frameType;       // speed of MTC : 24fps / 25fps / 30 drop frame / 30 fps

const int midiBufferSize = 64;
byte buf[midiBufferSize];             // Serial Midi buffer.
const int midiTCBufferSize = 20;
byte tc[midiTCBufferSize];            // Array to old midi timecode.

// Array for pattern storage.
// It has to be x minutes long and contain the number of channels.
const int patternSeconds = 15;
const int numPatternRows = (1000 / recPlayInterval) * patternSeconds;

// Boolean for pattern recording.
bool recordingPattern = false;

// Pattern array.
int pattern[numberOfChannels][numPatternRows];
int patternLowerLimit = 0;
int patternUpperLimit = 0;
bool patternEnabled = false;
bool patternPlaying = false;
int patternCurrentLine = 0;

// LED flashing handler.
bool LEDLit = false;
bool LEDFlash = false;
unsigned long LEDTime = 500;
elapsedMillis LEDTimer = 0;

// Start time.
int start_h = 0;
int start_m = 0;
int start_s = 0;
int start_f = 0;

// Track 1 start time.
int track1_h = 1;
int track1_m = 0;
int track1_s = 0;
int track1_f = 0;

// Track 2 start time.
int track2_h = 3;
int track2_m = 0;
int track2_s = 0;
int track2_f = 0;

// Track 3 start time.
int track3_h = 5;
int track3_m = 0;
int track3_s = 0;
int track3_f = 0;

// Track 4 start time.
int track4_h = 7;
int track4_m = 0;
int track4_s = 0;
int track4_f = 0;

// Track 5 start time.
int track5_h = -1;
int track5_m = 0;
int track5_s = 0;
int track5_f = 0;

// Track 6 start time.
int track6_h = -1;
int track6_m = 0;
int track6_s = 0;
int track6_f = 0;

// Flag to show a time triggered play is happening
// so we don't retrigger it continually.
bool file1Playing = false;
bool file2Playing = false;
bool file3Playing = false;
bool file4Playing = false;
bool file5Playing = false;
bool file6Playing = false;
bool timedFilePlaying = false;

// Last time update received.
elapsedMillis lastTimeUpdate = 0;
elapsedMillis triggered = 0;
const long lastTimePeriod = 500;
int timeCodeCount = 0;
const int ignoreTimeCodeCount = 3;

// Printing dots while playing.
int dotCounter = 0;
const int numberOfDots = 80;

////////////////////////////////////////////////////////////
// Setup function.
////////////////////////////////////////////////////////////
void setup()
{
  // Serial for the USB debugging.
  Serial.begin(9600);
  Serial.print("Started. Version: ");
  Serial.println(versionNumber);

  // Serial for MIDI in.
  Serial2.begin(midi_baud); // For SMPTE receiver.
  //Serial2.begin(midi_baud, SERIAL_8N1_RXINV); // For MIDI receiver.

  // Serial for servo driver out.
  Serial3.begin(servo_driver_baud);

  // Set up RC receiver input pins.
  pinMode(rc_1_pin, INPUT);
  pinMode(rc_2_pin, INPUT);
  pinMode(rc_3_pin, INPUT);
  pinMode(rc_4_pin, INPUT);

  // Output pins.
  pinMode(scopePin, OUTPUT);
  digitalWrite(scopePin, HIGH);
  pinMode(LEDPin, OUTPUT);
  LEDOff();

  // Analog output pin 14.
  // 12 bit resolution 0 - 4095.
  analogWriteResolution(12);

  // Button pins.
  pinMode(startButton, INPUT_PULLUP);
  pinMode(testButton, INPUT_PULLUP);
  pinMode(stopButton, INPUT_PULLUP);

  // Debouncers.
  debouncerTest.attach(testButton);
  debouncerTest.interval(25);
  debouncerStart.attach(startButton);
  debouncerStart.interval(25);
  debouncerStop.attach(stopButton);
  debouncerStop.interval(25);

  // Attach an interrupt handlers to be called whenever
  // the RC pin changes from LOW to HIGH or vice versa.
  attachInterrupt(rc_1_pin, RCchannel_1, CHANGE);
  attachInterrupt(rc_2_pin, RCchannel_2, CHANGE);
  attachInterrupt(rc_3_pin, RCchannel_3, CHANGE);
  attachInterrupt(rc_4_pin, RCchannel_4, CHANGE);

  // Scope pin low.
  digitalWrite(scopePin, LOW);

  // Seed the random number generator.
  randomSeed(analogRead(0));

  // Initialise the SD card.
  SDInit();

  // Initialise the serial command buffer.
  commandReceived = false;
  commandBufferIndex = 0;
  commandBuffer[0] = 0;

  // Initialise the channel buffer.
  for (int i = 0; i < numberOfChannels; i++)
  {
    channelPositions[i] = 0;
    channelEnabled[i] = true;
  }

  // Initialise time code buffer.
  for (int i = 0; i < timeCodeBufferSize; i++)
  {
    timeCodeBuffer[i] = 0;
  }

  // Initialise the midi buffers.
  for (int i = 0; i < midiBufferSize; i++)
  {
    buf[i] = 0;
  }
  for (int i = 0; i < midiTCBufferSize; i++)
  {  
    tc[i] = 0;
  }
  
  delay(3000);

  // Test mode.
  testMode = false;

  // Time mode.
  timeMode = true;

  // Performance mode.
  performanceMode = true;

  // Initial mode variable.
  state = idling;

  // Start time.
  start_h = 0;
  start_m = 0;
  start_s = 0;
  start_f = 0;

  // Enumerate all the files.
  InitialiseFiles();
  Serial.println("\nType \"help\" for a list of commands.");

  // Output help information.
  Help();

  // Show status information.
  ShowStatus();

  if ( performanceMode == true )
  {
    Serial.println("Starting performance mode. Waiting for time codes...");
  }

  // Call stop to make sure we are in a good state.
  Stop();
}

////////////////////////////////////////////////////////////
// Initialise what files we have.
// The root directory is checked for files with numbers
// as their file names. These are put into the array of
// files available for playback.
////////////////////////////////////////////////////////////
void InitialiseFiles (void)
{
  Serial.println("Initialising files for playback.");

  // Setup the active files array.
  for ( int i = 0; i < numberOfFiles; i++ )
  {
    // All files inactive.
    fileActive[i] = false;
  }

  File root = SD.open("/");
  fileCounter++;

  // Loop until there are no more files.
  while (true)
  {
    // Get the next file.
    File entry = root.openNextFile();
    fileCounter++;
    if (!entry)
    {
      // Close this file.
      FileClose(entry);
      // No more files.
      break;
    }

    // Is it a file?
    if (!(entry.isDirectory()))
    {
      // We look at the name, convert to a number then
      // set the flag in our array of playable files.
      int fileNumber = atoi(entry.name());
      // Is it in range?
      if ( (fileNumber > 0 ) && (fileNumber <= numberOfFiles) )
      {
        // Store it in the array.
        fileActive[fileNumber - 1] = true;
        Serial.print("Found file: ");
        Serial.print(fileNumber);
        Serial.println(". Setting as active.");
      }
    }

    // Close this file.
    FileClose(entry);
  }

  // Close the root directory.
  FileClose(root);
}

////////////////////////////////////////////////////////////
// Show the minutes and seconds.
////////////////////////////////////////////////////////////
void ShowMinutesSeconds(int totalMilliSeconds)
{  
  int totalSeconds = totalMilliSeconds / 1000;
  int minutes = totalSeconds / 60; 
  int seconds = totalSeconds % 60;
  int milliSeconds = totalMilliSeconds - ( totalSeconds * 1000);
  Serial.print(minutes);
  Serial.print("m ");
  Serial.print(seconds);
  Serial.print(".");
  Serial.print(milliSeconds);
  Serial.println("s");
}
                      
////////////////////////////////////////////////////////////
// LED flash on.
////////////////////////////////////////////////////////////
void LEDFlashOn(int flashTime)
{  
  LEDTime = flashTime;
  LEDFlash = true;
  LEDTimer = 0;
}

////////////////////////////////////////////////////////////
// LED flash.
////////////////////////////////////////////////////////////
void LEDFlasher()
{
  if ( LEDTimer > LEDTime )
  {
    if ( LEDLit )
    {
      digitalWrite(LEDPin, LOW);
      LEDLit = false;      
    }
    else
    {
      digitalWrite(LEDPin, HIGH);
      LEDLit = true;
    }
    // Reset the timer.
    LEDTimer = 0;
  }
}

////////////////////////////////////////////////////////////
// LED on.
////////////////////////////////////////////////////////////
void LEDOn()
{
  digitalWrite(LEDPin, HIGH);
  LEDLit = true;
  LEDFlash = false;
}

////////////////////////////////////////////////////////////
// LED off.
////////////////////////////////////////////////////////////
void LEDOff()
{
  digitalWrite(LEDPin, LOW);
  LEDLit = false;
  LEDFlash = false;
}

////////////////////////////////////////////////////////////
// RC channel interrupt handler routines.
////////////////////////////////////////////////////////////
void RCchannel_1()
{
  // If the pin is HIGH, start a timer.
  if ( digitalRead(rc_1_pin) == HIGH )
  {
    channel_1_start = micros();
  }
  else
  {
    // The pin is now LOW so output the difference
    // between when the timer was started and now.
    channel_1 = (uint16_t) (micros() - channel_1_start);
    //Serial.println(channel);
    gotRCValue = true;

    // Set the pattern trigger if the pattern is enabled and we are recording or adding.
    if ( patternEnabled && ((state == record) || (state == add)) )
    {
      // If we are looking for a less than trigger.
      if ( patternUpperLimit == 0 )
      {
        // If we aren't already playing and we see the trigger value.
        if ( (patternPlaying == false) && (channel_1 < patternLowerLimit ) )
        {
          // Set playing to true and reset the counter.
          patternPlaying = true;
          patternCurrentLine = 0;
          Serial.println("Pattern triggered.");
        }
      }
      // Else if we are looking for a greater than trigger.
      else if ( patternLowerLimit == 0 )
      {
        // If we aren't already playing and we see the trigger value.
        if ( (patternPlaying == false) && (channel_1 > patternUpperLimit) )
        {
          // Set playing to true and reset the counter.
          patternPlaying = true;
          patternCurrentLine = 0;
          Serial.println("Pattern triggered.");
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////
// RC channel interrupt handler routines.
////////////////////////////////////////////////////////////
void RCchannel_2()
{
  // If the pin is HIGH, start a timer.
  if ( digitalRead(rc_2_pin) == HIGH )
  {
    channel_2_start = micros();
  }
  else
  {
    // The pin is now LOW so output the difference
    // between when the timer was started and now.
    channel_2 = (uint16_t) (micros() - channel_2_start);
    //Serial.println(channel);
    gotRCValue = true;
  }
}

////////////////////////////////////////////////////////////
// RC channel interrupt handler routines.
////////////////////////////////////////////////////////////
void RCchannel_3()
{
  // If the pin is HIGH, start a timer.
  if ( digitalRead(rc_3_pin) == HIGH )
  {
    channel_3_start = micros();
  }
  else
  {
    // The pin is now LOW so output the difference
    // between when the timer was started and now.
    channel_3 = (uint16_t) (micros() - channel_3_start);
    //Serial.println(channel);
    gotRCValue = true;
  }
}

////////////////////////////////////////////////////////////
// RC channel interrupt handler routines.
////////////////////////////////////////////////////////////
void RCchannel_4()
{
  // If the pin is HIGH, start a timer.
  if ( digitalRead(rc_4_pin) == HIGH )
  {
    channel_4_start = micros();
  }
  else
  {
    // The pin is now LOW so output the difference
    // between when the timer was started and now.
    channel_4 = (uint16_t) (micros() - channel_4_start);
    //Serial.println(channel);
    gotRCValue = true;
  }
}

////////////////////////////////////////////////////////////
// Called when we get an RC value update.
////////////////////////////////////////////////////////////
void HandleRCCommand()
{
  // Loop through all channels and store the positions.
  for ( int i = 0; i < numberOfChannels; i++ )
  {
    // Store the positions.
    if ( i == channel1Edit )
    {   
      channelPositions[i] = channel_1;
    }
    else if ( i == channel2Edit )
    {
      channelPositions[i] = channel_2;
    }
    else if ( i == channel3Edit )
    {
      channelPositions[i] = channel_3;
    }
    else if ( i == channel4Edit )
    {
      channelPositions[i] = channel_4;
    }
    else
    {
      // If we are recording a pattern we store
      // -1 in unused channels.
      if ( recordingPattern == true )
      {
        channelPositions[i] = -1;
      }
      // Else we store the neutral position for the servos.
      else
      {
        if ( i == 0 )
        {
          channelPositions[i] = dynamixelNeutral;
        }
        else
        {
          channelPositions[i] = servoNeutral;
        }
      }
    }
  }

  // Update the flag to say we handled it.
  gotRCValue = false;
}

////////////////////////////////////////////////////////////
// Tokenise the line buffer.
// Token pointers stored in tokenArray.
////////////////////////////////////////////////////////////
int TokeniseLineBuffer()
{
  int channelIndex = 0;

  // Clear the array.
  for (int i = 0; i < numberOfChannels + 1; i++)
  {
    tokenArray[i] = 0;
  }

  // Get the first part.
  tokenArray[channelIndex] = strtok(lineBuffer, delimiter);

  // Now get the rest.
  while ( tokenArray[channelIndex] != NULL )
  {
    // Increment the index.
    channelIndex++;
    tokenArray[channelIndex] = strtok(NULL, delimiter);
  }

  return channelIndex;
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
// Output channel status.
// Show if a channel is enabled or not.
////////////////////////////////////////////////////////////
void OutputChannelStatus()
{
  // Loop through the channels.
  Serial.println("Channel:\tLast position:\tState:");
  for (int i = 0; i < numberOfChannels; i++)
  {
    Serial.print(i); Serial.print(":\t\t"); Serial.print(channelPositions[i]); Serial.print("\t\t");
    if ( channelEnabled[i] )
    {
      Serial.println("ON");
    }
    else
    {
      Serial.println("OFF");
    }
  }
}

////////////////////////////////////////////////////////////
// SDInit
////////////////////////////////////////////////////////////
void SDInit()
{
  Serial.println("Initializing SD card...");

  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed or not present");
    return;
  }
  Serial.println("Card initialised.");
}

////////////////////////////////////////////////////////////
// List files on the SD card.
////////////////////////////////////////////////////////////
void ListSDFiles()
{

  Serial.println("Listing files in SD root directory...");

  Serial.print("Files currently open: ");
  Serial.println(fileCounter);

  File root = SD.open("/");
  fileCounter++;

  // Check it opened.
  if (!root)
  {
    FileClose(root);
    Serial.println("Opening root directory failed.");
    return;
  }

  // Loop until there are no more files.
  while (true)
  {
    // Get the next file.
    File entry = root.openNextFile();
    fileCounter++;
    if (!entry)
    {
      // No more files.
      Serial.println("No more files found.");
      // Close this file.
      FileClose(entry);
      break;
    }

    // Is it a file?
    if (!(entry.isDirectory()))
    {
      Serial.print("File found: ");
      Serial.println(entry.name());
    }
    // Close this file.
    FileClose(entry);
  }

  // Close the root directory.
  FileClose(root);
}

////////////////////////////////////////////////////////////
// Open for write function.
////////////////////////////////////////////////////////////
void FileOpenWrite(File* file, String fileName)
{
  File returnFile;
  elapsedMillis openTime = 0;

  if (fileName.length() == 0)
  {
    Serial.println("No file specified.");
    return;
  }

  Serial.print("Opening file for write: ");
  Serial.println(fileName);

  char charFileName[fileName.length() + 1];
  fileName.toCharArray(charFileName, sizeof(charFileName));

  if (SD.exists(charFileName))
  {
    Serial.print(charFileName);
    Serial.println(" exists. Opening...");
  }
  else
  {
    Serial.print(charFileName);
    Serial.println(" does not exist. Creating...");
  }

  // Open (create) the file for writing.
  *file = SD.open(charFileName, O_CREAT | O_WRITE);
  fileCounter++;

  if (file)
  {
    Serial.print("File opened in (mS): ");
    Serial.println(openTime);
  }
  else
  {
    Serial.println("File not opened.");
  }

  return;
}

////////////////////////////////////////////////////////////
// Open for read function.
////////////////////////////////////////////////////////////
void FileOpenRead(File* file, String fileName)
{
  elapsedMillis openTime = 0;

  if (fileName.length() == 0)
  {
    Serial.println("No file specified.");
    return;
  }

  Serial.print("Opening file for read: ");
  Serial.println(fileName);

  char charFileName[fileName.length() + 1];
  fileName.toCharArray(charFileName, sizeof(charFileName));

  if (SD.exists(charFileName))
  {
    Serial.print(charFileName);
    Serial.println(" exists. Opening...");
  }
  else
  {
    Serial.print(charFileName);
    Serial.println(" does not exist.");
  }

  // Open (create) the file for writing.
  *file = SD.open(charFileName, FILE_READ);
  fileCounter++;

  if (file)
  {
    Serial.print("File opened in (mS): ");
    Serial.println(openTime);
  }
  else
  {
    Serial.println("File not opened.");
  }
  return;
}


////////////////////////////////////////////////////////////
// Get a line from a file.
////////////////////////////////////////////////////////////
String FileReadLine(File* file)
{
  String returnLine;

  // If the file is available read from it.
  if (file)
  {
    while (file->available())
    {
      // Get the character.
      int character = file->read();
      // If it is the end of the line return the line.
      if ( character == '\n' )
      {
        return returnLine;
      }
      // Add characters from the line to the return string.
      else
      {
        returnLine = returnLine + character;
      }
    }
  }
  else
  {
    Serial.println("Data file not open.");
  }
  return returnLine;
}

////////////////////////////////////////////////////////////
// Get the number of lines in the file.
////////////////////////////////////////////////////////////
unsigned int FileNumberLines(File* file)
{
  unsigned int lineCount = 0;

  // If the file is available read from it.
  if (file)
  {
    while (file->available())
    {
      byte data = file->readBytesUntil('\n', lineBuffer, lineBufferSize);           
      if ( data )
      {
        lineCount++;
      }
      else
      {
        break;
      }
    }
  }        
  else
  {
    Serial.println("File not open.");
  }

  return lineCount;
}

////////////////////////////////////////////////////////////
// List out a file.
////////////////////////////////////////////////////////////
void FileList(File* file)
{
  // If the file is available read from it.
  if (file)
  {
    while (file->available())
    {
      Serial.write(file->read());
    }
  }
  else
  {
    Serial.println("File not open.");
  }
  Serial.println("\nFile listing complete.");
}

////////////////////////////////////////////////////////////
// Write function.
////////////////////////////////////////////////////////////
void FileWrite(File file, String stringToWrite)
{
  // If the file is available read from it.
  if (file)
  {
    digitalWrite(scopePin, HIGH);
    file.println(stringToWrite);
    linesWritten++;
    digitalWrite(scopePin, LOW);
    //Serial.println(stringToWrite);
  }
  else
  {
    Serial.println("File not open.");
  }
}

////////////////////////////////////////////////////////////
// Delete file.
////////////////////////////////////////////////////////////
void FileDelete(String fileName)
{

  if (fileName.length() == 0)
  {
    Serial.println("No file specified.");
    return;
  }

  Serial.print("Deleting file: ");
  Serial.println(fileName);

  char charFileName[fileName.length() + 1];
  fileName.toCharArray(charFileName, sizeof(charFileName));

  SD.remove(charFileName);
  if (SD.exists(charFileName))
  {
    Serial.println("File not deleted.");
  }
  else
  {
    Serial.println("File deleted.");
    // Reinitialise the file array.
    InitialiseFiles();
  }
}

////////////////////////////////////////////////////////////
// Close file.
////////////////////////////////////////////////////////////
void FileClose(File file)
{
  //if (file)
  // {
  //Serial.print("Closing file... ");
  file.close();
  fileCounter--;
  //Serial.println("Done.");
  //Serial.print("Files open: ");
  //Serial.println(fileCounter);
  // }
}

////////////////////////////////////////////////////////////
// Recording.
////////////////////////////////////////////////////////////
void Recording()
{
  unsigned long thisElapsedPeriod = 0;

  // If the record/play time interval has passed we record and write a position.
  if ( (recPlayTotalTime > ((linesWritten + 1) * recPlayInterval))
       && (recPlayTotalTime >= nextInterval ) )
  {

    /*
        Serial.print("Record difference: ");
        Serial.println((linesWritten + 1) * recPlayInterval);
        Serial.print("Elapsed: ");
        Serial.println(recPlayElapsed);
        Serial.print("Total (mS): ");
        Serial.print(recPlayTotalTime);
        Serial.print("\t(Sec): ");
        Serial.print(recPlayTotalTime / 1000);
        Serial.print(" \t(Min): ");
        Serial.println((recPlayTotalTime / 1000) / 60);
        Serial.print("linesWritten: ");
        Serial.println(linesWritten);
    */

    // Remember how long the period was.
    thisElapsedPeriod = recPlayElapsed;
    // Reset the timer.
    recPlayElapsed = 0;
    // Remember the next interval.
    nextInterval = ((linesWritten + 1) * recPlayInterval) + recPlayInterval;

    // Add pattern if playing.
    if ( patternPlaying )
    {
      PlayingPattern();
    }

    // Dump out the channel position information.
    String lineToWrite;
    for (int i = 0; i < numberOfChannels; i++ )
    {
      lineToWrite.append(channelPositions[i]);

      // If not the last channel append a comma.
      if ( i < (numberOfChannels - 1) )
      {
        lineToWrite.append(delimiter);
      }
    }

    // Write the line out to the terminal and the file.
    Serial.println(lineToWrite);
    FileWrite(recordFile, lineToWrite);
   
    // Move the servos.
    MoveServos();

    // Handle any other writes needed.
    // If the elapsed period was more than two intervals we must have missed some
    // writes so we do extra writes to compensate.
    if ( thisElapsedPeriod > (recPlayInterval * 2) )
    {
      thisElapsedPeriod = thisElapsedPeriod - recPlayInterval;
      while ( thisElapsedPeriod > recPlayInterval)
      {        
        // Write the line out.
        Serial.println("Handling multiple writes.");
        FileWrite(recordFile, lineToWrite);
        thisElapsedPeriod = thisElapsedPeriod - recPlayInterval;
      }
    }
  }
}

////////////////////////////////////////////////////////////
// Adding.
////////////////////////////////////////////////////////////
void Adding()
{
  int bufferIndex = 0;
  bool dataToRead = true;

  // If the record/play time interval has passed we read and move to a position.
  if ( (recPlayTotalTime >= ((linesPlayed + 1) * recPlayInterval))
       && (recPlayTotalTime >= nextInterval ) )
  {

    // Reset the timer.
    recPlayElapsed = 0;

    // Remember the next interval.
    nextInterval = ((linesPlayed + 1) * recPlayInterval) + recPlayInterval;

    // Read a line if any available.
    if ( playFile.available() )
    {
      while ( dataToRead )
      {
        int data = playFile.read();
        //Serial.write(data);
        if ( data != '\n' )
        {
          if ( bufferIndex < lineBufferSize - 1 )
          {
            lineBuffer[bufferIndex] = data;            
            // Null terminate.
            lineBuffer[bufferIndex + 1] = 0;
            bufferIndex++;
          }
          else
          {          
            dataToRead = false;
          }
        }
        else
        {
          dataToRead = false;
        }
      }

      // Increment the line counter.
      linesPlayed++;

      // Tokenise it.
      int numberOfTokens = TokeniseLineBuffer();

      // Loop through the values.
      for (int i = 0; i < numberOfTokens; i++ )
      {
        // If the position is one of the channels we are editing
        // we skip it as the value will be put into the position
        // array from the RC values.
        if ( (i == channel1Edit) || (i == channel2Edit) || (i == channel3Edit) || (i == channel4Edit) )
        {
          // Do nothing. Values come from RC.
        }
        else
        {
          // Get them out of the token array.
          int positionValue = atoi(tokenArray[i]);
          // Convert to a value and store it.
          channelPositions[i] = positionValue;
        }
      }

      // Add pattern if playing.
      if ( patternPlaying )
      {
        PlayingPattern();
      }

      // Dump out the channel position information.
      String lineToWrite;
      for (int i = 0; i < numberOfChannels; i++ )
      {
        lineToWrite.append(channelPositions[i]);
        // If not the last channel append a comma.
        if ( i < (numberOfChannels - 1) )
        {
          lineToWrite.append(delimiter);
        }
      }      
      
      // Write the line out.
      Serial.println(lineToWrite);
      FileWrite(addFile, lineToWrite);

      // Move the servos.
      MoveServos();

      /*
      // Handle any other writes needed.
      // If the elapsed period was more than two intervals we must have missed some
      // writes so we do extra writes to compensate.
      if ( thisElapsedPeriod > (recPlayInterval * 2) )
      {
        thisElapsedPeriod = thisElapsedPeriod - recPlayInterval;
        while ( thisElapsedPeriod > recPlayInterval)
        {          
          // Write the line out but only write the file once the start button is pressed.
          Serial.println("Handling multiple writes.");
          FileWrite(addFile, lineToWrite);         
          thisElapsedPeriod = thisElapsedPeriod - recPlayInterval;
        }
      }
      */
    }
    else
    {
      // No more data to play.
      Stop();
      
      Serial.print("Adding file ");
      Serial.print(fileToAdd);
      Serial.println(" complete.");      
      Serial.print("Play time ");
      ShowMinutesSeconds(recPlayTotalTime);       
      Serial.println("Exiting add mode.");
    }
  }
}

////////////////////////////////////////////////////////////
// Twerking.
////////////////////////////////////////////////////////////
void Twerking()
{
  int bufferIndex = 0;
  bool dataToRead = true;

  // If the record/play time interval has passed we read and move to a position.
  if ( (recPlayTotalTime >= ((linesPlayed + 1) * recPlayInterval))
       && (recPlayTotalTime >= nextInterval ) )
  {

    // Reset the timer.
    recPlayElapsed = 0;

    // Remember the next interval.
    nextInterval = ((linesPlayed + 1) * recPlayInterval) + recPlayInterval;

    // Read a line if any available.
    if ( playFile.available() )
    {
      while ( dataToRead )
      {
        int data = playFile.read();
        //Serial.write(data);
        if ( data != '\n' )
        {
          if ( bufferIndex < lineBufferSize - 1 )
          {
            lineBuffer[bufferIndex] = data;            
            // Null terminate.
            lineBuffer[bufferIndex + 1] = 0;
            bufferIndex++;
          }
          else
          {          
            dataToRead = false;
          }
        }
        else
        {
          dataToRead = false;
        }
      }

      // Increment the line counter.
      linesPlayed++;

      // Tokenise it.
      int numberOfTokens = TokeniseLineBuffer();

      // Loop through the values.
      for (int i = 0; i < numberOfTokens; i++ )
      {
        // If the position is the one we are twerking we change the value
        // based on the RC value difference from 1500 (neutral position).
        
        // Get the value out of the token array.
        int positionValue = atoi(tokenArray[i]);

        if ( i == channel1Edit )
        {
           // Work out the difference.
             int difference = channel_1 - 1500;  
           // Multiply by two to give full deflection.
           difference = difference * 2;

           // Work out the new value.
           positionValue = positionValue + difference;
           // Constrain it to min/max.
           positionValue = constrain(positionValue, minRC, maxRC);
        }
        if ( i == channel2Edit )
        {           
           // Work out the difference.
           int difference = channel_2 - 1500;   
           // Work out the new value.
           positionValue = positionValue + difference;
           // Constrain it to min/max.
           positionValue = constrain(positionValue, minRC, maxRC);
        }
        if ( i == channel3Edit )
        {           
           // Work out the difference.
           int difference = channel_3 - 1500;   
           // Work out the new value.
           positionValue = positionValue + difference;
           // Constrain it to min/max.
           positionValue = constrain(positionValue, minRC, maxRC);
        }
        if ( i == channel4Edit )
        {           
           // Work out the difference.
           int difference = channel_4 - 1500;   
           // Work out the new value.
           positionValue = positionValue + difference;
           // Constrain it to min/max.
           positionValue = constrain(positionValue, minRC, maxRC);
        }
        
        // Convert to a value and store it.
        channelPositions[i] = positionValue;        
      }

      // Dump out the channel position information.
      String lineToWrite;
      for (int i = 0; i < numberOfChannels; i++ )
      {
        lineToWrite.append(channelPositions[i]);
        // If not the last channel append a comma.
        if ( i < (numberOfChannels - 1) )
        {
          lineToWrite.append(delimiter);
        }
      }      
      
      // Write the line out.
      Serial.println(lineToWrite);
      FileWrite(addFile, lineToWrite);

      // Move the servos.
      MoveServos();
    }
    else
    {
      // No more data to play.
      Stop();      
      Serial.print("Twerking file ");
      Serial.print(fileToPlay);
      Serial.println(" complete.");
      Serial.print("Play time ");
      ShowMinutesSeconds(recPlayTotalTime);     
      Serial.println("Exiting twerk mode.");
    }
  }
}

////////////////////////////////////////////////////////////
// PlayingPattern
////////////////////////////////////////////////////////////
void PlayingPattern()
{
  int minusOneCount = 0;

  // Read the line from the pattern file.
  for (int i = 0; i < numberOfChannels; i++ )
  {
    // If it is not a -1 we insert it into the channel positions array.
    if ( pattern[i][patternCurrentLine] != -1 )
    {
      channelPositions[i] = pattern[i][patternCurrentLine];
    }
    // If there is a -1 we count them.
    else
    {
      minusOneCount++;
      // If we receive a row of all -1 values we know the pattern is finished.
      if ( minusOneCount >= numberOfChannels )
      {
        // We stop pattern playback.
        Serial.println("Stopping pattern playback.");
        patternPlaying = false;
      }
    }
  }

  // Have we played all the lines?
  if (patternCurrentLine >= numPatternRows)
  {
    // We stop pattern playback.
    Serial.println("Stopping pattern playback.");
    patternPlaying = false;
  }
  else
  {
    // Increment the line number.
    patternCurrentLine++;
  }
}

////////////////////////////////////////////////////////////
// Playing.
////////////////////////////////////////////////////////////
void Playing()
{
  int bufferIndex = 0;
  bool dataToRead = true;
  
  // If the record/play time interval has passed we read and move to a position.
  if ( (recPlayTotalTime >= ((linesPlayed + 1) * recPlayInterval))
       && (recPlayTotalTime >= nextInterval ) )
  {
    /*
        Serial.print("Play difference: ");
        Serial.println((linesPlayed + 1) * recPlayInterval);
        Serial.print("Elapsed: ");
        Serial.println(recPlayElapsed);
        Serial.print("Total (mS): ");
        Serial.print(recPlayTotalTime);
        Serial.print("\t(Sec): ");
        Serial.print(recPlayTotalTime / 1000);
        Serial.print(" \t(Min): ");
        Serial.println((recPlayTotalTime / 1000) / 60);
        Serial.print("linesPlayed: ");
        Serial.println(linesPlayed);
    */
    // Reset the timer.
    recPlayElapsed = 0;
    // Remember the next interval.
    nextInterval = ((linesPlayed + 1) * recPlayInterval) + recPlayInterval;

    // Read a line if any available.
    if ( playFile.available() )
    {
      while ( dataToRead )
      {
        int data = playFile.read();
        //Serial.write(data);
        if ( data != '\n' )
        {
          if (  bufferIndex < lineBufferSize - 1 )
          {
            lineBuffer[bufferIndex] = data;
            // Null terminate.
            lineBuffer[bufferIndex + 1] = 0;
            bufferIndex++;
          }
          else
          {
            dataToRead = false;
          }
        }
        else
        {
          dataToRead = false;
        }
      }

      // Output to the screen. 
      Serial.println(lineBuffer);
      //Serial.print(".");
      
      dotCounter++;
      if ( dotCounter > numberOfDots )
      {
        Serial.println();
        dotCounter = 0;
      }
      
      linesPlayed++;

      // Tokenise it.
      int numberOfTokens = TokeniseLineBuffer();

      // Loop through the values.
      for (int i = 0; i < numberOfTokens; i++ )
      {
        // Get them out of the token array.
        int positionValue = atoi(tokenArray[i]);
        // Convert to a value and store it.
        channelPositions[i] = positionValue;
        
        // If it is channel 0 send out the analogue value on pin 14.
        if ( i == 0 )
        {
          // Send the analog signal.
          // Map the rc values to the voltage.
          int analogVoltage = map (positionValue, minRC, maxRC, minAnalogVoltage, maxAnalogVoltage);          
          float analogWriteValue = ((float)analogVoltage/1000)/(3.3/4095);
          //Serial.println(analogWriteValue);
          analogWrite(A14, (int)analogWriteValue);
        }        
      }
      // Move the servos.
      MoveServos();      
    }
    else
    {
      // No more data to play.
      Stop();           
      Serial.print("Playing file ");
      Serial.print(fileToPlay);
      Serial.println(" complete.");
      Serial.print("Play time ");
      ShowMinutesSeconds(recPlayTotalTime);
      Serial.println("Exiting playback mode.");
    }
  }
}
  
////////////////////////////////////////////////////////////
// MoveServos
////////////////////////////////////////////////////////////
void MoveServos()
{
  // Wait for any transmitted data still in buffers to actually transmit.
  // If no data is waiting in a buffer to transmit, flush() returns immediately.
  Serial3.flush();

  // Loop through the channels.
  for (int i = 0; i < numberOfChannels; i++)
  {
    // If enabled, move the servo to that position.
    if ( channelEnabled[i] == true)
    {
      if (counterForSSC32Send >= multipleForSSC32Send)
      {
        Serial3.print("#");  // channel command
        Serial3.print(i);  // channel number
        Serial3.print("P");  // position command
        // If the channel position is outside the servo min/max values
        // we change it to be the servo neutral position.
        if ( (channelPositions[i] < minRC) || (channelPositions[i] > maxRC) )
        {
          if ( i == 0 )
          {
            Serial3.print(dynamixelNeutral);  // Neutral position.
          }
          else
          {
            Serial3.print(servoNeutral);  // Neutral position.
          }
        }
        else
        {
          Serial3.print(channelPositions[i]);  // position of the servo
        }
        //Serial3.print("S")  // speed command if we need to specify the speed
        //Serial3.print(channelSpeed[i])  // speed value
      }
    }
  }
  // To send every recPlayInterval or multiple of it
  if (counterForSSC32Send >= multipleForSSC32Send)
  {
    counterForSSC32Send = 1;
    Serial3.print("T");  // travel duration command
    Serial3.println(recPlayInterval * multipleForSSC32Send); // travel duration value
  }
  else
  {
    counterForSSC32Send += 1;
  }
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
    }
  }
}

////////////////////////////////////////////////////////////
// Start.
////////////////////////////////////////////////////////////
void Start(unsigned long delayMilliseconds)
{
  Serial.println("Start command received.");
  
  if (state == play_wait)
  {
    Play(delayMilliseconds);
  }

  if (state == record_wait)
  {
    Record(delayMilliseconds);        
  }

  if (state == add_wait)
  {
    Add();
  }

  if (state == twerk_wait)
  {
    Twerk(delayMilliseconds);
  }
}

////////////////////////////////////////////////////////////
// Stop.
////////////////////////////////////////////////////////////
void Stop()
{
  LEDOff();
  
  // Servos to neutral positions.
  for ( int i = 0; i < numberOfChannels; i++ )
  {    
    if ( i == 0 )
    {
      // Dynamixel servo.
      channelPositions[i] = dynamixelNeutral;
    }
    else
    {
    
      // All other servos.
      channelPositions[i] = servoNeutral;
    }
  }
  
  // Move to position.
  MoveServos();
  
  // Handle stopping in each state.
  if (state == record_wait)
  {
    FileClose(recordFile);    
  }
  
  if (state == record)
  {
    Serial.print("Record file ");
    Serial.print(fileToRecord);
    Serial.println(" complete.");
     
    Serial.print("Total record time (mS): ");
    Serial.print("Lines written to ");
    Serial.print(fileToRecord);
    Serial.print(": ");
    Serial.println(linesWritten);
    FileClose(recordFile);
    // If we were recording a pattern we are now done.
    recordingPattern = false;
    // Reinitialise the file array.
    InitialiseFiles();
  }

  if (state == add_wait)
  {
    FileClose(playFile);
    FileClose(addFile);    
  }
  
  if (state == add)
  {    
    Serial.print("Total add time ");
    ShowMinutesSeconds(recPlayTotalTime);
    Serial.print("Lines written to ");
    Serial.print(fileToAdd);
    Serial.print(": ");
    Serial.println(linesWritten);
    FileClose(playFile);
    FileClose(addFile);
    // Reinitialise the file array.
    InitialiseFiles();
  }

  if (state == twerk_wait)
  {
    FileClose(playFile);
    FileClose(addFile);    
  }
  
  if (state == twerk)
  {    
    Serial.print("Total twerk time ");
    ShowMinutesSeconds(recPlayTotalTime);
    Serial.print("Lines written to ");
    Serial.print(fileToAdd);
    Serial.print(": ");
    Serial.println(linesWritten);
    FileClose(playFile);
    FileClose(addFile);
    // Reinitialise the file array.
    InitialiseFiles();
  }

  if (state == play_wait)
  {
    FileClose(playFile);
  }
  
  if (state == play)
  {
    Serial.print("\nTotal play time ");
    ShowMinutesSeconds(recPlayTotalTime);
    Serial.print("Lines played: ");
    Serial.println(linesPlayed);
    FileClose(playFile);
  }

  // Go out of test mode.
  testMode = false;
  
  // Update the mode.
  state = idling; 

  // Update the timed playing flag.
  file1Playing = false;
  file2Playing = false;
  file3Playing = false;
  file4Playing = false;
  file5Playing = false;
  file6Playing = false;
  timedFilePlaying = false;
}

////////////////////////////////////////////////////////////
// Play wait.
////////////////////////////////////////////////////////////
void Play_Wait()
{
  // Open it for reading.
  FileOpenRead(&playFile, fileToPlay);
  if ( timeMode == true )
  {
    Serial.print("Waiting for start button or start time of: ");
    ShowStartTime();
  }
  else
  {
    Serial.println("Waiting for start button...");
  }  
  state = play_wait;
  LEDFlashOn(250);
}

////////////////////////////////////////////////////////////
// Skip ahead.
// Skip ahead the given number of milliseconds. 
// Return how long this took in mS
////////////////////////////////////////////////////////////
unsigned long SkipAhead(int skipAheadMilliseconds)
{
  elapsedMillis timeTaken = 0;
  int bufferIndex = 0;
  int lineCount = 0; 
  
  // Work out how many lines to skip. Work it out as a float and add 1
  // and ensure we round up any partial result to the next full line.
  // Float to int always truncates.
  
  float totalLines = (float)skipAheadMilliseconds / (float)recPlayInterval;  
  Serial.print("Total lines: ");
  Serial.println(totalLines);
  
  int lines = round(totalLines);
  Serial.print("Lines: ");
  Serial.println(lines);

  Serial.print("Skipping ahead: ");
  ShowMinutesSeconds(skipAheadMilliseconds); 
  Serial.print("Lines: ");
  Serial.println(lines); 
  
  // Skip ahead to the given line.    
  if ( playFile.available() )
  {
    while ( lineCount < lines )
    {
      byte data = playFile.readBytesUntil('\n', lineBuffer, lineBufferSize);      
      if ( data )
      {
        lineCount++;
        if ( state == twerk_wait )
        {
          // If twerking we need to add it to the add file. 
          addFile.print(lineBuffer); 
        }
      }
      else
      {
        // No more data so return.
        Serial.println("No more data available."); 
        // Set out count so we break out of the loop.
        lineCount = lines;            
      }
    }
  }
  else
  {
    // File not available.
    Serial.println("File not available.");    
  }
  
  // We skipped the required number of lines. 
  Serial.print(lineCount);
  Serial.println(" lines skipped.");
  return timeTaken;
}
      
////////////////////////////////////////////////////////////
// Play.
////////////////////////////////////////////////////////////
void Play(unsigned long skipAheadMilliseconds)
{
  unsigned long skiptime = 0;
    
  Serial.print("Starting play from time (mS): ");
  Serial.println(skipAheadMilliseconds);

  // Keep jumping ahead until we are in sync.
  skiptime = SkipAhead(skipAheadMilliseconds);
  while ( skiptime > recPlayInterval )
  { 
    // We have to jump forward again the time taken to do the last jump.   
    skiptime = SkipAhead(skiptime);
  }
  
  //Serial.print("Trigger time to play: ");
  //ShowMinutesSeconds(triggered); 
  Serial.print("Total lines skipped should be: ");
  Serial.println((float)triggered/(float)recPlayInterval); 

  recPlayElapsed = 0;
  recPlayTotalTime = 0;
  linesPlayed = 0;
  nextInterval = recPlayInterval;
  state = play;
  LEDOn();
}

////////////////////////////////////////////////////////////
// Record wait.
////////////////////////////////////////////////////////////
void Record_Wait(int channel1, int channel2, int channel3, int channel4 )
{
  // Make sure file is deleted.
  FileDelete(fileToRecord);

  // Open file for writing.
  FileOpenWrite(&recordFile, fileToRecord);
  if (recordFile)
  {
    channel1Edit = channel1;
    channel2Edit = channel2;
    channel3Edit = channel3;
    channel4Edit = channel4;

    Serial.print("Record mode. Channels ");
    Serial.print(channel1Edit);
    Serial.print("  ");
    Serial.print(channel2Edit);
    Serial.print("  ");
    Serial.print(channel3Edit);
    Serial.print("  ");
    Serial.println(channel4Edit);

    if ( timeMode == true )
    {
      Serial.print("Waiting for start button or start time of: ");
      ShowStartTime();
    }
    else
    {
      Serial.println("Waiting for start button...");
    }
    
    state = record_wait;   
    LEDFlashOn(250);   
  }
  else
  {
    Stop();    
    Serial.println("Can't enter record mode.");
  }
}

////////////////////////////////////////////////////////////
// Record.
////////////////////////////////////////////////////////////
void Record(unsigned long delayMilliseconds)
{
  elapsedMillis extraLinesWriteTime = 0;

  // Get channel position information.
  String lineToWrite;
  for (int i = 0; i < numberOfChannels; i++ )
  {
    lineToWrite.append(channelPositions[i]);

    // If not the last channel append a comma.
    if ( i < (numberOfChannels - 1) )
    {
      lineToWrite.append(delimiter);
    }
  }   
    
  // We need to write extra lines to account for the delay
  // in starting to record.
  int extraLines = delayMilliseconds / recPlayInterval;
  Serial.print("Record delay: ");
  Serial.print(delayMilliseconds);
  Serial.print("Writing extra lines: ");
  Serial.print(extraLines);

  // Write out the correct number of lines. 
  for ( int i = 0; i < extraLines; i++ )
  {
    // Write the lines out to the file.
    Serial.println(lineToWrite);
    FileWrite(recordFile, lineToWrite);
  }

  Serial.print("Time to write extra lines: ");
  Serial.print(extraLinesWriteTime);

  // Write any extras needed due to the delay in writing lines.
  while ( extraLinesWriteTime > recPlayInterval )
  {    
     extraLinesWriteTime = extraLinesWriteTime - recPlayInterval;
     Serial.print("Time to write extra, extra lines: ");
     Serial.print(extraLinesWriteTime);
     FileWrite(recordFile, lineToWrite);
  }
  
  recPlayElapsed = 0;
  recPlayTotalTime = 0;
  linesWritten = 0;
  nextInterval = recPlayInterval;
  state = record;  
  LEDOn();
  Serial.println("Starting record.");
}

////////////////////////////////////////////////////////////
// Add wait.
////////////////////////////////////////////////////////////
void Add_Wait(int channel1, int channel2, int channel3, int channel4 )
{
  // Open the source file for reading.
  Serial.println("Opening source file.");
  FileOpenRead(&playFile, fileToPlay);
  if (!playFile)
  {
    Stop();
    Serial.println("Can't enter add mode.");
    return;
  }

  // Make sure destination file is deleted.
  FileDelete(fileToAdd);

  // Open destination file for writing.
  FileOpenWrite(&addFile, fileToAdd);
  if (addFile)
  {
    channel1Edit = channel1;
    channel2Edit = channel2;
    channel3Edit = channel3;
    channel4Edit = channel4;

    Serial.print("Add mode. Channels ");
    Serial.print(channel1Edit);
    Serial.print("  ");
    Serial.print(channel2Edit);
    Serial.print("  ");
    Serial.print(channel3Edit);
    Serial.print("  ");
    Serial.println(channel4Edit);
    if ( timeMode == true )
    {
      Serial.print("Waiting for start button or start time of: ");
      ShowStartTime();
    }
    else
    {
      Serial.println("Waiting for start button...");
    }
   
    state = add_wait; 
    LEDFlashOn(250);   
  }
  else
  {
    Stop();
    Serial.println("Can't enter add mode.");
    FileClose(playFile);
  }
}

////////////////////////////////////////////////////////////
// Add.
////////////////////////////////////////////////////////////
void Add()
{
  recPlayElapsed = 0;
  recPlayTotalTime = 0;
  linesPlayed = 0;
  linesWritten = 0;
  nextInterval = recPlayInterval;
  state = add;
  LEDOn();
  Serial.println("Starting add.");
}

////////////////////////////////////////////////////////////
// Twerk wait.
////////////////////////////////////////////////////////////
void Twerk_Wait(int channel1, int channel2, int channel3, int channel4 )
{
  // Open the source file for reading.
  Serial.println("Opening source file.");
  FileOpenRead(&playFile, fileToPlay);
  if (!playFile)
  {
    Stop();
    Serial.println("Can't enter twerk mode.");
    return;
  }

  // Make sure destination file is deleted.
  FileDelete(fileToAdd);

  // Open destination file for writing.
  FileOpenWrite(&addFile, fileToAdd);
  if (addFile)
  {
    channel1Edit = channel1;
    channel2Edit = channel2;
    channel3Edit = channel3;
    channel4Edit = channel4;

    Serial.print("Twerk mode. Channels ");
    Serial.print(channel1Edit);
    Serial.print("  ");
    Serial.print(channel2Edit);
    Serial.print("  ");
    Serial.print(channel3Edit);
    Serial.print("  ");
    Serial.println(channel4Edit);
    if ( timeMode == true )
    {
      Serial.print("Waiting for start button or start time of: ");
      ShowStartTime();
    }
    else
    {
      Serial.println("Waiting for start button...");
    }
   
    state = twerk_wait; 
    LEDFlashOn(250);   
  }
  else
  {
    Stop();
    Serial.println("Can't enter twerk mode.");
    FileClose(playFile);
  }
}

////////////////////////////////////////////////////////////
// Twerk.
////////////////////////////////////////////////////////////
void Twerk(unsigned long skipAheadMilliseconds)
{
  unsigned long skiptime = 0;
    
  Serial.print("Starting twerk from time (mS): ");
  Serial.println(skipAheadMilliseconds);

  // Keep jumping ahead until we are in sync.
  skiptime = SkipAhead(skipAheadMilliseconds);
  while ( skiptime > 0 )
  { 
    // We have to jump forward again the time taken to do the last jump.   
    skiptime = SkipAhead(skiptime);
  }
  
  Serial.print("Delay from trigger to twerk (mS): ");
  Serial.println(triggered);   

  recPlayElapsed = 0;
  recPlayTotalTime = 0;
  linesPlayed = 0;
  linesWritten = 0;
  nextInterval = recPlayInterval;
  state = twerk;
  LEDOn();
  Serial.println("Starting twerk.");
}

////////////////////////////////////////////////////////////
// Help.
////////////////////////////////////////////////////////////
void Help(void)
{
  Serial.println("\n - Commands available -");
  Serial.println("\"help\"\n\tThis information.");
  Serial.println("\"status\"\n\tStatus information.");
  Serial.println("\"starttime <h> <m> <s> <f>\"\n\tEnter the start time specifying hours, minutes, seconds and frame. Enter with no arguments to display current time setting.");
  Serial.println("\"record <filename> <channel1> <channel2> <channel3> <channel4> <p>\"\n\tRecord to a file for the given channels.\n\tUse -1 to ignore a channel.\n\tUse p to record a pattern (must specify all four channels).");
  Serial.println("\"add <source filename> <destination filename> <channel1> <channel2> <channel3> <channel4>\"\n\tAdd to a file.\n\tPlay from the source and record to destination.\n\tUse -1 to ignore a channel.");
  Serial.println("\"twerk <source filename> <destination filename> <channel1> <channel2> <channel3> <channel4>\"\n\tTwerk a file.\n\tPlay from the source and record to destination editing the given channels.");
  Serial.println("\"play <filename>\"\n\tPlay from given file.");
  Serial.println("\"stop\"\n\tStop everything.");
  Serial.println("\"dir\"\n\tList files and directories on SD card.");
  Serial.println("\"list <filename>\"\n\tOutput the contents of the given file.");
  Serial.println("\"info <filename>\"\n\tOutput information about the given file.");
  Serial.println("\"delete <filename>\"\n\tDelete the given file.");
  Serial.println("\"y\"\n\tConfirm file deletion.");
  Serial.println("\"n\"\n\tCancel file deletion.");
  Serial.println("\"channel <number> <number> ...\"\n\tOutput given channels status. No numbers lists all channels.");
  Serial.println("\"enable <number> <number> ...\"\n\tEnable given channels. No numbers enables all channels.");
  Serial.println("\"disable <number> <number> ...\"\n\tDisable given channels. No numbers disables all channels.");
  Serial.println("\"pattern\"\n\tClear pattern. ");
  Serial.println("\"pattern <filename> < + or - > <trigger>\"\n\tEnable pattern 1 reading from file given.\n\tSpecify greater than (+) or less than (-) trigger.");
  Serial.println("\"test\"\n\tEnter test mode. Stop to exit.");
  Serial.println("\"time\"\n\tEnter timed mode. Stop to exit.");
  Serial.println("\"performance\"\n\tToggle performance mode.");
  Serial.println("");
}

////////////////////////////////////////////////////////////
// Show status.
////////////////////////////////////////////////////////////
void ShowStatus(void)
{
  Serial.println();
  Serial.println("--- Status ---");
  Serial.println();
  Serial.print("Version: ");
  Serial.println(versionNumber);
  if ( performanceMode )
  {
    Serial.println("Performance mode: ON");
  }
  else
  {
    Serial.println("Performance mode: OFF");
  }
  if ( timeMode )
  {
    Serial.println("Time mode: ON");
  }
  else
  {
    Serial.println("Time mode: OFF");
  }
  if ( testMode )
  {
    Serial.println("Test mode: ON");
  }
  else
  {
    Serial.println("Test mode: OFF");
  }
  if ( state == idling )
  {
    Serial.println("State: idling");
  }
  if ( state == play_wait )
  {
    Serial.println("State: play_wait");
    Serial.print("File: ");
    Serial.println(fileToPlay);
  }
  if ( state == record_wait )
  {
    Serial.println("State: record_wait");
    Serial.print("File: ");
    Serial.println(fileToRecord);
  }
  if ( state == add_wait )
  {
    Serial.println("State: add_wait");
    Serial.print("File to play: ");
    Serial.print(fileToPlay);
    Serial.print("File to record to: ");
    Serial.println(fileToAdd);
  }
  if ( state == twerk_wait )
  {
    Serial.println("State: twerk_wait");
    Serial.print("File to play: ");
    Serial.print(fileToPlay);
    Serial.print("File to record to: ");
    Serial.println(fileToAdd);
  }
  if ( state == play )
  {    
    Serial.print("Playing: ");
    Serial.println(fileToPlay);
  }
  if ( state == record )
  {    
    Serial.print("Recording: ");
    Serial.println(fileToRecord);
  }
  if ( state == add )
  {    
    Serial.print("Adding: ");
    Serial.println(fileToAdd);
  }
  if ( state == twerk )
  {
    Serial.print("Twerking: ");
    Serial.println(fileToAdd);
  }
  Serial.print("Start time: ");
  ShowStartTime();

  if (patternEnabled)
  {
    Serial.println("Pattern: ON");
    if (patternUpperLimit)
    {
      Serial.print("Trigger when CH1 over: ");
      Serial.println(patternUpperLimit);
    }
    else if (patternLowerLimit)
    {
      Serial.print("Trigger when CH1 under: ");
      Serial.println(patternLowerLimit);
    }
  }
  else
  {
    Serial.println("Pattern: OFF");
  }
  Serial.println();
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
    if (strncmp(commandTokenArray[0], "record", 6) == 0 )
    {
      // Remember the name of the file to record.
      if (commandCount == 1)
      {
        Serial.println("No file specified.");
      }
      else
      {
        fileToRecord = commandTokenArray[1];
        fileToRecord = fileToRecord.trim();
        Serial.print("File name to record: ");
        Serial.println(fileToRecord);
  
        // Convert given channels to record to an int.
        // If no channel specifies set it to -1.
        int channel1 = -1;
        int channel2 = -1;
        int channel3 = -1;
        int channel4 = -1;
  
        if ( commandTokenArray[2] )
        {
          channel1 = atoi(commandTokenArray[2]);
        }
        if ( commandTokenArray[3] )
        {
          channel2 = atoi(commandTokenArray[3]);
        }
        if ( commandTokenArray[4] )
        {
          channel3 = atoi(commandTokenArray[4]);
        }
        if ( commandTokenArray[5] )
        {
          channel4 = atoi(commandTokenArray[5]);
        }
  
        // If the last parameter is a p then we are recording a pattern.
        if ( (commandCount == 7) && (strncmp(commandTokenArray[commandCount - 1], "p", 1) == 0) )
        {
          Serial.println("Recording a pattern.");
          recordingPattern = true;
          patternEnabled = false;
        }
  
        // Record the given channels.
        Record_Wait(channel1, channel2, channel3,  channel4);
        validCommand = true;        
      }
      break;
    }
  
    if (strncmp(commandTokenArray[0], "add", 3) == 0 )
    {
      // Remember the name of the file to add.
      if (commandCount == 1)
      {
        Serial.println("Source or destination file not specified.");
      }
      else
      {
        fileToPlay = commandTokenArray[1];
        fileToPlay = fileToPlay.trim();
        Serial.println("File name to play from: ");
        Serial.println(fileToPlay);
  
        fileToAdd = commandTokenArray[2];
        fileToAdd = fileToAdd.trim();
        Serial.println("File name to record to: ");
        Serial.println(fileToAdd);
  
        // Convert given channels to record to an int.
        // If no channel specifies set it to -1.
        int channel1 = -1;
        int channel2 = -1;
        int channel3 = -1;
        int channel4 = -1;
  
        if ( commandTokenArray[3] )
        {
          channel1 = atoi(commandTokenArray[3]);
        }
        if ( commandTokenArray[4] )
        {
          channel2 = atoi(commandTokenArray[4]);
        }
        if ( commandTokenArray[5] )
        {
          channel3 = atoi(commandTokenArray[5]);
        }
        if ( commandTokenArray[6] )
        {
          channel4 = atoi(commandTokenArray[6]);
        }
  
        // Add the given channels.
        Add_Wait(channel1, channel2, channel3,  channel4);
        validCommand = true;
      }
      break;
    }
  
    if (strncmp(commandTokenArray[0], "twerk", 5) == 0 )
    {
      // Remember the name of the file to add to.
      if (commandCount == 1)
      {
        Serial.println("Source or destination file not specified.");
      }
      else
      {
        fileToPlay = commandTokenArray[1];
        fileToPlay = fileToPlay.trim();
        Serial.println("File name to play from: ");
        Serial.println(fileToPlay);
  
        fileToAdd = commandTokenArray[2];
        fileToAdd = fileToAdd.trim();
        Serial.println("File name to record to: ");
        Serial.println(fileToAdd);
  
        // Convert given channels to record to an int.
        // If no channel specifies set it to -1.
        int channel1 = -1;
        int channel2 = -1;
        int channel3 = -1;
        int channel4 = -1;
  
        if ( commandTokenArray[3] )
        {
          channel1 = atoi(commandTokenArray[3]);
        }
        if ( commandTokenArray[4] )
        {
          channel2 = atoi(commandTokenArray[4]);
        }
        if ( commandTokenArray[5] )
        {
          channel3 = atoi(commandTokenArray[5]);
        }
        if ( commandTokenArray[6] )
        {
          channel4 = atoi(commandTokenArray[6]);
        }
        
        // Twerk the given channel.
        Twerk_Wait(channel1, channel2, channel3, channel4);
        validCommand = true;
      }
      break;
    }
  
    if (strncmp(commandTokenArray[0], "starttime", 8) == 0 )
    { 
      // If values are given update the time.
      // If none we just display the existing time.
      if (commandCount > 1)
      {      
        // Default to zero.
        start_h = 0;
        start_m = 0;
        start_s = 0;
        start_f = 0;
        // Then override with given values.
        if ( commandTokenArray[1] )
        {
          start_h = atoi(commandTokenArray[1]);
        }      
        if ( commandTokenArray[2] )
        {
          start_m = atoi(commandTokenArray[2]);
        }
        if ( commandTokenArray[3] )
        {
          start_s = atoi(commandTokenArray[3]);
        }
        if ( commandTokenArray[4] )
        {
          start_f = atoi(commandTokenArray[4]);
        }
      }
      
      // Output the time.
      Serial.print("Start time set to: ");
      Serial.print(start_h);
      Serial.print(":");
      Serial.print(start_m);
      Serial.print(":");
      Serial.print(start_s);
      Serial.print(":");
      Serial.println(start_f);
      
      validCommand = true; 
      break;   
    }
    
    if (strncmp(commandTokenArray[0], "play", 4) == 0 )
    {
      // Remember the name of the file to record.
      if (commandCount == 1)
      {
        Serial.println("No file specified.");
      }
      else
      {
        fileToPlay = commandTokenArray[1];
        fileToPlay = fileToPlay.trim();
        Serial.print("File name to play: ");
        Serial.println(fileToPlay);
        Play_Wait();
        validCommand = true;
      }
      break;
    }
  
    // Delete file.
    if (strncmp(commandTokenArray[0], "delete", 6) == 0 )
    {
      // Remember the name of the file to delete.
      if (commandCount == 1)
      {
        Serial.println("No file specified.");
      }
      else
      {
        fileToDelete = commandTokenArray[1];
        fileToDelete = fileToDelete.trim();
        Serial.println("File name to delete: ");
        Serial.println(fileToDelete);
        state = deleteFileConfirm;
        Serial.print("Press 'y' to confirm deletion or 'n' to cancel: ");
        validCommand = true;
      }
      break;
    }
  
    // Delete file confirmation.
    if (strncmp(commandTokenArray[0], "y", 1) == 0 )
    {
      if ( state == deleteFileConfirm )
      {
        Serial.println("Delete confirmed.");
        FileDelete(fileToDelete);
        state = idling;
        validCommand = true;
      }
      break;
    }
  
    // Cancel deleting file.
    if (strncmp(commandTokenArray[0], "n", 1) == 0 )
    {
      if ( state == deleteFileConfirm )
      {
        Serial.println("Delete cancelled.");
        state = idling;
        validCommand = true;
      }
      break;
    }
  
    // List out the file.
    if (strncmp(commandTokenArray[0], "list", 4) == 0 )
    {
      // Get the filename.
      if (commandCount == 1)
      {
        Serial.println("No file specified.");
      }
      else
      {
        String fileName = commandTokenArray[1];
        fileName = fileName.trim();
        Serial.print("File name to list: ");
        Serial.println(fileName);
        // Open it.
        File file;
        FileOpenRead(&file, fileName);
        // Dump it.
        FileList(&file);
        // Close it.
        FileClose(file);
        // Update the mode.
        state = idling;
        validCommand = true;
      }
      break;
    }
  
    // List files and directories on SD card.
    if (strncmp(commandTokenArray[0], "dir", 3) == 0 )
    {
      ListSDFiles();
      validCommand = true;
      break;
    }
  
    // Start.
    if (strncmp(commandTokenArray[0], "start", 5) == 0 )
    {
      Start(0);
      validCommand = true;
      break;
    }
    
    // Stop.
    if (strncmp(commandTokenArray[0], "stop", 4) == 0 )
    {
      Stop();
      validCommand = true;
      break;
    }
  
    // Outputs file info.
    if (strncmp(commandTokenArray[0], "info", 4) == 0 )
    {
      // Get the filename.
      if (commandCount == 1)
      {
        Serial.println("No file specified.");
      }
      else
      {
        String fileName = commandTokenArray[1];
        fileName = fileName.trim();
        Serial.print("File info for: ");
        Serial.println(fileName);
        File file;
        FileOpenRead(&file, fileName);
        Serial.print("File size is ");
        Serial.print(file.size() / 1000);
        Serial.println(" kilobytes.");
        unsigned int numberOfLines = FileNumberLines(&file);
        Serial.print("Number of lines is ");
        Serial.print(numberOfLines);
        Serial.println(".");
        Serial.print("Playback time is ");
        int totalMilliSeconds = (numberOfLines * recPlayInterval);                       
        ShowMinutesSeconds(totalMilliSeconds);
        FileClose(file);
        validCommand = true;
      }
      break;
    }
  
    // Output channel info.
    if (strncmp(commandTokenArray[0], "channel", 6) == 0 )
    {
      // If no channel specified output all.
      if (commandCount == 1)
      {
        // Output all channels.
        OutputChannelStatus();
      }
      else
      {
        Serial.println("Channel:\tLast position:\tState:");
        for ( int i = 1; i < commandCount; i++ )
        {
          // Convert given channels to an int.
          int channelNumber = atoi(commandTokenArray[i]);
  
          // Output the channel.
          Serial.print(channelNumber); Serial.print(":\t\t"); Serial.print(channelPositions[channelNumber]); Serial.print("\t\t");
          if ( channelEnabled[channelNumber] )
          {
            Serial.println("ON");
          }
          else
          {
            Serial.println("OFF");
          }
        }
        validCommand = true;
      }
      break;
    }
  
    // Enable channels.
    if (strncmp(commandTokenArray[0], "enable", 6) == 0 )
    {
      // If no channel specified enable all.
      if (commandCount == 1)
      {
        // Enable all channels.
        Serial.println("Enabling all channels.");
        for (int i = 0; i < numberOfChannels; i++)
        {
          channelEnabled[i] = true;
        }
      }
      else
      {
        // Enable specified channels.
        for ( int i = 1; i < commandCount; i++ )
        {
          // Convert given channels to an int.
          int channelNumber = atoi(commandTokenArray[i]);
  
          // Enable the channel.
          Serial.print("Enabling channel: ");
          Serial.println(channelNumber);
          channelEnabled[channelNumber] = true;
        }
      }
      validCommand = true;
      break;
    }
  
    // Disable channels.
    if (strncmp(commandTokenArray[0], "disable", 7) == 0 )
    {
      // If no channel specified disable all.
      if (commandCount == 1)
      {
        // Enable all channels.
        Serial.println("Disabling all channels.");
        for (int i = 0; i < numberOfChannels; i++)
        {
          channelEnabled[i] = false;
        }
      }
      else
      {
        // Disable specified channels.
        for ( int i = 1; i < commandCount; i++ )
        {
          // Convert given channels to an int.
          int channelNumber = atoi(commandTokenArray[i]);
  
          // Enable the channel.
          Serial.print("Disabling channel: ");
          Serial.println(channelNumber);
          channelEnabled[channelNumber] = false;
        }
      }
      validCommand = true;
      break;
    }
  
    // Set up pattern.
    if (strncmp(commandTokenArray[0], "pattern", 7) == 0 )
    {
      // If nothing specified we disable the pattern.
      if ( commandCount == 1 )
      {
        Serial.println("Pattern cleared.");
        patternEnabled = false;
        patternPlaying = false;
      }
      else if ( (commandCount > 1) && (commandCount < 4) )
      {
        // Not enough paramaters specified.
        Serial.println("Incorrect file or trigger parameters given.");
      }
      else
      {
        // Get the filename.
        String fileName = commandTokenArray[1];
        fileName = fileName.trim();
  
        // Check the limit.
        int triggerValue = atoi(commandTokenArray[3]);
        if ( (triggerValue < minRC) || (triggerValue > maxRC) )
        {
          Serial.print("Trigger must be between ");
          Serial.print(minRC);
          Serial.print(" and ");
          Serial.println(maxRC);
        }
        else
        {
          // Get if we want an upper or lower trigger.
          if (strncmp(commandTokenArray[2], "-", 1) == 0)
          {
            Serial.print("Setting trigger to be less than: ");
            Serial.println(triggerValue);
            patternLowerLimit = triggerValue;
            patternUpperLimit = 0;
            SetUpPattern(pattern, fileName);
          }
          else if (strncmp(commandTokenArray[2], "+", 1) == 0)
          {
            Serial.print("Setting trigger to be greater than: ");
            Serial.println(triggerValue);
            patternLowerLimit = 0;
            patternUpperLimit = triggerValue;
            SetUpPattern(pattern, fileName);
          }
          else
          {
            Serial.println("Use \"+\" (greater than) or \"-\" (less than) for trigger.");
          }
        }
      }
      validCommand = true;
      break;
    }
  
    // Outputs file info.
    if (strncmp(commandTokenArray[0], "help", 4) == 0 )
    {
      Help();
      validCommand = true;
      break;
    }
  
    // Timed control mode.
    if (strncmp(commandTokenArray[0], "time", 4) == 0 )
    {
      // Always reset the time.      
      h = 0;
      m = 0;
      s = 0;
      f = 0;
        
      // If in time mode we get out of it.
      if ( timeMode == true )
      {
        Serial.println("Exiting timed mode.");
        timeMode = false;        
      }
      else
      {
        // We go into it.
        Serial.println("Starting timed mode. Waiting for time codes...");
        timeMode = true;             
      }
      // Test mode off in test mode.
      testMode = false;
      validCommand = true;
      break;
    }
  
    // Test mode.
    if (strncmp(commandTokenArray[0], "test", 4) == 0 )
    {    
      // We go into test mode.
      Serial.println("Entering test mode.");
      testMode = true; 
      
      // Timed mode and performance mode off in test mode.
      timeMode = false;
      performanceMode = false;     
      validCommand = true;
      break;
    }
  
    // Performance mode.
    if (strncmp(commandTokenArray[0], "performance", 11) == 0 )
    {    
      
      // If not in performance mode we go into performance mode.
      if ( performanceMode == true )
      {
        Serial.println("Performance mode OFF.");
        performanceMode = false;
        timeMode = false;
        // Reset the time.
        h = 0;
        m = 0;
        s = 0;
        f = 0;
      }
      else
      {
        Serial.println("Entering performance mode.");
        performanceMode = true;
        // Performance mode needs time mode to trigger performances.
        timeMode = true;
        // Test mode off.
        testMode = false;
      }
      
      // Test mode off.  
      testMode = false;      
      validCommand = true;
      break;
    }
  
    // Show status.
    if (strncmp(commandTokenArray[0], "status", 5) == 0 )
    {    
      ShowStatus();   
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
// CompareTime
// Compares the time signal with the given one and 
// returns the difference in milliseconds.
////////////////////////////////////////////////////////////
long CompareTime(int hours, int minutes, int seconds, int frames)
{
  // Return the number of milliseconds difference.
  long time1 = (h * 60 * 60 * 1000) + (m * 60 * 1000) + (s * 1000) + (f * 40);  // 40 mS for 25 frame signals per second.
  long time2 = (hours * 60 * 60 * 1000) + (minutes * 60 * 1000) + (seconds * 1000) + (frames * 40); // 40 mS for 25 frame signals per second.
  long milliSecondsDifference = time1 - time2;
  
  /* 
  Serial.print("Time1 (current time): ");
  Serial.println(time1);
  Serial.print("Time2 (track start time): ");
  Serial.println(time2); 
  Serial.print("difference: ");
  Serial.println(milliSecondsDifference); 
  */
    
  return milliSecondsDifference;  
}

////////////////////////////////////////////////////////////
// CompareStartTime
// Compares the time signal with the start time.
// Returns difference in milliseconds.
////////////////////////////////////////////////////////////
long CompareStartTime()
{

  // Return the number of milliseconds difference.
  long time1 = (h * 60 * 60 * 1000) + (m * 60 * 1000) + (s * 1000) + (f * 40);  // 40 mS for 25 frame signals per second.
  long time2 = (start_h * 60 * 60 * 1000) + (start_m * 60 * 1000) + (start_s * 1000) + (start_f * 40); // 40 mS for 25 frame signals per second.
  long milliSecondsDifference = time1 - time2;

  /*
  Serial.print("Time1: ");
  Serial.println(time1);
  Serial.print("Time2: ");
  Serial.println(time2); 
  Serial.print("difference: ");
  Serial.println(milliSecondsDifference); 
  */
  
  return milliSecondsDifference;  
}

////////////////////////////////////////////////////////////
// SetUpPattern.
////////////////////////////////////////////////////////////
void SetUpPattern(int pattern[numberOfChannels][numPatternRows], String fileName)
{

  int bufferIndex = 0;
  int rows = 0;

  // Open the pattern file.
  FileOpenRead(&patternFile, fileName);

  if ( !patternFile )
  {
    FileClose(patternFile);
    return;
  }

  // Initialise the pattern array to -1.
  // Iterate the rows (time).
  for (int i = 0; i < numPatternRows; i++)
  {
    // Iterate the columns (channels).
    for (int j = 0; j < numberOfChannels; j++)
    {
      // Update the column/row.
      pattern[j][i] = -1;
    }
  }

  // Read a line if any available.
  while ( patternFile.available() )
  {
    int data = patternFile.read();

    if ( data != '\n' )
    {
      lineBuffer[bufferIndex] = data;
      bufferIndex++;
      lineBuffer[bufferIndex] = 0;
    }
    else
    {
      // We received a line.
      //Serial.print("Line read from pattern file: ");
      //Serial.println(lineBuffer);

      // Tokenise it.
      int numberOfTokens = TokeniseLineBuffer();

      // Loop through the values.
      for (int i = 0; i < numberOfTokens; i++ )
      {
        // Get them out of the token array.
        int positionValue = atoi(tokenArray[i]);
        // Convert to a value and store it.
        pattern[i][rows] = positionValue;
      }

      // Reset the buffer count.
      bufferIndex = 0;

      // Increment row.
      rows++;

      // If we have filled up memory we are done.
      if ( rows >= numPatternRows )
      {
        // No more data available.
        Serial.println("Pattern buffer full.");
        break;
      }
    }
  }

  // No more data available.
  Serial.println("Pattern file read complete.");

  // Close the file.
  FileClose(patternFile);

  // Print out the pattern array.
  // Iterate the rows (time).
  for (int i = 0; i < numPatternRows; i++)
  {
    // Iterate the columns (channels).
    for (int j = 0; j < numberOfChannels; j++)
    {
      // Update the column/row.
      Serial.print(pattern[j][i]);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Pattern now enabled.
  patternEnabled = true;
  patternPlaying = false;
  Serial.println("Pattern enabled.");
}

////////////////////////////////////////////////////////////
// HandleButtons.
////////////////////////////////////////////////////////////
void HandleButtons(void)
{
  
  // If in play_wait, record_wait or add_wait mode we can stop.
  if ( (state == play_wait)     
    || (state == record_wait)     
    || (state == add_wait)
    || (state == twerk_wait))
  {
    if ( debouncerStart.fell() )
    {
      triggered = 0;
      Start(0);
    }
  }
  
  // If in waiting, play, record, add, twerking or test mode we can stop.
  if ( (state == play_wait) 
    || (state == play) 
    || (state == record_wait) 
    || (state == record) 
    || (state == add_wait)
    || (state == add)
    || (state == twerk_wait)
    || (state == twerk ) )
  {
    if ( debouncerStop.fell() )
    {
      Serial.println("\nStop button pressed.");                
      Stop();      
    }
  }

  // Handle test button press.
  if ( state == idling )
  {
    if ( debouncerTest.fell() )
    {
      Serial.println("\nTest button pressed.");
      Stop();
      testMode = true;
    }
  }
}

////////////////////////////////////////////////////////////
// Test mode.
////////////////////////////////////////////////////////////
void TestMode(void)
{
  // If we aren't playing currently choose a random file
  // to play and play it.
  if ( state == idling )
  {
    // Generate a random number from 0 to numberOfFiles - 1;
    long randNumber = random(numberOfFiles);
    Serial.println(randNumber + 1);

    // See if we have an active file with that number.
    if ( fileActive[randNumber] == true )
    {
      // Convert it to a string.
      String file = (randNumber + 1);
      // Open it for reading.
      FileOpenRead(&playFile, file);
      Serial.println("Test mode...");
      recPlayElapsed = 0;
      recPlayTotalTime = 0;
      linesPlayed = 0;
      nextInterval = recPlayInterval;
      state = play;
    }
  }
}


////////////////////////////////////////////////////////////
// Timed mode.
////////////////////////////////////////////////////////////
void TimeMode(void)
{
  elapsedMillis startDelay;
  
  // Valid time received.
  bool gotTime = false;
  
  // Time difference. 
  long timeDifference = 0;
  
  // Check for time serial data.
  if ( (Serial2.available()) )
  {
    // Reset the last time signal received.
    lastTimeUpdate = 0;
    
    // Decode timecode.
    gotTime = DecodeTimeCode();
    
    // Decode midi timecode.
    //gotTime = DecodeMidi();
    
    if ( gotTime )
    {
      
      // Get the difference to the stored start time.
      timeDifference = CompareStartTime(); 
      
      // Times but NOT performance mode.
      // Wait for the given start time then trigger the
      // waiting event.
      if ( (state == play_wait)
       || (state == record_wait)
       || (state == add_wait)
       || (state == twerk_wait)
       && (performanceMode == false) )
      {       
        // If the time matches we start whatever we were waiting for.
        if ((timeDifference >= 0) && (!timedFilePlaying) )
        {  
          // Set the trigger time.
          triggered = 0;
          //Serial.print("Normal mode difference: ");
          //Serial.println(timeDifference);     
          timedFilePlaying = true;    
          Start(timeDifference);
        }
      }
  
      // Show the current time if in performance mode and idling.
      if ( (state == idling) && (performanceMode == true) )
      {
        Serial.print("Time: ");
        ShowTime();
      }
       
      // If in performance mode we just trigger tracks based on time.
      if ( performanceMode == true )
      {
        // First track.
        if ( track1_h > 0 )
        {
          timeDifference = CompareTime(track1_h, track1_m , track1_s, track1_f);                      
          startDelay = 0;
          if ((timeDifference >= 0) && (timeDifference < 3600 * 1000) && !file1Playing)
          {
            // Set the trigger time.
            triggered = timeDifference;
            Stop();
            file1Playing = true;
            Serial.print("Time trigger at: ");
            ShowTime();
            Serial.println("Playing file: 1");
            Serial.print("Start delay: ");
            Serial.println(startDelay);  
            FileOpenRead(&playFile, "1");
            Play(timeDifference + startDelay);           
          }
        }
        
        // Second track.
        if ( track2_h > 0 )
        {
          timeDifference = CompareTime(track2_h, track2_m , track2_s, track2_f);        
          startDelay = 0;
          if ((timeDifference >= 0) && (timeDifference < 3600 * 1000) && !file2Playing)
          {
            // Set the trigger time.
            triggered = timeDifference;
            Stop();
            file2Playing = true;
            Serial.print("Time trigger at: ");
            ShowTime();
            Serial.println("Playing file: 2");
            Serial.print("Start delay: ");
            Serial.println(startDelay);  
            FileOpenRead(&playFile, "2");
            Play(timeDifference + startDelay);        
          }
        }
        
        // Third track.   
        if ( track3_h > 0 )
        {
          timeDifference = CompareTime(track3_h, track3_m , track3_s, track3_f);        
          startDelay = 0;
          if ((timeDifference >= 0) && (timeDifference < 3600 * 1000) && !file3Playing)
          {
            // Set the trigger time.
            triggered = timeDifference;
            Stop();
            file3Playing = true;
            Serial.print("Time trigger at: ");
            ShowTime();
            Serial.println("Playing file: 3");
            Serial.print("Start delay: ");
            Serial.println(startDelay);  
            FileOpenRead(&playFile, "3");
            Play(timeDifference + startDelay);                  
          }
        }
        
        // Fourth track.    
        if ( track4_h > 0 )
        {
          timeDifference = CompareTime(track4_h, track4_m , track4_s, track4_f);        
          startDelay = 0;
          if ((timeDifference >= 0) && (timeDifference < 3600 * 1000) && !file4Playing)
          {
            // Set the trigger time.
            triggered = timeDifference;
            Stop();
            file4Playing = true;
            Serial.print("Time trigger at: ");
            ShowTime();
            Serial.println("Playing file: 4");
            Serial.print("Start delay: ");
            Serial.println(startDelay);  
            FileOpenRead(&playFile, "4");
            Play(timeDifference + startDelay);                
          }
        }
        
        // Fifth track.    
        if ( track5_h > 0 )
        {
          timeDifference = CompareTime(track5_h, track5_m , track5_s, track5_f);
          startDelay = 0;
          if ((timeDifference >= 0) && (timeDifference < 3600 * 1000) && !file5Playing)
          {
            // Set the trigger time.
            triggered = timeDifference;
            Stop();
            file5Playing = true;
            Serial.print("Time trigger at: ");
            ShowTime();
            Serial.println("Playing file: 5");
            Serial.print("Start delay: ");
            Serial.println(startDelay);  
            FileOpenRead(&playFile, "5");
            Play(timeDifference + startDelay);        
          }
        }
        
        // Sixth track.    
        if ( track6_h > 0 )
        {
          timeDifference = CompareTime(track6_h, track6_m , track6_s, track6_f);
          startDelay = 0;
          if ((timeDifference >= 0) && (timeDifference < 3600 * 1000) && !file6Playing)
          {
            // Set the trigger time.
            triggered = timeDifference;
            Stop();
            file6Playing = true;
            Serial.print("Time trigger at: ");
            ShowTime();
            Serial.println("Playing file: 6");
            Serial.print("Start delay: ");
            Serial.println(startDelay);  
            FileOpenRead(&playFile, "6");
            Play(timeDifference + startDelay);        
          } 
        }     
      }               
    }
  }  
  else // if ( (Serial2.available()) )
  {
    // We haven't had any time updates so assume we have no time signal
    // if enough time has passed.
    if ( (lastTimeUpdate > lastTimePeriod) 
    && ((h + m + s + f) > 0) )
    {
      Serial.println("\nNo time signals received. Setting back to: 0:0:0:0");
      // We aren't getting time signals anymore. Reset the time.
      h = 0;
      m = 0;
      s = 0;
      f = 0;
      
      // Reset the counter.
      timeCodeCount = 0;
    } 
  }
}

////////////////////////////////////////////////////////////
// Decode midi.
////////////////////////////////////////////////////////////
bool DecodeMidi(void)
{
  bool validTime = false;
  int bytestoRead;
  
  bytestoRead = Serial2.available();

  for (int i = 0; i < bytestoRead; i++ )
  {
    buf[i] = Serial2.read();
    
    Serial.println(buf[i],HEX);
      
    if ( buf[i] == 0xF0 )
    {
      Serial.println("Full frame!");
    }

    if ( buf[i] != 0xF1 )
    {
      int indice;
      indice = (buf[i] & 0xf0) >> 4;

      if ( indice > 7 )
      {
        continue;
      }

      tc[indice] = buf[i] & 0x0f;

      if (indice == 7)
      {
        char toDisp[30];
        char typeStr[10];
        byte frameType;

        frameType = tc[7] & 0x06;

        h = (tc[7] & 0x01) * 16 + tc[6];
        m = tc[5] * 16 + tc[4];
        s = tc[3] * 16 + tc[2];
        f = tc[1] * 16 + tc[0];

        if (h > 23)  h = 23;
        if (m > 59)  m = 59;
        if (s > 59)  s = 59;
        if (f > 30)  f = 30;

        switch ( frameType )
        {
          case F24:
            strcpy( typeStr, "24 fps" ); break;
          case F25:
            strcpy( typeStr, "25 fps" ); break;
          case F30DF:
            strcpy( typeStr, "30df fps" ); break;
          case F30:
            strcpy( typeStr, "30 fps" ); break;
          default:
            strcpy( typeStr, "Error" ); break;
        }

        sprintf( toDisp, "TC %02d:%02d:%02d.%02d / %s", h, m, s, f, typeStr );
        Serial.println( toDisp );
        validTime = true;
      }
    }
  }
  return validTime;
}

////////////////////////////////////////////////////////////
// Decode time code.
////////////////////////////////////////////////////////////
bool DecodeTimeCode(void)
{
  bool validTime = false;
  byte data;
  int bytestoRead;
  bytestoRead = Serial2.available();

  // For now just dump out the data.
  for (int i = 0; i < bytestoRead; i++ )
  {
    data = Serial2.read();
    if ( data == '\n' )
    {
      // Print out each line.
      //Serial.print("timeCodeBuffer: ");  
      //Serial.println(timeCodeBuffer);      

      // We ignore codes until we have enough to 
      // ensure we get a clean signal.
      if ( timeCodeCount < ignoreTimeCodeCount )
      {
        timeCodeCount++;
        validTime = false;
        //Serial.print("timeCodeCount: ");
        //Serial.println(timeCodeCount);
      }
      else
      {
        // Get it into our time variables.
        // Do this in reverse. We need to terminate the 
        // buffer as we go so this is destructive.
        f = atoi(timeCodeBuffer + 9);
        timeCodeBuffer[8] = '\0';
        s = atoi(timeCodeBuffer + 6);
        timeCodeBuffer[5] = '\0';
        m = atoi(timeCodeBuffer + 3);
        timeCodeBuffer[2] = '\0';
        h = atoi(timeCodeBuffer);  

        // Else check the values are sensible.
        if ( (h <= 24) && (m <= 60) && (s <= 60) )
        { 
          validTime = true;
        }
      }
      // Done with the buffer now. Reset the index.
      timeCodeBufferIndex = 0;
    }
    else
    {
      // Check we haven't overflowed the buffer.
      if ( timeCodeBufferIndex < timeCodeBufferSize - 1 )
      {
        timeCodeBuffer[timeCodeBufferIndex] = data; 
        // Null terminate.
        timeCodeBuffer[timeCodeBufferIndex + 1] = 0;      
        timeCodeBufferIndex++;
      }
    } 
  }
  
  // Return if we have a valid time signal.
  return validTime;
}

////////////////////////////////////////////////////////////
// Show the current time.
////////////////////////////////////////////////////////////
void ShowTime()
{  
  Serial.print(h);
  Serial.print(":");
  Serial.print(m);
  Serial.print(":");
  Serial.print(s);
  Serial.print(":");
  Serial.println(f);  
}

////////////////////////////////////////////////////////////
// Show the set start time.
////////////////////////////////////////////////////////////
void ShowStartTime()
{  
  Serial.print(start_h);
  Serial.print(":");
  Serial.print(start_m);
  Serial.print(":");
  Serial.print(start_s);
  Serial.print(":");
  Serial.println(start_f);  
}
      
////////////////////////////////////////////////////////////
// Main loop function.
////////////////////////////////////////////////////////////
void loop(void)
{
  // Update the Bouncer.
  debouncerTest.update();
  debouncerStart.update();
  debouncerStop.update();

  // Handle the buttons.
  HandleButtons();

  // Handle the LED.
  if ( LEDFlash == true )
  {
    // Call the flasher.
    LEDFlasher();
  }

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

  // Handle test mode.
  if ( testMode == true )
  {
    TestMode();
  }

  // Handle time mode.
  if ( timeMode == true )
  {
    TimeMode();
  }

  // Switch based on the current mode.
  switch (state)
  {
    // No mode.
    case idling:
      {
        // Do nothing.
        break;
      }

    // Record wait mode.
    case record_wait:
      {        
        if ( gotRCValue == true )
        {
          HandleRCCommand();
        }
        // Waiting for start button.
        MoveServos();
        break;
      }
      
    // Record mode.
    case record:
      {
        if ( gotRCValue == true )
        {
          HandleRCCommand();
        }
        // Record the value.
        Recording();
        break;
      }

    // Add wait mode.
    case add_wait:
      {
        if ( gotRCValue == true )
        {
          HandleRCCommand();
        }
        // Waiting for start button.
        MoveServos();       
        break;
      }
      
    // Add mode.
    case add:
      {
        if ( gotRCValue == true )
        {
          HandleRCCommand();
        }
        // Add the value.
        Adding();
        break;
      }

    // Twerk wait mode.
    case twerk_wait:
      {
        if ( gotRCValue == true )
        {
          HandleRCCommand();
        }
        // Waiting for start button.
        MoveServos();
        break;
      }
      
    // Twerk mode.
    case twerk:
      {        
        if ( gotRCValue == true )
        {
          HandleRCCommand();
        }
        // Twerk the value.
        Twerking();
        break;
      }
      
    // Play wait mode.
    case play_wait:
      {
        break;
      }
      
    // Playback mode.
    case play:
      {
        Playing();
        break;
      }

    // Deletion confirm mode.
    case deleteFileConfirm:
      {
        // Do nothing.
        break;
      }

    // Unknown mode (should never happen).
    default:
      {
        Serial.println("Unknown mode!");
        break;
      }
  }
}
