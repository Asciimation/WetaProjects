//Author: Simon Jansen

const String version_string = "8";

// DMX library.
#include <DmxSimple.h>
#include "RunningAverage.h"

const int numberOfCells = 6;

const unsigned long sensorCountPeriod = 500;      // Period to count sensor readings over in mS.
const unsigned long coolDownPeriod = 1200;         // Period for faster cooling down of average.
const unsigned long sendDMXPeriod = 100;          // Period to send DMX values in mS.
elapsedMicros sensorReadTime = 0;                 // Sensor read elapsed time.  
elapsedMillis sensorCountTime = 0;                // Sensor count elapsed time.  
elapsedMillis coolDownTime = 0;                   // Cool down elapsed time.  
elapsedMillis sendDMXTime = 0;                    // Send DMX elapsed time.  

// DMX output pin.
const int DMXOutputPin = 1;

// Sensor pins and variables.
const int rubbingSensor1aPin = 3;                 // Sensor 1a input pin.
const int rubbingSensor1bPin = 4;                 // Sensor 1b input pin.
int rubbingSensor1aLastState = 0;                 // Sensor 1a last state.
int rubbingSensor1bLastState = 0;                 // Sensor 1b last state.
int rubbingCell1Count = 0;                        // Counter of how many rubs we had over a period.

const int rubbingSensor2aPin = 5;                 // Sensor 2a input pin.
const int rubbingSensor2bPin = 6;                 // Sensor 2b input pin.
int rubbingSensor2aLastState = 0;                 // Sensor 2a last state.
int rubbingSensor2bLastState = 0;                 // Sensor 2b last state.
int rubbingCell2Count = 0;                        // Counter of how many rubs we had over a period.

const int rubbingSensor3aPin = 7;                 // Sensor 3a input pin.
const int rubbingSensor3bPin = 8;                 // Sensor 3b input pin.
int rubbingSensor3aLastState = 0;                 // Sensor 3a last state.
int rubbingSensor3bLastState = 0;                 // Sensor 3b last state.
int rubbingCell3Count = 0;                        // Counter of how many rubs we had over a period.

const int rubbingSensor4aPin = 9;                 // Sensor 4a input pin.
const int rubbingSensor4bPin = 10;                // Sensor 4b input pin.
int rubbingSensor4aLastState = 0;                 // Sensor 4a last state.
int rubbingSensor4bLastState = 0;                 // Sensor 4b last state.
int rubbingCell4Count = 0;                        // Counter of how many rubs we had over a period.

const int rubbingSensor5aPin = 11;                // Sensor 5a input pin.
const int rubbingSensor5bPin = 12;                // Sensor 5b input pin.
int rubbingSensor5aLastState = 0;                 // Sensor 5a last state.
int rubbingSensor5bLastState = 0;                 // Sensor 5b last state.
int rubbingCell5Count = 0;                        // Counter of how many rubs we had over a period.

const int rubbingSensor6aPin = 13;                // Sensor 6a input pin.
const int rubbingSensor6bPin = 14;                // Sensor 6b input pin.
int rubbingSensor6aLastState = 0;                 // Sensor 6a last state.
int rubbingSensor6bLastState = 0;                 // Sensor 6b last state.
int rubbingCell6Count = 0;                        // Counter of how many rubs we had over a period.

// Min and max rub counts.
const int minRubCount = 0;
const int maxRubCount = 10;

// Running average variables.
const int periodsToAverageTotal = 20;                             // The periods to average. This times the sensorCountPeriod 
RunningAverage totalRunningAverage(periodsToAverageTotal);        // gives the time to average over.

const int periodsToAverageCells = 20;                             // The periods to average over for each cell.
RunningAverage cell1RunningAverage(periodsToAverageCells);  
RunningAverage cell2RunningAverage(periodsToAverageCells);  
RunningAverage cell3RunningAverage(periodsToAverageCells);  
RunningAverage cell4RunningAverage(periodsToAverageCells);  
RunningAverage cell5RunningAverage(periodsToAverageCells);  
RunningAverage cell6RunningAverage(periodsToAverageCells);  

// DMX channels.
const int DMXCellChannel1 = 81;
const int DMXCellChannel2 = 82;
const int DMXCellChannel3 = 83;
const int DMXCellChannel4 = 84;
const int DMXCellChannel5 = 85;
const int DMXCellChannel6 = 86;
const int DMXAverageChannel = 87;

// DMX value to send.
int DMXValue = 0;

// ----------------------------------------------------------------------
// Setup.
// ----------------------------------------------------------------------
void setup()   
{

  // Inputs.
  pinMode(rubbingSensor1aPin, INPUT);
  pinMode(rubbingSensor1bPin, INPUT);

  pinMode(rubbingSensor2aPin, INPUT);
  pinMode(rubbingSensor2bPin, INPUT);

  pinMode(rubbingSensor3aPin, INPUT);
  pinMode(rubbingSensor3bPin, INPUT);

  pinMode(rubbingSensor4aPin, INPUT);
  pinMode(rubbingSensor4bPin, INPUT);

  pinMode(rubbingSensor5aPin, INPUT);
  pinMode(rubbingSensor5bPin, INPUT);

  pinMode(rubbingSensor6aPin, INPUT);
  pinMode(rubbingSensor6bPin, INPUT);
  
  //Serial.begin(115200);
  
  // Pin used for DMX output.
  DmxSimple.usePin(DMXOutputPin);

  /* DMX devices typically need to receive a complete set of channels
  ** even if you only need to adjust the first channel. You can
  ** easily change the number of channels sent here. If you don't
  ** do this, DmxSimple will set the maximum channel number to the
  ** highest channel you DmxSimple.write() to. */
  DmxSimple.maxChannel(90);

  // Reset the elapsed millis counters.
  sensorReadTime = 0;
  sensorCountTime = 0;
  coolDownTime = 0;
  sendDMXTime = 0;

  // Reset the counters.
  rubbingCell1Count = 0;
  rubbingCell2Count = 0;
  rubbingCell3Count = 0;
  rubbingCell4Count = 0;
  rubbingCell5Count = 0;
  rubbingCell6Count = 0;

  // Clear the running averages.
  cell1RunningAverage.clear();
  cell2RunningAverage.clear();
  cell3RunningAverage.clear();
  cell4RunningAverage.clear();
  cell5RunningAverage.clear();
  cell6RunningAverage.clear();
  totalRunningAverage.clear();

  // Reset the states.
  rubbingSensor1aLastState = 0;
  rubbingSensor1bLastState = 0;
  rubbingSensor2aLastState = 0;
  rubbingSensor2bLastState = 0;
  rubbingSensor3aLastState = 0;
  rubbingSensor3bLastState = 0;
  rubbingSensor4aLastState = 0;
  rubbingSensor4bLastState = 0;
  rubbingSensor5aLastState = 0;
  rubbingSensor5bLastState = 0;
  rubbingSensor6aLastState = 0;
  rubbingSensor6bLastState = 0;

  // Clear the DMX value.
  DMXValue = 0;

  Serial.print("Bee Hornet controller started.  Version: ");  
  Serial.println(version_string);  
}

// ----------------------------------------------------------------------
// Check cell 1.
// ----------------------------------------------------------------------
void CheckCell1() 
{
  // Check the rubbing sensor pins for cell 1.
  int sensor1aTriggered = digitalRead(rubbingSensor1aPin);
  int sensor1bTriggered = digitalRead(rubbingSensor1bPin);

  // Check sensor triggers. Check if we had a state change.
  
  // Sensor a.
  if ( rubbingSensor1aLastState != sensor1aTriggered ) 
  {
    rubbingCell1Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Sensor b.
  if ( rubbingSensor1bLastState != sensor1bTriggered ) 
  {
    rubbingCell1Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Reset the last states.
  rubbingSensor1aLastState = sensor1aTriggered;
  rubbingSensor1bLastState = sensor1bTriggered;
}


// ----------------------------------------------------------------------
// Check cell 2.
// ----------------------------------------------------------------------
void CheckCell2() 
{
  // Check the rubbing sensor pins for cell 2.
  int sensor2aTriggered = digitalRead(rubbingSensor2aPin);
  int sensor2bTriggered = digitalRead(rubbingSensor2bPin);

  // Check sensor triggers. Check if we had a state change.
  
  // Sensor a.
  if ( rubbingSensor2aLastState != sensor2aTriggered ) 
  {    
    rubbingCell2Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Sensor b.
  if ( rubbingSensor2bLastState != sensor2bTriggered ) 
  {
    rubbingCell2Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Reset the last states.
  rubbingSensor2aLastState = sensor2aTriggered;
  rubbingSensor2bLastState = sensor2bTriggered;
}

// ----------------------------------------------------------------------
// Check cell 3.
// ----------------------------------------------------------------------
void CheckCell3() 
{
  // Check the rubbing sensor pins for cell 3.
  int sensor3aTriggered = digitalRead(rubbingSensor3aPin);
  int sensor3bTriggered = digitalRead(rubbingSensor3bPin);

  // Check sensor triggers. Check if we had a state change.
  
  // Sensor a.
  if ( rubbingSensor3aLastState != sensor3aTriggered ) 
  {    
    rubbingCell3Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Sensor b.
  if ( rubbingSensor3bLastState != sensor3bTriggered ) 
  {
    rubbingCell3Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Reset the last states.
  rubbingSensor3aLastState = sensor3aTriggered;
  rubbingSensor3bLastState = sensor3bTriggered;
}

// ----------------------------------------------------------------------
// Check cell 4.
// ----------------------------------------------------------------------
void CheckCell4() 
{
  // Check the rubbing sensor pins for cell 4.
  int sensor4aTriggered = digitalRead(rubbingSensor4aPin);
  int sensor4bTriggered = digitalRead(rubbingSensor4bPin);

  // Check sensor triggers. Check if we had a state change.
  
  // Sensor a.
  if ( rubbingSensor4aLastState != sensor4aTriggered ) 
  {    
    rubbingCell4Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Sensor b.
  if ( rubbingSensor4bLastState != sensor4bTriggered ) 
  {
    rubbingCell4Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Reset the last states.
  rubbingSensor4aLastState = sensor4aTriggered;
  rubbingSensor4bLastState = sensor4bTriggered;
}

// ----------------------------------------------------------------------
// Check cell 5.
// ----------------------------------------------------------------------
void CheckCell5() 
{
  // Check the rubbing sensor pins for cell 5.
  int sensor5aTriggered = digitalRead(rubbingSensor5aPin);
  int sensor5bTriggered = digitalRead(rubbingSensor5bPin);

  // Check sensor triggers. Check if we had a state change.
  
  // Sensor a.
  if ( rubbingSensor5aLastState != sensor5aTriggered ) 
  {    
    rubbingCell5Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Sensor b.
  if ( rubbingSensor5bLastState != sensor5bTriggered ) 
  {
    rubbingCell5Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Reset the last states.
  rubbingSensor5aLastState = sensor5aTriggered;
  rubbingSensor5bLastState = sensor5bTriggered;
}

// ----------------------------------------------------------------------
// Check cell 6.
// ----------------------------------------------------------------------
void CheckCell6() 
{
  // Check the rubbing sensor pins for cell 6.
  int sensor6aTriggered = digitalRead(rubbingSensor6aPin);
  int sensor6bTriggered = digitalRead(rubbingSensor6bPin);

  // Check sensor triggers. Check if we had a state change.
  
  // Sensor a.
  if ( rubbingSensor6aLastState != sensor6aTriggered ) 
  {    
    rubbingCell6Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }
  
  // Sensor b.
  if ( rubbingSensor6bLastState != sensor6bTriggered ) 
  {
    rubbingCell6Count++;
    // Reset the cool down timer.
    coolDownTime = 0;
  }

  // Reset the last states.
  rubbingSensor6aLastState = sensor6aTriggered;
  rubbingSensor6bLastState = sensor6bTriggered;
}

// ----------------------------------------------------------------------
// Main loop.
// ----------------------------------------------------------------------
void loop() 
{
  // Read the sensors.
  CheckCell1();
  CheckCell2();
  CheckCell3();
  CheckCell4();
  CheckCell5();
  CheckCell6();

  // Send DMX if enough time has passed.
  if ( sendDMXTime > sendDMXPeriod )
  {    
    // Reset the send DMX elapsed time.
    sendDMXTime = 0; 
     
    // Output the values.   
    Serial.print("Cell 1 average: ");    
    int cell1Average = cell1RunningAverage.getAverage();
    Serial.print(cell1Average);   
    if ( cell1Average > maxRubCount )
    {
      cell1Average = maxRubCount;
    }
    Serial.print("   Cell 1 average after limit: ");        
    Serial.print(cell1Average);     
    DMXValue = map(cell1Average, minRubCount, maxRubCount, 0, 255);
    
    DmxSimple.write(DMXCellChannel1, DMXValue);
    Serial.print("   Cell 1 DMX: ");
    Serial.println(DMXValue);

    Serial.print("Cell 2 average: ");    
    int cell2Average = cell2RunningAverage.getAverage();
    Serial.print(cell2Average);   
    if ( cell2Average > maxRubCount )
    {
      cell2Average = maxRubCount;
    }
    Serial.print("   Cell 2 average after limit: ");        
    Serial.print(cell2Average);     
    DMXValue = map(cell2Average, minRubCount, maxRubCount, 0, 255);
    
    DmxSimple.write(DMXCellChannel2, DMXValue);
    Serial.print("   Cell 2 DMX: ");
    Serial.println(DMXValue);
 
    Serial.print("Cell 3 average: ");    
    int cell3Average = cell3RunningAverage.getAverage();
    Serial.print(cell3Average);   
    if ( cell3Average > maxRubCount )
    {
      cell3Average = maxRubCount;
    }
    Serial.print("   Cell 3 average after limit: ");        
    Serial.print(cell3Average);     
    DMXValue = map(cell3Average, minRubCount, maxRubCount, 0, 255);
        
    DmxSimple.write(DMXCellChannel3, DMXValue);
    Serial.print("   Cell 3 DMX: ");
    Serial.println(DMXValue);

    Serial.print("Cell 4 average: ");    
    int cell4Average = cell4RunningAverage.getAverage();
    Serial.print(cell4Average);   
    if ( cell4Average > maxRubCount )
    {
      cell4Average = maxRubCount;
    }
    Serial.print("   Cell 4 average after limit: ");        
    Serial.print(cell4Average);     
    DMXValue = map(cell4Average, minRubCount, maxRubCount, 0, 255);
    
    DmxSimple.write(DMXCellChannel4, DMXValue);
    Serial.print("   Cell 4 DMX: ");
    Serial.println(DMXValue);    

    Serial.print("Cell 5 average: ");    
    int cell5Average = cell5RunningAverage.getAverage();
    Serial.print(cell5Average);   
    if ( cell5Average > maxRubCount )
    {
      cell5Average = maxRubCount;
    }
    Serial.print("   Cell 5 average after limit: ");        
    Serial.print(cell5Average);     
    DMXValue = map(cell5Average, minRubCount, maxRubCount, 0, 255);
    
    DmxSimple.write(DMXCellChannel5, DMXValue);
    Serial.print("   Cell 5 DMX: ");
    Serial.println(DMXValue);
  
    Serial.print("Cell 6 average: ");    
    int cell6Average = cell6RunningAverage.getAverage();
    Serial.print(cell6Average);   
    if ( cell6Average > maxRubCount )
    {
      cell6Average = maxRubCount;
    }
    Serial.print("   Cell 6 average after limit: ");        
    Serial.print(cell6Average);     
    DMXValue = map(cell6Average, minRubCount, maxRubCount, 0, 255);
    
    DmxSimple.write(DMXCellChannel6, DMXValue);
    Serial.print("   Cell 6 DMX: ");
    Serial.println(DMXValue);
    
    Serial.print("Average: ");
    Serial.print(totalRunningAverage.getAverage());

    // Send via DMX.
    DMXValue = map(totalRunningAverage.getAverage(), minRubCount, maxRubCount * numberOfCells, 0, 255);
    DmxSimple.write(DMXAverageChannel, DMXValue);
    Serial.print("   Overall DMX: ");
    Serial.print(DMXValue);

    Serial.print(" ");
    for ( int i = 0; i < int(totalRunningAverage.getAverage()); i++ )
    {
      if ( (i % 4) == 0 )
      {
        Serial.print("+");      
      }
    }
    Serial.println("\n");  
  }
  
  // Update the averages if enough time has passed.
  if ( sensorCountTime > sensorCountPeriod )
  {       
    // Reset the count elapsed time.
    sensorCountTime = 0; 

    // Add to the averages for each cell.
    cell1RunningAverage.addValue(rubbingCell1Count);
    cell2RunningAverage.addValue(rubbingCell2Count);
    cell3RunningAverage.addValue(rubbingCell3Count);
    cell4RunningAverage.addValue(rubbingCell4Count);
    cell5RunningAverage.addValue(rubbingCell5Count);
    cell6RunningAverage.addValue(rubbingCell6Count);

    // Maintain the total running average.
    totalRunningAverage.addValue(rubbingCell1Count + rubbingCell2Count + rubbingCell3Count + rubbingCell4Count + rubbingCell5Count + rubbingCell6Count);  

    // Reset the counts.
    rubbingCell1Count = 0;
    rubbingCell2Count = 0;
    rubbingCell3Count = 0;
    rubbingCell4Count = 0;
    rubbingCell5Count = 0;
    rubbingCell6Count = 0;
  }

  // Cool down handler. 
  // We add extra zeros to quickly lower the average if there has been no activity for a while.
  if ( coolDownTime > coolDownPeriod )
  {
    // Maintain the running average.
    totalRunningAverage.addValue(0);
    
    // Reset the count elapsed time.
    coolDownTime = 0; 
  }   
   
}



