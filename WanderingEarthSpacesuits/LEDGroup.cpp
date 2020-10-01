// LEDGroup library. 
// Provides a way to create LED objects that can then be manipulated using
// the FastLED library.

#include "Arduino.h"
#include "LEDGroup.h"


// ----------------------------------------------------------------------
// LED groups class.
// ----------------------------------------------------------------------

// Constructor.
LEDGroup::LEDGroup( const char* name, CRGB* LEDArray, int fromLED, int toLED )
{
  // Name. 
  for ( int i = 0; i < max_string; i++ )
  {
    _name[i] = 0; 
  }
  strncpy( _name, name, max_string );
  
  // Store which strip we are on. 
  _LEDArray = LEDArray;
  
  // Work out the max and min.
  _minLEDAddress = min(fromLED, toLED);
  _maxLEDAddress = max(fromLED, toLED);
}

// Destructor.
LEDGroup::~LEDGroup()
{
}

// Get name.
char* LEDGroup::GetName(void)
{
  return _name;
}

// Set colour using RGB values.
void LEDGroup::SetRGB( int r, int g, int b )
{
  Serial.print("Setting RGB: ");
  Serial.print(r);
  Serial.print(", ");
  Serial.print(g);
  Serial.print(", ");
  Serial.print(b);      
  Serial.print(" for group: ");
  Serial.print(GetName());
  
  // Check the values.
  if ( (r < 0) || (r > 255) ) return;
  if ( (g < 0) || (g > 255) ) return;
  if ( (b < 0) || (b > 255) ) return;
  
  // Iterate and set colour for each.
  for ( int i = _minLEDAddress; i <= _maxLEDAddress; i++ )
  {
	  _LEDArray[i].setRGB(r, g, b);    
  }
  Serial.println(" RGB set."); 
  _red = r;
  _green = g;
  _blue = b;
  _hue = -1;
  _saturation = -1;
  _value = -1;
  // Display change.
  FastLED.show(); 
}

// Set colour using HSV values.
void LEDGroup::SetHSV( int h, int s, int v )
{
  Serial.print("Setting HSV: ");
  Serial.print(h);
  Serial.print(", ");
  Serial.print(s);
  Serial.print(", ");
  Serial.print(v);
  Serial.print(" for group: ");
  Serial.print(GetName());
  
  // Check the values.
  if ( (h < 0) || (h > 255) ) return;
  if ( (s < 0) || (s > 255) ) return;
  if ( (v < 0) || (v > 255) ) return;
  
  // Iterate and set colour for each.
  for ( int i = _minLEDAddress; i <= _maxLEDAddress; i++ )
  {
    _LEDArray[i].setHSV(h, s, v);    
  }
  Serial.println(" HSV set.");
  _red = -1;
  _green = -1;
  _blue = -1;
  _hue = h;
  _saturation = s;
  _value = v; 
  // Display change.
  FastLED.show();
}

// Turn on.
void LEDGroup::On()
{
  Serial.print("Turning on LEDs for group: ");
  Serial.println(GetName());
  // Iterate and set value for each LED.
  for ( int i = _minLEDAddress; i <= _maxLEDAddress; i++ )
  {
   _LEDArray[i].setHSV(_hue, _saturation, _value);
  }
  // Display change.
  FastLED.show(); 
}

// Turn off.
void LEDGroup::Off()
{
  Serial.print("Turning off LEDs for group: ");
  Serial.println(GetName());
  // Iterate and set value for each LED.
  for ( int i = _minLEDAddress; i <= _maxLEDAddress; i++ )
  {
   _LEDArray[i].setHSV(_hue, _saturation, 0);
  }
  FastLED.show();
}

// Set bargraph across group going up.
void LEDGroup::SetBargraphUp( int percentage )
{
  Serial.print("Setting bargraph up percentage to: ");
  Serial.print(percentage);
  Serial.print("  for group: ");
  Serial.println(GetName());
  
  // Check the values.
  if ( (percentage < 0) || (percentage > 100) ) return;

  // Work out how many LEDs need to be lit.
  int numberOfLEDs = _maxLEDAddress - _minLEDAddress + 1;

  float fvalue = ((float)percentage / 100) * float(numberOfLEDs);

  int numberOfLEDsToLight = (int)(fvalue);
  Serial.print("Light ");
  Serial.print(numberOfLEDsToLight);
  Serial.print(" out of ");
  Serial.print(numberOfLEDs);
  Serial.println(" LEDs");
  
  // Iterate and set all to off.
  for ( int i = _minLEDAddress; i <= _maxLEDAddress; i++ )
  {
    _LEDArray[i].setRGB(0, 0, 0);
  }  
  
  // Iterate and set colour for each based on the last updated 
  // colour that was set.
  for ( int i = _minLEDAddress; i < _minLEDAddress + numberOfLEDsToLight; i++ )
  { 
    // If we have RGB values use them.
    if ( _red != -1 )
    {
      _LEDArray[i].setRGB(_red, _green, _blue);      
    }
    // Else if we have HSV values use them
    else if ( _hue != -1 )
    {
      _LEDArray[i].setHSV(_hue, _saturation, _value);
    }
    // Else no colour has been set yet.
    else
    {
      Serial.println("No colour previously set on LEDs.");
    }  
  }
  // Display change.
  FastLED.show();     
}

// Set bargraph across group going down.
void LEDGroup::SetBargraphDown( int percentage )
{
  Serial.print("Setting bargraph down percentage to: ");
  Serial.print(percentage);
  Serial.print(" for group: ");
  Serial.println(GetName());
  
  // Check the values.
  if ( (percentage < 0) || (percentage > 100) ) return;

  // Work out how many LEDs need to be lit.
  int numberOfLEDs = _maxLEDAddress - _minLEDAddress + 1;

  float fvalue = ((float)percentage / 100) * float(numberOfLEDs);

  int numberOfLEDsToLight = (int)(fvalue);
  Serial.print("Light ");
  Serial.print(numberOfLEDsToLight);
  Serial.print(" out of ");
  Serial.print(numberOfLEDs);
  Serial.println(" LEDs");
  
  // Iterate and set all to off.
  for ( int i = _minLEDAddress; i <= _maxLEDAddress; i++ )
  {
    _LEDArray[i].setRGB(0, 0, 0);
  }  
  
  // Iterate and set colour for each based on the last updated 
  // colour that was set.
  for ( int i = _maxLEDAddress; i > _maxLEDAddress - numberOfLEDsToLight ; i-- )
  { 
    // If we have RGB values use them.
    if ( _red != -1 )
    {
      _LEDArray[i].setRGB(_red, _green, _blue);
    }
    // Else if we have HSV values use them
    else if ( _hue != -1 )
    {
      _LEDArray[i].setHSV(_hue, _saturation, _value);
    }
    // Else no colour has been set yet.
    else
    {
      Serial.println("No colour previously set on LEDs.");
    }  
  }
  // Display change.
  FastLED.show();     
}
// ----------------------------------------------------------------------
// End class.
// ----------------------------------------------------------------------
