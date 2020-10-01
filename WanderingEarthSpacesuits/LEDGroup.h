// LEDGroup library. 
// Provides a way to create LED objects that can then be manipulated using
// the FastLED library.

#ifndef LEDGroup_h
#define LEDGroup_h

#include "Arduino.h"
#include "FastLED.h"

const int max_string = 32;

// ----------------------------------------------------------------------
// LED groups class.
// ----------------------------------------------------------------------
class LEDGroup
{
  public:
    LEDGroup( const char* name, CRGB* LEDArray, int fromLED, int toLED );
    ~LEDGroup();
    char* GetName();
    void SetRGB( int red, int green, int blue );
    void SetHSV( int hue, int saturation, int value );
    void On();
    void Off();
    void SetBargraphUp( int percentage );
    void SetBargraphDown( int percentage );
    
  private:
    char _name[max_string];
    CRGB* _LEDArray;
    int _minLEDAddress = 0;
    int _maxLEDAddress = 0;
    int _red = 0;
    int _green = 0;
    int _blue = 0;
    int _hue = -1;
    int _saturation = -1;
    int _value = -1;
	
};

// ----------------------------------------------------------------------
// End class.
// ----------------------------------------------------------------------

#endif
