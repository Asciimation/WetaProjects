// WetaServo library.
// Simon Jansen
// November 2015
// Library to be used with the Weta Teensy servo driver.
# include <math.h>
# include <WetaServo.h>


# define TURNSACDCTABLESTEPS  10
const int TurnsACDCLookupTable[] = 
{
	10, 20, 30, 40, 50, 
	60, 70, 80, 90, 100
};

# define ACDCTABLESTEPS  128
// SIN table.
const int ACDCLookupTable[] = 
{
	1,  1,  1,  1,  1,  1,  1,  1,  1,	2,
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

# define ACDCSERVOTABLESTEPS  10

const int ACDCServoLookupTable[] = 
{
	10, 20, 30, 40, 50, 
	60, 70, 80, 90, 100
};

/*
# define ACDCTABLESTEPS  100
// Linear table.
const int ACDCLookupTable[] = 
{
	 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
	10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
	30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
	50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
	60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
	70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
	80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
	90, 91, 92, 93, 94, 95, 96, 97, 98, 99
};
*/


////////////////////////////////////////////////////////////
// StandardServo class.
////////////////////////////////////////////////////////////  

// Constructor.
StandardServo::StandardServo( byte pin )
{
  this->pin = pin;   // Hardware pin this servo is on.
  ser.attach(pin);   // Attach the servo.
  minPos = 0;        // Set default minimum. 
  maxPos = 180;      // Set default maximum.
}

////////////////////////////////////////////////////////////
// Move to given position.
////////////////////////////////////////////////////////////  
void StandardServo::MoveTo( byte pos )
{
  // Check we are within the min and max values.
  if ( (pos >= minPos) && (pos <= maxPos) )
  {
    // Move the servo.
    ser.write(pos);    
  }
  else
  {
	Serial.print("Position: ");
	Serial.print(pos);
	Serial.print(" outside range of: ");
	Serial.print(minPos);
	Serial.print(" to: ");
	Serial.println(maxPos);
  }
}

////////////////////////////////////////////////////////////
// Set min and max positions.
////////////////////////////////////////////////////////////  
void StandardServo::SetMinMax( byte min, byte max )
{
  minPos = min;
  maxPos = max;
}

////////////////////////////////////////////////////////////
// DynamixelServo class.
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// Constructor.
////////////////////////////////////////////////////////////  
DynamixelServo::DynamixelServo(byte id, const char* name, const char* cw_string, const char* ccw_string)
{
  this->id = id;    // Servo ID.
  
  // Name. 
  strncpy( this->name, name, MAX_STRING );
  // CW string.
  strncpy( this->cw_string, cw_string, MAX_STRING );
  // CCW string.
  strncpy( this->ccw_string, ccw_string, MAX_STRING );
  
  // Servo limits.
  cw_limit = CW_LIMIT_ANGLE;
  ccw_limit = CCW_LIMIT_ANGLE;
  
  // Time scale (percentage).
  time_scale = TIME_SCALE_NORM;
  
  // Direction.
  direction = DLEFT;
  
  // Speed.
  cw_speed = 0;
  ccw_speed = 0;
  
  // Enabled by default.
  enabled = true;
  
  // Acceleration.
  acdc = 0;
  
  // Positions.
  initialPosition = 0;
  goalPosition = 0;
  newPosition = 0;
  oldPosition = 0;
  
  // Number of steps.
  steps = 0;
  stepsToRun = 0;
  totalSteps = 0;
  
  // Number of full turns done.
  turns = 0;
  turnsToRun = 0;
  
  // Set the start time to now.  
  timeStart = micros();
  timeToRun = 0;
  
  // Doesn't need servicing.
  needsService = false;
  serviceMode = MNONE;
  
  // Reset the lookup table index and
  // acceleration variables.
  lookupTableIndex = 0;
  timeAccDcc = 0;
  decStart = 0;
  accelerating = false;
  decelerating = false;
  atMaxSpeed = false;
  increment = 0;
  acdcStopActual = 0;
  decStartActual = 0;
  
}

////////////////////////////////////////////////////////////
// Private functions.
////////////////////////////////////////////////////////////  

////////////////////////////////////////////////////////////
// Return scaled time value.
////////////////////////////////////////////////////////////  
unsigned long DynamixelServo::ScaleTime(unsigned long tm)
{	
  unsigned long temp = (tm * 100) / time_scale;
  return temp;    
}

////////////////////////////////////////////////////////////
// Return scaled speed value.
////////////////////////////////////////////////////////////  
unsigned long DynamixelServo::ScaleSpeed(int spd)
{
  unsigned long temp = spd * time_scale; 
  temp = temp / 100; 
  if ( temp <= MIN_SPEED )
  {
	  temp = MIN_SPEED;
  }
  if ( temp >= MAX_SPEED )
  {
	  temp = MAX_SPEED;
  }
  return temp;    
}

////////////////////////////////////////////////////////////
// Check for wrapping.
////////////////////////////////////////////////////////////  
bool DynamixelServo::Wrapped(int value, int oldValue)
{
  // Have we wrapped going up? 
  // Check if we have passed from the last quarter to the first.      
  if ( (value < oldValue) && (oldValue > 3072) && (value < 1024) )
  {
      return true;
  }
  // Have we wrapped going down? 
  // Check if we have passed from the first quarter to the last.      
  else if ( (value > oldValue) && (oldValue < 1024) && (value > 3072) )
  {
      return true;
  }
  else
  {
    return false;
  }
}

////////////////////////////////////////////////////////////
// Start acceleration.
////////////////////////////////////////////////////////////  
void DynamixelServo::StartAcc()
{
	lookupTableIndex = 0;
	accelerating = true;
	decelerating = false;
}

////////////////////////////////////////////////////////////
// Start deceleration.
////////////////////////////////////////////////////////////  
void DynamixelServo::StartDcc()
{
	lookupTableIndex = 0;
	accelerating = false;
	decelerating = true;
	decStartActual = micros();
}

////////////////////////////////////////////////////////////
// Stop acceleration.
////////////////////////////////////////////////////////////  
void DynamixelServo::StopAccDcc()
{
	lookupTableIndex = 0;
	accelerating = false;
	decelerating = false;
	acdcStopActual = micros();
}

////////////////////////////////////////////////////////////
// Get the next speed step from the acceleration 
// lookup table.
////////////////////////////////////////////////////////////  
int DynamixelServo::NextStep()
{
	int returnValue = 0;
	
	// If accelerating go up through the table.
	if ( (accelerating == true) && (decelerating == false) )
	{		
		// Get the value.
		returnValue = ACDCLookupTable[lookupTableIndex];		
	}
	// if decelerating go back down through the table.
	else if ( (accelerating == false) && (decelerating == true) )
	{	
		// Get the value.
		returnValue = ACDCLookupTable[(ACDCTABLESTEPS - 1) - lookupTableIndex];			
	}
	else
	{
		// Neither accelerating or decelerating so 
		// return maximum.
		returnValue = 100;
	}
	
	// Increment the index into the table.
	if ( lookupTableIndex < ACDCTABLESTEPS - 1)
	{			
		lookupTableIndex++;
	}
	//Serial.print("Index: ");
	//Serial.print(lookupTableIndex);	
	//Serial.print("   NextStep: ");
	//Serial.println(returnValue);
	return returnValue;
}

////////////////////////////////////////////////////////////
// Get the next speed step from the acceleration 
// lookup table for servos.
////////////////////////////////////////////////////////////  
int DynamixelServo::NextServoStep()
{
	int returnValue = 0;
	
	// If accelerating go up through the table.
	if ( (accelerating == true) && (decelerating == false) )
	{		
		// Get the value.
		returnValue = ACDCServoLookupTable[lookupTableIndex];		
	}
	// if decelerating go back down through the table.
	else if ( (accelerating == false) && (decelerating == true) )
	{	
		// Get the value.
		returnValue = ACDCServoLookupTable[(ACDCSERVOTABLESTEPS - 1) - lookupTableIndex];			
	}
	else
	{
		// Neither accelerating or decelerating so 
		// return maximum.
		returnValue = 100;
	}
	
	// Increment the index into the table.
	if ( lookupTableIndex < ACDCSERVOTABLESTEPS - 1)
	{			
		lookupTableIndex++;
	}
	//Serial.print("Index: ");
	//Serial.print(lookupTableIndex);	
	//Serial.print("   NextStep: ");
	//Serial.println(returnValue);
	return returnValue;
}

////////////////////////////////////////////////////////////
// If in servo mode set the position and speed.
// We can handle immediate or prload commands.
////////////////////////////////////////////////////////////  
void DynamixelServo::DoServo(int pos, bool preLoad)
{  
    //Serial.print("---- Servo command. Position: ");
	//Serial.println(pos);
	//Serial.print("Id: ");
	//Serial.println(id);
	
	int scaled_speed = 0;
	
	// Store the goal position.
	goalPosition = pos;
	
	if ( this->enabled == true )
	{
		if (this->mode == SERVO)
		
			// If there is any acceleration/deceleration.
			if ( this->acdc > 0)
			{						
				// Set up the initial position.
				initialPosition = Dynamixel.readPosition(id);
				
				// Get how many steps we are running.
				stepsToRun = abs(goalPosition - initialPosition);
				
				// Reset number of steps actually run.
				steps = 0;
				
				// Based on the acdc value get how many steps
				// we spend accelerating/decelerating.
				//Serial.print("this->acdc: ");
				//Serial.println(this->acdc);
				float multiplier = (float)this->acdc / 100;
				//Serial.print("multiplier: ");
				//Serial.println(multiplier);
				stepsAccDcc = stepsToRun * multiplier; 						
				//Serial.print("stepsAccDcc: ");
				//Serial.println(stepsAccDcc);
				// Divide the steps into two, half accelerating
				// and half decelerating.
				stepsAccDcc = stepsAccDcc / 2;
				
				// Work out the increments for speed changes.
				increment = stepsAccDcc/ACDCSERVOTABLESTEPS;
				
				// Work out after how many steps to start decelerating.	
				decStart = stepsToRun - stepsAccDcc;
			
				// Set the mode and flag.
				serviceMode = MSERVO;
				needsService = true;
				// Speed.
				atMaxSpeed = false;	
				
				// And update the start time to now.
				timeStart = micros();			
				
				/*		
				Serial.print("initialPosition: ");
				Serial.println(initialPosition);
				Serial.print("Goal position: ");
				Serial.println(goalPosition);
				Serial.print("Steps to move: ");
				Serial.println(stepsToRun);
				Serial.print("Steps to acc/dcc: ");
				Serial.println(stepsAccDcc);
				Serial.print("increment: ");
				Serial.println(increment);
				*/
				
				// Set the start flag.				
				StartAcc();
				
				// If the position to move to is less than the current we are going clockwise.
				if (goalPosition < initialPosition)
				{
					//Serial.println("Moving clockwise.");
					scaled_speed = ScaleSpeed(cw_speed);
				}
				// If the position to move to is greater than the current we are going counter clockwise.
				else if (goalPosition > initialPosition)
				{
					//Serial.println("Moving counter-clockwise.");
					scaled_speed = ScaleSpeed(ccw_speed);
				}
				
				// Command the servo. In servo mode we must
				// manually start it moving!
				int startSpeed = (scaled_speed * ACDCServoLookupTable[0])/100;
				if ( !preLoad )
				{
					//Serial.println("Initiating servo command.");
					Dynamixel.servo(id, goalPosition, startSpeed);			
				}
				else
				{
					//Serial.println("Initiating servo preload command.");
					Dynamixel.servoPreload(id, goalPosition, startSpeed);			
				}
				//Serial.print("Start speed: ");
				//Serial.println(startSpeed);	
									
			}
			// Else there is no acceleration/deceleration so we just move the 
			// servo.
			else				
			{  				
				// Check which direction we are going.
				int position = Dynamixel.readPosition(id);
				// If the position to move to is less than the current we are going clockwise.
				if (goalPosition < position)
				{
					//Serial.println("Moving clockwise.");
					scaled_speed = ScaleSpeed(cw_speed);
				}
				// If the position to move to is greater than the current we are going counter clockwise.
				else if (goalPosition > position)
				{
					//Serial.println("Moving counter-clockwise.");
					scaled_speed = ScaleSpeed(ccw_speed);
				}
				// If they are the same we don't need to more at all.
				else
				{
					//Serial.println("Already in position.");
					return;
				}		
				//Serial.print("Scaled Speed: ");
				//Serial.println(scaled_speed);
				if ( !preLoad )
				{
					//Serial.print("Initiating servo command.");
					Dynamixel.servo(id, goalPosition, scaled_speed);			
				}
				else
				{
					//Serial.print("Initiating servo preload command.");
					Dynamixel.servoPreload(id, goalPosition, scaled_speed);			
				}
			}			
		else
		{
			Serial.println("Not in SERVO mode.");	
		}
	}
	else
	{
		Serial.println("Not enabled.");	
	}
}

////////////////////////////////////////////////////////////
// Public functions.
//////////////////////////////////////////////////////////// 

////////////////////////////////////////////////////////////
// Start function. Called after setting up Dynamixels.
////////////////////////////////////////////////////////////  
void DynamixelServo::Start(byte mode, int cw_speed, int ccw_speed, int acdc, int cw_limit, int ccw_limit)
{
	Serial.print("Starting Dynamixel: ");
	Serial.println(id);
	this->mode = mode;
	this->cw_speed = cw_speed;
	this->ccw_speed = ccw_speed;
	this->acdc = acdc;
	this->cw_limit = cw_limit;
	this->ccw_limit = ccw_limit;
	Dynamixel.setBaudRate(id, SERVO_SET_Baudrate );
	delay(250);		
	Dynamixel.setMode(id, this->mode, this->cw_limit, this->ccw_limit);
	delay(250);
	Serial.print("CW Speed: ");
	Serial.println(this->cw_speed);
	Serial.print("CCW Speed: ");
	Serial.println(this->ccw_speed);
	Serial.print("Acceleration: ");
	Serial.println(this->acdc);
	Serial.print("CW Limit: ");
	Serial.println(this->cw_limit);
	Serial.print("CCW Limit: ");
	Serial.println(this->ccw_limit);
		
	// Get the initial position.    
	initialPosition = Dynamixel.readPosition(id);
	newPosition = initialPosition;
	oldPosition = initialPosition;
	Serial.print("Initial position: ");
	Serial.println(initialPosition);
	
	// Set the acceleration.
	if ( mode == SERVO )
	{
		int acceleration = acdc;
		if ( acceleration > 100 )
		{	
			acceleration = 100;
		}
		//Serial.print("In SERVO mode setting acceleration to: ");
		//Serial.println(acceleration);
		Dynamixel.setAcceleration(id, acceleration);
	}
}

////////////////////////////////////////////////////////////
// Set the drive mode.
////////////////////////////////////////////////////////////  
bool DynamixelServo::SetDriveMode(byte drive_mode)
{
	Serial.print("Setting drive mode to: ");
	Serial.println(drive_mode);
	if ( (drive_mode == 0) || (drive_mode == 1) || (drive_mode == 2) || (drive_mode == 3) )
	{
		int returnValue = Dynamixel.setDriveMode(id, drive_mode);
		delay(250);
		if ( returnValue != id )
		{
			Serial.print("Error:");
			Serial.println(returnValue);
			return false;		
		}	
		Serial.println("New drive mode value set.");
		return true;
	}
	else
	{
		Serial.print("Drive mode: ");
		Serial.print(drive_mode);
		Serial.println(" is not a valid value (0, 1, 2 or 3)");
		return false;			
	}
}

////////////////////////////////////////////////////////////
// Set the PID.
////////////////////////////////////////////////////////////  
bool DynamixelServo::SetPID(byte P, byte I, byte D)
{
	if ( (P < 0) || (P > 255) )
	{
	  	Serial.print("P value: ");
		Serial.print(P);
		Serial.println(" out of range (0 - 255)" );
		return false;
	}
	
	if ( (I < 0) || (I > 255) )
	{
	  	Serial.print("I value: ");
		Serial.print(I);
		Serial.println(" out of range (0 - 255)" );
		return false;
	}
	
	if ( (D < 0) || (D > 255) )
	{
	
	  	Serial.print("D value: ");
		Serial.print(D);
		Serial.println(" out of range (0 - 255)" );
		return false;
	}
	
	int returnValue = Dynamixel.setPID(id, P, I, D);
	delay(250);
	if ( returnValue != id )
	{
		Serial.print("Error:");
		Serial.println(returnValue);
		return false;		
	}	
	Serial.println("New PID values set.");
	return true;
}
	
////////////////////////////////////////////////////////////
// Enable the Dynamixel.
////////////////////////////////////////////////////////////  
void DynamixelServo::Enable(bool enabled)
{
	if ( enabled )
	{
		Serial.print("Dynamixel enabled.");
		this->enabled = true;
	}
	else
	{
		Serial.print("Dynamixel disabled.");
		this->enabled = false;
	}
}	

////////////////////////////////////////////////////////////
// Set the clockwise speed.
////////////////////////////////////////////////////////////  
void DynamixelServo::SetCWSpeed(int speed)
{
	//Serial.print("Setting Dynamixel CW speed: ");
	//Serial.println(speed);
	this->cw_speed = speed;
}

////////////////////////////////////////////////////////////
// Set the counter clockwise speed.
////////////////////////////////////////////////////////////  
void DynamixelServo::SetCCWSpeed(int speed)
{
	//Serial.print("Setting Dynamixel CCW speed: ");
	//Serial.println(speed);
	this->ccw_speed = speed;
}

////////////////////////////////////////////////////////////
// Set the direction.
////////////////////////////////////////////////////////////  
void DynamixelServo::SetDirection(byte dir)
{
	/*
	Serial.print("Setting Dynamixel direction: ");
	if (dir == DLEFT)
	{
	  Serial.println("LEFT");
	}
	if (dir == DRIGHT)
	{
	  Serial.println("RIGHT");
	}
	*/
	direction = dir;
}

////////////////////////////////////////////////////////////
// Set the servo CW limit.
////////////////////////////////////////////////////////////  
bool DynamixelServo::SetCWLimit(int cw_limit)
{
	Serial.print("Setting CW limit for ID: ");
	Serial.print(id);
	Serial.print(" to: ");
	Serial.println(cw_limit);
	
	// Only works in servo mode.
	if ( mode == WHEEL )
	{
		Serial.println("Not in servo mode!");
		return false;
	}
		
	// Limits must be within the max and minimum.
	if ( cw_limit < CW_LIMIT_ANGLE )
	{
		// Limits out of range.
		Serial.println("Limit out of range.");
		return false;		
	}
		
	// Set the new limits.
	this->cw_limit = cw_limit;	
	int returnValue = Dynamixel.setMode(id, mode, this->cw_limit, this->ccw_limit);
	delay(250);
	if ( returnValue != id )
	{
		Serial.print("Error:");
		Serial.println(returnValue);
		return false;		
	}	
	Serial.println("New limit set.");
	return true;
}

////////////////////////////////////////////////////////////
// Set the servo CCW limit.
////////////////////////////////////////////////////////////  
bool DynamixelServo::SetCCWLimit(int ccw_limit)
{
	Serial.print("Setting CCW limit for ID: ");
	Serial.print(id);
	Serial.print(" to: ");
	Serial.println(ccw_limit);
	
	// Only works in servo mode.
	if ( mode == WHEEL )
	{
		Serial.println("Not in servo mode!");
		return false;
	}
	
	// Limits must be within the max and minimum.
	if ( ccw_limit >  CCW_LIMIT_ANGLE )
	{
		// Limits out of range.
		Serial.println("Limit out of range.");
		return false;		
	}
		
	// Set the new limits.
	this->ccw_limit = ccw_limit;	
	int returnValue = Dynamixel.setMode(id, mode, this->cw_limit, this->ccw_limit);
	delay(250);
	if ( returnValue != id )
	{
		Serial.print("Error:");
		Serial.println(returnValue);
		return false;		
	}	
	Serial.println("New limit set.");
	return true;
}

////////////////////////////////////////////////////////////
// Set the acceleration.
////////////////////////////////////////////////////////////  
void DynamixelServo::SetACDC(int acdc)
{
	//Serial.print("Setting Dynamixel acceleration. ");
	int acceleration = acdc;
	
	// If the mode is SERVO mode we use a percentage of travel
	// as the acceleration/deceleration amount
	if ( mode == SERVO )
	{
		// Make sure it is from 0 to 100.	
		if ( acceleration < 0 )
		{	
			acceleration = 0;
		}
		if ( acceleration > 100 )
		{	
			acceleration = 100;
		}
		//Serial.print("In SERVO mode setting acceleration % to: ");		
	}
	else // WHEEL mode.
	// Else we set a time.
	{
		//Serial.print("In WHEEL mode setting acceleration time to: ");		
	}
	//Serial.println(acceleration);	
	this->acdc = acceleration;
}

////////////////////////////////////////////////////////////
// Set mode to WHEEL or SERVO.
// The CW and CCW limits are set to maximums. 
// Call the SetCWLimit and SetCCWLimit functions to 
// specify new limits.
////////////////////////////////////////////////////////////  
void DynamixelServo::SetMode(int mode)
{
	Serial.print("Changing mode for ID: ");
	Serial.print(id);
	Serial.print(" to: ");	
	if ( mode == WHEEL )
	{
		this->mode = WHEEL;
		Serial.println("WHEEL");
	}
	else if ( mode == SERVO )
	{
		this->mode = SERVO;
		Serial.println("SERVO");
	}
	else
	{
		this->mode = SERVO;
		Serial.println("SERVO");
	}		
	Dynamixel.setMode(id, this->mode, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);
	
	// Reset the total number of steps.
	totalSteps = 0;
	// Reset any acceleration to zero.
	this->acdc = 0;
}

////////////////////////////////////////////////////////////
// Set the holding torque.
////////////////////////////////////////////////////////////  
void DynamixelServo::SetHolding(bool hold)
{
	//Serial.print("Setting Dynamixel holding torque for ID: ");
	//Serial.print(id);
	if ( hold ) 
	{
		//Serial.println(" ON");
	}
	else
	{
		//Serial.println(" OFF");	
	}
	Dynamixel.setHoldingTorque(id, hold);
}

/////////////////////////////////////////////////////////////
// Set the time scaling.
////////////////////////////////////////////////////////////  
void DynamixelServo::SetTimeScale(int scale)
{
	if ( (scale >= TIME_SCALE_MIN) && (scale <= TIME_SCALE_MAX) )
	{
		time_scale = scale;
		//Serial.print("Setting time scale to: ");
		//Serial.print(this->time_scale);
		//Serial.print(" for ID: ");
		//Serial.println(this->id);
	}	
}

////////////////////////////////////////////////////////////
// Get the servo ID.
////////////////////////////////////////////////////////////  
byte DynamixelServo::GetID()
{
	return id;
}

////////////////////////////////////////////////////////////
// Get the servo name.
////////////////////////////////////////////////////////////  
char* DynamixelServo::GetName(void)
{
	//Serial.print("Getting name for ID: ");
	//Serial.println(this->id);
	return name;
}

////////////////////////////////////////////////////////////
// Get the CW string.
////////////////////////////////////////////////////////////  
char* DynamixelServo::GetCWString(void)
{
	//Serial.print("Getting CW string for: ");
	//Serial.println(this->id);
	return cw_string;
}

////////////////////////////////////////////////////////////
// Get the CCW string.
////////////////////////////////////////////////////////////  
char* DynamixelServo::GetCCWString(void)
{
	//Serial.print("Getting CCW string for: ");
	//Serial.println(this->id);
	return ccw_string;
}

////////////////////////////////////////////////////////////
// Get the servo mode.
////////////////////////////////////////////////////////////  
int DynamixelServo::GetMode()
{
	return mode;
}

////////////////////////////////////////////////////////////
// Get the servo clockwise speed setting.
////////////////////////////////////////////////////////////  
int DynamixelServo::GetCWSpeed(void)
{
	//Serial.print("Getting speed for ID: ");
	//Serial.println(this->id);
	return cw_speed;
}

////////////////////////////////////////////////////////////
// Get the servo counter clockwise speed setting.
////////////////////////////////////////////////////////////  
int DynamixelServo::GetCCWSpeed(void)
{
	//Serial.print("Getting speed for ID: ");
	//Serial.println(this->id);
	return ccw_speed;
}

////////////////////////////////////////////////////////////
// Get the servo speed.
////////////////////////////////////////////////////////////  
int DynamixelServo::GetCurrentSpeed(void)
{
	//Serial.print("Getting actual speed for ID: ");
	//Serial.println(this->id);
	return Dynamixel.readSpeed(id);
}

////////////////////////////////////////////////////////////
// Get the servo acceleration.
////////////////////////////////////////////////////////////  
int DynamixelServo::GetACDC(void)
{
	//Serial.print("Getting ACDC for ID: ");
	//Serial.println(this->id);
	return acdc;
}

////////////////////////////////////////////////////////////
// Get the servo CW limit.
////////////////////////////////////////////////////////////  
int DynamixelServo::GetCWLimit(void)
{
	//Serial.print("Getting CW limit for ID: ");
	//Serial.println(this->id);
	return cw_limit;
}

////////////////////////////////////////////////////////////
// Get the servo CCW limit.
////////////////////////////////////////////////////////////  
int DynamixelServo::GetCCWLimit(void)
{
	//Serial.print("Getting CCW limit for ID: ");
	//Serial.println(this->id);
	return ccw_limit;
}

////////////////////////////////////////////////////////////
// Get the servo position: 0 to 4096.
////////////////////////////////////////////////////////////  
int DynamixelServo::GetPosition(void)
{
	//Serial.print("Getting position for ID: ");
	//Serial.println(this->id);
	return Dynamixel.readPosition(id);
}

////////////////////////////////////////////////////////////
// Get the steps traveled.
////////////////////////////////////////////////////////////  
long DynamixelServo::GetTotalSteps(void)
{
	return totalSteps;
}

////////////////////////////////////////////////////////////
// If in wheel mode set the speed and direction.
// If the time is greater than zero we will run
// for the specified number of miliseconds.
////////////////////////////////////////////////////////////  
void DynamixelServo::Wheel(unsigned long tm)
{ 

  if ( this->enabled == true )
  {
	  // Internal timing is done in uS so multiply the mS inputs
	  // by 1000.
	  unsigned long time;
	  // If the time is set to zero run the maximum time. 
	  if ( tm == 0 )
	  {
		 time = 4294967295; // 1.12 hours.
	  }		  
	  else
	  {
		 time = tm * 1000;
	  }
	  
	  int acdctm = acdc * 1000;
	  
	  if (mode == WHEEL)
	  { 
		// Scaling.	
		int scaled_cw_speed = ScaleSpeed(cw_speed);
		int scaled_ccw_speed = ScaleSpeed(ccw_speed);
		unsigned long scaled_tm = ScaleTime(time);
		unsigned long scaled_acdctm = ScaleTime(acdctm);	
		
		/*
		Serial.println("---- Wheel command.");
		Serial.print("CW Speed: ");
		Serial.println(cw_speed);
		Serial.print("CCW Speed: ");
		Serial.println(ccw_speed);	
		Serial.print("Time: ");
		Serial.println(tm);
		Serial.print("AcDc time: ");
		Serial.println(acdc);
		Serial.println("Scaled values.");
		Serial.print("Scale %: ");
		Serial.println(time_scale);
		Serial.print("Scaled CW Speed: ");	
		Serial.println(scaled_cw_speed);
		Serial.print("Scaled CCW Speed: ");	
		Serial.println(scaled_ccw_speed);
		Serial.print("Scaled Time: ");
		Serial.println(scaled_tm);
		Serial.print("Scaled AcDc time: ");
		Serial.println(scaled_acdctm);
		*/

		// Set the mode and flag.
		serviceMode = MTIMED;
		needsService = true;
		// Speed.
		atMaxSpeed = false;
		// And the time to run.
		timeToRun = scaled_tm;	
		// And update the time to now.
		timeStart = micros();
		// Start acceleration/deceleration if specified.		
		if ( scaled_acdctm > 0)
		{
			timeAccDcc = scaled_acdctm;
			decStart = timeStart + timeToRun - timeAccDcc;
			increment = timeAccDcc/ACDCTABLESTEPS;				
			StartAcc();
		}
		else
		{
			//Serial.println("No acceleration.");
			timeAccDcc = 0;
			decStart = 0;
			increment = 0;
			StopAccDcc();	
		}
		
		/*
		Serial.print("Time to run: ");
		Serial.println(timeToRun);
		Serial.print("timeStart: ");
		Serial.println(timeStart);
		Serial.print("timeAccDcc: ");
		Serial.println(timeAccDcc);
		Serial.print("decStart: ");
		Serial.println(decStart);
		Serial.print("increment: ");
		Serial.println(increment);
		*/
	  }
	}
 	else
	{
		Serial.println("Not enabled.");	
	}
}

////////////////////////////////////////////////////////////
// Move the given number of steps.
// Accelerate and decelerate the given number of steps.
////////////////////////////////////////////////////////////  
void DynamixelServo::Step(long stps, long acdcStps)
{  
	if (this->mode == WHEEL)
	{ 
		/*
		Serial.print("Step command: ");
		Serial.print("Steps: ");
		Serial.print(stps);		
		Serial.print(" acdcStps: ");
		Serial.println(acdcStps);		
		*/
		
		int scaled_speed = 0;		
		
		// Set the direction.
		if ( direction == DRIGHT )
		{ 
			//Serial.print("Moving clockwise.");
		    scaled_speed = ScaleSpeed(cw_speed);		
		}
		else
		{
			//Serial.print("Moving counter clockwise.");
		    scaled_speed = ScaleSpeed(ccw_speed);		
		}	

		//Serial.print("Scaled Speed: ");
		//Serial.println(scaled_speed);		
			
		// Steps.
		steps = 0;
		stepsToRun = stps;	
	
		// Set up the initial position.
		initialPosition = Dynamixel.readPosition(id);
		// Set the mode and flag.
		serviceMode = MSTEPS;
		needsService = true;
		// Start acceleration/deceleration if specified.		
		if ( acdcStps > 0)
		{
			// Number of steps spent accelerating/decelerating.
			stepsAccDcc = acdcStps;
			increment = stepsAccDcc/TURNSACDCTABLESTEPS;	
			decStart = abs(stepsToRun) - stepsAccDcc;
			StartAcc();
		}
		else
		{
			//Serial.println("No acceleration.");
			stepsAccDcc = 0;
			increment = 0;
			decStart = 0;
			StopAccDcc();	
		}	

		/*
		Serial.print("Total steps: ");	
		Serial.println(stepsToRun);
		Serial.print("stepsAccDcc: ");	
		Serial.println(stepsAccDcc);
		Serial.print("decStart: ");
		Serial.println(decStart);
		Serial.print("increment: ");
		Serial.println(increment);	
		*/
		
		// Command the servo. In turns mode we must
		// manually start it moving!
		int startSpeed = (scaled_speed * TurnsACDCLookupTable[0])/100;
		Dynamixel.wheel(id, direction, startSpeed);	
	}
}

////////////////////////////////////////////////////////////
// If in turn mode set the speed, direction, number of turns
// and acceleration turns.
////////////////////////////////////////////////////////////  
void DynamixelServo::Turns(float turns, float acdc)
{
	if ( this->enabled == true )
	{	
		long steps = turns * 4095;
		long acdcSteps = acdc * 4095;
		
		// Call the step function.
		Step(steps, acdcSteps);		
	}
	else
	{
		Serial.println("Not enabled.");	
	}
}	

////////////////////////////////////////////////////////////
// Servo command.
////////////////////////////////////////////////////////////  
void DynamixelServo::Servo(int pos)
{
	DoServo(pos, false);
}

////////////////////////////////////////////////////////////
// Preload servo command.
////////////////////////////////////////////////////////////  
void DynamixelServo::ServoPreload(int pos)
{
	DoServo(pos, true);
}

////////////////////////////////////////////////////////////
// Stop the servo.
////////////////////////////////////////////////////////////  
void DynamixelServo::Stop()
{
	// Get the current position.
	int position = Dynamixel.readPosition(id);

	//Serial.print("id: ");	
	//Serial.println(id);
	//Serial.print("Current position: ");	
	//Serial.println(position);
	
    if (this->mode == WHEEL)
	{	
		Dynamixel.wheel(id, direction, 0);	  
	}  
	else
	{
		Dynamixel.servo(id, position, 1);
		position = Dynamixel.readPosition(id);	
		//Serial.print("Stopped position: ");	
		//Serial.println(position);
	}
	// Reset variables.
	serviceMode = MNONE;
	needsService = false;
	timeToRun = 0;	
	timeStart = 0;
	timeAccDcc = 0;
	StopAccDcc();	
}

////////////////////////////////////////////////////////////
// Send the action command to this ID.
////////////////////////////////////////////////////////////  
void DynamixelServo::Action()
{
	Serial.print("Sending action command to: ");		
	Serial.println(id);
	Dynamixel.action(id);
}

////////////////////////////////////////////////////////////
// Send the action command to ALL Dynamixels.
////////////////////////////////////////////////////////////  
void DynamixelServo::ActionAll()
{
	Serial.println("Sending action command to all.");	
	// ID 254 means ALL Dynamixels.	
	Dynamixel.action(254);
}

////////////////////////////////////////////////////////////
// Check if the Dynamixel is currently moving.
////////////////////////////////////////////////////////////  
bool DynamixelServo::IsMoving(void)
{
	return Dynamixel.checkMovement(id);
}

////////////////////////////////////////////////////////////
// Ping.
// If the Dynamixel is present and working it should 
// respond with it's own ID.
////////////////////////////////////////////////////////////  
bool DynamixelServo::Ping()
{
	int returnedID = Dynamixel.ping(id);
	if ( returnedID == id )
	{
		return true;
	}
	else 
	{
		return false;
	}
}

////////////////////////////////////////////////////////////
// Handle any servicing this servo needs.
////////////////////////////////////////////////////////////  
void DynamixelServo::Service()
{
	int newSpeed = 0;
	
	// If servicing is required.
	if (needsService)
	{
		// Do what we need to based on the service mode.
		switch (serviceMode)
		{
			// Timed mode.			
			case MTIMED:
			{									
				// If accelerating or decelerating get the speed from the table.
				if ( accelerating == true )
				{
					// Check the time. We divide the total acc/dcc time into the
					// number of steps in the table taking into account how many have already
					// passed.					
					if ( micros() >= timeStart + (increment * (lookupTableIndex + 1)) )					
					{
						
						//Serial.print("lookupTableIndex: ");
						//Serial.println(lookupTableIndex);
						//Serial.print("micros: ");
						//Serial.println(micros());
						//Serial.print("(increment * (lookupTableIndex + 1)): ");
						//Serial.println((increment * (lookupTableIndex + 1)));
						//Serial.print("timeStart + (increment * (lookupTableIndex + 1)): ");
						//Serial.println(timeStart + (increment * (lookupTableIndex + 1)));						
						
						// Get the speed. 
						if ( direction == DRIGHT )
						{
							newSpeed = (cw_speed * NextStep()) / 100;													
						}
						else
						{
							newSpeed = (ccw_speed * NextStep()) / 100;													
						}
						int scaled_speed = ScaleSpeed(newSpeed);
						/*
						Serial.print("+");
						Serial.print("New speed: ");
						Serial.println(scaled_speed);												
						*/
						
						// Command the servo.
						Dynamixel.wheel(id, direction, scaled_speed);							

					}
					// Once the acceleration time has expired we stop 
					// accelerating.
					if ( micros() >= timeStart + timeAccDcc )					
					{
						StopAccDcc();					
					}				
				} 
				else if ( decelerating == true )
				{
					// Check the time. We divide the total acc/dcc time into the number
					// of steps in the table taking into account how many have already passed.
				
					if ( micros() >= decStart + (increment * (lookupTableIndex + 1)))				
					{
						
						//Serial.print("lookupTableIndex: ");
						//Serial.println(lookupTableIndex);
						//Serial.print("micros: ");
						//Serial.println(micros());
						//Serial.print("(increment * (lookupTableIndex + 1)): ");
						//Serial.println((increment * (lookupTableIndex + 1)));
						//Serial.print("timeStart + (increment * (lookupTableIndex + 1)): ");
						//Serial.println(timeStart + (increment * (lookupTableIndex + 1)));
						
						
						// Get the speed. 
						if ( direction == DRIGHT )
						{
							newSpeed = (cw_speed * NextStep()) / 100;													
						}
						else
						{
							newSpeed = (ccw_speed * NextStep()) / 100;													
						}													
						int scaled_speed = ScaleSpeed(newSpeed);
						
						//Serial.print("-");
						//Serial.print("New speed: ");
						//Serial.println(scaled_speed);												
						
						// Command the servo.
						Dynamixel.wheel(id, direction, scaled_speed);						
					}					
				}
				else
				{
					// Command the servo.
					if ( !atMaxSpeed )
					{
						// Go to maximum speed!
						int scaled_speed = 0;								
						if ( direction == DRIGHT )
						{
							scaled_speed = ScaleSpeed(cw_speed);								
						}
						else
							
						{
							scaled_speed = ScaleSpeed(ccw_speed);																					
						}						
						Dynamixel.wheel(id, direction, scaled_speed);
						//Serial.print("Setting max speed: ");
						//Serial.println(scaled_speed);
						atMaxSpeed = true;
					}					
					
					// If acc/dcc is specified check the time to see if we should start decelerating.
					if ( (timeAccDcc > 0 ) && (micros() >= decStart) ) 
					{
						// Start deceleration.
						StartDcc();
					}						
				}				
				
				// If the total time has expired we're done.
				unsigned long timeNow = micros();
				if ( timeNow  >= timeStart + timeToRun )
				{										
					// Command the servo.
					Dynamixel.wheel(id, direction, 0);
					
					// Update the servicing flag and mode.
					needsService = false;
					serviceMode = MNONE;					
					
					/*			
					Serial.println("Dynamixel stopped!");										
					Serial.print("timeToRun: ");							
					Serial.println(timeToRun);
					Serial.print("timeStart: ");							
					Serial.println(timeStart);														
					Serial.print("Total run time: ");							
					Serial.println(timeNow - timeStart);									
					Serial.print("Difference: ");							
					Serial.println( (timeNow - timeStart) - timeToRun);
					Serial.print("acdcStop calculated: ");							
					Serial.println(timeStart + timeAccDcc);
					Serial.print("acdcStop actual: ");							
					Serial.println(acdcStopActual);
					Serial.print("Difference: ");
					long diff = (timeStart + timeAccDcc) - acdcStopActual;		
					Serial.println(diff);
					Serial.print("decStart calculated: ");							
					Serial.println(decStart);
					Serial.print("decStart actual: ");							
					Serial.println(decStartActual);						
					Serial.print("Difference: ");				
				    diff = decStart - decStartActual;					
					Serial.println(diff);
					*/
					
					// Clear the times.
					timeToRun = 0;
					timeStart = 0;
					
					// Clear acceleration/deceleration.
					StopAccDcc();											
										
				}				
				break;
			}
			
			// Servo mode.			
			case MSERVO:
			{	
				// Read the position.
				newPosition = Dynamixel.readPosition(id);
				
				// Track how many steps have been taken.
				steps = steps + abs(newPosition - oldPosition);
				//Serial.print("Steps taken: ");
			    //Serial.println(steps);
				//Serial.print("Steps to go: ");							
			    //Serial.println(abs(newPosition - goalPosition));
						
				// If accelerating or decelerating get the speed from the table.
				if ( accelerating == true )
				{
					// Check how many steps have been taken. See if we need to change speed.
					if ( steps >= increment * (lookupTableIndex + 1) ) 										
					{	
						/*
						Serial.print("goalPosition: ");
						Serial.println(goalPosition);				
						Serial.print("newPosition: ");
						Serial.println(newPosition);
						Serial.print("oldPosition: ");
						Serial.println(oldPosition);
						Serial.print("abs(newPosition - oldPosition): ");
						Serial.println(abs(newPosition - oldPosition));						
						Serial.print("increment: ");
						Serial.println(increment);
						*/
						
						// Get the speed based on direction. 
						// Position going down is CW rotation.
						if ( newPosition <= oldPosition )
						{
							newSpeed = (cw_speed * NextServoStep()) / 100;													
						}
						else
						{
							newSpeed = (ccw_speed * NextServoStep()) / 100;													
						}
						int scaled_speed = ScaleSpeed(newSpeed);
						
						//Serial.print("+");
						//Serial.print("New speed: ");
						//Serial.println(scaled_speed);																	
						
						// Command the servo.
						Dynamixel.servo(id, goalPosition, scaled_speed);							

					}
					// Once we are past accelerating position we
					// stop accelerating.
					if ( steps > stepsAccDcc)
					{
						StopAccDcc();					
					}				
				} 
				else if ( decelerating == true )
				{
					// Check the time. We divide the total acc/dcc time into the number
					// of steps in the table taking into account how many have already passed.
				
					if ( steps >= decStart + (increment * (lookupTableIndex + 1)))				
					{
						/*
						Serial.print("Goal position: ");
						Serial.println(goalPosition);
						Serial.print("newPosition: ");
						Serial.println(newPosition);
						Serial.print("oldPosition: ");
						Serial.println(oldPosition);
						Serial.print("abs(newPosition - oldPosition): ");
						Serial.println(abs(newPosition - oldPosition));						
						Serial.print("increment: ");
						Serial.println(increment);
						*/
						// Get the speed based on direction. 
						// Position going down is CW rotation.
						if ( newPosition <= oldPosition )
						{
							newSpeed = (cw_speed * NextServoStep()) / 100;													
						}
						else
						{
							newSpeed = (ccw_speed * NextServoStep()) / 100;													
						}													
						int scaled_speed = ScaleSpeed(newSpeed);
						
						//Serial.print("-");
						//Serial.print("New speed: ");
						//Serial.println(scaled_speed);												
						
						// Command the servo.
						Dynamixel.servo(id, goalPosition, scaled_speed);								
					}					
				}
				else
				{
					// Command the servo.
					if ( !atMaxSpeed )
					{
						// Go to maximum speed!
						int scaled_speed = 0;								
						if ( newPosition <= oldPosition )
						{
							scaled_speed = ScaleSpeed(cw_speed);								
						}
						else
						{
							scaled_speed = ScaleSpeed(ccw_speed);																					
						}						
						// Command the servo.
						Dynamixel.servo(id, goalPosition, scaled_speed);	
						//Serial.print("Setting max speed: ");
						//Serial.println(scaled_speed);
						atMaxSpeed = true;
					}					
					
					// Check if we have done enough steps to start decelerating.
					if ( steps > decStart ) 
					{
						// Start deceleration.
						Serial.println("Starting deceleration phase.");
						StartDcc();
					}						
				}				
				
				// If in position we are done.
				if ( (newPosition == goalPosition) || (steps > stepsToRun) )
				{																			
					// Update the servicing flag and mode.
					needsService = false;
					serviceMode = MNONE;															
					unsigned long timeNow = micros();														
					/*
					Serial.println("Dynamixel stopped!");										
					Serial.print("timeStart: ");							
					Serial.println(timeStart);														
					Serial.print("Total run time: ");							
					Serial.println(timeNow - timeStart);			
					*/
					// Clear acceleration/deceleration.
					StopAccDcc();																				
				}
				
				// Update the position. 
				oldPosition = newPosition;				
				break;
			}			
			
			case MSTEPS:
			{
				// Read the position.
				newPosition = Dynamixel.readPosition(id);
				
				// Servo moving left - position going up.
				if ( direction == LEFT )
				{
				  // Add on the steps taken.
				  if ( newPosition >= oldPosition )
				  {
					steps = steps + (newPosition - oldPosition);
					totalSteps = + (newPosition - oldPosition);				
					// Update the position.
					oldPosition = newPosition;        
				  }
				  else if ( Wrapped(newPosition, oldPosition) )
				  {
					// We handle the wrap around here.
					steps = steps + ((4096 + newPosition) - oldPosition);
					totalSteps = totalSteps + ((4096 + newPosition) - oldPosition);
					// Update the position.
					oldPosition = newPosition; 
				  }				  
				}

				// Servo moving right - position going down.  
				if ( direction == RIGHT )
				{
				  // Subtract off the steps taken.
				  if ( newPosition <= oldPosition )
				  {
					steps = steps - (oldPosition - newPosition);
					totalSteps = totalSteps - (oldPosition - newPosition);
					// Update the position.
					oldPosition = newPosition;        
				  }
				  else if ( Wrapped(newPosition, oldPosition) )
				  {
					// We handle the wrap around here.
					steps = steps - ((4096 + oldPosition) - newPosition);				
					totalSteps = totalSteps - ((4096 + oldPosition) - newPosition);
					// Update the position.
					oldPosition = newPosition;
				  }				  				 
				}
				//Serial.print("steps: ");
				//Serial.println(steps);
				//Serial.print("totalSteps: ");
				//Serial.println(totalSteps);
									
				// If accelerating or decelerating get the speed from the table.
				if ( accelerating == true )
				{
					
					// Get the next speed. 
					//Serial.print("abs(steps): ");
					//Serial.println(abs(steps));
					//Serial.print("stepsAccDcc: ");
					//Serial.println(stepsAccDcc);
					
					long indexMultiplier = abs(steps) * TURNSACDCTABLESTEPS;
					//Serial.print("indexMultiplier: ");
					//Serial.println(indexMultiplier);
					int index = indexMultiplier / stepsAccDcc;
					if ( index >= TURNSACDCTABLESTEPS )
					{
						index = TURNSACDCTABLESTEPS - 1;
					}
					//Serial.print("index: ");
					//Serial.println(index);
					
					int speedMultiplier = TurnsACDCLookupTable[index];						
					//Serial.print("speedMultiplier: ");
					//Serial.println(speedMultiplier);
					
					// Get the correct speed based on direction.							
					if ( direction == DRIGHT )
					{
						newSpeed = (cw_speed * speedMultiplier) / 100;		
					}
					else
					{
						newSpeed = (ccw_speed * speedMultiplier) / 100;																			
					}
					// Adjust the minimum.						
					if (newSpeed < 10 )
					{
						newSpeed = 10;
					}
				    // Scale the speed.			
					int scaled_speed = ScaleSpeed(newSpeed);
					
					//Serial.print("+");
					//Serial.print("New speed: ");
					//Serial.println(scaled_speed);	
					//Serial.print("steps: ");
					//Serial.println(steps);							
											
					// Command the servo.
					Dynamixel.wheel(id, direction, scaled_speed);							
						
					// Once the acceleration amount has expired we stop 
					// accelerating.
					if ( abs(steps) >= stepsAccDcc )					
					{
						StopAccDcc();					
					}											
				} 
				else if ( decelerating == true )
				{

					if ( abs(steps) >= decStart )	
					{									
	
						// Get the next speed. 
						//Serial.print("abs(steps): ");
						//Serial.println(abs(steps) - decStart);
						//Serial.print("stepsAccDcc: ");
						//Serial.println(stepsAccDcc);
						
						long indexMultiplier = (abs(steps) - decStart) * TURNSACDCTABLESTEPS;
						//Serial.print("indexMultiplier: ");
						//Serial.println(indexMultiplier);
						int index = indexMultiplier / stepsAccDcc;
						if ( index >= TURNSACDCTABLESTEPS )
						{
							index = ACDCTABLESTEPS - 1;
						}
						//Serial.print("index: ");
						//Serial.println(index);
						
						int speedMultiplier = TurnsACDCLookupTable[TURNSACDCTABLESTEPS - 1 - index];						
						//Serial.print("speedMultiplier: ");
						//Serial.println(speedMultiplier);							
						if ( direction == DRIGHT )
						{
							newSpeed = (cw_speed * speedMultiplier) / 100;		
						}
						else
						{
							newSpeed = (ccw_speed * speedMultiplier) / 100;																			
						}	
						int scaled_speed = ScaleSpeed(newSpeed);
						
						//Serial.print("-");
						//Serial.print("New speed: ");
						//Serial.println(scaled_speed);
						//Serial.print("steps: ");
						//Serial.println(steps);							
						
						// Command the servo.
						Dynamixel.wheel(id, direction, scaled_speed);				
					}
				}
				else
				{
					// Command the servo.
					if ( !atMaxSpeed )
					{
						int scaled_speed = 0;								
						// Go to maximum speed.
						if ( direction == DRIGHT )
						{
							scaled_speed = ScaleSpeed(cw_speed);								
						}
						else
						{
							scaled_speed = ScaleSpeed(ccw_speed);																											
						}							
						Dynamixel.wheel(id, direction, scaled_speed);
						//Serial.print("Setting max speed: ");
						//Serial.println(scaled_speed);
						atMaxSpeed = true;
					}
					
					// If acc/dcc is specified check the time to see if we should start decelerating.
					if ( abs(steps) >=  decStart ) 
					{
						// Start decelerating.
						StartDcc();
					}				
				}				
								
				//Serial.print("Steps: ");
				//Serial.println(steps);
				//Serial.print("stepsToRun: ");
				//Serial.println(stepsToRun);
				
				// If we have reached the right point we stop.				
				if ( abs(steps) >= abs(stepsToRun) )
				{
				  //Serial.println("Stopping servo.");				  
				  // Stop the servo.
				  Dynamixel.wheel( id, direction, 0);  
				  // Update the servicing flag and mode.
				  needsService = false;
				  serviceMode = MNONE;  
				  //Serial.print("Total steps: ");
				  //Serial.println(totalSteps);
				}   				
				break;
			}
			default:
			{
				break;
			}
		}
	}	
}