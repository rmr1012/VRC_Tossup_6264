#pragma config(Motor,  port2,           frontRight,    tmotorNormal, openLoop)
#pragma config(Motor,  port3,           backRight,     tmotorNormal, openLoop)
#pragma config(Motor,  port8,           backLeft,     tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port9,           frontLeft,      tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port7,           armmotR,      tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port5,           armmotL,      tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port4,           roller,      tmotorNormal, openLoop, reversed)
#pragma config(Motor,  port6,           conveyor,      tmotorNormal, openLoop, reversed)
#pragma config(Sensor, in1,    armPotentiometerL,    sensorPotentiometer)
#pragma config(Sensor, in2,    armPotentiometerR,    sensorPotentiometer)
#pragma config(Sensor, dgtl8,  solenoid2,      sensorDigitalOut)
#pragma config(Sensor, dgtl9,  solenoid,       sensorDigitalOut)




/*----------------------------------------------------------------------------------------------------*/
bool liftDa;
void mecanumdrive();
task raiseOrLowerArmL();
task raiseOrLowerArmR();
task projectile();
int joystick;

//-----------------------============================---------------------------==============================----------------

task main()
{
  StartTask(raiseOrLowerArmL);
  StartTask(raiseOrLowerArmR);
	StartTask(projectile);
  while(true) //Loop forever
  {
		mecanumdrive();
//		motor[armmot]=vexRT[Ch2];

		if(vexRT[Btn5U] == 1)
      motor[roller] = 127;
    else if(vexRT[Btn5D] == 1)
    	motor[roller] = -127;
    else
    	motor[roller] = 0;


    motor[conveyor] = vexRT[Ch3Xmtr2];



   if(SensorValue[armPotentiometerL]<=1450)
    {
    	liftDa=true;
			motor[armmotL]=motor[armmotR]=joystick;
		}
		else
		{
			liftDa=false;
		}
	}
}



  /*  if(vexRT[Btn7D] == 1)         // If button 6U (upper right shoulder button) is pressed:
    {
      SensorValue[solenoid] = 1;
      SensorValue[solenoid2] = 1;  // ...activate the solenoid.
    }
    else if(vexRT[Btn7U] == 1)                         // If button 6U (upper right shoulder button) is  NOT pressed:
    {
      SensorValue[solenoid] = 0;
      SensorValue[solenoid2] = 0;  // ..deactivate the solenoid.
    }
		if(topmode==true)
			joystick=+127;
		else
			joystick=vexRT[Ch2];

    if(vexRT[Btn5U] == 1)
      motor[roller] = 127;
    else if(vexRT[Btn5D] == 1)
    	motor[roller] = -127;
    else
    	motor[roller] = 0;

 	  SensorValue[led1]=handlock;
 	  if(vexRT[Btn7R] == 1)
 	  	descore();*/


void mecanumdrive()
{
  //Create "deadzone" variables. Adjust threshold value to increase/decrease deadzone
	int X2 = 0, Y1 = 0, X1 = 0, threshold = 15;
  //Create "deadzone" for Y1/Ch3
  if(abs(vexRT[Ch3]) > threshold)
    Y1 = vexRT[Ch3];
  else
    Y1 = 0;
  //Create "deadzone" for X1/Ch4
  if(abs(vexRT[Ch4]) > threshold)
    X1 = vexRT[Ch4];
  else
    X1 = 0;
  //Create "deadzone" for X2/Ch1
  if(abs(vexRT[Ch1]) > threshold)
    X2 = vexRT[Ch1];
  else
    X2 = 0;

  //Remote Control Commands
  motor[frontRight] = Y1 - X2 - X1;
  motor[backRight] =  Y1 - X2 + X1;
  motor[frontLeft] = Y1 + X2 + X1;
  motor[backLeft] =  Y1 + X2 - X1;
}

//-----------------------------------------------------------------------------------

task projectile()
{
	while(1){
		while(vexRT[Btn7DXmtr2] == 1)         // If button 6U (upper right shoulder button) is pressed:
    {
      SensorValue[solenoid] = 1;
      SensorValue[solenoid2] = 1;  // ...activate the solenoid.
      joystick=vexRT[Ch2];
    }
    	joystick=vexRT[Ch2];
    	SensorValue[solenoid] = 0;
      SensorValue[solenoid2] = 0;
	}
}
//-----------------------------------------------------------------------------------

task raiseOrLowerArmL()
{
  const int kSlowSpeedRange = 50;
  const int kMinPos = 1300;
  const int kMaxPos = 3000;

  const int kSlowSpeedPosLowering = kMinPos + kSlowSpeedRange;
  const int kSlowSpeedRaising     = kMaxPos - kSlowSpeedRange;

  const int kMotorSpeedUp   = +127;
  const int kMotorSpeedDown = -127;

  int nPositionToHold;
  int nLastError;
  bool bLastUpdateWasButtonPress = false;
  bool bButtonsHaveBeenPressed   = false;
  while(true)
  {
  	while(liftDa){}
	  if (joystick <= -90)// button is puchsed indicating ARM should be lowered
	  {
	    if (SensorValue[armPotentiometerL] < kMinPos)
	    {
	      motor[armmotL] = 0; // Arm is fully lowered. Stop the motor
	    }
	    else if (SensorValue[armPotentiometerL] < kSlowSpeedPosLowering)
	      motor[armmotL] = kMotorSpeedDown / 2;
	    else
	      motor[armmotL] = kMotorSpeedDown;
	    bLastUpdateWasButtonPress = true;
	  }

	  else if (joystick >= 90)// button is puchsed indicating ARM should be raised
	  {
	    if (SensorValue[armPotentiometerL] > kMaxPos)
	    {
	      motor[armmotL] = 0;// Arm is fully raised. Stop the motor
	    }
	    else if (SensorValue[armPotentiometerL] > kSlowSpeedRaising)
	      motor[armmotL] = kMotorSpeedUp / 2;
	    else
	      motor[armmotL] = kMotorSpeedUp;
	    bLastUpdateWasButtonPress = true;
	  }

	  else// No buttons are pushed.
	  {


	    if (bLastUpdateWasButtonPress) // A button has just been released.- Remember the position.-- Stop motor
	    {
	      bLastUpdateWasButtonPress = false;
	      nPositionToHold = (SensorValue[armPotentiometerL]+SensorValue[armPotentiometerL])/2;
	      if (nPositionToHold < kMinPos)
	        nPositionToHold = kMinPos;
	      else if (nPositionToHold > kMaxPos)
	        nPositionToHold = kMaxPos;
	      motor[armmotL] = 0;
	      ClearTimer(T4);
	      bButtonsHaveBeenPressed = true;
	      nLastError = SensorValue[armPotentiometerL] - nPositionToHold;
	    }
	    else if (bButtonsHaveBeenPressed)
	    {
	      const int kUpdateCycle    = 10;
	      const int kSpeedIncrement = 5;  //need to be adjusted

	      int nError;

	      if (time1[T4] > kUpdateCycle)
	      { // Not enough time has elapsed

		      ClearTimer(T4);
		      nError = SensorValue[armPotentiometerL] - nPositionToHold;

		      // Adjust the motor "hold" speed based on whether the position error is increasing or
		      // decreasing. Don't worry about small errors in the range -5 to +5.

		      if (nError < -10)
		      {
		        if (nError < nLastError)
		        {
		          // Error is increasing in magnitude. Adjust the speed.
		          if (SensorValue[armPotentiometerL] < kMaxPos)
		            motor[armmotL] += kSpeedIncrement;
		        }
		        else if (motor[armmotL] > 0)
		          motor[armmotL] -= kSpeedIncrement;
		      }
		      else if (nError > +10)
		      {
		        if (nError > nLastError)
		        {
		          if (SensorValue[armPotentiometerL] > kMinPos)
		            motor[armmotL] -= kSpeedIncrement;
		        }
		        else if (motor[armmotL] > 0)
		          motor[armmotL] += kSpeedIncrement;
		      }
		      nLastError = nError;
	  	  }
	    }
	    else{}
	  }
	}
}

//-----------------------------------------------------------------------------------

task raiseOrLowerArmR()
{
  const int kSlowSpeedRange = 50;
  const int kMinPos = 1300;
  const int kMaxPos = 3000;

  const int kSlowSpeedPosLowering = kMinPos + kSlowSpeedRange;
  const int kSlowSpeedRaising     = kMaxPos - kSlowSpeedRange;

  const int kMotorSpeedUp   = +127;
  const int kMotorSpeedDown = -127;

  int nPositionToHold;
  int nLastError;
  bool bLastUpdateWasButtonPress = false;
  bool bButtonsHaveBeenPressed   = false;
  while(true)
  {
	  while(liftDa){}
  	if (joystick <= -90)// button is puchsed indicating ARM should be lowered
	  {
	    if (SensorValue[armPotentiometerR] < kMinPos)
	    {
	      motor[armmotR] = 0; // Arm is fully lowered. Stop the motor
	    }
	    else if (SensorValue[armPotentiometerR] < kSlowSpeedPosLowering)
	      motor[armmotR] = kMotorSpeedDown / 2;
	    else
	      motor[armmotR] = kMotorSpeedDown;
	    bLastUpdateWasButtonPress = true;
	  }

	  else if (joystick >= 90)// button is puchsed indicating ARM should be raised
	  {
	    if (SensorValue[armPotentiometerR] > kMaxPos)
	    {
	      motor[armmotR] = 0;// Arm is fully raised. Stop the motor
	    }
	    else if (SensorValue[armPotentiometerR] > kSlowSpeedRaising)
	      motor[armmotR] = kMotorSpeedUp / 2;
	    else
	      motor[armmotR] = kMotorSpeedUp;
	    bLastUpdateWasButtonPress = true;
	  }

	  else// No buttons are pushed.
	  {


	    if (bLastUpdateWasButtonPress) // A button has just been released.- Remember the position.-- Stop motor
	    {
	      bLastUpdateWasButtonPress = false;
	      nPositionToHold = (SensorValue[armPotentiometerL]+SensorValue[armPotentiometerR])/2;
	      if (nPositionToHold < kMinPos)
	        nPositionToHold = kMinPos;
	      else if (nPositionToHold > kMaxPos)
	        nPositionToHold = kMaxPos;
	      motor[armmotR] = 0;
	      ClearTimer(T4);
	      bButtonsHaveBeenPressed = true;
	      nLastError = SensorValue[armPotentiometerR] - nPositionToHold;
	    }
	    else if (bButtonsHaveBeenPressed)
	    {
	      const int kUpdateCycle    = 10;
	      const int kSpeedIncrement = 5;  //need to be adjusted

	      int nError;

	      if (time1[T4] > kUpdateCycle)
	      { // Not enough time has elapsed

		      ClearTimer(T4);
		      nError = SensorValue[armPotentiometerR] - nPositionToHold;

		      // Adjust the motor "hold" speed based on whether the position error is increasing or
		      // decreasing. Don't worry about small errors in the range -5 to +5.

		      if (nError < -10)
		      {
		        if (nError < nLastError)
		        {
		          // Error is increasing in magnitude. Adjust the speed.
		          if (SensorValue[armPotentiometerR] < kMaxPos)
		            motor[armmotR] += kSpeedIncrement;
		        }
		        else if (motor[armmotR] > 0)
		          motor[armmotR] -= kSpeedIncrement;
		      }
		      else if (nError > +10)
		      {
		        if (nError > nLastError)
		        {
		          if (SensorValue[armPotentiometerR] > kMinPos)
		            motor[armmotR] -= kSpeedIncrement;
		        }
		        else if (motor[armmotR] > 0)
		          motor[armmotR] += kSpeedIncrement;
		      }
		      nLastError = nError;
	  	  }
	    }
	    else{}
	  }
	}
}
