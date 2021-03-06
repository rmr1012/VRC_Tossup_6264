#pragma config(Sensor, in1,    armPotentiometerL, sensorPotentiometer)
#pragma config(Sensor, in2,    armPotentiometerR, sensorPotentiometer)
#pragma config(Sensor, dgtl1,  RH,             sensorTouch)
#pragma config(Sensor, dgtl2,  RM,             sensorTouch)
#pragma config(Sensor, dgtl3,  BH,             sensorTouch)
#pragma config(Sensor, dgtl4,  BM,             sensorTouch)
#pragma config(Sensor, dgtl8,  solenoid2,      sensorDigitalOut)
#pragma config(Sensor, dgtl9,  solenoid,       sensorDigitalOut)
#pragma config(Motor,  port2,           frontRight,    tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port3,           backRight,     tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port4,           roller,        tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port5,           armmotL,       tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port6,           conveyor,      tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port7,           armmotR,       tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port8,           backLeft,      tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port9,           frontLeft,     tmotorServoContinuousRotation, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Setting

/*----------------------------------------------------------------------------------------------------*/
bool liftDa;
void mecanumdrive();
void GoRH();
void GoRM();
void GoBH();
void GoBM();
task raiseOrLowerArmL();
task raiseOrLowerArmR();
task projectile();
int joystick,StartingTile;

//-----------------------============================---------------------------==============================----------------

/////////////////////////////////////////////////////////////////////////////////////////
//
//                          Pre-Autonomous Functions
//
// You may want to perform some actions before the competition starts. Do them in the
// following function.
//
/////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task
//
// This task is used to control your robot during the autonomous phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task main()
{
	SensorValue[RH]=0;
	SensorValue[RM]=0;
	SensorValue[BH]=0;
	SensorValue[BM]=0;
	while(!(RH||RM||BH||BM)){  }
	if(RH)
		StartingTile=1;
	else if(RM)
		StartingTile=2;
	else if(BH)
		StartingTile=3;
	else if(BM)
		StartingTile=4;


	//unfold
	motor[conveyor]=127;
	wait10Msec(50);
	motor[roller]=127;
	wait10Msec(50);
	motor[conveyor]=0;
	motor[roller]=0;



	switch(StartingTile)
	{
		case 1:
			GoRH();
			break;
		case 2:
			GoRM();
			break;
		case 3:
			GoBH();
			break;
		case 4:
			GoBM();
			break;
		default:
			break;
	}
}


void GoRH()
{

}
void GoRM()
{

}
void GoBH()
{

}
void GoBM()
{

}
