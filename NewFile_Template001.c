#pragma config(Sensor, in1,    armPotentiometerL, sensorPotentiometer)
#pragma config(Sensor, in2,    armPotentiometerR, sensorPotentiometer)
#pragma config(Sensor, in3,    gyro,           sensorGyro)
#pragma config(Sensor, in4,    LineR,          sensorLineFollower)//tgr 2600
#pragma config(Sensor, in5,    LineL,          sensorLineFollower)//tgr2500
#pragma config(Sensor, dgtl1,  RH,             sensorTouch)
#pragma config(Sensor, dgtl2,  led1,           sensorLEDtoVCC)
#pragma config(Sensor, dgtl3,  led2,           sensorLEDtoVCC)
#pragma config(Sensor, dgtl4,  led3,           sensorLEDtoVCC)
#pragma config(Sensor, dgtl5,  led4,           sensorLEDtoVCC)
#pragma config(Sensor, dgtl8,  solenoid2,      sensorDigitalOut)
#pragma config(Sensor, dgtl9,  solenoid,       sensorDigitalOut)
#pragma config(Sensor, I2C_1,  rightIEM,       sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  leftIEM,        sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port2,           frontRight,    tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port3,           backRight,     tmotorServoContinuousRotation, openLoop, encoder, encoderPort, I2C_1, 1000))
#pragma config(Motor,  port4,           roller,        tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port5,           armmotL,       tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port6,           conveyor,      tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port7,           armmotR,       tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port8,           backLeft,      tmotorServoContinuousRotation, openLoop, reversed, encoder, encoderPort, I2C_2, 1000))
#pragma config(Motor,  port9,           frontLeft,     tmotorServoContinuousRotation, openLoop, reversed)

//#include "Vex_Competition_v2.0.c"
#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!
//#include "Vex_Competition_v3.0_autonomus.h"
int joystick,StartingTile,encoderL,encoderR,gyroref,kMaxPos = 3000;
bool fire=false,passL,passR,bpassL,bpassR;


void drift(int power ,int howfar);
void toLine(int power);
void pre_auton(){
	bStopTasksBetweenModes = true;

	}
task usercontrol(){}
task autonomous()
{
toline(35);
}
void toLine(int power)
{
		passL=passR=false;
		bpassL=bpassR=true;
	while(!passL & !passR)
	{
	  if(SensorValue[LineR] > 2600){
			motor[backRight] = power;
			motor[frontRight] = power;}
		if(SensorValue[LineR] < 2600){
			passR=true;
			motor[backRight] = 0;
			motor[frontRight] = 0;}

		if(SensorValue[LineL] > 2600){
			motor[backLeft] = power;
			motor[frontLeft] = power;}
		if(SensorValue[LineL] < 2600){
			passL=true;
			motor[backLeft] = 0;
			motor[frontLeft] = 0;}
	}
	if(SensorValue[LineR] < 2600){
			passR=true;
			motor[backRight] = 0;
			motor[frontRight] = 0;}
  motor[frontRight] = -power/3;
  motor[backRight] = -power/3;
  motor[frontLeft] = -power/3;
  motor[backLeft] = -power/3;
  wait10Msec(6);
  motor[frontRight] = 0;
  motor[backRight] = 0;
  motor[frontLeft] = 0;
  motor[backLeft] = 0;


  if(!passL){
	  while(!passL){
	  	if(SensorValue[LineL] > 2600){
				motor[backLeft] = power;
				motor[frontLeft] = power;}
			if(SensorValue[LineL] < 2600){
				passL=true;
				motor[backLeft] = 0;
				motor[frontLeft] = 0;
				passR=false;
				while(!passR){
					if(SensorValue[LineR] > 2600){
						motor[backRight] = -power;
						motor[frontRight] = -power;}
					if(SensorValue[LineR] < 2600){
						passR=true;
						motor[backRight] = 0;
						motor[frontRight] = 0;}
				}
				passL=false;
				while(!passL){
					if(SensorValue[LineL] > 2600){
						motor[backLeft] = power;
						motor[frontLeft] = power;}
					if(SensorValue[LineL] < 2600){
						passL=true;
						motor[backLeft] = 0;
						motor[frontLeft] = 0;}
				}
				passR=false;
				while(!passR){
					if(SensorValue[LineR] > 2600){
						motor[backRight] = -power;
						motor[frontRight] = -power;}
					if(SensorValue[LineR] < 2600){
						passR=true;
						motor[backRight] = 0;
						motor[frontRight] = 0;}
				}
			}
		}
	}
	if(!passR){
		while(!passR){
			if(SensorValue[LineR] > 2600){
				motor[backRight] = power;
				motor[frontRight] = power;}
			if(SensorValue[LineR] < 2600){
				passR=true;
				motor[backRight] = 0;
				motor[frontRight] = 0;
				passL=false;
				while(!passL){
					if(SensorValue[LineL] > 2600){
						motor[backLeft] = -power;
						motor[frontLeft] = -power;}
					if(SensorValue[LineL] < 2600){
						passL=true;
						motor[backLeft] = 0;
						motor[frontLeft] = 0;}
				}
				passR=false;
				while(!passR){
					if(SensorValue[LineR] > 2600){
						motor[backRight] = power;
						motor[frontRight] = power;}
					if(SensorValue[LineR] < 2600){
						passR=true;
						motor[backRight] = 0;
						motor[frontRight] = 0;}
				}
				passL=false;
				while(!passL){
					if(SensorValue[LineL] > 2600){
						motor[backLeft] = -power;
						motor[frontLeft] = -power;}
					if(SensorValue[LineL] < 2600){
						passL=true;
						motor[backLeft] = 0;
						motor[frontLeft] = 0;}
				}
			}
		}
	}
}


void drift(int power, int howfar)
{
	SensorValue[gyro]=0;
	nMotorEncoder[backRight] = 0;
	nMotorEncoder[backLeft] = 0;
	if(howfar>0)
	{
		while(nMotorEncoder[backLeft] < (howfar-15))
		{
			motor[frontLeft]=motor[backRight]=-127;
			motor[backLeft]=motor[frontRight]=127;
		}
		motor[frontRight] = 0;
	  motor[backRight] = 0;
	  motor[frontLeft] = 0;
  	motor[backLeft] = 0;
		if(SensorValue[gyro] > 0)
	  {
      while(SensorValue[gyro]>10|SensorValue[gyro]<-10)
      {
	    	motor[frontRight] = 30;
			  motor[backRight] = 30;
			  motor[frontLeft] = -30;
		  	motor[backLeft] = -30;
			}
    }
    else
    {
	    while(SensorValue[gyro]>10|SensorValue[gyro]<-10)
      {
	    	motor[frontRight] = -30;
			  motor[backRight] = -30;
			  motor[frontLeft] = 30;
		  	motor[backLeft] = 30;
			}
		}
		motor[frontRight] = 0;
	  motor[backRight] = 0;
	  motor[frontLeft] = 0;
  	motor[backLeft] = 0;
	}
	else
	{
		while(nMotorEncoder[backRight] > (howfar+15))
		{
			motor[frontLeft]=motor[backRight]=127;
			motor[backLeft]=motor[frontRight]=-127;
		}
		motor[frontRight] = 0;
	  motor[backRight] = 0;
	  motor[frontLeft] = 0;
  	motor[backLeft] = 0;
		if(SensorValue[gyro] > 0)
	  {
      while(SensorValue[gyro]>10|SensorValue[gyro]<-10)
      {
	    	motor[frontRight] = 30;
			  motor[backRight] = 30;
			  motor[frontLeft] = -30;
		  	motor[backLeft] = -30;
			}
    }
    else
    {
	    while(SensorValue[gyro]>10|SensorValue[gyro]<-10)
      {
	    	motor[frontRight] = -30;
			  motor[backRight] = -30;
			  motor[frontLeft] = 30;
		  	motor[backLeft] = 30;
			}
		}
		motor[frontRight] = 0;
	  motor[backRight] = 0;
	  motor[frontLeft] = 0;
  	motor[backLeft] = 0;
	}
}
