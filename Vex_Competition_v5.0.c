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
#pragma config(Sensor, dgtl6,  warning,        sensorLEDtoVCC)
#pragma config(Sensor, dgtl7,  solenoid,      sensorDigitalOut)
#pragma config(Sensor, dgtl8,  solenoid2,       sensorDigitalOut)
#pragma config(Sensor, dgtl9,  solenoid3,      sensorDigitalOut)
#pragma config(Sensor, dgtl10,  GO,             sensorTouch)
#pragma config(Sensor, I2C_1,  rightIEM,       sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  leftIEM,        sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port2,           frontRight,    tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port3,           backRight,     tmotorServoContinuousRotation, openLoop, ,reversed,encoder, encoderPort, I2C_1, 1000))
#pragma config(Motor,  port4,           roller,        tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port5,           armmotL,       tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port6,           conveyor,      tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port7,           armmotR,       tmotorServoContinuousRotation, openLoop, reversed)
#pragma config(Motor,  port8,           backLeft,      tmotorServoContinuousRotation, openLoop, encoder, encoderPort, I2C_2, 1000))
#pragma config(Motor,  port9,           frontLeft,     tmotorServoContinuousRotation, openLoop, reversed)


#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!
//#include "Vex_Competition_v3.0_autonomus.h"
/*----------------------------------------------------------------------------------------------------*/
bool liftDa,hangmode=false;
void mecanumdrive();
void GoRH();
void GoRH1();
void GoRM();
void GoRM1();
void GoBH();
void GoBH1();
void GoBM();
void GoBM1();
void GoSkill();
void drift(int power, int howfar);
void forward(int power,int howfar);
void backward(int power,int howfar);
void toLine(int power);
void turn(int power ,int angle);
void selfcheck();
task raiseOrLowerArmL();
task raiseOrLowerArmR();
task projectile();
task masterreset();
task autoslection();
task shift();
int joystick,StartingTile,encoderL,encoderR,gyroref,kMaxPos = 3000,npl,npr;
bool fire=false;


//-----------------------============================---------------------------==============================----------------

/////////////////////////////////////////////////////////////////////////////////////////
//
//                          Pre-Autonomous Functions
//
// You may want to perform some actions before the competition starts. Do them in the
// following function.
//
/////////////////////////////////////////////////////////////////////////////////////////

void pre_auton()
{
  SensorValue[solenoid3]=0;
	SensorValue[led1]=0;
  SensorValue[led2]=0;
  SensorValue[led3]=0;
  SensorValue[led4]=0;// Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
  StartTask(autoslection);// Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	StartingTile=5;

//All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}
void selfcheck()
{
	bool skip=false;
	nMotorEncoder[backRight]=0;
	nMotorEncoder[backLeft]=0;
	SensorValue[gyro]=0;
	ClearTimer(T2);
	while(time100[T2]<3){}
	if(abs(SensorValue[gyro])>50){
		SensorValue[warning]=1;
		skip=true;
	}
	else
		SensorValue[warning]=0;
	if(!skip)
	{
		motor[frontRight] = 127;
	  motor[backRight] = 127;
	  motor[frontLeft] = -127;
	  motor[backLeft] = -127;
		wait10Msec(13);
		motor[frontRight] = 0;
	  motor[backRight] = 0;
	  motor[frontLeft] = 0;
	  motor[backLeft] = 0;
		if(nMotorEncoder[backLeft]!=0&nMotorEncoder[backRight]!=0&SensorValue[gyro]<=-15)
			SensorValue[warning]=0;
		else
			SensorValue[warning]=1;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task
//
// This task is used to control your robot during the autonomous phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////
task autoslection()
{
	while(1){
		switch (StartingTile){
				case 1:
				SensorValue[led1]=0;SensorValue[led2]=0;SensorValue[led3]=0;SensorValue[led4]=1;
				break;
				case 2:
				SensorValue[led1]=0;SensorValue[led2]=0;SensorValue[led3]=1;SensorValue[led4]=0;
				break;
				case 3:
				SensorValue[led1]=0;SensorValue[led2]=0;SensorValue[led3]=1;SensorValue[led4]=1;
				break;
				case 4:
				SensorValue[led1]=0;SensorValue[led2]=1;SensorValue[led3]=0;SensorValue[led4]=0;
				break;
				case 5:
				SensorValue[led1]=0;SensorValue[led2]=1;SensorValue[led3]=0;SensorValue[led4]=1;
				break;
				case 6:
				SensorValue[led1]=0;SensorValue[led2]=1;SensorValue[led3]=1;SensorValue[led4]=0;
				break;
				case 7:
				SensorValue[led1]=0;SensorValue[led2]=1;SensorValue[led3]=1;SensorValue[led4]=1;
				break;
				case 8:
				SensorValue[led1]=1;SensorValue[led2]=0;SensorValue[led3]=0;SensorValue[led4]=0;
				break;
				case 9:
				SensorValue[led1]=1;SensorValue[led2]=1;SensorValue[led3]=1;SensorValue[led4]=1;
				break;

			}
		while(!SensorValue[RH]){}
		StartingTile++;
		while(SensorValue[RH]){}
		wait10Msec(50);
		if(SensorValue[RH]==0)
		{
			if(StartingTile>9)
				StartingTile=1;
		}
	}
}

task autonomous()
{
	//unfold
	liftDa=false;
  StopTask(usercontrol);
  StopTask(raiseOrLowerArmL);
  StopTask(raiseOrLowerArmR);
	StartTask(projectile);
	StartTask(shift);

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
		case 5:
			GoRH1();
			break;
		case 6:
			GoRM1();
			break;
		case 7:
			GoBH1();
			break;
		case 8:
			GoBM1();
			break;
		case 9:
			GoSkill();
			break;
		default:
			break;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 User Control Task
//
// This task is used to control your robot during the user control phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task usercontrol()
{
	// User control code here, inside the loop
	selfcheck();
	kMaxPos = 2810;
	StartTask(raiseOrLowerArmL);
  StartTask(raiseOrLowerArmR);
	StartTask(projectile);
	StartTask(autoslection);
	StartTask(shift);
	StartTask(masterreset);
  while(true) //Loop forever
  {
		mecanumdrive();
//		motor[armmot]=vexRT[Ch2];
		joystick=vexRT[Ch2];
		if(vexRT[Btn5U] == 1)
      motor[roller] = 127;
    else if(vexRT[Btn5D] == 1)
    	motor[roller] = -127;
    else
    	motor[roller] = 0;

		if(vexRT[Btn8DXmtr2]==1)
			kMaxPos=2400;
		else
			kMaxPos=2845;
		if(hangmode)
		{
			motor[armmotR]=vexRT[Ch3Xmtr2];
			motor[armmotL]=vexRT[Ch2Xmtr2];
			motor[conveyor] =0;
		}
		else
    	motor[conveyor] = vexRT[Ch3Xmtr2];



   if(SensorValue[armPotentiometerL]<=1550)
    {
    	liftDa=true;
    	if(joystick<=-40)
    			motor[armmotL]=motor[armmotR]=-40;
    	else
				motor[armmotL]=motor[armmotR]=joystick;
		}
		else
		{
			liftDa=false;
		}
	}
}



void mecanumdrive()
{
  //Create "deadzone" variables. Adjust threshold value to increase/decrease deadzone
	int X2 = 0, Y1 = 0, X1 = 0, threshold = 25;
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
  if(hangmode)
  {
 		motor[frontLeft] =0;
  	motor[frontRight] =0;
  	motor[backRight] =  -Y1;
  	motor[backLeft] =  -Y1;
	}
	else{
    motor[frontLeft] = Y1 + X2 + X1;
  	motor[frontRight] = Y1 - X2 - X1;
  	motor[backRight] =  Y1 - X2 + X1;
  	motor[backLeft] =  Y1 + X2 - X1;
	}
}

void GoSkill()
{
	kMaxPos=2400;
	StartTask(raiseOrLowerArmL);
	StartTask(raiseOrLowerArmR);
	motor[conveyor]=-128;
	motor[roller]=127;
	//unfold



	//seg1-a
	joystick=-128;
	wait10Msec(60);
	joystick=0;
	motor[conveyor]=0;
	motor[roller]=0;
	forward(128,2300);
	joystick=-128;
	motor[roller]=128;
//	forward(128,300);
	forward(50,900);

	backward(128,600);
	motor[roller]=10;
	joystick=128;
	turn(128,-90);
	forward(128,600);
	backward(128,130);
	turn(128,90);
	forward(128,450);
	turn(128,-50);
	turn(128,40);
	joystick=-128;
	toLine(-35);
	joystick=-0;
	backward(50,150);

	turn(128,-90);
	forward(80,850);
	wait10Msec(50);
	kMaxPos=2950;
	joystick=128;
	wait10Msec(100);
	forward(128,100);
	forward(60,430);
	motor[roller]=-128;
	motor[conveyor]=-128;


	//seg1-b  drive back
	wait10Msec(150);

	motor[conveyor]=0;
	kMaxPos=2950;
	joystick=128;
	backward(128,200);
	turn(128,-80);
	joystick=-128;
	backward(128,340);
	forward(128,100);
	toLine(35);

	forward(128,2450);
	motor[roller]=0;
	turn(128,-90);
	joystick=0;
	forward(128,1500);

while(!SensorValue[GO]){}
//segment 2-a
	motor[conveyor]=128;
	motor[roller]=127;
	joystick=-128;
	forward(127,700);


	forward(30,415);

	wait10Msec(80);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=25;
	wait10Msec(100);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=0;
	backward(127,400);
	joystick=0;
	motor[conveyor]=-20;
	fire=true;
	motor[roller]=0;
	backward(127,200);
	fire=false;
	backward(127,800);

	turn(128,-90);
	while(!SensorValue[GO]){}

	forward(128,650);
	turn(128,-90);
	forward(128,1300);
	kMaxPos=2950;
	joystick=128;
	wait10Msec(160);
	forward(128,500);
	motor[roller]=-128;
	motor[conveyor]=-128;
	//seg2-b  drive back
	wait10Msec(250);

	joystick=128;
	backward(128,250);
	kMaxPos=2450;
	joystick=0;
	turn(128,-175);
	forward(128,300);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=25;
	wait10Msec(40);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=0;
	turn(128,-85);
	turn(128,85);
	drift(128,300);
	turn(128,85);
	backward(128,200);




}

void GoRH()//if teammate no autonomus
{
	kMaxPos=2400;
	StartTask(raiseOrLowerArmL);
	StartTask(raiseOrLowerArmR);
	motor[roller]=127;//intake
	motor[conveyor]=-60;
	fire=true;
	wait10Msec(50);
	fire=false;
	wait10Msec(50);
	motor[conveyor]=80;
	//unfold

	forward(127,140);

	forward(30,265);

	wait10Msec(80);
	joystick=-128;
	backward(127,50);
	SensorValue[gyro]=0;
	wait10Msec(10);//drift side
	drift(128,-350);
	motor[roller]=10;
	turn(128,97);
	backward(128,400);
	turn(128,-15);
	backward(128,550);
	motor[conveyor]=0;
	turn(128,-90);

	joystick=0;
	backward(127,600);
	fire=true;
	backward(128,760);
	fire=false;
	turn(128,175);
	joystick=128;
	wait10Msec(80);
	forward(128,600);
	motor[roller]=-120;
	wait10Msec(120);
	backward(128,350);
/*	joystick=-128;

	turn(128,45);
	forward(128,550);
	turn(128,-30);
	forward(128,600);
	turn(128,-90);
	forward(128,500);
	toLine(50);
	backward(128,1000);
//	motor[backLeft]=motor[backRight]=motor[frontLeft]=motor[frontRight]=

/*	backward(128,700);
	turn(128,-160);
	motor[conveyor]=0;
	turn(128,-90);

	backward(128,200);

	forward(128,350);
	turn(128,30);
	backward(128,300);*/
}

void GoRH1()//if teammate no autonomus
{
	kMaxPos=2400;
	StartTask(raiseOrLowerArmL);
	StartTask(raiseOrLowerArmR);
	kMaxPos=2400;
	StartTask(raiseOrLowerArmL);
	StartTask(raiseOrLowerArmR);
	motor[roller]=127;//intake
	motor[conveyor]=-60;
	joystick=128;
	wait10Msec(80);
	motor[conveyor]=0;
	motor[roller]=0;
	//unfold
	forward(127,500);
	backward(127,350);
	turn(128,50);
	forward(128,800);

	joystick=-128;
	backward(128,800);
	joystick=0;
	while(!SensorValue[GO]){}

	forward(128,650);
	turn(128,-90);
	forward(128,1300);
	kMaxPos=2950;
	joystick=128;
	wait10Msec(160);
	forward(128,500);
	motor[roller]=-128;
	motor[conveyor]=-128;

}

void GoRM()
{
	kMaxPos=2400;
	StartTask(raiseOrLowerArmL);
	StartTask(raiseOrLowerArmR);
	joystick=127;
	motor[conveyor]=127;
	wait10Msec(50);
	motor[roller]=127;//intake
	wait10Msec(50);
	motor[conveyor]=0;
	motor[roller]=-127;

	forward(127,800);
	backward(127,100);
	turn(128,95);
	forward(127,1000);
	wait1Msec(1);
	turn(128,-48);
	turn(128,75);

  toLine(38);

  joystick=-128;

  /*
	forward(128,250);
	turn(128,-43);
	joystick=0;
	wait1Msec(1);
	forward(128,2150);
	joystick=128;
  turn(128,90);
  turn(128,-33);
	fire=true;
	forward(128,1500);
	fire=false;
	turn(128,100);  */
}

void GoRM1()
{
	kMaxPos=2400;
	StartTask(raiseOrLowerArmL);
	StartTask(raiseOrLowerArmR);
	joystick=127;
	motor[conveyor]=127;
	wait10Msec(50);
	motor[roller]=127;//intake
	wait10Msec(50);
	motor[conveyor]=0;
	motor[roller]=-127;

	forward(127,900);
	backward(127,100);
	turn(128,100);
	forward(127,1200);
	wait1Msec(1);
	turn(128,-48);
	turn(128,58);
  joystick=-128;
  forward(128,800);
  turn(128,32);
  fire=true;
  forward(128,2000);
  fire=false;


}

void GoBH()
{
		kMaxPos=2400;
	StartTask(raiseOrLowerArmL);
	StartTask(raiseOrLowerArmR);
	motor[conveyor]=127;
	fire=true;
	wait10Msec(50);
	fire=false;
	motor[roller]=127;//intake
	motor[conveyor]=-60;
	wait10Msec(50);
	motor[conveyor]=80;
	motor[roller]=0;
	//unfold

	forward(127,80);

	motor[roller]=127;
	forward(30,265);

	wait10Msec(80);
	joystick=-128;
	backward(127,50);
	SensorValue[gyro]=0;
	wait10Msec(10);//drift side
	drift(128,350);
	motor[roller]=10;
	turn(128,-97);
	backward(128,400);
	turn(128,15);
	backward(128,550);
	motor[conveyor]=0;
	turn(128,90);

	joystick=0;
	backward(127,600);
	fire=true;
	backward(128,760);
	fire=false;
	turn(128,-175);
	joystick=128;
	wait10Msec(80);
	forward(128,600);
	motor[roller]=-120;
	wait10Msec(120);
	backward(128,350);
}

void GoBH1()//if teammate no autonomus
{
	kMaxPos=2400;
	StartTask(raiseOrLowerArmL);
	StartTask(raiseOrLowerArmR);
	motor[conveyor]=127;
	wait10Msec(50);
	motor[roller]=127;//intake
	wait10Msec(50);
	motor[conveyor]=0;
	motor[roller]=0;
	//unfold
	forward(127,350);
	motor[roller]=127;
	fire=true;
	forward(127,300);
	fire=false;
	wait10Msec(120);
	motor[conveyor]=127;
	backward(127,80);
	SensorValue[gyro]=0;
	wait10Msec(10);//drift side

	//power turn
	SensorValue[gyro]=0;
	wait10Msec(10);
  while(SensorValue[gyro] >= -900 )
  {
    motor[frontRight] = 127;
    motor[backRight] = 127;
    motor[frontLeft] = -127;
    motor[backLeft] = -127;
	}
	motor[frontRight] = 0;
  motor[backRight] = 0;
  motor[frontLeft] = 0;
  motor[backLeft] = 0;
	//--------- fire

	motor[conveyor]=0;
	//-------------
	backward(127,1400);
	turn(127,105);
	fire=true;
	wait10Msec(10);
	fire=false;
	backward(127,1000);
	forward(127,1000);
	drift(128,600);
	turn(128,165);
	forward(127,500);
	joystick=128;
	forward(127,700);
	motor[roller]=-127;

}

void GoBM()
{
	kMaxPos=2400;
	StartTask(raiseOrLowerArmL);
	StartTask(raiseOrLowerArmR);
	joystick=127;
	motor[conveyor]=127;
	wait10Msec(50);
	motor[roller]=127;//intake
	wait10Msec(50);
	motor[conveyor]=0;
	motor[roller]=-127;

	forward(127,900);
	backward(127,100);
	turn(128,-90);
	forward(127,1100);
	wait1Msec(1);
	turn(128,55);
	turn(128,-53);
  joystick=-128;
	forward(128,520);
	turn(128,48);
	joystick=0;
	forward(128,2030);
	joystick=128;
  turn(128,-80);
	turn(128,40);
	fire=true;
  forward(128,1400);
  fire=false;
	turn(128,-100);
}

void GoBM1()
{
		kMaxPos=2400;
	StartTask(raiseOrLowerArmL);
	StartTask(raiseOrLowerArmR);
	joystick=127;
	motor[conveyor]=127;
	wait10Msec(50);
	motor[roller]=127;//intake
	wait10Msec(50);
	motor[conveyor]=0;
	motor[roller]=-127;

	forward(127,900);
	backward(127,100);
	turn(128,-88);
	forward(127,1000);
	wait1Msec(1);
	turn(128,55);
	turn(128,-50);
  joystick=-128;
  forward(128,1100);
  turn(128,-30);
  fire=true;
  forward(128,2000);
  fire=false;


}

void turn(int power,int angle)
{
	int decidegrees = 10*angle;
	int error = 5;

  //While the absolute value of the gyro is less than the desired rotation - 100...
	if(decidegrees>=0)
	{
		SensorValue[gyro]=0;
	  while(abs(SensorValue[gyro]) < decidegrees-100 )
	  {
	    motor[frontRight] = -power;
	    motor[backRight] = -power;
	    motor[frontLeft] = power;
	    motor[backLeft] = power;
	  }
	  //Brief brake to eliminate some drift
	  motor[frontRight] = 5;
	  motor[backRight] = 5;
	  motor[frontLeft] = -5;
	  motor[backLeft] = -5;
	  wait1Msec(100);

	  //Second while loop to move the robot more slowly to its goal, also setting up a range
	  //for the amount of acceptable error in the system
	  ClearTimer(T1);
	  while(abs(SensorValue[gyro]) > decidegrees + error || abs(SensorValue[gyro]) < decidegrees - error)
	  {
	    if(abs(SensorValue[gyro]) > decidegrees)
	    {
	      motor[frontRight] = -30;
			  motor[backRight] = -30;
			  motor[frontLeft] = 30;
			  motor[backLeft] = 30;
	    }
	    else
	    {
	      motor[frontRight] = 30;
			  motor[backRight] = 30;
			  motor[frontLeft] = -30;
			  motor[backLeft] = -30;
			}
			if(time10[T1]>abs(angle/4)/(power/128)){break;}
		}
	}
	else
	{
		SensorValue[gyro]=0;
		wait1Msec(1);
		while(SensorValue[gyro] > decidegrees+100 )
	  {
	  	wait1Msec(1);
	    motor[frontRight] = power;
	    motor[backRight] = power;
	    motor[frontLeft] = -power;
	    motor[backLeft] = -power;
	  }
	  //Brief brake to eliminate some drift
	  motor[frontRight] = -5;
	  motor[backRight] = -5;
	  motor[frontLeft] = 5;
	  motor[backLeft] = 5;
	  wait1Msec(100);

	  //Second while loop to move the robot more slowly to its goal, also setting up a range
	  //for the amount of acceptable error in the system
	  ClearTimer(T1);
	  while(abs(SensorValue[gyro]) > decidegrees + error || abs(SensorValue[gyro]) < decidegrees - error)
	  {
	    if(abs(SensorValue[gyro]) < decidegrees)
	    {
	      motor[frontRight] = -20;
			  motor[backRight] = -20;
			  motor[frontLeft] = 20;
			  motor[backLeft] = 20;
	    }
	    else
	    {
	      motor[frontRight] = 20;
			  motor[backRight] = 20;
			  motor[frontLeft] = -20;
			  motor[backLeft] = -20;
			}
			if(time10[T1]>abs(angle/4)/(power/128)){break;}
		}

	}
  //Stop
  motor[frontRight] = 0;
  motor[frontRight] = 0;
  motor[frontLeft] = 0;
  motor[backLeft] = 0;

}


void drift(int power, int howfar)
{
	SensorValue[gyro]=0;
	nMotorEncoder[backRight] = 0;
	nMotorEncoder[backLeft] = 0;
	if(howfar>0)
	{
		while(nMotorEncoder[backRight] < (howfar-15))
		{
			motor[frontLeft]=motor[backRight]=127;
			motor[backLeft]=motor[frontRight]=-127;
		}
		motor[frontRight] = 0;
	  motor[backRight] = 0;
	  motor[frontLeft] = 0;
  	motor[backLeft] = 0;
  	ClearTimer(T1);
		if(SensorValue[gyro] > 0)
	  {
      while(SensorValue[gyro]>10|SensorValue[gyro]<-10)
      {
	    	motor[frontRight] = 30;
			  motor[backRight] = 30;
			  motor[frontLeft] = -30;
		  	motor[backLeft] = -30;
		  	if(time10[T1]>howfar/2){break;}
			}
    }
    else
    {
	    while(SensorValue[gyro]>10|SensorValue[gyro]<-10)
      {
	    	motor[frontRight] = -40;
			  motor[backRight] = -40;
			  motor[frontLeft] = 40;
		  	motor[backLeft] = 40;
		  	if(time10[T1]>howfar/2){break;}
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
			motor[frontLeft]=motor[backRight]=-127;
			motor[backLeft]=motor[frontRight]=+127;
		}
		motor[frontRight] = 0;
	  motor[backRight] = 0;
	  motor[frontLeft] = 0;
  	motor[backLeft] = 0;
  	ClearTimer(T1);
		if(SensorValue[gyro] > 0)
	  {
      while(SensorValue[gyro]>10|SensorValue[gyro]<-10)
      {
	    	motor[frontRight] = 30;
			  motor[backRight] = 30;
			  motor[frontLeft] = -30;
		  	motor[backLeft] = -30;
		  	if(time10[T1]>howfar/2){break;}
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
		  	if(time10[T1]>howfar/2){break;}
			}
		}
		motor[frontRight] = 0;
	  motor[backRight] = 0;
	  motor[frontLeft] = 0;
  	motor[backLeft] = 0;
	}
}

void forward(int power,int howfar)
{
	nMotorEncoder[backRight] = 0;
	nMotorEncoder[backLeft] = 0;
	while(nMotorEncoder[backRight] < (howfar-15) | nMotorEncoder[backLeft] < (howfar-15))
	{
		if(nMotorEncoder[backRight] < howfar ){
			motor[backRight] = power;
			motor[frontRight] = power;}
		else if(nMotorEncoder[backRight]>howfar){
			motor[backRight] = 0;
			motor[frontRight] = 0;}

		if(nMotorEncoder[backLeft] < howfar ){
			motor[backLeft] = power;
			motor[frontLeft] = power;}
		else if(nMotorEncoder[backLeft]>howfar){
			motor[backLeft] = 0;
			motor[frontLeft] = 0;}
		}
  motor[frontRight] = 0;
  motor[backRight] = 0;
  motor[frontLeft] = 0;
  motor[backLeft] = 0;
}

void backward(int power,int howfar)
{
	nMotorEncoder[backRight] = 0;
	nMotorEncoder[backLeft] = 0;
	while(nMotorEncoder[backRight] > (-howfar+15) | nMotorEncoder[backLeft] > (-howfar+15))
	{
		if(nMotorEncoder[backRight] > -howfar ){
			motor[backRight] = -power;
			motor[frontRight] = -power;}
		else if(nMotorEncoder[backRight]<-howfar){
			motor[backRight] = 0;
			motor[frontRight] = 0;}

		if(nMotorEncoder[backLeft] > -howfar ){
			motor[backLeft] = -power;
			motor[frontLeft] = -power;}
		else if(nMotorEncoder[backLeft]<-howfar){
			motor[backLeft] = 0;
			motor[frontLeft] = 0;}
		}
  motor[frontRight] = 0;
  motor[backRight] = 0;
  motor[frontLeft] = 0;
  motor[backLeft] = 0;
}

//-----------------------------------------------------------------------------------

task projectile()
{
	while(1){
		while(vexRT[Btn7DXmtr2] == 1|fire)         // If button 6U (upper right shoulder button) is pressed:
    {
      SensorValue[solenoid] = 1;
      SensorValue[solenoid2] = 1;  // ...activate the solenoid.
      encoderL=nMotorEncoder[backLeft];
      encoderR=nMotorEncoder[backRight];
      gyroref=SensorValue[gyro];
    }
    	gyroref=SensorValue[gyro];
    	SensorValue[solenoid] = 0;
      SensorValue[solenoid2] = 0;
      encoderL=nMotorEncoder[backLeft];
      encoderR=nMotorEncoder[backRight];
	}
}
task shift()
{
	while(1){
		while(vexRT[Btn7U] == 0)
		{
			if(vexRT[Btn8R])//hangmode
			{
				while(vexRT[Btn8R]){}
				liftDa=true;
				hangmode=true;
				SensorValue[solenoid3]=1;
				StopTask(raiseOrLowerArmL);
				StopTask(raiseOrLowerArmR);
			}
		}
		SensorValue[solenoid3] =!SensorValue[solenoid3];
		while(vexRT[Btn7U] == 1){}

	}
}
task masterreset()
{
	while(1){
		while(vexRT[Btn8U] == 1){}
		while(vexRT[Btn8U] == 0){}
		StopTask(raiseOrLowerArmL);
		StopTask(raiseOrLowerArmR);
		StopTask(usercontrol);
		StopTask(shift);
		StopTask(projectile);
		StopTask(autoslection);
		StartTask(raiseOrLowerArmL);
		StartTask(raiseOrLowerArmR);
		StartTask(usercontrol);
		StartTask(shift);
		StartTask(projectile);
		StartTask(autoslection);
		hangmode=false;
		liftDa=false;
		SensorValue[solenoid3]=0;
	}
}
//-----------------------------------------------------------------------------------

task raiseOrLowerArmL()
{
  const int kSlowSpeedRange = 50;
  const int kMinPos = 1280;

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
	      nPositionToHold = ((SensorValue[armPotentiometerL]+SensorValue[armPotentiometerL])/2)+100;
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
	      const int kUpdateCycle    = 50;
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
  const int kMinPos = 1430;

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
	      nPositionToHold = ((SensorValue[armPotentiometerL]+SensorValue[armPotentiometerR])/2)-100;
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
	      const int kUpdateCycle    = 50;
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

void toLine(int power)
{
	bool passL=false,passR=false;
	while(!passL & !passR)
	{
	  if(SensorValue[LineR] > 2700){
			motor[backRight] = power;
			motor[frontRight] = power;}
		if(SensorValue[LineR] < 2700){
			passR=true;
			motor[backRight] = 0;
			motor[frontRight] = 0;}

		if(SensorValue[LineL] > 2700){
			motor[backLeft] = power;
			motor[frontLeft] = power;}
		if(SensorValue[LineL] < 2700){
			passL=true;
			motor[backLeft] = 0;
			motor[frontLeft] = 0;}
	}
	if(SensorValue[LineR] < 2700){
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
	  	if(SensorValue[LineL] > 2700){
				motor[backLeft] = 1.5*power;
				motor[frontLeft] = 1.5*power;}
			if(SensorValue[LineL] < 2700){
				passL=true;
				motor[backLeft] = 0;
				motor[frontLeft] = 0;
				passR=false;
				while(!passR){
					if(SensorValue[LineR] > 2700){
						motor[backRight] = -1.5*power;
						motor[frontRight] = -1.5*power;}
					if(SensorValue[LineR] < 2700){
						passR=true;
						motor[backRight] = 0;
						motor[frontRight] = 0;}
				}
				passL=false;
				while(!passL){
					if(SensorValue[LineL] > 2700){
						motor[backLeft] = 1.5*power;
						motor[frontLeft] = 1.5*power;}
					if(SensorValue[LineL] < 2700){
						passL=true;
						motor[backLeft] = 0;
						motor[frontLeft] = 0;}
				}
				passR=false;
				while(!passR){
					if(SensorValue[LineR] > 2700){
						motor[backRight] = -1.5*power;
						motor[frontRight] = -1.5*power;}
					if(SensorValue[LineR] < 2700){
						passR=true;
						motor[backRight] = 0;
						motor[frontRight] = 0;}
				}
			}
		}
	}
	if(!passR){
		while(!passR){
			if(SensorValue[LineR] > 2700){
				motor[backRight] = 1.5*power;
				motor[frontRight] = 1.5*power;}
			if(SensorValue[LineR] < 2700){
				passR=true;
				motor[backRight] = 0;
				motor[frontRight] = 0;
				passL=false;
				while(!passL){
					if(SensorValue[LineL] > 2700){
						motor[backLeft] = -1.5*power;
						motor[frontLeft] = -1.5*power;}
					if(SensorValue[LineL] < 2700){
						passL=true;
						motor[backLeft] = 0;
						motor[frontLeft] = 0;}
				}
				passR=false;
				while(!passR){
					if(SensorValue[LineR] > 2700){
						motor[backRight] = 1.5*power;
						motor[frontRight] = 1.5*power;}
					if(SensorValue[LineR] < 2700){
						passR=true;
						motor[backRight] = 0;
						motor[frontRight] = 0;}
				}
				passL=false;
				while(!passL){
					if(SensorValue[LineL] > 2700){
						motor[backLeft] = -1.5*power;
						motor[frontLeft] = -1.5*power;}
					if(SensorValue[LineL] < 2700){
						passL=true;
						motor[backLeft] = 0;
						motor[frontLeft] = 0;}
				}
			}
		}
	}
}
