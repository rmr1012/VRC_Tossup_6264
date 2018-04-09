#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    armPotentiometerL, sensorPotentiometer)
#pragma config(Sensor, in7,    LineB, sensorPotentiometer)
#pragma config(Sensor, in3,    LineL,          sensorLineFollower)
#pragma config(Sensor, in4,    LineM,          sensorLineFollower)
#pragma config(Sensor, in5,    LineR,          sensorLineFollower)
#pragma config(Sensor, in6,    nPowerex,       sensorAnalog)
#pragma config(Sensor, in2,    forkPot,        sensorPotentiometer)
#pragma config(Sensor, in8,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  GOL,            sensorTouch)
#pragma config(Sensor, dgtl2,  solBig,         sensorDigitalOut)
#pragma config(Sensor, dgtl3,  solShift,       sensorDigitalOut)
#pragma config(Sensor, dgtl4,  fork1,          sensorDigitalOut)
#pragma config(Sensor, dgtl5,  fork2,          sensorDigitalOut)
#pragma config(Sensor, dgtl6,  GOR,            sensorTouch)
#pragma config(Sensor, dgtl9,  prgenc,         sensorQuadEncoder)
#pragma config(Sensor, dgtl11, winchencoder,   sensorQuadEncoder)
#pragma config(Sensor, I2C_1,  rightIEM,       sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  leftIEM,        sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port2,           frontRight,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           backRight,     tmotorVex393_MC29,  reversed, encoderPort, I2C_1)
#pragma config(Motor,  port4,           roller,        tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           armmotL,       tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           fork,          tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           armmotR,        tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           backLeft,      tmotorVex393_MC29, encoderPort, I2C_2)
#pragma config(Motor,  port9,           frontLeft,     tmotorVex393_MC29, openLoop, reversed)

#pragma platform(VEX)
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)
#define LCD_left nLCDButtons==0x01
#define LCD_mid nLCDButtons==0x02
#define LCD_right nLCDButtons==0x04
#define eyeL (SensorValue[LineL]<threshold)
#define eyeM (SensorValue[LineM]<threshold)
#define eyeR (SensorValue[LineR]<threshold)
#define eyeB (SensorValue[LineB]<threshold)

#include "Vex_Competition_Includes.c"//Main competition background code...do not modify!
//#include "Dennis++_v1.0.h"


//autonumos options
void GoRH();
void GoRM();
void GoBH();
void GoBM();
void GoRH2();
void GoBH2();
void GoDrive();
void GoSkill();
void GoSkillRisk();

//calibration mothods
void gyrocali();
void encodercali();

//maneuver mothods
void drift(int power, int howfar);
void forward(int power,int howfar);
void backward(int power,int howfar);
void turn(int power ,float angle);
void turnIEC(int power,int degrees);

//breaking mothods
void breakDrive();
void breakDrivet();
void breakDrivesideL();
void breakDrivesideR();

//line tracking mothods
void turnthenfollow(int power,int howfar,int approx);
void forwardLine(int power,int howfar);
void winchup (int howfar);
void toLine(int power);
void mecanumdrive();
void securedDump();
void autohang();

//servace/background tasks
task raiseOrLowerArm();
task projectile();
task masterreset();
task autoslection();
task shift();
task raiseOrLowerFork();
task openFork();
task visualMem();


//global variable delarations
int threshold=600,lfc=0;
int gyrocorrection=0,joystick,joystickf,StartingTile,encoderL,encoderR,gyroref,kMaxPos = 3000,linethreshold=2400;
bool fire=false,liftDa=false,hangmode=false,skipcali=false,openup=false;
bool eyeLm=false,eyeMm=false,eyeRm=false,eyeBm=false,memEn=false;//eye memory section;
float ki=0.0001,kd=0.40,kp=0.48;//arm PID consts
float kdf=0,kpf=0.17,kif=0;//fork PID consts
float kil=0.00,kdl=0,kpl=15;// Line following PID consts
int kMaxPosf = 3100;
int nPositionToHold;
int kMotorSpeedUp   = +127;


void pre_auton()
{
	SensorValue[solShift]=0;
	clearLCDLine(0);
	clearLCDLine(1);// Set bstopTasksBetweenModes to false if you want to keep user created tasks running between
	// Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
	SensorType[in8] = sensorNone;
	SensorType[in7] = sensorNone;
	wait1Msec(1000);
	//Reconfigure Analog Port 8 as a Gyro sensor and allow time for ROBOTC to calibrate it
	SensorType[in8] = sensorGyro;
	SensorType[in7] = sensorPotentiometer;

	wait1Msec(400);
	SensorValue[gyro]=0;
	bStopTasksBetweenModes = true;
	bLCDBacklight = true;
	StartingTile=7;

	startTask(autoslection);
	//	startTask(tune);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task

task autoslection()
{
	string mBattery,eBattery,fkpt;
	wait1Msec(10);
	while(1){
		while(nLCDButtons!=kButtonLeft)
		{
			//clearLCDLine(1);
			sprintf(mBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0);
			sprintf(eBattery, "%1.2f%c", (SensorValue[nPowerex]/284.0));
			sprintf(fkpt, "%4.0f%c", (SensorValue[forkPot]));
			displayLCDString(1,0,mBattery);
			displayLCDString(1,5,eBattery);
			writeDebugStreamLine(fkpt);

			if(hangmode)
				displayLCDString(1,10,"hang");
			else
				displayLCDString(1,10,"normal");
			if(LCD_mid)
			{
				while(LCD_mid);
				gyrocali();

			}
			if(LCD_right)
			{
				while(LCD_right);
				encodercali();
			}
			wait1Msec(100);
			clearLCDLine(0);
			switch (StartingTile){
			case 1:
				displayLCDPos(0,0); displayNextLCDString("Red Hanging");
				break;
			case 2:
				displayLCDPos(0,0); displayNextLCDString("Red Hanging 2");
				break;
			case 3:
				displayLCDPos(0,0); displayNextLCDString("Red Middle");
				break;
			case 4:
				displayLCDPos(0,0); displayNextLCDString("Bule Hanging");
				break;
			case 5:
				displayLCDPos(0,0); displayNextLCDString("Bule Hanging 2");
				break;
			case 6:
				displayLCDPos(0,0); displayNextLCDString("Blue Middle");
				break;
			case 7:
				displayLCDPos(0,0); displayNextLCDString("Challenge");
				break;
			case 8:
				displayLCDPos(0,0); displayNextLCDString("Challenge Risk");
				break;

			}
		}
		StartingTile++;
		while(LCD_left){}
		wait1Msec(100);

		if(StartingTile>8)
			StartingTile=1;
	}
}

task autonomous()
{
	//unfold
	liftDa=false;
	stopTask(usercontrol);
	stopTask(raiseOrLowerArm);
	startTask(projectile);
	//	startTask(shift);

	switch(StartingTile)
	{
	case 1:
		GoRH();
		break;
	case 2:
		GoRH2();
		break;
	case 3:
		GoRM();
		break;
	case 4:
		GoBH();
		break;
	case 5:
		GoBH2();
		break;
	case 6:
		GoBM();
		break;
	case 7:
		GoSkill();
		break;
	case 8:
		GoSkillRisk();
		break;
	default:
		break;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 User Control Task

task usercontrol()
{
	bool forkopen=false;
	kMaxPos = 3070;
	kMaxPosf = 3100;
	startTask(raiseOrLowerArm);
	startTask(raiseOrLowerFork);
	startTask(projectile);
	startTask(autoslection);
	startTask(shift);
	startTask(openFork);
	startTask(masterreset);

	//	startTask(tune);
	while(true) //Loop forever
	{
		mecanumdrive();
		joystick=vexRT[Ch2];

		if(vexRT[Btn6UXmtr2])//action
		{
			while(vexRT[Btn6UXmtr2]){}
			forkopen=!forkopen;
			if(forkopen)
				openup=true;
			else
				openup=false;
		}

		joystickf = vexRT[Ch2Xmtr2];
		if(vexRT[Btn5U] == 1)
			motor[roller] = 127;
		else if(vexRT[Btn5D] == 1)
			motor[roller] = -127;
		else
			motor[roller] = 0;
		if(vexRT[Btn7R]==1)
		{
		while(vexRT[Btn7R]==1){}
			GoDrive();
		}
		/*	if(vexRT[Btn8DXmtr2]==1)
		kMaxPos=2400;
		else
		kMaxPos=2845;*/
		if(hangmode)
		{
			motor[armmotR]=motor[armmotL]=vexRT[Ch3Xmtr2];
		}
		else{}
		if(SensorValue[armPotentiometerL]<=1500&!hangmode)
		{
			liftDa=true;
			if(joystick<=-70)
				motor[armmotL]=motor[armmotR]=joystick/3;
			else if(joystick>=100)
				motor[armmotL]=motor[armmotR]=joystick;
			else if(SensorValue[armPotentiometerL]>1400)
				motor[armmotL]=motor[armmotR]=-24;
			else if(SensorValue[armPotentiometerL]<1430)
				motor[armmotL]=motor[armmotR]=-8;
		}
		else if(SensorValue[armPotentiometerL]>=1500&!hangmode)
		{
			liftDa=false;
		}
	}
}

void mecanumdrive()
{
	int X2 = 0, Y1 = 0, X1 = 0, threshold = 25;
	if(abs(vexRT[Ch3]) > threshold)
		Y1 = vexRT[Ch3];
	else
		Y1 = 0;
	if(abs(vexRT[Ch4]) > threshold)
		X1 = vexRT[Ch4];
	else
		X1 = 0;
	if(abs(vexRT[Ch1]) > threshold)
		X2 = vexRT[Ch1];
	else
		X2 = 0;
	if(hangmode)
	{
		motor[frontLeft] = Y1 + X2 + X1;
		motor[frontRight] = Y1 - X2 - X1;
		motor[backRight] =  vexRT[Ch2];
		motor[backLeft] =  vexRT(Ch2);
	}
	else if(!hangmode)
	{
		motor[frontLeft] = Y1 + X2 + X1;
		motor[frontRight] = Y1 - X2 - X1;
		motor[backRight] =  Y1 - X2 + X1;
		motor[backLeft] =  Y1 + X2 - X1;
	}
}

void GoSkill()
{
	SensorValue[solShift]=0;
	kMaxPos=3100;
	kMaxPosf = 1700;
	bool scan;
	startTask(raiseOrLowerArm);
	startTask(raiseOrLowerFork);
	startTask(openFork);
	startTask(visualMem);
	SensorValue[solBig]=1;

	wait1Msec(200);	//unfold
	SensorValue[solBig]=0;
	motor[roller]=127;//intake
	forward(127,300);
	liftDa=true;
	motor[armmotL]=motor[armmotR]=-20;
	joystickf=128;
	forward(50,200);
	breakDrive();



	wait1Msec(1100);

	backward(128,1430);
	motor[roller]=25;
	turn(128,-160);




	while(!(SensorValue[GOL]|SensorValue[GOR])){}
	motor[roller]=20;//intake
	stopTask(raiseOrLowerFork);
	forward(128,300);
	SensorValue[solBig]=1;
	motor[fork]=-40;
	forward(128,300);
	SensorValue[solBig]=0;
	joystick=0;
	memEn=false;//init memory
	forward(128,800);
	kMaxPos=3000;
	liftDa=false;
	joystick=128;

	joystick=128;
	forward(70,400);
	breakDrive();
	while(SensorValue[armPotentiometerL]<2800);
	forward(128,500);


	motor[roller]=-128;
	clearTimer(T3);
	memEn=true;
	while(time1[T3]<1600)
	{
		motor[backLeft]=-80;
		motor[backRight]=+80;//turncw
		while(!eyeB)
		{
			if(time1[T3]>1600)
				break;
		}
		motor[backLeft]=0;
		motor[backRight]=0;//stop
		breakDrivet();
		while(time1[T3]<1600);

	}
	memEn=false;

	startTask(raiseOrLowerFork);

	backward(128,300);
	motor[roller]=0;
	while(SensorValue[armPotentiometerL]>1400){
		joystick=-128;}
	joystick=0;
	liftDa=true;
	//	while(!(SensorValue[GOL]|SensorValue[GOR])){}


	motor[armmotL]=motor[armmotR]=0;


	backward(128,300);
	motor[roller]=0;
	motor[backRight]=motor[frontLeft]=80;
	motor[backLeft]=motor[frontRight]=-80;
	wait1Msec(400);
	motor[backRight]=motor[frontLeft]=0;
	motor[backLeft]=motor[frontRight]=0;
	stopTask(raiseOrLowerFork);
	motor[fork]=0;
	joystickf=0;
	backward(80,400);
	breakDrive();
	SensorValue[solBig]=1; //hits big ball
	wait1Msec(250);
	SensorValue[solBig]=0;

	backward(80,300);
	turn(128,75);
	backward(128,400);



	while(!(SensorValue[GOL]|SensorValue[GOR])){}
	startTask(raiseOrLowerFork);
	forward(128,2400);
	motor[roller]=127;
	liftDa=true;
	motor[armmotL]=motor[armmotR]=-20;
	forward(80,950);
	breakDrive();

	//	wait1Msec(1000);
	kMaxPos=1380;
	//	liftDa=false;
	//	joystick=128;
	//	nPositionToHold=1400;
	turn(128,90);
	joystick=0;
	kMaxPosf=2500;
	joystickf=128;
	openup=true;

	forward(127,500);
	joystick=-128;
	wait1Msec(700);
	backward(128,600);
	stopTask(raiseOrLowerArm);


	backward(128,530);
	forward(128,150);

	openup=false;
	liftDa=true;
	stopTask(raiseOrLowerArm);
	motor[armmotL]=motor[armmotR]=-25;
	turn(128,90);
	motor[armmotL]=motor[armmotR]=0;
	joystickf=-128;
	forward(100,200);

	//turn then follow


	stopTask(raiseOrLowerFork);
	motor[fork]=0;

	while(!(eyeM|eyeR|eyeL))//eyehits line)		//
	{
		motor[backLeft]=motor[frontLeft]=55;
		motor[backRight]=motor[frontRight]=55;
	}
	forward(60,70);
	stopTask(raiseOrLowerFork);
	breakDrive();
	motor[fork]=-40;
	SensorValue[gyro]=0;
	memEn=true;

	while( SensorValue[gyro]>-1400  &  (!(eyeM|eyeR|eyeL|eyeMm|eyeRm|eyeLm)) )
	{
		motor[frontRight] =	motor[backRight] = -80;
		motor[frontLeft] =motor[backLeft] =80;
	}
	breakDrivet();
	if((eyeL|eyeM|eyeR)&eyeB)//if perfect
	{

	}
	else if(eyeBm)//back passed 2A
	{
		while(!eyeB)//until it's good, drift and turn ccw
		{
			motor[backLeft]=-85;
			motor[backRight]=+80;//turncw
			while(!eyeB);
			motor[backLeft]=0;
			motor[backRight]=0;//stop
		}

	}
	else if(!eyeBm)//back not pass 2B
	{
		while(!eyeB)//until it's good, drift and turn cw
		{
			motor[backLeft]=85;
			motor[backRight]=-80;//turncw
			while(!eyeB);
			motor[backLeft]=0;
			motor[backRight]=0;//stop
		}
	}
	memEn=false;

/*	while(!eyeM&!eyeL&!eyeR)//eyehits line)		//drift
	{
		motor[backRight]=motor[frontLeft]=-79;
		motor[backLeft]=motor[frontRight]=80;
	}*/
	nMotorEncoder[backLeft]=nMotorEncoder[backRight]=0;
	kpl=15.0;
	//	while(!(SensorValue[GOL]|SensorValue[GOR])){}
	forwardLine(75,1100);
	motor[fork]=0;


	liftDa=false;
	kMaxPos=3050;
	startTask(raiseOrLowerArm);
	joystick=128;

	forward(70,250);
	while(SensorValue[armPotentiometerL]<2800);
	forward(128,350);

	motor[roller]=-128;
	joystick=0;
	wait1Msec(400);
	joystick=128;
	wait1Msec(400);
	joystick=0;
	wait1Msec(400);
	joystick=128;
	wait1Msec(400);
	joystick=0;
	backward(128,250);
	motor[roller]=0;
	startTask(raiseOrLowerFork);
	while(SensorValue[armPotentiometerL]>1380){
		joystick=-128;}
	joystick=0;
	liftDa=true;
	stopTask(raiseOrLowerArm);
	motor[armmotL]=motor[armmotR]=-25;
	motor[roller]=-128;
	kMaxPosf=1500;
	joystickf=128;
	backward(128,900);
	motor[armmotL]=motor[armmotR]=0;
	SensorValue[solBig]=1;
	wait1Msec(200);
	SensorValue[solBig]=0;
	wait1Msec(200);
	backward(128,600);
	nMotorEncoder[backLeft]=0;
	scan=true;
	motor[armmotL]=motor[armmotR]=-20;
	while(!eyeL&!eyeM&!eyeR)//eyehits line)		//drift
	{

		if(nMotorEncoder[backLeft]>300&scan)
		{
			scan=false;
			motor[backRight]=motor[frontLeft]=79;
			motor[backLeft]=motor[frontRight]=-80;
		}
		else if(nMotorEncoder[backLeft]<500&scan)
		{
			motor[backRight]=motor[frontLeft]=-79;
			motor[backLeft]=motor[frontRight]=80;
		}
		else if(!scan&nMotorEncoder[backLeft]>-300)
		{
			scan=false;
			motor[backRight]=motor[frontLeft]=79;
			motor[backLeft]=motor[frontRight]=-80;
		}
		else if(!scan&nMotorEncoder[backLeft]<-300){
			scan=true;
		}

	}
	motor[armmotL]=motor[armmotR]=0;
	motor[backLeft]=motor[frontLeft]=0;
	motor[backRight]=motor[frontRight]=0;


	backward(128,400);
	forward(128,20);

	joystickf=-128;
	openup=true;
	wait1Msec(300);
	backward(128,700);

	kMaxPosf=1700;
	joystickf=128;
	joystick=0;
	wait1Msec(500);
	forward(68,50);
	breakDrive();
	turnIEC(128,93);

	startTask(raiseOrLowerArm);
	liftDa=false;
	kMaxPos=3000;
	joystick=128;

	joystickf=0;
	forward(128,220);
	breakDrive();
	turnIEC(128,46);

	forward(128,200);
	breakDrive();


	////autohang

	stopTask(raiseOrLowerArm);
	SensorValue[solShift]=1;
	SensorValue[winchencoder]=0;
	motor[frontLeft]=motor[frontRight]=50;
	while(SensorValue[winchencoder]==0)
	{
		motor[backLeft]=motor[backRight]=80;
		wait1Msec(200);
		motor[backLeft]=motor[backRight]=-80;
		wait1Msec(200);
	}
	motor[backLeft]=motor[backRight]=0;
	motor[frontLeft]=motor[frontRight]=128;
	kMaxPosf=2700;
	joystickf=128;
	while(SensorValue[armPotentiometerL]<3000)
		motor[armmotL]=motor[armmotR]=128;
	SensorValue[solBig]=1;
	wait1Msec(1300);
	motor[armmotL]=motor[armmotR]=0;

	winchup(500);
	motor[frontLeft]=motor[frontRight]=0;
	joystickf=0;
	stopTask(raiseOrLowerFork);
	motor[fork]=8;
	motor[armmotL]=motor[armmotR]=-128;
	while(SensorValue[armPotentiometerL]>1450)
	{
		motor[backLeft]=motor[backRight]=128;
	}

	wait1Msec(200);
	//	motor[fork]=0;
	motor[backLeft]=motor[backRight]=100;
	wait1Msec(200);
	motor[backLeft]=motor[backRight]=80;
	wait1Msec(200);
	motor[backLeft]=motor[backRight]=60;
	wait1Msec(200);
	motor[backLeft]=motor[backRight]=40;
	wait1Msec(200);
	motor[armmotL]=motor[armmotR]=-10;
	SensorValue[solBig]=0;
	motor[backLeft]=motor[backRight]=20;
	wait1Msec(200);
	motor[backLeft]=motor[backRight]=10;
	SensorValue[solBig]=0;
	wait1Msec(1000);




}


void GoSkillRisk(){}

void GoRH()
{
	kMaxPos=3000;
	startTask(raiseOrLowerArm);
	motor[roller]=127;//intake
	wait1Msec(400);	//unfold
	forward(127,300);

	joystick=-128;
	forward(40,230);

	wait1Msec(1000);
	joystick=-0;
	stopTask(raiseOrLowerArm);
	motor[armmotL]=motor[armmotR]=0;
	backward(128,450);
	turn(128,-85);
	motor[roller]=10;//intake

	while(!(SensorValue[GOL]|SensorValue[GOR])){}
	forward(128,620);
	turn(128,-88);

	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=98;
	wait1Msec(100);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=0;
	wait1Msec(100);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=98;
	wait1Msec(100);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=0;
	wait1Msec(100);


	forward(80,850);
	startTask(raiseOrLowerArm);
	joystick=-128;
	forward(128,500);
	joystick=0;
	forward(128,650);
	joystick=128;
	while(SensorValue[armPotentiometerL]<2700);
	forward(50,150);
	forward(128,400);
	motor[roller]=-128;
}


void GoRM()
{
	kMaxPos=3050;
	startTask(raiseOrLowerArm);

	SensorValue[solBig]=1;
	wait1Msec(400);
	SensorValue[solBig]=0;
	//unfold

	motor[roller]=20;//intake
	stopTask(raiseOrLowerFork);
	forward(128,300);
	SensorValue[solBig]=1;
	motor[fork]=-40;
	forward(128,300);
	SensorValue[solBig]=0;
	joystick=0;
	memEn=false;//init memory
	forward(128,800);
	kMaxPos=3000;
	liftDa=false;
	joystick=128;

	joystick=128;
	forward(70,400);
	breakDrive();
	while(SensorValue[armPotentiometerL]<2800);
	forward(128,500);


	motor[roller]=-128;
	clearTimer(T3);
	memEn=true;
	while(time1[T3]<1300)
	{
		motor[backLeft]=-80;
		motor[backRight]=+80;//turncw
		while(!eyeB)
		{
			if(time1[T3]>3300)
				break;
		}
		motor[backLeft]=0;
		motor[backRight]=0;//stop
		breakDrivet();
		while(time1[T3]<1300);

	}
	memEn=false;

	startTask(raiseOrLowerFork);

	backward(128,300);
	motor[roller]=0;
	while(SensorValue[armPotentiometerL]>1400){
		joystick=-128;}
	joystick=0;
	liftDa=true;
	//	while(!(SensorValue[GOL]|SensorValue[GOR])){}


	motor[armmotL]=motor[armmotR]=0;


	backward(128,300);
	motor[roller]=0;
	motor[backRight]=motor[frontLeft]=80;
	motor[backLeft]=motor[frontRight]=-80;
	wait1Msec(300);
	motor[backRight]=motor[frontLeft]=0;
	motor[backLeft]=motor[frontRight]=0;
	stopTask(raiseOrLowerFork);
	motor[fork]=0;
	joystickf=0;
	backward(80,300);
	breakDrive();
	SensorValue[solBig]=1; //hits big ball
	wait1Msec(250);
	SensorValue[solBig]=0;

	backward(80,400);
	turn(128,75);

	backward(128,400);
}


void GoBH()
{
	kMaxPos=3000;
	startTask(raiseOrLowerArm);
	motor[roller]=127;//intake
	wait1Msec(400);	//unfold
	forward(127,300);

	joystick=-128;
	forward(40,230);

	wait1Msec(1000);
	joystick=-0;
	stopTask(raiseOrLowerArm);
	motor[armmotL]=motor[armmotR]=0;
	backward(128,450);
	turn(128,88);
	motor[roller]=10;//intake
	while(!(SensorValue[GOL]|SensorValue[GOR])){}
	forward(128,625);
	turn(128,90);


	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=98;
	wait1Msec(100);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=0;
	wait1Msec(100);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=98;
	wait1Msec(100);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=0;
	wait1Msec(100);


	forward(80,850);
	startTask(raiseOrLowerArm);
	joystick=-128;
	forward(128,500);
	joystick=0;
	forward(128,650);
	joystick=128;
	//	turn(128,-2);
	while(SensorValue[armPotentiometerL]<2700);
	forward(50,170);
	forward(128,400);
	motor[roller]=-128;
}



void GoBM()
{
	kMaxPos=3050;
	startTask(raiseOrLowerArm);

	SensorValue[solBig]=1;
	wait1Msec(400);
	SensorValue[solBig]=0;
	//unfold

	motor[roller]=20;//intake
	stopTask(raiseOrLowerFork);
	forward(128,300);
	SensorValue[solBig]=1;
	motor[fork]=-40;
	forward(128,300);
	SensorValue[solBig]=0;
	joystick=0;
	memEn=false;//init memory
	forward(128,800);
	kMaxPos=3000;
	liftDa=false;
	joystick=128;

	joystick=128;
	forward(70,400);
	breakDrive();
	while(SensorValue[armPotentiometerL]<2800);
	forward(128,500);

	//	while(SensorValue[armPotentiometerL]<2800);


	//	motor[frontRight] =motor[backRight]=motor[frontLeft] =motor[backLeft] =0;
	motor[roller]=-128;
	clearTimer(T3);
	memEn=true;
	while(time1[T3]<1300)
	{
		motor[backLeft]=80;
		motor[backRight]=-80;//turncw
		while(!eyeB)
		{
			if(time1[T3]>3300)
				break;
		}
		motor[backLeft]=0;
		motor[backRight]=0;//stop
		breakDrivet();
		while(time1[T3]<1300);

	}
	memEn=false;

	startTask(raiseOrLowerFork);

	backward(128,300);
	motor[roller]=0;
	while(SensorValue[armPotentiometerL]>1400){
		joystick=-128;}
	joystick=0;
	liftDa=true;
	//	while(!(SensorValue[GOL]|SensorValue[GOR])){}


	motor[armmotL]=motor[armmotR]=0;


	backward(128,300);
	motor[roller]=0;
	motor[backRight]=motor[frontLeft]=80;
	motor[backLeft]=motor[frontRight]=-80;
	wait1Msec(300);
	motor[backRight]=motor[frontLeft]=0;
	motor[backLeft]=motor[frontRight]=0;
	stopTask(raiseOrLowerFork);
	motor[fork]=0;
	joystickf=0;
	backward(80,300);
	breakDrive();
	SensorValue[solBig]=1; //hits big ball
	wait1Msec(250);
	SensorValue[solBig]=0;

	backward(80,400);
	turn(128,-75);

	backward(128,400);
}



void GoRH2()
{
	stopTask(projectile);
	startTask(openFork);
	startTask(raiseOrLowerFork);
//	wait1Msec(800);
	joystickf=-128;
	kMaxPosf=2600;
	openup=true;
	SensorValue[fork1]=SensorValue[fork2]=true;
	wait1Msec(500);
	backward(80,150);
//	breakDrive();
	SensorValue[solBig]=true;
//	breakDrive():
	wait1Msec(200);
	SensorValue[solBig]=false;
	kMotorSpeedUp   = 85;
	joystickf=128;
	backward(60,300);
	while(SensorValue[forkPot]<2700);
	kMotorSpeedUp   = 128;
	joystickf=-128;
//	backward(60,400);

	while(SensorValue[forkPot]>1200);
	backward(60,350);
	kMaxPosf=1800;
	joystickf=128;
	backward(80,350);
	turn(128,90);
	forward(128,1450);
	breakDrive();
	motor[roller]=-128;
	SensorValue[solBig]=true;
	wait1Msec(300);
	SensorValue[solBig]=false;
	kMaxPosf=3100;
	joystickf=128;
	while(SensorValue[forkPot]<2800);
	wait1Msec(300);
	SensorValue[solBig]=true;
	wait1Msec(300);
	SensorValue[solBig]=false;
	wait1Msec(1000);
	joystickf=-128;
	openup=false;
	backward(128,300);
	turn(128,180);
	motor[roller]=0;
	forward(128,1000);

}




void GoBH2()
{
	stopTask(projectile);
	startTask(openFork);
	startTask(raiseOrLowerFork);
//	wait1Msec(800);
	joystickf=-128;
	kMaxPosf=2600;
	openup=true;
	SensorValue[fork1]=SensorValue[fork2]=true;
	wait1Msec(500);
	backward(80,150);
//	breakDrive();
	SensorValue[solBig]=true;
//	breakDrive():
	wait1Msec(200);
	SensorValue[solBig]=false;
	kMotorSpeedUp   = 85;
	joystickf=128;
	backward(60,300);
	while(SensorValue[forkPot]<2700);
	kMotorSpeedUp   = 128;
	joystickf=-128;
//	backward(60,400);

	while(SensorValue[forkPot]>1200);

	backward(60,350);
	kMaxPosf=1800;
	joystickf=128;
	backward(80,350);
	turn(128,-90);
	forward(128,1450);
	breakDrive();
	motor[roller]=-128;
	SensorValue[solBig]=true;
	wait1Msec(300);
	SensorValue[solBig]=false;
	kMaxPosf=3100;
	joystickf=128;
	while(SensorValue[forkPot]<2800);
	wait1Msec(300);
	SensorValue[solBig]=true;
	wait1Msec(300);
	SensorValue[solBig]=false;
	wait1Msec(1000);
	joystickf=-128;
	openup=false;
	backward(128,300);
	turn(128,180);
	motor[roller]=0;
	forward(128,1000);

}

void GoDrive()
{
	stopTask(projectile);
	startTask(openFork);
	startTask(raiseOrLowerFork);
//	wait1Msec(800);
	joystickf=-128;
	kMaxPosf=2600;
	openup=true;
	SensorValue[fork1]=SensorValue[fork2]=true;
	wait1Msec(500);
	backward(80,150);
//	breakDrive();
	SensorValue[solBig]=true;
//	breakDrive():
	wait1Msec(200);
	SensorValue[solBig]=false;
	kMotorSpeedUp   = 85;
	joystickf=128;
	backward(60,300);
	while(SensorValue[forkPot]<2700);
	kMotorSpeedUp   = 128;
	joystickf=-128;
//	backward(60,400);

	while(SensorValue[forkPot]>1200);

	backward(60,350);
	kMaxPosf=1800;
	joystickf=128;
	backward(80,350);
	turn(128,-90);
	forward(128,1450);
	breakDrive();
	motor[roller]=-128;
	SensorValue[solBig]=true;
	wait1Msec(300);
	SensorValue[solBig]=false;
	kMaxPosf=3100;
	joystickf=128;
	while(SensorValue[forkPot]<2800);
	wait1Msec(300);
	SensorValue[solBig]=true;
	wait1Msec(300);
	SensorValue[solBig]=false;
	wait1Msec(1000);
	joystickf=-128;
	openup=false;
}


void turn(int power,float angle)
{
	int decidegrees = -10*angle;
	int error = 3;

	//While the absolute value of the gyro is less than the desired rotation - 100...
	if(decidegrees>=0)
	{
		decidegrees=(float)decidegrees+(decidegrees/900)*gyrocorrection;
		SensorValue[gyro]=0;
		while(abs(SensorValue[gyro]) < decidegrees-100 )
		{
			motor[frontRight] = power;
			motor[backRight] = power;
			motor[frontLeft] = -power;
			motor[backLeft] = -power;
		}
		motor[frontRight] = -5;
		motor[backRight] = -5;
		motor[frontLeft] = 5;
		motor[backLeft] = 5;
		wait1Msec(100);

		clearTimer(T3);
		while(SensorValue[gyro] > decidegrees + error || SensorValue[gyro] < decidegrees - error)
		{
			if(SensorValue[gyro] < decidegrees)
			{
				motor[frontRight] = 40;
				motor[backRight] = 40;
				motor[frontLeft] = -40;
				motor[backLeft] = -40;
			}
			else
			{
				motor[frontRight] = -40;
				motor[backRight] = -40;
				motor[frontLeft] = 40;
				motor[backLeft] = 40;
			}
		}
	}
	else
	{

		decidegrees=decidegrees-gyrocorrection;
		SensorValue[gyro]=0;
		wait1Msec(10);
		while(SensorValue[gyro] > decidegrees+100 )
		{
			wait1Msec(10);
			motor[frontRight] = -power;
			motor[backRight] = -power;
			motor[frontLeft] = power;
			motor[backLeft] = power;
		}
		motor[frontRight] = 5;
		motor[backRight] = 5;
		motor[frontLeft] = -5;
		motor[backLeft] = -5;
		wait1Msec(100);
		clearTimer(T3);
		while(SensorValue[gyro] > decidegrees + error || SensorValue[gyro] < decidegrees - error)
		{
			if(SensorValue[gyro] < decidegrees)
			{
				motor[frontRight] = 40;
				motor[backRight] = 40;
				motor[frontLeft] = -40;
				motor[backLeft] = -40;
			}
			else
			{
				motor[frontRight] = -40;
				motor[backRight] = -40;
				motor[frontLeft] = 40;
				motor[backLeft] = 40;
			}
		}

	}
	motor[frontRight] = 0;
	motor[backRight] = 0;
	motor[frontLeft] = 0;
	motor[backLeft] = 0;

}

void drift(int power,int howfar)
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
		clearTimer(T3);
		if(SensorValue[gyro] > 0)
		{
			while(SensorValue[gyro]>10|SensorValue[gyro]<-10)
			{
				motor[frontRight] = 30;
				motor[backRight] = 30;
				motor[frontLeft] = -30;
				motor[backLeft] = -30;
				if(time1[T3]/10>howfar/2){break;}
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
				if(time1[T3]/10>howfar/2){break;}
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
		clearTimer(T3);
		if(SensorValue[gyro] > 0)
		{
			while(SensorValue[gyro]>10|SensorValue[gyro]<-10)
			{
				motor[frontRight] = 30;
				motor[backRight] = 30;
				motor[frontLeft] = -30;
				motor[backLeft] = -30;
				if(time1[T1]/10>howfar/2){break;}
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
				if(time1[T3]/10>howfar/2){break;}
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
	while(howfar-nMotorEncoder[backRight]>5 | howfar-nMotorEncoder[backLeft]>5)
	{
		if(nMotorEncoder[backRight] < howfar ){
			motor[backRight] = power;
			motor[frontRight] = power;}
		else {
			motor[backRight] = 0;
			motor[frontRight] = 0;}

		if(nMotorEncoder[backLeft] < howfar ){
			motor[backLeft] = power;
			motor[frontLeft] = power;}
		else {
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
	nMotorEncoder[backLeft]  = 0;
	while(nMotorEncoder[backRight]+howfar>10 | nMotorEncoder[backLeft]+howfar>10)
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

void autohang()
{
	stopTask(raiseOrLowerArm);
	SensorValue[solShift]=1;
	SensorValue[winchencoder]=0;
	while(SensorValue[winchencoder]==0)
	{
		motor[backLeft]=motor[backRight]=80;
		wait1Msec(200);
		motor[backLeft]=motor[backRight]=-80;
		wait1Msec(200);
	}
	motor[backLeft]=motor[backRight]=0;
	while(SensorValue[armPotentiometerL]<3050)
		motor[armmotL]=motor[armmotR]=128;
	SensorValue[solBig]=1;
	motor[frontLeft]=motor[frontRight]=128;
	wait1Msec(600);
	motor[armmotL]=motor[armmotR]=0;
	motor[frontLeft]=motor[frontRight]=0;
	winchup(2500);

	motor[armmotL]=motor[armmotR]=-128;
	while(SensorValue[armPotentiometerL]>1380)
	{
		motor[backLeft]=motor[backRight]=128;
	}
	wait1Msec(200);
	motor[backLeft]=motor[backRight]=100;
	wait1Msec(200);
	motor[backLeft]=motor[backRight]=80;
	wait1Msec(200);
	motor[backLeft]=motor[backRight]=60;
	wait1Msec(200);
	motor[backLeft]=motor[backRight]=40;
	wait1Msec(200);
	motor[armmotL]=motor[armmotR]=-10;
	SensorValue[solBig]=0;
	motor[backLeft]=motor[backRight]=20;
	wait1Msec(200);
	motor[backLeft]=motor[backRight]=10;
	SensorValue[solBig]=0;
	wait1Msec(1000);
}
void winchup (int howfar)
{

	SensorValue[winchencoder]=0;
	while(SensorValue[winchencoder]<=howfar){motor[backLeft]=motor[backRight]=128;}
	motor[backLeft]=motor[backRight]=0;
}

//-----------------------------------------------------------------------------------

task projectile()
{
	while(1){

		while(vexRT[Btn5UXmtr2] == 0&!fire);
		while(vexRT[Btn5UXmtr2] == 1|fire)         // If button 6U (upper right shoulder button) is pressed:
		{
			if(!hangmode)
				SensorValue[solBig] = 1;
			encoderL=nMotorEncoder[backLeft];
			encoderR=nMotorEncoder[backRight];
			gyroref=SensorValue[gyro];
		}
		gyroref=SensorValue[gyro];
		if(hangmode)
			SensorValue[solBig] = !SensorValue[solBig] ;
		else
			SensorValue[solBig] = 0;
		encoderL=nMotorEncoder[backLeft];
		encoderR=nMotorEncoder[backRight];
		wait1Msec(100);
	}
}
task shift()
{
	while(true){
		while(vexRT[Btn7U] == 0)
		{
			if(vexRT[Btn8R])//hangmode
			{
				while(vexRT[Btn8R]){}
				liftDa=true;
				hangmode=true;
				motor[backLeft]=motor[backRight]=128;
				wait1Msec(100);
				motor[backLeft]=motor[backRight]=-128;
				SensorValue[solShift]=1;
				stopTask(raiseOrLowerArm);
				motor[backLeft]=motor[backRight]=128;
				wait1Msec(100);
				motor[backLeft]=motor[backRight]=-128;
				wait1Msec(100);
				motor[backLeft]=motor[backRight]=128;
				wait1Msec(100);
				motor[backLeft]=motor[backRight]=-128;
				wait1Msec(100);
				motor[backLeft]=motor[backRight]=0;

			}
		}
		SensorValue[solShift] =!SensorValue[solShift];
		while(vexRT[Btn7U] == 1){}

	}
}
task openFork()
{
	bool openupsav;
	while(1)
	{
		if(!openup)//action
		{
			SensorValue[fork1]=0;
			wait1Msec(200);
			SensorValue[fork2]=0;
		}
		else
		{
			SensorValue[fork2]=1;
			wait1Msec(200);
			SensorValue[fork1]=1;
		}
		openupsav=openup;
//		while(openup==openupsav){}


	}
}


task masterreset()
{
	while(1){
		while(vexRT[Btn8U] == 1){}
		while(vexRT[Btn8U] == 0){}
		stopTask(raiseOrLowerArm);
		stopTask(raiseOrLowerFork);
		stopTask(usercontrol);
		stopTask(shift);
		stopTask(projectile);
		stopTask(autoslection);
		stopTask(openFork);
		startTask(raiseOrLowerArm);
		startTask(raiseOrLowerFork);
		startTask(usercontrol);
		startTask(shift);
		startTask(projectile);
		startTask(autoslection);
		startTask(openFork);
		kMaxPos=3070;
		hangmode=false;
		liftDa=false;
		openup=false;
		SensorValue[solBig]=0;
	}
}
//-----------------------------------------------------------------------------------

task raiseOrLowerArm()
{
	const int kSlowSpeedRange = 10;
	const int kMinPos = 1325;

	const int kSlowSpeedPosLowering = kMinPos + kSlowSpeedRange;
	const int kSlowSpeedRaising     = kMaxPos - kSlowSpeedRange;

	const int kMotorSpeedUp   = +127;
	const int kMotorSpeedDown = -127;
	float ITerm;
	int error;
	const float outMax=128,outMin=-128;
	bool bLastUpdateWasButtonPress = false;
	bool bButtonsHaveBeenPressed   = false;

	while(true)
	{
		while(liftDa){}
		if (joystick <= -90)// button is puchsed indicating ARM should be lowered
		{
			if (SensorValue[armPotentiometerL] < kMinPos)
			{
				motor[armmotL] = 0;
				motor[armmotR] = 0;// Arm is fully lowered. Stop the motor
			}
			else if (SensorValue[armPotentiometerL] < kSlowSpeedPosLowering){
				motor[armmotL] = kMotorSpeedDown / 2;
				motor[armmotR] = kMotorSpeedDown / 2;
			}
			else{
				motor[armmotL] = kMotorSpeedDown;
				motor[armmotR] = kMotorSpeedDown;
			}
			nPositionToHold = (SensorValue[armPotentiometerL]+10);
			bLastUpdateWasButtonPress = true;
		}

		else if (joystick >= 90)// button is puchsed indicating ARM should be raised
		{
			if (SensorValue[armPotentiometerL] > kMaxPos)
			{
				motor[armmotL] = 10;
				motor[armmotR] = 10;// Arm is fully raised. Stop the motor
			}
			else if (SensorValue[armPotentiometerL] > kSlowSpeedRaising){
				motor[armmotL] = kMotorSpeedUp / 2;
				motor[armmotR] = kMotorSpeedUp / 2;
			}
			else{
				motor[armmotL] = kMotorSpeedUp;
				motor[armmotR] = kMotorSpeedUp;
			}

			nPositionToHold = (SensorValue[armPotentiometerL]+10);
		}


		else// No buttons are pushed.
		{


			if (bLastUpdateWasButtonPress) // A button has just been released.- Remember the position.-- Stop motor
			{
				bLastUpdateWasButtonPress = false;
				if (nPositionToHold < kMinPos)
					nPositionToHold = kMinPos;
				else if (nPositionToHold > kMaxPos)
					nPositionToHold = kMaxPos;
				motor[armmotL] = 0;
				motor[armmotR] = 0;
				clearTimer(T4);
				bButtonsHaveBeenPressed = true;
			}
			else if (bButtonsHaveBeenPressed)
			{
				int SampleTime=10;
				long lastInput;
				if(time1[T4]>=SampleTime)
				{
					float input = SensorValue[armPotentiometerL];
					error = nPositionToHold - input;

					ITerm+= (ki * error);
					ITerm= 0;
					if(ITerm > outMax) ITerm= outMax;
					else if(ITerm < outMin) ITerm= outMin;
					if(ki==0) ITerm=0;
					float dInput = (input - lastInput);
					/*Compute PID Output*/
					float output = kp * error + ITerm- kd * dInput;
					if(output > outMax) output = outMax;
					else if(output < outMin) output = outMin;
					motor[armmotL]= motor[armmotR]= output;
					lastInput = input;
					clearTimer(T4);
				}
			}
			else
			{
			}
		}
	}
}


//-----------------------------------------------------------------------------------


void toLine(int power)
{
	bool passL=false,passR=false;
	while(!passL & !passR)
	{
		if(SensorValue[LineR] > linethreshold){
			motor[backRight] = power;
			motor[frontRight] = power;}
		if(SensorValue[LineR] < linethreshold){
			passR=true;
			motor[backRight] = 0;
			motor[frontRight] = 0;}

		if(SensorValue[LineL] > linethreshold){
			motor[backLeft] = power;
			motor[frontLeft] = power;}
		if(SensorValue[LineL] < linethreshold){
			passL=true;
			motor[backLeft] = 0;
			motor[frontLeft] = 0;}
	}
	if(SensorValue[LineR] < linethreshold){
		passR=true;
		motor[backRight] = 0;
		motor[frontRight] = 0;}
	motor[frontRight] = -power/3;
	motor[backRight] = -power/3;
	motor[frontLeft] = -power/3;
	motor[backLeft] = -power/3;
	wait1Msec(60);
	motor[frontRight] = 0;
	motor[backRight] = 0;
	motor[frontLeft] = 0;
	motor[backLeft] = 0;


	if(!passL){
		while(!passL){
			if(SensorValue[LineL] > linethreshold){
				motor[backLeft] = 1.5*power;
				motor[frontLeft] = 1.5*power;}
			if(SensorValue[LineL] < linethreshold){
				passL=true;
				motor[backLeft] = 0;
				motor[frontLeft] = 0;
				passR=false;
				while(!passR){
					if(SensorValue[LineR] > linethreshold){
						motor[backRight] = -1.5*power;
						motor[frontRight] = -1.5*power;}
					if(SensorValue[LineR] < linethreshold){
						passR=true;
						motor[backRight] = 0;
						motor[frontRight] = 0;}
				}
				passL=false;
				while(!passL){
					if(SensorValue[LineL] > linethreshold){
						motor[backLeft] = 1.5*power;
						motor[frontLeft] = 1.5*power;}
					if(SensorValue[LineL] < linethreshold){
						passL=true;
						motor[backLeft] = 0;
						motor[frontLeft] = 0;}
				}
				passR=false;
				while(!passR){
					if(SensorValue[LineR] > linethreshold){
						motor[backRight] = -1.5*power;
						motor[frontRight] = -1.5*power;}
					if(SensorValue[LineR] < linethreshold){
						passR=true;
						motor[backRight] = 0;
						motor[frontRight] = 0;}
				}
			}
		}
	}
	if(!passR){
		while(!passR){
			if(SensorValue[LineR] > linethreshold){
				motor[backRight] = 1.5*power;
				motor[frontRight] = 1.5*power;}
			if(SensorValue[LineR] < linethreshold){
				passR=true;
				motor[backRight] = 0;
				motor[frontRight] = 0;
				passL=false;
				while(!passL){
					if(SensorValue[LineL] > linethreshold){
						motor[backLeft] = -1.5*power;
						motor[frontLeft] = -1.5*power;}
					if(SensorValue[LineL] < linethreshold){
						passL=true;
						motor[backLeft] = 0;
						motor[frontLeft] = 0;}
				}
				passR=false;
				while(!passR){
					if(SensorValue[LineR] > linethreshold){
						motor[backRight] = 1.5*power;
						motor[frontRight] = 1.5*power;}
					if(SensorValue[LineR] < linethreshold){
						passR=true;
						motor[backRight] = 0;
						motor[frontRight] = 0;}
				}
				passL=false;
				while(!passL){
					if(SensorValue[LineL] > linethreshold){
						motor[backLeft] = -1.5*power;
						motor[frontLeft] = -1.5*power;}
					if(SensorValue[LineL] < linethreshold){
						passL=true;
						motor[backLeft] = 0;
						motor[frontLeft] = 0;}
				}
			}
		}
	}
}

task raiseOrLowerFork()
{

	const int kMinPosf = 900;
	float error,input;
	const int kSlowSpeedRange = 80;
	const int kSlowSpeedLowering = kMinPosf + kSlowSpeedRange;
	const int kSlowSpeedRaising     = kMaxPosf - kSlowSpeedRange;


	const float outMax=128,outMin=-128;
	int nPositionToHold;
	float ITerm;
	bool bLastUpdateWasButtonPress = false;
	bool bButtonsHaveBeenPressed   = false;
	while(true)
	{
		if (joystickf<-90)// button is puchsed indicating ARM should be lowered
		{
			if (SensorValue[forkPot] <= kMinPosf)
			{
				motor[fork] = 10; // Arm is fully lowered. Stop the motor
			}
			else if (SensorValue[forkPot] <= kSlowSpeedLowering)
				motor[fork] =-kMotorSpeedUp / 2;
			else
				motor[fork] = -kMotorSpeedUp;
			nPositionToHold = SensorValue[forkPot];
			bLastUpdateWasButtonPress = true;
		}

		else if (joystickf>90)// button is puchsed indicating ARM should be raised
		{
			if (SensorValue[forkPot] >= kMaxPosf)
			{
				motor[fork] = -10;// Arm is fully raised. Stop the motor
			}
			else if (SensorValue[forkPot] >= kSlowSpeedRaising)
				motor[fork] = kMotorSpeedUp / 2;
			else
				motor[fork] = kMotorSpeedUp;
			nPositionToHold = SensorValue[forkPot];
			bLastUpdateWasButtonPress = true;
		}

		else  // No buttons are pushed.
		{
			if (bLastUpdateWasButtonPress) // A button has just been released.- Remember the position.-- Stop motor
			{
				bLastUpdateWasButtonPress = false;
				if (nPositionToHold < kMinPosf)
					nPositionToHold = kMinPosf;
				else if (nPositionToHold > kMaxPosf)
					nPositionToHold = kMaxPosf;
				motor[fork] = 0;
				motor[fork] = 0;
				clearTimer(T2);
				bButtonsHaveBeenPressed = true;
			}
			else if (bButtonsHaveBeenPressed)
			{
				unsigned long now = time1[T2];
				long lastInput;
				unsigned long lastTime;
				unsigned long SampleTime=10;

				unsigned long timeChange = (now - lastTime);
				if(timeChange>=SampleTime)
				{
					input = SensorValue[forkPot];
					error = -(nPositionToHold - input);
					ITerm+= (ki * error);
					if(ITerm > outMax) ITerm= outMax;
					else if(ITerm < outMin) ITerm= outMin;
					if(kif==0) ITerm=0;

					float dInput = (input - lastInput);
					/*Compute PID Output*/
					float output = -(kpf * error + ITerm - kdf * dInput);
					if(output > outMax) output = outMax;
					else if(output < outMin) output = outMin;
					motor[fork]= output;
					lastInput = input;
					lastTime = now;
				}
			}
			else
			{
			}

		}
	}
}




void turnthenfollow(int power,int howfar,int approx)
{
	int degturned;
	//	const int b=150;//lengh factor of the robot
	SensorValue[gyro]=0;
	while(!((eyeM&eyeL)|(eyeM&eyeR)|(eyeL&eyeR)))//eyehits line)		//turn left
	{
		motor[backLeft]=motor[frontLeft]=power;
		motor[backRight]=motor[frontRight]=power;
	}
	motor[backLeft]=motor[frontLeft]=0;
	motor[backRight]=motor[frontRight]=0;
	wait1Msec(100);
	forward(power/2,40);
	while(!eyeL&-SensorValue[gyro]>approx*15)//eyehits line)		//turn left
	{
		motor[backLeft]=motor[frontLeft]=-power;
		motor[backRight]=motor[frontRight]=power;
	}
	if(eyeL)
	{
		motor[backLeft]=motor[frontLeft]=15;
		motor[backRight]=motor[frontRight]=-15;
		wait1Msec(100);
		motor[backLeft]=motor[frontLeft]=0;
		motor[backRight]=motor[frontRight]=0;//brief break
		nMotorEncoder[backLeft]=nMotorEncoder[backRight]=0;
		degturned=SensorValue(gyro);//read gyro
	}
	else if(!eyeL&-SensorValue[gyro]<=approx*15)
	{
		while(!eyeL&-SensorValue[gyro]<approx*5)//eyehits line)		//turn left
		{
			motor[backLeft]=motor[frontLeft]=power;
			motor[backRight]=motor[frontRight]=-power;
		}
		if(eyeL)
		{
			motor[backLeft]=motor[frontLeft]=-15;
			motor[backRight]=motor[frontRight]=15;
			wait1Msec(100);
			motor[backLeft]=motor[frontLeft]=0;
			motor[backRight]=motor[frontRight]=0;//brief break
			nMotorEncoder[backLeft]=nMotorEncoder[backRight]=0;
			degturned=SensorValue(gyro);//read gyro
		}
	}
	forwardLine(power,howfar);//calc corrected distance



}
void forwardLine(int power,int howfar)//howfar uses encoder
{
	float ITerm;
	int error;
	int lastInput;
	const float outMax=128,outMin=-128;
	nMotorEncoder[backLeft]=nMotorEncoder[backRight]=0;
	clearTimer(T3);
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=power;
	//	motor[frontLeft]=motor[frontRight]=power;
	while((nMotorEncoder[backLeft]+nMotorEncoder[backRight])/2<howfar)
	{
		//motor[frontLeft]=power-lfc;//don't forget the back wheels
		//	motor[frontRight]=power+lfc;

		motor[frontLeft]=motor[backLeft]=power-lfc;//don't forget the back wheels
		motor[frontRight]=motor[backRight]=power+lfc;
		long SampleTime=100;
		if(!eyeL&eyeM&!eyeR)//on track, clear stuff
		{
			ITerm=0;
			lfc=0;//line follower correction zero
		}
		else if(!(!eyeL&eyeM&!eyeR)&time1[T3]>=SampleTime)//off track, then react. sample time minaly for motor reaction delay
		{
			static float input;
			if(!eyeL&!eyeM&!eyeR&(lastInput==-4|lastInput==-8))//to much too the right
				input = -8;
			if(eyeL&!eyeM&!eyeR)//to much too the right
				input = -4;//give a negative
			if(eyeL&eyeM&!eyeR)
				input = -1;
			if(!eyeL&eyeM&!eyeR)//
				input = 0;//perfect
			if(eyeL&eyeM&eyeR)//
				input = 0;//horizontal line
			if(!eyeL&eyeM&eyeR)
				input = 1;
			if(!eyeL&!eyeM&eyeR)
				input = 4;
			if(!eyeL&!eyeM&!eyeR&(lastInput==4|lastInput==8))
				input = 8;


			error = - input;

			ITerm+= kil * error;//should be the longer time refreshes, the better
			if(ITerm > outMax) ITerm= outMax;
			else if(ITerm < outMin) ITerm= outMin;
			//		float dInput = (input - lastInput)/timeChange;
			/*Compute PID Output*/
			float dInput=0;
			float output = kpl * error + ITerm- kdl * dInput;
			if(output > outMax) output = outMax;
			else if(output < outMin) output = outMin;
			lfc= output;
			lastInput = input;
			clearTimer(T3);
		}
	}
	motor[frontLeft]=motor[frontRight]=motor[backLeft]=motor[backRight]=0;
}

void securedDump()
{
	//int gyromov;
	//	SensorValue[gyro]=0;
	kMaxPos=2900;
	joystick=128;
	while(SensorValue[armPotentiometerL]<2800);
	forward(128,350);
	motor[roller]=-128;
	wait1Msec(2000);
	backward(128,300);
	motor[roller]=0;
	while(SensorValue[armPotentiometerL]>1400){
		joystick=-128;}
	//	gyromov=SensorValue[gyro]/10;
	//turn(40,gyromov);

}



void breakDrive()
{
	int speed;
	nMotorEncoder[backLeft]=0;
	wait1Msec(10);
	int initspeed=nMotorEncoder[backLeft];
	int power=-initspeed*1000;
	speed=initspeed;
	motor[frontRight] =	motor[backRight] =motor[frontLeft] =motor[backLeft] = power;
	clearTimer(T4);
	while(abs(speed)>1)
	{
		speed=nMotorEncoder[backLeft];
		wait1Msec(20);
		nMotorEncoder[backLeft]=0;
		if(((speed<=0) & (initspeed>0))|((speed>=0) & (initspeed<0)))
		{
			motor[frontRight] =	motor[backRight] =motor[frontLeft] =motor[backLeft] = 0;
			break;
		}
		if(time1[T4]>300)
			break;
	}
	motor[frontRight] =	motor[backRight] =motor[frontLeft] =motor[backLeft] = 0;
}


void breakDrivet()
{
	int speedl,speedr;
	nMotorEncoder[backLeft]=0;
	nMotorEncoder[backRight]=0;
	wait1Msec(10);
	int initspeedl=nMotorEncoder[backLeft];
	int initspeedr=nMotorEncoder[backRight];
	int powerl=-initspeedl*1000;
	int powerr=-initspeedr*1000;
	speedr=initspeedr;
	speedl=initspeedl;
	motor[frontRight] =	motor[backRight] = powerr;
	motor[frontLeft] =motor[backLeft] =powerl;
	clearTimer(T4);
	while(abs(speedl)>1|abs(speedr)>1)
	{
		speedl=nMotorEncoder[backLeft];
		speedr=nMotorEncoder[backRight];
		wait1Msec(20);
		nMotorEncoder[backLeft]=0;
		nMotorEncoder[backRight]=0;
		if(((speedl<=0) & (initspeedl>0))|((speedl>=0) & (initspeedl<0)))
		{
			motor[frontLeft] =motor[backLeft] = 0;
		}
		if(((speedr<=0) & (initspeedr>0))|((speedr>=0) & (initspeedr<0)))
		{
			motor[frontRight] =	motor[backRight]=0;
		}
		if(time1[T4]>300)
			break;
	}
	if((speedl<=0&initspeedl>0)|(speedl>=0&initspeedl<0))
		motor[frontLeft] =motor[backLeft] = 0;
	if((speedr<=0&initspeedr>0)|(speedr>=0&initspeedr<0))
		motor[frontRight] =	motor[backRight] =0;
	motor[frontRight] =	motor[backRight] =motor[frontLeft] =motor[backLeft] = 0;
}
void breakDrivesideL()
{

	int speedl;
	nMotorEncoder[backLeft]=0;
	wait1Msec(10);
	int initspeedl=nMotorEncoder[backLeft];
	int powerl=-initspeedl*1000;
	speedl=initspeedl;
	motor[frontLeft] =motor[backLeft] =powerl;
	clearTimer(T4);
	while(abs(speedl)>1)
	{
		speedl=nMotorEncoder[backLeft];
		wait1Msec(20);
		nMotorEncoder[backLeft]=0;
		if(((speedl<=0) & (initspeedl>0))|((speedl>=0) & (initspeedl<0)))
		{
			motor[frontLeft] =motor[backLeft] = 0;
		}
		if(time1[T4]>300)
			break;
	}
	if((speedl<=0&initspeedl>0)|(speedl>=0&initspeedl<0))
		motor[frontLeft] =motor[backLeft] = 0;


}

void breakDrivesideR()
{

	int speedr;
	nMotorEncoder[backRight]=0;
	wait1Msec(10);
	int initspeedr=nMotorEncoder[backRight];
	int powerr=-initspeedr*1000;
	speedr=initspeedr;
	motor[frontRight] =motor[backRight] =powerr;
	clearTimer(T4);
	while(abs(speedr)>1)
	{
		speedr=nMotorEncoder[backRight];
		wait1Msec(20);
		nMotorEncoder[backRight]=0;
		if(((speedr<=0) & (initspeedr>0))|((speedr>=0) & (initspeedr<0)))
		{
			motor[frontRight] =motor[backRight] = 0;
		}
		if(time1[T4]>300)
			break;
	}
	if((speedr<=0&initspeedr>0)|(speedr>=0&initspeedr<0))
		motor[frontRight] =motor[backRight] = 0;
}

task visualMem()//auton only
{
	while(true)
	{
		if(memEn)
		{
			if(eyeL)
				eyeLm=true;
			if(eyeM)
				eyeMm=true;
			if(eyeR)
				eyeRm=true;
			if(eyeB)
				eyeBm=true;
		}
		else
			eyeLm=eyeRm=eyeMm=eyeBm=false;

	}
}

void turnIEC(int power,int degrees)
{
	bool booml=false,boomr=false,leftgood=false,rightgood=false;//break-ed
	int howfar=(int)degrees*6.98;
	if(howfar<=0)
	{
		nMotorEncoder[backRight] = 0;
		nMotorEncoder[backLeft] = 0;
		while(!leftgood | !rightgood)
		{
			if(nMotorEncoder[backRight] < -howfar&(!boomr) ){
				motor[backRight] = power;
				motor[frontRight] = power;}
			else if(nMotorEncoder[backRight] >howfar &(!boomr))
			{
				breakDrivesideR();
				boomr=true;
			}
			else if(boomr)
			{
				motor[backRight] = 	motor[frontRight] = 0;
				rightgood=true;
			}
			if(nMotorEncoder[backLeft] > howfar&(!booml) )
			{
				motor[backLeft] = -power;
				motor[frontLeft] = -power;
			}
			else if(nMotorEncoder[backLeft] < -howfar&(!booml))
			{
				breakDrivesideL();
				booml=true;
			}
			else if(booml)
			{
				motor[backLeft] = 	motor[frontLeft] = 0;
				leftgood=true;
			}
		}
	}
	else if(howfar>=0)
	{
		nMotorEncoder[backRight] = 0;
		nMotorEncoder[backLeft] = 0;
		while(!leftgood | !rightgood)
		{
			if(nMotorEncoder[backLeft] < howfar&(!booml) ){
				motor[backLeft] = power;
				motor[frontLeft] = power;}
			else if(nMotorEncoder[backLeft] >howfar &(!booml))
			{
				breakDrivesideL();
				booml=true;
			}
			else if(booml)
			{
				motor[backLeft] = 	motor[frontLeft] = 0;
				leftgood=true;
			}
			if(nMotorEncoder[backRight] > -howfar&(!boomr) )
			{
				motor[backRight] = -power;
				motor[frontRight] = -power;
			}
			else if(nMotorEncoder[backRight] < -howfar&(!boomr))
			{
				breakDrivesideR();
				boomr=true;
			}
			else if(boomr)
			{
				motor[backRight] = 	motor[frontRight] = 0;
				rightgood=true;
			}

		}
	}

}

void gyrocali()//gyro calibration
{
	SensorValue[gyro]=0;
	displayLCDString(0,0,"turn -90");
	while(nLCDButtons!=kButtonCenter)
	{
		displayLCDPos(0,9);
		displayNextLCDNumber(SensorValue[gyro]);
		wait1Msec(100);
	}
	gyrocorrection=SensorValue[gyro]-900;
	clearLCDLine(0);
	clearLCDLine(1);
	displayLCDString(0,0,"Gyro Good");
	displayLCDPos(0,10);
	displayNextLCDNumber(gyrocorrection,5);
	while(LCD_mid);
	clearLCDLine(0);
	clearLCDLine(1);
}

void encodercali()
{
	bool redo=true;
	while(redo)
	{
		redo=false;
		int samplel1,samplel2;
		int sampler1,sampler2;
		nMotorEncoder[backLeft]=nMotorEncoder[backRight]=0;
		displayLCDCenteredString(0,"testing....");

		wait1Msec(300);
		samplel1=nMotorEncoder[backLeft];
		sampler1=nMotorEncoder[backRight];

		wait1Msec(300);
		samplel2=nMotorEncoder[backLeft];
		sampler2=nMotorEncoder[backRight];

		motor[backLeft]=motor[backRight]=0;
		wait1Msec(50);

		if(samplel1!=0|samplel2!=0)
		{
			displayLCDPos(0,0);
			displayNextLCDString("LL Good");
		}
		else
		{
			displayLCDPos(0,0);
			displayNextLCDString("LL BAD");
			redo=true;
		}

		if(sampler1!=0|sampler2!=0)
		{
			displayLCDPos(0,8);
			displayNextLCDString("RR Good");
		}
		else
		{
			displayLCDPos(0,8);
			displayNextLCDString("RR BAD");
			redo=true;
		}

		while(nLCDButtons==kButtonNone)
		{
			displayLCDPos(1,5);
			displayNextLCDString("redo  skip");
		}
		if(nLCDButtons==kButtonRight)
		{
			redo=false;
			while(nLCDButtons==kButtonRight);
		}

		else if(nLCDButtons==kButtonCenter)
		{
			redo=true;
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDPos(0,0);
			displayNextLCDString("reboot encoders");
			displayLCDPos(1,0);
			displayNextLCDString("......");
//			resetMotorEncoder(backRight);
//			resetMotorEncoder(backLeft);
			SensorType[I2C_1]=sensorNone;
			SensorType[I2C_2]=sensorNone;
			wait1Msec(400);
			SensorType[I2C_1]=sensorQuadEncoderOnI2CPort;
			SensorType[I2C_2]=sensorQuadEncoderOnI2CPort;

		}

	}
}

task tune()
{
	int val=0;
	string value;
	while(true)
	{
		switch (val)
		{
		case 0:
			displayLCDPos(0,0); displayNextLCDString("Kppp=");break;
		case 1:
			displayLCDPos(0,0); displayNextLCDString("Kiii=");break;
		case 2:
			displayLCDPos(0,0); displayNextLCDString("Kddd=");break;
		}

		while(!(SensorValue[GOL]|SensorValue[GOR]))
		{
			SensorValue[prgenc]=0;
			wait1Msec(50);
			switch (val)
			{
			case 0:
				kpf+=(float)(SensorValue[prgenc]/10000.0);
				if(kpf<0)
					kpf=0;
				sprintf(value, "%1.4f%c", kpf);
				break;
			case 1:
				kif+=(float)(SensorValue[prgenc]/10000.0);
				if(kif<0)
					kif=0;
				sprintf(value, "%1.4f%c", kif);
				break;
			case 2:
				kdf+=(float)(SensorValue[prgenc]/10000.0);
				if(kdf<0)
					kdf=0;
				sprintf(value, "%1.4f%c", kdf);
				break;
			}
			displayLCDString(0,5,value);
		}
		while(SensorValue[GOL]){}
		val++;
		if(val>=3)
			val=0;
	}
}
