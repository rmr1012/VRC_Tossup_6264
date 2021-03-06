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
#pragma config(Motor,  port7,           armmotR,       tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           backLeft,      tmotorVex393_MC29, encoderPort, I2C_2)
#pragma config(Motor,  port9,           frontLeft,     tmotorVex393_MC29, openLoop, reversed)


#define eyeL (SensorValue[LineL]<threshold)
#define eyeM (SensorValue[LineM]<threshold)
#define eyeR (SensorValue[LineR]<threshold)
#define eyeB (SensorValue[LineB]<threshold)

#include "Vex_Competition_Includes.c"
int kMaxPos=3100;
int threshold=300, lfc=0;
float kil=0.00,kdl=0,kpl=15;
bool eyeLm=false,eyeMm=false,eyeRm=false,eyeBm=false,memEn=false;//eye memory section;

void forward(int power,int howfar);
void breakDrive();
void breakDrivet();


void pre_auton()
{
	bLCDBacklight=true;
}
task visualMem()//auton only
{
	while(true)
	{
		while(!memEn)
			eyeLm=eyeRm=eyeMm=eyeBm=false;

		while(eyeL|eyeM|eyeR|eyeB)
		{
			if(eyeL)
				eyeLm=true;
			if(eyeM)
				eyeLm=true;
			if(eyeR)
				eyeLm=true;
			if(eyeB)
				eyeLm=true;
		}
	}
}
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
task autonomous()
{
	startTask(visualMem);
	while(!(eyeM|eyeR|eyeL))//eyehits line)		//
	{
		motor[backLeft]=motor[frontLeft]=55;
		motor[backRight]=motor[frontRight]=55;
	}
	forward(60,70);

	SensorValue[gyro]=0;
	memEn=true;

	while((SensorValue[gyro]>-900)&!((eyeL|eyeM|eyeR|eyeLm|eyeMm|eyeRm)&eyeB))
	{
		motor[frontRight] =	motor[backRight] = -128;
		motor[frontLeft] =motor[backLeft] =128;
	}
	breakDrivet();
	if((eyeL|eyeM|eyeR)&eyeB)//if perfect
	{

	}
	else
	{
		if((eyeLm|eyeMm|eyeRm)&(!eyeBm))//sinario 1  fron crossed, back not
		{
			memEn=false;
			motor[backRight]=motor[frontLeft]=-128;
			motor[backLeft]=motor[frontRight]=128;
			while(!eyeL&!eyeM&!eyeR&!eyeB);//drift left until anyhits
				motor[backRight]=motor[frontLeft]=-0;
			motor[backLeft]=motor[frontRight]=0;
			if(eyeB&(eyeL|eyeM|eyeR)){}
			else if(eyeB)//if back hit first. 1A
			{
				motor[frontLeft]=-128;
				motor[frontRight]=128;//turn
				while(!(eyeL|eyeM|eyeR));
				motor[frontLeft]=0;
				motor[frontRight]=0;//stop

			}
			else if(eyeL|eyeM|eyeR)//if front hit first. 1B
			{
				motor[backLeft]=80;
				motor[backRight]=-80;//turn
				while(!eyeB);
				motor[frontLeft]=0;
				motor[frontRight]=0;//stop
				while(!(eyeL|eyeM|eyeR))
				{
					motor[backRight]=motor[frontLeft]=-80;
					motor[backLeft]=motor[frontRight]=80;
				}
				motor[backRight]=motor[frontLeft]=0;
				motor[backLeft]=motor[frontRight]=0;
			}
		}
		else if(!eyeLm&!eyeMm&!eyeRm&!eyeBm) //sinario 2 none not cross
		{

			while(!eyeL&!eyeM&!eyeR)//front eyehits line)		//turn right
			{
				motor[backLeft]=motor[frontLeft]=80;
				motor[backRight]=motor[frontRight]=-80;
			}

			if(eyeB&(eyeL|eyeM|eyeR)){}
			else if(eyeBm)//back passed 2A
			{
				while(!eyeB)//until it's good, drift and turn ccw
				{
					motor[frontLeft]=-80;
					motor[frontRight]=+80;//turncw
					while(!eyeB);
					motor[frontLeft]=0;
					motor[frontRight]=0;//stop
				}

			}
			else if(!eyeBm)//back not pass 2B
			{
				while(!eyeB)//until it's good, drift and turn cw
				{
					motor[frontLeft]=80;
					motor[frontRight]=-80;//turncw
					while(!eyeB);
					motor[frontLeft]=0;
					motor[frontRight]=0;//stop
				}
			}
			memEn=false;
		}
		else if(eyeBm&!eyeLm&!eyeMm&!eyeRm) //sinario 3 back crossed, not front
		{
			while(!eyeL&!eyeM&!eyeR)//front eyehits line)		//turn right
			{
				motor[backLeft]=motor[frontLeft]=80;
				motor[backRight]=motor[frontRight]=-80;
			}

			if(eyeB&(eyeL|eyeM|eyeR)){}
			else if(eyeBm)//back passed 2A
			{
				while(!eyeB)//until it's good, drift and turn ccw
				{
					motor[frontLeft]=-80;
					motor[frontRight]=+80;//turncw
					while(!eyeB);
					motor[frontLeft]=0;
					motor[frontRight]=0;//stop
				}

			}
			else if(!eyeBm)//back not pass 2B
			{
				while(!eyeB)//until it's good, drift and turn cw
				{
					motor[frontLeft]=80;
					motor[frontRight]=-80;//turncw
					while(!eyeB);
					motor[frontLeft]=0;
					motor[frontRight]=0;//stop
				}
			}
			memEn=false;

		}
		else if(!eyeBm&(eyeL|eyeM|eyeR)) //sinario 4 front on. back have passed
		{


		}
	}

}
task usercontrol(){bLCDBacklight=true;
	while(true)
	{
		if(vexRT[Btn8D]){
			motor[frontRight] =	motor[backRight] =motor[frontLeft] =motor[backLeft] = -128;
			while(vexRT[Btn8D]){}
			breakDrive();
		}
		else if(vexRT[Btn8U])
		{
			motor[frontRight] =	motor[backRight] =motor[frontLeft] =motor[backLeft] = 128;
			while(vexRT[Btn8U]){}
			breakDrive();
		}
		if(vexRT[Btn8L]){
			motor[backRight] = -128;
			motor[backLeft] =128
			while(vexRT[Btn8L]){}
			breakDrivet();
		}
		else if(vexRT[Btn8R])
		{
			motor[backRight] = 128;
			motor[backLeft] =-128;
			while(vexRT[Btn8R]){}
			breakDrivet();
		}
	}
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

void movement(int powerl,int powerr,int distancel,int distancer,bool breakm)
{
	slaveMotor(frontLeft, backLeft);
	slaveMotor(frontRight, backRight);

	moveMotorTarget(backRight, distancer, powerr, breakm);
	moveMotorTarget(backLeft, distancel, powerl, breakm);

	while(!getMotorTargetCompleted(backLeft)&!getMotorTargetCompleted(backRight) )
	{
		////Do nothing (Idle Loop)
	}
	resetMotorEncoder(frontLeft);
	resetMotorEncoder(frontRight);
	resetMotorEncoder(backLeft);
	resetMotorEncoder(backRight);
}


/*void leftMotorPID()
{
int maxSpeedTick;//tick/refresh
int refreshTime;
nMotorEncoder[leftMotor]=0

while(true)
{


if(time1[T4]>refreshTime)
{
speed=nMotorEncoder[leftMotor]; //speed is p
error=Pleft-speed;
dIn=speed-lastinput;
iIn+=error*ki

float output = -(kpf * error + ITerm - kdf * dInput);

motor[backLeft]=motor[frontLeft]=output;


lastinput=speed;
nMotorEncoder[leftMotor]=0
}




}


}*/
//hit line
