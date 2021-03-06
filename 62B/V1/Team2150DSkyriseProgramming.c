#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    s_gyro,         sensorGyro)
#pragma config(Sensor, dgtl1,  p_claw,         sensorDigitalOut)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_4,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_5,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_6,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_7,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_8,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port2,           m_FL,          tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_4)
#pragma config(Motor,  port3,           m_FR,          tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port4,           m_BL,          tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_3)
#pragma config(Motor,  port5,           m_BR,          tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port6,           m_LB,          tmotorVex393_MC29, openLoop, reversed, driveLeft, encoderPort, I2C_5)
#pragma config(Motor,  port7,           m_LT,          tmotorVex393_MC29, openLoop, driveLeft, encoderPort, I2C_6)
#pragma config(Motor,  port8,           m_RT,          tmotorVex393_MC29, openLoop, reversed, driveRight, encoderPort, I2C_7)
#pragma config(Motor,  port9,           m_RB,          tmotorVex393_MC29, openLoop, reversed, driveRight, encoderPort, I2C_8)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!
#include "BNSLib.h" //BNS Lib
#include "PID.h" // PID Controllers
#include "FC.h" // Field Centric Driver Control
#include "menu.h" // Menu Controller
#include "auton.h" // Auton Storage

void pre_auton()
{
	// This Stops Tasks In Between Modes!
  bStopTasksBetweenModes = true;

  setDriveTicksPerRev();
  calibrateGyro();

  startTask(ProgramChooser);

  BNS();
}

task autonomous()
{
	resetSensors();
  startTask(handleEncoders);
  startTask(gyroPID);
  startTask(drivePID);
  startTask(syncPID);
  startTask(liftPID);
	stopTask(ProgramChooser);
	switch(auto)
	{
		case 0: blueSkyrise(); break;
		case 1: redSkyrise(); break;
		case 2: blueNorise(); break;
		case 3: redNorise(); break;
	}
}

task usercontrol()
{
	while(true)
	{
		stopTask(handleEncoders);
	  stopTask(gyroPID);
	  stopTask(drivePID);
	  startTask(ProgramChooser);
	  startTask(syncPID);
	  startTask(liftPID);
	}
}
