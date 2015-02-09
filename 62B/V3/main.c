#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!
#include "BNSLib.h" //The Backend For Most Sensory Algorithm
#include "PID.h" // The PID Backend
#include "auton.h"
#include "menu.h"

void pre_auton()
{
  bStopTasksBetweenModes = true;

  // Initialize Pre Auton Tasks
  startTask(ProgramChooser)
}

task autonomous()
{
	switch(auto)
	{
		case 0: blueSkyrise; break;
		case 1: blueNorise; break;
		case 2: redSkyrise; break;
		case 3: redNorise; break;
	}
}

task usercontrol()
{
	// User control code here, inside the loop

	while (true)
	{

	}
}
