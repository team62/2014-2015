/*
SetDistance(20inches)
SetAccel(1in/ms/ms, 1in/ms/ms)
SetVelocity(0, 25ms/sec, 0)
*/


void straight(float distance)
{
  // Motion Profile Constants
  MotionProfile trapezoidal1;

  // Initiate the motion profile with a distance of 0
  MotionProfileInit(&trapezoidal1);
  MotionProfileSetAccel(&trapezoidal1, 1, 1); // Accel = 0.5, decel = 1
  MotionProfileSetDistance(&trapezoidal1, distance); // Distance = 1000
  MotionProfileSetVelocity(&trapezoidal1, 0, 25, 0); // V_0 = 0, V_max = 25, V_exit = 0

  float time = 0;
  while(!MotionProfileIsComplete(&trapezoidal1, time))
  {
    float velocity = MotionProfileCompute(&trapezoidal1, time);
    writeDebugStreamLine("%f", velocity);
    time++;
  }
}

float calculateDelta()
{
  float deltaP1 = sensorValue[m_BL];
  wait1Msec(1000);
  float deltaP2 = sensorValue[m_BL];

  float deltaT = deltaP1 - deltaP2;
}

task motionPID()
{
  // The PID controller that will be used
  // The basic usage of this PID controller is as following:
  //
  // PID pid1;
  // PIDInit(&pid1, PConstant, IConstant, DConstant);
  // float feedback = PIDCompute(&pid1, your_error);
  //
  PID motionPID;
  PIDInit(&motionPID, 0.1, 0, 0.1); // Set P, I, and D constants

  // This calculates how far off we are from the true value
  //  The PID will return a response that will hopefully minimize this error over time
  float pidResult = PIDCompute(&motionPID, velocity - calculateDelta());

  // Add pid to motor value
  motorSpeed += pidResult;
}
