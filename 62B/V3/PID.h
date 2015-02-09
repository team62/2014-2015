#include <FirmwareVersion.h> // the "TMotorType" values below have changed a bit between versions of RobotC

#if kRobotCVersionNumeric >= 400
#define tmotorVex269 9989
#define tmotorVex393 9990
#define tmotorVex393HighSpeed 9991
#else
#define tmotorVex269_HBridge 9992
#define tmotorVex269_MC29 9993
#define tmotorVex393_HBridge 9994
#define tmotorVex393_MC29 9995
#define tmotorVex393HighSpeed_HBridge 9996
#define tmotorVex393HighSpeed_MC29 9997
#endif
#if kRobotCVersionNumeric < 426
#define tmotorVex393TurboSpeed_HBridge 9998
#define tmotorVex393TurboSpeed_MC29 9999
#endif

const tMotor kMotorPort[kNumbOfTotalMotors] = {port1, port2, port3, port4, port5, port6, port7, port8, port9, port10};
const ubyte kDefaultLoopDelay = 25; // milliseconds

////////////////////////////////////////////////
// edit these constants to your robot's specs //
////////////////////////////////////////////////
// drive
const tSensors kGyroPort = in1;
const ubyte kNumDriveEncoders = 2;
const tSensors kDriveEncoderPort[kNumDriveEncoders] = {port1, port2};
const ubyte kNumDriveMotors = 4;
const tMotor kDriveMotorPort[kNumDriveMotors] = {port1, port2, port3, port4};
const float kWheelDiameter = 4.0; // inches
const bool kDriveIME = true; // set to false if using quadrature encoders
// lift
const ubyte kNumLiftPots = 2;
const tSensors kLiftPotPort[kNumLiftPots] = {in2, in3}; // {left potentiometer, right potentiometer}
const ubyte kNumLiftMotors = 4;
const tMotor kLiftMotorPort[kNumLiftMotors] = {port5, port6, port7, port8};
const ubyte kNumLiftValues = 2;
const word kLiftValues[kNumLiftValues] = {500, 3500}; // sort in order from low to high
typedef enum liftHeight {kDownLim, kUpLim}; // sort in order from low to high

float driveTicksPerRev = 0.0;
bool driveDone = false;
bool turnDone = false;
bool liftDone = false;
word maxDriveSpeed = 127;
word maxLiftSpeed = 127;
word turnSpeed = 0;
word syncSpeed = 0;
long motorEncoder[kNumbOfTotalMotors];
long setDistance = 0;
float setGyro = 0.0;
word setLift = SensorValue[kLiftPotPort[0]];

// convert from inches to encoder ticks
long inchesToTicks(const float kInches) {
  return kInches*driveTicksPerRev/kWheelDiameter/PI;
}

// used to limit the input to the given bounds (max limit is ± if no min limit is given)
float limit(const float kInput, const float kMaxLimit, float minLimit = 2097200.0) {
  if(kInput > kMaxLimit)
    return kMaxLimit;
  if(minLimit == 2097200.0)
    minLimit = -kMaxLimit;
  if(kInput < minLimit)
    return minLimit;
  return kInput;
}

void setDriveTicksPerRev() {
  const TMotorTypes kMotorType = (TMotorTypes)motorType[kDriveEncoderPort[0]]; // save drive encoder motors' type

  // determine number of ticks per revolution of drive wheels
  if(kDriveIME) {
	  if(kMotorType == tmotorVex269 || kMotorType == tmotorVex269_HBridge || kMotorType == tmotorVex269_MC29)
	    driveTicksPerRev = 240.448; // 269
	  else if(kMotorType == tmotorVex393 || kMotorType == tmotorVex393_HBridge || kMotorType == tmotorVex393_MC29)
	    driveTicksPerRev = 627.2; // 393
	  else if(kMotorType == tmotorVex393HighSpeed || kMotorType == tmotorVex393HighSpeed_HBridge || kMotorType == tmotorVex393HighSpeed_MC29)
	    driveTicksPerRev = 392.0; // 393 high speed
	  else if(kMotorType == tmotorVex393TurboSpeed_HBridge || kMotorType == tmotorVex393TurboSpeed_MC29)
	    driveTicksPerRev = 261.333; // 393 turbo speed
	}
	else
	  driveTicksPerRev = 360.0; // quadrature encoders
}

void calibrateGyro() {
  // completely clear out any previous sensor readings by setting the port to "sensorNone"
  SensorType[kGyroPort] = sensorNone;
  wait1Msec(1000);
  SensorType[kGyroPort] = sensorGyro; // reconfigure as sensorGyro
  wait1Msec(2000); // wait for calibration: ROBOT MUST STAY STILL

  SensorScale[kGyroPort] = 138; // adjust SensorScale to correct the scaling for your gyro
  SensorFullCount[kGyroPort] = 3599; // fix rollover to be "...3598, 3599, 0, 1..."
}

void resetSensors() {
  for(ubyte i=0; i<kNumbOfTotalMotors; i++)
    nMotorEncoder[kMotorPort[i]] = 0;
  SensorValue[kGyroPort] = 0;
}

void drive(const float kInches, const word kSpeed = 127) {
  const long kDistance = inchesToTicks(kInches);

  maxDriveSpeed = abs(kSpeed);
  setDistance += kDistance < 0 ? kDistance : sgn(kSpeed)*kDistance; // if kInches and/or kSpeed is negative, assume reverse
}

void turn(const float kDegrees, const word kSpeed = 127) {
  const word kAngle = kDegrees*10; // convert from degrees to 1/10 degrees

  maxDriveSpeed = abs(kSpeed);
  setGyro += kAngle < 0 ? kAngle : sgn(kSpeed)*kAngle; // if kInches and/or kSpeed is negative, assume left
}

void lift(const word kHeight, const word kSpeed = 127) {
  maxLiftSpeed = abs(kSpeed);
  setLift = kHeight;
}

void waitForDrive() {
  driveDone = false;

  while(!driveDone)
    wait1Msec(kDefaultLoopDelay);
}

void waitForTurn() {
  turnDone = false;

  while(!turnDone)
    wait1Msec(kDefaultLoopDelay);
}

void waitForLift() {
  liftDone = false;

  while(!liftDone)
    wait1Msec(kDefaultLoopDelay);
}

// stores encoder values in an array, allowing values >32,767, up to 2,147,483,647
task handleEncoders() {
  signed int prevValue[kNumbOfTotalMotors];
  signed int encoderValue = 0;

  for(ubyte i=0; i<kNumbOfTotalMotors; i++) {
    motorEncoder[i] = 0;
    prevValue[i] = 0;
  }
  while(true) {
    for(ubyte i=0; i<kNumbOfTotalMotors; i++) {
      encoderValue = nMotorEncoder[kMotorPort[i]];
      if(abs(encoderValue) > 32000) {
        nMotorEncoder[kMotorPort[i]] = 0;
        prevValue[i] -= encoderValue;
        encoderValue = nMotorEncoder[kMotorPort[i]];
      }
      motorEncoder[i] += encoderValue - prevValue[i];
      prevValue[i] = encoderValue;
    }
    wait1Msec(kDefaultLoopDelay);
  }
}

task gyroPID() {
  const float kP = 0.1;
  const float kI = 0.02;
  const float kD = 0.05;
  const float kL = 2.0;
  const float kDoneThreshold = 1.0; // degrees
  const short kSettleTime = 250; // milliseconds

  word error = 0;
  word prevError = 0;
  word p = 0;
  word i = 0;
  word d = 0;

  while(true) {
    error = setGyro - SensorValue[kGyroPort];

    p = error;
    i = limit(i + error, kL);
    d = error - prevError;

    turnSpeed = p*kP + i*kI + d*kD;

    prevError = error;

    if(abs(error) <= kDoneThreshold*10)
      if(time1[T2] >= kSettleTime) // if close to setGyro for enough time
        turnDone = true;
    else
      clearTimer(T2);

    wait1Msec(kDefaultLoopDelay);
  }
}

task drivePID() {
  const float kP = 1.0;
  const float kI = 0.5;
  const float kD = 0.5;
  const float kL = 50.0;
  const float kDoneThreshold = 0.5; // inches
  const short kSettleTime = 250; // milliseconds

  long error = 0;
  long prevError = 0;
  float avgEncoder = 0.0;
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;
  word driveSpeed = 0;
  word speedL = 0;
  word speedR = 0;

  while(true) {
    avgEncoder = motorEncoder[kDriveEncoderPort[0]];
    for(ubyte j=1; j<kNumDriveEncoders; j++)
      avgEncoder = (avgEncoder + motorEncoder[kDriveEncoderPort[j]])/2;
    error = setDistance - avgEncoder;

    p = error;
    i = limit(i + error, kL);
    d = error - prevError;

    driveSpeed = p*kP + i*kI + d*kD;

    // normalize so that turnSpeed can always be applied in full
    speedR = limit(driveSpeed + turnSpeed/2, maxDriveSpeed); // in case (driveSpeed + turnSpeed/2) > maxSpeed
    speedL = limit(speedR - turnSpeed, maxDriveSpeed); // in case (driveSpeed - turnSpeed/2) < -maxSpeed
    speedR = limit(speedL + turnSpeed, maxDriveSpeed);

    for(ubyte j=0; j<kNumDriveMotors; j++)
      motor[kDriveMotorPort[j]] = nMotorDriveSide[kDriveMotorPort[j]] == driveLeft ? speedL : speedR;

    prevError = error;

    if(abs(error) <= inchesToTicks(kDoneThreshold))
      if(time1[T1] >= kSettleTime) // if close to setDrive for enough time
        driveDone = true;
    else
      clearTimer(T1);

    wait1Msec(kDefaultLoopDelay);
  }
}

task syncPID() {
  const float kP = 0.5;
  const float kI = 0.1;
  const float kD = 0.2;
  const float kL = 50.0;
  const word kPotDiff = 0; // diff = left - right

  word error = 0;
  word prevError = 0;
  word p = 0;
  word i = 0;
  word d = 0;

  while(true) {
    error = SensorValue[kLiftPotPort[0]] - SensorValue[kLiftPotPort[1]] - kPotDiff;

    p = error;
    i = limit(i + error, kL);
    d = error - prevError;
    syncSpeed = p*kP + i*kI + d*kD;

    prevError = error;

    wait1Msec(kDefaultLoopDelay);
  }
}

task liftPID() {
  const TVexJoysticks kJoyCh = Ch2;
  const float kP = 0.1;
  const float kI = 0.02;
  const float kD = 0.04;
  const float kL = 10.0;
  const word kJoyThresh = 10;
  const word kHoldPower = 10;
  const word kDoneThreshold = 50; // ticks
  const word kSettleTime = 250; // milliseconds

  float avgPotentiometer = 0.0;
  float error = 0.0;
  float prevError = 0.0;
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;
  word joyValue = 0;
  word liftSpeed = 0;
  word speedL = 0;
  word speedR = 0;
  byte holdDir = 1;

  while(true) {
    joyValue = vexRT[kJoyCh];

    avgPotentiometer = SensorValue[kLiftPotPort[0]];
    for(ubyte j=1; j<kNumLiftPots; j++)
      avgPotentiometer = (avgPotentiometer + SensorValue[kLiftPotPort[j]])/2;

    if(!bIfiAutonomousMode && (joyValue >= kJoyThresh && avgPotentiometer <= kLiftValues[kUpLim] || joyValue <= -kJoyThresh && avgPotentiometer >= kLiftValues[kDownLim])) {
      setLift = avgPotentiometer;
      liftSpeed = joyValue + (127 - abs(joyValue))/(127 - kJoyThresh)*kHoldPower;
    }
    else {
      if(avgPotentiometer < kLiftValues[kDownLim] + 100) {
        holdDir = -1;
        setLift = kLiftValues[kDownLim];
      }
      else
        holdDir = 1;

	    error = setLift - avgPotentiometer;

	    p = error;
	    i = limit(i + error, kL);
	    d = error - prevError;
	    liftSpeed = p*kP + i*kI + d*kD + holdDir*kHoldPower;

	    prevError = error;

	    if(abs(error) <= kDoneThreshold)
	      if(time1[T3] >= kSettleTime) // if close to setLift for enough time
	        liftDone = true;
	    else
	      clearTimer(T3);
	  }
	  // normalize so that turnSpeed can always be applied in full
	  speedR = limit(liftSpeed + syncSpeed/2, maxLiftSpeed); // in case (liftSpeed + syncSpeed/2) > maxSpeed
    speedL = limit(speedR - syncSpeed, maxLiftSpeed); // in case (liftSpeed - syncSpeed/2) < -maxSpeed
    speedR = limit(speedL + syncSpeed, maxLiftSpeed);

    // set lift motors' powers based on lift side
    for(ubyte j=0; j<kNumLiftMotors; j++)
      motor[kLiftMotorPort[j]] = nMotorDriveSide[kLiftMotorPort[j]] == driveLeft ? speedL : speedR;

    wait1Msec(kDefaultLoopDelay);
  }
}

task main() {
  // void pre_auton() {
  setDriveTicksPerRev();
  calibrateGyro();
  // }

  // task autonomous() {
  resetSensors();
  startTask(handleEncoders);
  startTask(gyroPID);
  startTask(drivePID);
  startTask(syncPID);
  startTask(liftPID);

  drive(12.0); //begin driving forward one foot at max power
  waitForDrive();
  turn(90.0, 100); // begin turning to 90 degrees from the origin at 100 power
  waitForTurn();
  lift(2000); //begin moving lift to a value of "2000" on the potentiometer(s)
  waitForLift();
  // }

  // task usercontrol() {
  stopTask(handleEncoders);
  stopTask(gyroPID);
  stopTask(drivePID);
  startTask(syncPID);
  startTask(liftPID);
  // }
}
