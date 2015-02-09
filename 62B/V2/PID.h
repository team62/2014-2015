// convert from inches to encoder ticks
long inchesToTicks(const float kInches) {
  return (kInches*driveTicksPerRev/kWheelDiameter/PI)/1.41;
}

task liftPID()
{
  // Initialize the BNS Library
  BNS();

  // The PID controller that will be used
  // The basic usage of this PID controller is as following:
  //
  // PID pid1;
  // PIDInit(&pid1, PConstant, IConstant, DConstant);
  // float feedback = PIDCompute(&pid1, your_error);
  //

  PID lift;
  PIDInit(&lift, 0.3, 0.4, 0.2); // Set P, I, and D constants

  // We start at 0 units and want to reach 100 units
  float currentDistance = nMotorEncoder[liftLF];
  float targetDistance = 400;

  // This calculates how far off we are from the true value
  //  The PID will return a response that will hopefully minimize this error over time
  float pidResult = PIDCompute(&lift, targetDistance - currentDistance);

  // Add pid to motor value
  currentDistance += pidResult;
}

task drivePID()
{
  // Initialize the BNS Library
  BNS();

  // The PID controller that will be used
  // The basic usage of this PID controller is as following:
  //
  // PID pid1;
  // PIDInit(&pid1, PConstant, IConstant, DConstant);
  // float feedback = PIDCompute(&pid1, your_error);
  //

  PID driveFL;
  PID driveFR;
  PID driveBL;
  PID driveBR;
  PIDInit(&driveFL, 0.3, 0.4, 0.2); // Set P, I, and D constants
  PIDInit(&driveFR, 0.3, 0.4, 0.2); // Set P, I, and D constants
  PIDInit(&driveBL, 0.3, 0.4, 0.2); // Set P, I, and D constants
  PIDInit(&driveBR, 0.3, 0.4, 0.2); // Set P, I, and D constants

  // We start at 0 units and want to reach 100 units
  float currentDistanceFL = nMotorEncoder[m_FL];
  float targetDistanceFL = inchesToTicks(distance);

  float currentDistanceFR = nMotorEncoder[m_FR];
  float targetDistanceFR = inchesToTicks(distance);

  float currentDistanceBL = nMotorEncoder[m_BL];
  float targetDistanceBL = inchesToTicks(distance);

  float currentDistanceBR = nMotorEncoder[m_BR];
  float targetDistanceBR = inchesToTicks(distance);

  // This calculates how far off we are from the true value
  //  The PID will return a response that will hopefully minimize this error over time
  float pidResultFL = PIDCompute(&driveFL, targetDistanceFL - currentDistanceFL;
  float pidResultFR = PIDCompute(&driveFR, targetDistanceFR - currentDistanceFR);
  float pidResultBL = PIDCompute(&driveBL, targetDistanceBL - currentDistanceBL);
  float pidResultBR = PIDCompute(&driveBR, targetDistanceBR - currentDistanceBR);

  // Add pid to motor value
  currentDistanceFL += pidResultFL;
  currentDistanceFR += pidResultFR;
  currentDistanceBL += pidResultBL;
  currentDistanceBR += pidResultBR;
}

task gyroPIDKal()
{
  BNS();

  PID gyro;
  PIDInit(&gyro, 0.2, 0.9, 1.0);

  float currentPos = SensorValue[s_gyro];
  float targetPos = 900;

  float pidResultPos = PIDCompute(&gyro, targetPos - currentPos);

  currentPos += pidResultPos;

  float dt = 0.1;
  Matrix updateMatrix;
  CreateZerosMatrix(updateMatrix, 4, 4);
  SetMatrixAt(updateMatrix, 0, 0, 1);
  SetMatrixAt(updateMatrix, 0, 1, dt);
  SetMatrixAt(updateMatrix, 1, 0, 0);
  SetMatrixAt(updateMatrix, 1, 1, 1);
  SetMatrixAt(updateMatrix, 2, 2, 1);
  SetMatrixAt(updateMatrix, 2, 3, dt);
  SetMatrixAt(updateMatrix, 3, 2, 0);
  SetMatrixAt(updateMatrix, 3, 3, 1);

  // Extraction Matrix
  // This matrix relates our input to output
  // It tells us what we'd expect if we got PERFECT
  //   sensor data
  // So we'd expect position, x1 & x3, to be exactly what
  //   our sensors tell us
  Matrix extractionMatrix;
  CreateMatrix(extractionMatrix,
            "1 0 0 0;
             0 0 1 0");

  // Covariance of our output
  // This simply tell us how "sure" we are of our estimations
  // The main digonal tells us for each x1, x2, x3, x4 how good
  //   each estimation is.  If they are 0, we have perfect results,
  //   if we have high numbers, like 1000, we have very unsure results
  // Note: This will naturally converge over time, so it's a good idea to
  //   keep the variance very high at the start
  Matrix covarianceMatrixX;
  CreateMatrix(covarianceMatrixX,
            "1000 0 0 0;
            0 1000 0 0;
            0 0 1000 0;
            0 0 0 1000");

  // Covariance of our sensor data
  // How "good" is our sensor information
  // This does not converge over time, so it's important you tune to a fair number
  Matrix covarianceMatrixZ;
  CreateMatrix(covarianceMatrixZ, "1 0; 0 1");

  // Move Vector
  // The control input we know is happening.  Typically left at 0
  Matrix moveVector;
  CreateZerosMatrix(moveVector, 4, 1);

  // Our data we will update into the filter :)
  Matrix data;
  CreateZerosMatrix(data, 2, 1);
  SetMatrixAt(data, 0, 0, 1);
  SetMatrixAt(data, 1, 0, 2);

  // Create a new kalman filter and input our matricies
  KalmanFilter filter;
  KalmanInit(filter, 4, 2,
                    updateMatrix,
                    extractionMatrix,
                    covarianceMatrixX,
                    covarianceMatrixZ,
                    moveVector
                  );

  // This is where we will input our "data" into the filter
  // For this example, we are giving "i" and "i*2" as an input position
  // So we expect our position to be "20" and "40", with the velocity to be
  //   1/dt = "10" and 2/dt = "20"
  // Remember i = position, velocity is how much we're incrementing our i each loop!
  int endPosition = 20;
  int velocity = 1;
  for(int i = 1; i <= endPosition; i+=velocity)
  {
    // "Fake" sensor noise, between -0.5 and 0.5
    float noise = (rand() % 1000)/1000.0 - 0.5;

    // Update Data Matrix
    SetMatrixAt(data, 0, 0, i + noise);
    SetMatrixAt(data, 1, 0, i*2 + noise);

    // Predict and Update Kalman Filter
    KalmanPredict(filter);
    KalmanUpdate(filter, data);
  }

  // Output estimation 1
  writeDebugStreamLine("Pos1 = %f, Vel1 = %f",
  GetMatrixAt(filter.meanVector, 0, 0),
  GetMatrixAt(filter.meanVector, 1, 0));

  // Output estimation 2
  writeDebugStreamLine("Pos2 = %f, Vel2 = %f",
  GetMatrixAt(filter.meanVector, 2, 0),
  GetMatrixAt(filter.meanVector, 3, 0));
}
