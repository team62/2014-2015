bool doUseGyro = true; // enable gyroscopic sensor
float gyroOffset = 90.0; // degrees CCW from +x axis robot faces at gyro=0

task userDriveHolo()
{
  const TVexJoysticks kChY = Ch3; // y-axis joystick channel
  const TVexJoysticks kChX = Ch4; // x-axis joystick channel
  const TVexJoysticks kChR = Ch1; // rotation joystick channel
  const short kDelay = 25; // milliseconds for loop delay
  const ubyte kNumWheels = 4; // number of drive wheels
  const ubyte kNumMotorsPerWheel = 2; // max number of motors per wheel
  const tMotor kNone = -1; // used for indicating the lack of an additional motor
  const tMotor kMotorPort[kNumWheels][kNumMotorsPerWheel] = { // drive motor ports/names
    {m_FL, kNone}, // front-left
    {m_FR, kNone}, // front-right
    {m_BL, kNone}, // back-left
    {m_BR, kNone}  // back-right
  };

  word x,y,r;
  float gyro,radius,theta,a,b,wheelSpeed[kNumWheels],topSpeed;

  while(true)
  {
    // ==== collect joystick & sensor values ====
    x = vexRT[kChX]; // x component
    y = vexRT[kChY]; // y component
    r = vexRT[kChR]; // rotation
    gyro = gyroOffset + (doUseGyro ? SensorValue[kGyroPort]/10.0 : 0.0); // if using gyro, scale its value to degrees

    // ==== convert joystick values to polar ====
    radius = sqrt(pow(x,2) + pow(y,2)); // r = sqrt(x^2 + y^2)
    theta = atan2(y,x)*180.0/PI; // t = arctan(y/x) [converted from radians to degrees]

    theta -= gyro; // adjust for gyro angle

    // ==== calculate opposite-side speeds ====
    a = (cosDegrees(theta + 90.0) + sinDegrees(theta + 90.0))*radius; // front-left and back-right
    b = (cosDegrees(theta) + sinDegrees(theta))*radius; // front-right and back-left

    // ==== set speeds, including rotation ====
    wheelSpeed[0] = a + r; // front-left
    wheelSpeed[1] = b - r; // front-right
    wheelSpeed[2] = b + r; // back-left
    wheelSpeed[3] = a - r; // back-right

    // ==== normalize speeds ====
    topSpeed = 0.0;
    for(ubyte i=0; i<kNumWheels; i++)
      if(abs(wheelSpeed[i]) > topSpeed)
        topSpeed = abs(wheelSpeed[i]); // find highest desired speed
    if(topSpeed > 127.0)
      for(ubyte i=0; i<kNumWheels; i++)
        wheelSpeed[i] /= topSpeed/127.0; // downscale all speeds so none are above 127

    // ==== update motor powers ====
    for(ubyte i=0; i<kNumWheels; i++) // cycle through all wheels
      for(ubyte j=0; j<kNumMotorsPerWheel; j++) // cycle through all motors for each wheel
        if(kMotorPort[i][j] != kNone) // check existence of motor
          motor[kMotorPort[i][j]] = (word)wheelSpeed[i]; // update motor power

    wait1Msec(kDelay);
  }
}

task driverControl()
{

}
