task kalman()
{
  // Create a Simple KalmanFilter with an update every 0.1 seconds
  KalmanFilter filter1;
  SimpleKalmanInit(&filter1, 0.1);

  float gyroOutFixed = SimpleKalmanUpdate(&filter1, SensorValue[s_gyro]);
}
