void blueSkyrise()
{
	drive(12.0); //begin driving forward one foot at max power
  waitForDrive();
  turn(90.0, 100); // begin turning to 90 degrees from the origin at 100 power
  waitForTurn();
  lift(2000); //begin moving lift to a value of "2000" on the potentiometer(s)
  waitForLift();
}

void blueNorise()
{

}

void redSkyrise()
{

}

void redNorise()
{

}
