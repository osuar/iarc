initialize() etc...




loop() {


	currentTime = micros();
	deltaTime = currentTime - previousTime;
	 

	if (deltaTime >= DesiredLoopSpeed){


	gyro.measure();		//update values.
	accel.measure();	//update values.


	//get new flight angle using new values.
	flightAngle->calculate(          gyro.getData(ROLL), //getData is a float  \
		               		 gyro.getData(PITCH),                      \
		     		         gyro.getData(YAW),                        \
		                         accel.getData(XAXIS),                     \
		                         accel.getData(YAXIS),                     \
		                         accel.getData(ZAXIS),                     \
		                         0.0,                                      \
		                         0.0,                                      \
		                         0.0);

	proccessFlightControl();	//combines joystick commands with sensor data.

}

//This is for the gyro, measure for accell looks similar. Reads data and then smooths and scales.
 void measure(void) {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      if (axis == PITCH)
        gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
      else
        gyroADC[axis] = gyroZero[axis] - analogRead(gyroChannel[axis]);
      gyroData[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor);
    }

//calls more math functions.
calculate(blah){

	matrixUpdate(blah);
	normalize();
	driftCorrection(blah);
	eulerAngles();
	earthAxisAccels(blah);
}

proccessFlightControl(){
	calculateFlightError();
	processHeading();
	processAltitudeHold();
	//then calculates motor commands.
	//processes min/max motor commands.
	motors.write();
	
	
}


