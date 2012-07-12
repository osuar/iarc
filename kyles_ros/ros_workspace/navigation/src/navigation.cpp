




int main()
{
	int leftIR = 0;
	int rightIR = 0;
	int frontIR = 0;
	int servoAngle = 0;
	int slope = 0;
	int inflectionAngle = 0;
	int prevDistance[5];
	int currentDistance;
	int slopeSign = 0; //0 == negative, 1 == positive	

	while(ros::ok)
	{
		updateIR(&leftIR, &rightIR, &frontIR);
		updateAngle(&servoAngle);
		
		updateSlope(&slope, prevDistance, &rightIR);
		
		if((slopeSign == 0) && (slope > 0))	//inflection point reached
		{
			inflectionAngle = servoAngle;
			slopeSign = 1;
		}
		else if((slopeSign == 1) && (slope < 0))	//inflection point reached
		{
			inflectionAngle = servoAngle;
			slopeSign = 0;
		}
		
		if( (servoAngle < 50) && (servoAngle > 40) && (frontIR < FRONT_WALL_THRESH) ) //Wall in front
		{
			//YAW COUNTERCLOCKWISE
		}
		else if(inflectionAngle > 50)	//YAW CLOCKWISE
		else if(inflectionAngle < 40)	// YAW COUNTERCLOCKWISE
		else // IN THE MIDDLE, DONT YAW, GO FORWARD		
	}

	
}

void updateSlope(int* slope, int prevDistance[5], int* currentDistance)
{
	int i = 0;
	for (i=0 ; i<4 ; i++)
	{
		prevDistance[i] = prevDistance[i+1];
	}
	prevDistance[4] = *currentDistance;

	*slope = 0;
	int i = 0;
	for (i=0 ; i<5 ; i++)
	{
		*slope = *slope + prevDistance[i];
	}
	*slope = *slope/5;
}

void updateIR(int* leftIR, int* rightIR, int* frontIR)
{
	//handle ros subscription stuff Daniel....
}

void updateAngle(int *angle)
{
	//handle ros subscription stuff Daniel...
}
