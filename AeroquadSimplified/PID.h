//We my choose to copy this style of PID. This update PID function is called throughout the code.


float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters) {
  float error;
  float dTerm;
  // AKA PID experiments
  float deltaPIDTime = (currentTime - PIDparameters->previousPIDTime) / 1000000.0;

  PIDparameters->previousPIDTime = currentTime;  // AKA PID experiments
  error = targetPosition - currentPosition;

// AKA PID experiments
// special case of +/- PI
/*
  if (PIDparameters->typePID == TYPEPI) {
    if (error >= PI) error -= (2*PI);
    if (error < -PI) error += (2*PI);
  }
*/    
  
  if (PIDparameters->firstPass) { // AKA PID experiments
    PIDparameters->firstPass = false;
    return (constrain(error, -PIDparameters->windupGuard, PIDparameters->windupGuard));
  }

  PIDparameters->integratedError += error * deltaPIDTime;
  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -PIDparameters->windupGuard, PIDparameters->windupGuard);
  
  dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition) / (deltaPIDTime * 100); // dT fix from Honk

  PIDparameters->lastPosition = currentPosition;
  
  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
}

void zeroIntegralError() __attribute__ ((noinline));
void zeroIntegralError() {
  for (byte axis = ROLL; axis < LASTLEVELAXIS; axis++) {
    PID[axis].integratedError = 0;
    PID[axis].previousPIDTime = currentTime;
  }
}



