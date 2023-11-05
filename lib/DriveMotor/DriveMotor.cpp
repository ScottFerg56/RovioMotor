#include "DriveMotor.h"

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void DriveMotor::Loop(unsigned long dmsec)
{
    Count = MotorEncoder->getEncoderCount();
    RPM = (Count * 60000.0) / dmsec / CountsPerRev;
    MotorEncoder->setEncoderCount(0);

    if (SpeedGoal == 0 && RPM < 4)
    {
      // avoid problems at low RPM
      Power = 0;
      Output = 0;
      ErrorSum = 0;
      dError = 0;
    }
    else
    {
      // PID computations
      float dsec = dmsec / 1.0e3;
      float error = SpeedGoal - RPM;
      ErrorSum += error * dsec;
      //ErrorSum = constrain(ErrorSum, MinOut * 1.1, MaxOut * 1.1);
      dError = (error - Error) / dsec;

      // Calculate the new output by adding all three elements together
      Output = Kp * error + Ki * ErrorSum + Kd * dError;
      Output = constrain(Output, MinOut, MaxOut);

      Error = error;
      Power += Output;
#if false
      // stupid direct control for testing purposes
      Power = SpeedGoal;
#endif
    }
    if (sgn(Power) != sgn(SpeedGoal))  // avoid reversing the motor harshly!
      Power = 0;
    Motor->setSpeed(abs(Power));
    Motor->run(Power == 0 ? RELEASE : (Power < 0 ? BACKWARD : FORWARD));
}