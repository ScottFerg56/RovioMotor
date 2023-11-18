#include "DriveMotor.h"

Adafruit_MotorShield DriveMotor::MotorShield = Adafruit_MotorShield();
bool DriveMotor::MotorShieldInitialized = false;
bool DriveMotor::DirectDrive = false;

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void DriveMotor::SetProperty(int8_t property, int8_t value)
{
    switch (property)
    {
    case MotorProperties_Goal:
        SpeedGoal = value;
        break;
    
    case MotorProperties_DirectDrive:
        DirectDrive = value != 0;
        break;

    case MotorProperties_RPM:   // read only!
    case MotorProperties_Power: // read only!
    default:                    // invalid property
        // UNDONE: error reporting
        break;
    }
}

void DriveMotor::Loop(unsigned long dmsec)
{
    Count = MotorEncoder.getEncoderCount();
    float rpm = (Count * 60000.0) / dmsec / CountsPerRev;
    RPMChanged |= rpm != RPM;
    RPM = rpm;
    MotorEncoder.setEncoderCount(0);
    float power = 0;
    if (SpeedGoal == 0 && RPM < 4)
    {
      // avoid problems at low RPM
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
        power += Output;
        if (DirectDrive)    // stupid direct control for testing purposes
            power = SpeedGoal;
    }
    if (sgn(power) != sgn(SpeedGoal))  // avoid reversing the motor harshly!
        power = 0;
    PowerChanged |= power != Power;
    Power = power;
    
    Motor->setSpeed(abs(Power));
    Motor->run(Power == 0 ? RELEASE : (Power < 0 ? BACKWARD : FORWARD));
}