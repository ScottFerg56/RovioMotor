#include "DriveMotor.h"

Adafruit_MotorShield DriveMotor::MotorShield = Adafruit_MotorShield();
bool DriveMotor::MotorShieldInitialized = false;
bool DriveMotor::DirectDrive = false;

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void DriveMotor::setProperty(int8_t property, int16_t value)
{
    switch (property)
    {
    case MotorProperties_Goal:
        Goal = value;
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

int16_t DriveMotor::getProperty(int8_t property)
{
    switch (property)
    {
    case MotorProperties_Goal:
        return (int16_t)roundf(Goal);

    case MotorProperties_RPM:
        return (int16_t)roundf(RPM);
    
    case MotorProperties_Power:
        return Power;
    
    case MotorProperties_DirectDrive:
        return (int8_t)DirectDrive;

    default:                    // invalid property
        // UNDONE: error reporting
        return -1;
    }
}

bool DriveMotor::getPropertyChanged(int8_t property)
{
    bool changed = false;
    switch (property)
    {
    case MotorProperties_Goal:
    case MotorProperties_DirectDrive:
        break;

    case MotorProperties_RPM:
        changed = RPMChanged;
        RPMChanged = false;
        break;
    
    case MotorProperties_Power:
        changed = PowerChanged;
        PowerChanged = false;
        break;
    }
    return changed;
}
void DriveMotor::Loop(unsigned long dmsec)
{
    Count = MotorEncoder.getEncoderCount();
    float rpm = (Count * 60000.0) / dmsec / CountsPerRev;
    RPMChanged |= rpm != RPM;
    RPM = rpm;
    MotorEncoder.setEncoderCount(0);
    int power = Power;
    if (Goal == 0 && RPM < 4)
    {
      // avoid problems at low RPM
        power = 0;
        Output = 0;
        ErrorSum = 0;
        dError = 0;
    }
    else
    {
        // PID computations
        float dsec = dmsec / 1.0e3;
        float error = Goal - RPM;
        ErrorSum += error * dsec;
        //ErrorSum = constrain(ErrorSum, MinOut * 1.1, MaxOut * 1.1);
        dError = (error - Error) / dsec;

        // Calculate the new output by adding all three elements together
        Output = Kp * error + Ki * ErrorSum + Kd * dError;
        power += Output;
        power = constrain(power, MinOut, MaxOut);

        Error = error;
        if (DirectDrive)    // stupid direct control for testing purposes
            power = Goal;
    }
    if (sgn(power) != sgn(Goal))  // avoid reversing the motor harshly!
        power = 0;
    PowerChanged |= power != Power;
    Power = power;
    
    Motor->setSpeed(abs(Power));
    Motor->run(Power == 0 ? RELEASE : (Power < 0 ? BACKWARD : FORWARD));
}