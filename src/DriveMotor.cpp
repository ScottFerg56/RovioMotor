#include "DriveMotor.h"
#include "FLogger.h"

Adafruit_MotorShield DriveMotor::MotorShield = Adafruit_MotorShield();
bool DriveMotor::MotorShieldInitialized = false;
bool DriveMotor::DirectDrive = false;

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void DriveMotor::setProperty(Properties property, int16_t value)
{
    switch (property)
    {
    case Properties_Goal:
        Goal = value;
        break;
    
    case Properties_DirectDrive:
        DirectDrive = value != 0;
        break;

    case Properties_RPM:   // read only!
    case Properties_Power: // read only!
    default:                    // invalid property
        // UNDONE: error reporting
        break;
    }
}

int16_t DriveMotor::getProperty(Properties property)
{
    switch (property)
    {
    case Properties_Goal:
        return (int16_t)roundf(Goal);

    case Properties_RPM:
        return (int16_t)roundf(RPM);
    
    case Properties_Power:
        return Power;
    
    case Properties_DirectDrive:
        return (int16_t)DirectDrive;

    default:                    // invalid property
        // UNDONE: error reporting
        return -1;
    }
}

bool DriveMotor::getPropertyChanged(Properties property)
{
    bool changed = false;
    switch (property)
    {
    case Properties_Goal:
    case Properties_DirectDrive:
        break;

    case Properties_RPM:
        changed = RPMChanged;
        RPMChanged = false;
        break;
    
    case Properties_Power:
        changed = PowerChanged;
        PowerChanged = false;
        break;
    }
    return changed;
}

bool DriveMotor::propertyToBot(Properties property)
{
    switch (property)
    {
    case Properties_Goal:
    case Properties_DirectDrive:
        return true;
    default:
        return false;
    }
}

bool DriveMotor::propertyFromBot(Properties property)
{
    switch (property)
    {
    case Properties_RPM:
    case Properties_Power:
        return true;
    default:
        return false;
    }
}

void DriveMotor::Init(int motorNum, byte pinA, byte pinB)
{
    if (!MotorShieldInitialized)
    {
        if (!MotorShield.begin())
            flogf("MotorShield init failed");
        MotorShieldInitialized = true;
    }
    Motor = MotorShield.getMotor(motorNum);
    if (Motor == nullptr)
        flogf("MotorShield getMotor failed");
    MotorEncoder.Init(pinA, pinB);
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
    if (PowerChanged)
    {
        Power = power;
        Motor->setSpeed(abs(Power));
        Motor->run(Power == 0 ? RELEASE : (Power < 0 ? BACKWARD : FORWARD));
    }
}