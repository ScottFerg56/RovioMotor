#include "DriveMotor.h"
#include "FLogger.h"

Adafruit_MotorShield DriveMotor::MotorShield = Adafruit_MotorShield();
bool DriveMotor::MotorShieldInitialized = false;
bool DriveMotor::DirectDrive = false;

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
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
    // compute current RPM from encoder count
    Count = MotorEncoder.getEncoderCount();
    int16_t rpm = (int16_t)roundf((Count * 60000.0) / dmsec / CountsPerRev);
    RPM.Set(rpm);
    MotorEncoder.setEncoderCount(0);
    int power = Power.Get();
    int goal = Goal.Get();
    if (goal == 0 && rpm < 4)
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
        float error = goal - rpm;
        ErrorSum += error * dsec;
        //ErrorSum = constrain(ErrorSum, MinOut * 1.1, MaxOut * 1.1);
        dError = (error - Error) / dsec;

        // Calculate the new output by adding all three elements together
        Output = Kp * error + Ki * ErrorSum + Kd * dError;
        // NOTE: Here I'm using the PID output to modulate the Power
        //      rather than to set it directly as seen in other PID controllers
        //      But that direct method doesn't seem to be stable here!
        //      And I've been unable to tune that approach to make it work
        power += Output;
        power = constrain(power, MinOut, MaxOut);

        Error = error;
        if (DirectDrive)    // stupid direct control for testing purposes
            power = goal;
    }
    if (sgn(power) != sgn(goal))  // avoid reversing the motor harshly!
        power = 0;
    Power.Set(power);
    if (Power.IsChanged())
    {
        // pass Power on to the motor!
        Motor->setSpeed(abs(power));
        Motor->run(power == 0 ? RELEASE : (power < 0 ? BACKWARD : FORWARD));
    }
}