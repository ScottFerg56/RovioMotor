#ifndef DriveMotor_h
#define DriveMotor_h

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <QuadEncoder.h>

class DriveMotor
{
protected:
    Adafruit_DCMotor* Motor;
    Encoders* MotorEncoder;
public:
    float SpeedGoal = 0;
    long Count = 0;
    int CountsPerRev = 960;
    float RPM = 0;
    int Output = 0;
    int Power = 0;
    float Kp = 3.0;
    float Ki = 0.1;
    float Kd = 0.0;
    float MinOut = -255;
    float MaxOut = 255;
    float Error = 0;
    float ErrorSum = 0;
    float dError;

    void Init(Adafruit_DCMotor* motor, Encoders* motorEncoder)
    {
        Motor = motor;
        MotorEncoder = motorEncoder;
    }

    void Loop(unsigned long dmsec);
};

#endif