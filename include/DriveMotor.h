#ifndef DriveMotor_h
#define DriveMotor_h

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <QuadEncoder.h>
#include "Packet.h"

class DriveMotor : Entity
{
protected:
    Adafruit_DCMotor* Motor;
    Encoders MotorEncoder;
    static Adafruit_MotorShield MotorShield;
    static bool MotorShieldInitialized;
public:
    float SpeedGoal = 0;
    bool DirectDrive = false;
    long Count = 0;
    int CountsPerRev = 960;
    float RPM = 0;
    bool RPMChanged = false;
    int Output = 0;
    int Power = 0;
    float Kp = 3.0;
    float Ki = 0.1;
    float Kd = 0.0;
    const float MinOut = -255;
    const float MaxOut = 255;
    float Error = 0;
    float ErrorSum = 0;
    float dError;

    void Init(int motorNum, byte pinA, byte pinB)
    {
        if (!MotorShieldInitialized)
        {
            MotorShield.begin();
            MotorShieldInitialized = true;
        }
        Motor = MotorShield.getMotor(motorNum);
        MotorEncoder.Init(pinA, pinB);
    }

    void Loop(unsigned long dmsec);

    void SetProperty(int8_t property, int8_t value);
};

#endif