#ifndef _DRIVEMOTOR_H
#define _DRIVEMOTOR_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <QuadEncoder.h>
#include "Packet.h"

class DriveMotor : Entity
{
protected:
    Adafruit_DCMotor* Motor;
    Encoders MotorEncoder;
    static bool MotorShieldInitialized;
public:
    static Adafruit_MotorShield MotorShield;
    float Goal = 0;
    static bool DirectDrive;
    long Count = 0;
    const int CountsPerRev = 960;
    float RPM = 0;
    bool RPMChanged = false;
    int16_t Output = 0;
    int16_t Power = 0;
    bool PowerChanged = false;
    float Kp = 3.0;
    float Ki = 0.1;
    float Kd = 0.0;
    const int MinOut = -255;
    const int MaxOut = 255;
    float Error = 0;
    float ErrorSum = 0;
    float dError;

    void Init(int motorNum, byte pinA, byte pinB);
    void Loop(unsigned long dmsec);

    void setProperty(Properties property, int16_t value);
    int16_t getProperty(Properties property);
    bool getPropertyChanged(Properties property);
    bool propertyToBot(Properties property);
    bool propertyFromBot(Properties property);
};

#endif // _DRIVEMOTOR_H
