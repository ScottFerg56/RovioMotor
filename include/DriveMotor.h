#ifndef _DRIVEMOTOR_H
#define _DRIVEMOTOR_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <QuadEncoder.h>
#include "MotorBase.h"

/**
 * @brief Entity to support hardware interface to the Rovio drive motors
 */
class DriveMotor : public MotorBase
{
protected:
    /**
     * @brief The motor control hardware interface through Adafruit DC motor FeatherWing
     */
    Adafruit_DCMotor* Motor;
    /**
     * @brief Library to read Rovio motor encoders
     */
    Encoders MotorEncoder;
    /**
     * @brief Flag to remember that DC motor FeatherWing has been initialized
     */
    static bool MotorShieldInitialized;
    unsigned long msPrevLoop;

public:
    /**
     * @brief Construct a new Drive Motor object
     * 
     * @param entity The Entity ID
     * @param name The Entity Name
     */
    DriveMotor(EntityID entity, const char* name) : MotorBase(entity, name) {}
    /**
     * @brief DC motor FeatherWing access
     */
    static Adafruit_MotorShield MotorShield;
    /**
     * @brief direct control of motor Power, bypassing PID for testing purposes
     */
    static bool DirectDrive;
    /**
     * @brief Last encoder Count
     */
    long Count = 0;
    /**
     * @brief The number of encoder counts per wheel revolution
     */
    const int CountsPerRev = 960;
    /**
     * @brief Last PID output value
     */
    int16_t Output = 0;
    /**
     * @brief PID proportional coefficient
     */
    float Kp = 3.0;
    /**
     * @brief PID integral coefficient
     */
    float Ki = 0.1;
    /**
     * @brief PID derivative coefficient
     */
    float Kd = 0.0;
    /**
     * @brief Minimum Power value
     */
    const int MinOut = -255;
    /**
     * @brief Maximum Power value
     */
    const int MaxOut = 255;
    /**
     * @brief Last PID proportional error value
     */
    float Error = 0;
    /**
     * @brief Last PID integral error sum
     */
    float ErrorSum = 0;
    /**
     * @brief Last PID derivitaive error value
     */
    float dError;
    /**
     * @brief Initialize motor interface parameters
     * 
     * @param motorNum The DC Motor FeatherWing motor number
     * @param pinA first Quad encoder Input pin
     * @param pinB second Quad encoder Input pin
     */
    void Init(int motorNum, byte pinA, byte pinB);
    /**
     * @brief loop() processing to occur regularly
     */
    void Loop();
};

#endif // _DRIVEMOTOR_H
