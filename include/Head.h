#ifndef _HEAD_H
#define _HEAD_H

#include "DriveMotor.h"
#include "HeadBase.h"

/**
 * @brief Entity to support hardware interface to the Rovio head motor
 */
class Head : public HeadBase
{
protected:
    /**
     * @brief The motor control hardware interface through Adafruit DC motor FeatherWing
     */
    Adafruit_DCMotor* Motor;
    /**
     * @brief The analog input pin for the Rovio head motor position potentiometer
     */
    uint8_t potPin;
public:
    /**
     * @brief Construct a new Head object
     * 
     * @param entity The Entity ID
     * @param name The Entity Name
     */
    Head(EntityID entity, const char* name) : HeadBase(entity, name) {}
    /**
     * @brief Initialize head motor interface parameters
     * 
     * @param motorNum The DC Motor FeatherWing motor number
     * @param pin The analog input pin for the Rovio head motor position potentiometer
     */
    void Init(int motorNum, byte pin);
    /**
     * @brief loop() processing to occur regularly
     */
    void Loop();
};

#endif // _HEAD_H
