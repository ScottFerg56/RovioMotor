#ifndef _NAVLIGHTS_H
#define _NAVLIGHTS_H

#include "NavLightsBase.h"

/**
 * @brief Entity to support hardware interface to the Rovio head motor
 */
class NavLights : public NavLightsBase
{
protected:
    /**
     * @brief The analog input pin for the Rovio head motor position potentiometer
     */
    uint8_t Pin;
public:
    /**
     * @brief Construct a new Head object
     * 
     * @param entity The Entity ID
     * @param name The Entity Name
     */
    NavLights(EntityID entity, const char* name) : NavLightsBase(entity, name) {}
    /**
     * @brief Initialize head motor interface parameters
     */
    void Init();
    /**
     * @brief loop() processing to occur regularly
     */
    void Loop();
};

#endif // _NAVLIGHTS_H
