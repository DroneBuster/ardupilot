#pragma once

#include "AP_RotaryEncoder.h"

class AP_RotaryEncoder_Backend
{
public:
    AP_RotaryEncoder_Backend(AP_RotaryEncoder &encoder);

    // each driver must provide an update method to copy accumulated
    // data to the frontend
    virtual void update() = 0;

    // accumulate function. This is used for backends that don't use a
    // timer, and need to be called regularly by the main code to
    // trigger them to read the sensor
    virtual void accumulate(void) {}

protected:
    // reference to frontend object
    AP_RotaryEncoder &_frontend;

    void _copy_to_frontend(uint8_t instance, float angle, float raw_angle);

    // semaphore for access to shared frontend data
    AP_HAL::Semaphore *_sem;    
};
