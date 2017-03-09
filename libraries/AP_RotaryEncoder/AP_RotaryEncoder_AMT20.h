#pragma once

#include "AP_RotaryEncoder_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/SPIDevice.h>

class AP_RotaryEncoder_AMT20 : public AP_RotaryEncoder_Backend
{
public:
    void update();

    static AP_RotaryEncoder_Backend *probe(AP_RotaryEncoder &encoder, AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev);
    
private:
    AP_RotaryEncoder_AMT20(AP_RotaryEncoder &encoder, AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev);
    virtual ~AP_RotaryEncoder_AMT20(void) {};
    
    bool _init();
    bool _timer();

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    uint8_t _instance;
    float _last_angle; //Rad from -pi to +pi (Positive direction CCW)
    float _last_raw_angle;
    uint32_t _last_time_read_cmd;
    uint16_t _failed_read_count;

    enum {
        WAITING_FOR_NEXT_READ,
        WAITING_FOR_READ_RESPONSE,
        READING_DATA,
    } AMT20_states;

    uint8_t _state;

};
