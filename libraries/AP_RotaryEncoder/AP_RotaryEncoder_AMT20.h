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
    /*
     * Update @accum and @count with the new sample in @val, taking into
     * account a maximum number of samples given by @max_count; in case
     * maximum number is reached, @accum and @count are updated appropriately
     */
    static void _update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                             uint8_t *count, uint8_t max_count);

    AP_RotaryEncoder_AMT20(AP_RotaryEncoder &encoder, AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev);
    virtual ~AP_RotaryEncoder_AMT20(void) {};
    
    bool _init();
    bool _timer();
    void _cmd_read_angle();
    void _read_angle();
    bool _data_ready();

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    uint8_t _state;
    uint8_t _instance;
    float _last_angle; //Rad from -pi to +pi (Positive direction CCW)
    uint16_t _state_count;
    uint16_t _failed_read_count;

    bool _cmd_read_angle_sent;

};
