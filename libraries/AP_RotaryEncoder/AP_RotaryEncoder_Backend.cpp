#include "AP_RotaryEncoder_Backend.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_RotaryEncoder_Backend::AP_RotaryEncoder_Backend(AP_RotaryEncoder &encoder) :
    _frontend(encoder)
{
    _sem = hal.util->new_semaphore();    
}

/*
  copy latest data to the frontend from a backend
 */
void AP_RotaryEncoder_Backend::_copy_to_frontend(uint8_t instance, float angle)
{
    if (instance >= _frontend._num_sensors) {
        return;
    }
    _frontend.sensors[instance].angle = angle;
}
