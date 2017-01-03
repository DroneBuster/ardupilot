/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_RotaryEncoder_AMT20.h"

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

/*
  constructor
 */
AP_RotaryEncoder_AMT20::AP_RotaryEncoder_AMT20(AP_RotaryEncoder &encoder, AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev)
    : AP_RotaryEncoder_Backend(encoder)
    , _dev(std::move(dev))
{
    _cmd_read_angle_sent = false;
    _state_count = 0;
    _last_angle = 0.0f;
    _failed_read_count = 0;
}

AP_RotaryEncoder_Backend *AP_RotaryEncoder_AMT20::probe(AP_RotaryEncoder &encoder,
                                       AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_RotaryEncoder_AMT20 *sensor = new AP_RotaryEncoder_AMT20(encoder, std::move(dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_RotaryEncoder_AMT20::_init()
{
    if (!_dev) {
        return false;
    }

    if (!_dev->get_semaphore()->take(0)) {
        AP_HAL::panic("PANIC: AP_RotaryEncoder_AMT20: failed to take serial semaphore for init");
    }
    
    _instance = _frontend.register_sensor();
    
    _dev->get_semaphore()->give();

    //100Hz
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_RotaryEncoder_AMT20::_timer, bool));
    return true;
}

bool AP_RotaryEncoder_AMT20::_timer() {
    _state_count++;
    if(_cmd_read_angle_sent) {
        _read_angle();
        return true;
    }

    if(_state_count > 10) {
        _cmd_read_angle();
        _state_count = 0;
    }
    return true;
}

void AP_RotaryEncoder_AMT20::update()
{
    _copy_to_frontend(_instance, _last_angle);
}

void AP_RotaryEncoder_AMT20::_cmd_read_angle() {
    if(_cmd_read_angle_sent)
    {
        printf("Angle is already sent");
        return;
    }

    uint8_t angle_cmd = 0x10;
    _dev->transfer(&angle_cmd, 1, nullptr, 0);

    _cmd_read_angle_sent = true;
}

void AP_RotaryEncoder_AMT20::_read_angle() {
    if(!_data_ready()){
        _failed_read_count++;
        if(_failed_read_count > 300) {
            _cmd_read_angle_sent = false;
            _failed_read_count = 0;
            printf("AP_RotaryEncoder_AMT20: Device not responding\n");
        }
        return;
    }
    uint8_t angle_buf[2];
    for(uint8_t i = 0; i < 2; i++) {
        _dev->transfer(nullptr, 0, &angle_buf[i], 1);
    }

    _cmd_read_angle_sent = false;
    uint16_t reading = angle_buf[0] << 8 | angle_buf[1];
    _last_angle = wrap_PI((M_2PI/4096.0f)*reading);

//    int32_t angle = _last_angle * 1000.0f;
 //   printf("%i\n", angle);
}

bool AP_RotaryEncoder_AMT20::_data_ready() {
    if(!_cmd_read_angle_sent) {
        return false;
    }
    uint8_t cmd_buf;
    _dev->transfer(nullptr, 0, &cmd_buf, 1);

    if(cmd_buf == 0x10){
        return true;
    }
    return false;
}
