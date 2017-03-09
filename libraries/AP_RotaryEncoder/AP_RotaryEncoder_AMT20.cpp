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
    _last_angle = 0.0f;
    _state = WAITING_FOR_NEXT_READ;
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

    //1000Hz
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_RotaryEncoder_AMT20::_timer, bool));
    return true;
}

bool AP_RotaryEncoder_AMT20::_timer() {
    uint8_t buf[2];
    switch (_state) {
        case WAITING_FOR_NEXT_READ:
            if(_last_time_read_cmd + 10 <= AP_HAL::millis()) {
                buf[0] = 0x10;
                _dev->transfer_fullduplex(&buf[0], &buf[1], 1);
                _last_time_read_cmd = AP_HAL::millis();
                if(buf[1] == 0x10) _state = READING_DATA;
                else _state = WAITING_FOR_READ_RESPONSE;
            }
            break;
        case WAITING_FOR_READ_RESPONSE:
            _dev->transfer(nullptr, 0, buf, 1);
            if(buf[0] == 0x10){
                _state = READING_DATA;
            } else if(buf[0] != 0xA5) {
                //printf("Dat: %u\n", buf[0]);
                _failed_read_count++;
                _state = WAITING_FOR_NEXT_READ;
            }
            if(_last_time_read_cmd + 9 <= AP_HAL::millis()) {
                //printf("Rotary not responding %u %u\n", buf[0], buf[1]);
                _state = WAITING_FOR_NEXT_READ;
            }
            break;
        case READING_DATA:
            for(uint8_t i = 0; i < 2; i++) {
                _dev->transfer(nullptr, 0, &buf[i], 1);
            }
            uint16_t reading = buf[0] << 8 | buf[1];
            _last_angle = wrap_PI((M_2PI/4096.0f)*reading - _frontend._offset);
            _last_raw_angle = wrap_PI((M_2PI/4096.0f)*reading);
            _state = WAITING_FOR_NEXT_READ;
            _failed_read_count = 0;
            /*static uint8_t cnt = 0;
            if(++cnt > 5) {
                cnt = 0;
                int16_t read = _last_angle * 1000.0f;
                //printf("Reading: %i\n", read);
            }*/
            break;
    }

    return true;
}

void AP_RotaryEncoder_AMT20::update()
{
    _copy_to_frontend(_instance, _last_angle, _last_raw_angle);
}
