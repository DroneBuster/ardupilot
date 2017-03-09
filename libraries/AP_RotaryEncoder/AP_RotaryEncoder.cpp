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

/*
 *       APM_RotaryEncoder.cpp - rotary encoder driver
 *
 */
#include "AP_RotaryEncoder.h"

#include <utility>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "AP_RotaryEncoder_AMT20.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_RotaryEncoder::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Encoder type
    // @Description: This sets encoder type
    // @Values: -1:Disabled,0:AMT20
    // @User: Advanced
    AP_GROUPINFO("TYPE", 0, AP_RotaryEncoder, _encoder_type, 0),
    
    // @Param: OFFSET
    // @DisplayName: Encoder offset
    // @Description: Sets encoder offset
    // @Range: -pi +pi
    // @User: Advanced
    AP_GROUPINFO("OFFSET", 1, AP_RotaryEncoder, _offset, 0),

    AP_GROUPEND
};

/*
  AP_RotaryEncoder constructor
 */
AP_RotaryEncoder::AP_RotaryEncoder()
{
    AP_Param::setup_object_defaults(this, var_info);
    _primary = 0; //For now assume single sensor
}

// calibrate the encoder
void AP_RotaryEncoder::calibrate()
{
    // start by assuming all sensors are calibrated (for healthy() test)
    for (uint8_t i=0; i<_num_sensors; i++) {
    }
 }

bool AP_RotaryEncoder::_add_backend(AP_RotaryEncoder_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (_num_drivers >= ROTARY_MAX_DRIVERS) {
        AP_HAL::panic("Too many rotary encoder drivers");
    }
    drivers[_num_drivers++] = backend;
    return true;
}

/*
  macro to add a backend with check for too many sensors
 We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(backend) \
    do { _add_backend(backend);     \
       if (_num_drivers == ROTARY_MAX_DRIVERS || \
          _num_sensors == ROTARY_MAX_INSTANCES) { \
          return; \
       } \
    } while (0)

/*
  initialise the barometer object, loading backend drivers
 */
void AP_RotaryEncoder::init(void)
{
    if(_encoder_type == Encoder_AMT20T) {
        ADD_BACKEND(AP_RotaryEncoder_AMT20::probe(*this,
                std::move(hal.spi->get_device("external0m0"))));
    }
    if (_num_drivers == 0 || _num_sensors == 0 || drivers[0] == nullptr) {
        AP_HAL::panic("AP_RotaryEncoder: unable to initialise driver");
    }
}


/*
  call update on all drivers
 */
void AP_RotaryEncoder::update(void)
{
    for (uint8_t i=0; i<_num_drivers; i++) {
        drivers[i]->update();
    }
}

/*
  get angle from primary
 */
float AP_RotaryEncoder::get_primary_angle() const {
    return sensors[_primary].angle;
}

float AP_RotaryEncoder::get_primary_raw_angle() const {
    return sensors[_primary].raw_angle;
}




/* register a new sensor, claiming a sensor slot. If we are out of
   slots it will panic
*/
uint8_t AP_RotaryEncoder::register_sensor(void)
{
    if (_num_sensors >= ROTARY_MAX_INSTANCES) {
        AP_HAL::panic("Too many rotary encoders");
    }
    return _num_sensors++;
}


/*
  check if all encoders are healthy
 */
bool AP_RotaryEncoder::all_healthy(void) const
{
     for (uint8_t i=0; i<_num_sensors; i++) {
         if (!healthy(i)) {
             return false;
         }
     }
     return _num_sensors > 0;
}
