// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_OT_Parachute.h>
#include <RC_Channel.h>
#include <AP_HAL.h>
#include <AP_Notify.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_OT_Parachute::var_info[] PROGMEM = {

    // @Param: ENABLED
    // @DisplayName: Parachute release enabled or disabled
    // @Description: Parachute release enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLED", 0, AP_OT_Parachute, _enabled, 0),

    // @Param: SERVO_ON
    // @DisplayName: Parachute Servo ON PWM value
    // @Description: Parachute Servo PWM value when parachute is released
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SERVO_ON", 1, AP_OT_Parachute, _servo_on_pwm, AP_OT_PARACHUTE_SERVO_ON_PWM_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: Parachute Servo OFF PWM value
    // @Description: Parachute Servo PWM value when parachute is released
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("SERVO_OFF", 2, AP_OT_Parachute, _servo_off_pwm, AP_OT_PARACHUTE_SERVO_OFF_PWM_DEFAULT),

    // @Param: FS_HEIGHT
    // @DisplayName: Parachute FS height
    // @Description: When plane descends below this altitude parachute will open
    // @Range: 10 32000
    // @Units: meters
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FS_HEIGHT", 3, AP_OT_Parachute, _fs_height, AP_OT_PARACHUTE_FS_HEIGHT_DEFAULT),

    AP_GROUPEND
};

void AP_OT_Parachute::enable_FS(bool state)
{
    _enabled_FS = state;
}

void AP_OT_Parachute::release(uint16_t delay, uint8_t tries, uint16_t time)
{
    _release_delay = delay;
    _release_tries = tries;
    _release_toggle_time = time;
    _release_time = hal.scheduler->millis();
    _to_release = true;
}

void AP_OT_Parachute::release(uint16_t delay)
{
    _release_delay = delay;
    _release_tries = 0;
    _release_toggle_time = 0;
    _release_time = hal.scheduler->millis();
}

void AP_OT_Parachute::update(int32_t altitude)
{
    if (!_enabled) return;

    uint32_t time = hal.scheduler->millis();

    if (!_released && !_to_release && _enabled_FS)
    {
        if ( _fs_height * 100 > altitude) release(1000, 3, 500);
    }
    else if (_to_release)
    {
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_ignition_control); //turn ignition off
        //we need to turn throttle off
        if (uint16_t(time - _release_time) > _release_delay)
        {
            _released = true;
            _to_release = false;
            _last_toggle = time;
            AP_Notify::flags.parachute_release = 1;           //reikia nuli kazkada nustatyti :) 
        }
    }

    if (_released)
    {
        if (_release_tries > 0 && (uint16_t)(time - _last_toggle) >= _release_toggle_time)
        {
            if (_open)
            {
                RC_Channel_aux::set_radio_to_max(RC_Channel_aux::k_ot_parachute_release); //close servo
                _open = false;
                _release_tries = _release_tries - 1;
            }
            else
            {
                RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_ot_parachute_release); //open servo
                _open = true;
            }
            _last_toggle = time;
        }
        else if (_release_tries <= 0)
        {
            RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_ot_parachute_release); //open servo
            _open = true;
        }
    }
    else
    {
        RC_Channel_aux::set_radio_to_max(RC_Channel_aux::k_ot_parachute_release);
        _open = false;
    }
}