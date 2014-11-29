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
    AP_GROUPINFO("GMB_DWN", 1, AP_OT_Parachute, _servo_gmb_dwn, AP_OT_PARACHUTE_SERVO_GMB_DWN_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: Parachute Servo OFF PWM value
    // @Description: Parachute Servo PWM value when parachute is released
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("GMB_UP", 2, AP_OT_Parachute, _servo_gmb_up, AP_OT_PARACHUTE_SERVO_GMB_UP_DEFAULT),

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

void AP_OT_Parachute::enable_FS(int8_t state)
{
    if (state == FS_ON)
        _enabled_FS = true;
    else if (state == FS_OFF)
        _enabled_FS = false;
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
    _to_release = true;
}

void AP_OT_Parachute::set_ignition(int8_t state)
{
    if (state == IGNITION_ON/* && !_ignition_on*/)
    {
        RC_Channel_aux::set_radio_to_max(RC_Channel_aux::k_ignition_control);
        _ignition_on = true;
    }
    else if (state == IGNITION_OFF /*&&_ignition_on*/)
    {
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_ignition_control);
        _ignition_on = false;
    }

}

void AP_OT_Parachute::set_parachute_servo(int8_t state)
{
    if (state == SERVO_CLOSED/* && _open*/)
    {
        RC_Channel_aux::set_radio_to_max(RC_Channel_aux::k_ot_parachute_release); //close servo
        _open = false;
        //AP_Notify::flags.parachute_release = 0;
    }
    else if (state == SERVO_OPEN/* && !_open*/)
    {
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_ot_parachute_release); //open servo
        _open = true;
    }
}

void AP_OT_Parachute::set_gimbal_pos(int8_t state)
{
    if (state == GIMBAL_DOWN)
        _gimbal_pwm = _servo_gmb_dwn;

    else if (state == GIMBAL_UP)
        _gimbal_pwm = _servo_gmb_up;

    _gimb_state = state;
}

/*void AP_OT_Parachute::control_msg(mavlink_message_t* msg)
{
    //ugly hack
    __mavlink_mount_configure_t packet;
    mavlink_msg_mount_configure_decode(msg, &packet);
    if (mavlink_check_target(packet.target_system, packet.target_component)) {
        // not for us
        return;
    }
    if (packet.mount_mode == 42) // magic reset number
    {
        reset();
        return;
    }

    enable_FS(packet.mount_mode);
    if (packet.stab_roll == 1)
    {
        release(2000, 3, 1000);
    }
    set_ignition(packet.stab_pitch);
    set_parachute_servo(packet.stab_yaw);

}*/

void AP_OT_Parachute::control_msg(int8_t p1, int8_t p2, int8_t p3, int8_t p4)
{
    
    if (p1 == 42) // magic reset number
    {
        reset();
    }

    enable_FS(p1);
    if (p2 == 1)
    {
        release(2000, 3, 1000);
    }
    set_ignition(p3);
    set_parachute_servo(p4);

}

void AP_OT_Parachute::reset(void)
{
    _released = false;
    _to_release = false;
    _release_time = 0;
    _release_delay = 0;
    _release_tries = 0;
    _release_toggle_time = 0;
}



void AP_OT_Parachute::update(int32_t altitude)
{
    if (!_enabled) return;

    uint32_t time = hal.scheduler->millis();

    if (!_released && !_to_release && _enabled_FS)
    {
        if ( _fs_height * 100 > altitude) release(2000, 3, 1000); // Hard coded maybe params?
    }

    if (_to_release)
    {
        set_ignition(IGNITION_OFF);
        set_gimbal_pos(GIMBAL_UP);

        //we need to turn throttle off for electric maybe new mode?

        if (uint16_t(time - _release_time) > _release_delay)
        {
            _released = true;
            _to_release = false;
            _last_toggle = time;
           // AP_Notify::flags.parachute_release = 1; 
        }
    }

    if (_released)
    {
        if (_release_tries > 0 && (int16_t)(time - _last_toggle) >= _release_toggle_time)
        {
            if (_open)
            {
                set_parachute_servo(SERVO_CLOSED);
                _release_tries = _release_tries - 1;
                _last_toggle = time;
            }
            else if (!_open)
            {
                set_parachute_servo(SERVO_OPEN);
                _last_toggle = time;
            }
        }
        else if (_release_tries <= 0)
            set_parachute_servo(SERVO_OPEN);
    }

    float tmp = 500.0f * 0.02f; //100 pwm per second
    if (tmp < 1)
        tmp = 1;
    _gimbal_pwm = constrain_int16(_gimbal_pwm, _last_gimb_pos - tmp, _last_gimb_pos + tmp);
    _last_gimb_pos = _gimbal_pwm;

    RC_Channel_aux::set_radio(RC_Channel_aux::k_gimbal_retract, _gimbal_pwm);
    set_gimbal_pos(_gimb_state);
}