// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_OT_Parachute.h
/// @brief    Parachute release library for KMTI UAV use only. Use at your own risk!

#ifndef AP_OT_PARACHUTE_H
#define AP_OT_PARACHUTE_H

#include <AP_Param.h>
#include <AP_Common.h>

#define AP_OT_PARACHUTE_SERVO_ON_PWM_DEFAULT 1900
#define AP_OT_PARACHUTE_SERVO_OFF_PWM_DEFAULT 1100
#define AP_OT_PARACHUTE_FS_HEIGHT_DEFAULT 150

/// @class    AP_Parachute
/// @brief    Class managing the release of a parachute and parachute FS

class AP_OT_Parachute {

public:

    ///Constructor
    AP_OT_Parachute()
    {
        _released = false;
        _enabled_FS = false;
        _to_release = false;
        _release_time = -1; // means it is not released
        _release_delay = 0;
        _release_tries = 0;
        _release_toggle_time = 0;
        _open = false;
        AP_Param::setup_object_defaults(this, var_info);
    }

    ///Enable parachute FS
    void enable_FS(bool state);

    ///Returns state of parachute FS
    bool enable_FS() const { return _enabled_FS; }

    ///release parachute (delay - between engine shutoff and parachute deployment (ms)  tries - how many tries to open parachute time - time for servo toogle)
    void release(uint16_t delay, uint8_t tries, uint16_t time);

    void release(uint16_t delay);

    ///if parachute is realesed
    bool release() const { return _released; }

    ///Update loop for parachute FS
    void update(int32_t altitude);

    static const AP_Param::GroupInfo var_info[];

private:

    // variables
    bool _released;            //when squence is finished
    bool _to_release;        //set true to start release sequance
    bool _enabled_FS;
    bool _open;
    uint32_t _release_time; //time when release sequance is started
    uint16_t _release_delay;
    uint8_t _release_tries;
    uint16_t _release_toggle_time;
    uint32_t _last_toggle;
    

    // Parameter
    AP_Int8 _enabled;
    AP_Int16 _servo_on_pwm;
    AP_Int16 _servo_off_pwm;
    AP_Int16 _fs_height;
    

};

#endif