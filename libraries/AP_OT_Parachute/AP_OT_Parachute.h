// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file    AP_OT_Parachute.h
/// @brief    Parachute release library for KMTI UAV use only. Use at your own risk!

#ifndef AP_OT_PARACHUTE_H
#define AP_OT_PARACHUTE_H

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define AP_OT_PARACHUTE_SERVO_GMB_DWN_DEFAULT 1800
#define AP_OT_PARACHUTE_SERVO_GMB_UP_DEFAULT 1100
#define AP_OT_PARACHUTE_FS_HEIGHT_DEFAULT 150

typedef enum PARACHUTE_SERVO_STATE{
    SERVO_NO_CHANGE = 0,
    SERVO_CLOSED = 1,
    SERVO_OPEN = 2
}PARACHUTE_SERVO_STATE;

typedef enum IGNITION_STATE{
    IGNITION_NO_CHANGE = 0,
    IGNITION_ON = 1,
    IGNITION_OFF = 2
}IGNITION_STATE;

typedef enum PARACHUTE_FS_STATE{
    FS_NO_CHANGE,
    FS_ON = 1,
    FS_OFF = 2
} PARACHUTE_FS_STATE;

typedef enum GIMBAL_RETRACT{
    NO_CHANGE,
    GIMBAL_DOWN = 1,
    GIMBAL_UP = 2
} GIMBAL_RETRACT;

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
        _release_time = 0;
        _release_delay = 0;
        _release_tries = 0;
        _release_toggle_time = 0;
        _open = false;
        _ignition_on = false;
        _last_gimb_pos = 1500;
        AP_Param::setup_object_defaults(this, var_info);
    }

    ///Enable parachute FS
    void enable_FS(int8_t state); // 1 - enabled. -1 - disabled

    ///Returns state of parachute FS
    bool enable_FS() const { return _enabled_FS; }

    ///release parachute (delay - between engine shutoff and parachute deployment (ms)  tries - how many tries to open parachute time - time for servo toogle)
    void release(uint16_t delay, uint8_t tries, uint16_t time);

    void release(uint16_t delay);

    ///set ignition state
    void set_ignition(int8_t state); // -1 - off, 1 - on  Enumas!!!!!!!!!!!!!!!!!

    ///set parachute servo only
    void set_parachute_servo(int8_t state); // -1 - open, 1 - closed

    ///set gimbal position
    void set_gimbal_pos(int8_t state);

    ///Handling of MAVLINK CMD command
   // void control_msg(mavlink_message_t* msg);
    void control_msg(int8_t p1, int8_t p2, int8_t p3, int8_t p4);

    ///if parachute is realesed
    bool release() const { return _released; }

    ///Update loop for parachute FS
    void update(float altitude);

    ///Reset after FS
    void reset(void);

    static const AP_Param::GroupInfo var_info[];

private:

    // variables
    bool _released;            //when squence is finished
    bool _to_release;        //set true to start release sequance
    bool _enabled_FS;
    bool _open;
    bool _ignition_on;
    uint8_t _gimb_state;
    uint32_t _release_time; //time when release sequance is started
    uint16_t _release_delay;
    uint8_t _release_tries;
    uint16_t _release_toggle_time;
    uint32_t _last_toggle;
    int16_t _last_gimb_pos; //last gimbal position
    int16_t _gimbal_pwm; //gimbal pwm
    

    // Parameter
    AP_Int8 _enabled;
    AP_Int16 _servo_gmb_dwn;
    AP_Int16 _servo_gmb_up;
    AP_Int16 _fs_height; 

};

#endif
