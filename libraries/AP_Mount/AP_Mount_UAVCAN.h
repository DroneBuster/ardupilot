/*
  SToRM32 mount backend class
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_Mount_Backend.h"

#define AP_MOUNT_UAVCAN_RESEND_MS  1000    // resend angle targets to gimbal once per second
#define AP_MOUNT_UAVCAN_SEARCH_MS  60000   // search for gimbal for 1 minute after startup

class AP_Mount_UAVCAN : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_UAVCAN(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager) {}

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

    // configure - allows to configure mount control modes
    virtual void configure(enum MAV_MOUNT_MODE mount_mode, uint8_t stab_roll, uint8_t stab_pitch, uint8_t stab_yaw, enum AP_Mount::ControlMode roll_mode, enum AP_Mount::ControlMode pitch_mode, enum AP_Mount::ControlMode yaw_mode);

private:

    // search for gimbal
    void find_gimbal();

    void send_mount_control();

    // internal variables
    bool _initialised;              // true once the driver has been initialised
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal

    enum AP_Mount::ControlMode _control_mode; //Which control mode currently is used
};
