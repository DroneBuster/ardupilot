#include "AP_Mount_UAVCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#if HAL_WITH_UAVCAN
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

extern const AP_HAL::HAL& hal;

AP_Mount_UAVCAN::AP_Mount_UAVCAN(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _initialised(false),
    _last_send(0),
    _control_mode(AP_Mount::Control_Angle_Absolute_Frame)
{}

// update mount position - should be called periodically
void AP_Mount_UAVCAN::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    switch(get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT:
            {
            const Vector3f &target = _state._retract_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            _control_mode = AP_Mount::Control_Angle_Body_Frame;
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            _control_mode = AP_Mount::Control_Angle_Body_Frame;
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            resend_now = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            resend_now = true;
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            // do nothing. _state._roi_target Should be set by GCS. Calculations done by gimbal
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // resend target angles at least once per second
    if (resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_UAVCAN_RESEND_MS)) {
        send_mount_control();
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_UAVCAN::has_pan_control() const
{
    // we have pan control
    return true;
}

// set_mode - sets mount's mode
void AP_Mount_UAVCAN::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_UAVCAN::status_msg(mavlink_channel_t chan)
{
    // return target angles as gimbal's actual attitude.  To-Do: retrieve actual gimbal attitude and send these instead
    mavlink_msg_mount_status_send(chan, 0, 0, ToDeg(_angle_ef_target_rad.y)*100, ToDeg(_angle_ef_target_rad.x)*100, ToDeg(_angle_ef_target_rad.z)*100);
}

void AP_Mount_UAVCAN::configure(enum MAV_MOUNT_MODE mount_mode, uint8_t stab_roll, uint8_t stab_pitch, uint8_t stab_yaw, enum AP_Mount::ControlMode roll_mode, enum AP_Mount::ControlMode pitch_mode, enum AP_Mount::ControlMode yaw_mode)
{
    set_mode(mount_mode);
    _control_mode = roll_mode;
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_UAVCAN::find_gimbal()
{
    // return immediately if initialised
    if (_initialised) {
        return;
    }

    // return if search time has has passed
    if (AP_HAL::millis() > AP_MOUNT_UAVCAN_SEARCH_MS) {
        return;
    }

    _initialised = true;
}

void AP_Mount_UAVCAN::send_mount_control()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    if (AP_BoardConfig_CAN::get_can_num_ifaces() != 0) {
        for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
            if (hal.can_mgr[i] != nullptr) {
                AP_UAVCAN *uavcan = hal.can_mgr[i]->get_UAVCAN();
                if (uavcan != nullptr) {
                    uavcan->mount_write(get_mode() == MAV_MOUNT_MODE_GPS_POINT, _angle_ef_target_rad, _state._roi_target, _control_mode);
                }
            }
        }
    }

    // store time of send
    _last_send = AP_HAL::millis();
}
#endif
