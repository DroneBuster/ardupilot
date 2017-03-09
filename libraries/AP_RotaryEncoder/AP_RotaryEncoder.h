#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

// maximum number of sensor instances
#define ROTARY_MAX_INSTANCES 1

// maximum number of drivers. Note that a single driver can provide
// multiple sensor instances
#define ROTARY_MAX_DRIVERS 1

class AP_RotaryEncoder_Backend;

class AP_RotaryEncoder
{
    friend class AP_RotaryEncoder_Backend;

    enum Encoder_Type {
        Encoder_AMT20T,
    };

public:
    // constructor
    AP_RotaryEncoder();

    // initialise the rotary encoder object, loading backend drivers
    void init(void);

    // update the rotary encoder object, asking backends to push data to
    // the frontend
    void update(void);

    // healthy - returns true if sensor and derived altitude are good
    bool healthy(void) const { return healthy(_primary); }
    bool healthy(uint8_t instance) const { return sensors[instance].healthy;}

    // check if all rotary encoders are healthy - used for SYS_STATUS report
    bool all_healthy(void) const;

    float get_primary_angle() const;

    float get_primary_raw_angle() const;

    // calibrate the rotary encoder.
    void calibrate(void);

    // get last time sample was taken (in ms)
    uint32_t get_last_update(void) const { return get_last_update(_primary); }
    uint32_t get_last_update(uint8_t instance) const { return sensors[_primary].last_update_ms; }

    // settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // register a new sensor, claiming a sensor slot. If we are out of
    // slots it will panic
    uint8_t register_sensor(void);

    // return number of registered sensors
    uint8_t num_instances(void) const { return _num_sensors; }

    AP_Float _offset;

private:
    // how many drivers do we have?
    uint8_t _num_drivers;
    AP_RotaryEncoder_Backend *drivers[ROTARY_MAX_DRIVERS];

    // how many sensors do we have?
    uint8_t _num_sensors;

    // what is the primary sensor at the moment?
    uint8_t _primary;

    struct sensor {
        uint32_t last_update_ms;        // last update time in ms
        bool healthy:1;                 // true if sensor is healthy
        float angle;                    // absolute angle
        float raw_angle;
    } sensors[ROTARY_MAX_INSTANCES];

    AP_Int8     _encoder_type; // encoder type chosen by user

    // when did we last notify the GCS of new pressure reference?
    uint32_t                            _last_notify_ms;

    bool _add_backend(AP_RotaryEncoder_Backend *backend);
};
