/*
  external AHRS airspeed backend
 */
#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_EXTERNAL_ENABLED

#include "AP_Airspeed_Backend.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

class AP_Airspeed_External : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_External(AP_Airspeed &airspeed, uint8_t instance);

    bool init(void) override {
        return true;
    }

    void handle_external(const AP_ExternalAHRS::airspeed_data_message_t &pkt) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override;

    // this reads airspeed directly
    bool has_airspeed() override;

    // read the from the sensor
    bool get_airspeed(float &airspeed) override;

    // use a calculated airspeed
    void set_external_airspeed_enabled(bool &airspeed_enabled) override;

private:
    float sum_pressure;
    uint8_t press_count;
    float sum_temperature;
    uint8_t temperature_count;
    uint8_t airspeed_count;
    float sum_airspeed;
    bool ext_airspeed_enabled;

    uint32_t last_update_ms; // time last message was received

};

#endif // AP_AIRSPEED_EXTERNAL_ENABLED

