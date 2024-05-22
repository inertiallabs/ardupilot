#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED

#include "AP_ExternalAHRS_command_context.h"

#include <AP_Common/AP_Common.h>

#include <stdint.h>

namespace InertialLabs::AidingData {

struct PACKED External_position {
    int32_t latitude;      // deg*1.0e-7
    int32_t longitude;     // deg*1.0e-7
    int32_t altitude;      // mm
    uint16_t latitudeStd;  // 0.01m
    uint16_t longitudeStd; // 0.01m
    uint16_t altitudeStd;  // 0.01m
    uint16_t latency;      // msec
};

struct PACKED External_horizontal_position {
    int32_t latitude;      // deg*1.0e-7
    int32_t longitude;     // deg*1.0e-7
    uint16_t latitudeStd;  // 0.01m
    uint16_t longitudeStd; // 0.01m
    uint16_t latency;      // msec
};

struct PACKED External_altitude {
    int32_t altitude;      // m*1000
    uint16_t altitudeStd;  // m*100
};

struct PACKED Wind {
    int16_t north;     // kt*0.01
    int16_t east;      // kt*0.01
    uint16_t northStd; // kt*0.01
    uint16_t eastStd;  // kt*0.01
};

struct PACKED Ambient_air {
    int16_t temperature; // C*10
    int32_t altitude;    // m*100
    uint16_t pressure;   // Pa/2
};

struct PACKED External_heading {
    uint16_t heading;    // 0.01deg
    uint16_t headingStd; // 0.01deg
    uint16_t latency;    // msec
};

} // namespace InertialLabs::AidingData

#endif  // AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED
