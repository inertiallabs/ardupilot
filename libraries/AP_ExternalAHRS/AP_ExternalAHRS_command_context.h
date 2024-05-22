#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_VECTORNAV_ENABLED

#include <stdint.h>

enum class ExternalAHRS_command {
    INVALID,
    START_UDD,
    STOP,
    ENABLE_GNSS,
    DISABLE_GNSS,
    START_VG3D_CALIBRATION_IN_FLIGHT,
    STOP_VG3D_CALIBRATION_IN_FLIGHT,
    AIDING_DATA_EXTERNAL_POSITION,
    AIDING_DATA_EXTERNAL_HORIZONTAL_POSITION,
    AIDING_DATA_EXTERNAL_ALTITUDE,
    AIDING_DATA_WIND,
    AIDING_DATA_AMBIENT_AIR,
    AIDING_DATA_EXTERNAL_HEADING,
};

struct ExternalAHRS_command_data{
    float param1; /*<  PARAM1, see MAV_CMD enum*/
    float param2; /*<  PARAM2, see MAV_CMD enum*/
    float param3; /*<  PARAM3, see MAV_CMD enum*/
    float param4; /*<  PARAM4, see MAV_CMD enum*/
    int32_t x; /*<  PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7*/
    int32_t y; /*<  PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7*/
    float z; /*<  PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).*/
};

#endif
