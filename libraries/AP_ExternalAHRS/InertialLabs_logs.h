#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

namespace InertialLabs {

struct SensorsData;

void write_logs(const SensorsData &sensors_data);

} // namespace InertialLabs

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
