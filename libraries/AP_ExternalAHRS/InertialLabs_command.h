#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include "AP_ExternalAHRS_command_context.h"

#include <stdint.h>

namespace InertialLabs {

constexpr size_t DATA_BUFFER_SIZE = 100;

struct Data_context {
    uint16_t length{0};
    uint8_t data[DATA_BUFFER_SIZE]{0};
};

uint16_t calculate_checksum(const uint8_t * buf, uint16_t size);

bool fill_command_payload(Data_context & context,
                          ExternalAHRS_command command,
                          const ExternalAHRS_command_data &data);

bool fill_transport_protocol_data(Data_context & context);

void _fill_aiding_data_external_position_payload(Data_context & context, const ExternalAHRS_command_data &data);
void _fill_aiding_data_external_horizontal_position_payload(Data_context & context, const ExternalAHRS_command_data &data);
void _fill_aiding_data_external_altitude_payload(Data_context & context, const ExternalAHRS_command_data &data);
void _fill_aiding_data_wind_payload(Data_context & context, const ExternalAHRS_command_data &data);
void _fill_aiding_data_ambient_air_payload(Data_context & context, const ExternalAHRS_command_data &data);
void _fill_aiding_data_external_heading_payload(Data_context & context, const ExternalAHRS_command_data &data);
void _fill_aiding_data_air_speed_payload(Data_context & context, const ExternalAHRS_command_data &data);
void _fill_aiding_data_doppler_velocity_log_payload(Data_context & context, const ExternalAHRS_command_data &data);

} // namespace InertialLabs

namespace InertialLabs { namespace Command {

const char ENABLE_GNSS[] = "\xAA\x55\x00\x00\x07\x00\x71\x78\x00";
const char DISABLE_GNSS[] = "\xAA\x55\x00\x00\x07\x00\x72\x79\x00";
const char START_VG3DCLB_FLIGHT[] = "\xAA\x55\x00\x00\x07\x00\x26\x2D\x00";
const char STOP_VG3DCLB_FLIGHT[] = "\xAA\x55\x00\x00\x07\x00\x27\x2E\x00";
const char START_UDD[] = "\xAA\x55\x00\x00\x07\x00\x95\x9C\x00";
const char STOP[] = "\xAA\x55\x00\x00\x07\x00\xFE\x05\x01";

}} // namespace Inertiallabs::Command

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
