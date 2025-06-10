#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <stddef.h>
#include <stdint.h>

#include "InertialLabs_command.h"

class GCS_MAVLINK;

namespace InertialLabs {

class Sensor;
struct SensorsData;
struct StatusMessage;

constexpr size_t USW_MESSAGE_LIST_SIZE = 14;
constexpr size_t USW2_MESSAGE_LIST_SIZE = 10;
constexpr size_t ADU_MESSAGE_LIST_SIZE = 7;

class Sender
{
public:
    Sender() = default;
    Sender(const Sender &)            = delete;
    Sender &operator=(const Sender &) = delete;
    ~Sender() = default;

    void send_gcs_messages(const SensorsData &sensors_data);

    void send_gcs_eahrs_status_flags(GCS_MAVLINK &link, const SensorsData &sensors_data) const;

    void send_sensor_airspeed_aiding_data(Sensor &sensor) const;

    void send_sensor_command(Sensor &sensor,
                            ExternalAHRS_command command,
                            const ExternalAHRS_command_data &data) const;

private:
    struct GcsMessageState {
        uint16_t last_unit_status{0};
        uint16_t last_unit_status2{0};
        uint16_t last_air_data_status{0};
        uint8_t last_spoof_status{0};
        uint8_t last_jam_status{0};
        uint8_t last_ins_sol_status{0};
        uint8_t last_mag_clb_status{0};
        uint8_t last_mag_clb_accuracy{0};
        uint32_t last_mag_ms{0};
        uint64_t usw_message_timestamp_list_ms[USW_MESSAGE_LIST_SIZE]{0};
        uint64_t usw2_message_timestamp_list_ms[USW2_MESSAGE_LIST_SIZE]{0};
        uint64_t adu_message_timestamp_list_ms[ADU_MESSAGE_LIST_SIZE]{0};
    } state{};
};

} // namespace InertialLabs

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED