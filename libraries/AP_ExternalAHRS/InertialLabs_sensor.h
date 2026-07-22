#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include "InertialLabs_data.h"
#include "InertialLabs_readDataEnum.h"

namespace InertialLabs {

constexpr uint16_t BUFFER_SIZE = 500;

class Sensor
{
public:
    Sensor();
    Sensor(const Sensor &)            = delete;
    Sensor &operator=(const Sensor &) = delete;
    ~Sensor()                         = default;

    bool init();
    bool is_initialized() const;
    int8_t get_port() const;

    DataReadStatus update_data();
    const SensorsData & get_sensors_data() const;
    const SensorDiagnosticData & get_diagnostic_data() const;
    void write_bytes(const char *bytes, uint8_t len);

private:
    DataReadStatus read_data();
    bool move_message_header_to_buffer_start();

    bool parse_udd_payload();

    bool has_udd_required_data_types() const;

private:
    AP_HAL::UARTDriver *_uart{nullptr};
    int8_t _port_num{-1};
    uint32_t _baudrate{0};

    bool _is_initialized{false};

    uint8_t _buf[BUFFER_SIZE]{};
    uint16_t _buf_offset{0};

    SensorsData _sensors_data{};
    SensorDiagnosticData _diagnostic_data{};
};

} // namespace Inertiallabs

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
