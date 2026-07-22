#include "InertialLabs_sensor.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

namespace {

// acceleration due to gravity in m/s^2 used in IL INS
constexpr float IL_GRAVITY_MSS = 9.8106f;

/*
    Every data package consist of:
        MessageHeader (6 bytes)
        Payload
        Checksum (2 bytes)

    Whole data package size = MessageHeader->msgLen + 2(0x55AA)

    User Defined Data payload consist of:
        Number of data structures
        List of data structures ID
        List of data structures Values

*/
struct PACKED MessageHeader {
    uint16_t magic_number{0};    // 0x55AA
    uint8_t msg_type{0};         // always 1 for INS data
    uint8_t msg_id{0};           // always 0x95
    uint16_t msg_len{0};         // payload + 6(msgType + msgId + msgLen + checksum). Not included packageHeader
};

constexpr uint16_t MAGIC_NUMBER = 0x55AA;
constexpr uint16_t MAGIC_NUMBER_LEN = sizeof(MAGIC_NUMBER);
constexpr uint16_t MESSAGE_HEADER_LEN = sizeof(MessageHeader);
constexpr uint16_t MESSAGE_CHECKSUM_LEN = sizeof(uint16_t);
constexpr uint16_t MESSAGE_LEN_WITH_EMPTY_PAYLOAD = MESSAGE_HEADER_LEN + MESSAGE_CHECKSUM_LEN;

bool is_message_header_valid(const MessageHeader *h)
{
    return h->magic_number == MAGIC_NUMBER &&
        h->msg_type == 1 &&
        h->msg_id == 0x95 &&
        h->msg_len > MESSAGE_LEN_WITH_EMPTY_PAYLOAD &&
        h->msg_len <= InertialLabs::BUFFER_SIZE - MAGIC_NUMBER_LEN;
}

} // namespace

namespace InertialLabs {

Sensor::Sensor()
{
    auto &sm = AP::serialmanager();
    _uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!_uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "InertialLabs ExternalAHRS no UART");
        return;
    }
    _baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    _port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
}

bool Sensor::init()
{
    // We assume the user has already configured the device
    // Open port in the thread
    _uart->begin(_baudrate, 1024, 512);
    _is_initialized = true;
    return true;
}

bool Sensor::is_initialized() const
{
    return _is_initialized;
}

int8_t Sensor::get_port() const
{
    if (!_uart) {
        return -1;
    }
    return _port_num;
};

DataReadStatus Sensor::update_data()
{
    DataReadStatus res = read_data();
    if (res != DataReadStatus::SUCCESS) {
        return res;
    }

    _sensors_data.package_timestamp_us = AP_HAL::micros64();

    if(!parse_udd_payload()) {
        _sensors_data.package_timestamp_us = 0;
        return DataReadStatus::NEED_WAIT;
    }

    _buf_offset = 0;
    return DataReadStatus::SUCCESS;
}

const SensorsData & Sensor::get_sensors_data() const
{
    return _sensors_data;
}

const SensorDiagnosticData & Sensor::get_diagnostic_data() const
{
    return _diagnostic_data;
}

void Sensor::write_bytes(const char *bytes, uint8_t len)
{
    const size_t nwritten = _uart->write(reinterpret_cast<const uint8_t *>(bytes), len);
    if (nwritten != len) {
        _diagnostic_data.uart_write_fail_count++;
    }
}

DataReadStatus Sensor::read_data()
{
    uint32_t n = _uart->available();
    if (!n) {
        return DataReadStatus::NEED_WAIT;
    }
    if (n + _buf_offset > BUFFER_SIZE) {
        n = BUFFER_SIZE - _buf_offset;
    }

    const MessageHeader *message_header = reinterpret_cast<MessageHeader *>(_buf);

    if (_buf_offset < MESSAGE_HEADER_LEN) {
        n = MIN(n, uint32_t(MESSAGE_HEADER_LEN - _buf_offset));
    } else {
        if (!is_message_header_valid(message_header)) {
            move_message_header_to_buffer_start();
            return DataReadStatus::NO_WAIT;
        }

        if (_buf_offset > message_header->msg_len + MESSAGE_CHECKSUM_LEN) {
            move_message_header_to_buffer_start();
            return DataReadStatus::NO_WAIT;
        }

        // Here header is correct
        const uint16_t bytes_till_message_end = message_header->msg_len + MESSAGE_CHECKSUM_LEN - _buf_offset;
        n = MIN(n, bytes_till_message_end);
    }

    ssize_t nread = _uart->read(&_buf[_buf_offset], n);
    if (nread != ssize_t(n)) {
        move_message_header_to_buffer_start();
        return DataReadStatus::NO_WAIT;
    }

    _buf_offset += nread;

    if (_buf_offset < MESSAGE_HEADER_LEN) {
        return DataReadStatus::NEED_WAIT;
    }

    if (!is_message_header_valid(message_header)) {
        move_message_header_to_buffer_start();
        return DataReadStatus::NO_WAIT;
    }

    const uint16_t total_message_len = message_header->msg_len + MESSAGE_CHECKSUM_LEN;
    if (_buf_offset < total_message_len) {
        // Try to read least message data
        n = total_message_len - _buf_offset;
        if (_uart->available() < n) {
            // need more data
            return DataReadStatus::NEED_WAIT;
        }
        nread = _uart->read(&_buf[_buf_offset], n);
        if (nread != ssize_t(n)) {
            move_message_header_to_buffer_start();
            return DataReadStatus::NO_WAIT;
        }
        _buf_offset += nread;
    }

    // Check checksum
    const uint16_t calculated_checksum = crc_sum_of_bytes_16(&_buf[2], message_header->msg_len - MESSAGE_CHECKSUM_LEN);
    const uint16_t read_checksum = le16toh_ptr(&_buf[message_header->msg_len]);
    if (calculated_checksum != read_checksum) {
        _diagnostic_data.checksum_fail_count++;
        move_message_header_to_buffer_start();
        return DataReadStatus::NO_WAIT;
    }

    return DataReadStatus::SUCCESS;
}

bool Sensor::move_message_header_to_buffer_start()
{
    if (!_buf_offset) {
        return false;
    }

    if (_buf_offset <= MAGIC_NUMBER_LEN) {
        _buf_offset = 0;
        return false;
    }

    const uint8_t *start_package_pos = (const uint8_t *)memmem(&_buf[1],
                                       _buf_offset - MAGIC_NUMBER_LEN,
                                       &MAGIC_NUMBER,
                                       MAGIC_NUMBER_LEN);
    if (!start_package_pos)
    {
        _buf_offset = 0;
        return false;
    }

    const uint16_t bytes_from_buffer_start = start_package_pos - _buf;
    memmove(_buf, start_package_pos, _buf_offset - bytes_from_buffer_start);
    _buf_offset -= bytes_from_buffer_start;
    return true;
}

bool Sensor::parse_udd_payload()
{
    const MessageHeader *message_header = reinterpret_cast<MessageHeader *>(_buf);
    const uint16_t payload_size = message_header->msg_len - MESSAGE_HEADER_LEN;
    const uint8_t *payload = &_buf[6];

    const uint8_t message_count = payload[0];
    if (message_count == 0 || message_count > payload_size - 1)
    {
        _diagnostic_data.udd_parse_fail_count++;
        move_message_header_to_buffer_start();
        return false;
    }

    // 1 byte for message count + byte list of messages types
    const uint8_t *message_data_offset = &payload[1 + message_count];

    _sensors_data.udd_data_types_list.clearall();

    for (uint8_t i = 0; i < message_count; i++) {
        uint8_t message_length = 0;
        UDDMessageData &udd = *(UDDMessageData *)message_data_offset;
        uint8_t message_type = payload[1 + i];

        _sensors_data.udd_data_types_list.set(static_cast<uint16_t>(message_type));

        switch (message_type) {
            case DataType::GPS_INS_TIME_MS: {
                // this is the GPS tow timestamp in ms for when the IMU data was sampled
                _sensors_data.ins.ms_tow = udd.gps_time_ms;
                message_length = sizeof(udd.gps_time_ms);
                break;
            }
            case DataType::GPS_WEEK: {
                _sensors_data.gps.gps_week = udd.gps_week;
                message_length = sizeof(udd.gps_week);
                break;
            }
            case DataType::ACCEL_DATA_HR: {
                _sensors_data.accel = udd.accel_data_hr.tofloat().rfu_to_frd()*IL_GRAVITY_MSS*1.0e-6f; // NED, in m/s^2
                message_length = sizeof(udd.accel_data_hr);
                break;
            }
            case DataType::GYRO_DATA_HR: {
                _sensors_data.gyro = udd.gyro_data_hr.tofloat().rfu_to_frd()*1.0e-5f*DEG_TO_RAD; // NED, in rad/s
                message_length = sizeof(udd.gyro_data_hr);
                break;
            }
            case DataType::BARO_DATA: {
                _sensors_data.pressure = static_cast<float>(udd.baro_data.pressure_pa2)*2.0f; // Pa
                _sensors_data.ins.baro_alt = static_cast<float>(udd.baro_data.baro_alt)*0.01f; // m
                message_length = sizeof(udd.baro_data);
                break;
            }
            case DataType::MAG_DATA: {
                _sensors_data.mag = udd.mag_data.tofloat().rfu_to_frd()*(10.0f*NTESLA_TO_MGAUSS); // NED, in milligauss
                message_length = sizeof(udd.mag_data);
                break;
            }
            case DataType::SENSOR_BIAS: {
                _sensors_data.ins.sensor_bias = udd.sensor_bias;
                message_length = sizeof(udd.sensor_bias);
                break;
            }
            case DataType::ORIENTATION_ANGLES: {
                _sensors_data.ins.yaw = static_cast<float>(udd.orientation_angles.yaw)*0.01f; // deg
                _sensors_data.ins.pitch = static_cast<float>(udd.orientation_angles.pitch)*0.01f; // deg
                _sensors_data.ins.roll = static_cast<float>(udd.orientation_angles.roll)*0.01f; // deg
                message_length = sizeof(udd.orientation_angles);
                break;
            }
            case DataType::VELOCITIES: {
                _sensors_data.ins.velocity = udd.velocity.tofloat().rfu_to_frd()*0.01f; // NED, in m/s
                message_length = sizeof(udd.velocity);
                break;
            }
            case DataType::POSITION: {
                _sensors_data.ins.latitude = udd.position.lat; // deg*1.0e7
                _sensors_data.ins.longitude = udd.position.lon; // deg*1.0e7
                _sensors_data.ins.altitude = udd.position.alt; // cm
                message_length = sizeof(udd.position);
                break;
            }
            case DataType::KF_VEL_COVARIANCE: {
                _sensors_data.ins.kf_vel_covariance = udd.kf_vel_covariance.tofloat().rfu_to_frd(); // NED, in mm/s
                break;
            }
            case DataType::KF_POS_COVARIANCE: {
                _sensors_data.ins.kf_pos_covariance = udd.kf_pos_covariance.tofloat(); // NED, in mm
                break;
            }
            case DataType::UNIT_STATUS: {
                _sensors_data.ins.unit_status = udd.unit_status;
                message_length = sizeof(udd.unit_status);
                break;
            }
            case DataType::GNSS_EXTENDED_INFO: {
                _sensors_data.gps.fix_type = udd.gnss_extended_info.fix_type;
                _sensors_data.gps.spoof_status = udd.gnss_extended_info.spoofing_status;
                message_length = sizeof(udd.gnss_extended_info);
                break;
            }
            case DataType::GNSS_POSITION: {
                _sensors_data.gps.latitude = udd.gnss_position.lat; // deg*1.0e7
                _sensors_data.gps.longitude = udd.gnss_position.lon; // deg*1.0e7
                _sensors_data.gps.altitude = udd.gnss_position.alt; // cm
                message_length = sizeof(udd.gnss_position);
                break;
            }
            case DataType::GNSS_VEL_TRACK: {
                _sensors_data.gps.hor_speed = static_cast<float>(udd.gnss_vel_track.hor_speed)*0.01f; // m/s
                _sensors_data.gps.track_over_ground = static_cast<float>(udd.gnss_vel_track.track_over_ground)*0.01f; // deg
                _sensors_data.gps.ver_speed = static_cast<float>(udd.gnss_vel_track.ver_speed)*0.01f; // m/s
                message_length = sizeof(udd.gnss_vel_track);
                break;
            }
            case DataType::GNSS_POS_TIMESTAMP: {
                _sensors_data.gps.ms_tow = udd.gnss_pos_timestamp;
                message_length = sizeof(udd.gnss_pos_timestamp);
                break;
            }
            case DataType::GNSS_NEW_DATA: {
                _sensors_data.gps.new_data = udd.gnss_new_data;
                message_length = sizeof(udd.gnss_new_data);
                break;
            }
            case DataType::GNSS_JAM_STATUS: {
                _sensors_data.gps.jam_status = udd.gnss_jam_status;
                message_length = sizeof(udd.gnss_jam_status);
                break;
            }
            case DataType::DIFFERENTIAL_PRESSURE: {
                _sensors_data.diff_press = static_cast<float>(udd.differential_pressure)*0.01f; // Pa
                message_length = sizeof(udd.differential_pressure);
                break;
            }
            case DataType::TRUE_AIRSPEED: {
                _sensors_data.ins.true_airspeed = static_cast<float>(udd.true_airspeed)*0.01f; // m/s
                message_length = sizeof(udd.true_airspeed);
                break;
            }
            case DataType::CALIBRATED_AIRSPEED: {
                _sensors_data.ins.calibrated_airspeed = static_cast<float>(udd.calibrated_airspeed)*0.01f; // m/s
                message_length = sizeof(udd.calibrated_airspeed);
                break;
            }
            case DataType::WIND_SPEED: {
                _sensors_data.ins.airspeed_sf = udd.wind_speed.tofloat().z*1.0e-3f;
                _sensors_data.ins.wind_speed = udd.wind_speed.tofloat().rfu_to_frd()*0.01f; // NED, in m/s
                _sensors_data.ins.wind_speed.z = 0.0f;
                message_length = sizeof(udd.wind_speed);
                break;
            }
            case DataType::AIR_DATA_STATUS: {
                _sensors_data.ins.air_data_status = udd.air_data_status;
                message_length = sizeof(udd.air_data_status);
                break;
            }
            case DataType::SUPPLY_VOLTAGE: {
                _sensors_data.supply_voltage = static_cast<float>(udd.supply_voltage)*0.01f; // V
                message_length = sizeof(udd.supply_voltage);
                break;
            }
            case DataType::TEMPERATURE: {
                _sensors_data.temperature = static_cast<float>(udd.temperature)*0.1f; // degC
                message_length = sizeof(udd.temperature);
                break;
            }
            case DataType::UNIT_STATUS2: {
                _sensors_data.ins.unit_status2 = udd.unit_status2;
                message_length = sizeof(udd.unit_status2);
                break;
            }
            case DataType::GNSS_DOP: {
                _sensors_data.gps.dop.gdop = udd.gnss_dop.gdop;
                _sensors_data.gps.dop.pdop = udd.gnss_dop.pdop;
                _sensors_data.gps.dop.hdop = udd.gnss_dop.hdop;
                _sensors_data.gps.dop.vdop = udd.gnss_dop.vdop;
                _sensors_data.gps.dop.tdop = udd.gnss_dop.tdop;
                message_length = sizeof(udd.gnss_dop);
                break;
            }
            case DataType::INS_SOLUTION_STATUS: {
                _sensors_data.ins.ins_sol_status = udd.ins_sol_status;
                message_length = sizeof(udd.ins_sol_status);
                break;
            }
            case DataType::INS_POS_VEL_ACCURACY: {
                _sensors_data.ins.ins_accuracy.lat = udd.ins_accuracy.lat;
                _sensors_data.ins.ins_accuracy.lon = udd.ins_accuracy.lon;
                _sensors_data.ins.ins_accuracy.alt = udd.ins_accuracy.alt;
                _sensors_data.ins.ins_accuracy.east_vel = udd.ins_accuracy.east_vel;
                _sensors_data.ins.ins_accuracy.north_vel = udd.ins_accuracy.north_vel;
                _sensors_data.ins.ins_accuracy.ver_vel = udd.ins_accuracy.ver_vel;
                message_length = sizeof(udd.ins_accuracy);
                break;
            }
            case DataType::FULL_SAT_INFO: {
                _sensors_data.gps.full_sat_info.SVs = udd.full_sat_info.SVs;
                _sensors_data.gps.full_sat_info.SolnSVs = udd.full_sat_info.SolnSVs;
                _sensors_data.gps.full_sat_info.SolnL1SVs = udd.full_sat_info.SolnL1SVs;
                _sensors_data.gps.full_sat_info.SolnMultiSVs = udd.full_sat_info.SolnMultiSVs;
                _sensors_data.gps.full_sat_info.signal_used1 = udd.full_sat_info.signal_used1;
                _sensors_data.gps.full_sat_info.signal_used2 = udd.full_sat_info.signal_used2;
                _sensors_data.gps.full_sat_info.GPS_time_status = udd.full_sat_info.GPS_time_status;
                _sensors_data.gps.full_sat_info.ext_sol_status = udd.full_sat_info.ext_sol_status;
                message_length = sizeof(udd.full_sat_info);
                break;
            }
            case DataType::GNSS_VEL_LATENCY: {
                _sensors_data.gps.vel_latency = udd.gnss_vel_latency;
                message_length = sizeof(udd.gnss_vel_latency);
                break;
            }
            case DataType::GNSS_SOL_STATUS: {
                _sensors_data.gps.gnss_sol_status = udd.gnss_sol_status;
                message_length = sizeof(udd.gnss_sol_status);
                break;
            }
            case DataType::GNSS_POS_VEL_TYPE: {
                _sensors_data.gps.gnss_pos_vel_type = udd.gnss_pos_vel_type;
                message_length = sizeof(udd.gnss_pos_vel_type);
                break;
            }
            case DataType::NEW_AIDING_DATA: {
                _sensors_data.ext.new_aiding_data = udd.new_aiding_data;
                message_length = sizeof(udd.new_aiding_data);
                break;
            }
            case DataType::NEW_AIDING_DATA2: {
                _sensors_data.ext.new_aiding_data2 = udd.new_aiding_data2;
                message_length = sizeof(udd.new_aiding_data2);
                break;
            }
            case DataType::EXT_SPEED: {
                _sensors_data.ext.speed = udd.external_speed;
                message_length = sizeof(udd.external_speed);
                break;
            }
            case DataType::EXT_HOR_POS: {
                _sensors_data.ext.hor_pos = udd.ext_hor_pos;
                message_length = sizeof(udd.ext_hor_pos);
                break;
            }
            case DataType::EXT_ALT: {
                _sensors_data.ext.alt = udd.ext_alt;
                message_length = sizeof(udd.ext_alt);
                break;
            }
            case DataType::EXT_HEADING: {
                _sensors_data.ext.heading = udd.ext_heading;
                message_length = sizeof(udd.ext_heading);
                break;
            }
            case DataType::EXT_AMBIENT_AIR_DATA: {
                _sensors_data.ext.ambient_air_data = udd.ext_ambient_air_data;
                message_length = sizeof(udd.ext_ambient_air_data);
                break;
            }
            case DataType::EXT_WIND_DATA: {
                _sensors_data.ext.wind_data = udd.ext_wind_data;
                message_length = sizeof(udd.ext_wind_data);
                break;
            }
            case DataType::MAG_CLB_ACCURACY: {
                _sensors_data.ins.mag_clb_accuracy = udd.mag_clb_accuracy;
                message_length = sizeof(udd.mag_clb_accuracy);
                break;
            }
            case DataType::DOPPLER_VELOCITY_LOG: {
                _sensors_data.ext.doppler_velocity_log = udd.doppler_velocity_log;
                message_length = sizeof(udd.doppler_velocity_log);
                break;
            }
            default: {
                _diagnostic_data.udd_parse_fail_count++;
                // GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ILAB: Unknown message type: %d. UDD format incorrect!",
                //     message_type);
                move_message_header_to_buffer_start();
                return false;
            }
        }

        message_data_offset += message_length;

        const uint8_t *data_end = payload + payload_size;
        if (message_data_offset > data_end) {
            _diagnostic_data.udd_parse_fail_count++;
            move_message_header_to_buffer_start();
            return false;
        }
    }

    if(!has_udd_required_data_types()) {
        _diagnostic_data.udd_parse_fail_count++;
        move_message_header_to_buffer_start();
        return false;
    }

    return true;
}

bool Sensor::has_udd_required_data_types() const
{
    static constexpr DataType required_types[] {
        DataType::GPS_INS_TIME_MS,
        DataType::ACCEL_DATA_HR,
        DataType::GYRO_DATA_HR,
        DataType::UNIT_STATUS,
        DataType::TEMPERATURE,
        DataType::INS_SOLUTION_STATUS,
    };

    for (const DataType type : required_types) {
        if (!_sensors_data.udd_data_types_list.get(static_cast<uint16_t>(type))) {
            return false;
        }
    }

    return true;
}

} // namespace InertialLabs

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
