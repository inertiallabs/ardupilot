/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for serial connected InertialLabs INS system
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include "AP_ExternalAHRS_InertialLabs_message_list.h"

class AP_ExternalAHRS_InertialLabs : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_InertialLabs(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(class GCS_MAVLINK &link) const override;
    void write_bytes(const char *bytes, uint8_t len) override;
    bool get_wind_estimation(Vector3f &wind) override;

    // check for new data
    void update() override {
        check_uart();
    }

    // Get model/type name
    const char* get_name() const override {
        return "ILabs";
    }

    enum class MessageType : uint8_t {
        GPS_INS_TIME_MS = 0x01,
        GPS_WEEK = 0x3C,
        ACCEL_DATA_HR = 0x23,
        GYRO_DATA_HR = 0x21,
        BARO_DATA = 0x25,
        MAG_DATA = 0x24,
        ORIENTATION_ANGLES = 0x07,
        VELOCITIES = 0x12,
        POSITION = 0x10,
        UNIT_STATUS = 0x53,
        GNSS_EXTENDED_INFO = 0x4A,
        GNSS_POSITION = 0x30,
        GNSS_VEL_TRACK = 0x32,
        GNSS_POS_TIMESTAMP = 0x3E,
        GNSS_NEW_DATA = 0x41,
        GNSS_JAM_STATUS = 0xC0,
        DIFFERENTIAL_PRESSURE = 0x28,
        TRUE_AIRSPEED = 0x86,
        WIND_SPEED = 0x8A,
        AIR_DATA_STATUS = 0x8D,
        SUPPLY_VOLTAGE = 0x50,
        TEMPERATURE = 0x52,
        UNIT_STATUS2 = 0x5A,
        GNSS_DOP = 0x42,
        INS_SOLUTION_STATUS = 0x54,
        INS_POS_VEL_ACCURACY = 0x5F,
        FULL_SAT_INFO = 0x37,
        GNSS_VEL_LATENCY = 0x3D,
        GNSS_SOL_STATUS = 0x38,
        NEW_AIDING_DATA = 0x65,
        NEW_AIDING_DATA2 = 0xA1,
        EXT_SPEED = 0x61,
        EXT_HOR_POS = 0x6E,
        EXT_ALT = 0x6C,
        EXT_HEADING = 0x66,
        EXT_AMBIENT_DATA = 0x6B,
        EXT_WIND_DATA = 0x62,
    };

    /*
      packets consist of:
         ILabsHeader
         list of MessageType
         sequence of ILabsData
         checksum
     */
    struct PACKED ILabsHeader {
        uint16_t magic; // 0x55AA
        uint8_t msg_type; // always 1 for INS data
        uint8_t msg_id; // always 0x95
        uint16_t msg_len; // msg_len+2 is total packet length
    };

    struct PACKED vec3_16_t {
        int16_t x,y,z;
        Vector3f tofloat(void) {
            return Vector3f(x,y,z);
        }
    };

    struct PACKED vec3_32_t {
        int32_t x,y,z;
        Vector3f tofloat(void) {
            return Vector3f(x,y,z);
        }
    };

    struct PACKED gnss_extended_info_t {
        uint8_t fix_type;
        uint8_t spoofing_status;
    };

    struct PACKED full_sat_info_t {
        uint8_t SVs;
        uint8_t SolnSVs;
        uint8_t SolnL1SVs;
        uint8_t SolnMultiSVs;
        uint8_t signal_used1;
        uint8_t signal_used2;
        uint8_t GPS_time_status;
        uint8_t ext_sol_status;
    };

    struct PACKED gnss_dop_t {
        uint16_t gdop; // *103
        uint16_t pdop; // *103
        uint16_t hdop; // *103
        uint16_t vdop; // *103
        uint16_t tdop; // *103
    };

    struct PACKED ins_accuracy_t {
        int32_t lat; // m*1000
        int32_t lon; // m*1000
        int32_t alt; // m*1000
        int32_t east_vel; // m/s*1000
        int32_t north_vel; // m/s*1000
        int32_t ver_vel; // m/s*1000
    };

    struct PACKED ext_hor_pos_t {
        int32_t lat; // deg*1.0e7
        int32_t lon; // deg*1.0e7
        uint16_t lat_std; // m*100
        uint16_t lon_std; // m*100
        uint16_t pos_latency; // sec*1000
    };

    struct PACKED ext_alt_t {
        int32_t alt; // m*1000
        uint16_t alt_std; // m*100
    };

    struct PACKED ext_heading_t {
        uint16_t heading; // deg*100
        uint16_t std; // deg*100
        uint16_t latency; // sec*1000
    };

    struct PACKED ext_ambient_data_t {
        int16_t air_temp; // degC*10
        int32_t alt; // m*100
        uint16_t abs_press; // Pa/2
    };

    struct PACKED ext_wind_data_t {
        int16_t n_wind_vel; // kt*100
        int16_t e_wind_vel; // kt*100
        uint16_t n_std_wind; // kt*100
        uint16_t e_std_wind; // kt*100
    };

    union PACKED ILabsData {
        uint32_t gps_time_ms; // ms since start of GPS week
        uint16_t gps_week;
        vec3_32_t accel_data_hr; // g * 1e6
        vec3_32_t gyro_data_hr; // deg/s * 1e5
        struct PACKED {
            uint16_t pressure_pa2; // Pascals/2
            int32_t baro_alt; // meters*100
        } baro_data;
        vec3_16_t mag_data; // nT/10
        struct PACKED {
            uint16_t yaw; // deg*100
            int16_t pitch; // deg*100
            int16_t roll; // deg*100
        } orientation_angles; // 321 euler order?
        vec3_32_t velocity; // m/s * 100
        struct PACKED {
            int32_t lat; // deg*1e7
            int32_t lon; // deg*1e7
            int32_t alt; // m*100, AMSL
        } position;
        uint16_t unit_status; // set ILABS_UNIT_STATUS_*
        gnss_extended_info_t gnss_extended_info;
        struct PACKED {
            int32_t hor_speed; // m/s*100
            uint16_t track_over_ground; // deg*100
            int32_t ver_speed; // m/s*100
        } gnss_vel_track;
        uint32_t gnss_pos_timestamp; // ms
        uint8_t gnss_new_data;
        uint8_t gnss_jam_status;
        int32_t differential_pressure; // mbar*1e4
        int16_t true_airspeed; // m/s*100
        vec3_16_t wind_speed; // m/s*100
        uint16_t air_data_status;
        uint16_t supply_voltage; // V*100
        int16_t temperature; // degC*10
        uint16_t unit_status2;
        gnss_dop_t gnss_dop;
        uint8_t ins_sol_status;
        ins_accuracy_t ins_accuracy;
        full_sat_info_t full_sat_info;
        uint16_t gnss_vel_latency;
        uint8_t gnss_sol_status;
        uint16_t new_aiding_data;
        uint16_t new_aiding_data2;
        int16_t external_speed;
        ext_hor_pos_t ext_hor_pos;
        ext_alt_t ext_alt;
        ext_heading_t ext_heading;
        ext_ambient_data_t ext_ambient_air_data;
        ext_wind_data_t ext_wind_data;
    };

    AP_ExternalAHRS::gps_data_message_t gps_data;
    AP_ExternalAHRS::mag_data_message_t mag_data;
    AP_ExternalAHRS::baro_data_message_t baro_data;
    AP_ExternalAHRS::ins_data_message_t ins_data;
    AP_ExternalAHRS::airspeed_data_message_t airspeed_data;

    uint16_t buffer_ofs;
    uint8_t buffer[512]; // payload is 218 bytes, full message is 264 bytes
    uint8_t tx_buffer[64];

private:
    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    uint32_t baudrate;
    bool setup_complete;

    void update_thread();
    bool check_uart();
    bool check_header(const ILabsHeader *h) const;
    uint16_t get_num_points_to_dec(const uint16_t rate) const;
    void send_EAHRS_status_report(uint16_t &last_state, uint16_t &current_state, const ILStatusMessage* msg_list, const size_t &msg_list_size, uint64_t* last_msg);
    void make_tx_packet(uint8_t *packet) const;

    // re-sync on header bytes
    void re_sync(void);

    static const struct MessageLength {
        MessageType mtype;
        uint8_t length;
    } message_lengths[];

    struct ILAB_SENSORS_DATA {
        Vector3f accel;
        Vector3f gyro;
        Vector3f mag;
        float pressure;
        float diff_press;
        float temperature;
        float supply_voltage;
    };

    struct ILAB_GPS_DATA {
        uint32_t ms_tow;
        uint16_t gps_week;
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        float hor_speed;
        float ver_speed;
        float track_over_ground;
        gnss_dop_t dop;
        uint8_t new_data;
        uint8_t fix_type;
        uint8_t spoof_status;
        uint8_t jam_status;
        full_sat_info_t full_sat_info;
        uint16_t vel_latency;
        uint8_t gnss_sol_status;
    };

    struct ILAB_INS_DATA{
        float yaw;
        float pitch;
        float roll;
        uint32_t ms_tow;
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        Vector3f velocity;
        uint16_t unit_status;
        uint16_t unit_status2;
        uint8_t ins_sol_status;
        ins_accuracy_t ins_accuracy;
        float baro_alt;
        float true_airspeed;
        Vector3f wind_speed;
        float airspeed_sf;
        uint16_t air_data_status;
    };

    struct ILAB_EXT_DATA {
        uint16_t new_aiding_data;
        uint16_t new_aiding_data2;
        int16_t external_speed;
        ext_hor_pos_t ext_hor_pos;
        ext_alt_t ext_alt;
        ext_heading_t ext_heading;
        ext_ambient_data_t ext_ambient_air_data;
        ext_wind_data_t ext_wind_data;
    };

    ILAB_SENSORS_DATA ilab_sensors_data;
    ILAB_GPS_DATA ilab_gps_data;
    ILAB_INS_DATA ilab_ins_data;
    ILAB_EXT_DATA ilab_ext_data;

    ILAB_SENSORS_DATA ilab_sensors_data_avr;
    ILAB_INS_DATA ilab_ins_data_avr;

   struct {
        uint16_t unit_status;
        uint16_t unit_status2;
        uint16_t air_data_status;
        uint8_t spoof_status;
        uint8_t jam_status;
        uint8_t ins_sol_status;
        uint8_t mag_clb_status;
    } last_ins_status;

    uint32_t last_att_ms;
    uint32_t last_vel_ms;
    uint32_t last_pos_ms;
    uint32_t last_gps_ms;

    uint16_t log_counter = 0;
    uint16_t tx_counter = 0;
};

#endif  // AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED

