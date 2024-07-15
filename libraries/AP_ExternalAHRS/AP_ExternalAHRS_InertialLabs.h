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
    void handle_command(ExternalAHRS_command command, const ExternalAHRS_command_data &data) override;
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
        GPS_INS_TIME_MS         = 0x01,
        GPS_WEEK                = 0x3C,
        ACCEL_DATA_HR           = 0x23,
        GYRO_DATA_HR            = 0x21,
        BARO_DATA               = 0x25,
        MAG_DATA                = 0x24,
        ORIENTATION_ANGLES      = 0x07,
        VELOCITIES              = 0x12,
        POSITION                = 0x10,
        UNIT_STATUS             = 0x53,
        GNSS_EXTENDED_INFO      = 0x4A,
        NUM_SATS                = 0x3B,
        GNSS_POSITION           = 0x30,
        GNSS_VEL_TRACK          = 0x32,
        GNSS_POS_TIMESTAMP      = 0x3E,
        GNSS_INFO_SHORT         = 0x36,
        GNSS_NEW_DATA           = 0x41,
        GNSS_JAM_STATUS         = 0xC0,
        DIFFERENTIAL_PRESSURE   = 0x28,
        TRUE_AIRSPEED           = 0x86,
        WIND_SPEED              = 0x8A,
        AIR_DATA_STATUS         = 0x8D,
        SUPPLY_VOLTAGE          = 0x50,
        TEMPERATURE             = 0x52,
        UNIT_STATUS2            = 0x5A,
        GNSS_DOP                = 0x42,
        INS_SOLUTION_STATUS     = 0x54,
        NEW_AIDING_DATA         = 0x65,
        EXT_POS                 = 0x63,
        EXT_HOR_POS             = 0x6E,
        EXT_ALT                 = 0x6C,
        EXT_HEADING             = 0x66,
        EXT_AMBIENT_DATA        = 0x6B,
        EXT_WIND_DATA           = 0x62,
        EXT_ADC_DATA            = 0xA2,
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

    struct PACKED EXT_POS {
        int32_t lat; // deg*1.0e7
        int32_t lon; // deg*1.0e7
        int32_t alt; // m*1000
        uint16_t lat_std; // m*100
        uint16_t lon_std; // m*100
        uint16_t alt_std; // m*100
        uint16_t pos_latency; // sec*1000
    };

    struct PACKED EXT_HOR_POS {
        int32_t lat; // deg*1.0e7
        int32_t lon; // deg*1.0e7
        uint16_t lat_std; // m*100
        uint16_t lon_std; // m*100
        uint16_t pos_latency; // sec*1000
    };

    struct PACKED EXT_ALT {
        int32_t alt; // m*1000
        uint16_t alt_std; // m*100
    };

    struct PACKED EXT_HEADING {
        uint16_t heading; // deg*100
        uint16_t std; // deg*100
        uint16_t latency; // sec*1000
    };

    struct PACKED EXT_AMBIENT_DATA {
        int16_t air_temp; // degC*10
        int32_t alt; // m*100
        uint16_t abs_press; // Pa/2
    };

    struct PACKED EXT_WIND_DATA {
        int16_t n_wind_vel; // kt*100
        int16_t e_wind_vel; // kt*100
        uint16_t n_std_wind; // kt*100
        uint16_t e_std_wind; // kt*100
    };

    struct PACKED EXT_ADC_DATA {
        uint32_t static_press; // Pa
        int32_t diff_press; // Pa*10
    };

    union PACKED ILabsData {
        uint32_t gps_ins_time_ms; // ms since start of GPS week, rounded to 1000/(output data rate)
        uint16_t gps_week;
        vec3_32_t accel_data_hr; // g * 1e6
        vec3_32_t gyro_data_hr; // deg/s * 1e5
        struct PACKED {
            uint16_t pressure; // Pascals/2
            int32_t baro_alt; // meters*100
        } baro_data;
        vec3_16_t mag_data; // nT/10
        struct PACKED {
            uint16_t yaw; // deg*100
            int16_t pitch; // deg*100
            int16_t roll; // deg*100
        } orientation_angles;
        vec3_32_t velocity; // East-North-Vertical, m/s*100
        struct PACKED {
            int32_t lat; // deg*1e7
            int32_t lng; // deg*1e7
            int32_t alt; // m*100
        } position;
        uint16_t unit_status; // unit status word (USW)
        struct PACKED {
            uint8_t fix_type; // GPSfix
            uint8_t gnss_spoof_status; // spoofing detect status
        } gnss_extended_info;
        uint8_t num_sats;
        struct PACKED {
            int32_t lat; // deg*1e7
            int32_t lng; // deg*1e7
            int32_t alt; // m*100
        } gnss_position;
        struct PACKED {
            int32_t hor_speed; // m/s*100
            uint16_t track_over_ground; // deg*100
            int32_t ver_speed; // m/s*100
        } gnss_vel_track;
        uint32_t gnss_pos_timestamp; // ms
        struct PACKED {
            uint8_t info1; // position type, pseudorange iono correction
            uint8_t info2; // solution status, time status, GNSS constellations in use
        } gnss_info_short;
        uint8_t gnss_new_data; // indicator of new update of GPS data
        uint8_t gnss_jam_status; // jamming detect status
        int32_t differential_pressure; // mbar*1e4
        int16_t true_airspeed; // m/s*100
        vec3_16_t wind_speed; // East-North-Reserved, m/s*100
        uint16_t air_data_status; // ADU
        uint16_t supply_voltage; // V*100
        int16_t temperature; // degC*10
        uint16_t unit_status2; // unit status word (USW2)
        struct PACKED {
            uint16_t gdop; // geometric
            uint16_t pdop; // position
            uint16_t hdop; // horizontal
            uint16_t vdop; // vertical
            uint16_t tdop; // time
        } gnss_dop; // DOP, *10e3
        uint8_t nav_sol_status; // INS solution status
        uint16_t new_aiding_data; // new aiding data indicator
        EXT_POS ext_pos; // external position
        EXT_HOR_POS ext_hor_pos; // external horizontal position
        EXT_ALT ext_alt; // external altitude
        EXT_HEADING ext_heading; // external heading
        EXT_AMBIENT_DATA ext_ambient_air_data; // external pressure, temperature, altitude
        EXT_WIND_DATA ext_wind_data; // external wind speed
        EXT_ADC_DATA ext_ADC_data; // external static and dynamic pressure
    };

    AP_ExternalAHRS::gps_data_message_t ardu_gps_data;
    AP_ExternalAHRS::ins_data_message_t ardu_imu_data;
    AP_ExternalAHRS::mag_data_message_t ardu_mag_data;
    AP_ExternalAHRS::baro_data_message_t ardu_baro_data;
    AP_ExternalAHRS::airspeed_data_message_t ardu_airspeed_data;

    uint16_t buffer_ofs;
    uint8_t buffer[256]; // max for normal message set is 167+8
    uint8_t tx_count = 0; // RX count/4 20ms 50hz
    uint8_t tx_buffer[64]; // TX buffer for send to IL INS

private:
    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    uint32_t baudrate;
    bool setup_complete;

    void update_thread();
    bool check_uart();
    bool check_header(const ILabsHeader *h) const;
    void make_tx_packet(uint8_t *packet) const; // create a TX packet to transmit static and diff. pressure to IL INS via UART
    uint8_t get_num_points_to_average() const; // get number of points to average the INS data for logging
    void send_EAHRS_status_report(uint16_t &last_state, uint16_t &current_state, const ILStatusMessage* msg_list, const size_t &msg_list_size, uint64_t* last_msg); // send INS status to GCS via MAVLink

    // re-sync on header bytes
    void re_sync(void);

    static const struct MessageLength {
        MessageType mtype;
        uint8_t length;
    } message_lengths[];

struct SENSORS_DATA {
        Vector3f accel;
        Vector3f gyro;
        Vector3f mag;
        float pressure;
        float diff_press;
        float temperature;
        float supply_voltage;
    };

    struct ATT_DATA {
        float yaw;
        float pitch;
        float roll;
    };

    struct INS_DATA {
        uint32_t ms_tow;
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        Vector3f velocity;
        uint16_t unit_status;
        uint16_t unit_status2;
        uint8_t nav_sol_status;
    };

    struct GNSS_DATA {
        uint32_t ms_tow;
        uint16_t gps_week;
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        float hor_speed;
        float ver_speed;
        float track_over_ground;
        float gdop;
        float pdop;
        float hdop;
        float vdop;
        float tdop;
        uint8_t new_data;
        uint8_t num_sat;
        uint8_t fix_type;
        uint8_t spoof_status;
        uint8_t jam_status;
        uint8_t info1;
        uint8_t info2;
    };

    struct AIR_DATA {
        float baro_alt;
        float true_airspeed;
        Vector3f wind_speed;
        uint16_t air_data_status;
    };

    struct INS_EXT_DATA {
        uint16_t new_aiding_data;
        EXT_POS ext_pos;
        EXT_HOR_POS ext_hor_pos;
        EXT_ALT ext_alt;
        EXT_HEADING ext_heading;
        EXT_AMBIENT_DATA ext_ambient_air_data;
        EXT_WIND_DATA ext_wind_data;
        EXT_ADC_DATA ext_ADC_data;
    };

    SENSORS_DATA sensors_data;
    ATT_DATA att_data;
    INS_DATA ins_data;
    GNSS_DATA gnss_data;
    AIR_DATA air_data;
    INS_EXT_DATA ins_ext_data;

    SENSORS_DATA sensors_data_log;
    ATT_DATA att_data_log;
    INS_DATA ins_data_log;
    AIR_DATA air_data_log;

    struct {
        float latitude;
        float longitude;
        float altitude;
        float latitude_std;
        float longitude_std;
        float altitude_std;
        float pos_latency;
    } ins_ext_pos_data_log;

   struct {
        uint16_t unit_status;
        uint16_t unit_status2;
        uint16_t adu_status;
        uint8_t spoof_status;
        uint8_t jam_status;
        uint8_t nav_sol_status;
    } last_ins_status;

    uint8_t n_avr;
    uint8_t counter;

    uint32_t last_att_msg_ms;
    uint32_t last_ins_msg_ms;
    uint32_t last_gps_msg_ms;
};

#endif  // AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED
