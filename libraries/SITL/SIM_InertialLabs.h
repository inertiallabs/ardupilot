//usage:
//PARAMS:
// param set AHRS_EKF_TYPE 11
// param set EAHRS_TYPE 5
// param set SERIAL4_PROTOCOL 36
// param set SERIAL4_BAUD 460800
// sim_vehicle.py -v ArduPlane -D --console --map -A --serial4=sim:ILabs
#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL
{

class InertialLabs : public SerialDevice
{
public:

    InertialLabs();

    // update state
    void update(void);

private:
    void send_packet(void);

    struct PACKED vec3_16_t {
        int16_t x,y,z;
    };

    struct PACKED vec3_32_t {
        int32_t x,y,z;
    };

    struct PACKED ILabsPacket {
        uint16_t magic = 0x55AA;
        uint8_t msg_type = 0x01;
        uint8_t msg_id = 0x95;
        uint16_t msg_len; // total packet length-2

        // send Table4, 41 messages
        uint8_t num_messages = 41;
        uint8_t messages[41] = {
            0x01, 0x3C, 0x23, 0x21, 0x25, 0x24, 0x26, 0x07, 0x12, 0x10, 0x53, 0x4A, 0x30, 0x32, 0x3E,
            0x41, 0xC0, 0x28, 0x86, 0x85, 0x8A, 0x8D, 0x50, 0x52, 0x5A, 0x42, 0x54, 0x5F, 0x37, 0x3D,
            0x38, 0x39, 0x65, 0xA1, 0x61, 0x6E, 0x6C, 0x66, 0x6B, 0x62, 0x9A
            };

        uint32_t gps_time_ms;
        uint16_t gps_week;
        vec3_32_t accel_data_hr;
        vec3_32_t gyro_data_hr;
        struct PACKED {
            uint16_t pressure_pa2;
            int32_t baro_alt;
        } baro_data;
        vec3_16_t mag_data;
        struct PACKED {
            int8_t gyroX;
            int8_t gyroY;
            int8_t gyroZ;
            int8_t accX;
            int8_t accY;
            int8_t accZ;
            int8_t reserved;
        } sensor_bias;
        struct PACKED {
            uint16_t yaw;
            int16_t pitch;
            int16_t roll;
        } orientation_angles;
        vec3_32_t velocity;
        struct PACKED {
            int32_t lat;
            int32_t lon;
            int32_t alt;
        } position;
        uint16_t unit_status;
        struct PACKED {
            uint8_t fix_type;
            uint8_t spoofing_status;
        } gnss_extended_info;
        struct PACKED {
            int32_t lat;
            int32_t lon;
            int32_t alt;
        } gnss_position;
        struct PACKED {
            int32_t hor_speed;
            uint16_t track_over_ground;
            int32_t ver_speed;
        } gnss_vel_track;
        uint32_t gnss_pos_timestamp;
        uint8_t gnss_new_data;
        uint8_t gnss_jam_status;
        int32_t differential_pressure;
        int16_t true_airspeed;
        int16_t calibrated_airspeed;
        vec3_16_t wind_speed;
        uint16_t air_data_status;
        uint16_t supply_voltage;
        int16_t temperature;
        uint16_t unit_status2;
        struct PACKED {
            uint16_t gdop;
            uint16_t pdop;
            uint16_t hdop;
            uint16_t vdop;
            uint16_t tdop;
        } gnss_dop;
        uint8_t ins_sol_status;
        struct PACKED {
            int32_t lat;
            int32_t lon;
            int32_t alt;
            int32_t east_vel;
            int32_t north_vel;
            int32_t ver_vel;
        } ins_accuracy;
        struct PACKED {
            uint8_t SVs;
            uint8_t SolnSVs;
            uint8_t SolnL1SVs;
            uint8_t SolnMultiSVs;
            uint8_t signal_used1;
            uint8_t signal_used2;
            uint8_t GPS_time_status;
            uint8_t ext_sol_status;
        } full_sat_info;
        uint16_t gnss_vel_latency;
        uint8_t gnss_sol_status;
        uint8_t gnss_pos_vel_type;
        uint16_t new_aiding_data;
        uint16_t new_aiding_data2;
        int16_t external_speed;
        struct PACKED {
            int32_t lat;
            int32_t lon;
            uint16_t lat_std;
            uint16_t lon_std;
            uint16_t pos_latency;
        } ext_hor_pos;
        struct PACKED {
            int32_t alt;
            uint16_t alt_std;
        } ext_alt;
        struct PACKED {
            uint16_t heading;
            uint16_t std;
            uint16_t latency;
        } ext_heading;
        struct PACKED {
            int16_t air_temp;
            int32_t alt;
            uint16_t abs_press;
        } ext_ambient_data;
        struct PACKED {
            int16_t n_wind_vel;
            int16_t e_wind_vel;
            uint16_t n_std_wind;
            uint16_t e_std_wind;
        } ext_wind_data;
        uint8_t mag_clb_accuracy;
        uint16_t crc;
    } pkt;

    uint32_t last_pkt_us;
    const uint16_t pkt_rate_hz = 200;
    const uint16_t gps_rate_hz = 10;
    const uint16_t gps_frequency = pkt_rate_hz / gps_rate_hz;
    uint32_t packets_sent;
};

}