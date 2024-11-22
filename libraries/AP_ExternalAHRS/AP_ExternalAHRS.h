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
  support for serial connected AHRS systems
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if HAL_EXTERNAL_AHRS_ENABLED

#include "AP_ExternalAHRS_command_context.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <AP_NavEKF/AP_Nav_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS_backend;

class AP_ExternalAHRS {

public:
    friend class AP_ExternalAHRS_backend;
    friend class AP_ExternalAHRS_VectorNav;

    AP_ExternalAHRS();

    void init(void);

    static const struct AP_Param::GroupInfo var_info[];

    enum class DevType : uint8_t {
        None   = 0,
#if AP_EXTERNAL_AHRS_VECTORNAV_ENABLED
        VecNav = 1,
#endif
#if AP_EXTERNAL_AHRS_MICROSTRAIN5_ENABLED
        MicroStrain5 = 2,
#endif
#if AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED
        InertialLabs = 5,
#endif
        // 3 reserved for AdNav
        // 4 reserved for CINS
        // 6 reserved for Trimble
#if AP_EXTERNAL_AHRS_MICROSTRAIN7_ENABLED
        MicroStrain7 = 7,
#endif
        // 8 reserved for SBG
        // 9 reserved for EulerNav
        // 10 reserved for Aeron
    };

    static AP_ExternalAHRS *get_singleton(void) {
        return _singleton;
    }

    // expected IMU rate in Hz
    float get_IMU_rate(void) const {
        return rate.get();
    }

    // get rate in Hz to log external AHRS data
    float get_log_rate(void) const {
        return log_rate.get();
    }

    // Get model/type name
    const char* get_name() const;

    enum class AvailableSensor {
        GPS = (1U<<0),
        IMU = (1U<<1),
        BARO = (1U<<2),
        COMPASS = (1U<<3),
    };

    // get serial port number, -1 for not enabled
    int8_t get_port(AvailableSensor sensor) const;

    struct state_t {
        HAL_Semaphore sem;

        Vector3f accel;
        Vector3f gyro;
        Quaternion quat; // NED
        Location location;
        Vector3f velocity;
        Location origin;

        bool have_quaternion;
        bool have_origin;
        bool have_location;
        bool have_velocity;

        uint32_t last_location_update_us;
    } state;

    // accessors for AP_AHRS
    bool enabled() const;
    bool healthy(void) const;
    bool initialised(void) const;
    bool get_quaternion(Quaternion &quat);
    bool get_origin(Location &loc);
    bool get_location(Location &loc);
    Vector2f get_groundspeed_vector();
    bool get_velocity_NED(Vector3f &vel);
    bool get_speed_down(float &speedD);
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;
    void get_filter_status(nav_filter_status &status) const;
    bool get_gyro(Vector3f &gyro);
    bool get_accel(Vector3f &accel);
    void send_status_report(class GCS_MAVLINK &link) const;
    void write_bytes(const char *bytes, uint8_t len);
    void handle_command(ExternalAHRS_command command, const ExternalAHRS_command_data &data);
    bool get_estimate_wind(Vector3f &wind) const;
    void handle_msg(mavlink_channel_t chan, const mavlink_message_t &msg);
    void send_eahrs_status_flag(class GCS_MAVLINK &link) const;

    // update backend
    void update();

    /*
      structures passed to other subsystems
     */
    typedef struct {
        uint8_t instance;
        float pressure_pa;
        float temperature;
    } baro_data_message_t;

    typedef struct {
        Vector3f field;
    } mag_data_message_t;

    typedef struct {
        uint16_t gps_week;                   // GPS week, 0xFFFF if not available
        uint32_t ms_tow;
        uint8_t  fix_type;
        uint8_t  satellites_in_view;
        float horizontal_pos_accuracy;
        float vertical_pos_accuracy;
        float horizontal_vel_accuracy;
        float hdop;
        float vdop;
        int32_t  longitude;
        int32_t  latitude;
        int32_t longitude_raw;
        int32_t latitude_raw;
        int32_t altitude_raw;
        int32_t track_over_ground_raw;
        uint8_t gps_raw_status;
        int32_t  msl_altitude;       // cm
        float  ned_vel_north;
        float  ned_vel_east;
        float  ned_vel_down;
        uint32_t ins_lat_accuracy;
        uint32_t ins_lng_accuracy;
        uint32_t ins_alt_accuracy;
    } gps_data_message_t;

    typedef struct {
        Vector3f accel;
        Vector3f gyro;
        float temperature;
    } ins_data_message_t;

    typedef struct {
        float differential_pressure; // Pa
        float temperature; // degC
        float airspeed; // m/s
    } airspeed_data_message_t;

    // set GNSS disable for auxillary function GPS_DISABLE
    void set_gnss_disable(bool disable) {
        gnss_is_disabled = disable;
    }

    enum class OPTIONS {
        VN_UNCOMP_IMU           = (1U << 0),
        ILAB_TRANSMIT_AIRSPEED  = (1U << 1), // transmit airspeed to IL INS
        ILAB_SEND_STATUS        = (1U << 2), // send IL INS status messages to GCS
        RESERVED                = (1U << 3),
        ILAB_USE_AIRSPEED       = (1U << 4), // use IL INS calculated true airspeed in ArduPilot subsystems
        ILAB_DISABLE_GPS_TRICK  = (1U << 5), // disable IL INS GNSS solution (fix type, number of satellites, DOP) substitution in GNSS-denied environments
    };

protected:

    bool option_is_set(OPTIONS option) const { return (options.get() & int32_t(option)) != 0; }

private:
    AP_ExternalAHRS_backend *backend;

    AP_Enum<DevType> devtype;
    AP_Int16         rate;
    AP_Int16         log_rate;
    AP_Int16         options;
    AP_Int16         sensors;

    static AP_ExternalAHRS *_singleton;

    // check if a sensor type is enabled
    bool has_sensor(AvailableSensor sensor) const {
        return (uint16_t(sensors.get()) & uint16_t(sensor)) != 0;
    }

    // set default of EAHRS_SENSORS
    void set_default_sensors(uint16_t _sensors) {
        sensors.set_default(_sensors);
    }

    uint32_t last_log_ms;

    // true when user has disabled the GNSS
    bool gnss_is_disabled;
};

namespace AP {
    AP_ExternalAHRS &externalAHRS();
};

#endif  // HAL_EXTERNAL_AHRS_ENABLED

