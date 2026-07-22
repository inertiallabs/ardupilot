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
  support for serial connected InertialLabs INS
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/Bitmask.h>
#include <AP_Common/ExpandingString.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "AP_ExternalAHRS_InertialLabs.h"
#include "InertialLabs_logs.h"

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_InertialLabs::AP_ExternalAHRS_InertialLabs(AP_ExternalAHRS *_frontend,
                                                           AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    // don't offer IMU by default, at 200Hz it is too slow for many aircraft
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_InertialLabs::update_thread, void), "ILabs", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("InertialLabs Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs ExternalAHRS initialized");
}

int8_t AP_ExternalAHRS_InertialLabs::get_port() const
{
    return sensor.get_port();
};

bool AP_ExternalAHRS_InertialLabs::healthy() const
{
    WITH_SEMAPHORE(state.sem);
    return AP_HAL::millis() - handled_sensor_data.attitude_timestamp < 100;
}

bool AP_ExternalAHRS_InertialLabs::initialised() const
{
    return sensor.is_initialized();
}

bool AP_ExternalAHRS_InertialLabs::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!sensor.is_initialized()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs unhealthy");
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    uint32_t now = AP_HAL::millis();
    const uint32_t dt_limit = 10;
    if (now - handled_sensor_data.attitude_timestamp > dt_limit ||
        now - handled_sensor_data.pos_timestamp > dt_limit ||
        now - handled_sensor_data.vel_timestamp > dt_limit) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs not up to date");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_InertialLabs::get_filter_status(nav_filter_status &status) const
{
    // We don't know the meaning of the status bits yet, so assume all OK if we have GPS lock
    using InertialLabs::USW;

    WITH_SEMAPHORE(state.sem);

    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();

    uint32_t now = AP_HAL::millis();
    const uint32_t dt_limit = 200;
    const uint32_t dt_limit_gps = 500;
    memset(&status, 0, sizeof(status));

    const bool init_ok = (sensors_data.ins.unit_status & (USW::INITIAL_ALIGNMENT_FAIL|USW::OPERATION_FAIL)) == 0;

    status.flags.initalized = init_ok;

    status.flags.attitude = init_ok && (now - handled_sensor_data.attitude_timestamp < dt_limit);
    status.flags.vert_vel = init_ok && (now - handled_sensor_data.vel_timestamp < dt_limit);
    status.flags.vert_pos = init_ok && (now - handled_sensor_data.pos_timestamp < dt_limit);
    status.flags.horiz_vel = status.flags.vert_vel;
    status.flags.horiz_pos_abs = status.flags.vert_pos;
    status.flags.horiz_pos_rel = status.flags.vert_pos;
    status.flags.pred_horiz_pos_rel = status.flags.vert_pos;
    status.flags.pred_horiz_pos_abs = status.flags.vert_pos;

    status.flags.using_gps = (now - handled_sensor_data.gps_timestamp < dt_limit_gps);
    status.flags.gps_quality_good = (now - handled_sensor_data.gps_timestamp < dt_limit_gps);

    status.flags.rejecting_airspeed = false;
}

bool AP_ExternalAHRS_InertialLabs::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();

    velVar = sensors_data.ins.kf_vel_covariance.length() * 1.0e-3f * vel_gate_scale;      // m/s
    posVar = sensors_data.ins.kf_pos_covariance.xy().length() * 1.0e-3f * pos_gate_scale; // m
    hgtVar = sensors_data.ins.kf_pos_covariance.z * 1.0e-3f * hgt_gate_scale;             // m
    magVar.zero();
    tasVar = 0;
    return true;
}

void AP_ExternalAHRS_InertialLabs::write_bytes(const char *bytes, uint8_t len)
{
    sensor.write_bytes(bytes, len);
}

void AP_ExternalAHRS_InertialLabs::handle_command(ExternalAHRS_command command, const ExternalAHRS_command_data &data)
{
    sender.send_sensor_command(sensor, command, data);
}

bool AP_ExternalAHRS_InertialLabs::get_wind_estimation(Vector3f &wind)
{
    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();
    wind = sensors_data.ins.wind_speed;
    return true;
}

void AP_ExternalAHRS_InertialLabs::send_eahrs_status_flag(GCS_MAVLINK &link) const
{
    sender.send_gcs_eahrs_status_flags(link, sensor.get_sensors_data());
}

void AP_ExternalAHRS_InertialLabs::format_status(class ExpandingString &str)
{
    const InertialLabs::SensorDiagnosticData & sensor_diagnostic_data = sensor.get_diagnostic_data();
    str.printf("Inertial Labs EAHRS status\n");
    str.printf("Checksum fail count:                       %llu\n", (unsigned long long)sensor_diagnostic_data.checksum_fail_count);
    str.printf("UDD-format fail count:                     %llu\n", (unsigned long long)sensor_diagnostic_data.udd_parse_fail_count);
    str.printf("Send data to sensor fail count:            %llu\n", (unsigned long long)sensor_diagnostic_data.uart_write_fail_count);
    str.printf("Good package count:                        %llu\n", (unsigned long long)driver_diagnostic_data.good_package_count);

    const uint64_t avg_duration_between_good_packages_us = driver_diagnostic_data.good_package_count > 1 ?
        static_cast<uint64_t>(driver_diagnostic_data.summary_duration_between_good_packages_us / (driver_diagnostic_data.good_package_count - 1))
        : 0;
    str.printf("Average duration between good packages (us): %llu\n", (unsigned long long)avg_duration_between_good_packages_us);
    str.printf("Last duration between good packages (us):    %llu\n", (unsigned long long)driver_diagnostic_data.last_duration_between_good_packages_us);

    const uint64_t avg_good_package_handle_duration_us = driver_diagnostic_data.good_package_count ?
        static_cast<uint64_t>(driver_diagnostic_data.summary_good_package_handle_duration_us / driver_diagnostic_data.good_package_count)
        : 0;

    str.printf("Average good package handle duration (us): %llu\n", (unsigned long long)avg_good_package_handle_duration_us);
    str.printf("Last good package handle duration (us):    %llu\n", (unsigned long long)driver_diagnostic_data.last_good_package_handle_duration_us);
}

void AP_ExternalAHRS_InertialLabs::update()
{
    // A separate thread already processes the data in a loop.
    // Don't call handle_full_circle() here. It may cause a race condition.
}

InertialLabs::DataReadStatus AP_ExternalAHRS_InertialLabs::handle_full_circle()
{
    if (!sensor.is_initialized()) {
        return InertialLabs::DataReadStatus::NEED_WAIT;
    }

    WITH_SEMAPHORE(state.sem);

    const uint64_t start_time_us = AP_HAL::micros64();
    InertialLabs::DataReadStatus res = sensor.update_data();
    if (res != InertialLabs::DataReadStatus::SUCCESS) {
        return res;
    }

    handle_sensor_data();
    send_data_to_sensor();
    write_logs(sensor.get_sensors_data());

    const bool need_send = option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_SEND_STATUS);
    if (need_send)
    {
        sender.send_gcs_messages(sensor.get_sensors_data());
    }

    const uint64_t finish_time_us = AP_HAL::micros64();

    driver_diagnostic_data.good_package_count++;
    driver_diagnostic_data.last_good_package_handle_duration_us = finish_time_us - start_time_us;
    driver_diagnostic_data.summary_good_package_handle_duration_us += driver_diagnostic_data.last_good_package_handle_duration_us;

    if (driver_diagnostic_data.last_good_package_handle_timestamp_us > 0) {
        driver_diagnostic_data.last_duration_between_good_packages_us = finish_time_us - driver_diagnostic_data.last_good_package_handle_timestamp_us;
        driver_diagnostic_data.summary_duration_between_good_packages_us += driver_diagnostic_data.last_duration_between_good_packages_us;
    }
    driver_diagnostic_data.last_good_package_handle_timestamp_us = finish_time_us;
    return InertialLabs::DataReadStatus::SUCCESS;
}

void AP_ExternalAHRS_InertialLabs::update_thread()
{
    if (!sensor.init()) {
        AP_HAL::panic("InertialLabs Failed to initialize sensor");
    }

    while (true) {
        if(handle_full_circle() == InertialLabs::DataReadStatus::NEED_WAIT) {
            hal.scheduler->delay_microseconds(250);
        }
    }
}

// Check has UDD message type in the received data
#define GOT_MSG(msg) sensors_data.udd_data_types_list.get(static_cast<uint16_t>(InertialLabs::DataType::msg))

void AP_ExternalAHRS_InertialLabs::handle_sensor_data()
{
    using InertialLabs::ADU;
    using InertialLabs::InsSolution;
    using InertialLabs::NewGPSData;
    using InertialLabs::USW;
    using InertialLabs::USW2;

    const InertialLabs::SensorsData &sensors_data = sensor.get_sensors_data();

    const bool filter_ok = (sensors_data.ins.unit_status & USW::INITIAL_ALIGNMENT_FAIL) == 0 &&
                           (sensors_data.ins.ins_sol_status != InsSolution::INVALID);

    const uint32_t package_timestamp_ms = static_cast<uint32_t>(sensors_data.package_timestamp_us / 1000);
    if (filter_ok && GOT_MSG(ORIENTATION_ANGLES)) {
        // use IL INS attitude data in the ArduPilot algorithm instead of EKF3 or DCM
        state.quat.from_euler(static_cast<float>(radians(sensors_data.ins.roll)),
                              static_cast<float>(radians(sensors_data.ins.pitch)),
                              static_cast<float>(radians(sensors_data.ins.yaw)));
        state.have_quaternion = true;
        handled_sensor_data.attitude_timestamp = package_timestamp_ms;
    }

    if (filter_ok && (sensors_data.ins.unit_status & (USW::GYRO_FAIL|USW::ACCEL_FAIL)) == 0) {
        // use IL INS IMU outputs in the ArduPilot algorithm
        state.accel = sensors_data.accel;
        state.gyro = sensors_data.gyro;
        ins_data.accel = sensors_data.accel;
        ins_data.gyro = sensors_data.gyro;
        ins_data.temperature = sensors_data.temperature;
        AP::ins().handle_external(ins_data);
    }

    const bool hasNewGpsData = (sensors_data.gps.new_data & (NewGPSData::NEW_GNSS_POSITION|NewGPSData::NEW_GNSS_VELOCITY)) != 0; // true if received new GNSS position or velocity

    if (filter_ok && GOT_MSG(POSITION) && GOT_MSG(VELOCITIES)) {
        // use IL INS navigation solution instead of EKF3 or DCM
        state.location.lat = sensors_data.ins.latitude;
        state.location.lng = sensors_data.ins.longitude;
        state.location.alt = sensors_data.ins.altitude;
        state.velocity = sensors_data.ins.velocity;
        state.have_velocity = true;
        state.have_location = true;
        state.last_location_update_us = AP_HAL::micros();

        handled_sensor_data.vel_timestamp = package_timestamp_ms;
        handled_sensor_data.pos_timestamp = package_timestamp_ms;

        if (GOT_MSG(INS_POS_VEL_ACCURACY)) {
            gps_data.ins_lat_accuracy = static_cast<uint32_t>(sensors_data.ins.ins_accuracy.lat);
            gps_data.ins_lng_accuracy = static_cast<uint32_t>(sensors_data.ins.ins_accuracy.lon);
            gps_data.ins_alt_accuracy = static_cast<uint32_t>(sensors_data.ins.ins_accuracy.alt);
        }

        if (hasNewGpsData &&
            GOT_MSG(UNIT_STATUS2) &&
            GOT_MSG(FULL_SAT_INFO) &&
            GOT_MSG(GNSS_POSITION) &&
            GOT_MSG(GNSS_NEW_DATA) &&
            GOT_MSG(GNSS_EXTENDED_INFO) &&
            GOT_MSG(GPS_WEEK) &&
            GOT_MSG(GNSS_VEL_TRACK) &&
            GOT_MSG(GNSS_SOL_STATUS)) {
            // use IL INS navigation solution instead of GNSS solution
            gps_data.ms_tow = sensors_data.ins.ms_tow;
            gps_data.gps_week = sensors_data.gps.gps_week;
            gps_data.latitude = sensors_data.ins.latitude;
            gps_data.longitude = sensors_data.ins.longitude;
            gps_data.msl_altitude = sensors_data.ins.altitude;
            gps_data.ned_vel_north = sensors_data.ins.velocity.x;
            gps_data.ned_vel_east = sensors_data.ins.velocity.y;
            gps_data.ned_vel_down = sensors_data.ins.velocity.z;

            const bool gps_sol_trick = option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_DISABLE_GPS_TRICK);
            const bool gps_solution = ((sensors_data.ins.unit_status2 & USW2::GNSS_FUSION_OFF) == 0) &&
                                      (sensors_data.gps.gnss_sol_status == InsSolution::GOOD) &&
                                      (sensors_data.gps.fix_type == 2);
            if (gps_sol_trick || gps_solution) { // use valid GNSS data as is
                gps_data.fix_type = AP_GPS_FixType(sensors_data.gps.fix_type + 1);
                gps_data.satellites_in_view = sensors_data.gps.full_sat_info.SolnSVs;
                // use GNSS DOP = 90.0f (0.9) by default if no UDD message
                gps_data.hdop = GOT_MSG(GNSS_DOP) ? static_cast<float>(sensors_data.gps.dop.hdop)*0.1f : 90.0f;
                gps_data.vdop = GOT_MSG(GNSS_DOP) ? static_cast<float>(sensors_data.gps.dop.vdop)*0.1f : 90.0f;
            } else { // set fixed values to continue normal flight in GNSS-denied environments
                gps_data.fix_type = AP_GPS_FixType::FIX_3D;
                gps_data.satellites_in_view = 77;
                gps_data.hdop = 90.0f; // 0.9
                gps_data.vdop = 90.0f; // 0.9
            }

            gps_data.latitude_raw = sensors_data.gps.latitude;
            gps_data.longitude_raw = sensors_data.gps.longitude;
            gps_data.altitude_raw = sensors_data.gps.altitude;
            gps_data.track_over_ground_raw = static_cast<int32_t>(sensors_data.gps.track_over_ground*100.0f);
            gps_data.gps_raw_status = sensors_data.gps.gnss_sol_status;

            uint8_t instance{0};
            if (AP::gps().get_first_external_instance(instance)) {
                AP::gps().handle_external(gps_data, instance);
            }
            if (gps_data.satellites_in_view > 3) {
                if (handled_sensor_data.gps_timestamp == 0) {
                    if (!state.have_origin) {
                        state.origin = Location{
                            gps_data.latitude,
                            gps_data.longitude,
                            gps_data.msl_altitude,
                            Location::AltFrame::ABSOLUTE};
                        state.have_origin = true;
                    }
                }
                handled_sensor_data.gps_timestamp = package_timestamp_ms;
            }
        }
    }

#if AP_BARO_EXTERNALAHRS_ENABLED
    if (GOT_MSG(BARO_DATA) && GOT_MSG(UNIT_STATUS2) &&
        (sensors_data.ins.unit_status2 & USW2::ADU_BARO_FAIL) == 0) {
        // use IL INS barometer output in the ArduPilot algorithm
        baro_data.pressure_pa = sensors_data.pressure;
        baro_data.temperature = sensors_data.temperature;
        AP::baro().handle_external(baro_data);
    }
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    if (GOT_MSG(MAG_DATA) &&
        (sensors_data.ins.unit_status & USW::MAG_FAIL) == 0) {
        // use IL INS magnetometer outputs in the ArduPilot algorithm
        mag_data.field = sensors_data.mag;
        AP::compass().handle_external(mag_data);
    }
#endif

#if AP_AIRSPEED_EXTERNAL_ENABLED && (APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane))
    // only on plane and copter as others do not link AP_Airspeed
    if (GOT_MSG(DIFFERENTIAL_PRESSURE) && GOT_MSG(TRUE_AIRSPEED) && GOT_MSG(UNIT_STATUS2) &&
        (sensors_data.ins.unit_status2 & USW2::ADU_DIFF_PRESS_FAIL) == 0) {
        airspeed_data.differential_pressure = sensors_data.diff_press;
        airspeed_data.temperature = sensors_data.temperature;
        airspeed_data.airspeed = sensors_data.ins.true_airspeed;
        auto *arsp = AP::airspeed();
        if (arsp != nullptr) {
            if (option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_USE_AIRSPEED)) {
                // use IL INS calculated true airspeed
                bool airspeed_enabled = false;
                if (filter_ok && GOT_MSG(AIR_DATA_STATUS) && (sensors_data.ins.air_data_status & ADU::AIRSPEED_FAIL) == 0) {
                    airspeed_enabled = true;
                }
                arsp->set_external_airspeed_enabled(airspeed_enabled);
            }
            arsp->handle_external(airspeed_data);
        }
    }
#endif // AP_AIRSPEED_EXTERNAL_ENABLED
}

#undef GOT_MSG

void AP_ExternalAHRS_InertialLabs::send_data_to_sensor()
{
    const bool transmit_airspeed = option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_TRANSMIT_AIRSPEED);
    if (transmit_airspeed) {
        const uint16_t inu_data_rate = get_rate(); // Hz
        const uint16_t max_aiding_data_rate = 50; // Hz
        uint16_t ticks_for_one_send = (inu_data_rate / max_aiding_data_rate);
        if (inu_data_rate % max_aiding_data_rate)
        {
            ++ticks_for_one_send;
        }

        if (handled_sensor_data.airspeed_message_counter < ticks_for_one_send)
        {
            ++handled_sensor_data.airspeed_message_counter;
        }
        else
        {
            sender.send_sensor_airspeed_aiding_data(sensor);
            handled_sensor_data.airspeed_message_counter = 0;
        }
    }
}

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED