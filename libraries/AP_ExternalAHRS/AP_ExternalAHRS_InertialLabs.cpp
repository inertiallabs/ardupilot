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

#if AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED

#include "AP_ExternalAHRS_InertialLabs.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Common/Bitmask.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL &hal;

// unit status bits
#define ILABS_UNIT_STATUS_ALIGNMENT_FAIL   0x0001
#define ILABS_UNIT_STATUS_OPERATION_FAIL   0x0002
#define ILABS_UNIT_STATUS_GYRO_FAIL        0x0004
#define ILABS_UNIT_STATUS_ACCEL_FAIL       0x0008
#define ILABS_UNIT_STATUS_MAG_FAIL         0x0010
#define ILABS_UNIT_STATUS_ELECTRONICS_FAIL 0x0020
#define ILABS_UNIT_STATUS_GNSS_FAIL        0x0040
#define ILABS_UNIT_STATUS_RUNTIME_CAL      0x0080
#define ILABS_UNIT_STATUS_VOLTAGE_LOW      0x0100
#define ILABS_UNIT_STATUS_VOLTAGE_HIGH     0x0200
#define ILABS_UNIT_STATUS_X_RATE_HIGH      0x0400
#define ILABS_UNIT_STATUS_Y_RATE_HIGH      0x0800
#define ILABS_UNIT_STATUS_Z_RATE_HIGH      0x1000
#define ILABS_UNIT_STATUS_MAG_FIELD_HIGH   0x2000
#define ILABS_UNIT_STATUS_TEMP_RANGE_ERR   0x4000
#define ILABS_UNIT_STATUS_RUNTIME_CAL2     0x8000

// unit status2 bits
#define ILABS_UNIT_STATUS2_ACCEL_X_HIGH           0x0001
#define ILABS_UNIT_STATUS2_ACCEL_Y_HIGH           0x0002
#define ILABS_UNIT_STATUS2_ACCEL_Z_HIGH           0x0004
#define ILABS_UNIT_STATUS2_BARO_FAIL              0x0008
#define ILABS_UNIT_STATUS2_DIFF_PRESS_FAIL        0x0010
#define ILABS_UNIT_STATUS2_MAGCAL_2D_ACT          0x0020
#define ILABS_UNIT_STATUS2_MAGCAL_3D_ACT          0x0040
#define ILABS_UNIT_STATUS2_GNSS_FUSION_OFF        0x0080
#define ILABS_UNIT_STATUS2_DIFF_PRESS_FUSION_OFF  0x0100
#define ILABS_UNIT_STATUS2_MAG_FUSION_OFF         0x0200
#define ILABS_UNIT_STATUS2_GNSS_POS_VALID         0x0400

// air data status bits
#define ILABS_AIRDATA_INIT_FAIL                   0x0001
#define ILABS_AIRDATA_DIFF_PRESS_INIT_FAIL        0x0002
#define ILABS_AIRDATA_STATIC_PRESS_FAIL           0x0004
#define ILABS_AIRDATA_DIFF_PRESS_FAIL             0x0008
#define ILABS_AIRDATA_STATIC_PRESS_RANGE_ERR      0x0010
#define ILABS_AIRDATA_DIFF_PRESS_RANGE_ERR        0x0020
#define ILABS_AIRDATA_PRESS_ALT_FAIL              0x0100
#define ILABS_AIRDATA_AIRSPEED_FAIL               0x0200
#define ILABS_AIRDATA_BELOW_THRESHOLD             0x0400

// acceleration due to gravity in m/s/s used in IL INS
#define IL_GRAVITY_MSS     9.8106f

// constructor
AP_ExternalAHRS_InertialLabs::AP_ExternalAHRS_InertialLabs(AP_ExternalAHRS *_frontend,
                                                           AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "InertialLabs ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    // don't offer IMU by default, at 200Hz it is too slow for many aircraft
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));
    
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_InertialLabs::update_thread, void), "ILabs", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("InertialLabs Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs ExternalAHRS initialised");
}

/*
  re-sync buffer on parse failure
 */
void AP_ExternalAHRS_InertialLabs::re_sync(void)
{
    if (buffer_ofs > 3) {
        /*
          look for the 2 byte header and try to sync to that
         */
        const uint16_t header = 0x55AA;
        const uint8_t *p = (const uint8_t *)memmem(&buffer[1], buffer_ofs-3, &header, sizeof(header));
        if (p != nullptr) {
            const uint16_t n = p - &buffer[0];
            memmove(&buffer[0], p, buffer_ofs - n);
            buffer_ofs -= n;
        } else {
            buffer_ofs = 0;
        }
    } else {
        buffer_ofs = 0;
    }
}

// macro for checking we don't run past end of message buffer
#define CHECK_SIZE(d) need_re_sync = (message_ofs + (msg_len=sizeof(d)) > buffer_end); if (need_re_sync) break

// lookup a message in the msg_types bitmask to see if we received it in this packet
#define GOT_MSG(mtype) msg_types.get(unsigned(MessageType::mtype))

/*
  check header is valid
 */
bool AP_ExternalAHRS_InertialLabs::check_header(const ILabsHeader *h) const
{
    return h->magic == 0x55AA &&
        h->msg_type == 1 &&
        h->msg_id == 0x95 &&
        h->msg_len <= sizeof(buffer)-2;
}

/*
  check the UART for more data
  returns true if we have consumed potentially valid bytes
 */
bool AP_ExternalAHRS_InertialLabs::check_uart()
{
    WITH_SEMAPHORE(state.sem);

    if (!setup_complete) {
        return false;
    }
    // ensure we own the uart
    uart->begin(0);
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }
    if (n + buffer_ofs > sizeof(buffer)) {
        n = sizeof(buffer) - buffer_ofs;
    }
    const ILabsHeader *h = (ILabsHeader *)&buffer[0];

    if (buffer_ofs < sizeof(ILabsHeader)) {
        n = MIN(n, sizeof(ILabsHeader)-buffer_ofs);
    } else {
        if (!check_header(h)) {
            re_sync();
            return false;
        }
        if (buffer_ofs > h->msg_len+8) {
            re_sync();
            return false;
        }
        n = MIN(n, uint32_t(h->msg_len + 2 - buffer_ofs));
    }

    const ssize_t nread = uart->read(&buffer[buffer_ofs], n);
    if (nread != ssize_t(n)) {
        re_sync();
        return false;
    }
    buffer_ofs += n;

    if (buffer_ofs < sizeof(ILabsHeader)) {
        return true;
    }

    if (!check_header(h)) {
        re_sync();
        return false;
    }

    if (buffer_ofs < h->msg_len+2) {
        /*
          see if we can read the rest immediately
         */
        const uint16_t needed = h->msg_len+2 - buffer_ofs;
        if (uart->available() < needed) {
            // need more data
            return true;
        }
        const ssize_t nread2 = uart->read(&buffer[buffer_ofs], needed);
        if (nread2 != needed) {
            re_sync();
            return false;
        }
        buffer_ofs += nread2;
    }

    // check checksum
    const uint16_t crc1 = crc_sum_of_bytes_16(&buffer[2], buffer_ofs-4);
    const uint16_t crc2 = le16toh_ptr(&buffer[buffer_ofs-2]);
    if (crc1 != crc2) {
        re_sync();
        return false;
    }

    const uint8_t *buffer_end = &buffer[buffer_ofs];
    const uint16_t payload_size = h->msg_len - 6;
    const uint8_t *payload = &buffer[6];
    if (payload_size < 3) {
        re_sync();
        return false;
    }
    const uint8_t num_messages = payload[0];
    if (num_messages == 0 ||
        num_messages > payload_size-1) {
        re_sync();
        return false;
    }
    const uint8_t *message_ofs = &payload[num_messages+1];
    bool need_re_sync = false;

    // bitmask for what types we get
    Bitmask<256> msg_types;
    uint32_t now_ms = AP_HAL::millis();

    for (uint8_t i=0; i<num_messages; i++) {
        if (message_ofs >= buffer_end) {
            re_sync();
            return false;
        }
        MessageType mtype = (MessageType)payload[1+i];
        ILabsData &u = *(ILabsData*)message_ofs;
        uint8_t msg_len = 0;

        msg_types.set(unsigned(mtype));

        switch (mtype) {
            case MessageType::GPS_INS_TIME_MS: {
                // this is the GPS tow timestamp in ms for when the IMU data was sampled
                CHECK_SIZE(u.gps_time_ms);
                ilab_ins_data.ms_tow = u.gps_time_ms;
                break;
            }
            case MessageType::GPS_WEEK: {
                CHECK_SIZE(u.gps_week);
                ilab_gps_data.gps_week = u.gps_week;
                break;
            }
            case MessageType::ACCEL_DATA_HR: {
                CHECK_SIZE(u.accel_data_hr);
                ilab_sensors_data.accel = u.accel_data_hr.tofloat().rfu_to_frd()*IL_GRAVITY_MSS*1.0e-6f; // NED, in m/s/s
                break;
            }
            case MessageType::GYRO_DATA_HR: {
                CHECK_SIZE(u.gyro_data_hr);
                ilab_sensors_data.gyro = u.gyro_data_hr.tofloat().rfu_to_frd()*DEG_TO_RAD*1.0e-5f; // NED, in rad
                break;
            }
            case MessageType::BARO_DATA: {
                CHECK_SIZE(u.baro_data);
                ilab_sensors_data.pressure = static_cast<float>(u.baro_data.pressure_pa2)*2.0f;
                ilab_ins_data.baro_alt = static_cast<float>(u.baro_data.baro_alt)*0.01f; // m
                break;
            }
            case MessageType::MAG_DATA: {
                CHECK_SIZE(u.mag_data);
                ilab_sensors_data.mag = u.mag_data.tofloat().rfu_to_frd()*(10.0f*NTESLA_TO_MGAUSS); // NED, in milligauss
                break;
            }
            case MessageType::ORIENTATION_ANGLES: {
                CHECK_SIZE(u.orientation_angles);
                ilab_ins_data.yaw = static_cast<float>(u.orientation_angles.yaw)*0.01f; // deg
                ilab_ins_data.pitch = static_cast<float>(u.orientation_angles.pitch)*0.01f; // deg
                ilab_ins_data.roll = static_cast<float>(u.orientation_angles.roll)*0.01f; // deg
                break;
            }
            case MessageType::VELOCITIES: {
                CHECK_SIZE(u.velocity);
                ilab_ins_data.velocity = u.velocity.tofloat().rfu_to_frd()*0.01f; // NED, in m/s
                break;
            }
            case MessageType::POSITION: {
                CHECK_SIZE(u.position);
                ilab_ins_data.latitude = u.position.lat; // deg*1.0e7
                ilab_ins_data.longitude = u.position.lon; // deg*1.0e7
                ilab_ins_data.altitude = u.position.alt; // cm
                break;
            }
            case MessageType::UNIT_STATUS: {
                CHECK_SIZE(u.unit_status);
                ilab_ins_data.unit_status = u.unit_status;
                break;
            }
            case MessageType::GNSS_EXTENDED_INFO: {
                CHECK_SIZE(u.gnss_extended_info);
                ilab_gps_data.fix_type = u.gnss_extended_info.fix_type;
                ilab_gps_data.spoof_status = u.gnss_extended_info.spoofing_status;
                break;
            }
            case MessageType::GNSS_POSITION: {
                CHECK_SIZE(u.position);
                ilab_gps_data.latitude = u.position.lat;
                ilab_gps_data.longitude = u.position.lon;
                ilab_gps_data.altitude = u.position.alt;
                break;
            }
            case MessageType::GNSS_VEL_TRACK: {
                CHECK_SIZE(u.gnss_vel_track);
                ilab_gps_data.hor_speed = static_cast<float>(u.gnss_vel_track.hor_speed)*0.01f; // m/s
                ilab_gps_data.track_over_ground = static_cast<float>(u.gnss_vel_track.track_over_ground)*0.01f; // deg
                ilab_gps_data.ver_speed = static_cast<float>(u.gnss_vel_track.ver_speed)*0.01f; // m/s
                break;
            }
            case MessageType::GNSS_POS_TIMESTAMP: {
                CHECK_SIZE(u.gnss_pos_timestamp);
                ilab_gps_data.ms_tow = u.gnss_pos_timestamp;
                break;
            }
            case MessageType::GNSS_NEW_DATA: {
                CHECK_SIZE(u.gnss_new_data);
                ilab_gps_data.new_data = u.gnss_new_data;
                break;
            }
            case MessageType::GNSS_JAM_STATUS: {
                CHECK_SIZE(u.gnss_jam_status);
                ilab_gps_data.jam_status = u.gnss_jam_status;
                break;
            }
            case MessageType::DIFFERENTIAL_PRESSURE: {
                CHECK_SIZE(u.differential_pressure);
                ilab_sensors_data.diff_press = static_cast<float>(u.differential_pressure)*0.01f; // Pa
                break;
            }
            case MessageType::TRUE_AIRSPEED: {
                CHECK_SIZE(u.true_airspeed);
                ilab_ins_data.true_airspeed = static_cast<float>(u.true_airspeed)*0.01f;
                break;
            }
            case MessageType::WIND_SPEED: {
                CHECK_SIZE(u.wind_speed);
                ilab_ins_data.airspeed_sf = u.wind_speed.tofloat().z*1.0e-3f;
                ilab_ins_data.wind_speed = u.wind_speed.tofloat().rfu_to_frd()*0.01f; // NED, in m/s
                ilab_ins_data.wind_speed.z = 0.0f;
                break;
            }
            case MessageType::AIR_DATA_STATUS: {
                CHECK_SIZE(u.air_data_status);
                ilab_ins_data.air_data_status = u.air_data_status;
                break;
            }
            case MessageType::SUPPLY_VOLTAGE: {
                CHECK_SIZE(u.supply_voltage);
                ilab_sensors_data.supply_voltage = static_cast<float>(u.supply_voltage)*0.01f;
                break;
            }
            case MessageType::TEMPERATURE: {
                CHECK_SIZE(u.temperature);
                ilab_sensors_data.temperature = static_cast<float>(u.temperature)*0.1f;
                break;
            }
            case MessageType::UNIT_STATUS2: {
                CHECK_SIZE(u.unit_status2);
                ilab_ins_data.unit_status2 = u.unit_status2;
                break;
            }
            case MessageType::GNSS_DOP: {
                CHECK_SIZE(u.gnss_dop);
                ilab_gps_data.dop.gdop = u.gnss_dop.gdop;
                ilab_gps_data.dop.pdop = u.gnss_dop.pdop;
                ilab_gps_data.dop.hdop = u.gnss_dop.hdop;
                ilab_gps_data.dop.vdop = u.gnss_dop.vdop;
                ilab_gps_data.dop.tdop = u.gnss_dop.tdop;
                break;
            }
            case MessageType::INS_SOLUTION_STATUS: {
                CHECK_SIZE(u.ins_sol_status);
                ilab_ins_data.ins_sol_status = u.ins_sol_status;
                break;
            }
            case MessageType::INS_POS_VEL_ACCURACY: {
                CHECK_SIZE(u.ins_accuracy);
                ilab_ins_data.ins_accuracy.lat = u.ins_accuracy.lat;
                ilab_ins_data.ins_accuracy.lon = u.ins_accuracy.lon;
                ilab_ins_data.ins_accuracy.alt = u.ins_accuracy.alt;
                ilab_ins_data.ins_accuracy.east_vel = u.ins_accuracy.east_vel;
                ilab_ins_data.ins_accuracy.north_vel = u.ins_accuracy.north_vel;
                ilab_ins_data.ins_accuracy.ver_vel = u.ins_accuracy.ver_vel;
                break;
            }
            case MessageType::FULL_SAT_INFO: {
                CHECK_SIZE(u.full_sat_info);
                ilab_gps_data.full_sat_info.SVs = u.full_sat_info.SVs;
                ilab_gps_data.full_sat_info.SolnSVs = u.full_sat_info.SolnSVs;
                ilab_gps_data.full_sat_info.SolnL1SVs = u.full_sat_info.SolnL1SVs;
                ilab_gps_data.full_sat_info.SolnMultiSVs = u.full_sat_info.SolnMultiSVs;
                ilab_gps_data.full_sat_info.signal_used1 = u.full_sat_info.signal_used1;
                ilab_gps_data.full_sat_info.signal_used2 = u.full_sat_info.signal_used2;
                ilab_gps_data.full_sat_info.GPS_time_status = u.full_sat_info.GPS_time_status;
                ilab_gps_data.full_sat_info.ext_sol_status = u.full_sat_info.ext_sol_status;
                break;
            }
            case MessageType::GNSS_VEL_LATENCY: {
                CHECK_SIZE(u.gnss_vel_latency);
                ilab_gps_data.vel_latency = u.gnss_vel_latency;
                break;
            }
            case MessageType::GNSS_SOL_STATUS: {
                CHECK_SIZE(u.gnss_sol_status);
                ilab_gps_data.gnss_sol_status = u.gnss_sol_status;
                break;
            }
            case MessageType::NEW_AIDING_DATA: {
                CHECK_SIZE(u.new_aiding_data);
                ilab_ext_data.new_aiding_data = u.new_aiding_data;
                break;
            }
            case MessageType::NEW_AIDING_DATA2: {
                CHECK_SIZE(u.new_aiding_data2);
                ilab_ext_data.new_aiding_data2 = u.new_aiding_data2;
                break;
            }
            case MessageType::EXT_SPEED: {
                CHECK_SIZE(u.external_speed);
                ilab_ext_data.external_speed = u.external_speed;
                break;
            }
            case MessageType::EXT_HOR_POS: {
                CHECK_SIZE(u.ext_hor_pos);
                ilab_ext_data.ext_hor_pos = u.ext_hor_pos;
                break;
            }
            case MessageType::EXT_ALT: {
                CHECK_SIZE(u.ext_alt);
                ilab_ext_data.ext_alt = u.ext_alt;
                break;
            }
            case MessageType::EXT_HEADING: {
                CHECK_SIZE(u.ext_heading);
                ilab_ext_data.ext_heading = u.ext_heading;
                break;
            }
            case MessageType::EXT_AMBIENT_DATA: {
                CHECK_SIZE(u.ext_ambient_air_data);
                ilab_ext_data.ext_ambient_air_data = u.ext_ambient_air_data;
                break;
            }
            case MessageType::EXT_WIND_DATA: {
                CHECK_SIZE(u.ext_wind_data);
                ilab_ext_data.ext_wind_data = u.ext_wind_data;
                break;
            }
        }

        if (msg_len == 0) {
            // got an unknown message
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: unknown msg 0x%02x", unsigned(mtype));
            buffer_ofs = 0;
            return false;
        }
        message_ofs += msg_len;

        if (msg_len == 0 || need_re_sync) {
            re_sync();
            return false;
        }
    }

    if (h->msg_len != message_ofs-buffer) {
        // we had stray bytes at the end of the message
        re_sync();
        return false;
    }

    if (GOT_MSG(ORIENTATION_ANGLES)) {
        // use IL INS attitude data in the ArduPilot algorithm
        state.quat.from_euler(radians(ilab_ins_data.roll),
                              radians(ilab_ins_data.pitch),
                              radians(ilab_ins_data.yaw));
        state.have_quaternion = true;
        if (last_att_ms == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: got link");
        }

        last_att_ms = now_ms;
    }

    if (GOT_MSG(ACCEL_DATA_HR) &&
        GOT_MSG(GYRO_DATA_HR)) {
        // use IL INS IMU outputs in the ArduPilot algorithm instead of EKF3 or DCM
        ins_data.accel = ilab_sensors_data.accel;
        ins_data.gyro = ilab_sensors_data.gyro;
        AP::ins().handle_external(ins_data);
        state.accel = ins_data.accel;
        state.gyro = ins_data.gyro;
    }
    if (GOT_MSG(POSITION) &&
        GOT_MSG(VELOCITIES)) {
        state.location.lat = ilab_ins_data.latitude;
        state.location.lng = ilab_ins_data.longitude;
        state.location.alt = ilab_ins_data.altitude;
        state.velocity = ilab_ins_data.velocity;
        state.have_velocity = true;
        state.have_location = true;
        state.last_location_update_us = AP_HAL::micros();

        last_vel_ms = now_ms;
        last_pos_ms = now_ms;

        if (GOT_MSG(GPS_INS_TIME_MS) &&
            GOT_MSG(FULL_SAT_INFO) &&
            GOT_MSG(GNSS_NEW_DATA) &&
            GOT_MSG(GNSS_EXTENDED_INFO) &&
            ilab_gps_data.new_data != 0) {
            // use IL INS navigation solution instead of GNSS solution
            gps_data.ms_tow = ilab_ins_data.ms_tow;
            gps_data.longitude = ilab_ins_data.latitude;
            gps_data.latitude = ilab_ins_data.longitude;
            gps_data.msl_altitude = ilab_ins_data.altitude;
            gps_data.ned_vel_north = ilab_ins_data.velocity.x;
            gps_data.ned_vel_east = ilab_ins_data.velocity.y;
            gps_data.ned_vel_down = ilab_ins_data.velocity.z;
            gps_data.satellites_in_view = ilab_gps_data.full_sat_info.SolnSVs;
            gps_data.hdop = static_cast<float>(ilab_gps_data.dop.hdop)*0.1f;
            gps_data.vdop = static_cast<float>(ilab_gps_data.dop.vdop)*0.1f;
            gps_data.fix_type = ilab_gps_data.fix_type+1;

            uint8_t instance;
            if (AP::gps().get_first_external_instance(instance)) {
                AP::gps().handle_external(gps_data, instance);
            }
            if (gps_data.satellites_in_view > 3) {
                if (last_gps_ms == 0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: got GPS lock");
                    if (!state.have_origin) {
                        state.origin = Location{
                            gps_data.latitude,
                            gps_data.longitude,
                            gps_data.msl_altitude,
                            Location::AltFrame::ABSOLUTE};
                        state.have_origin = true;
                    }
                }
                last_gps_ms = now_ms;
            }
        }
    }
#if AP_BARO_EXTERNALAHRS_ENABLED
    if (GOT_MSG(BARO_DATA) &&
        GOT_MSG(TEMPERATURE)) {
        baro_data.pressure_pa = ilab_sensors_data.pressure;
        baro_data.temperature = ilab_sensors_data.temperature;
        AP::baro().handle_external(baro_data);
    }
#endif
    #if AP_COMPASS_EXTERNALAHRS_ENABLED
    if (GOT_MSG(MAG_DATA)) {
        mag_data.field = ilab_sensors_data.mag;
        AP::compass().handle_external(mag_data);
    }
#endif
#if AP_AIRSPEED_EXTERNAL_ENABLED && (APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane))
    // only on plane and copter as others do not link AP_Airspeed
    if (GOT_MSG(DIFFERENTIAL_PRESSURE) &&
        GOT_MSG(TEMPERATURE)) {
        airspeed_data.differential_pressure = ilab_sensors_data.diff_press;
        airspeed_data.temperature = ilab_sensors_data.temperature;
        auto *arsp = AP::airspeed();
        if (arsp != nullptr) {
            arsp->handle_external(airspeed_data);
        }
    }
#endif // AP_AIRSPEED_EXTERNAL_ENABLED
    buffer_ofs = 0;

#if HAL_LOGGING_ENABLED
    if (GOT_MSG(POSITION) &&
        GOT_MSG(ORIENTATION_ANGLES) &&
        GOT_MSG(VELOCITIES)) {

        float roll, pitch, yaw;
        state.quat.to_euler(roll, pitch, yaw);
        uint64_t now_us = AP_HAL::micros64();

        // @LoggerMessage: ILB2
        // @Description: InertialLabs AHRS data3
        // @Field: TimeUS: Time since system startup
        // @Field: Stat1: unit status1
        // @Field: Stat2: unit status2
        // @Field: FType: fix type
        // @Field: SpStat: spoofing status
        // @Field: GI1: GNSS Info1
        // @Field: GI2: GNSS Info2
        // @Field: GJS: GNSS jamming status
        // @Field: TAS: true airspeed
        // @Field: WVN: Wind velocity north
        // @Field: WVE: Wind velocity east
        // @Field: WVD: Wind velocity down

        AP::logger().WriteStreaming("ILB2", "TimeUS,Stat1,Stat2,FType,SpStat,GI1,GI2,GJS,TAS,WVN,WVE,WVD",
                                    "s---------",
                                    "F---------",
                                    "QHHBBBffff",
                                    now_us,
                                    ilab_ins_data.unit_status, ilab_ins_data.unit_status2,
                                    ilab_gps_data.fix_type, ilab_gps_data.spoof_status,
                                    ilab_gps_data.jam_status,
                                    ilab_ins_data.true_airspeed,
                                    ilab_ins_data.wind_speed.x, ilab_ins_data.wind_speed.y, ilab_ins_data.airspeed_sf);
    }
#endif  // HAL_LOGGING_ENABLED

    return true;
}

void AP_ExternalAHRS_InertialLabs::update_thread()
{
    // Open port in the thread
    uart->begin(baudrate, 1024, 512);

    /*
      we assume the user has already configured the device
     */

    setup_complete = true;
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay_microseconds(250);
        }
    }
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_InertialLabs::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS_InertialLabs::healthy(void) const
{
    WITH_SEMAPHORE(state.sem);
    return AP_HAL::millis() - last_att_ms < 100;
}

bool AP_ExternalAHRS_InertialLabs::initialised(void) const
{
    if (!setup_complete) {
        return false;
    }
    return true;
}

bool AP_ExternalAHRS_InertialLabs::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs unhealthy");
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    uint32_t now = AP_HAL::millis();
    if (now - last_att_ms > 10 ||
        now - last_pos_ms > 10 ||
        now - last_vel_ms > 10) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs not up to date");
        return false;
    }
    return true;
}

/*
  get filter status. We don't know the meaning of the status bits yet,
  so assume all OK if we have GPS lock
 */
void AP_ExternalAHRS_InertialLabs::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);
    uint32_t now = AP_HAL::millis();
    const uint32_t dt_limit = 200;
    const uint32_t dt_limit_gps = 500;
    memset(&status, 0, sizeof(status));
    const bool init_ok = (ilab_ins_data.unit_status & (ILABS_UNIT_STATUS_ALIGNMENT_FAIL|ILABS_UNIT_STATUS_OPERATION_FAIL))==0;
    status.flags.initalized = init_ok;
    status.flags.attitude = init_ok && (now - last_att_ms < dt_limit) && init_ok;
    status.flags.vert_vel = init_ok && (now - last_vel_ms < dt_limit);
    status.flags.vert_pos = init_ok && (now - last_pos_ms < dt_limit);
    status.flags.horiz_vel = status.flags.vert_vel;
    status.flags.horiz_pos_abs = status.flags.vert_pos;
    status.flags.horiz_pos_rel = status.flags.horiz_pos_abs;
    status.flags.pred_horiz_pos_rel = status.flags.horiz_pos_abs;
    status.flags.pred_horiz_pos_abs = status.flags.horiz_pos_abs;
    status.flags.using_gps = (now - last_gps_ms < dt_limit_gps) &&
        (ilab_ins_data.unit_status & (ILABS_UNIT_STATUS_GNSS_FAIL|ILABS_UNIT_STATUS2_GNSS_FUSION_OFF)) == 0;
    status.flags.gps_quality_good = (now - last_gps_ms < dt_limit_gps) &&
        (ilab_ins_data.unit_status2 & ILABS_UNIT_STATUS2_GNSS_POS_VALID) != 0 &&
        (ilab_ins_data.unit_status & ILABS_UNIT_STATUS_GNSS_FAIL) == 0;
    status.flags.rejecting_airspeed = (ilab_ins_data.air_data_status & ILABS_AIRDATA_AIRSPEED_FAIL);
}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS_InertialLabs::send_status_report(GCS_MAVLINK &link) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    const float vel_var = 0;
    const float pos_var = 0;
    const float hgt_var = 0;
    const float mag_var = 0;
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags, vel_var, pos_var, hgt_var, mag_var, 0, 0);
}

#endif  // AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED

