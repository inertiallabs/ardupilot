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
#include "AP_ExternalAHRS_InertialLabs_command.h"
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

// acceleration due to gravity in m/s/s used in IL INS
#define IL_GRAVITY_MSS     9.8106f

static const uint64_t dt_critical_msg = 10000; // delay between critical messages to send GCS
static const uint16_t max_aiding_data_rate = 50; // Maximum Aiding data rate in Hz

// initial array of timestamp of IL INS statuses
uint64_t IL_usw_last_msg_ms[sizeof(IL_usw_msg) / sizeof(IL_usw_msg[0])] = {0};
uint64_t IL_usw2_last_msg_ms[sizeof(IL_usw2_msg) / sizeof(IL_usw2_msg[0])] = {0};
uint64_t IL_adu_last_msg_ms[sizeof(IL_adu_msg) / sizeof(IL_adu_msg[0])] = {0};

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
        const uint8_t *p = (const uint8_t *)memmem(&buffer[1], buffer_ofs-2, &header, sizeof(header));
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
        return true;
    }
    // ensure we own the uart
    uart->begin(0);
    uint32_t n = uart->available();
    if (n == 0) {
        return true;
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
        if (buffer_ofs > h->msg_len+2) {
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: packet skipping");
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

    const bool transmit_airspeed = option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_TRANSMIT_AIRSPEED);
    if (transmit_airspeed) {
        uint16_t points_to_decimate = get_num_points_to_dec(max_aiding_data_rate);
        if (tx_counter >= points_to_decimate) {
            make_tx_packet(tx_buffer);
            tx_counter = 0;
        }
        tx_counter++;
    }

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
                ilab_sensors_data.pressure = static_cast<float>(u.baro_data.pressure_pa2)*2.0f; // Pa
                ilab_ins_data.baro_alt = static_cast<float>(u.baro_data.baro_alt)*0.01f; // m
                break;
            }
            case MessageType::MAG_DATA: {
                CHECK_SIZE(u.mag_data);
                ilab_sensors_data.mag = u.mag_data.tofloat().rfu_to_frd()*(10.0f*NTESLA_TO_MGAUSS); // NED, in milligauss
                break;
            }
            case MessageType::SENSOR_BIAS: {
                CHECK_SIZE(u.sensor_bias);
                ilab_ins_data.sensor_bias = u.sensor_bias;
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
                CHECK_SIZE(u.gnss_position);
                ilab_gps_data.latitude = u.gnss_position.lat; // deg*1.0e7
                ilab_gps_data.longitude = u.gnss_position.lon; // deg*1.0e7
                ilab_gps_data.altitude = u.gnss_position.alt; // cm
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
                ilab_ins_data.true_airspeed = static_cast<float>(u.true_airspeed)*0.01f; // m/s
                break;
            }
            case MessageType::CALIBRATED_AIRSPEED: {
                CHECK_SIZE(u.calibrated_airspeed);
                ilab_ins_data.calibrated_airspeed = static_cast<float>(u.calibrated_airspeed)*0.01f; // m/s
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
                ilab_sensors_data.supply_voltage = static_cast<float>(u.supply_voltage)*0.01f; // V
                break;
            }
            case MessageType::TEMPERATURE: {
                CHECK_SIZE(u.temperature);
                ilab_sensors_data.temperature = static_cast<float>(u.temperature)*0.1f; // degC
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

    bool filter_ok = (ilab_ins_data.unit_status & IL_USW::INITIAL_ALIGNMENT_FAIL) == 0 && (ilab_ins_data.ins_sol_status != 8);

    if (filter_ok) {
        // use IL INS attitude data in the ArduPilot algorithm instead of EKF3 or DCM
        state.quat.from_euler(radians(ilab_ins_data.roll),
                              radians(ilab_ins_data.pitch),
                              radians(ilab_ins_data.yaw));
        state.have_quaternion = true;
        if (last_att_ms == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: got link");
        }

        last_att_ms = now_ms;
    }

    if (filter_ok && (ilab_ins_data.unit_status & (IL_USW::GYRO_FAIL|IL_USW::ACCEL_FAIL)) == 0) {
        // use IL INS IMU outputs in the ArduPilot algorithm
        ins_data.accel = ilab_sensors_data.accel;
        ins_data.gyro = ilab_sensors_data.gyro;
        AP::ins().handle_external(ins_data);
        state.accel = ins_data.accel;
        state.gyro = ins_data.gyro;
    }

    bool hasNewGpsData = (ilab_gps_data.new_data & (IL_NEWGPS::NEW_GNSS_POSITION|IL_NEWGPS::NEW_GNSS_VELOCITY)) != 0; // true if received new GNSS position or velocity

    if (filter_ok) {
        // use IL INS navigation solution instead of EKF3 or DCM
        state.location.lat = ilab_ins_data.latitude;
        state.location.lng = ilab_ins_data.longitude;
        state.location.alt = ilab_ins_data.altitude;
        state.velocity = ilab_ins_data.velocity;
        state.have_velocity = true;
        state.have_location = true;
        state.last_location_update_us = AP_HAL::micros();

        last_vel_ms = now_ms;
        last_pos_ms = now_ms;

        gps_data.ins_lat_accuracy = static_cast<uint32_t>(ilab_ins_data.ins_accuracy.lat);
        gps_data.ins_lng_accuracy = static_cast<uint32_t>(ilab_ins_data.ins_accuracy.lon);
        gps_data.ins_alt_accuracy = static_cast<uint32_t>(ilab_ins_data.ins_accuracy.alt);

        if (hasNewGpsData) {
            // use IL INS navigation solution instead of GNSS solution
            gps_data.ms_tow = ilab_ins_data.ms_tow;
            gps_data.gps_week = ilab_gps_data.gps_week;
            gps_data.latitude = ilab_ins_data.latitude;
            gps_data.longitude = ilab_ins_data.longitude;
            gps_data.msl_altitude = ilab_ins_data.altitude;
            gps_data.ned_vel_north = ilab_ins_data.velocity.x;
            gps_data.ned_vel_east = ilab_ins_data.velocity.y;
            gps_data.ned_vel_down = ilab_ins_data.velocity.z;

         if ((ilab_ins_data.unit_status2 & IL_USW2::GNSS_FUSION_OFF) != 0 ||
                (ilab_gps_data.fix_type != 2) ||
                (ilab_gps_data.gnss_sol_status != 0)) {
                gps_data.satellites_in_view = 77;
                gps_data.hdop = 90.0f; // 0.9
                gps_data.vdop = 90.0f; // 0.9
                gps_data.fix_type = 3;
            } else {
                gps_data.satellites_in_view = ilab_gps_data.full_sat_info.SolnSVs;
                gps_data.hdop = static_cast<float>(ilab_gps_data.dop.hdop)*0.1f;
                gps_data.vdop = static_cast<float>(ilab_gps_data.dop.vdop)*0.1f;
                gps_data.fix_type = ilab_gps_data.fix_type+1;
            }

            gps_data.latitude_raw = ilab_gps_data.latitude;
            gps_data.longitude_raw = ilab_gps_data.longitude;
            gps_data.altitude_raw = ilab_gps_data.altitude;
            gps_data.track_over_ground_raw = static_cast<int32_t>(ilab_gps_data.track_over_ground);
            gps_data.gps_raw_status = ilab_gps_data.gnss_sol_status;

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
    if ((ilab_ins_data.unit_status2 & IL_USW2::ADU_BARO_FAIL) == 0) {
        // use IL INS barometer output in the ArduPilot algorithm
        baro_data.pressure_pa = ilab_sensors_data.pressure;
        baro_data.temperature = ilab_sensors_data.temperature;
        AP::baro().handle_external(baro_data);
    }
#endif

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    if ((ilab_ins_data.unit_status & IL_USW::MAG_FAIL) == 0) {
        // use IL INS magnetometer outputs in the ArduPilot algorithm
        mag_data.field = ilab_sensors_data.mag;
        AP::compass().handle_external(mag_data);
    }
#endif

#if AP_AIRSPEED_EXTERNAL_ENABLED && (APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane))
    // only on plane and copter as others do not link AP_Airspeed
    if ((ilab_ins_data.unit_status2 & IL_USW2::ADU_DIFF_PRESS_FAIL) == 0) {
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
    uint64_t now_us = AP_HAL::micros64();
    uint16_t log_rate = get_eahrs_log_rate();
    uint16_t n_avr = get_num_points_to_dec(log_rate);

    ilab_sensors_data_avr.accel += ilab_sensors_data.accel;
    ilab_sensors_data_avr.gyro += ilab_sensors_data.gyro;
    ilab_sensors_data_avr.mag += ilab_sensors_data.mag;
    ilab_sensors_data_avr.pressure += ilab_sensors_data.pressure;
    ilab_sensors_data_avr.diff_press += ilab_sensors_data.diff_press;
    ilab_sensors_data_avr.temperature += ilab_sensors_data.temperature;
    ilab_sensors_data_avr.supply_voltage += ilab_sensors_data.supply_voltage;

    ilab_ins_data_avr.yaw += ilab_ins_data.yaw;
    ilab_ins_data_avr.pitch += ilab_ins_data.pitch;
    ilab_ins_data_avr.roll += ilab_ins_data.roll;

    ilab_ins_data_avr.latitude += ilab_ins_data.latitude;
    ilab_ins_data_avr.longitude += ilab_ins_data.longitude;
    ilab_ins_data_avr.altitude += ilab_ins_data.altitude;
    ilab_ins_data_avr.velocity += ilab_ins_data.velocity;
    ilab_ins_data_avr.unit_status |= ilab_ins_data.unit_status;
    ilab_ins_data_avr.unit_status2 |= ilab_ins_data.unit_status2;
    ilab_ins_data_avr.ins_sol_status |= ilab_ins_data.ins_sol_status;

    ilab_ins_data_avr.baro_alt += ilab_ins_data.baro_alt;
    ilab_ins_data_avr.true_airspeed += ilab_ins_data.true_airspeed;
    ilab_ins_data_avr.calibrated_airspeed += ilab_ins_data.calibrated_airspeed;
    ilab_ins_data_avr.wind_speed += ilab_ins_data.wind_speed;
    ilab_ins_data_avr.airspeed_sf += ilab_ins_data.airspeed_sf;
    ilab_ins_data_avr.air_data_status |= ilab_ins_data.air_data_status;

    log_counter++;

    if (log_counter >= n_avr) {
        ilab_sensors_data_avr.accel /= n_avr;
        ilab_sensors_data_avr.gyro /= n_avr;
        ilab_sensors_data_avr.mag /= n_avr;
        ilab_sensors_data_avr.pressure /= n_avr;
        ilab_sensors_data_avr.diff_press /= n_avr;
        ilab_sensors_data_avr.temperature /= n_avr;
        ilab_sensors_data_avr.supply_voltage /= n_avr;

        ilab_ins_data_avr.yaw /= n_avr;
        ilab_ins_data_avr.pitch /= n_avr;
        ilab_ins_data_avr.roll /= n_avr;

        ilab_ins_data_avr.latitude /= n_avr;
        ilab_ins_data_avr.longitude /= n_avr;
        ilab_ins_data_avr.altitude /= n_avr;
        ilab_ins_data_avr.velocity /= n_avr;

        ilab_ins_data_avr.baro_alt /= n_avr;
        ilab_ins_data_avr.true_airspeed /= n_avr;
        ilab_ins_data_avr.calibrated_airspeed /= n_avr;
        ilab_ins_data_avr.wind_speed /= n_avr;
        ilab_ins_data_avr.airspeed_sf /= n_avr;

        // @LoggerMessage: ILB1
        // @Description: InertialLabs IMU and Mag data
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: GyrX: Gyro X
        // @Field: GyrY: Gyro Y
        // @Field: GyrZ: Gyro z
        // @Field: AccX: Accelerometer X
        // @Field: AccY: Accelerometer Y
        // @Field: AccZ: Accelerometer Z
        // @Field: MagX: Magnetometer X
        // @Field: MagY: Magnetometer Y
        // @Field: MagZ: Magnetometer Z

        AP::logger().WriteStreaming("ILB1", "TimeUS,IMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,MagX,MagY,MagZ",
                                    "s-kkkoooGGG",
                                    "F----------",
                                    "QIfffffffff",
                                    now_us, ilab_ins_data.ms_tow,
                                    ilab_sensors_data_avr.gyro.x, ilab_sensors_data_avr.gyro.y, ilab_sensors_data_avr.gyro.z,
                                    ilab_sensors_data_avr.accel.x, ilab_sensors_data_avr.accel.y, ilab_sensors_data_avr.accel.z,
                                    ilab_sensors_data_avr.mag.x, ilab_sensors_data_avr.mag.y, ilab_sensors_data_avr.mag.z);

        // @LoggerMessage: ILBX
        // @Description: InertialLabs sensors bias data
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: GyrX: Gyro bias X
        // @Field: GyrY: Gyro bias Y
        // @Field: GyrZ: Gyro bias Z
        // @Field: AccX: Accel bias X
        // @Field: AccY: Accel bias Y
        // @Field: AccZ: Accel bias Z

        AP::logger().WriteStreaming("ILBX", "TimeUS,IMS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ",
                                    "s-kkk---",
                                    "F-------",
                                    "QIffffff",
                                    now_us, ilab_ins_data.ms_tow,
                                    static_cast<float>(ilab_ins_data.sensor_bias.gyroX)*2.0f*1.0e-4f,
                                    static_cast<float>(ilab_ins_data.sensor_bias.gyroY)*2.0f*1.0e-4f,
                                    static_cast<float>(ilab_ins_data.sensor_bias.gyroZ)*2.0f*1.0e-4f,
                                    static_cast<float>(ilab_ins_data.sensor_bias.accX)*2.0f*1.0e-5f,
                                    static_cast<float>(ilab_ins_data.sensor_bias.accY)*2.0f*1.0e-5f,
                                    static_cast<float>(ilab_ins_data.sensor_bias.accZ)*2.0f*1.0e-5f);

        // @LoggerMessage: ILB2
        // @Description: InertialLabs ADC data
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: Press: Static pressure
        // @Field: Diff: Differential pressure
        // @Field: Temp: Temperature
        // @Field: Alt: Baro altitude
        // @Field: TAS: true airspeed
        // @Field: CAS: calibrated airspeed
        // @Field: VWN: Wind velocity north
        // @Field: VWE: Wind velocity east
        // @Field: ArspSF: The scale factor (SF) for measured air speed
        // @Field: ADU: Air Data Unit status

        AP::logger().WriteStreaming("ILB2", "TimeUS,IMS,Press,Diff,Temp,Alt,TAS,CAS,VWN,VWE,ArspSF,ADU",
                                    "s-PPOmnnnn--",
                                    "F-----------",
                                    "QIfffffffffH",
                                    now_us, ilab_ins_data.ms_tow,
                                    ilab_sensors_data_avr.pressure, ilab_sensors_data_avr.diff_press, ilab_sensors_data_avr.temperature,
                                    ilab_ins_data_avr.baro_alt, ilab_ins_data_avr.true_airspeed, ilab_ins_data_avr.calibrated_airspeed,
                                    ilab_ins_data_avr.wind_speed.x, ilab_ins_data_avr.wind_speed.y, ilab_ins_data_avr.airspeed_sf,
                                    ilab_ins_data_avr.air_data_status);

        // @LoggerMessage: ILB3
        // @Description: InertialLabs INS data
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: Roll: euler roll
        // @Field: Pitch: euler pitch
        // @Field: Yaw: euler yaw
        // @Field: VN: velocity north
        // @Field: VE: velocity east
        // @Field: VD: velocity down
        // @Field: Lat: latitude
        // @Field: Lng: longitude
        // @Field: Alt: altitude MSL
        // @Field: USW: USW1
        // @Field: USW2: USW2
        // @Field: Vdc: Supply voltage
        // @Field: ISS: INS Navigation (Solution) Status

        AP::logger().WriteStreaming("ILB3", "TimeUS,IMS,Roll,Pitch,Yaw,VN,VE,VD,Lat,Lng,Alt,USW,USW2,Vdc,ISS",
                                    "s-dddnnnDUm--v-",
                                    "F--------------",
                                    "QIfffffffffHHfB",
                                    now_us, ilab_ins_data.ms_tow,
                                    ilab_ins_data_avr.roll, ilab_ins_data_avr.pitch, ilab_ins_data_avr.yaw,
                                    ilab_ins_data_avr.velocity.x, ilab_ins_data_avr.velocity.y, ilab_ins_data_avr.velocity.z,
                                    static_cast<float>(ilab_ins_data_avr.latitude)*1.0e-7f,
                                    static_cast<float>(ilab_ins_data_avr.longitude)*1.0e-7f,
                                    static_cast<float>(ilab_ins_data_avr.altitude)*0.01f,
                                    ilab_ins_data_avr.unit_status, ilab_ins_data_avr.unit_status2,
                                    ilab_sensors_data_avr.supply_voltage, ilab_ins_data_avr.ins_sol_status);

        ilab_sensors_data_avr = {};
        ilab_ins_data_avr = {};
        log_counter = 0;
    }

    if (hasNewGpsData) {
        // @LoggerMessage: ILB4
        // @Description: InertialLabs GPS data1
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: GMS: GNSS Position timestamp
        // @Field: GWk: GPS Week
        // @Field: NewGPS: Indicator of new update of GPS data
        // @Field: Lat: GNSS Latitude
        // @Field: Lng: GNSS Longitude
        // @Field: Alt: GNSS Altitude
        // @Field: GCrs: GNSS Track over ground
        // @Field: Spd: GNSS Horizontal speed
        // @Field: VZ: GNSS Vertical speed

        AP::logger().WriteStreaming("ILB4", "TimeUS,IMS,GMS,GWk,NewGPS,Lat,Lng,Alt,GCrs,Spd,VZ",
                                    "s----DUmhnn",
                                    "F----------",
                                    "QIIHBffffff",
                                    now_us, ilab_ins_data.ms_tow, ilab_gps_data.ms_tow, ilab_gps_data.gps_week, ilab_gps_data.new_data,
                                    static_cast<float>(ilab_gps_data.latitude)*1.0e-7f,
                                    static_cast<float>(ilab_gps_data.longitude)*1.0e-7f,
                                    static_cast<float>(ilab_gps_data.altitude)*0.01f,
                                    ilab_gps_data.track_over_ground, ilab_gps_data.hor_speed, ilab_gps_data.ver_speed);

        // @LoggerMessage: ILB5
        // @Description: InertialLabs GPS data2
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: FType: fix type
        // @Field: GSS: GNSS spoofing status
        // @Field: GJS: GNSS jamming status
        // @Field: VL: GNSS Velocity latency
        // @Field: SolS: GNSS Solution status
        // @Field: GDOP: GNSS GDOP
        // @Field: PDOP: GNSS PDOP
        // @Field: HDOP: GNSS HDOP
        // @Field: VDOP: GNSS VDOP
        // @Field: TDOP: GNSS TDOP

        AP::logger().WriteStreaming("ILB5", "TimeUS,IMS,FType,GSS,GJS,VL,SolS,GDOP,PDOP,HDOP,VDOP,TDOP",
                                    "s-----------",
                                    "F-----------",
                                    "QIBBBHBfffff",
                                    now_us, ilab_ins_data.ms_tow, ilab_gps_data.fix_type,
                                    ilab_gps_data.spoof_status, ilab_gps_data.jam_status, ilab_gps_data.vel_latency, ilab_gps_data.gnss_sol_status,
                                    static_cast<float>(ilab_gps_data.dop.gdop)*1.0e-3f,
                                    static_cast<float>(ilab_gps_data.dop.pdop)*1.0e-3f,
                                    static_cast<float>(ilab_gps_data.dop.hdop)*1.0e-3f,
                                    static_cast<float>(ilab_gps_data.dop.vdop)*1.0e-3f,
                                    static_cast<float>(ilab_gps_data.dop.tdop)*1.0e-3f);

        // @LoggerMessage: ILB6
        // @Description: InertialLabs GPS data3
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: SVs: number of satellites tracked
        // @Field: SolSVs: number of satellites used in solution
        // @Field: SolL1: number of satellites with L1/E1/B1 signals used in solution
        // @Field: SolMult: number of satellites with multi-frequency signals used in solution
        // @Field: SU1: Galileo and BeiDou signal-used mask
        // @Field: SU2: GPS and GLONASS signal-used mask
        // @Field: TimeS: GPS time status
        // @Field: SolS: Extended solution status

        AP::logger().WriteStreaming("ILB6", "TimeUS,IMS,SVs,SolSVs,SolL1,SolMult,SU1,SU2,GTimeS,SolS",
                                    "s---------",
                                    "F---------",
                                    "QIBBBBBBBB",
                                    now_us, ilab_ins_data.ms_tow,
                                    ilab_gps_data.full_sat_info.SVs, ilab_gps_data.full_sat_info.SolnSVs,
                                    ilab_gps_data.full_sat_info.SolnL1SVs, ilab_gps_data.full_sat_info.SolnMultiSVs,
                                    ilab_gps_data.full_sat_info.signal_used1, ilab_gps_data.full_sat_info.signal_used2,
                                    ilab_gps_data.full_sat_info.GPS_time_status, ilab_gps_data.full_sat_info.ext_sol_status);
    }

    if ((ilab_ext_data.new_aiding_data & (IL_NewAidingData::NEW_EXT_POS |
                                          IL_NewAidingData::NEW_EXT_HOR_POS |
                                          IL_NewAidingData::NEW_ALTITUDE |
                                          IL_NewAidingData::NEW_HEADING)) != 0) {
        // @LoggerMessage: ILB7
        // @Description: InertialLabs aiding data1
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: Lat: Latitude external
        // @Field: Lng: Longitude external
        // @Field: Alt: Altitude external
        // @Field: LatS: Latitude external STD
        // @Field: LngS: Longitude external STD
        // @Field: AltS: Altitude external STD
        // @Field: PosL: External position latency
        // @Field: Yaw: Heading external
        // @Field: YawS: Heading external STD
        // @Field: YawL: Heading external latency

        AP::logger().WriteStreaming("ILB7", "TimeUS,IMS,Lat,Lng,Alt,LatS,LngS,AltS,PosL,Yaw,YawS,YawL",
                                    "s-DUmmmm-hh-",
                                    "F-----------",
                                    "QIffffffffff",
                                    now_us, ilab_ins_data.ms_tow,
                                    static_cast<float>(ilab_ext_data.ext_hor_pos.lat)*1.0e-7f,
                                    static_cast<float>(ilab_ext_data.ext_hor_pos.lon)*1.0e-7f,
                                    static_cast<float>(ilab_ext_data.ext_alt.alt)*1.0e-3f,
                                    static_cast<float>(ilab_ext_data.ext_hor_pos.lat_std)*0.01f,
                                    static_cast<float>(ilab_ext_data.ext_hor_pos.lon_std)*0.01f,
                                    static_cast<float>(ilab_ext_data.ext_alt.alt_std)*0.01f,
                                    static_cast<float>(ilab_ext_data.ext_hor_pos.pos_latency)*1.0e-3f,
                                    static_cast<float>(ilab_ext_data.ext_heading.heading)*0.01f,
                                    static_cast<float>(ilab_ext_data.ext_heading.std)*0.01f,
                                    static_cast<float>(ilab_ext_data.ext_heading.latency)*1.0e-3f);
    }

    if ((ilab_ext_data.new_aiding_data & (IL_NewAidingData::NEW_AMBIENT |
                                          IL_NewAidingData::NEW_WIND |
                                          IL_NewAidingData::NEW_AIRSPEED)) != 0) {
        // @LoggerMessage: ILB8
        // @Description: InertialLabs aiding data2
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: Spd: External air or ground speed
        // @Field: Temp: External temperature
        // @Field: Alt: External altitude
        // @Field: Press: External pressure
        // @Field: WN: External North wind component
        // @Field: WE: External East wind component
        // @Field: WNS: External North wind STD
        // @Field: WES: External East wind component

        AP::logger().WriteStreaming("ILB8", "TimeUS,IMS,Spd,Temp,Alt,Press,WN,WE,WNS,WES",
                                    "s-nOmPnnnn",
                                    "F---------",
                                    "QIffffffff",
                                    now_us, ilab_ins_data.ms_tow,
                                    static_cast<float>(ilab_ext_data.external_speed)*0.5144*0.01f,
                                    static_cast<float>(ilab_ext_data.ext_ambient_air_data.air_temp)*0.1f,
                                    static_cast<float>(ilab_ext_data.ext_ambient_air_data.alt)*0.01f,
                                    static_cast<float>(ilab_ext_data.ext_ambient_air_data.abs_press)*2,
                                    static_cast<float>(ilab_ext_data.ext_wind_data.e_wind_vel)*0.5144*0.01f,
                                    static_cast<float>(ilab_ext_data.ext_wind_data.n_wind_vel)*0.5144*0.01f,
                                    static_cast<float>(ilab_ext_data.ext_wind_data.e_std_wind)*0.5144*0.01f,
                                    static_cast<float>(ilab_ext_data.ext_wind_data.n_std_wind)*0.5144*0.01f);
    }

    // @LoggerMessage: ILB9
    // @Description: InertialLabs AHRS data9
    // @Field: TimeUS: Time since system startup
    // @Field: IMS: GPS INS time (round)
    // @Field: NAD1: New Aiding Data
    // @Field: NAD2: New Aiding Data
    // @Field: dLat: Latitude accuracy
    // @Field: dLon: Longitude accuracy
    // @Field: dAlt: Altitude accuracy
    // @Field: dVelE: East velocity accuracy
    // @Field: dVelN: North velocity accuracy
    // @Field: dVelV: Vertical velocity accuracy

    AP::logger().WriteStreaming("ILB9", "TimeUS,IMS,NAD1,NAD2,dLat,dLon,dAlt,dVelE,dVelN,dVelV",
                                "s---mmmnnn",
                                "F---------",
                                "QIHHffffff",
                                now_us, ilab_ins_data.ms_tow, ilab_ext_data.new_aiding_data, ilab_ext_data.new_aiding_data2,
                                static_cast<float>(ilab_ins_data.ins_accuracy.lat)*1.0e-3,
                                static_cast<float>(ilab_ins_data.ins_accuracy.lon)*1.0e-3,
                                static_cast<float>(ilab_ins_data.ins_accuracy.alt)*1.0e-3,
                                static_cast<float>(ilab_ins_data.ins_accuracy.north_vel)*1.0e-3,
                                static_cast<float>(ilab_ins_data.ins_accuracy.east_vel)*1.0e-3,
                                static_cast<float>(ilab_ins_data.ins_accuracy.ver_vel)*1.0e-3);

#endif  // HAL_LOGGING_ENABLED

    const bool send_ilab_status = option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_SEND_STATUS);
    if (send_ilab_status) {
        // Send IL INS status messages to GCS via MAVLink
        if (ilab_ins_data.unit_status != last_ins_status.unit_status) {
            send_EAHRS_status_report(last_ins_status.unit_status, ilab_ins_data.unit_status, IL_usw_msg, IL_usw_msg_size, IL_usw_last_msg_ms); // IL INS Unit Status Word (USW) messages

            if (ilab_ins_data.unit_status & IL_USW::MAG_VG3D_CLB_RUNTIME) {
                if ((last_ins_status.mag_clb_status & (1 << 0)) == 0) {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: On-the-fly calibration data accumulation");
                    last_ins_status.mag_clb_status |= (1 << 0);
                    last_ins_status.mag_clb_status &= ~(1 << 2);
                } else {
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: On-the-fly calibration calculation");
                    last_ins_status.mag_clb_status |= (1 << 1);
                    last_ins_status.mag_clb_status &= ~(1 << 0);
                }
            }

            if (ilab_ins_data.unit_status & IL_USW::MAG_VG3D_CLB_SUCCESS) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: On-the-fly calibration successful");
                last_ins_status.mag_clb_status |= (1 << 2);
                last_ins_status.mag_clb_status &= ~(1 << 1);
            }

            if (!(last_ins_status.unit_status & IL_USW::MAG_VG3D_CLB_RUNTIME) && ((last_ins_status.mag_clb_status & (1 << 1)) != 0)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: On-the-fly calibration unsuccessful");
                last_ins_status.mag_clb_status &= ~(1 << 1);
            }
        }

        if (ilab_ins_data.unit_status2 != last_ins_status.unit_status2) {
            send_EAHRS_status_report(last_ins_status.unit_status2, ilab_ins_data.unit_status2, IL_usw2_msg, IL_usw2_msg_size, IL_usw2_last_msg_ms); // IL INS Unit Status Word 2 (USW2) messages
        }

        if (ilab_ins_data.air_data_status != last_ins_status.air_data_status) {
            send_EAHRS_status_report(last_ins_status.air_data_status, ilab_ins_data.air_data_status, IL_adu_msg, IL_adu_msg_size, IL_adu_last_msg_ms); // IL Air Data Unit (ADU) messages
        }

        if (last_ins_status.spoof_status != ilab_gps_data.spoof_status) {
            // IL INS spoofing detection messages
            if ((last_ins_status.spoof_status == 2 || last_ins_status.spoof_status == 3) && (ilab_gps_data.spoof_status == 1)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS no spoofing");
            }

            if (last_ins_status.spoof_status == 2) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: GNSS spoofing indicated");
            }

            if (last_ins_status.spoof_status == 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: GNSS multiple spoofing indicated");
            }

            last_ins_status.spoof_status = ilab_gps_data.spoof_status;
        }

        if (last_ins_status.jam_status != ilab_gps_data.jam_status) {
            // IL INS jamming detection messages
            if ((last_ins_status.jam_status == 3) && (ilab_gps_data.jam_status == 1)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS no jamming");
            }

            if (ilab_gps_data.jam_status == 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: GNSS jamming indicated and no fix");
            }

            last_ins_status.jam_status = ilab_gps_data.jam_status;
        }

        if (last_ins_status.ins_sol_status != ilab_ins_data.ins_sol_status) {
            // IL INS navigation solution status messages
            if ((last_ins_status.ins_sol_status == 4 ||
                last_ins_status.ins_sol_status == 6 ||
                last_ins_status.ins_sol_status == 8) &&
                ilab_ins_data.ins_sol_status == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: INS solution is good");
            }

            if (ilab_ins_data.ins_sol_status == 4) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: INS is operating in autonomous mode");
            }

            if (ilab_ins_data.ins_sol_status == 6) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: INS froze position and velocity");
            }

            if (ilab_ins_data.ins_sol_status == 8) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: INS solution is invalid");
            }

            last_ins_status.ins_sol_status = ilab_ins_data.ins_sol_status;
        }
    }

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
        if (check_uart()) {
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

    const bool init_ok = (ilab_ins_data.unit_status & (IL_USW::INITIAL_ALIGNMENT_FAIL|IL_USW::OPERATION_FAIL)) == 0;

    status.flags.initalized = init_ok;

    status.flags.attitude = init_ok && (now - last_att_ms < dt_limit);
    status.flags.vert_vel = init_ok && (now - last_vel_ms < dt_limit);
    status.flags.vert_pos = init_ok && (now - last_pos_ms < dt_limit);
    status.flags.horiz_vel = status.flags.vert_vel;
    status.flags.horiz_pos_abs = status.flags.vert_pos;
    status.flags.horiz_pos_rel = status.flags.vert_pos;
    status.flags.pred_horiz_pos_abs = status.flags.vert_pos;

    status.flags.using_gps = (now - last_gps_ms < dt_limit_gps);
    status.flags.gps_quality_good = (now - last_gps_ms < dt_limit_gps);

    status.flags.rejecting_airspeed = false;
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

    // Send IL INS status flag
    const mavlink_eahrs_status_info_t package{ilab_ins_data.unit_status,
                                              ilab_ins_data.unit_status2,
                                              ilab_ins_data.air_data_status,
                                              (uint16_t)(ilab_gps_data.fix_type-1), //< Send ILabs AHRS output as is. Without inc
                                              ilab_gps_data.spoof_status};
    mavlink_msg_eahrs_status_info_send_struct(link.get_chan(), &package);
}

// get number of points to downsampling
uint16_t AP_ExternalAHRS_InertialLabs::get_num_points_to_dec(const uint16_t rate) const
{
    uint16_t data_rate = get_rate();
    if (rate == 0 || data_rate == 0) {
        return 0;
    }
    uint16_t a = data_rate;
    uint16_t b = rate;
    while (b != 0) {
        uint16_t temp = b;
        b = a % b;
        a = temp;
    }
    return data_rate / a;
}

// send INS status to GCS via MAVLink
void AP_ExternalAHRS_InertialLabs::send_EAHRS_status_report(uint16_t &last_state,
                                                            uint16_t &current_state,
                                                            const ILStatusMessage* msg_list,
                                                            const size_t &msg_list_size,
                                                            uint64_t* last_msg)
{
    uint64_t now_ms = AP_HAL::millis();

    for (size_t i = 0; i < msg_list_size; i++) {
        bool current_status = current_state & msg_list[i].status;
        bool last_status = last_state & msg_list[i].status;
        if ((msg_list[i].severity == MAV_SEVERITY_CRITICAL) && (current_status == true)) {
            if ((current_status != last_status) || (now_ms - last_msg[i] > dt_critical_msg)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: %s", msg_list[i].msg_true);
                last_msg[i] = now_ms;
                continue;
            }
        }
        if (current_status != last_status) {
            if (current_status) {
				GCS_SEND_TEXT(msg_list[i].severity, "ILAB: %s", msg_list[i].msg_true);
            } else {
				GCS_SEND_TEXT(msg_list[i].severity, "ILAB: %s", msg_list[i].msg_false);
            }
        }
    }
    last_state = current_state;
}

// Transmit airspeed to the IL INS
void AP_ExternalAHRS_InertialLabs::make_tx_packet(uint8_t *packet) const
{
    uint8_t *tmp_ptr = packet;
    uint8_t hdr[] = {0xAA, 0x55, 0x01, 0x62}; // 0xAA 0x55 - packet header, 0x01 - packet type, 0x62 - packet ID
    int16_t new_data;
    float external_speed;

    uint16_t len_packet = 1/*type*/ + 1/*ID*/ + 2/*lenght*/ + 1/*meas num*/ + 1/*meas list*/ + sizeof(int16_t)/*airspeed*/ + sizeof(uint16_t)/*checksum*/;

    memcpy(tmp_ptr, hdr, sizeof(hdr)); // header
    tmp_ptr += sizeof(hdr);
    memcpy(tmp_ptr, &len_packet, sizeof(len_packet));
    tmp_ptr += sizeof(len_packet);
    // Air speed , 0x02
    *tmp_ptr++ = 0x01;
    *tmp_ptr++ = 0x02;
    external_speed = floorf(AP::airspeed()->get_raw_airspeed() * 1.94384449f * 100.0f + 0.5f);
    if (external_speed > 32267.0f)  external_speed = 32267.0f;
    if (external_speed < -32268.0f) external_speed = -32268.0f;
    new_data = external_speed;
    memcpy(tmp_ptr, &new_data, sizeof(new_data));
    tmp_ptr += sizeof(new_data);

    uint16_t tmp_crc = crc_sum_of_bytes_16(packet + 2, (tmp_ptr - packet) - 2); // -0xAA55
    memcpy(tmp_ptr, &tmp_crc, sizeof(tmp_crc)); // checksum
    tmp_ptr += sizeof(tmp_crc);

    uart->write(packet, (tmp_ptr - packet));
}

// Use IL INS estimated wind speed in ArduPilot subsystems
bool AP_ExternalAHRS_InertialLabs::get_wind_estimation(Vector3f &wind)
{
    if(option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_USE_WIND_EST)) {
        wind =  ilab_ins_data.wind_speed;
        return true;
    }
    return false;
}

// Transmit data to IL INS
void AP_ExternalAHRS_InertialLabs::write_bytes(const char *bytes, uint8_t len)
{
    uart->write(reinterpret_cast<const uint8_t *>(bytes), len);
}

void AP_ExternalAHRS_InertialLabs::handle_command(ExternalAHRS_command command, const ExternalAHRS_command_data &data)
{
    switch (command) {
        case ExternalAHRS_command::START_UDD:
            write_bytes(InertialLabs::Command::START_UDD,
                        sizeof(InertialLabs::Command::START_UDD) - 1);
            break;
        case ExternalAHRS_command::STOP:
            write_bytes(InertialLabs::Command::STOP,
                        sizeof(InertialLabs::Command::STOP) - 1);
            break;
        case ExternalAHRS_command::ENABLE_GNSS:
            write_bytes(InertialLabs::Command::ENABLE_GNSS,
                        sizeof(InertialLabs::Command::ENABLE_GNSS) - 1);
            break;
        case ExternalAHRS_command::DISABLE_GNSS:
            write_bytes(InertialLabs::Command::DISABLE_GNSS,
                        sizeof(InertialLabs::Command::DISABLE_GNSS) - 1);
            break;
        case ExternalAHRS_command::START_VG3D_CALIBRATION_IN_FLIGHT:
            write_bytes(InertialLabs::Command::START_VG3DCLB_FLIGHT,
                        sizeof(InertialLabs::Command::START_VG3DCLB_FLIGHT) - 1);
            break;
        case ExternalAHRS_command::STOP_VG3D_CALIBRATION_IN_FLIGHT:
            write_bytes(InertialLabs::Command::STOP_VG3DCLB_FLIGHT,
                        sizeof(InertialLabs::Command::STOP_VG3DCLB_FLIGHT) - 1);
            break;
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_POSITION:
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_HORIZONTAL_POSITION:
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_ALTITUDE:
        case ExternalAHRS_command::AIDING_DATA_WIND:
        case ExternalAHRS_command::AIDING_DATA_AMBIENT_AIR:
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_HEADING:
        {
            InertialLabs::Data_context context;
            InertialLabs::fill_command_pyload(context, command, data);
            InertialLabs::fill_transport_protocol_data(context);
            AP::externalAHRS().write_bytes(reinterpret_cast<const char *>(context.data), context.length);
            break;
        }

        default:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Invalid command for handling");
    }

}

void AP_ExternalAHRS_InertialLabs::handle_msg(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_GPS_INPUT: {
            mavlink_gps_input_t packet;
            mavlink_msg_gps_input_decode(&msg, &packet);

            ExternalAHRS_command_data data;
            data.x = packet.lat;
            data.y = packet.lon;
            data.param1 = 0.0f;
            data.param2 = 0.0f;
            data.param3 = 0.0f;
            data.param4 = 0.0f;
            data.z = 0.0f;
            AP::externalAHRS().handle_command(ExternalAHRS_command::AIDING_DATA_EXTERNAL_HORIZONTAL_POSITION, data);

            break;
        }
    }
}

#endif  // AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED