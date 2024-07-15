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

#define IL_GRAVITY_MSS  9.8106f // g-value for IL INS
#define MBAR_TO_PA      100 // mbar to Pa
#define dt_critical_msg 10000 // delay between critical messages to send GCS
#define n_avr           get_num_points_to_average()

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

    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::IMU) |
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

    // create a TX packet to transmit static and diff. pressure to IL INS via UART
    if(option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_trans_diff_pressure)) {
        make_tx_packet(tx_buffer);
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
                CHECK_SIZE(u.gps_ins_time_ms);
                ins_data.ms_tow = u.gps_ins_time_ms;
                break;
            }
            case MessageType::GPS_WEEK: {
                CHECK_SIZE(u.gps_week);
                gnss_data.gps_week = u.gps_week;
                break;
            }
            case MessageType::ACCEL_DATA_HR: {
                CHECK_SIZE(u.accel_data_hr);
                sensors_data.accel = u.accel_data_hr.tofloat().rfu_to_frd() * IL_GRAVITY_MSS * 1.0e-6f; // m/s/s
                break;
            }
            case MessageType::GYRO_DATA_HR: {
                CHECK_SIZE(u.gyro_data_hr);
                sensors_data.gyro = u.gyro_data_hr.tofloat().rfu_to_frd() * DEG_TO_RAD * 1.0e-5f; // rad/s
                break;
            }
            case MessageType::BARO_DATA: {
                CHECK_SIZE(u.baro_data);
                sensors_data.pressure = u.baro_data.pressure * 2; // Pa
                air_data.baro_alt = u.baro_data.baro_alt * 0.01f; // m
                break;
            }
            case MessageType::MAG_DATA: {
                CHECK_SIZE(u.mag_data);
                sensors_data.mag = u.mag_data.tofloat().rfu_to_frd() * NTESLA_TO_MGAUSS * 10; // milligauss
                break;
            }
            case MessageType::ORIENTATION_ANGLES: {
                CHECK_SIZE(u.orientation_angles);
                att_data.yaw = u.orientation_angles.yaw * 0.01f; // deg
                att_data.pitch = u.orientation_angles.pitch * 0.01f; // deg
                att_data.roll = u.orientation_angles.roll * 0.01f; // deg
                break;
            }
            case MessageType::VELOCITIES: {
                CHECK_SIZE(u.velocity);
                ins_data.velocity = u.velocity.tofloat().rfu_to_frd() * 0.01f; // m/s
                break;
            }
            case MessageType::POSITION: {
                CHECK_SIZE(u.position);
                ins_data.latitude = u.position.lat; // deg*1.0e7
                ins_data.longitude = u.position.lng; // deg*1.0e7
                ins_data.altitude = u.position.alt; // cm
                break;
            }
            case MessageType::UNIT_STATUS: {
                CHECK_SIZE(u.unit_status);
                ins_data.unit_status = u.unit_status;
                break;
            }
            case MessageType::GNSS_EXTENDED_INFO: {
                CHECK_SIZE(u.gnss_extended_info);
                gnss_data.fix_type = u.gnss_extended_info.fix_type;
                gnss_data.spoof_status = u.gnss_extended_info.gnss_spoof_status;
                break;
            }
            case MessageType::NUM_SATS: {
                CHECK_SIZE(u.num_sats);
                gnss_data.num_sat = u.num_sats;
                break;
            }
            case MessageType::GNSS_POSITION: {
                CHECK_SIZE(u.gnss_position);
                gnss_data.latitude = u.gnss_position.lat; // deg*1.0e7
                gnss_data.longitude = u.gnss_position.lng; // deg*1.0e7
                gnss_data.altitude = u.gnss_position.alt; // cm
                break;
            }
            case MessageType::GNSS_VEL_TRACK: {
                CHECK_SIZE(u.gnss_vel_track);
                gnss_data.hor_speed = u.gnss_vel_track.hor_speed * 0.01f; // m/s
                gnss_data.track_over_ground = u.gnss_vel_track.track_over_ground * 0.01f; // deg
                gnss_data.ver_speed = u.gnss_vel_track.ver_speed * 0.01f; // m/s
                break;
            }
            case MessageType::GNSS_POS_TIMESTAMP: {
                CHECK_SIZE(u.gnss_pos_timestamp);
                gnss_data.ms_tow = u.gnss_pos_timestamp;
                break;
            }
            case MessageType::GNSS_INFO_SHORT: {
                CHECK_SIZE(u.gnss_info_short);
                gnss_data.info1 = u.gnss_info_short.info1;
                gnss_data.info2 = u.gnss_info_short.info2;
                break;
            }
            case MessageType::GNSS_NEW_DATA: {
                CHECK_SIZE(u.gnss_new_data);
                gnss_data.new_data = u.gnss_new_data;
                break;
            }
            case MessageType::GNSS_JAM_STATUS: {
                CHECK_SIZE(u.gnss_jam_status);
                gnss_data.jam_status = u.gnss_jam_status;
                break;
            }
            case MessageType::DIFFERENTIAL_PRESSURE: {
                CHECK_SIZE(u.differential_pressure);
                sensors_data.diff_press = u.differential_pressure * MBAR_TO_PA * 1.0e-4f; // Pa
                break;
            }
            case MessageType::TRUE_AIRSPEED: {
                CHECK_SIZE(u.true_airspeed);
                air_data.true_airspeed = u.true_airspeed * 0.01f; // m/s
                break;
            }
            case MessageType::WIND_SPEED: {
                CHECK_SIZE(u.wind_speed);
                air_data.wind_speed = u.wind_speed.tofloat().rfu_to_frd() * 0.01f; // m/s
                break;
            }
            case MessageType::AIR_DATA_STATUS: {
                CHECK_SIZE(u.air_data_status);
                air_data.air_data_status = u.air_data_status;
                break;
            }
            case MessageType::SUPPLY_VOLTAGE: {
                CHECK_SIZE(u.supply_voltage);
                sensors_data.supply_voltage = u.supply_voltage * 0.01f; // V
                break;
            }
            case MessageType::TEMPERATURE: {
                CHECK_SIZE(u.temperature);
                sensors_data.temperature = u.temperature * 0.1f; // degC
                break;
            }
            case MessageType::UNIT_STATUS2: {
                CHECK_SIZE(u.unit_status2);
                ins_data.unit_status2 = u.unit_status2;
                break;
            }
            case MessageType::GNSS_DOP: {
                CHECK_SIZE(u.gnss_dop);
                gnss_data.gdop = u.gnss_dop.gdop * 1.0e-3f;
                gnss_data.pdop = u.gnss_dop.pdop * 1.0e-3f;
                gnss_data.hdop = u.gnss_dop.hdop * 1.0e-3f;
                gnss_data.vdop = u.gnss_dop.vdop * 1.0e-3f;
                gnss_data.tdop = u.gnss_dop.tdop * 1.0e-3f;
                break;
            }
            case MessageType::INS_SOLUTION_STATUS: {
                CHECK_SIZE(u.nav_sol_status);
                ins_data.nav_sol_status = u.nav_sol_status;
                break;
            }
            case MessageType::NEW_AIDING_DATA: {
                CHECK_SIZE(u.new_aiding_data);
                ins_ext_data.new_aiding_data = u.new_aiding_data;
                break;
            };
            case MessageType::EXT_POS: {
                CHECK_SIZE(u.ext_pos);
                ins_ext_data.ext_pos = u.ext_pos;
                break;
            };
            case MessageType::EXT_HOR_POS: {
                CHECK_SIZE(u.ext_hor_pos);
                ins_ext_data.ext_hor_pos = u.ext_hor_pos;
                break;
            };
            case MessageType::EXT_ALT: {
                CHECK_SIZE(u.ext_alt);
                ins_ext_data.ext_alt = u.ext_alt;
                break;
            };
            case MessageType::EXT_HEADING: {
                CHECK_SIZE(u.ext_heading);
                ins_ext_data.ext_heading = u.ext_heading;
                break;
            };
            case MessageType::EXT_AMBIENT_DATA: {
                CHECK_SIZE(u.ext_ambient_air_data);
                ins_ext_data.ext_ambient_air_data = u.ext_ambient_air_data;
                break;
            };
            case MessageType::EXT_WIND_DATA: {
                CHECK_SIZE(u.ext_wind_data);
                ins_ext_data.ext_wind_data = u.ext_wind_data;
                break;
            };
            case MessageType::EXT_ADC_DATA: {
                CHECK_SIZE(u.ext_ADC_data);
                ins_ext_data.ext_ADC_data = u.ext_ADC_data;
                break;
            };
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

    buffer_ofs = 0;

    /*
    Start of processing of received IL INS data
    */

    if (!(ins_data.unit_status & IL_USW::INITIAL_ALIGNMENT_FAIL) && (ins_data.nav_sol_status != 8)) {
        // use IL INS attitude data in the ArduPilot algorithm
        state.quat.from_euler(radians(att_data.roll),
                              radians(att_data.pitch),
                              radians(att_data.yaw));
        state.have_quaternion = true;
        if (last_att_msg_ms == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: got link");
        }

        last_att_msg_ms = now_ms;
    }

    if (!(ins_data.unit_status & (IL_USW::ACCEL_FAIL | IL_USW::GYRO_FAIL))) {
        // use IL INS IMU outputs in the ArduPilot algorithm
        ardu_imu_data.accel = sensors_data.accel;
        ardu_imu_data.gyro = sensors_data.gyro;
        ardu_imu_data.temperature = sensors_data.temperature;
        AP::ins().handle_external(ardu_imu_data);
        state.accel = ardu_imu_data.accel;
        state.gyro = ardu_imu_data.gyro;
    }

    if (!(ins_data.unit_status & IL_USW::MAG_FAIL)) {
        // use IL INS magnetometer outputs in the ArduPilot algorithm
        ardu_mag_data.field = sensors_data.mag;
        AP::compass().handle_external(ardu_mag_data);
    }

    bool hasNewGpsData = (gnss_data.new_data & (IL_NEWGPS::NEW_GNSS_POSITION | IL_NEWGPS::NEW_GNSS_VELOCITY)) != 0; // true if received new GNSS position or velocity
    bool info2Valid = (gnss_data.info2 & 0x03) == 0x00; // true if GNSS solution is computed

    uint8_t nav_source = 0;
    if (ins_data.nav_sol_status != 8 && ins_data.nav_sol_status != 6) {
        // use IL INS navigation solution if its navigation filter is valid and the position/speed is not frozen
        state.have_velocity = true;
        state.have_location = true;
        state.last_location_update_us = AP_HAL::micros();
        state.location.lat = ins_data.latitude;
        state.location.lng = ins_data.longitude;
        state.location.alt = ins_data.altitude;
        state.velocity = ins_data.velocity;

        // use IL INS navigation solution instead of GNSS solution
        ardu_gps_data.ms_tow = ins_data.ms_tow;
        ardu_gps_data.longitude = state.location.lng;
        ardu_gps_data.latitude = state.location.lat;
        ardu_gps_data.msl_altitude = state.location.alt;
        ardu_gps_data.ned_vel_north = state.velocity.x;
        ardu_gps_data.ned_vel_east = state.velocity.y;
        ardu_gps_data.ned_vel_down = state.velocity.z;

        nav_source = 1;
        last_ins_msg_ms = now_ms;
    } else if (!(ins_data.unit_status & IL_USW::GNSS_FAIL) && info2Valid) {
        // use GNSS solution directly if INS navigation filter is invalid and GNSS solution is computed
        ardu_gps_data.ms_tow = gnss_data.ms_tow;
        ardu_gps_data.longitude = gnss_data.longitude;
        ardu_gps_data.latitude = gnss_data.latitude;
        ardu_gps_data.msl_altitude = gnss_data.altitude;
        ardu_gps_data.ned_vel_north = cosF(radians(gnss_data.track_over_ground * 0.01f)) * gnss_data.hor_speed * 0.01f;
        ardu_gps_data.ned_vel_east = sinF(radians(gnss_data.track_over_ground * 0.01f)) * gnss_data.hor_speed * 0.01f;
        ardu_gps_data.ned_vel_down = -gnss_data.ver_speed * 0.01f;

        nav_source = 2;
    }

    if (nav_source != 0 && hasNewGpsData) {
        // update the ardupilot GPS data if INS has received a new position or velocity from the GNSS receiver
        if (info2Valid && (gnss_data.num_sat > 4)) {
            // use real values ​​if the GNSS computed is calculated and the number of satellites is more than 4
            ardu_gps_data.satellites_in_view = gnss_data.num_sat;
            ardu_gps_data.hdop = gnss_data.hdop * 100;
            ardu_gps_data.vdop = gnss_data.vdop * 100;
            ardu_gps_data.fix_type = gnss_data.fix_type + 1;
        } else {
            ardu_gps_data.satellites_in_view = 77;
            ardu_gps_data.hdop = 90; // 0.9
            ardu_gps_data.hdop = 90; // 0.9
            ardu_gps_data.fix_type = 3;
        }

        ardu_gps_data.gps_week = gnss_data.gps_week;

        uint8_t instance;
        if (AP::gps().get_first_external_instance(instance)) {
            AP::gps().handle_external(ardu_gps_data, instance);
        }

        if (last_gps_msg_ms == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "InertialLabs: got GPS lock");
            if (!state.have_origin) {
                state.origin = Location{
                    ardu_gps_data.latitude,
                    ardu_gps_data.longitude,
                    ardu_gps_data.msl_altitude,
                    Location::AltFrame::ABSOLUTE};
                state.have_origin = true;
            }
        }

        last_gps_msg_ms = now_ms;
    }

    if (!(ins_data.unit_status2 & IL_USW2::ADU_BARO_FAIL)) {
        ardu_baro_data.pressure_pa = sensors_data.pressure;
        ardu_baro_data.temperature = sensors_data.temperature;
        AP::baro().handle_external(ardu_baro_data);
    }

#if AP_AIRSPEED_EXTERNAL_ENABLED && (APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane))
    // only on plane and copter as others do not link AP_Airspeed
    if (!(ins_data.unit_status2 & IL_USW2::ADU_DIFF_PRESS_FAIL)) {
        ardu_airspeed_data.differential_pressure = sensors_data.diff_press;
        ardu_airspeed_data.temperature = sensors_data.temperature;
        ardu_airspeed_data.airspeed = air_data.true_airspeed;
        auto *arsp = AP::airspeed();
        if (arsp != nullptr) {
            arsp->handle_external(ardu_airspeed_data);
            if (option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_USE_AIRSPEED)) {
                // use airspeed calculated by IL INS instead of ArduPilot
                if (!(air_data.air_data_status & IL_ADU::AIRSPEED_FAIL) ||
                    ((ins_data.nav_sol_status != 8) && (ins_data.nav_sol_status != 6))) {
                    arsp->set_airspeed_enable(true);
                } else {
                    arsp->set_airspeed_enable(false);
                }
            }
        }
    }
#endif // AP_AIRSPEED_EXTERNAL_ENABLED

    /*
    End of processing of received IL INS data
    */

    // Send IL INS status messages to GCS via MAVLink
    if (option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_SEND_STATUS)) {
        send_EAHRS_status_report(last_ins_status.unit_status, ins_data.unit_status, IL_usw_msg, IL_usw_msg_size, IL_usw_last_msg_ms); // IL INS Unit Status Word (USW) messages

        send_EAHRS_status_report(last_ins_status.unit_status2, ins_data.unit_status2, IL_usw2_msg, IL_usw2_msg_size, IL_usw2_last_msg_ms); // IL INS Unit Status Word 2 (USW2) messages

        send_EAHRS_status_report(last_ins_status.adu_status, air_data.air_data_status, IL_adu_msg, IL_adu_msg_size, IL_adu_last_msg_ms); // IL Air Data Unit (ADU) messages

        if (last_ins_status.spoof_status != gnss_data.spoof_status) {
            // IL INS spoofing detection messages
            if ((last_ins_status.spoof_status == 2 || last_ins_status.spoof_status == 3) && (gnss_data.spoof_status == 1)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS no spoofing");
            }

            if (last_ins_status.spoof_status == 2) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: GNSS spoofing indicated");
            }

            if (last_ins_status.spoof_status == 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: GNSS multiple spoofing indicated");
            }

            last_ins_status.spoof_status = gnss_data.spoof_status;
        }

        if (last_ins_status.jam_status != gnss_data.jam_status) {
            // IL INS jamming detection messages
            if ((last_ins_status.jam_status == 3) && (gnss_data.jam_status == 1)) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS no jamming");
            }

            if (gnss_data.jam_status == 3) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: GNSS jamming indicated and no fix");
            }

            last_ins_status.jam_status = gnss_data.jam_status;
        }

        if (last_ins_status.nav_sol_status != ins_data.nav_sol_status) {
            // IL INS navigation solution status messages
            if ((last_ins_status.nav_sol_status == 4 ||
                 last_ins_status.nav_sol_status == 6 ||
                 last_ins_status.nav_sol_status == 8) &&
                ins_data.nav_sol_status == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: INS solution is good");
            }

            if (ins_data.nav_sol_status == 4) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: INS is operating in autonomous mode");
            }

            if (ins_data.nav_sol_status == 6) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: INS froze position and velocity");
            }

            if (ins_data.nav_sol_status == 8) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: INS solution is invalid");
            }

            last_ins_status.nav_sol_status = ins_data.nav_sol_status;
        }
    }

#if HAL_LOGGING_ENABLED
    uint64_t now_ms_log = AP_HAL::micros64();
    // uint8_t n_avr = get_num_points_to_average(); // the number of points to average the IL INS data for logging

    if (counter == n_avr) {

        sensors_data_log.accel /= n_avr;
        sensors_data_log.gyro /= n_avr;
        sensors_data_log.mag /= n_avr;
        sensors_data_log.temperature /= n_avr;
        sensors_data_log.supply_voltage /= n_avr;
        sensors_data_log.pressure /= n_avr;
        sensors_data_log.diff_press /= n_avr;

        att_data_log.yaw /= n_avr;
        att_data_log.pitch /= n_avr;
        att_data_log.roll /= n_avr;

        ins_data_log.velocity /= n_avr;
        ins_data_log.latitude /= n_avr;
        ins_data_log.longitude /= n_avr;
        ins_data_log.altitude /= n_avr;
        ins_data_log.unit_status |= ins_data.unit_status;
        ins_data_log.unit_status2 |= ins_data.unit_status2;
        ins_data_log.nav_sol_status |= ins_data.nav_sol_status;

        air_data_log.baro_alt /= n_avr;
        air_data_log.true_airspeed /= n_avr;
        air_data_log.wind_speed /= n_avr;
        air_data_log.air_data_status |= air_data.air_data_status;

        // @LoggerMessage: ILB1
        // @Description: InertialLabs AHRS data1
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
                                    "ssEEEoooGGG",
                                    "F----------",
                                    "QIfffffffff",
                                    now_ms_log, ins_data.ms_tow,
                                    sensors_data_log.gyro.x, sensors_data_log.gyro.y, sensors_data_log.gyro.z,
                                    sensors_data_log.accel.x, sensors_data_log.accel.y, sensors_data_log.accel.z,
                                    sensors_data_log.mag.x, sensors_data_log.mag.y, sensors_data_log.mag.z);

        // @LoggerMessage: ILB2
        // @Description: InertialLabs AHRS data2
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: Press: Static pressure
        // @Field: Diff: Differential pressure
        // @Field: Temp: Temperature
        // @Field: Alt: Baro altitude
        // @Field: TAS: true airspeed
        // @Field: VWN: Wind velocity north
        // @Field: VWE: Wind velocity east
        // @Field: ADU: Air Data Unit status

        AP::logger().WriteStreaming("ILB2", "TimeUS,IMS,Press,Diff,Temp,Alt,TAS,VWN,VWE,ADU",
                                    "ssPPOmnnn-",
                                    "F---------",
                                    "QIfffffffH",
                                    now_ms_log, ins_data.ms_tow,
                                    sensors_data_log.pressure, sensors_data_log.diff_press, sensors_data_log.temperature,
                                    air_data_log.baro_alt, air_data_log.true_airspeed,
                                    air_data_log.wind_speed.x, air_data_log.wind_speed.y,
                                    air_data_log.air_data_status);

        // @LoggerMessage: ILB3
        // @Description: InertialLabs AHRS data3
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: Yaw: euler yaw
        // @Field: Pitch: euler pitch
        // @Field: Roll: euler roll
        // @Field: VN: velocity north
        // @Field: VE: velocity east
        // @Field: VD: velocity down
        // @Field: Lat: latitude
        // @Field: Lng: longitude
        // @Field: Alt: altitude MSL
        // @Field: USW: USW1
        // @Field: USW2: USW2
        // @Field: ISS: INS Navigation (Solution) Status

        AP::logger().WriteStreaming("ILB3", "TimeUS,IMS,Yaw,Pitch,Roll,VN,VE,VD,Lat,Lng,Alt,USW,USW2,ISS",
                                    "ssdddnnnDUm---",
                                    "F-------------",
                                    "QIfffffffffHHB",
                                    now_ms_log, ins_data.ms_tow,
                                    att_data.yaw, att_data.pitch, att_data.roll,
                                    ins_data.velocity.x, ins_data.velocity.y, ins_data.velocity.z,
                                    (float)(ins_data.latitude*1.0e-7f), (float)(ins_data.longitude*1.0e-7f), (float)(ins_data.altitude*0.01f),
                                    ins_data.unit_status, ins_data.unit_status2, ins_data.nav_sol_status);

        sensors_data_log = {};
        att_data_log = {};
        ins_data_log = {};
        air_data_log = {};

        counter = 0;
    } else {
        sensors_data_log.accel += sensors_data.accel;
        sensors_data_log.gyro += sensors_data.gyro;
        sensors_data_log.mag += sensors_data.mag;
        sensors_data_log.temperature += sensors_data.temperature;
        sensors_data_log.supply_voltage += sensors_data.supply_voltage;
        sensors_data_log.pressure += sensors_data.pressure;
        sensors_data_log.diff_press += sensors_data.diff_press;

        att_data_log.yaw += att_data.yaw;
        att_data_log.pitch += att_data.pitch;
        att_data_log.roll += att_data.roll;

        ins_data_log.velocity += ins_data.velocity;
        ins_data_log.latitude += ins_data.latitude;
        ins_data_log.longitude += ins_data.longitude;
        ins_data_log.altitude += ins_data.altitude;
        ins_data_log.unit_status |= ins_data.unit_status;
        ins_data_log.unit_status2 |= ins_data.unit_status2;
        ins_data_log.nav_sol_status |= ins_data.nav_sol_status;

        air_data_log.baro_alt += air_data.baro_alt;
        air_data_log.true_airspeed += air_data.true_airspeed;
        air_data_log.wind_speed += air_data.wind_speed;
        air_data_log.air_data_status |= air_data.air_data_status;

        counter++;
    }

    if (hasNewGpsData) {
        // @LoggerMessage: ILB4
        // @Description: InertialLabs AHRS data4
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: GMS: GNSS Position timestamp
        // @Field: GWk: GPS Week
        // @Filed: NSat: Number of satellites
        // @Field: NewGPS: Indicator of new update of GPS data
        // @Field: Lat: GNSS Latitude
        // @Field: Lng: GNSS Longitude
        // @Field: Alt: GNSS Altitude
        // @Field: GCrs: GNSS Track over ground
        // @Field: Spd: GNSS Horizontal speed
        // @Field: VZ: GNSS Vertical speed

        AP::logger().WriteStreaming("ILB4", "TimeUS,IMS,GMS,GWk,NSat,NewGPS,Lat,Lng,Alt,GCrs,Spd,VZ",
                                    "sss---DUmhnn",
                                    "F-----------",
                                    "QIIHBBffffff",
                                    now_ms_log, ins_data_log.ms_tow, gnss_data.ms_tow, gnss_data.gps_week, gnss_data.num_sat, gnss_data.new_data,
                                    (float)(gnss_data.latitude*1.0e-7f), (float)(gnss_data.longitude*1.0e-7f), (float)(gnss_data.altitude*0.01f),
                                    gnss_data.track_over_ground, gnss_data.hor_speed, gnss_data.ver_speed);

        // @LoggerMessage: ILB5
        // @Description: InertialLabs AHRS data5
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: FType: fix type
        // @Field: GSS: GNSS spoofing status
        // @Field: GJS: GNSS jamming status
        // @Field: GI1: GNSS Info1
        // @Field: GI2: GNSS Info2
        // @Field: GDOP: GNSS GDOP
        // @Field: PDOP: GNSS PDOP
        // @Field: HDOP: GNSS HDOP
        // @Field: VDOP: GNSS VDOP
        // @Field: TDOP: GNSS TDOP

        AP::logger().WriteStreaming("ILB5", "TimeUS,IMS,FType,GSS,GJS,GI1,GI2,GDOP,PDOP,HDOP,VDOP,TDOP",
                                    "ss----------",
                                    "F-----------",
                                    "QIBBBBBfffff",
                                    now_ms_log, ins_data.ms_tow, gnss_data.fix_type, gnss_data.spoof_status,
                                    gnss_data.jam_status, gnss_data.info1, gnss_data.info2,
                                    gnss_data.gdop, gnss_data.pdop, gnss_data.hdop, gnss_data.vdop, gnss_data.tdop);
    }

    if ((ins_ext_data.new_aiding_data & (IL_NewAidingData::NEW_EXT_POS |
                                         IL_NewAidingData::NEW_EXT_HOR_POS |
                                         IL_NewAidingData::NEW_ALTITUDE |
                                         IL_NewAidingData::NEW_HEADING)) != 0) {

        ins_ext_pos_data_log = {};

        if (ins_ext_data.new_aiding_data & IL_NewAidingData::NEW_EXT_POS) {
            ins_ext_pos_data_log.latitude = (float)ins_ext_data.ext_pos.lat * 1.0e-7f;
            ins_ext_pos_data_log.longitude = (float)ins_ext_data.ext_pos.lon * 1.0e-7f;
            ins_ext_pos_data_log.latitude_std = (float)ins_ext_data.ext_pos.lat_std * 0.01f;
            ins_ext_pos_data_log.longitude_std = (float)ins_ext_data.ext_pos.lon_std * 0.01f;
            ins_ext_pos_data_log.pos_latency = (float)ins_ext_data.ext_pos.pos_latency * 1.0e-3f;
        } else if (ins_ext_data.new_aiding_data & IL_NewAidingData::NEW_EXT_HOR_POS) {
            ins_ext_pos_data_log.latitude = (float)ins_ext_data.ext_hor_pos.lat * 1.0e-7f;
            ins_ext_pos_data_log.longitude = (float)ins_ext_data.ext_hor_pos.lon * 1.0e-7f;
            ins_ext_pos_data_log.latitude_std = (float)ins_ext_data.ext_hor_pos.lat_std * 0.01f;
            ins_ext_pos_data_log.longitude_std = (float)ins_ext_data.ext_hor_pos.lon_std * 0.01f;
            ins_ext_pos_data_log.pos_latency = (float)ins_ext_data.ext_hor_pos.pos_latency * 1.0e-3f;
        }

         if (ins_ext_data.new_aiding_data & IL_NewAidingData::NEW_EXT_POS) {
            ins_ext_pos_data_log.altitude = (float)ins_ext_data.ext_pos.alt * 1.0e-3f;
            ins_ext_pos_data_log.altitude_std = (float)ins_ext_data.ext_pos.alt_std * 0.01f;
        } else if (ins_ext_data.new_aiding_data & IL_NewAidingData::NEW_ALTITUDE) {
            ins_ext_pos_data_log.altitude = (float)ins_ext_data.ext_alt.alt * 1.0e-3f;
            ins_ext_pos_data_log.altitude_std = (float)ins_ext_data.ext_alt.alt_std * 0.01f;
        }

        // @LoggerMessage: ILB6
        // @Description: InertialLabs AHRS data6
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: NAD: New Aiding (External) Data status
        // @Field: ExLat
        // @Field: ExLng
        // @Field: ExAlt
        // @Field: ExLatSD
        // @Field: ExLngSD
        // @Field: ExAltSD
        // @Field: ExPosL
        // @Field: ExYaw
        // @Field: ExYawSD
        // @Field: ExYawL

        AP::logger().WriteStreaming("ILB6", "TimeUS,IMS,NAD",
                                    "ss-",
                                    "F--",
                                    "QIH",
                                    now_ms_log, ins_data.ms_tow, ins_ext_data.new_aiding_data,
                                    ins_ext_pos_data_log.latitude, ins_ext_pos_data_log.longitude, ins_ext_pos_data_log.altitude,
                                    ins_ext_pos_data_log.latitude_std, ins_ext_pos_data_log.longitude_std, ins_ext_pos_data_log.altitude_std,
                                    ins_ext_pos_data_log.pos_latency,
                                    (float)ins_ext_data.ext_heading.heading * 0.01f,
                                    (float)ins_ext_data.ext_heading.std * 0.01f,
                                    (float)ins_ext_data.ext_heading.latency * 1.0e-3f);
        }

    if ((ins_ext_data.new_aiding_data & (IL_NewAidingData::NEW_WIND | IL_NewAidingData::NEW_AMBIENT | IL_NewAidingData::NEW_ADC)) != 0) {
        // @LoggerMessage: ILB7
        // @Description: InertialLabs AHRS data7
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: NAD: New Aiding (External) Data status
        // @Field: ExTemp
        // @Field: ExAlt
        // @Field: ExPress
        // @Field: ExWn
        // @Field: ExWe
        // @Field: ExWnSD
        // @Field: ExWeSD
        // @Field: ExADCs
        // @Field: ExADCd

        AP::logger().WriteStreaming("ILB7", "TimeUS,IMS,NAD",
                                    "ss-",
                                    "F--",
                                    "QIH",
                                    now_ms_log, ins_data.ms_tow, ins_ext_data.new_aiding_data,
                                    (float)ins_ext_data.ext_ambient_air_data.air_temp * 0.1f,
                                    (float)ins_ext_data.ext_ambient_air_data.alt * 0.01f,
                                    (float)ins_ext_data.ext_ambient_air_data.abs_press * 2,
                                    (float)ins_ext_data.ext_wind_data.n_wind_vel * 0.01f,
                                    (float)ins_ext_data.ext_wind_data.e_wind_vel * 0.01f,
                                    (float)ins_ext_data.ext_wind_data.n_std_wind * 0.01f,
                                    (float)ins_ext_data.ext_wind_data.e_std_wind * 0.01f,
                                    (float)ins_ext_data.ext_ADC_data.static_press,
                                    (float)ins_ext_data.ext_ADC_data.diff_press * 0.1f);
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
    return AP_HAL::millis() - last_att_msg_ms < 100;
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
    if (now - last_att_msg_ms > 10 ||
        now - last_ins_msg_ms > 10) {
        hal.util->snprintf(failure_msg, failure_msg_len, "InertialLabs not up to date");
        return false;
    }
    return true;
}

// get filter status.
void AP_ExternalAHRS_InertialLabs::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);

    uint32_t now = AP_HAL::millis();
    const uint32_t dt_limit = 200;

    bool is_got_nav_msg = (now - last_ins_msg_ms < dt_limit);
    bool is_valid_nav_status = (ins_data.nav_sol_status != 6) && (ins_data.nav_sol_status != 8);

    memset(&status, 0, sizeof(status));

    status.flags.attitude = (now - last_att_msg_ms < dt_limit) && (ins_data.nav_sol_status != 8);

    status.flags.horiz_vel = is_got_nav_msg && is_valid_nav_status;
    status.flags.vert_vel = status.flags.horiz_vel;

    status.flags.horiz_pos_abs = is_got_nav_msg && is_valid_nav_status;
    status.flags.horiz_pos_rel = status.flags.horiz_pos_abs;
    status.flags.vert_pos = status.flags.horiz_pos_abs;
    status.flags.const_pos_mode = ins_data.nav_sol_status == 6;
    status.flags.pred_horiz_pos_rel = status.flags.horiz_pos_abs;
    status.flags.pred_horiz_pos_abs = status.flags.horiz_pos_abs;

    status.flags.using_gps = true;
    status.flags.gps_quality_good = true;

    status.flags.initalized = (ins_data.unit_status & IL_USW::INITIAL_ALIGNMENT_FAIL) == 0;

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

    mavlink_msg_ekf_status_report_send(link.get_chan(), flags, 0, 0, 0, 0, 0, 0);

    // todo: Move to the separate function
    // Send status flags
    const mavlink_eahrs_status_info_t package{last_ins_status.unit_status,
                                              last_ins_status.unit_status2,
                                              last_ins_status.adu_status,
                                              (uint16_t)(gnss_data.fix_type-1), //< Send ILabs AHRS output as is. Without inc
                                              gnss_data.spoof_status};
    mavlink_msg_eahrs_status_info_send_struct(link.get_chan(), &package);
}

void AP_ExternalAHRS_InertialLabs::send_EAHRS_status_report(uint16_t &last_state,
                                                            uint16_t &current_state,
                                                            const ILStatusMessage* msg_list,
                                                            const size_t &msg_list_size,
                                                            uint64_t* last_msg)
{
    uint64_t now_ms = AP_HAL::millis();

    for (size_t i = 0; i <= msg_list_size; i++) {
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

uint8_t AP_ExternalAHRS_InertialLabs::get_num_points_to_average() const
{
    uint16_t log_rate = get_eahrs_log_rate();
    uint16_t data_rate = get_rate();

    if (log_rate != 0) {

        while (data_rate % log_rate != 0) {
            log_rate--;
            if (log_rate == 0) {
                return 0;
            }
        }
        return static_cast<uint8_t>(data_rate / log_rate);
    } else {
        return 0;
    }
}

bool AP_ExternalAHRS_InertialLabs::get_wind_estimation(Vector3f &wind)
{
    if (option_is_set(AP_ExternalAHRS::OPTIONS::ILAB_USE_AIRSPEED)) {
        wind =  air_data.wind_speed;
        return true;
    }

    return false;
}

void AP_ExternalAHRS_InertialLabs::make_tx_packet(uint8_t *packet) const //AVK 28.05.2024
{
    static int div = 0;

    uint8_t *tmp_ptr = packet;
    uint8_t hdr[] = {0xAA, 0x55, 0x01, 0x62}; // 0xAA 0x55 - packet header, 0x01 - packet type, 0x62 - packet ID
    uint32_t tmp;
    float press ;

    uint16_t len_packet = 1/*type*/ + 1/*ID*/ + 2/*lenght*/ + 1/*meas num*/ + 1/*meas list*/ + \
                        sizeof(uint32_t/*absolute pressure*/) + \
                        sizeof(int32_t)/*differential pressure*/ + \
                        sizeof(uint16_t)/*checksum*/;

  if (!(div & 3)) { // 200 Hz / 4 = 50 Hz send packet to ILab
        memcpy(tmp_ptr, hdr, sizeof(hdr)); // header
        tmp_ptr += sizeof(hdr);
        memcpy(tmp_ptr, &len_packet, sizeof(len_packet));
        tmp_ptr += sizeof(len_packet);
        *tmp_ptr++ = 0x01;
        *tmp_ptr++ = 0x12;
        press = AP::baro().get_pressure();
        tmp = (uint32_t)(press); // static pressure in Pa
        memcpy(tmp_ptr, &tmp, sizeof(tmp));
        tmp_ptr += sizeof(tmp);
        press = AP::airspeed()->get_differential_pressure();
        tmp = (int32_t)(press * 10.0); // differential pressure in Pa
        memcpy(tmp_ptr, &tmp, sizeof(tmp));
        tmp_ptr += sizeof(tmp);

        uint16_t tmp_crc = crc_sum_of_bytes_16(packet + 2, (tmp_ptr - packet) - 2); // -0xAA55
        memcpy(tmp_ptr, &tmp_crc, sizeof(tmp_crc)); // checksum
        tmp_ptr += sizeof(tmp_crc);

        uart->write(packet, (tmp_ptr - packet));
    }

    div++;
}

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

#endif  // AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED
