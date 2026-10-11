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
  simulate InertialLabs external AHRS
*/
#include "SIM_InertialLabs.h"
#include <GCS_MAVLink/GCS.h>
#include "SIM_GPS.h"

using namespace SITL;

namespace {

// gravity the device uses for accelerations in g (ICD)
constexpr float IL_GRAVITY_MSS = 9.8106f;

// unit status word 2 bits
constexpr uint16_t USW2_GNSS_INPUT_OFF = 1U << 7;
constexpr uint16_t USW2_GNSS_POS_INVALID = 1U << 10;

// navigation solution status
constexpr uint8_t NSS_GOOD = 0;
constexpr uint8_t NSS_AUTONOMOUS = 4;

// 0x41 New GPS bits
constexpr uint8_t NEW_GNSS_POSITION = 1U << 0;
constexpr uint8_t NEW_GNSS_VELOCITY = 1U << 1;

// GNSS solution status
constexpr uint8_t GNSS_SOL_COMPUTED = 0;
constexpr uint8_t GNSS_SOL_INSUFFICIENT_OBS = 1;

// new aiding data bits
constexpr uint16_t NAD_AIRSPEED = 1U << 1;
constexpr uint16_t NAD_WIND = 1U << 2;
constexpr uint16_t NAD_EXT_POSITION = 1U << 3;
constexpr uint16_t NAD_EXT_HEADING = 1U << 5;
constexpr uint16_t NAD_AMBIENT_AIR = 1U << 10;
constexpr uint16_t NAD_EXT_ALTITUDE = 1U << 11;
constexpr uint16_t NAD_EXT_HOR_POSITION = 1U << 13;

// copy an aiding data item if it has the expected size
bool copy_aiding_data(void *dest, size_t size, const uint8_t *data, uint16_t len)
{
    if (len != size) {
        return false;
    }
    memcpy(dest, data, size);
    return true;
}

}  // namespace

// the transmit buffer holds many packets, so a late reader gets delayed rather than truncated data
InertialLabs::InertialLabs() : SerialDevice::SerialDevice(8192, 512)
{
}

void InertialLabs::send_packet(void)
{
    const auto &fdm = _sitl->state;
    const bool gnss_available = _sitl->gps[0].enabled;

    pkt.msg_len = sizeof(pkt)-2;

    const auto gps_tow = GPS_Backend::gps_time();

    // 0x01 GPS INS Time (round)
    pkt.gps_time_ms = gps_tow.ms;

    // 0x23 Accelerometer data HR
    const float accel_noise = 0.005f; // g
    pkt.accel_data_hr.x = ((fdm.yAccel / IL_GRAVITY_MSS) + accel_noise * rand_float()) * 1.0e6;  // g*1.0e6
    pkt.accel_data_hr.y = ((fdm.xAccel / IL_GRAVITY_MSS) + accel_noise * rand_float()) * 1.0e6;  // g*1.0e6
    pkt.accel_data_hr.z = ((-fdm.zAccel / IL_GRAVITY_MSS) + accel_noise * rand_float()) * 1.0e6; // g*1.0e6

    // 0x21 Gyro data HR
    const float gyro_noise = 0.05f; // deg/s
    pkt.gyro_data_hr.x = (fdm.pitchRate + gyro_noise * rand_float()) * 1.0e5; // deg/s*1.0e5
    pkt.gyro_data_hr.y = (fdm.rollRate + gyro_noise * rand_float()) * 1.0e5;  // deg/s*1.0e5
    pkt.gyro_data_hr.z = (-fdm.yawRate + gyro_noise * rand_float()) * 1.0e5;  // deg/s*1.0e5

    // 0x25 Barometer data
    float p, t_K;
    AP_Baro::get_pressure_temperature_for_alt_amsl(fdm.altitude+rand_float()*0.25, p, t_K);
    pkt.baro_data.pressure_pa2 = p * 0.5f; // Pa/2
    pkt.baro_data.baro_alt = fdm.altitude * 100.0f; // m*100

    // 0x24 Magnetometer data
    const float mag_noise = 100.0f; // nT
    pkt.mag_data.x = ((fdm.bodyMagField.y / NTESLA_TO_MGAUSS) + mag_noise * rand_float()) * 0.1f;  // nT/10
    pkt.mag_data.y = ((fdm.bodyMagField.x / NTESLA_TO_MGAUSS) + mag_noise * rand_float()) * 0.1f;  // nT/10
    pkt.mag_data.z = ((-fdm.bodyMagField.z / NTESLA_TO_MGAUSS) + mag_noise * rand_float()) * 0.1f; // nT/10

    // 0x26 Sensors bias
    pkt.sensor_bias.gyroX = 0; // deg/s*0.5*1e4
    pkt.sensor_bias.gyroY = 0; // deg/s*0.5*1e4
    pkt.sensor_bias.gyroZ = 0; // deg/s*0.5*1e4
    pkt.sensor_bias.accX = 0; // g*0.5*1e5
    pkt.sensor_bias.accY = 0; // g*0.5*1e5
    pkt.sensor_bias.accZ = 0; // g*0.5*1e5
    pkt.sensor_bias.reserved = 0;

    // 0x07 Orientation angles
    float roll_rad, pitch_rad, yaw_rad, heading_deg;
    fdm.quaternion.to_euler(roll_rad, pitch_rad, yaw_rad);
    heading_deg = fmodf(degrees(yaw_rad), 360.0f);
    if (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    pkt.orientation_angles.roll = roll_rad * RAD_TO_DEG * 100.0f; // deg*100
    pkt.orientation_angles.pitch = pitch_rad * RAD_TO_DEG * 100.0f; // deg*100
    pkt.orientation_angles.yaw = heading_deg * 100.0f; // deg*100

    // 0x12 Velocities
    pkt.velocity.x = fdm.speedE * 100.0f;  // m/s*100
    pkt.velocity.y = fdm.speedN * 100.0f;  // m/s*100
    pkt.velocity.z = -fdm.speedD * 100.0f; // m/s*100

    // 0x10 Position
    pkt.position.lat = fdm.latitude * 1e7;  // deg*1.0e7
    pkt.position.lon = fdm.longitude * 1e7; // deg*1.0e7
    pkt.position.alt = fdm.altitude * 1e2;  // m*100

    // 0x53 Unit status word (USW)
    pkt.unit_status = 0; // INS data is valid

    // 0x28 Differential pressure
    pkt.differential_pressure = fdm.airspeed_raw_pressure[0] * 100.0f; // mbar*1.0e4

    // 0x86 True airspeed (TAS)
    pkt.true_airspeed = fdm.airspeed * AP_Baro::get_EAS2TAS_for_alt_amsl(fdm.altitude) * 100.0f; // m/s*100

    // 0x85 Calibrated airspeed (CAS)
    pkt.calibrated_airspeed = fdm.airspeed * 100.0f; // m/s*100

    // 0x8A Wind speed
    pkt.wind_speed.x = fdm.wind_ef.y * 100.0f; // m/s*100
    pkt.wind_speed.y = fdm.wind_ef.x * 100.0f; // m/s*100
    pkt.wind_speed.z = 1000.0f; // Air_speed_SF*1000

    // 0x8D ADU status
    pkt.air_data_status = 0; // ADU data is valid

    // 0x50 Supply voltage
    pkt.supply_voltage = 12.3f * 100.0f; // VDC*100

    // 0x52 Temperature
    pkt.temperature = KELVIN_TO_C(t_K) * 10; // degC

    // 0x5A Unit status word (USW2)
    pkt.unit_status2 = 0;
    if (!gnss_input_enabled) {
        pkt.unit_status2 |= USW2_GNSS_INPUT_OFF;
    }
    if (!gnss_available) {
        pkt.unit_status2 |= USW2_GNSS_POS_INVALID;
    }

    // 0x54 INS (Navigation) solution status
    pkt.ins_sol_status = gnss_available ? NSS_GOOD : NSS_AUTONOMOUS;

    // 0x5f INS position and velocity accuracy
    pkt.ins_accuracy.lat = 100; // m*1000
    pkt.ins_accuracy.lon = 100; // m*1000
    pkt.ins_accuracy.alt = 100; // m*1000
    pkt.ins_accuracy.east_vel = 10; // m/s*1000
    pkt.ins_accuracy.north_vel = 10; // m/s*1000
    pkt.ins_accuracy.ver_vel = 10; // m/s*1000

    // 0x65 New aiding data, the external data items keep the last aiding data received
    pkt.new_aiding_data = received_aiding_data;
    received_aiding_data = 0;

    // 0x9A
    pkt.mag_clb_accuracy = 0;

    // 0x8C Outside air temperature (OAT)
    pkt.outside_air_temperature = KELVIN_TO_C(t_K) * 100; // degC*100

    pkt.gnss_new_data = 0;

    if (packets_sent % gps_frequency == 0) {
        // 0x3C GPS week
        pkt.gps_week = gps_tow.week;

        // 0x4A GNSS extended info
        pkt.gnss_extended_info.fix_type = gnss_available ? 2 : 0; // 3D fix or no fix
        pkt.gnss_extended_info.spoofing_status = 1; // no spoofing indicated

        if (gnss_available) {
            // 0x30 GNSS Position
            pkt.gnss_position.lat = fdm.latitude * 1e7;  // deg*1.0e7
            pkt.gnss_position.lon = fdm.longitude * 1e7; // deg*1.0e7
            pkt.gnss_position.alt = fdm.altitude * 1e2;  // m*100

            // 0x32 GNSS Velocity, Track over ground
            Vector2d track{fdm.speedN, fdm.speedE};
            pkt.gnss_vel_track.hor_speed = norm(fdm.speedN, fdm.speedE) * 100.0f; // m/s*100
            pkt.gnss_vel_track.track_over_ground = wrap_360(degrees(track.angle())) * 100.0f; // deg*100
            pkt.gnss_vel_track.ver_speed = -fdm.speedD * 100.0f; // m/s*100

            // 0x3E GNSS Position timestamp
            pkt.gnss_pos_timestamp = gps_tow.ms;
        }

        // 0x41 New GPS, the receiver keeps reporting also without a fix
        pkt.gnss_new_data = NEW_GNSS_POSITION | NEW_GNSS_VELOCITY;

        // 0xС0 u-blox jamming status
        pkt.gnss_jam_status = 1; // ok (no significant jamming)

        // 0x42 Dilution of precision
        const uint16_t dop = gnss_available ? 1000 : UINT16_MAX; // *1.0e3
        pkt.gnss_dop.gdop = dop;
        pkt.gnss_dop.pdop = dop;
        pkt.gnss_dop.hdop = dop;
        pkt.gnss_dop.vdop = dop;
        pkt.gnss_dop.tdop = dop;

        // 0x37 Full satellites info, no satellites are used when the GNSS data input is off
        pkt.full_sat_info.SVs = gnss_available ? 32 : 0;
        pkt.full_sat_info.SolnSVs = (gnss_available && gnss_input_enabled) ? 32 : 0;
        pkt.full_sat_info.SolnL1SVs = 0;
        pkt.full_sat_info.SolnMultiSVs = 0;
        pkt.full_sat_info.signal_used1 = 127;
        pkt.full_sat_info.signal_used2 = 51;
        pkt.full_sat_info.GPS_time_status = 180;
        pkt.full_sat_info.ext_sol_status = 0;

        // 0x3D GNSS Velocity latency
        pkt.gnss_vel_latency = 5; // ms

        // 0x38 GNSS Solution status
        pkt.gnss_sol_status = gnss_available ? GNSS_SOL_COMPUTED : GNSS_SOL_INSUFFICIENT_OBS;

        // 0x39 GNSS Position or Velocity type
        pkt.gnss_pos_vel_type = 16; // single point position
    }

    const uint8_t *buffer = (const uint8_t *)&pkt;
    pkt.crc = crc_sum_of_bytes_16(&buffer[2], sizeof(pkt)-4);

    // a packet that does not fit is dropped whole, as a truncated packet would corrupt the stream
    if (to_autopilot->space() >= sizeof(pkt)) {
        write_to_autopilot((char *)&pkt, sizeof(pkt));
    }

    packets_sent++;
}

/*
  handle commands and aiding data received from the autopilot
 */
void InertialLabs::receive_data(void)
{
    const auto consume = [this](uint8_t n) {
        memmove(rx_buf, &rx_buf[n], rx_len - n);
        rx_len -= n;
    };

    while (true) {
        const ssize_t n = read_from_autopilot((char *)&rx_buf[rx_len], sizeof(rx_buf) - rx_len);
        if (n <= 0) {
            return;
        }
        rx_len += n;

        // ILPP packet: 0xAA 0x55, message type, message ID, length (packet length - 2), payload, checksum
        while (rx_len >= 6) {
            const uint16_t packet_len = (rx_buf[4] | (rx_buf[5] << 8)) + 2;
            if (rx_buf[0] != 0xAA || rx_buf[1] != 0x55 || packet_len < 9 || packet_len > sizeof(rx_buf)) {
                consume(1);
                continue;
            }
            if (rx_len < packet_len) {
                break;
            }
            const uint16_t checksum = rx_buf[packet_len - 2] | (rx_buf[packet_len - 1] << 8);
            if (crc_sum_of_bytes_16(&rx_buf[2], packet_len - 4) != checksum) {
                consume(1);
                continue;
            }
            handle_packet(rx_buf[2], rx_buf[3], &rx_buf[6], packet_len - 8);
            consume(packet_len);
        }
    }
}

void InertialLabs::handle_packet(uint8_t msg_type, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len)
{
    if (msg_type == 0x00 && msg_id == 0x00 && payload_len == 1) {
        // command (ICD 5.3), the simulated device always outputs UDD set 1
        switch (payload[0]) {
        case 0xFE: // Stop_cmd
            streaming = false;
            break;
        case 0xFD: // Start_cmd
        case 0x95: // User_Def_Data_1..4 startup commands
        case 0xC5:
        case 0xC8:
        case 0xCB:
            streaming = true;
            break;
        case 0x71: // GNSS data input on
            gnss_input_enabled = true;
            break;
        case 0x72: // GNSS data input off
            gnss_input_enabled = false;
            break;
        }
        return;
    }
    if (msg_type == 0x01 && msg_id == 0x62 && payload_len >= 2 && payload[0] == 1) {
        // aiding data packet with a single data item: count, type, data (ICD 6.5.1)
        handle_aiding_data(payload[1], &payload[2], payload_len - 2);
    }
}

/*
  store aiding data, reported back in the corresponding UDD data items
 */
void InertialLabs::handle_aiding_data(uint8_t type, const uint8_t *data, uint16_t len)
{
    switch (type) {
    case 0x02: // airspeed
        if (copy_aiding_data(&pkt.external_speed, sizeof(pkt.external_speed), data, len)) {
            received_aiding_data |= NAD_AIRSPEED;
        }
        break;
    case 0x03: // wind
        if (copy_aiding_data(&pkt.ext_wind_data, sizeof(pkt.ext_wind_data), data, len)) {
            received_aiding_data |= NAD_WIND;
        }
        break;
    case 0x04: { // position, reported in the horizontal position and altitude items
        struct PACKED {
            int32_t lat, lon, alt;
            uint16_t lat_std, lon_std, alt_std, latency;
        } pos;
        if (copy_aiding_data(&pos, sizeof(pos), data, len)) {
            pkt.ext_hor_pos.lat = pos.lat;
            pkt.ext_hor_pos.lon = pos.lon;
            pkt.ext_hor_pos.lat_std = pos.lat_std;
            pkt.ext_hor_pos.lon_std = pos.lon_std;
            pkt.ext_hor_pos.pos_latency = pos.latency;
            pkt.ext_alt.alt = pos.alt;
            pkt.ext_alt.alt_std = pos.alt_std;
            received_aiding_data |= NAD_EXT_POSITION;
        }
        break;
    }
    case 0x06: // heading
        if (copy_aiding_data(&pkt.ext_heading, sizeof(pkt.ext_heading), data, len)) {
            received_aiding_data |= NAD_EXT_HEADING;
        }
        break;
    case 0x0B: // ambient air data
        if (copy_aiding_data(&pkt.ext_ambient_data, sizeof(pkt.ext_ambient_data), data, len)) {
            received_aiding_data |= NAD_AMBIENT_AIR;
        }
        break;
    case 0x0C: // altitude
        if (copy_aiding_data(&pkt.ext_alt, sizeof(pkt.ext_alt), data, len)) {
            received_aiding_data |= NAD_EXT_ALTITUDE;
        }
        break;
    case 0x0E: // horizontal position
        if (copy_aiding_data(&pkt.ext_hor_pos, sizeof(pkt.ext_hor_pos), data, len)) {
            received_aiding_data |= NAD_EXT_HOR_POSITION;
        }
        break;
    }
}

/*
  receive commands and send InertialLabs data
 */
void InertialLabs::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }
    if (!gps_enable_checked) {
        // the INS GNSS receiver is simulated GPS1, enabled by default as in SIM_GPS
        if (!_sitl->gps[0].enabled.configured()) {
            _sitl->gps[0].enabled.set(1);
        }
        gps_enable_checked = true;
    }

    receive_data();

    const uint32_t us_between_packets = 1000000UL / pkt_rate_hz;
    const uint32_t now = AP_HAL::micros();
    if (now - last_pkt_us < us_between_packets) {
        return;
    }
    // keep the average rate independent of the simulation step, without a burst after a pause
    last_pkt_us = (now - last_pkt_us < 2 * us_between_packets) ? last_pkt_us + us_between_packets : now;

    if (streaming) {
        send_packet();
    }
}
