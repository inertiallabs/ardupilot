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

InertialLabs::InertialLabs() : SerialDevice::SerialDevice()
{
}

void InertialLabs::send_packet(void)
{
    const auto &fdm = _sitl->state;

    pkt.msg_len = sizeof(pkt)-2;

    const auto gps_tow = GPS_Backend::gps_time();

    // 0x01 GPS INS Time (round)
    pkt.gps_time_ms = gps_tow.ms;

    // 0x23 Accelerometer data HR
    const float accel_noise = 0.005f; // g
    pkt.accel_data_hr.x = ((fdm.yAccel / GRAVITY_MSS) + accel_noise * rand_float()) * 1.0e6;  // g*1.0e6
    pkt.accel_data_hr.y = ((fdm.xAccel / GRAVITY_MSS) + accel_noise * rand_float()) * 1.0e6;  // g*1.0e6
    pkt.accel_data_hr.z = ((-fdm.zAccel / GRAVITY_MSS) + accel_noise * rand_float()) * 1.0e6; // g*1.0e6

    // 0x21 Gyro data HR
    const float gyro_noise = 0.05f; // deg/s
    pkt.gyro_data_hr.x = (fdm.pitchRate + gyro_noise * rand_float()) * 1.0e5; // deg/s*1.0e5
    pkt.gyro_data_hr.y = (fdm.rollRate + gyro_noise * rand_float()) * 1.0e5;  // deg/s*1.0e5
    pkt.gyro_data_hr.z = (-fdm.yawRate + gyro_noise * rand_float()) * 1.0e5;  // deg/s*1.0e5

    // 0x25 Barometer data
    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere((fdm.altitude + rand_float() * 0.25f) * 0.001f, sigma, delta, theta);
    pkt.baro_data.pressure_pa2 = SSL_AIR_PRESSURE * delta * 0.5f; // Pa/2
    pkt.baro_data.baro_alt = fdm.altitude * 100.0f; // m*100

    // 0x24 Magnetometer data
    const float mag_noise = 100.0f; // nT
    pkt.mag_data.x = ((fdm.bodyMagField.y / NTESLA_TO_MGAUSS) + mag_noise * rand_float()) * 0.1f;  // nT/10
    pkt.mag_data.y = ((fdm.bodyMagField.x / NTESLA_TO_MGAUSS) + mag_noise * rand_float()) * 0.1f;  // nT/10
    pkt.mag_data.z = ((-fdm.bodyMagField.z / NTESLA_TO_MGAUSS) + mag_noise * rand_float()) * 0.1f; // nT/10

    // 0x26 Sensors bias
    pkt.sensor_bias.gyroX = 0; // deg/s*0.5*10e4
    pkt.sensor_bias.gyroY = 0; // deg/s*0.5*10e4
    pkt.sensor_bias.gyroZ = 0; // deg/s*0.5*10e4
    pkt.sensor_bias.accX = 0; // g*0.5*10e5
    pkt.sensor_bias.accY = 0; // g*0.5*10e5
    pkt.sensor_bias.accZ = 0; // g*0.5*10e5
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
    pkt.true_airspeed = fdm.airspeed * 100.0f; // m/s*100

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
    pkt.temperature = (KELVIN_TO_C(SSL_AIR_TEMPERATURE * theta) + 25.0f) * 10.0f; // degC*10

    // 0x5A Unit status word (USW2)
    pkt.unit_status2 = 0; // INS data is valid

    // 0x54 INS (Navigation) solution status
    pkt.ins_sol_status = 0; // INS solution is good

    // 0x5f INS position and velocity accuracy
    pkt.ins_accuracy.lat = 100; // m*1000
    pkt.ins_accuracy.lon = 100; // m*1000
    pkt.ins_accuracy.alt = 100; // m*1000
    pkt.ins_accuracy.east_vel = 10; // m/s*1000
    pkt.ins_accuracy.north_vel = 10; // m/s*1000
    pkt.ins_accuracy.ver_vel = 10; // m/s*1000

    // 0x65 New aiding data
    pkt.new_aiding_data = 0;

    // 0xA1 New aiding data 2
    pkt.new_aiding_data2 = 0;

    // 0x61 External Air or ground speed
    pkt.external_speed = 0; // kt*100

    // 0x6E External horizontal position
    pkt.ext_hor_pos.lat = 0; // deg*1.0e7
    pkt.ext_hor_pos.lon = 0; // deg*1.0e7
    pkt.ext_hor_pos.lat_std = 0; // m*100
    pkt.ext_hor_pos.lon_std = 0; // m*100
    pkt.ext_hor_pos.pos_latency = 0; // sec*1000

    // 0x6C Altitude external
    pkt.ext_alt.alt = 0; // m*1000
    pkt.ext_alt.alt_std = 0; // m*100

    // 0x66 Heading external
    pkt.ext_heading.heading = 0; // deg*100
    pkt.ext_heading.std = 0; // deg*100
    pkt.ext_heading.latency = 0; // sec*1000

    // 0x6B External ambient air data
    pkt.ext_ambient_data.air_temp = 0; // degC*10
    pkt.ext_ambient_data.alt = 0; // m*100
    pkt.ext_ambient_data.abs_press = 0; // Pa/2

    // 0x62 External wind data
    pkt.ext_wind_data.n_wind_vel = 0; // kt*100
    pkt.ext_wind_data.e_wind_vel = 0; // kt*100
    pkt.ext_wind_data.n_std_wind = 0; // kt*100
    pkt.ext_wind_data.e_std_wind = 0; // kt*100

    // 0x9A
    pkt.mag_clb_accuracy = 0;

    if (packets_sent % gps_frequency == 0) {
        // 0x3C GPS week
        pkt.gps_week = gps_tow.week;

        // 0x4A GNSS extended info
        pkt.gnss_extended_info.fix_type = 2; // 3D fix
        pkt.gnss_extended_info.spoofing_status = 1; // no spoofing indicated

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

        // 0x41 New GPS
        pkt.gnss_new_data = 127; // GNSS data updated

        // 0xС0 u-blox jamming status
        pkt.gnss_jam_status = 1; // ok (no significant jamming)

        // 0x42 Dilution of precision
        pkt.gnss_dop.gdop = 1000; // *1.0e3
        pkt.gnss_dop.pdop = 1000; // *1.0e3
        pkt.gnss_dop.hdop = 1000; // *1.0e3
        pkt.gnss_dop.vdop = 1000; // *1.0e3
        pkt.gnss_dop.tdop = 1000; // *1.0e3

        // 0x37 Full satellites info
        pkt.full_sat_info.SVs = 32;
        pkt.full_sat_info.SolnSVs = 32;
        pkt.full_sat_info.SolnL1SVs = 0;
        pkt.full_sat_info.SolnMultiSVs = 0;
        pkt.full_sat_info.signal_used1 = 127;
        pkt.full_sat_info.signal_used2 = 51;
        pkt.full_sat_info.GPS_time_status = 180;
        pkt.full_sat_info.ext_sol_status = 0;

        // 0x3D GNSS Velocity latency
        pkt.gnss_vel_latency = 5; // ms

        // 0x38 GNSS Solution status
        pkt.gnss_sol_status = 0; // solution computed

        // 0x39 GNSS Position or Velocity type
        pkt.gnss_pos_vel_type = 16; // single point position
    }

    const uint8_t *buffer = (const uint8_t *)&pkt;
    pkt.crc = crc_sum_of_bytes_16(&buffer[2], sizeof(pkt)-4);

    write_to_autopilot((char *)&pkt, sizeof(pkt));

    packets_sent++;
}

/*
  send InertialLabs data
 */
void InertialLabs::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }
    const uint32_t us_between_packets = 5000; // 200Hz
    const uint32_t now = AP_HAL::micros();
    if (now - last_pkt_us >= us_between_packets) {
        last_pkt_us = now;
        send_packet();
    }

}