#include "InertialLabs_logs.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <AP_Logger/AP_Logger.h>

#include "InertialLabs_data.h"

namespace InertialLabs {

void write_logs(const SensorsData &sensors_data)
{
    using InertialLabs::NewAidingData;

#if HAL_LOGGING_ENABLED

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
                                "s-EEEoooGGG",
                                "F----------",
                                "QIfffffffff",
                                sensors_data.package_timestamp_us, sensors_data.ins.ms_tow,
                                sensors_data.gyro.x, sensors_data.gyro.y, sensors_data.gyro.z,
                                sensors_data.accel.x, sensors_data.accel.y, sensors_data.accel.z,
                                sensors_data.mag.x, sensors_data.mag.y, sensors_data.mag.z);

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
                                sensors_data.package_timestamp_us, sensors_data.ins.ms_tow,
                                static_cast<float>(sensors_data.ins.sensor_bias.gyroY)*2.0f*1.0e-4f,
                                static_cast<float>(sensors_data.ins.sensor_bias.gyroX)*2.0f*1.0e-4f,
                                static_cast<float>(sensors_data.ins.sensor_bias.gyroZ)*2.0f*1.0e-4f*(-1.0f),
                                static_cast<float>(sensors_data.ins.sensor_bias.accY)*2.0f*1.0e-5f,
                                static_cast<float>(sensors_data.ins.sensor_bias.accX)*2.0f*1.0e-5f,
                                static_cast<float>(sensors_data.ins.sensor_bias.accZ)*2.0f*1.0e-5f*(-1.0f));

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

    AP::logger().WriteStreaming("ILB2", "TimeUS,IMS,Press,Diff,Temp,Alt,TAS,CAS,VWN,VWE,ArspSF",
                                "s-PPOmnnnn-",
                                "F----------",
                                "QIfffffffff",
                                sensors_data.package_timestamp_us, sensors_data.ins.ms_tow,
                                sensors_data.pressure, sensors_data.diff_press, sensors_data.temperature,
                                sensors_data.ins.baro_alt, sensors_data.ins.true_airspeed, sensors_data.ins.calibrated_airspeed,
                                sensors_data.ins.wind_speed.x, sensors_data.ins.wind_speed.y, sensors_data.ins.airspeed_sf);

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
    // @Field: Alt: altitude

    AP::logger().WriteStreaming("ILB3", "TimeUS,IMS,Roll,Pitch,Yaw,VN,VE,VD,Lat,Lng,Alt",
                                "s-dddnnnDUm",
                                "F----------",
                                "QIffffffddf",
                                sensors_data.package_timestamp_us, sensors_data.ins.ms_tow,
                                sensors_data.ins.roll, sensors_data.ins.pitch, sensors_data.ins.yaw,
                                sensors_data.ins.velocity.x, sensors_data.ins.velocity.y, sensors_data.ins.velocity.z,
                                static_cast<double>(sensors_data.ins.latitude)*1.0e-7,
                                static_cast<double>(sensors_data.ins.longitude)*1.0e-7,
                                static_cast<float>(sensors_data.ins.altitude)*0.01f);

    // @LoggerMessage: ILB9
    // @Description: InertialLabs service data
    // @Field: TimeUS: Time since system startup
    // @Field: IMS: GPS INS time (round)
    // @Field: USW: Unit Status Word
    // @Field: USW2: Unit Status Word 2
    // @Field: ADU: Air Data Unit status
    // @Field: ISS: INS Navigation (Solution) Status
    // @Field: NAD1: New Aiding Data
    // @Field: NAD2: New Aiding Data 2
    // @Field: Vdc: Supply voltage

    AP::logger().WriteStreaming("ILB9", "TimeUS,IMS,USW,USW2,ADU,ISS,NAD1,NAD2,Vdc",
                                "s-------v",
                                "F--------",
                                "QIHHHBHHf",
                                sensors_data.package_timestamp_us, sensors_data.ins.ms_tow, sensors_data.ins.unit_status, sensors_data.ins.unit_status2,
                                sensors_data.ins.air_data_status, sensors_data.ins.ins_sol_status,
                                sensors_data.ext.new_aiding_data, sensors_data.ext.new_aiding_data2, sensors_data.supply_voltage);

    if (sensors_data.gps.new_data != 0) {
        // @LoggerMessage: ILB4
        // @Description: InertialLabs GPS data1
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: GMS: GNSS Position timestamp
        // @Field: GWk: GPS Week
        // @Field: FType: fix type
        // @Field: NewGPS: Indicator of new update of GPS data
        // @Field: Lat: GNSS Latitude
        // @Field: Lng: GNSS Longitude
        // @Field: Alt: GNSS Altitude
        // @Field: GCrs: GNSS Track over ground
        // @Field: Spd: GNSS Horizontal speed
        // @Field: VZ: GNSS Vertical speed

        AP::logger().WriteStreaming("ILB4", "TimeUS,IMS,GMS,GWk,FType,NewGPS,Lat,Lng,Alt,GCrs,Spd,VZ",
                                    "s-----DUmhnn",
                                    "F-----------",
                                    "QIIHBBddffff",
                                    sensors_data.package_timestamp_us, sensors_data.ins.ms_tow, sensors_data.gps.ms_tow, sensors_data.gps.gps_week,
                                    sensors_data.gps.fix_type, sensors_data.gps.new_data,
                                    static_cast<double>(sensors_data.gps.latitude)*1.0e-7,
                                    static_cast<double>(sensors_data.gps.longitude)*1.0e-7,
                                    static_cast<float>(sensors_data.gps.altitude)*0.01f,
                                    sensors_data.gps.track_over_ground, sensors_data.gps.hor_speed, sensors_data.gps.ver_speed);

        // @LoggerMessage: ILB5
        // @Description: InertialLabs GPS data2
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: GSS: GNSS spoofing status
        // @Field: GJS: GNSS jamming status
        // @Field: VL: GNSS Velocity latency
        // @Field: SolS: GNSS Solution status
        // @Field: PVT: GNSS Position or Velocity type
        // @Field: GDOP: GNSS GDOP
        // @Field: PDOP: GNSS PDOP
        // @Field: HDOP: GNSS HDOP
        // @Field: VDOP: GNSS VDOP
        // @Field: TDOP: GNSS TDOP

        AP::logger().WriteStreaming("ILB5", "TimeUS,IMS,GSS,GJS,VL,SolS,PVT,GDOP,PDOP,HDOP,VDOP,TDOP",
                                    "s-----------",
                                    "F-----------",
                                    "QIBBHBBfffff",
                                    sensors_data.package_timestamp_us, sensors_data.ins.ms_tow,
                                    sensors_data.gps.spoof_status, sensors_data.gps.jam_status, sensors_data.gps.vel_latency,
                                    sensors_data.gps.gnss_sol_status, sensors_data.gps.gnss_pos_vel_type,
                                    static_cast<float>(sensors_data.gps.dop.gdop)*1.0e-3f,
                                    static_cast<float>(sensors_data.gps.dop.pdop)*1.0e-3f,
                                    static_cast<float>(sensors_data.gps.dop.hdop)*1.0e-3f,
                                    static_cast<float>(sensors_data.gps.dop.vdop)*1.0e-3f,
                                    static_cast<float>(sensors_data.gps.dop.tdop)*1.0e-3f);

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
        // @Field: GTimeS: GPS time status
        // @Field: SolS: Extended solution status

        AP::logger().WriteStreaming("ILB6", "TimeUS,IMS,SVs,SolSVs,SolL1,SolMult,SU1,SU2,GTimeS,SolS",
                                    "s---------",
                                    "F---------",
                                    "QIBBBBBBBB",
                                    sensors_data.package_timestamp_us, sensors_data.ins.ms_tow,
                                    sensors_data.gps.full_sat_info.SVs, sensors_data.gps.full_sat_info.SolnSVs,
                                    sensors_data.gps.full_sat_info.SolnL1SVs, sensors_data.gps.full_sat_info.SolnMultiSVs,
                                    sensors_data.gps.full_sat_info.signal_used1, sensors_data.gps.full_sat_info.signal_used2,
                                    sensors_data.gps.full_sat_info.GPS_time_status, sensors_data.gps.full_sat_info.ext_sol_status);
    }

    if ((sensors_data.ext.new_aiding_data & (NewAidingData::NEW_EXT_POS |
                                          NewAidingData::NEW_EXT_HOR_POS |
                                          NewAidingData::NEW_ALTITUDE |
                                          NewAidingData::NEW_HEADING)) != 0) {
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
                                    "QIddffffffff",
                                    sensors_data.package_timestamp_us, sensors_data.ins.ms_tow,
                                    static_cast<double>(sensors_data.ext.hor_pos.lat)*1.0e-7,
                                    static_cast<double>(sensors_data.ext.hor_pos.lon)*1.0e-7,
                                    static_cast<float>(sensors_data.ext.alt.alt)*1.0e-3f,
                                    static_cast<float>(sensors_data.ext.hor_pos.lat_std)*0.01f,
                                    static_cast<float>(sensors_data.ext.hor_pos.lon_std)*0.01f,
                                    static_cast<float>(sensors_data.ext.alt.alt_std)*0.01f,
                                    static_cast<float>(sensors_data.ext.hor_pos.pos_latency)*1.0e-3f,
                                    static_cast<float>(sensors_data.ext.heading.heading)*0.01f,
                                    static_cast<float>(sensors_data.ext.heading.std)*0.01f,
                                    static_cast<float>(sensors_data.ext.heading.latency)*1.0e-3f);
    }

    if ((sensors_data.ext.new_aiding_data & (NewAidingData::NEW_AMBIENT |
                                          NewAidingData::NEW_WIND |
                                          NewAidingData::NEW_AIRSPEED)) != 0) {
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
                                    sensors_data.package_timestamp_us, sensors_data.ins.ms_tow,
                                    static_cast<float>(sensors_data.ext.speed)*0.5144f*0.01f,
                                    static_cast<float>(sensors_data.ext.ambient_air_data.air_temp)*0.1f,
                                    static_cast<float>(sensors_data.ext.ambient_air_data.alt)*0.01f,
                                    static_cast<float>(sensors_data.ext.ambient_air_data.abs_press)*2.0f,
                                    static_cast<float>(sensors_data.ext.wind_data.e_wind_vel)*0.5144f*0.01f,
                                    static_cast<float>(sensors_data.ext.wind_data.n_wind_vel)*0.5144f*0.01f,
                                    static_cast<float>(sensors_data.ext.wind_data.e_std_wind)*0.5144f*0.01f,
                                    static_cast<float>(sensors_data.ext.wind_data.n_std_wind)*0.5144f*0.01f);
    }

    if (sensors_data.ext.new_aiding_data & NewAidingData::NEW_DVL) {
        // @LoggerMessage: ILBV
        // @Description: InertialLabs doppler velocity log data
        // @Field: TimeUS: Time since system startup
        // @Field: IMS: GPS INS time (round)
        // @Field: LV: Lateral velocity (m/sec)
        // @Field: FV: Forward velocity (m/sec)
        // @Field: VV: Vertical velocity (m/sec)
        // @Field: LVS: Lateral velocity STD (m/sec)
        // @Field: FVS: Forward velocity STD (m/sec)
        // @Field: VVS: Vertical velocity STD (m/sec)
        // @Field: VL: Velocity latency (ms)

        AP::logger().WriteStreaming("ILBV", "TimeUS,IMS,LV,FV,VV,LVS,FVS,VVS,VL",
                                    "s-nnnnnn-",
                                    "F--------",
                                    "QIffffffI",
                                    sensors_data.package_timestamp_us, sensors_data.ins.ms_tow,
                                    static_cast<float>(sensors_data.ext.doppler_velocity_log.lateralVel)*1.0e-3,
                                    static_cast<float>(sensors_data.ext.doppler_velocity_log.forwardVel)*1.0e-3,
                                    static_cast<float>(sensors_data.ext.doppler_velocity_log.verticalVel)*1.0e-3,
                                    static_cast<float>(sensors_data.ext.doppler_velocity_log.lateralVelStd)*1.0e-3,
                                    static_cast<float>(sensors_data.ext.doppler_velocity_log.forwardVelStd)*1.0e-3,
                                    static_cast<float>(sensors_data.ext.doppler_velocity_log.verticalVelStd)*1.0e-3,
                                    sensors_data.ext.doppler_velocity_log.velLatency);
    }
#endif  // HAL_LOGGING_ENABLED
}

} // namespace InertialLabs

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED