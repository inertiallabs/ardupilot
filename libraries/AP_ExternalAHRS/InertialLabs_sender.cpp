#include "InertialLabs_sender.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include <AP_Airspeed/AP_Airspeed.h>
#include <GCS_MAVLink/GCS.h>

#include "InertialLabs_command.h"
#include "InertialLabs_data.h"
#include "InertialLabs_sensor.h"

namespace InertialLabs {

namespace {

struct StatusMessage {
	unsigned int status;
	MAV_SEVERITY severity;
	const char* message_true;
	const char* message_false;
};

const StatusMessage usw_message_list[] = {
	{ INITIAL_ALIGNMENT_FAIL, MAV_SEVERITY_CRITICAL, "Unsuccessful initial alignment",        "Initial alignment is OK"          },
	{ OPERATION_FAIL,         MAV_SEVERITY_CRITICAL, "IMU data are incorrect",                "IMU data are correct"             },
	{ GYRO_FAIL,              MAV_SEVERITY_CRITICAL, "Gyros failure",                         "Gyros is OK"                      },
	{ ACCEL_FAIL,             MAV_SEVERITY_CRITICAL, "Accelerometers failure",                "Accelerometers is OK"             },
	{ MAG_FAIL,               MAV_SEVERITY_CRITICAL, "Magnetometers failure",                 "Magnetometers is OK"              },
	{ ELECTRONICS_FAIL,       MAV_SEVERITY_CRITICAL, "Electronics failure",                   "Electronics is OK"                },
	{ GNSS_FAIL,              MAV_SEVERITY_CRITICAL, "GNSS receiver failure",                 "GNSS receiver is OK"              },
	{ VOLTAGE_LOW,            MAV_SEVERITY_WARNING,  "Low input voltage",                     "Input voltage is in range"        },
	{ VOLTAGE_HIGH,           MAV_SEVERITY_WARNING,  "High input voltage",                    "Input voltage is in range"        },
	{ GYRO_X_RATE_HIGH,       MAV_SEVERITY_INFO,     "Y-axis angular rate is exceeded",       "Y-axis angular rate is in range"  },
	{ GYRO_Y_RATE_HIGH,       MAV_SEVERITY_INFO,     "X-axis angular rate is exceeded",       "X-axis angular rate is in range"  },
	{ GYRO_Z_RATE_HIGH,       MAV_SEVERITY_INFO,     "Z-axis angular rate is exceeded",       "Z-axis angular rate is in range"  },
	{ MAG_FIELD_HIGH,         MAV_SEVERITY_INFO,     "Large total magnetic field",            "Total magnetic field is in range" },
	{ TEMP_RANGE_ERR,         MAV_SEVERITY_INFO,     "Temperature is out of range",           "Temperature is in range"          }
};

const StatusMessage usw2_message_list[] = {
	{ ACCEL_X_HIGH,            MAV_SEVERITY_INFO,     "Y-axis acceleration is out of range",     "Y-axis acceleration is in range"     },
	{ ACCEL_Y_HIGH,            MAV_SEVERITY_INFO,     "X-axis acceleration is out of range",     "X-axis acceleration is in range"     },
	{ ACCEL_Z_HIGH,            MAV_SEVERITY_INFO,     "Z-axis acceleration is out of range",     "Z-axis acceleration is in range"     },
	{ ADU_BARO_FAIL,           MAV_SEVERITY_CRITICAL, "Baro altimeter failure",                  "Baro altimeter is OK"                },
	{ ADU_DIFF_PRESS_FAIL,     MAV_SEVERITY_CRITICAL, "Diff. pressure sensor failure",           "Diff. pressure sensor is OK"         },
	{ MAG_AUTO_CAL_2D_RUNTIME, MAV_SEVERITY_INFO,     "Automatic 2D calibration is in progress", "Automatic 2D calibration is stopped" },
	{ MAG_AUTO_CAL_3D_RUNTIME, MAV_SEVERITY_INFO,     "Automatic 3D calibration is in progress", "Automatic 3D calibration is stopped" },
	{ GNSS_FUSION_OFF,         MAV_SEVERITY_INFO,     "GNSS input switched off",                 "GNSS input switched on"              },
	{ DIFF_PRESS_FUSION_OFF,   MAV_SEVERITY_INFO,     "Diff. pressure input switched off",       "Diff. pressure input switched on"    },
	{ GNSS_POS_VALID,          MAV_SEVERITY_INFO,     "Incorrect GNSS position",                 "GNSS position is correct"            }
};

const StatusMessage adu_message_list[] = {
	{ BARO_INIT_FAIL,           MAV_SEVERITY_WARNING, "Static pressure sensor unsuccessful initialization", "Static pressure sensor initialization successful"},
	{ DIFF_PRESS_INIT_FAIL,     MAV_SEVERITY_WARNING, "Diff. pressure sensor unsuccessful initialization",  "Diff. pressure sensor initialization successful"},
	{ BARO_RANGE_ERR,           MAV_SEVERITY_INFO,    "Static pressure is out of range",                    "Static pressure is in range"      },
	{ DIFF_PRESS_RANGE_ERR,     MAV_SEVERITY_INFO,    "Diff. pressure is out of range",                     "Diff. pressure is in range"       },
	{ BARO_ALT_FAIL,            MAV_SEVERITY_WARNING, "Pressure altitude is incorrect",                     "Pressure altitude is correct"     },
	{ AIRSPEED_FAIL,            MAV_SEVERITY_WARNING, "Air speed is incorrect",                             "Air speed is correct"             },
	{ AIRSPEED_BELOW_THRESHOLD, MAV_SEVERITY_INFO,    "Air speed is below the threshold",                   "Air speed is above the threshold" }
};

constexpr size_t usw_message_list_size = sizeof(usw_message_list) / sizeof(StatusMessage);
constexpr size_t usw2_message_list_size = sizeof(usw2_message_list) / sizeof(StatusMessage);
constexpr size_t adu_message_list_size = sizeof(adu_message_list) / sizeof(StatusMessage);

static_assert(usw_message_list_size == USW_MESSAGE_LIST_SIZE,
              "USW_MESSAGE_LIST_SIZE must match usw_message_list");
static_assert(usw2_message_list_size == USW2_MESSAGE_LIST_SIZE,
              "USW2_MESSAGE_LIST_SIZE must match usw2_message_list");
static_assert(adu_message_list_size == ADU_MESSAGE_LIST_SIZE,
              "ADU_MESSAGE_LIST_SIZE must match adu_message_list");

void send_EAHRS_state_msg(uint32_t package_timestamp_ms,
                          uint16_t last_state_list,
                          uint16_t current_state_list,
                          const StatusMessage* message_list,
                          const size_t message_list_size,
                          uint64_t* message_timestamp_list)
{
    const uint64_t send_critical_messaged_delay = 10000;

    for (size_t i = 0; i < message_list_size; i++) {
        const bool current_status = current_state_list & message_list[i].status;
        const bool last_status = last_state_list & message_list[i].status;
        const bool is_status_changed = current_status != last_status;

        if (message_list[i].severity == MAV_SEVERITY_CRITICAL &&
                current_status &&
                (is_status_changed || (package_timestamp_ms - message_timestamp_list[i] > send_critical_messaged_delay))) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: %s", message_list[i].message_true);
            message_timestamp_list[i] = package_timestamp_ms;
            continue;
        }

        if (is_status_changed) {
            GCS_SEND_TEXT(message_list[i].severity,
                          "ILAB: %s",
                          current_status ? message_list[i].message_true : message_list[i].message_false);
        }
    }
}

} // namespace

void Sender::send_gcs_messages(const SensorsData &sensors_data)
{
    const uint32_t package_timestamp_ms = static_cast<uint32_t>(sensors_data.package_timestamp_us / 1000);

    // USW
    if (sensors_data.ins.unit_status != state.last_unit_status) {
        send_EAHRS_state_msg(package_timestamp_ms,
                             state.last_unit_status,
                             sensors_data.ins.unit_status,
                             usw_message_list,
                             usw_message_list_size,
                             state.usw_message_timestamp_list_ms); // IL INS Unit Status Word (USW) messages
        state.last_unit_status = sensors_data.ins.unit_status;

        // Magnetometers calibration
        if ((sensors_data.ins.unit_status & USW::MAG_VG3D_CLB_RUNTIME) != 0) {
            if ((state.last_mag_clb_status & (1 << 0)) == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: VG3D Mag calibration data accumulation");
                state.last_mag_clb_status |= (1 << 0); // set bit: VG3D mag calibration data is accumulated
            } else {
                state.last_mag_clb_status |= (1 << 1); // set bit: VG3D mag calibration parameters are calculated
                state.last_mag_clb_status &= ~(1 << 0);
                state.last_mag_ms = package_timestamp_ms;
            }
        }

        if ((sensors_data.ins.unit_status & USW::MAG_VG3D_CLB_SUCCESS) != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: VG3D Mag calibration successful");
            state.last_mag_clb_status |= (1 << 2); // set bit: VG3D mag calibration accuracy estimation in progress
            state.last_mag_clb_status &= ~(1 << 1);
        }

        if ((state.last_mag_clb_status & (1 << 1)) != 0 && (state.last_unit_status & USW::MAG_VG3D_CLB_RUNTIME) == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: VG3D Mag calibration unsuccessful");
            state.last_mag_clb_status &= ~(1 << 1);
        }
    }

    // Magnetometers accuracy
    if ((state.last_mag_clb_status & (1 << 2)) != 0) {
        const bool is_time_exceeded = (package_timestamp_ms > state.last_mag_ms) && (package_timestamp_ms - state.last_mag_ms > 10000U);
        const bool is_accuracy_changed = sensors_data.ins.mag_clb_accuracy != state.last_mag_clb_accuracy;
        if (is_time_exceeded || is_accuracy_changed) {
            if (sensors_data.ins.mag_clb_accuracy == 255) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: INS cannot estimate heading accuracy");
            } else if (sensors_data.ins.mag_clb_accuracy != 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: Predicted heading error is %.1f deg", static_cast<float>(sensors_data.ins.mag_clb_accuracy)*0.1f);
            }
            state.last_mag_clb_status &= ~(1 << 2);
            state.last_mag_clb_accuracy = sensors_data.ins.mag_clb_accuracy;
        }
    }

    // USW2
    if (sensors_data.ins.unit_status2 != state.last_unit_status2) {
        send_EAHRS_state_msg(package_timestamp_ms,
                             state.last_unit_status2,
                             sensors_data.ins.unit_status2,
                             usw2_message_list,
                             usw2_message_list_size,
                             state.usw2_message_timestamp_list_ms); // IL INS Unit Status Word 2 (USW2) messages
        state.last_unit_status2 = sensors_data.ins.unit_status2;
    }

    // Air Data
    if (sensors_data.ins.air_data_status != state.last_air_data_status) {
        send_EAHRS_state_msg(package_timestamp_ms,
                             state.last_air_data_status,
                             sensors_data.ins.air_data_status,
                             adu_message_list,
                             adu_message_list_size,
                             state.adu_message_timestamp_list_ms); // IL Air Data Unit (ADU) messages
        state.last_air_data_status = sensors_data.ins.air_data_status;
    }

    // Spoofing
    if (state.last_spoof_status != sensors_data.gps.spoof_status) {
        // IL INS spoofing detection messages
        if ((state.last_spoof_status == 2 || state.last_spoof_status == 3) && (sensors_data.gps.spoof_status == 1)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS no spoofing");
        }

        if (state.last_spoof_status == 2) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS spoofing indicated");
        }

        if (state.last_spoof_status == 3) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS multiple spoofing indicated");
        }

        state.last_spoof_status = sensors_data.gps.spoof_status;
    }

    // Jamming
    if (state.last_jam_status != sensors_data.gps.jam_status) {
        // IL INS jamming detection messages
        if ((state.last_jam_status == 3) && (sensors_data.gps.jam_status == 1)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS no jamming");
        }

        if (sensors_data.gps.jam_status == 3) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: GNSS jamming indicated and no fix");
        }

        state.last_jam_status = sensors_data.gps.jam_status;
    }

    // INS Solution
    if (state.last_ins_sol_status != sensors_data.ins.ins_sol_status) {
        // IL INS navigation solution status messages
        if ((state.last_ins_sol_status == 4 ||
                state.last_ins_sol_status == 6 ||
                state.last_ins_sol_status == 8) &&
                sensors_data.ins.ins_sol_status == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: INS solution is good");
        }

        if ((sensors_data.ins.ins_sol_status) == 4 && (state.last_ins_sol_status == 0)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: INS is operating in autonomous mode");
        }

        if (sensors_data.ins.ins_sol_status == 6) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ILAB: INS froze position and velocity");
        }

        if (sensors_data.ins.ins_sol_status == 8) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ILAB: INS solution is invalid");
        }

        state.last_ins_sol_status = sensors_data.ins.ins_sol_status;
    }
}

void Sender::send_gcs_eahrs_status_flags(GCS_MAVLINK &link, const SensorsData &sensors_data) const
{
    const mavlink_eahrs_status_info_t package{sensors_data.ins.unit_status,
                                              sensors_data.ins.unit_status2,
                                              sensors_data.ins.air_data_status,
                                              (uint16_t)sensors_data.gps.fix_type,
                                              sensors_data.gps.spoof_status};
    mavlink_msg_eahrs_status_info_send_struct(link.get_chan(), &package);
}

void Sender::send_sensor_airspeed_aiding_data(Sensor &sensor) const
{
#if AP_AIRSPEED_ENABLED
    const AP_Airspeed* airspeed = AP_Airspeed::get_singleton();
    if (!airspeed) {
        return;
    }

    Data_context context{};
    ExternalAHRS_command_data data{};

    data.param1 = floorf(airspeed->get_airspeed() * 1.94384449f * 100.0f + 0.5f); // in 0.01 kt
    if (data.param1 > 32767.0f)
    {
        data.param1 = 32767.0f;
    }
    else if (data.param1 < -32768.0f)
    {
        data.param1 = -32768.0f;
    }

    fill_command_payload(context,
                         ExternalAHRS_command::AIDING_DATA_AIR_SPEED,
                         data);
    fill_transport_protocol_data(context);
    sensor.write_bytes(reinterpret_cast<const char *>(context.data), context.length);
#endif // AP_AIRSPEED_ENABLED
}

void Sender::send_sensor_command(Sensor &sensor,
                         ExternalAHRS_command command,
                         const ExternalAHRS_command_data &data) const
{
    switch (command) {
        case ExternalAHRS_command::START_UDD:
            sensor.write_bytes(InertialLabs::Command::START_UDD,
                               sizeof(InertialLabs::Command::START_UDD) - 1);
            break;
        case ExternalAHRS_command::STOP:
            sensor.write_bytes(InertialLabs::Command::STOP,
                               sizeof(InertialLabs::Command::STOP) - 1);
            break;
        case ExternalAHRS_command::ENABLE_GNSS:
            sensor.write_bytes(InertialLabs::Command::ENABLE_GNSS,
                               sizeof(InertialLabs::Command::ENABLE_GNSS) - 1);
            break;
        case ExternalAHRS_command::DISABLE_GNSS:
            sensor.write_bytes(InertialLabs::Command::DISABLE_GNSS,
                               sizeof(InertialLabs::Command::DISABLE_GNSS) - 1);
            break;
        case ExternalAHRS_command::START_VG3D_CALIBRATION_IN_FLIGHT:
            sensor.write_bytes(InertialLabs::Command::START_VG3DCLB_FLIGHT,
                               sizeof(InertialLabs::Command::START_VG3DCLB_FLIGHT) - 1);
            break;
        case ExternalAHRS_command::STOP_VG3D_CALIBRATION_IN_FLIGHT:
            sensor.write_bytes(InertialLabs::Command::STOP_VG3DCLB_FLIGHT,
                               sizeof(InertialLabs::Command::STOP_VG3DCLB_FLIGHT) - 1);
            break;
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_POSITION:
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_HORIZONTAL_POSITION:
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_ALTITUDE:
        case ExternalAHRS_command::AIDING_DATA_WIND:
        case ExternalAHRS_command::AIDING_DATA_AMBIENT_AIR:
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_HEADING:
        case ExternalAHRS_command::AIDING_DATA_DVL:
        {
            InertialLabs::Data_context context;
            InertialLabs::fill_command_payload(context, command, data);
            InertialLabs::fill_transport_protocol_data(context);
            sensor.write_bytes(reinterpret_cast<const char *>(context.data), context.length);
            break;
        }

        default:
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ILAB: Invalid command for handling");
    }
}

} // namespace InertialLabs

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED