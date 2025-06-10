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
  support for serial connected InertialLabs INS system
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include "InertialLabs_data.h"
#include "InertialLabs_readDataEnum.h"
#include "InertialLabs_sender.h"
#include "InertialLabs_sensor.h"

class AP_ExternalAHRS_InertialLabs : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_InertialLabs(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);
    int8_t get_port() const override;  // return -1 if disabled
    bool healthy() const override;
    bool initialised() const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;
    void write_bytes(const char *bytes, uint8_t len) override;
    void handle_command(ExternalAHRS_command command, const ExternalAHRS_command_data &data) override;
    bool get_wind_estimation(Vector3f &wind) override;
    void send_eahrs_status_flag(class GCS_MAVLINK &link) const override;
    void format_status(class ExpandingString &str) override;
    void update() override;
    const char* get_name() const override { return "ILabs"; }
    uint8_t num_gps_sensors() const override { return 1; }

private:
    InertialLabs::DataReadStatus handle_full_circle();
    void update_thread();

    void handle_sensor_data();
    void send_data_to_sensor();

private:
    InertialLabs::Sensor sensor;
    InertialLabs::Sender sender;

    AP_ExternalAHRS::gps_data_message_t gps_data{};
    AP_ExternalAHRS::mag_data_message_t mag_data{};
    AP_ExternalAHRS::baro_data_message_t baro_data{};
    AP_ExternalAHRS::ins_data_message_t ins_data{};
    AP_ExternalAHRS::airspeed_data_message_t airspeed_data{};

    InertialLabs::HandledSensorsData handled_sensor_data{};

    InertialLabs::DriverDiagnosticData driver_diagnostic_data{};
};

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
