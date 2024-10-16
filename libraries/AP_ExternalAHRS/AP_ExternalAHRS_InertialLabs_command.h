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

#if AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED

#include "AP_ExternalAHRS_command_context.h"

#include <stdint.h>

namespace InertialLabs {

constexpr size_t BUFFER_SIZE = 100;

/// messageType(1 byte) + dataIdentifier(1 byte) + messageLength(2 byte) + checksum(2 byte) = 6 byte
const uint16_t PACKAGE_ADDITIONAL_PARAMS_SIZE = 6;

struct Data_context {
    uint16_t length{0};
    uint8_t data[BUFFER_SIZE];
};

uint16_t calculate_checksum(const uint8_t * buf, uint16_t size);

bool fill_command_pyload(Data_context & context,
                         ExternalAHRS_command command,
                         const ExternalAHRS_command_data &data);

bool fill_transport_protocol_data(Data_context & context);

} // namespace InertialLabs

namespace InertialLabs::Command {

const char ENABLE_GNSS[] = "\xAA\x55\x00\x00\x07\x00\x71\x78\x00";
const char DISABLE_GNSS[] = "\xAA\x55\x00\x00\x07\x00\x72\x79\x00";
const char START_VG3DCLB_FLIGHT[] = "\xAA\x55\x00\x00\x07\x00\x26\x2D\x00";
const char STOP_VG3DCLB_FLIGHT[] = "\xAA\x55\x00\x00\x07\x00\x27\x2E\x00";
const char START_UDD[] = "\xAA\x55\x00\x00\x07\x00\x95\x9C\x00";
const char STOP[] = "\xAA\x55\x00\x00\x07\x00\xFE\x05\x01";

} // namespace Inertiallabs::Command

#endif  // AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED
