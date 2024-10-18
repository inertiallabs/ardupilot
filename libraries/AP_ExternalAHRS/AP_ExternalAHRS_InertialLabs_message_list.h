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

#include <GCS_MAVLink/GCS.h>

// Unit Status Word bits
enum IL_USW {
	INITIAL_ALIGNMENT_FAIL	= (1U << 0),
	OPERATION_FAIL			= (1U << 1),
	GYRO_FAIL				= (1U << 2),
	ACCEL_FAIL				= (1U << 3),
	MAG_FAIL				= (1U << 4),
	ELECTRONICS_FAIL		= (1U << 5),
	GNSS_FAIL				= (1U << 6),
	MAG_VG3D_CLB_RUNTIME	= (1U << 7),
	VOLTAGE_LOW				= (1U << 8),
	VOLTAGE_HIGH			= (1U << 9),
	GYRO_X_RATE_HIGH		= (1U << 10),
	GYRO_Y_RATE_HIGH	    = (1U << 11),
	GYRO_Z_RATE_HIGH		= (1U << 12),
	MAG_FIELD_HIGH			= (1U << 13),
	TEMP_RANGE_ERR			= (1U << 14),
	MAG_VG3D_CLB_SUCCESS	= (1U << 15)
};

// Unit Status Word2 bits
enum IL_USW2 {
	ACCEL_X_HIGH           	 = (1U << 0),
	ACCEL_Y_HIGH           	 = (1U << 1),
	ACCEL_Z_HIGH           	 = (1U << 2),
	ADU_BARO_FAIL            = (1U << 3),
	ADU_DIFF_PRESS_FAIL   	 = (1U << 4),
	MAG_AUTO_CAL_2D_RUNTIME  = (1U << 5),
	MAG_AUTO_CAL_3D_RUNTIME  = (1U << 6),
	GNSS_FUSION_OFF        	 = (1U << 7),
	DIFF_PRESS_FUSION_OFF  	 = (1U << 8),
	GNSS_POS_VALID           = (1U << 10)
};

// Air Data Status bits
enum IL_ADU {
	BARO_INIT_FAIL            = (1U << 0),
	DIFF_PRESS_INIT_FAIL      = (1U << 1),
	BARO_FAIL       		  = (1U << 2),
	DIFF_PRESS_FAIL           = (1U << 3),
	BARO_RANGE_ERR  		  = (1U << 4),
	DIFF_PRESS_RANGE_ERR      = (1U << 5),
	BARO_ALT_FAIL             = (1U << 8),
	AIRSPEED_FAIL             = (1U << 9),
	AIRSPEED_BELOW_THRESHOLD  = (1U << 10)
};

// New GPS indicator
enum IL_NEWGPS {
	NEW_GNSS_POSITION     = (1U << 0),
	NEW_GNSS_VELOCITY     = (1U << 1),
	NEW_GNSS_HEADING      = (1U << 2),
	NEW_VALID_PPS         = (1U << 3),
	NEW_GNSS_BESTXYZ_LOG  = (1U << 4),
	NEW_GNSS_PSRDOP_LOG   = (1U << 5),
	NEW_PPS               = (1U << 6),
	NEW_RANGE_LOG         = (1U << 7)
};

enum IL_NewAidingData {
	NEW_AIRSPEED    = (1U << 1),
	NEW_WIND        = (1U << 2),
	NEW_EXT_POS     = (1U << 3),
	NEW_HEADING     = (1U << 5),
	NEW_AMBIENT     = (1U << 10),
	NEW_ALTITUDE    = (1U << 11),
	NEW_EXT_HOR_POS = (1U << 13),
	NEW_ADC         = (1U << 15)
};

struct ILStatusMessage {
	unsigned int status;
	MAV_SEVERITY severity;
	const char* msg_true;
	const char* msg_false;
};

const ILStatusMessage IL_usw_msg[] = {
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

const ILStatusMessage IL_usw2_msg[] = {
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

const ILStatusMessage IL_adu_msg[] = {
	{ BARO_INIT_FAIL,           MAV_SEVERITY_WARNING, "Static pressure sensor unsuccessful initialization", "Static pressure sensor initialization successful"},
	{ DIFF_PRESS_INIT_FAIL,     MAV_SEVERITY_WARNING, "Diff. pressure sensor unsuccessful initialization",  "Diff. pressure sensor initialization successful"},
	{ BARO_RANGE_ERR,           MAV_SEVERITY_INFO,    "Static pressure is out of range",                    "Static pressure is in range"      },
	{ DIFF_PRESS_RANGE_ERR,     MAV_SEVERITY_INFO,    "Diff. pressure is out of range",                     "Diff. pressure is in range"       },
	{ BARO_ALT_FAIL,            MAV_SEVERITY_WARNING, "Pressure altitude is incorrect",                     "Pressure altitude is correct"     },
	{ AIRSPEED_FAIL,            MAV_SEVERITY_WARNING, "Air speed is incorrect",                             "Air speed is correct"             },
	{ AIRSPEED_BELOW_THRESHOLD, MAV_SEVERITY_INFO,    "Air speed is below the threshold",                   "Air speed is above the threshold" }
};

const size_t IL_usw_msg_size = sizeof(IL_usw_msg) / sizeof(IL_usw_msg[0]);
const size_t IL_usw2_msg_size = sizeof(IL_usw2_msg) / sizeof(IL_usw2_msg[0]);
const size_t IL_adu_msg_size = sizeof(IL_adu_msg) / sizeof(IL_adu_msg[0]);

extern uint64_t IL_usw_last_msg_ms[];
extern uint64_t IL_usw2_last_msg_ms[];
extern uint64_t IL_adu_last_msg_ms[];

#endif  // AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED