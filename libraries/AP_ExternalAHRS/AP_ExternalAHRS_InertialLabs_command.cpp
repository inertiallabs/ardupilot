#include "AP_ExternalAHRS_InertialLabs_command.h"

#if AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED

#include "AP_ExternalAHRS_command_context.h"
#include "AP_ExternalAHRS_InertialLabs_aiding_data.h"

#include <AP_Math/AP_Math.h>

#include <string.h>

namespace InertialLabs {

uint16_t calculate_checksum(const uint8_t *buf, uint16_t size)
{
    uint16_t checksum{0};

    for (size_t i = 0; i < size; ++i)
    {
        checksum += buf[i];
    }

    return checksum;
}

bool fill_command_pyload(Data_context & context,
                         ExternalAHRS_command command,
                         const ExternalAHRS_command_data &data) {
    // transport + checksum + aidingData count + aidingData type
    context.length = 6 + 2 + 1 + 1;

    // The first 6 bites (0-5) and last 2 bytes (payload+2) reserved for the transport protocol info
    // Start payload from 7th byte
    context.data[6] = 0x01;

    switch (command) {
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_POSITION:
            context.data[7] = 0x04;
            {
                AidingData::External_position *d = (AidingData::External_position *) &context.data[8];
                d->latitude = data.x;
                d->longitude = data.y;
                d->altitude = static_cast<int32_t>(data.z * 1e3);
                d->latitudeStd = static_cast<int16_t>(data.param2 * 1e2);
                d->longitudeStd = static_cast<int16_t>(data.param3 * 1e2);
                d->altitudeStd = static_cast<int16_t>(data.param4 * 1e2);
                d->latency = static_cast<int16_t>(data.param1 * 1e3);
            }
            context.length += sizeof(AidingData::External_position);
            return true;

        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_HORIZONTAL_POSITION:
            context.data[7] = 0x0E;
            {
                AidingData::External_horizontal_position *d = (AidingData::External_horizontal_position *) &context.data[8];
                d->latitude = data.x;
                d->longitude = data.y;
                d->latitudeStd = static_cast<int16_t>(data.param1 * 1e2);
                d->longitudeStd = static_cast<int16_t>(data.param2 * 1e2);
                d->latency = static_cast<int16_t>(data.param3 * 1e3);
            }
            context.length += sizeof(AidingData::External_horizontal_position);
            return true;

        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_ALTITUDE:
            context.data[7] = 0x0C;
            {
                AidingData::External_altitude *d = (AidingData::External_altitude *) &context.data[8];
                d->altitude = static_cast<int32_t>(data.z * 1e3);
                d->altitudeStd = static_cast<int16_t>(data.param1 * 1e2);
            }
            context.length += sizeof(AidingData::External_altitude);
            return true;

        case ExternalAHRS_command::AIDING_DATA_WIND:
            context.data[7] = 0x03;
            {
                const float direction = data.param1;
                const float speed = data.param2;
                const float speedStd = data.param3;

                // Speed in m/s
                const float NWind = speed * cosf(direction * M_PI / 180);
                const float EWind = speed * sinf(direction * M_PI / 180);
                const float NWindStd = speedStd; //< as designed
                const float EWindStd = speedStd; //< as designed

                // Speed from m/s to kt
                const float m_per_s_to_kt = 1.943844;
                AidingData::Wind *d = (AidingData::Wind *) &context.data[8];
                d->north = static_cast<int16_t>(NWind * m_per_s_to_kt * 1e2);
                d->east = static_cast<int16_t>(EWind * m_per_s_to_kt * 1e2);
                d->northStd = static_cast<int16_t>(NWindStd * m_per_s_to_kt * 1e2);
                d->eastStd = static_cast<int16_t>(EWindStd * m_per_s_to_kt * 1e2);
            }
            context.length += sizeof(AidingData::Wind);
            return true;

        case ExternalAHRS_command::AIDING_DATA_AMBIENT_AIR:
            context.data[7] = 0x0B;
            {
                AidingData::Ambient_air *d = (AidingData::Ambient_air *) &context.data[8];
                d->temperature = static_cast<int16_t>(data.param1 * 10);
                d->altitude = static_cast<int32_t>(data.z * 1e2);
                d->pressure = static_cast<int16_t>(data.param2 / 2);
            }
            context.length += sizeof(AidingData::Ambient_air);
            return true;

        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_HEADING:
            context.data[7] = 0x06;
            {
                AidingData::External_heading *d = (AidingData::External_heading *) &context.data[8];
                d->heading = static_cast<int16_t>(data.param1 * 1e2);
                d->headingStd = static_cast<int16_t>(data.param2 * 1e2);
                d->latency = static_cast<int16_t>(data.param3 * 1e3);
            }
            context.length += sizeof(AidingData::External_heading);
            return true;

        default:
            context.length = 0;
            return false;
    }
}

bool fill_transport_protocol_data(Data_context &context) {
    if (!context.length)
    {
        return false;
    }

    context.data[0] = 0xAA; // header 1
    context.data[1] = 0x55; // header 2
    context.data[2] = 0x01; // message type for incoming data
    context.data[3] = 0x62; // message identifer of Aiding data packages

    const uint16_t messageLength = context.length - 2; // all package length without the header (first 2 bites)
    memcpy(&context.data[4], &messageLength, sizeof(uint16_t));

    const uint16_t calculatedChecksum = calculate_checksum(&context.data[2], messageLength - 2); // all package length without the header (first 2 bites) and checksum (last 2 bites)
    memcpy(&context.data[messageLength], &calculatedChecksum, sizeof(uint16_t));

    return true;
}

} // namespace Command

#endif  // AP_EXTERNAL_AHRS_INERTIAL_LABS_ENABLED
