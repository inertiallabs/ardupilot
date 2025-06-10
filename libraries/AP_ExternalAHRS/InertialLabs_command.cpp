#include "InertialLabs_command.h"

#if AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
#include <limits>
#include <string.h>

#include <AP_Math/AP_Math.h>

#include "AP_ExternalAHRS_command_context.h"
#include "InertialLabs_data.h"

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

bool fill_command_payload(Data_context & context,
                          ExternalAHRS_command command,
                          const ExternalAHRS_command_data &data) {
    // transport + checksum + aidingData count + aidingData type
    context.length = 6 + 2 + 1 + 1;

    // The first 6 bites (0-5) and last 2 bytes (payload+2) reserved for the transport protocol info
    // Start payload from 7th byte
    context.data[6] = 0x01;

    switch (command) {
        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_POSITION:
            _fill_aiding_data_external_position_payload(context, data);
            return true;

        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_HORIZONTAL_POSITION:
            _fill_aiding_data_external_horizontal_position_payload(context, data);
            return true;

        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_ALTITUDE:
            _fill_aiding_data_external_altitude_payload(context, data);
            return true;

        case ExternalAHRS_command::AIDING_DATA_WIND:
            _fill_aiding_data_wind_payload(context, data);
            return true;

        case ExternalAHRS_command::AIDING_DATA_AMBIENT_AIR:
            _fill_aiding_data_ambient_air_payload(context, data);
            return true;

        case ExternalAHRS_command::AIDING_DATA_EXTERNAL_HEADING:
            _fill_aiding_data_external_heading_payload(context, data);
            return true;

        case ExternalAHRS_command::AIDING_DATA_AIR_SPEED:
            _fill_aiding_data_air_speed_payload(context, data);
            return true;

        case ExternalAHRS_command::AIDING_DATA_DVL:
            _fill_aiding_data_doppler_velocity_log_payload(context, data);
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
    context.data[3] = 0x62; // message identifier of Aiding data packages

    const uint16_t messageLength = context.length - 2; // all package length without the header (first 2 bites)
    memcpy(&context.data[4], &messageLength, sizeof(uint16_t));

    const uint16_t calculatedChecksum = calculate_checksum(&context.data[2], messageLength - 2); // all package length without the header (first 2 bites) and checksum (last 2 bites)
    memcpy(&context.data[messageLength], &calculatedChecksum, sizeof(uint16_t));

    return true;
}

void _fill_aiding_data_external_position_payload(Data_context & context, const ExternalAHRS_command_data &data)
{
    context.data[7] = 0x04;
    AidingData::ExternalPosition *d = (AidingData::ExternalPosition *) &context.data[8];
    d->latitude = data.x;
    d->longitude = data.y;
    d->altitude = static_cast<int32_t>(data.z * 1.0e3f);
    if ((fabsf(data.param2) < std::numeric_limits<float>::epsilon()) && (fabsf(data.param3) < std::numeric_limits<float>::epsilon())) {
        d->latitudeStd = std::numeric_limits<uint16_t>::max();
        d->longitudeStd = std::numeric_limits<uint16_t>::max();
    } else {
        d->latitudeStd = static_cast<uint16_t>(data.param2 * 100.0f);
        d->longitudeStd = static_cast<uint16_t>(data.param3 * 100.0f);
    }
    if (fabsf(data.param4) < std::numeric_limits<float>::epsilon()) {
        d->altitudeStd = std::numeric_limits<uint16_t>::max();
    } else {
        d->altitudeStd = static_cast<uint16_t>(data.param4 * 100.0f);
    }
    d->latency = static_cast<uint16_t>(data.param1 * 1.0e3f);

    context.length += sizeof(AidingData::ExternalPosition);
}

void _fill_aiding_data_external_horizontal_position_payload(Data_context & context, const ExternalAHRS_command_data &data)
{
    context.data[7] = 0x0E;

    AidingData::ExternalHorizontalPosition *d = (AidingData::ExternalHorizontalPosition *) &context.data[8];
    d->latitude = data.x;
    d->longitude = data.y;
    if ((fabsf(data.param1) < std::numeric_limits<float>::epsilon()) && (fabsf(data.param2) < std::numeric_limits<float>::epsilon())) {
        d->latitudeStd = std::numeric_limits<uint16_t>::max();
        d->longitudeStd = std::numeric_limits<uint16_t>::max();
    } else {
        d->latitudeStd = static_cast<uint16_t>(data.param1 * 100.0f);
        d->longitudeStd = static_cast<uint16_t>(data.param2 * 100.0f);
    }
    d->latency = static_cast<uint16_t>(data.param3 * 1.0e3f);

    context.length += sizeof(AidingData::ExternalHorizontalPosition);
}

void _fill_aiding_data_external_altitude_payload(Data_context & context, const ExternalAHRS_command_data &data)
{
    context.data[7] = 0x0C;

    AidingData::ExternalAltitude *d = (AidingData::ExternalAltitude *) &context.data[8];
    d->altitude = static_cast<int32_t>(data.z * 1.0e3f);
    if (fabsf(data.param1) < std::numeric_limits<float>::epsilon()) {
        d->altitudeStd = std::numeric_limits<uint16_t>::max();
    } else {
        d->altitudeStd = static_cast<uint16_t>(data.param1 * 100.0f);
    }

    context.length += sizeof(AidingData::ExternalAltitude);
}

void _fill_aiding_data_wind_payload(Data_context & context, const ExternalAHRS_command_data &data)
{
    context.data[7] = 0x03;

    const float direction = data.param1;
    const float speed = data.param2;
    const float speedStd = data.param3;

    // Speed in m/s
    const float NWind = speed * cosf(direction * M_PI / 180.0f);
    const float EWind = speed * sinf(direction * M_PI / 180.0f);
    const float NWindStd = speedStd; //< as designed
    const float EWindStd = speedStd; //< as designed

    // Speed from m/s to kt
    const float m_per_s_to_kt = 1.94384449f;
    AidingData::Wind *d = (AidingData::Wind *) &context.data[8];
    d->north = static_cast<int16_t>(NWind * m_per_s_to_kt * 100.0f);
    d->east = static_cast<int16_t>(EWind * m_per_s_to_kt * 100.0f);
    d->northStd = static_cast<uint16_t>(NWindStd * m_per_s_to_kt * 100.0f);
    d->eastStd = static_cast<uint16_t>(EWindStd * m_per_s_to_kt * 100.0f);

    context.length += sizeof(AidingData::Wind);
}

void _fill_aiding_data_ambient_air_payload(Data_context & context, const ExternalAHRS_command_data &data)
{
    context.data[7] = 0x0B;

    AidingData::AmbientAir *d = (AidingData::AmbientAir *) &context.data[8];
    d->temperature = static_cast<int16_t>(data.param1 * 10.0f);
    d->altitude = static_cast<int32_t>(data.z * 100.0f);
    d->pressure = static_cast<uint16_t>(data.param2 * 0.5f);

    context.length += sizeof(AidingData::AmbientAir);
}

void _fill_aiding_data_external_heading_payload(Data_context & context, const ExternalAHRS_command_data &data)
{
    context.data[7] = 0x06;

    AidingData::ExternalHeading *d = (AidingData::ExternalHeading *) &context.data[8];
    d->heading = static_cast<uint16_t>(data.param1 * 100.0f);
    if (fabsf(data.param2) < std::numeric_limits<float>::epsilon()) {
        d->headingStd = std::numeric_limits<uint16_t>::max();
    } else {
        d->headingStd = static_cast<uint16_t>(data.param2 * 100.0f);
    }
    d->latency = static_cast<uint16_t>(data.param3 * 1.0e3f);

    context.length += sizeof(AidingData::ExternalHeading);
}

void _fill_aiding_data_air_speed_payload(Data_context & context, const ExternalAHRS_command_data &data)
{
    context.data[7] = 0x02;

    AidingData::AirSpeed *d = (AidingData::AirSpeed *) &context.data[8];
    d->airSpeed = static_cast<int16_t>(data.param1);

    context.length += sizeof(AidingData::AirSpeed);
}

void _fill_aiding_data_doppler_velocity_log_payload(Data_context & context, const ExternalAHRS_command_data &data)
{
    context.data[7] = 0x07;
    {
        AidingData::DopplerVelocityLog *d = (AidingData::DopplerVelocityLog *) &context.data[8];
        d->lateralVelocity = static_cast<int32_t>(data.param1);
        d->forwardVelocity = static_cast<int32_t>(data.param2);
        d->verticalVelocity = static_cast<int32_t>(data.param3);
        d->lateralVelocityStd = static_cast<uint16_t>(data.param4);
        d->forwardVelocityStd = static_cast<uint16_t>(data.x);
        d->verticalVelocityStd = static_cast<uint16_t>(data.y);
        d->latency = static_cast<uint16_t>(data.z);
        d->reserved = 0;
    }
    context.length += sizeof(AidingData::DopplerVelocityLog);
}

} // namespace InertialLabs

#endif  // AP_EXTERNAL_AHRS_INERTIALLABS_ENABLED
