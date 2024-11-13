#include "AP_Airspeed_External.h"

#if AP_AIRSPEED_EXTERNAL_ENABLED

#define TIMEOUT_MS 2000

AP_Airspeed_External::AP_Airspeed_External(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
    set_bus_id(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,0,_instance,0));
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_External::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);
    if (press_count == 0) {
        return false;
    }
    pressure = sum_pressure/press_count;
    press_count = 0;
    sum_pressure = 0;
    return true;
}

// get last temperature
bool AP_Airspeed_External::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);
    if (temperature_count == 0) {
        return false;
    }
    temperature = sum_temperature/temperature_count;
    temperature_count = 0;
    sum_temperature = 0;
    return true;
}

bool AP_Airspeed_External::get_airspeed(float &airspeed)
{
    WITH_SEMAPHORE(sem);
    if (airspeed_count == 0) {
        return false;
    }

    uint32_t now = AP_HAL::millis();

    airspeed = sum_airspeed/airspeed_count;
    airspeed_count = 0;
    sum_airspeed = 0;

    return (now - last_update_ms) < TIMEOUT_MS;
}

bool AP_Airspeed_External::has_airspeed()
{
    return ext_airspeed_enabled;
}

void AP_Airspeed_External::set_external_airspeed_enabled(bool &airspeed_enabled)
{
    ext_airspeed_enabled = airspeed_enabled;
}

void AP_Airspeed_External::handle_external(const AP_ExternalAHRS::airspeed_data_message_t &pkt)
{
    WITH_SEMAPHORE(sem);

    sum_pressure += pkt.differential_pressure;
    press_count++;
    if (press_count > 100) {
        // prevent overflow
        sum_pressure /= 2;
        press_count /= 2;
    }

    sum_temperature += pkt.temperature;
    temperature_count++;
    if (temperature_count > 100) {
        // prevent overflow
        sum_temperature /= 2;
        temperature_count /= 2;
    }

    sum_airspeed += pkt.airspeed;
    airspeed_count++;
    if (airspeed_count > 100) {
        // prevent overflow
        sum_airspeed /= 2;
        airspeed_count /= 2;
    }

    last_update_ms = AP_HAL::millis();
}

#endif // AP_AIRSPEED_EXTERNAL_ENABLED
