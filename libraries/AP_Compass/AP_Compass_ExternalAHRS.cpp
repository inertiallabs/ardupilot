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

#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Compass_ExternalAHRS.h"
#include <AP_Math/AP_Math.h>

#if AP_COMPASS_EXTERNALAHRS_ENABLED

AP_Compass_ExternalAHRS::AP_Compass_ExternalAHRS(uint8_t port)
{
    auto devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,port,0,0);
    register_compass(devid, instance);

    set_dev_id(instance, devid);
    set_external(instance, true);
    set_EAHRS(instance, true);
    set_rotation(instance, ROTATION_NONE);

    bool disableCal = AP::externalAHRS().check_eahrs_option(AP_ExternalAHRS::OPTIONS::ILAB_DISABLE_CLB);
    if (disableCal) {
        _compass.set_and_save_offsets(instance, Vector3f());
#if AP_COMPASS_DIAGONALS_ENABLED
        _compass.set_and_save_diagonals(instance, Vector3f());
        _compass.set_and_save_offdiagonals(instance, Vector3f());
#endif
        _compass.set_and_save_scale_factor(instance, 1);
    }

    // Workaround for InertialLabs AHRS: Device no need calibration and ready to use as is
    if (AP_ExternalAHRS::get_singleton())
    {
        save_dev_id(instance);
    }
}

void AP_Compass_ExternalAHRS::handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt)
{
    Vector3f field = pkt.field;
    accumulate_sample(field, instance);
}

void AP_Compass_ExternalAHRS::read(void)
{
    drain_accumulated_samples(instance);
}

#endif // AP_COMPASS_EXTERNALAHRS_ENABLED
