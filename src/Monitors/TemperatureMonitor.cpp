#include "TemperatureMonitor.hpp"

TemperatureMonitor::TemperatureMonitor()
{
    pinMode(constants::temperature::pin, INPUT);
}

void TemperatureMonitor::execute()
{
    if (!initialized) {
        sfr::temperature::temp_c_average->set_valid();
        sfr::temperature::temp_c_value->set_valid();
        initialized = true;
    }

    float val = (((analogRead(constants::temperature::pin) * 3.3) / 1023) - .5) * 100;
    sfr::temperature::temp_c_average->set_value(val);
    sfr::temperature::temp_c_value->set_value(val);

    if (sfr::temperature::temp_c_average->get_value(&val)) {
        sfr::temperature::in_sun = val >= constants::temperature::in_sun_val;
    } else {
        sfr::temperature::in_sun = false;
    }
}