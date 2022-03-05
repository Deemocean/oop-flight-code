#include "MissionMode.hpp"
#include "sfr.hpp"

void Boot::dispatch()
{
    if (millis() - sfr::mission::boot_start >= sfr::mission::max_boot_time) {
        sfr::mission::current_mode = sfr::mission::aliveSignal;
    }
}

void Boot::transition_to() {}

void AliveSignal::dispatch()
{
    // check_entrance_lp(sfr::mission::lowPowerAliveSignal);
}

void AliveSignal::transition_to() {}

void check_entrance_lp(MissionMode *lp_mode)
{
    // TODO and or check for voltage
    if (sfr::battery::voltage_average <= sfr::battery::min_battery) {
        sfr::mission::current_mode = lp_mode;
    }
}

void check_exit_lp(MissionMode *reg_mode)
{
    if (sfr::battery::voltage_average >= sfr::battery::acceptable_battery && sfr::fault::check_voltage) {
        sfr::mission::current_mode = reg_mode;
    }
}