#include "ClockManager.hpp"

ClockManager::ClockManager()
{
}

void ClockManager::execute()
{
    sfr::mission::cycle_no++;
    sfr::mission::boot_time_mins = sfr::mission::cycle_no % (constants::time::one_minute / constants::time::control_cycle_time_ms);

    cycle_time = millis() - sfr::mission::cycle_start;

    if (cycle_time < constants::time::control_cycle_time_ms) {
        delay(constants::time::control_cycle_time_ms - cycle_time);
    }
}
