#ifndef IMU_MONITOR_HPP_
#define IMU_MONITOR_HPP_

#include "ISensorMonitor.hpp"
#include "sfr.hpp"

class IMUMonitor : public TimedControlTask<void>, public ISensorMonitor{
    public:
        IMUMonitor(unsigned int offset);
        void execute();
        Adafruit_LSM9DS1 imu;

        void transition_to_normal();
        void transition_to_abnormal_init();
        void transition_to_abnormal_readings();
        void transition_to_retry();
        void transition_to_abandon();
    private:
        void capture_imu_values();
};

#endif