#include "ACSControlTask.hpp"

ACSControlTask::ACSControlTask()
{
}

void ACSControlTask::execute()
{

#ifdef ACS_SIM
    if (first) {
        plantObj.initialize(0.1, altitude_input, I_input, inclination_input, m_input, q0_input, wx_input, wy_input, wz_input);
    }
#endif

    if ((old_Id != constants::acs::Id_values[sfr::acs::Id_index] || old_Kd != constants::acs::Kd_values[sfr::acs::Kd_index] || old_Kp != constants::acs::Kp_values[sfr::acs::Kp_index] || old_c != constants::acs::c_values[sfr::acs::c_index]) || first) {
#ifdef VERBOSE
        Serial.println("Initialize starshot library");
        starshotObj.initialize(constants::acs::step_size_input, constants::acs::A_input, constants::acs::Id_values[sfr::acs::Id_index], constants::acs::Kd_values[sfr::acs::Kd_index], constants::acs::Kp_values[sfr::acs::Kp_index], constants::acs::c_values[sfr::acs::c_index], constants::acs::i_max_input, constants::acs::k_input, constants::acs::n_input);
        /// EKF Initalizing PARAMETERS///
        Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Zero(6, 6);
        // Q (process noise covariance) Matrix
        Eigen::MatrixXd Q = 0.02 * Eigen::MatrixXd::Identity(6, 6);
        Q.diagonal() << 0.008, 0.07, 0.005, 0.1, 0.1, 0.1;
        // Rd (measurement noise variance) Matrices
        Eigen::MatrixXd Rd(6, 6);
        Rd << 2.02559220e-01, 5.17515015e-03, -3.16669361e-02, -1.76503506e-04, -3.74891174e-05, -7.75657503e-05,
            5.17515015e-03, 1.55389381e-01, 1.07780468e-02, -2.90511952e-05, -8.02931174e-06, -1.26277622e-05,
            -3.16669361e-02, 1.07780468e-02, 3.93162684e-01, 9.29630074e-05, 1.22496815e-05, 5.67092127e-05,
            -1.76503506e-04, -2.90511952e-05, 9.29630074e-05, 1.80161545e-05, -2.27002599e-09, -6.07376965e-07,
            -3.74891174e-05, -8.02931174e-06, 1.22496815e-05, -2.27002599e-09, 6.70144060e-06, 2.97298687e-08,
            -7.75657503e-05, -1.26277622e-05, 5.67092127e-05, -6.07376965e-07, 2.97298687e-08, 8.52192033e-06;
        // Hd
        Eigen::MatrixXd Hd = Eigen::MatrixXd::Identity(6, 6);
        Serial.println("Initialize EKF library");
        ekfObj.initialize(constants::acs::step_size_input, initial_state, initial_cov, Q, Rd, Hd);
#endif
        first = false;
    }

    old_Id = constants::acs::Id_values[sfr::acs::Id_index];
    old_Kd = constants::acs::Kd_values[sfr::acs::Kd_index];
    old_Kp = constants::acs::Kp_values[sfr::acs::Kp_index];
    old_c = constants::acs::c_values[sfr::acs::c_index];

    imu_valid = sfr::imu::gyro_x_value->get_value(&gyro_x) && sfr::imu::gyro_y_value->get_value(&gyro_y) && sfr::imu::gyro_z_value->get_value(&gyro_z) && sfr::imu::mag_x_value->get_value(&mag_x) && sfr::imu::mag_y_value->get_value(&mag_y) && sfr::imu::mag_z_value->get_value(&mag_z);

#ifdef ACS_SIM
    // maybe want to add ekf here?
    gyro_x = plantObj.rtY.angularvelocity[0];
    gyro_y = plantObj.rtY.angularvelocity[1];
    gyro_z = plantObj.rtY.angularvelocity[2];
    mag_x = plantObj.rtY.magneticfield[0];
    mag_y = plantObj.rtY.magneticfield[1];
    mag_z = plantObj.rtY.magneticfield[2];

#ifdef VERBOSE
    Serial.print("Simulated MAG_X: ");
    Serial.println(mag_x);
    Serial.print("Simulated MAG_Y: ");
    Serial.println(mag_y);
    Serial.print("Simulated MAG_Z: ");
    Serial.println(mag_z);
    Serial.print("Simulated GYRO_X: ");
    Serial.println(gyro_x);
    Serial.print("Simulated GYRO_Y: ");
    Serial.println(gyro_y);
    Serial.print("Simulated GYRO_Z: ");
    Serial.println(gyro_z);
#endif

    plantObj.step();
#endif

    if (!sfr::acs::off) {
        if ((!imu_valid && (sfr::imu::mode == (uint16_t)sensor_mode_type::abnormal_init)) || (!imu_valid && (sfr::imu::mode == (uint16_t)sensor_mode_type::normal))) {
            sfr::acs::mode = (uint8_t)acs_mode_type::simple;
            //  consider IMU valid since it is not needed for simple mode
            imu_valid = true;
        }

        if (imu_valid) {
            if (!sfr::temperature::temp_c_value->get_value(&temp_c)) {
                temp_c = 0;
            }

            if (!sfr::battery::voltage_value->get_value(&voltage)) {
                voltage = 0;
            }

            IMUOffset(&mag_x, &mag_y, &mag_z, temp_c, voltage, pwm_x, pwm_y, pwm_z);

            // Convert from uT to T
            mag_x = mag_x / 1000000.0;
            mag_y = mag_y / 1000000.0;
            mag_z = mag_z / 1000000.0;

            // load sensor reading to EKF
            ekfObj.Z(0) = mag_x;
            ekfObj.Z(1) = mag_y;
            ekfObj.Z(2) = mag_z;
            ekfObj.Z(3) = gyro_x;
            ekfObj.Z(4) = gyro_y;
            ekfObj.Z(5) = gyro_z;

            ekfObj.step();

            // load filtered imu data from EKF to the controller
            starshotObj.rtU.Bfield_body[0] = ekfObj.state(0);
            starshotObj.rtU.Bfield_body[1] = ekfObj.state(1);
            starshotObj.rtU.Bfield_body[2] = ekfObj.state(2);
            starshotObj.rtU.w[0] = ekfObj.state(3);
            starshotObj.rtU.w[1] = ekfObj.state(4);
            starshotObj.rtU.w[2] = ekfObj.state(5);

            starshotObj.step();

            if (sfr::acs::mode == (uint8_t)acs_mode_type::detumble) {
                current_x = starshotObj.rtY.detumble[0];
                current_y = starshotObj.rtY.detumble[1];
                current_z = starshotObj.rtY.detumble[2];
            } else if (sfr::acs::mode == (uint8_t)acs_mode_type::point) {
                current_x = starshotObj.rtY.point[0];
                current_y = starshotObj.rtY.point[1];
                current_z = starshotObj.rtY.point[2];
            } else if (sfr::acs::mode == (uint8_t)acs_mode_type::simple) {
                current_x = 0;
                current_y = 0;
                current_z = 0;
                if (sfr::acs::simple_mag == (uint8_t)mag_type::x) {
                    current_x = sfr::acs::simple_current;
                } else if (sfr::acs::simple_mag == (uint8_t)mag_type::y) {
                    current_y = sfr::acs::simple_current;
                } else if (sfr::acs::simple_mag == (uint8_t)mag_type::z) {
                    current_z = sfr::acs::simple_current;
                }
            }
        }
    }

    if (sfr::acs::off || !imu_valid) {
        current_x = 0;
        current_y = 0;
        current_z = 0;
    }

    ACSWrite(constants::acs::xtorqorder, current_x, constants::acs::xout1, constants::acs::xout2, constants::acs::xPWMpin);
    ACSWrite(constants::acs::ytorqorder, current_y, constants::acs::yout1, constants::acs::yout2, constants::acs::yPWMpin);
    ACSWrite(constants::acs::ztorqorder, current_z, constants::acs::zout1, constants::acs::zout2, constants::acs::zPWMpin);
}

int ACSControlTask::current2PWM(float current)
{
    if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) < 8.062)
        return 0;
    else if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) > 255)
        return 255;
    else
        return int(633.5 * pow(fabs(current), 0.6043) + 8.062);
}

void ACSControlTask::ACSWrite(int torqorder, float current, int out1, int out2, int PWMpin)
{
    int PWM = current2PWM(current);
#ifdef VERBOSE
    Serial.print("PWM: ");
    Serial.println(PWM);
#endif
    if (PWMpin == constants::acs::xPWMpin) {
        pwm_x = PWM;
    } else if (PWMpin == constants::acs::yPWMpin) {
        pwm_y = PWM;
    } else if (PWMpin == constants::acs::zPWMpin) {
        pwm_z = PWM;
    }

    if (PWM == 0) {
        digitalWrite(out1, LOW);
        digitalWrite(out2, LOW);
    } else if (torqorder == 0) {
        if (current > 0) {
            digitalWrite(out1, HIGH);
            digitalWrite(out2, LOW);
            analogWrite(PWMpin, PWM);
        }
        if (current < 0) {
            digitalWrite(out1, LOW);
            digitalWrite(out2, HIGH);
            analogWrite(PWMpin, PWM);
        }
    } else if (torqorder == 1) {
        if (current > 0) {
            digitalWrite(out1, LOW);
            digitalWrite(out2, HIGH);
            analogWrite(PWMpin, PWM);
        }
        if (current < 0) {
            digitalWrite(out1, HIGH);
            digitalWrite(out2, LOW);
            analogWrite(PWMpin, PWM);
        }
    }
}

void ACSControlTask::IMUOffset(float *mag_x, float *mag_y, float *mag_z, float temp, float voltage, float pwmX, float pwmY, float pwmZ)
{
    using namespace constants::acs;

    /*Offset Contributions from PWM (ex: pwmX_oX is contribution of X mag to offset x)*/
    float pwmX_ox = pwmX_ox_1 * pwmX + pwmX_ox_2 * pow(pwmX, 2) + pwmX_ox_3 * pow(pwmX, 3);
    float pwmX_oy = pwmX_oy_1 * pwmX + pwmX_oy_2 * pow(pwmX, 2) + pwmX_oy_3 * pow(pwmX, 3);
    float pwmX_oz = pwmX_oz_1 * pwmX + pwmX_oz_2 * pow(pwmX, 2) + pwmX_oz_3 * pow(pwmX, 3);

    float pwmY_ox = pwmY_ox_1 * pwmY + pwmY_ox_2 * pow(pwmY, 2) + pwmY_ox_3 * pow(pwmY, 3);
    float pwmY_oy = pwmY_oy_1 * pwmY + pwmY_oy_2 * pow(pwmY, 2) + pwmY_oy_3 * pow(pwmY, 3);
    float pwmY_oz = pwmY_oz_1 * pwmY + pwmY_oz_2 * pow(pwmY, 2) + pwmY_oz_3 * pow(pwmY, 3);

    float pwmZ_ox = pwmZ_ox_1 * pwmZ + pwmZ_ox_2 * pow(pwmZ, 2) + pwmZ_ox_3 * pow(pwmZ, 3);
    float pwmZ_oy = pwmZ_oy_1 * pwmZ + pwmZ_oy_2 * pow(pwmZ, 2) + pwmZ_oy_3 * pow(pwmZ, 3);
    float pwmZ_oz = pwmZ_oz_1 * pwmZ + pwmZ_oz_2 * pow(pwmZ, 2) + pwmZ_oz_3 * pow(pwmZ, 3);
    /*******************************************/
    /*Voltage Adjustment Coefficients (ex: volX_ox = coef for pwmX_oX)*/
    float volX_ox = volX_ox_1 * voltage + volX_ox_c;
    float volX_oy = volX_oy_1 * voltage + volX_oy_c;
    float volX_oz = volX_oz_1 * voltage + volX_oz_c;

    float volY_ox = volY_ox_1 * voltage + volY_ox_c;
    float volY_oy = volY_oy_1 * voltage + volY_oy_c;
    float volY_oz = volY_oz_1 * voltage + volY_oz_c;

    float volZ_ox = volZ_ox_1 * voltage + volZ_ox_c;
    float volZ_oy = volZ_oy_1 * voltage + volZ_oy_c;
    float volZ_oz = volZ_oz_1 * voltage + volZ_oz_c;
    /*******************************************/
    /*Temperature Offset Terms*/
    float temp_x = temp_x_1 * temp + temp_x_2 * pow(temp, 2) + temp_x_3 * pow(temp, 3) + temp_x_c;
    float temp_y = temp_y_1 * temp + temp_y_2 * pow(temp, 2) + temp_y_3 * pow(temp, 3) + temp_y_c;
    float temp_z = temp_z_1 * temp + temp_z_2 * pow(temp, 2) + temp_z_3 * pow(temp, 3) + temp_z_c;
    /*******************************************/
    /*Total Offsets*/
    float xoffset = volX_ox * pwmX_ox + volY_ox * pwmY_ox + volZ_ox * pwmZ_ox + temp_x + hardiron_x;
    float yoffset = volX_oy * pwmX_oy + volY_oy * pwmY_oy + volZ_oy * pwmZ_oy + temp_y + hardiron_y;
    float zoffset = volX_oz * pwmX_oz + volY_oz * pwmY_oz + volZ_oz * pwmZ_oz + temp_z + hardiron_z;
    /*******************************************/
    /* Finally, adjust magnetometer readings*/

    *mag_x = *mag_x - xoffset;
    *mag_y = *mag_y - yoffset;
    *mag_z = *mag_z - zoffset;
}
