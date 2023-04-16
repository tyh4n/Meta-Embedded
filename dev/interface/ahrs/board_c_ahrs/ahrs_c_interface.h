//
// Created by Tianyi Han on 3/14/2023.
//

#ifndef META_EMBEDDED_AHRS_C_INTERFACE_H
#define META_EMBEDDED_AHRS_C_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "shell.h"

#include "ahrs_abstract.h"
#include "ahrs_math.hpp"
#include "ahrs_lib.h"
#include "bmi088_interface.h"
#include "ist8310_interface.h"

/**
 * @name AHRSOnBoard_C
 * @brief Attitude And Heading Reference System using on-board IMU and IST
 * @note This module make use of AHRS component from DJI standard program
 * @usage 1. Call start() to enable MPUOnBoard, ISTOnBoard and AHRS updating thread
 *        2. Make use of data, angle, etc.
 */
class AHRSOnBoard_C : public AbstractAHRS, protected chibios_rt::BaseStaticThread<512> {
public:
    /**
     * Start MPUOnBoard, ISTOnBoard and AHRS update thread
     * @note Should be called from NORMAL state (not in locks)
     * @param mpu_rotation_matrix  3x3 Matrix maps gyro and accel to desired coordinate system
     * @param update_thread_prio   priority of updating thread
     */
    void start(const Matrix33 mpu_rotation_matrix, tprio_t update_thread_prio);

    /**
     * Get data from gyroscope (rotated with mpu_rotation_matrix)
     * @return Rotated gyro data from gyroscope [deg/s]
     */
    Vector3D get_gyro() override { return gyro_deg;  /* rotated */ }

    /**
     * Get data from accelerometer (rotated with mpu_rotation_matrix)
     * @return Rotated acceleration data from accelerometer [m/s^2]
     */
    Vector3D get_accel() override { return accel;  /* rotated */ }

    /**
     * Get magnet data
     * @return Magnet data [uT]
     */
    Vector3D get_magnet() override { return magnet; }

    /**
     * Get board angle
     * @return Board angle [degree]
     * @note x - yaw, y - pitch, z - roll
     */
    Vector3D get_angle() override { return angle; }

    time_msecs_t get_ahrs_update_time() const override { return ahrs_update_time; }

    time_msecs_t get_ist_update_time() const override { return imu.imu_update_time; }

    time_msecs_t get_mpu_update_time() const override { return imu.imu_update_time; }

    static void cmdFeedback(void *);

    const Shell::Command shellCommands[3] = {
            {"_a",           nullptr,                                  cmdInfo,           this},
            {"_a_enable_fb", "Channel/All Feedback{Disabled,Enabled}", cmdEnableFeedback, this},
            {nullptr,        nullptr,                                  nullptr,           nullptr}
    };

private:
    BMI088Interface imu;
    IST8310Interface compass;

    float q[4];
    Vector3D angle;  // board angle
    time_msecs_t ahrs_update_time = 0;  // last update time from system start [ms]

    Matrix33 mpu_rotation_matrix;

    // Local storage (MPU rotated)
    Vector3D gyro_deg;  // angular speed [deg/s]
    Vector3D gyro_rad;  // angular speed [rad/s]
    Vector3D accel;     // acceleration [m/s^2]
    Vector3D magnet;    // magnetic field [uT]

    void fetch_data();  // fetch data from IMU, should be called from NORMAL state (not in locks)
    void update();      // fetch data and update angles, should be called from NORMAL state (not in locks)

    void main() override;

    static constexpr int UPDATE_THREAD_INTERVAL = 1;  // [ms]

    static DECL_SHELL_CMD(cmdInfo);

    static DECL_SHELL_CMD(cmdEnableFeedback);

    bool feedbackEnabled[4] = {false, false, false, false};

    friend class AHRSCalibrationThread;
};

#endif //META_EMBEDDED_AHRS_C_INTERFACE_H
