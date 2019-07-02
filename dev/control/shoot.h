//
// Created by liuzikai on 2019-05-01.
//

#ifndef META_INFANTRY_SHOOT_H
#define META_INFANTRY_SHOOT_H

#include "gimbal_interface.h"
#include "pid_controller.hpp"

/**
 * @name Shoot
 * @brief Shooter controller from high level to low level (by inheritance)
 * @note Share low-level GimbalInterface with Gimbal. Low-level initialization is done by Gimbal.
 */
class Shoot : public GimbalInterface, public PIDControllerBase {

public:

    /**
     * Initialize the shooter controller
     * @param degree_per_bullet
     * @param bullet_loader_v2i_params
    */

    static void init(float degree_per_bullet);

    /**
     * Change PID parameters of bullet loader
     * @param bullet_loader_v2i_params
     */

    static void change_pid_params(PIDControllerBase::pid_params_t bullet_loader_v2i_params);

    /**
     * Perform calculation from velocity to current and put result into target_current[]
     * @param bullet_per_second   shoot speed
     */
    static void calc(float bullet_per_second);

    /**
     * Set friction wheel duty cycle
     * @param duty_cycle from 0 to 1
     */
    static void set_friction_wheels(float duty_cycle);

    static PIDController v2i_pid[2];

private:
    static float degree_per_bullet_;

    static void calc_motor_(motor_id_t motor, float actual_velocity, float target_velocity);


    friend class ShootDebugThread;
};


#endif //META_INFANTRY_SHOOT_H