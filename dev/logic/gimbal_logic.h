//
// Created by liuzikai on 2019-06-26.
//

/**
 * @file    gimbal_logic.h
 * @brief   By-passing logic-level module to control GimbalLG
 *
 * @addtogroup gimbal
 * @{
 */

#ifndef META_INFANTRY_GIMBAL_LOGIC_H
#define META_INFANTRY_GIMBAL_LOGIC_H


#include "gimbal_scheduler.h"
#include "lidar_interface.h"
#include "vision_scheduler.h"
#include "trajectory_calculator.hpp"

using namespace chibios_rt;

/**
 * @name GimbalLG
 * @note LG stands for "logic"
 * @brief By-passing logic-level module to control GimbalLG
 * @pre GimbalSKD has started properly
 * @usage Invoke set_mode() and set_target_angle() to control gimbal
 * @note define ENABLE_VISION and ENABLE_SUBPITCH on external files to enable vision control and sub pitch controller.
 * @date 3.16, 2022
 * @version 3.0a
 */
class GimbalLG {
public:
    enum mode_t {
        FORCED_RELAX_MODE,  ///< Zero force (but still taking control of GimbalIF)
        CHASSIS_REF_MODE,   ///< Chassis Reference
        GIMBAL_REF_MODE,    ///< Gimbal Reference
        VISION_MODE         ///< Gimbal Reference, Vision override input
    };

    static mode_t mode;
    /**
     * Initialize the gimbal Logic
     * @param vision_prio Vision thread priority.
     * @param ballistic_compensate_prio Gimbal control thread priority.
     */
    static void init (tprio_t vision_prio, tprio_t ballistic_compensate_prio);

    /**
     * Set the target angle in current reference mode.
     * @param yaw_target_angle
     * @param pitch_target_angle
     */
    static void set_target_angle(float yaw_target_angle, float pitch_target_angle);

    /**
     * Set mode of gimbal logic.
     * @param mode_
     */
    static void set_mode(mode_t mode_);

    /**
     * Get current feedback of certain axis.
     * @param angle (GimbalSKD::angle_id_t) The desired axis.
     * @return The current feedback angle of certain axis.
     */
    static float get_feedback_angle(GimbalSKD::angle_id_t angle);

    /**
     * Get target angle of certain axis.
     * @param angle (GimbalSKD::angle_id_t) The desired axis.
     * @return The target angle of certain axis.
     */
    static float get_target_angle(GimbalSKD::angle_id_t angle);

    /**
     * Get feedback angle from motor encoder.
     * @param angle (GimbalSKD::angle_id_t) The desired axis,
     * @return The motor encoder feedback angel of certain axis.
     */
    static float get_motor_angle(GimbalSKD::angle_id_t angle);

private:
#if ENABLE_VISION == TRUE
    class VisionControlThread : public BaseStaticThread<512> {
        event_listener_t vision_listener{};
        void main() final;
    };

    static VisionControlThread vision_control_thread;
#endif
#if ENABLE_SUBPITCH == TRUE
    class BallisticCompensateThread : public BaseStaticThread<512>{
        time_msecs_t INTERVAL = 5;
        void main() final;
    };

    static BallisticCompensateThread ballistic_compensate_thread;
#endif
};

#endif //META_INFANTRY_GIMBAL_LOGIC_H

/** @} */