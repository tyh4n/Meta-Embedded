//
// Created by liuzikai on 2019-06-26.
//

/**
 * @file    gimbal_logic.cpp
 * @brief   By-passing logic-level module to control GimbalLG
 *
 * @addtogroup gimbal
 * @{
 */

#include "gimbal_logic.h"

GimbalLG::action_t GimbalLG::action = FORCED_RELAX_MODE;

void GimbalLG::init() {}

int GimbalLG::Auxiliary_ON = 0;

GimbalLG::action_t GimbalLG::get_action() {
    return action;
}

void GimbalLG::set_action(GimbalLG::action_t value) {
    if (value == action) return;  // avoid repeating setting target_theta, etc.

    action = value;
    if (action == FORCED_RELAX_MODE) {
        GimbalSKD::set_mode(GimbalSKD::FORCED_RELAX_MODE);
    } else if (action == ABS_ANGLE_MODE || action == AERIAL_MODE) {
        GimbalSKD::set_mode(GimbalSKD::ABS_ANGLE_MODE);
    } else if (action == SENTRY_MODE) {
        GimbalSKD::set_mode(GimbalSKD::SENTRY_MODE);
    }
}

void GimbalLG::set_target(float yaw_target_angle, float pitch_target_angle) {
    if (action != FORCED_RELAX_MODE) {

        GimbalSKD::set_target_angle(yaw_target_angle, pitch_target_angle);

    } else {
        LOG_ERR("GimbalLG - set_target(): invalid mode");
    }
}

float GimbalLG::get_accumulated_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_accumulated_angle(motor);
}

float GimbalLG::get_relative_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_relative_angle(motor);
}

float GimbalLG::get_current_target_angle(GimbalBase::motor_id_t motor) {
    return GimbalSKD::get_target_angle(motor);
}

int GimbalLG::should_override_operator(float mouse_yaw,float mouse_pitch){
    float bias_yaw = GimbalLG::abs_f(VisionPort::enemy_info.yaw_angle);
    float bias_pitch = GimbalLG::abs_f(VisionPort::enemy_info.pitch_angle);
    ///one more step to filter the bad detections
    ///
    ///
    mouse_yaw = GimbalLG::abs_f(mouse_yaw);
    mouse_pitch = GimbalLG::abs_f(mouse_pitch);
    if ((bias_yaw < 5)&&(bias_pitch < 5)&&(mouse_pitch<15)&&(mouse_yaw<15)){
        return 1;
    }
    else{
        return 0;
    }
}
float GimbalLG::abs_f(float val) {
    if (val < 0)
        return (-1*val);
    else
        return val;
}
/** @} */