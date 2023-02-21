//
// Created by Tianyi Han on 2/21/2023.
//

#include "can_motor_config.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_1, 0x201, CANMotorBase::GM6020, 3572},
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {10.0, 0.0f, 0.0, 100.0, 500.0},
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {10.0f,0.0f,0.0f,100.0,3000.0},
};

bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {
        false,
};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {
        false,
};