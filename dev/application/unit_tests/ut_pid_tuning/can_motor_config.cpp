//
// Created by Tianyi Han on 2/21/2023.
//

#include "can_motor_config.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        // {CANMotorBase::can_channel_1, 0x205, CANMotorBase::GM6020, 3572},  // 6020
        {CANMotorBase::can_channel_1, 0x202, CANMotorBase::M2006, 0},  //2006
        {CANMotorBase::can_channel_2, 0x201, CANMotorBase::M2006, 0},  //2006
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        // {60.0, 0.0f, 0.0, 1000.0, 30000.0},  //6020
        {10.0, 0.0f, 0.0, 100.0, 10000.0},  //2006
        {10.0, 0.0f, 0.0, 100.0, 10000.0},  //2006
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        // {2.0f,3.0f,0.0f,10000.0,30000.0},  //6020
        {20.0f,0.0f,0.0f,100.0,10000.0},  //2006
        {20.0f,0.0f,0.0f,100.0,10000.0},  //2006
};

bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {
        false,
        false,
};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {
        true,
        true,
};

bool CANMotorCFG::enable_limits[MOTOR_COUNT] = {
        false,
        false
};

float CANMotorCFG::min_limit_deg[MOTOR_COUNT] = {
        0.0f,
        0.0f
};

float CANMotorCFG::max_limit_deg[MOTOR_COUNT] = {
        0.0f,
        0.0f
};