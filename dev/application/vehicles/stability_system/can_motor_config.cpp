//
// Created by Tianyi Han on 3/18/2026.
//
// can_motor_config.cpp
//

#include "can_motor_config.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_1, 0x201, CANMotorBase::M2006, 0},
        {CANMotorBase::can_channel_2, 0x202, CANMotorBase::M2006, 0},
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {10.0, 0.1f, 0.0, 1000.0, 10000.0},
        {10.0, 0.1f, 0.0, 1000.0, 10000.0},
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {20.0f, 0.0f, 0.0f, 100.0, 10000.0},
        {20.0f, 0.0f, 0.0f, 100.0, 10000.0},
};

bool CANMotorCFG::enable_a2v[MOTOR_COUNT] {
        false,
        false
};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT] {
        true,
        true
};