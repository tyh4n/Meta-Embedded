//
// Created by Tianyi Han on 3/18/2026.
//
// can_motor_config.cpp
//

#include "can_motor_config.h"

CANMotorBase CANMotorCFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_1, 0x201, CANMotorBase::M2006, 0},
        {CANMotorBase::can_channel_2, 0x201, CANMotorBase::M2006, 0},
        {CANMotorBase::can_channel_1, 0x202, CANMotorBase::M2006, 0},
        {CANMotorBase::can_channel_2, 0x202, CANMotorBase::M2006, 0},
        {CANMotorBase::can_channel_1, 0x203, CANMotorBase::M2006, 0},
        {CANMotorBase::can_channel_2, 0x203, CANMotorBase::M2006, 0},
        {CANMotorBase::can_channel_1, 0x204, CANMotorBase::M2006, 0},
        {CANMotorBase::can_channel_2, 0x204, CANMotorBase::M2006, 0},
};

PIDController::pid_params_t CANMotorCFG::a2vParams[MOTOR_COUNT] = {
        {5.0, 0.1f, 2.0, 1000.0, 5000.0},
        {5.0, 0.1f, 2.0, 1000.0, 5000.0},
        {5.0, 0.1f, 1.0, 1000.0, 5000.0},
        {5.0, 0.1f, 1.0, 1000.0, 5000.0},
        {5.0, 0.1f, 1.0, 1000.0, 5000.0},
        {5.0, 0.1f, 1.0, 1000.0, 5000.0},
        {5.0, 0.1f, 1.0, 1000.0, 5000.0},
        {5.0, 0.1f, 1.0, 1000.0, 5000.0},
};

PIDController::pid_params_t CANMotorCFG::v2iParams[MOTOR_COUNT] = {
        {20.0f, 0.0f, 1.0f, 100.0, 5000.0},
        {20.0f, 0.0f, 1.0f, 100.0, 5000.0},
        {20.0f, 0.0f, 1.0f, 100.0, 5000.0},
        {20.0f, 0.0f, 1.0f, 100.0, 5000.0},
        {20.0f, 0.0f, 1.0f, 100.0, 5000.0},
        {20.0f, 0.0f, 1.0f, 100.0, 5000.0},
        {20.0f, 0.0f, 1.0f, 100.0, 5000.0},
        {20.0f, 0.0f, 1.0f, 100.0, 5000.0},
};

bool CANMotorCFG::enable_a2v[MOTOR_COUNT] = {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
};

bool CANMotorCFG::enable_v2i[MOTOR_COUNT] = {
        true,
        true,
        true,
        true,
        true,
        true,
        true,
        true,
};

bool CANMotorCFG::enable_limits[MOTOR_COUNT] = {
        false,
        false,
        false,
        false,
        false,
        false,
        false,
        false,
};

float CANMotorCFG::min_limit_deg[MOTOR_COUNT] = {
        -5000.0f,
        -360.0f,  //-900
        -5000.0f,
        -360.0f,  //-900
        -5000.0f,
        -360.0f,  //-900
        -5000.0f,
        -360.0f,  //-900
};

float CANMotorCFG::max_limit_deg[MOTOR_COUNT] = {
        360.0f,  //900
        10000.0f,
        360.0f,  //900
        10000.0f,
        360.0f,  //900
        10000.0f,
        360.0f,  //900
        10000.0f,
};