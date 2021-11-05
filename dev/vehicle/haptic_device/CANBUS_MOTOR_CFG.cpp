//
// Created by 钱晨 on 10/29/21.
//

#include "CANBUS_MOTOR_CFG.h"

CANMotorBase CANBUS_MOTOR_CFG::CANMotorProfile[MOTOR_COUNT] = {
        {CANMotorBase::can_channel_1, 0x205, CANMotorBase::M3508_without_deceleration, 3572}
};

PIDController::pid_params_t CANBUS_MOTOR_CFG::a2vParams[MOTOR_COUNT] = {
        {10, 0.0f, 0.2, 100, 500},
};

PIDController::pid_params_t CANBUS_MOTOR_CFG::v2iParams[MOTOR_COUNT] = {
        {10,0.2,0.000,4000.0,8100.0},
};

bool CANBUS_MOTOR_CFG::enable_a2v[MOTOR_COUNT] {
    true,
};

CANBUS_MOTOR_CFG::v2i_PID_status_t CANBUS_MOTOR_CFG::enable_v2i[MOTOR_COUNT] {
    CANBUS_MOTOR_CFG::WORKING,
};