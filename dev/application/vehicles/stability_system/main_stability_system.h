//
// main_stability_system.h
//

#ifndef MAIN_STABILITY_SYSTEM_H
#define MAIN_STABILITY_SYSTEM_H

#include "ch.hpp"
#include "hal.h"
#include <cmath>
#include <cstdlib>

#include "led.h"
#include "shell.h"
#include "can_motor_interface.h"
#include "can_motor_controller.h"
#include "remote_interpreter.h"
#include "hardware_conf.h"

// --- Physical Workspace Limits ---
constexpr float IK_MIN_R = 272.61f;
constexpr float IK_MAX_R = 385.14f;
constexpr float IK_MIN_H = 0.0f;
constexpr float IK_MAX_H = 50.0f;

/**
 * @brief Configuration parameters for motor homing sequences.
 */
struct HomingConfig {
    CANMotorCFG::motor_id_t m0;     /**< First motor in the leg linkage */
    CANMotorCFG::motor_id_t m1;     /**< Second motor in the leg linkage */
    float velocity0;                /**< Homing velocity for m0 */
    float velocity1;                /**< Homing velocity for m1 */
    float current_limit0;           /**< Stall detection current limit for m0 */
    float current_limit1;           /**< Stall detection current limit for m1 */
    float offset;                   /**< Back-off angle offset after mechanical stall */
};

/**
 * @brief Union to reconstruct 32-bit floating point values from 16-bit SPI words.
 */
union float_int {
    uint16_t parts[2];
    float value;
};

/**
 * @brief Solves the inverse kinematics for the linkage system to reach a Cartesian target.
 * * @param r_leg Target r coordinate in millimeters.
 * @param h_leg Target h coordinate in millimeters.
 * @param x1 Reference to the calculated linear position for slider 1.
 * @param x2 Reference to the calculated linear position for slider 2.
 * @return true if the target coordinates are mechanically reachable.
 * @return false if the target is out of the theoretical bounds of the linkages.
 */
bool solve_linkage_IK(float r_leg, float h_leg, float &x1, float &x2);

/**
 * @brief Shell command handler for manually setting an IK target.
 * * @param chp Pointer to the BaseSequentialStream for shell output.
 * @param argc Number of arguments passed to the command.
 * @param argv Array of character pointers to the arguments (expects tx and ty).
 * @return true if the command executed successfully.
 */
bool cmd_set_ik(BaseSequentialStream *chp, int argc, char *argv[]);

/**
 * @brief Overrides ChibiOS defaults to force the SPI2 driver into hardware slave mode.
 */
void forceSpiSlave();

#endif // MAIN_STABILITY_SYSTEM_H