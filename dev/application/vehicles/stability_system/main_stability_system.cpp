//
// Created by Tianyi Han on 3/18/2026.
// main_stability_system.cpp
//

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

using namespace chibios_rt;

// Motor homing configuration struct
struct HomingConfig {
    CANMotorCFG::motor_id_t m0;
    CANMotorCFG::motor_id_t m1;
    float velocity0;
    float velocity1;
    float current_limit0;
    float current_limit1;
    float offset;
};

/**
    Motor homing configuration table
 */
static const HomingConfig homing_list[] = {
    {CANMotorCFG::MOTOR1, CANMotorCFG::MOTOR2, 360.0f, -360.0f, 700.0f, 700.0f, 360.0f},
    {CANMotorCFG::MOTOR3, CANMotorCFG::MOTOR4, 360.0f, -360.0f, 700.0f, 700.0f, 360.0f},
    {CANMotorCFG::MOTOR5, CANMotorCFG::MOTOR6, 360.0f, -360.0f, 700.0f, 700.0f, 360.0f},
    {CANMotorCFG::MOTOR7, CANMotorCFG::MOTOR8, 360.0f, -360.0f, 700.0f, 700.0f, 360.0f},
};

#define HOMING_GROUP_COUNT (sizeof(homing_list) / sizeof(HomingConfig))

// CAN interface
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

// IK Globals
volatile float target_tx = 0.0f;
volatile float target_ty = 0.0f;
volatile bool new_target_available = false;

// Homing thread
class HomingThread : public chibios_rt::BaseStaticThread<512> {
public:
    volatile bool is_homed = false;

private:
    HomingConfig cfg;

public:
    HomingThread(HomingConfig config) : cfg(config) {}

protected:
    void main() final {
        setName("DualHoming");

        CANMotorCFG::motor_id_t motors[2] = {cfg.m0, cfg.m1};
        float velocity[2] = {cfg.velocity0, cfg.velocity1};
        float current_limit[2] = {cfg.current_limit0, cfg.current_limit1};
        float original_max_out[2];

        // Lower current limits and start moving toward mechanical limit
        for (int i = 0; i < 2; i++) {
            // Update the out_limit (Current Limit)
            original_max_out[i] = CANMotorCFG::v2iParams[motors[i]].out_limit;
            CANMotorCFG::v2iParams[motors[i]].out_limit = current_limit[i];

            // Push the updated struct to the live v2i controller
            CANMotorController::load_PID_params(motors[i], false, CANMotorCFG::v2iParams[motors[i]]);

            CANMotorCFG::enable_v2i[motors[i]] = true;
            CANMotorCFG::enable_a2v[motors[i]] = false;

            CANMotorController::set_target_vel(motors[i], velocity[i]);
        }

        chThdSleepMilliseconds(100);

        // Stall detection
        bool stalled[2] = {false, false};
        int stall_counter[2] = {0, 0};
        int timeout_counter = 0;

        while ((!stalled[0] || !stalled[1]) && timeout_counter < 10000) {
            for (int i = 0; i < 2; i++) {
                if (!stalled[i]) {
                    float current_vel = CANMotorIF::motor_feedback[motors[i]].actual_velocity;

                    if (current_vel > -5.0f && current_vel < 5.0f) {
                        if (++stall_counter[i] >= 10) {
                            stalled[i] = true;
                            CANMotorController::set_target_vel(motors[i], 0);
                        }
                    } else {
                        stall_counter[i] = 0;
                    }
                }
            }
            timeout_counter++;
            chThdSleepMilliseconds(10);
        }

        // Zero accumulate angle and move to PID offset
        chThdSleepMilliseconds(100);
        for (int i = 0; i < 2; i++) {
            CANMotorIF::motor_feedback[motors[i]].reset_accumulate_angle();

            // 1. Restore the struct
            CANMotorCFG::v2iParams[motors[i]].out_limit = original_max_out[i];

            // 2. Push the restored struct back to the active controller
            CANMotorController::load_PID_params(motors[i], false, CANMotorCFG::v2iParams[motors[i]]);

            CANMotorCFG::enable_a2v[motors[i]] = true;

            float target_angle = (velocity[i] < 0) ? cfg.offset : -cfg.offset;
            CANMotorController::set_target_angle(motors[i], target_angle);
        }
        // Wait for motor to reach target angle
        chThdSleepMilliseconds(1000);

        // Disable a2v control
        for (int i = 0; i < 2; i++) {
            CANMotorIF::motor_feedback[motors[i]].reset_accumulate_angle();
            CANMotorCFG::enable_limits[motors[i]] = true;
            CANMotorCFG::enable_a2v[motors[i]] = false;
        }

        is_homed = true;
    }
};

// Global array of homing thread pointers
HomingThread* homingThreads[HOMING_GROUP_COUNT];

// --- Inverse Kinematics Logic ---
bool solve_linkage_IK(float tx, float ty, float &x1, float &x2) {
    const float r1 = 43.4f;
    const float r2 = 82.09f;
    const float L_ext = 112.8f;
    const float L_perp = 11.65f;
    const float A1_x = 48.0f;
    const float B_x = 0.0f;

    // 1. Pre-calculate the rigid arm geometry (A1 to A3)
    float arm_long = r1 + L_ext;
    float R_sq = (arm_long * arm_long) + (L_perp * L_perp);
    float R = std::sqrt(R_sq);
    float phi_offset = std::atan2(L_perp, arm_long);

    // 2. Solve for A1_y (x2)
    float dx = tx - A1_x;
    float dy_sq = R_sq - (dx * dx);

    if (dy_sq < 0.0f) {
        return false; // Target X coordinate is physically unreachable
    }

    float A1_y = ty + std::sqrt(dy_sq);
    x2 = A1_y;

    // 3. Locate A2 (the joint) to find B
    float theta_A1A3 = std::atan2(ty - A1_y, tx - A1_x);
    float theta_A1A2 = theta_A1A3 - phi_offset;

    float A2_x = A1_x + r1 * std::cos(theta_A1A2);
    float A2_y = A1_y + r1 * std::sin(theta_A1A2);

    // 4. Solve for B_y (x1)
    float dx_B = A2_x - B_x;
    float dy_B_sq = (r2 * r2) - (dx_B * dx_B);

    if (dy_B_sq < 0.0f) {
        return false; // Target configuration results in A2 being too far from Slider B
    }

    x1 = A2_y - std::sqrt(dy_B_sq);
    return true;
}

// --- IK Control Thread ---
class IKControlThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("IKControl");

        while (!shouldTerminate()) {
            if (new_target_available) {
                new_target_available = false;

                float x1, x2;
                if (solve_linkage_IK(target_tx, target_ty, x1, x2)) {

                    // Convert sliders to motor angles based on specifications
                    float target_angle_m0 = (x1 + 110.0f) * 360.0f;
                    float target_angle_m1 = x2 * -360.0f;

                    // Apply to all configured motor groups
                    for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
                        // Re-enable a2v control (it was disabled at the end of homing)
                        CANMotorCFG::enable_a2v[homing_list[i].m0] = true;
                        CANMotorCFG::enable_a2v[homing_list[i].m1] = true;

                        CANMotorController::set_target_angle(homing_list[i].m0, target_angle_m0);
                        CANMotorController::set_target_angle(homing_list[i].m1, target_angle_m1);
                    }

                    Shell::printf("IK Success: Motors moving to X1: %.2f | X2: %.2f" SHELL_NEWLINE_STR, x1, x2);
                } else {
                    Shell::printf("IK Error: Target coordinates unreachable." SHELL_NEWLINE_STR);
                }
            }
            sleep(TIME_MS2I(20)); // Polling rate
        }
    }
} ikControlThread;

// --- Shell Command Handler for IK ---
bool cmd_set_ik(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)chp; // Suppress unused warning
    if (argc != 2) {
        Shell::printf("Usage: ik <tx> <ty>" SHELL_NEWLINE_STR);
        return false;
    }

    target_tx = std::atof(argv[0]);
    target_ty = std::atof(argv[1]);
    new_target_available = true;

    Shell::printf("New Target: tx=%.2f, ty=%.2f" SHELL_NEWLINE_STR, target_tx, target_ty);

    return true;
}

// Register the IK command
static const Shell::Command system_commands[] = {
    {"ik", "Set Inverse Kinematics target (tx ty)", cmd_set_ik, nullptr},
    {nullptr, nullptr, nullptr, nullptr}
};


// Remote Control Thread
class RemoteControlThread : public BaseStaticThread<1024> {
private:
    // Define physical limits
    static constexpr float MAX_DELTA_Y_MM = 54.1f; // Corresponds to -45 degrees
    static constexpr float MIN_DELTA_Y_MM = 0.0f;  // Corresponds to -90 degrees

    // Define pitch
    static constexpr float MM_PER_DEGREE = 2.0f / 360.0f;

    void main() final {
        setName("RemoteControl");

        while (!shouldTerminate()) {
            float target_vel_1 = Remote::rc.ch0 * 2000.0f;
            float target_vel_2 = - Remote::rc.ch2 * 2000.0f;

            // Deadband filter
            if (target_vel_1 > -100.0f && target_vel_1 < 100.0f) target_vel_1 = 0.0f;
            if (target_vel_2 > -100.0f && target_vel_2 < 100.0f) target_vel_2 = 0.0f;

            // Loop through all configured homing groups to apply control
            for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
                // Calculate current delta y
                float y_m0 = CANMotorIF::motor_feedback[homing_list[i].m0].accumulate_angle() * MM_PER_DEGREE;
                float y_m1 = CANMotorIF::motor_feedback[homing_list[i].m1].accumulate_angle() * MM_PER_DEGREE;

                // Check -45 degree limit
                if (y_m0 + y_m1 >= MAX_DELTA_Y_MM) {
                    // Prevent m0 from moving further positive
                    if (target_vel_1 > 0.0f) target_vel_1 = 0.0f;
                    // Prevent m1 from moving further positive
                    if (target_vel_2 > 0.0f) target_vel_2 = 0.0f;
                }

                // Check -90 degree limit
                if (y_m0 + y_m1 <= MIN_DELTA_Y_MM) {
                    // Prevent m0 from moving further negative
                    if (target_vel_1 < 0.0f) target_vel_1 = 0.0f;
                    // Prevent m1 from moving further negative
                    if (target_vel_2 < 0.0f) target_vel_2 = 0.0f;
                }

                CANMotorController::set_target_vel(homing_list[i].m0, target_vel_1);
                CANMotorController::set_target_vel(homing_list[i].m1, target_vel_2);
            }

            sleep(TIME_MS2I(20));
        }
    }
} remoteControlThread;

// Watchdog Thread
class RemoteWatchdogThread : public chibios_rt::BaseStaticThread<256> {
private:
    void main() final {
        setName("RemoteWdg");
        while (!shouldTerminate()) {
            if (chibios_rt::System::getTime() - Remote::last_update_time > TIME_MS2I(100)) {
                Remote::uart_synchronize(true);
                Remote::last_update_time = chibios_rt::System::getTime();
            }
            sleep(TIME_MS2I(50));
        }
    }
} remoteWatchdogThread;

int main(void) {
    halInit();
    System::init();

    // Start the Shell with our custom commands BEFORE anything else
    Shell::addCommands(system_commands);
    Shell::start(HIGHPRIO);

    LED::all_off();
    LED::red_on();

    Remote::start();
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO + 1);
    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);

    // Initialize and Start all homing threads from the config list
    for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
        homingThreads[i] = new HomingThread(homing_list[i]);
        homingThreads[i]->start(NORMALPRIO + 4);
    }

    // Block until all motors are homed
    bool all_ready = false;
    while (!all_ready) {
        all_ready = true;
        for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
            if (!homingThreads[i]->is_homed) {
                all_ready = false;
                break;
            }
        }
        chThdSleepMilliseconds(10);
    }

    // Start Operational Threads
    LED::red_off();
    LED::green_on();

    // Starting the new Inverse Kinematics Control Thread
    ikControlThread.start(NORMALPRIO + 4);

    remoteWatchdogThread.start(NORMALPRIO - 1);

#if CH_CFG_NO_IDLE_THREAD
    while (true) {}
#else
    BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}