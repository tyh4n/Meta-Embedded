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

// --- Physical Workspace Limits ---
constexpr float IK_MIN_X = 59.61f;
constexpr float IK_MAX_X = 172.14f;
constexpr float IK_MIN_Y = -211.2f; // -161.2 - 50.0
constexpr float IK_MAX_Y = -161.2f;

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
    {CANMotorCFG::MOTOR1, CANMotorCFG::MOTOR2, 360.0f, -360.0f, 700.0f, 700.0f, 900.0f},
    {CANMotorCFG::MOTOR3, CANMotorCFG::MOTOR4, 360.0f, -360.0f, 700.0f, 700.0f, 900.0f},
    {CANMotorCFG::MOTOR5, CANMotorCFG::MOTOR6, 360.0f, -360.0f, 700.0f, 700.0f, 900.0f},
    {CANMotorCFG::MOTOR7, CANMotorCFG::MOTOR8, 360.0f, -360.0f, 700.0f, 700.0f, 900.0f},
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
            original_max_out[i] = CANMotorCFG::v2iParams[motors[i]].out_limit;
            CANMotorCFG::v2iParams[motors[i]].out_limit = current_limit[i];
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

        chThdSleepMilliseconds(100);

        // Move to PID offset (Back off by 5mm)
        for (int i = 0; i < 2; i++) {
            CANMotorIF::motor_feedback[motors[i]].reset_accumulate_angle();

            CANMotorCFG::v2iParams[motors[i]].out_limit = original_max_out[i];
            CANMotorController::load_PID_params(motors[i], false, CANMotorCFG::v2iParams[motors[i]]);

            CANMotorCFG::enable_a2v[motors[i]] = true;
            float target_angle = (velocity[i] < 0) ? cfg.offset : -cfg.offset;
            CANMotorController::set_target_angle(motors[i], target_angle);
        }

        // Wait for motor to reach the back-off position
        chThdSleepMilliseconds(2000);

        // Define this back-off position as absolute zero for IK mapping
        for (int i = 0; i < 2; i++) {
            CANMotorIF::motor_feedback[motors[i]].reset_accumulate_angle();
            // Relying on Cartesian bounding box, disable motor hardware limits
            CANMotorCFG::enable_limits[motors[i]] = false;
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

    float arm_long = r1 + L_ext;
    float R_sq = (arm_long * arm_long) + (L_perp * L_perp);
    float R = std::sqrt(R_sq);
    float phi_offset = std::atan2(L_perp, arm_long);

    float dx = tx - A1_x;
    float dy_sq = R_sq - (dx * dx);

    if (dy_sq < 0.0f) return false;

    float A1_y = ty + std::sqrt(dy_sq);
    x2 = A1_y;

    float theta_A1A3 = std::atan2(ty - A1_y, tx - A1_x);
    float theta_A1A2 = theta_A1A3 - phi_offset;

    float A2_x = A1_x + r1 * std::cos(theta_A1A2);
    float A2_y = A1_y + r1 * std::sin(theta_A1A2);

    float dx_B = A2_x - B_x;
    float dy_B_sq = (r2 * r2) - (dx_B * dx_B);

    if (dy_B_sq < 0.0f) return false;

    x1 = A2_y - std::sqrt(dy_B_sq);
    return true;
}

// --- IK Control Thread (Idle) ---
class IKControlThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("IKControl");

        while (!shouldTerminate()) {
            if (new_target_available) {
                new_target_available = false;

                float x1, x2;
                if (solve_linkage_IK(target_tx, target_ty, x1, x2)) {

                    // Slider zero is now at x1=-115 and x2=-5. (180 deg per mm)
                    float target_angle_m0 = (x1 + 115.0f) * 180.0f;
                    float target_angle_m1 = (x2 + 5.0f) * -180.0f;

                    for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
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
    (void)chp;
    if (argc != 2) {
        Shell::printf("Usage: ik <tx> <ty>" SHELL_NEWLINE_STR);
        return false;
    }

    float tx = std::atof(argv[0]);
    float ty = std::atof(argv[1]);

    if (tx < IK_MIN_X || tx > IK_MAX_X || ty < IK_MIN_Y || ty > IK_MAX_Y) {
        Shell::printf("IK Error: Coordinate out of bounds (X: 59.61 to 172.14, Y: -211.2 to -161.2)." SHELL_NEWLINE_STR);
        return true;
    }

    target_tx = tx;
    target_ty = ty;
    new_target_available = true;

    Shell::printf("New Target: tx=%.2f, ty=%.2f" SHELL_NEWLINE_STR, target_tx, target_ty);
    return true;
}

static const Shell::Command system_commands[] = {
    {"ik", "Set Inverse Kinematics target (tx ty)", cmd_set_ik, nullptr},
    {nullptr, nullptr, nullptr, nullptr}
};

// --- Remote Cartesian Control Thread ---
class RemoteControlThread : public BaseStaticThread<1024> {
private:
    float current_x = 59.61f;
    float current_y = -161.2f;

    static constexpr float MAX_SPEED_MM_S = 10.0f;
    static constexpr float LOOP_DT_S = 0.01f;

    void main() final {
        setName("RemoteControl");

        // Engage position hold at physical zero immediately on startup
        for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
            CANMotorCFG::enable_a2v[homing_list[i].m0] = true;
            CANMotorCFG::enable_a2v[homing_list[i].m1] = true;
            CANMotorController::set_target_angle(homing_list[i].m0, 0.0f);
            CANMotorController::set_target_angle(homing_list[i].m1, 0.0f);
        }

        while (!shouldTerminate()) {
            float joy_x = Remote::rc.ch3;
            float joy_y = Remote::rc.ch2;

            if (joy_x > -0.05f && joy_x < 0.05f) joy_x = 0.0f;
            if (joy_y > -0.05f && joy_y < 0.05f) joy_y = 0.0f;

            if (joy_x != 0.0f || joy_y != 0.0f) {
                float next_x = current_x + (joy_x * MAX_SPEED_MM_S * LOOP_DT_S);
                float next_y = current_y + (joy_y * MAX_SPEED_MM_S * LOOP_DT_S);

                if (next_x < IK_MIN_X) next_x = IK_MIN_X;
                if (next_x > IK_MAX_X) next_x = IK_MAX_X;
                if (next_y < IK_MIN_Y) next_y = IK_MIN_Y;
                if (next_y > IK_MAX_Y) next_y = IK_MAX_Y;

                float x1, x2;
                if (solve_linkage_IK(next_x, next_y, x1, x2)) {
                    current_x = next_x;
                    current_y = next_y;

                    float target_angle_m0 = (x1 + 115.0f) * 180.0f;
                    float target_angle_m1 = (x2 + 5.0f) * -180.0f;

                    for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
                        CANMotorController::set_target_angle(homing_list[i].m0, target_angle_m0);
                        CANMotorController::set_target_angle(homing_list[i].m1, target_angle_m1);
                    }
                }
            }
            sleep(TIME_MS2I(10));
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

    Shell::addCommands(system_commands);
    Shell::start(HIGHPRIO);

    LED::all_off();
    LED::red_on();

    Remote::start();
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO + 1);
    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);

    for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
        homingThreads[i] = new HomingThread(homing_list[i]);
        homingThreads[i]->start(NORMALPRIO + 4);
    }

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

    LED::red_off();
    LED::green_on();

    // Switch to Remote Control Thread which now uses the Cartesian bounding box logic
    remoteControlThread.start(NORMALPRIO + 4);
    remoteWatchdogThread.start(NORMALPRIO - 1);

#if CH_CFG_NO_IDLE_THREAD
    while (true) {}
#else
    BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}