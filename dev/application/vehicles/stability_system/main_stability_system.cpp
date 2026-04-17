//
// Created by Tianyi Han on 3/18/2026.
// main_stability_system.cpp
//

#include "ch.hpp"
#include "hal.h"

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
    {CANMotorCFG::MOTOR1, CANMotorCFG::MOTOR2, 360.0f, -360.0f, 300.0f, 300.0f, 360.0f},
    {CANMotorCFG::MOTOR3, CANMotorCFG::MOTOR4, 360.0f, -360.0f, 300.0f, 300.0f, 360.0f},
    {CANMotorCFG::MOTOR5, CANMotorCFG::MOTOR6, 360.0f, -360.0f, 300.0f, 300.0f, 360.0f},
    {CANMotorCFG::MOTOR7, CANMotorCFG::MOTOR8, 360.0f, -360.0f, 300.0f, 300.0f, 360.0f},
};

#define HOMING_GROUP_COUNT (sizeof(homing_list) / sizeof(HomingConfig))

// CAN interface
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

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
            original_max_out[i] = CANMotorCFG::v2iParams[motors[i]].i_limit;
            CANMotorCFG::v2iParams[motors[i]].i_limit = current_limit[i];

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

                    if (i == 1)
                    {
                        Shell::printf("%.2f" SHELL_NEWLINE_STR, current_vel);
                    }

                    if (current_vel > -1.0f && current_vel < 1.0f) {
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
            CANMotorCFG::enable_limits[motors[i]] = true;

            // Restore original current limits and switch to angle control
            CANMotorCFG::v2iParams[motors[i]].i_limit = original_max_out[i];
            CANMotorCFG::enable_a2v[motors[i]] = true;

            float target_angle = (velocity[i] < 0) ? cfg.offset : -cfg.offset;
            CANMotorController::set_target_angle(motors[i], target_angle);
        }

        // Wait for motor to reach target angle
        chThdSleepMilliseconds(2000);

        // Disable a2v control
        for (int i = 0; i < 2; i++) {
            CANMotorCFG::enable_a2v[motors[i]] = false;
        }

        is_homed = true;
    }
};

// Global array of homing thread pointers
HomingThread* homingThreads[HOMING_GROUP_COUNT];

// Remote Control Thread
class RemoteControlThread : public BaseStaticThread<1024> {
private:
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
                CANMotorController::set_target_vel(homing_list[i].m0, target_vel_1);
                CANMotorController::set_target_vel(homing_list[i].m1, target_vel_2);
            }

            // Shell::printf("V1: %.2f | V2: %.2f" SHELL_NEWLINE_STR, target_vel_1, target_vel_2);

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

    remoteControlThread.start(NORMALPRIO + 4);
    remoteWatchdogThread.start(NORMALPRIO - 1);

#if CH_CFG_NO_IDLE_THREAD
    while (true) {}
#else
    BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}