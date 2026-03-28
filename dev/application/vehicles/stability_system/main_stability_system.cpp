//
// Created by Tianyi Han on 3/18/2026.
//
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

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

class RemoteControlThread : public BaseStaticThread<1024> {
private:
    void main() final {
        setName("RemoteControl");

        while (!shouldTerminate()) {
            // The remote channels return normalized values from -1.0 to 1.0
            // Multiply by 200.0f to scale the threshold to [-200, 200]
            float target_vel_1 = Remote::rc.ch0 * 2000.0f;
            float target_vel_2 = - Remote::rc.ch2 * 2000.0f;

            target_vel_1 = (-100.0f < target_vel_1 && target_vel_1 < 100.0f) ? 0.0f: target_vel_1;
            target_vel_2 = (-100.0f < target_vel_2 && target_vel_2 < 100.0f) ? 0.0f: target_vel_2;

            // Send target velocities to the PID controllers
            // CANMotorController::set_target_vel(CANMotorCFG::MOTOR1, target_vel_1);
            // CANMotorController::set_target_vel(CANMotorCFG::MOTOR2, target_vel_2);

            CANMotorController::set_target_vel(CANMotorCFG::MOTOR3, target_vel_1);
            CANMotorController::set_target_vel(CANMotorCFG::MOTOR4, target_vel_2);

            // CANMotorController::set_target_vel(CANMotorCFG::MOTOR5, target_vel_1);
            // CANMotorController::set_target_vel(CANMotorCFG::MOTOR6, target_vel_2);
            //
            // CANMotorController::set_target_vel(CANMotorCFG::MOTOR7, target_vel_1);
            // CANMotorController::set_target_vel(CANMotorCFG::MOTOR8, target_vel_2);

            // Print the values to the Shell
            Shell::printf("CH0: %.2f -> V1: %.2f | CH2: %.2f -> V2: %.2f" SHELL_NEWLINE_STR,
                          Remote::rc.ch0, target_vel_1,
                          Remote::rc.ch2, target_vel_2);

            // Run at ~50Hz
            sleep(TIME_MS2I(20));
        }
    }
} remoteControlThread;

class RemoteWatchdogThread : public chibios_rt::BaseStaticThread<256> {
private:
    void main() final {
        setName("RemoteWdg");
        while (!shouldTerminate()) {
            // If 100ms passes without a successful DBUS frame update
            if (chibios_rt::System::getTime() - Remote::last_update_time > TIME_MS2I(100)) {

                // Force the UART to resynchronize.
                // Passing 'true' makes it block here patiently until the cable is plugged back in.
                Remote::uart_synchronize(true);

                // Reset the timer so it doesn't instantly trigger again
                Remote::last_update_time = chibios_rt::System::getTime();
            }
            sleep(TIME_MS2I(50));
        }
    }
} remoteWatchdogThread;

/**
 * @brief Concurrent Homing Thread Template with PID Standoff
 * @tparam MOTOR_ID The ID of the motor to home
 * @tparam HOMING_CURRENT The open-loop current to apply to find the hard stop
 * @tparam HOME_OFFSET_DEG The degrees to precisely back away using PID before setting absolute zero
 */
template <CANMotorCFG::motor_id_t MOTOR_ID, int HOMING_CURRENT, int HOME_OFFSET_DEG>
class HomingThread : public chibios_rt::BaseStaticThread<256> {
public:
    volatile bool is_homed = false;

protected:
    void main() final {
        setName("HomingThd");

        // Disable both PID loops for open-loop hard-stop finding
        CANMotorCFG::enable_v2i[MOTOR_ID] = false;
        CANMotorCFG::enable_a2v[MOTOR_ID] = false;

        // Find the physical hard stop
        CANMotorController::set_target_current(MOTOR_ID, HOMING_CURRENT);
        chThdSleepMilliseconds(100);

        int stall_counter = 0;
        int timeout_counter = 0;

        while (stall_counter < 10 && timeout_counter < 1000) {
            float current_vel = CANMotorIF::motor_feedback[MOTOR_ID].actual_velocity;
            if (current_vel > -2.0f && current_vel < 2.0f) { stall_counter++; }
            else { stall_counter = 0; }
            timeout_counter++;
            chThdSleepMilliseconds(10);
        }

        // Stop and temporarily zero the encoder at the physical limit
        CANMotorController::set_target_current(MOTOR_ID, 0);
        chThdSleepMilliseconds(100);
        CANMotorIF::motor_feedback[MOTOR_ID].reset_accumulate_angle();

        // Re-enable BOTH loops for precise Angle tracking
        CANMotorCFG::enable_v2i[MOTOR_ID] = true;
        CANMotorCFG::enable_a2v[MOTOR_ID] = true;

        // Calculate the direction to back off
        float target_angle = (HOMING_CURRENT < 0) ? HOME_OFFSET_DEG : -HOME_OFFSET_DEG;
        CANMotorController::set_target_angle(MOTOR_ID, target_angle);

        int pid_timeout_counter = 0;

        // Wait for the PID controller to reach the target angle (with 3 second timeout)
        while (pid_timeout_counter < 300) {
            // Fetch the single-turn angle and the total number of revolutions
            float current_angle = CANMotorIF::motor_feedback[MOTOR_ID].accumulate_angle();

            // Calculate the error
            float error = target_angle - current_angle;

            // Using 5.0f to allow for steady-state friction error
            if (error > -5.0f && error < 5.0f) {
                break;
            }

            pid_timeout_counter++;
            chThdSleepMilliseconds(10);
        }

        // Give the PID a tiny moment to settle perfectly
        chThdSleepMilliseconds(100);

        // Set position as zero
        CANMotorIF::motor_feedback[MOTOR_ID].reset_accumulate_angle();
        // Enable software limits
        // CANMotorCFG::enable_limits[MOTOR_ID] = true;

        // Disable Angle PID, leaving only Velocity PID active for the remote thread
        CANMotorCFG::enable_a2v[MOTOR_ID] = false;

        is_homed = true;
    }
};

// Home with -500 current, establish a 360-degree Home Offset using PID
HomingThread<CANMotorCFG::MOTOR1, 350, 360> homingThread1;
HomingThread<CANMotorCFG::MOTOR2, -350, 360> homingThread2;
HomingThread<CANMotorCFG::MOTOR3, 900, 360> homingThread3;
HomingThread<CANMotorCFG::MOTOR4, -400, 360> homingThread4;
HomingThread<CANMotorCFG::MOTOR5, 450, 360> homingThread5;
HomingThread<CANMotorCFG::MOTOR6, -450, 360> homingThread6;
HomingThread<CANMotorCFG::MOTOR7, 450, 360> homingThread7;
HomingThread<CANMotorCFG::MOTOR8, -450, 360> homingThread8;

int main(void) {
    halInit();
    System::init();

    // Start Shell for printing
    Shell::start(HIGHPRIO);
    LED::all_off();
    LED::red_on();

    // Start the DBUS Remote interpreter
    Remote::start();

    // Start CAN Interface
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO + 1);

    // Start Motor Controller
    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);

    // Start homing threads
    // homingThread1.start(NORMALPRIO + 4);
    // homingThread2.start(NORMALPRIO + 4);
    homingThread3.start(NORMALPRIO + 4);
    homingThread4.start(NORMALPRIO + 4);
    // homingThread5.start(NORMALPRIO + 4);
    // homingThread6.start(NORMALPRIO + 4);
    // homingThread7.start(NORMALPRIO + 4);
    // homingThread8.start(NORMALPRIO + 4);

    // Block the main boot sequence until motors are homed
    // while (!homingThread1.is_homed || !homingThread2.is_homed ||
    //     !homingThread3.is_homed || !homingThread4.is_homed ||
    //     !homingThread5.is_homed || !homingThread6.is_homed ||
    //     !homingThread7.is_homed || !homingThread8.is_homed) {
    //     chThdSleepMilliseconds(10);
    // }
    while (!homingThread3.is_homed || !homingThread4.is_homed) {
        chThdSleepMilliseconds(10);
    }
    LED::red_off();

    // Start our custom mapping thread
    remoteControlThread.start(NORMALPRIO + 4);

    // Start the Watchdog
    remoteWatchdogThread.start(NORMALPRIO - 1);

    // Indicator light
    LED::green_on();

#if CH_CFG_NO_IDLE_THREAD
    while (true) {}
#else
    BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}