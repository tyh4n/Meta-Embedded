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
            float target_vel_2 = Remote::rc.ch2 * 2000.0f;

            // Send target velocities to the PID controllers
            CANMotorController::set_target_vel(CANMotorCFG::MOTOR1, target_vel_1);
            CANMotorController::set_target_vel(CANMotorCFG::MOTOR2, target_vel_2);

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

int main(void) {
    halInit();
    System::init();

    // Start Shell for printing
    Shell::start(HIGHPRIO);
    LED::all_off();

    // Start the DBUS Remote interpreter
    Remote::start();

    // Start CAN Interfaces
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO + 1);

    // Start Motor Controller
    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);

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