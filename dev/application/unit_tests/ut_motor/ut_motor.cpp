//
// Created by Tianyi Han on 2/20/2023.
//

#include "ch.hpp"
#include "hal.h"

#include "interface/led/led.h"
#include "shell.h"
#include "can_motor_interface.h"
#include "can_motor_controller.h"
#include "hardware_conf.h"

using namespace chibios_rt;
CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

class MotorControl :public BaseStaticThread<512> {
private:
    void main() final{
        setName("feedback");
        while(!shouldTerminate()){
            // CANMotorController::set_target_angle(CANMotorCFG::MOTOR1, 60);
            CANMotorController::set_target_vel(CANMotorCFG::MOTOR1, 500);
            CANMotorController::set_target_vel(CANMotorCFG::MOTOR2, 500);
            sleep(TIME_MS2I(10));
            // CANMotorController::set_target_angle(CANMotorCFG::MOTOR1, -60);
            // CANMotorController::set_target_vel(CANMotorCFG::MOTOR1, -10);
            // sleep(TIME_MS2I(1000));
            // Shell::printf("%.4f" ENDL, CANMotorIF::motor_feedback[0].actual_angle);
        }
    }
} ControlThread;

int main(void) {
    halInit();
    System::init();
    LED::all_off();

    // see CanMotorController feedback thread for shell feedback
    Shell::start(HIGHPRIO);

    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO+1);

    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);

    ControlThread.start(NORMALPRIO+4);
    // green LED on, indicating thread running
    LED::green_on();

#if CH_CFG_NO_IDLE_THREAD  // See chconf.h for what this #define means.
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When vehicle() quits, the vehicle thread will somehow enter an infinite loop, so we set the
    // priority to lowest before quitting, to let other threads run normally
    chibios_rt::BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}