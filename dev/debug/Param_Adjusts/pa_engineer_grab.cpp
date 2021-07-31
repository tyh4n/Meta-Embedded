//
// Created by Qian Chen on 3/27/21.
//

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "debug/shell/shell.h"
#include "engineer_grab_skd.h"
#include "engineer_grab_mech_interface.h"
#include "remote_interpreter.h"
#include "vehicle/infantry/vehicle_infantry.h"
// Other headers here

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

PIDController::pid_params_t a2vPIDParameter[EngGrabMechBase::MOTOR_COUNT] = {SHOOT_PID_BULLET_LOADER_A2V_PARAMS,
                                                                             SHOOT_PID_BULLET_LOADER_A2V_PARAMS,
                                                                             SHOOT_PID_BULLET_LOADER_A2V_PARAMS};

PIDController::pid_params_t v2iPIDParameter[EngGrabMechBase::MOTOR_COUNT] = {{60, 2, 0, 12000, 12000},
                                                                             SHOOT_PID_BULLET_LOADER_V2I_PARAMS,
                                                                             SHOOT_PID_BULLET_LOADER_V2I_PARAMS};

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_set_pid(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 7) {
        shellUsage(chp, "set_pid MOTOR(0/1/2/3/4) is_a2v(0/1) kp ki kd i_limit out_limit");
        return;
    }
    PIDController::pid_params_t NEW_Parameter = {Shell::atof(argv[2]), Shell::atof(argv[3]),
                                                 Shell::atof(argv[4]),Shell::atof(argv[5]),
                                                 Shell::atof(argv[6])};
    int motor_id = Shell::atoi(argv[0]);
    int is_a2v = Shell::atoi(argv[1]);
    if (motor_id < 0 || motor_id >= EngGrabMechBase::MOTOR_COUNT) {
        shellUsage(chp, "invalid id");
        return;
    }
    PIDController::pid_params_t *p = is_a2v ? a2vPIDParameter : v2iPIDParameter;
    p[motor_id] = NEW_Parameter;
    if (is_a2v) {
        EngineerGrabSKD::load_a2v_pid_params(p);
    } else {
        EngineerGrabSKD::load_v2i_pid_params(p);
    }
    chprintf(chp, "pid_set!" SHELL_NEWLINE_STR);
}

static void cmd_echo_fb(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_fb");
        return;
    }
    Shell::printf("GrabberL :    %f" SHELL_NEWLINE_STR, EngGrabMechIF::feedback[EngGrabMechIF::GRABER_L]->accumulated_angle());
    Shell::printf("GrabberR :    %f" SHELL_NEWLINE_STR, EngGrabMechIF::feedback[EngGrabMechIF::GRABER_R]->accumulated_angle());
    Shell::printf("HAND     :    %f" SHELL_NEWLINE_STR, EngGrabMechIF::feedback[EngGrabMechIF::ROTATION_HAND]->accumulated_angle());
    Shell::printf("STATUS   :    %d" SHELL_NEWLINE_STR, EngineerGrabSKD::echo_status());
}

static void cmd_rise(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "rise");
        return;
    }
    EngineerGrabSKD::invoke_rising();
}

static void cmd_lower(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "lower");
        return;
    }
    EngineerGrabSKD::invoke_lowering();
}

static void cmd_relaxing(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if(argc != 0) {
        shellUsage(chp, "relax");
        return;
    }
    EngineerGrabSKD::invoke_relaxing();
}

static void cmd_echo_pid_param(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_pid_param");
        return;
    }
    for (int i = 0; i < EngGrabMechBase::MOTOR_COUNT; i++) {
        Shell::printf("a2v: kp %f ki %f kd %f i_limit %f out_limit %f" SHELL_NEWLINE_STR,
                      a2vPIDParameter[i].kp, a2vPIDParameter[i].ki, a2vPIDParameter[i].kd, a2vPIDParameter[i].i_limit, a2vPIDParameter[i].out_limit);
    }
    for (int i = 0; i < EngGrabMechBase::MOTOR_COUNT; i++) {
        Shell::printf("v2i: kp %f ki %f kd %f i_limit %f out_limit %f" SHELL_NEWLINE_STR,
                      v2iPIDParameter[i].kp, v2iPIDParameter[i].ki, v2iPIDParameter[i].kd, v2iPIDParameter[i].i_limit, v2iPIDParameter[i].out_limit);
    }
}

static void cmd_echo_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "echo_angle");
        return;
    }
    for (int i = 0; i < EngGrabMechBase::MOTOR_COUNT; i++) {
        Shell::printf("%f" SHELL_NEWLINE_STR, EngGrabMechIF::feedback[i]->accumulated_angle(), EngGrabMechIF::feedback[i]->accumulated_angle());
    }
}

static void cmd_hold(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "hold");
        return;
    }
    EngineerGrabSKD::hold();
}

static void cmd_release(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "release");
        return;
    }
    EngineerGrabSKD::release();
}

// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"set_pid", cmd_set_pid},
        {"echo_fb", cmd_echo_fb},
        {"rise",    cmd_rise},
        {"lower",   cmd_lower},
        {"relax",   cmd_relaxing},
        {"echo_pid_param", cmd_echo_pid_param},
        {"echo_angle", cmd_echo_angle},
        {"hold", cmd_hold},
        {"release", cmd_release},
        {nullptr,    nullptr}
};


// Thread to ...
class ControlThread : public BaseStaticThread <512> {
private:
    void main() final {
        setName("pa_enginner_grab");
        while (!shouldTerminate()) {
            EngineerGrabSKD::set_belt_target_velocity(Remote::rc.ch1 * 1000.0f);
            if(Remote::rc.s1 == Remote::S_MIDDLE) {
                EngineerGrabSKD::invoke_rising();
            } else if(Remote::rc.s1 == Remote::S_DOWN) {
                EngineerGrabSKD::invoke_lowering();
            } else if(Remote::rc.s1 == Remote::S_UP) {
                EngineerGrabSKD::invoke_relaxing();
            }
            if(Remote::rc.s2 == Remote::S_UP) {
                EngineerGrabSKD::hold();
            } else if (Remote::rc.s2 == Remote::S_DOWN) {
                EngineerGrabSKD::release();
            }
            sleep(TIME_MS2I(100));
        }
    }
} controlThread;


int main(void) {

    halInit();
    System::init();
    Remote::start();
    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    can1.start(NORMALPRIO+2,NORMALPRIO+3);
    can2.start(NORMALPRIO+4, NORMALPRIO+5);
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);
    EngGrabMechIF::motor_can_config_t CANCONFIG[5] = {  {EngGrabMechIF::can_channel_1, 2, CANInterface::M3508},
                                                        {EngGrabMechIF::can_channel_1, 3, CANInterface::M2006},
                                                        {EngGrabMechIF::can_channel_1, 4, CANInterface::M2006}};
    EngGrabMechIF::init(&can1, &can2, CANCONFIG);
    controlThread.start(NORMALPRIO + 1);
    EngineerGrabSKD::install_direction direct[5] = {EngineerGrabSKD::install_direction::POSITIVE,
                                                    EngineerGrabSKD::install_direction::NEGATIVE,
                                                    EngineerGrabSKD::install_direction::POSITIVE};
//    PIDController::pid_params_t pidParams[4] = {SHOOT_PID_BULLET_LOADER_V2I_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS,
//                                                SHOOT_PID_BULLET_LOADER_V2I_PARAMS, SHOOT_PID_BULLET_LOADER_V2I_PARAMS};
//    PIDController::pid_params_t pidParams1[2] = {SHOOT_PID_BULLET_LOADER_A2V_PARAMS, SHOOT_PID_BULLET_LOADER_A2V_PARAMS};
    EngineerGrabSKD::start(NORMALPRIO + 1, direct);
    EngineerGrabSKD::load_v2i_pid_params(v2iPIDParameter);
    EngineerGrabSKD::load_a2v_pid_params(a2vPIDParameter);


#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}
