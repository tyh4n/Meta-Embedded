//
// Modified by Gemini for PID Tuning
// ut_pid_tuning.cpp
//

#include "ch.hpp"
#include "hal.h"

#include <cstdlib>
#include <cstring>
#include "chprintf.h"

#include "interface/led/led.h"
#include "shell.h"
#include "can_motor_interface.h"
#include "can_motor_controller.h"
#include "hardware_conf.h"
#include "can_motor_config.h"

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

// --- Tuning State Variables ---
enum TuningMode { MODE_V2I, MODE_A2V };
enum SignalProfile { SIG_CONST, SIG_STEP };

TuningMode current_mode = MODE_V2I;
SignalProfile current_profile = SIG_CONST;

float base_target = 0.0f;
bool enable_telemetry = false;
CANMotorCFG::motor_id_t tuning_motor_id = CANMotorCFG::MOTOR1; // Default motor to tune

// --- Telemetry & Control Thread ---
class PIDTuningThread : public BaseStaticThread<1024> {
private:
    void main() final {
        setName("pid_tuning");
        systime_t last_step_time = chVTGetSystemTime();
        bool step_high = true;

        while (!shouldTerminate()) {
            float current_target = 0.0f;

            // 1. Generate Signal Profile
            if (current_profile == SIG_CONST) {
                current_target = base_target;
            } else if (current_profile == SIG_STEP) {
                // Toggle target every 500ms (1Hz square wave)
                if (chVTTimeElapsedSinceX(last_step_time) > TIME_MS2I(500)) {
                    step_high = !step_high;
                    last_step_time = chVTGetSystemTimeX();
                }
                // Changed from 0.0f to -base_target to create back-and-forth motion!
                current_target = step_high ? base_target : -base_target;
            }

            // 2. Apply Control and Fetch Actual Values
            float actual_val = 0.0f;

            if (current_mode == MODE_V2I) {
                CANMotorController::set_target_vel(tuning_motor_id, current_target);
                actual_val = CANMotorIF::motor_feedback[tuning_motor_id].actual_velocity;
            } else {
                CANMotorController::set_target_angle(tuning_motor_id, current_target);
                actual_val = CANMotorIF::motor_feedback[tuning_motor_id].actual_angle;
            }

            // 3. Output Telemetry (Formatted for Teleplot extension)
            if (enable_telemetry) {
                Shell::printf(">target:%.2f\n>actual:%.2f\n", current_target, actual_val);
            }

            // Run loop at 100Hz
            sleep(TIME_MS2I(10));
        }
    }
} tuningThread;

// --- Shell Commands ---
static bool cmd_tune(BaseSequentialStream *chp, int argc, char *argv[]) {
    if (argc < 2) {
        chprintf(chp, "Usage: tune <cmd> [args...]\r\n");
        chprintf(chp, "  tune mode <v2i|a2v>\r\n");
        chprintf(chp, "  tune sig <const|step> <value>\r\n");
        chprintf(chp, "  tune pid <v|a> <p|i|d> <value>\r\n");
        chprintf(chp, "  tune print <on|off>\r\n");
        return true;
    }

    if (strcmp(argv[0], "status") == 0) {
        auto v_pid = CANMotorCFG::v2iParams[tuning_motor_id];
        auto a_pid = CANMotorCFG::a2vParams[tuning_motor_id];

        chprintf(chp, "\r\n=== STM32 Connected & Ready ===\r\n");
        chprintf(chp, "Active Mode: %s\r\n", current_mode == MODE_V2I ? "V2I (Velocity)" : "A2V (Angle)");
        chprintf(chp, "V2I PID -> P: %.4f | I: %.4f | D: %.4f\r\n", v_pid.kp, v_pid.ki, v_pid.kd);
        chprintf(chp, "A2V PID -> P: %.4f | I: %.4f | D: %.4f\r\n", a_pid.kp, a_pid.ki, a_pid.kd);
        chprintf(chp, "===============================\r\n\n");
        return false;
    }

    if (strcmp(argv[0], "mode") == 0) {
        if (strcmp(argv[1], "v2i") == 0) {
            current_mode = MODE_V2I;
            CANMotorCFG::enable_a2v[tuning_motor_id] = false;
            CANMotorCFG::enable_v2i[tuning_motor_id] = true;
            chprintf(chp, "Mode set to V2I (Velocity -> Current)\r\n");
        } else if (strcmp(argv[1], "a2v") == 0) {
            current_mode = MODE_A2V;
            CANMotorCFG::enable_a2v[tuning_motor_id] = true;
            CANMotorCFG::enable_v2i[tuning_motor_id] = true;
            chprintf(chp, "Mode set to A2V (Angle -> Velocity -> Current)\r\n");
        }
    }
    else if (strcmp(argv[0], "sig") == 0 && argc == 3) {
        if (strcmp(argv[1], "const") == 0) current_profile = SIG_CONST;
        else if (strcmp(argv[1], "step") == 0) current_profile = SIG_STEP;

        base_target = atof(argv[2]);
        chprintf(chp, "Signal set to %s with magnitude %.2f\r\n", argv[1], base_target);
    }
    else if (strcmp(argv[0], "pid") == 0 && argc == 4) {
        float val = atof(argv[3]);
        PIDController::pid_params_t* params = nullptr;

        if (strcmp(argv[1], "v") == 0) params = &CANMotorCFG::v2iParams[tuning_motor_id];
        else if (strcmp(argv[1], "a") == 0) params = &CANMotorCFG::a2vParams[tuning_motor_id];

        if (params != nullptr) {
            if (strcmp(argv[2], "p") == 0) params->kp = val;
            else if (strcmp(argv[2], "i") == 0) params->ki = val;
            else if (strcmp(argv[2], "d") == 0) params->kd = val;

            chprintf(chp, "PID %s loop updated: %s = %.4f\r\n", argv[1], argv[2], val);
        }
    }
    else if (strcmp(argv[0], "print") == 0) {
        enable_telemetry = (strcmp(argv[1], "on") == 0);
        chprintf(chp, "Telemetry %s\r\n", enable_telemetry ? "enabled" : "disabled");
    }

    return false;
}

// Change your array to this:
static const Shell::Command tuning_commands[] = {
    {"tune", "PID Tuning utility", cmd_tune, nullptr},
    {nullptr, nullptr, nullptr, nullptr}
};

// --- Main ---
int main(void) {
    halInit();
    System::init();
    LED::all_off();

    // Start CAN & Motor Controller
    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO + 1);
    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);

    // Start Shell and inject our custom tuning commands
    Shell::start(HIGHPRIO);
    Shell::addCommands(tuning_commands);

    // Start Tuning Generator
    tuningThread.start(NORMALPRIO + 4);

    LED::green_on();

#if CH_CFG_NO_IDLE_THREAD
    while (true) {}
#else
    chibios_rt::BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}