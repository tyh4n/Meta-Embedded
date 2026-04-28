//
// Modified for 5-Stage Step Response, Mutex, and Auto-PID Fetch
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

// --- Global Mutex for UART ---
mutex_t uart_mtx;

// --- Tuning State Variables ---
enum TuningMode { MODE_V2I, MODE_A2V };
enum TuningState { STATE_IDLE, STATE_PRE_ZERO, STATE_STEP_POS, STATE_STEP_NEG, STATE_POST_ZERO };

TuningMode current_mode = MODE_V2I;
TuningState current_state = STATE_IDLE;

float base_target = 0.0f;
uint32_t step_duration_ms = 500;
volatile bool trigger_new_step = false;

CANMotorCFG::motor_id_t tuning_motor_id = CANMotorCFG::MOTOR1;

// --- Telemetry & Control Thread ---
class PIDTuningThread : public BaseStaticThread<1024> {
private:
    void main() final {
        setName("pid_tuning");
        systime_t state_start_time = chVTGetSystemTime();

        while (!shouldTerminate()) {
            float current_target = 0.0f;
            bool send_telemetry = false;

            if (trigger_new_step) {
                trigger_new_step = false;
                current_state = STATE_PRE_ZERO;
                state_start_time = chVTGetSystemTimeX();
            }

            // --- 5-Stage State Machine ---
            if (current_state == STATE_PRE_ZERO) {
                current_target = 0.0f;
                send_telemetry = true;
                if (chVTTimeElapsedSinceX(state_start_time) > TIME_MS2I(2000)) { // 2s wait
                    current_state = STATE_STEP_POS;
                    state_start_time = chVTGetSystemTimeX();
                }
            }
            else if (current_state == STATE_STEP_POS) {
                current_target = base_target;
                send_telemetry = true;
                if (chVTTimeElapsedSinceX(state_start_time) > TIME_MS2I(step_duration_ms)) {
                    current_state = STATE_STEP_NEG;
                    state_start_time = chVTGetSystemTimeX();
                }
            }
            else if (current_state == STATE_STEP_NEG) {
                current_target = -base_target;
                send_telemetry = true;
                if (chVTTimeElapsedSinceX(state_start_time) > TIME_MS2I(step_duration_ms)) {
                    current_state = STATE_POST_ZERO;
                    state_start_time = chVTGetSystemTimeX();
                }
            }
            else if (current_state == STATE_POST_ZERO) {
                current_target = 0.0f;
                send_telemetry = true;
                if (chVTTimeElapsedSinceX(state_start_time) > TIME_MS2I(2000)) { // 2s wait
                    current_state = STATE_IDLE;
                }
            }
            else { // STATE_IDLE
                current_target = 0.0f;
                send_telemetry = false;
            }

            // --- Apply Control ---
            if (current_mode == MODE_V2I) {
                CANMotorController::set_target_vel(tuning_motor_id, current_target);
            } else {
                CANMotorController::set_target_angle(tuning_motor_id, current_target);
            }

            // Grab ALL data natively from the controller and feedback to ensure accuracy
            float t_a = CANMotorController::get_target_angle(tuning_motor_id);
            // Use accumulate_angle() so the plot doesn't jump wildly if it crosses 180 degrees
            float a_a = CANMotorIF::motor_feedback[tuning_motor_id].accumulate_angle();
            float t_v = CANMotorController::get_target_vel(tuning_motor_id);
            float a_v = CANMotorIF::motor_feedback[tuning_motor_id].actual_velocity;

            // --- Output Telemetry (Protected by Mutex) ---
            if (send_telemetry) {
                chMtxLock(&uart_mtx);
                Shell::printf(">t_a:%.2f\n>a_a:%.2f\n>t_v:%.2f\n>a_v:%.2f\n", t_a, a_a, t_v, a_v);
                chMtxUnlock(&uart_mtx);
            }

            chThdSleepMilliseconds(10);
        }
    }
} tuningThread;

// --- Shell Commands ---
static bool cmd_tune(BaseSequentialStream *chp, int argc, char *argv[]) {

    if (argc < 1) {
        chMtxLock(&uart_mtx);
        chprintf(chp, "Usage: tune mode|step|pid|get\r\n");
        chMtxUnlock(&uart_mtx);
        return false;
    }

    if (strcmp(argv[0], "mode") == 0 && argc >= 2) {
        if (strcmp(argv[1], "v2i") == 0) {
            current_mode = MODE_V2I;
            CANMotorCFG::enable_a2v[tuning_motor_id] = false;
            CANMotorCFG::enable_v2i[tuning_motor_id] = true;
            chMtxLock(&uart_mtx); chprintf(chp, "Mode set to V2I\r\n"); chMtxUnlock(&uart_mtx);
        } else if (strcmp(argv[1], "a2v") == 0) {
            current_mode = MODE_A2V;
            CANMotorCFG::enable_a2v[tuning_motor_id] = true;
            CANMotorCFG::enable_v2i[tuning_motor_id] = true;
            chMtxLock(&uart_mtx); chprintf(chp, "Mode set to A2V\r\n"); chMtxUnlock(&uart_mtx);
        }
    }
    else if (strcmp(argv[0], "step") == 0 && argc >= 3) {
        base_target = atof(argv[1]);
        step_duration_ms = atoi(argv[2]);
        trigger_new_step = true;
        chMtxLock(&uart_mtx);
        chprintf(chp, "Running step sequence: +/-%.2f for %d ms\r\n", base_target, step_duration_ms);
        chMtxUnlock(&uart_mtx);
    }
    else if (strcmp(argv[0], "pid") == 0 && argc >= 4) {
        float val = atof(argv[3]);
        PIDController::pid_params_t* params = nullptr;
        bool is_a2v = false; // We need to track which loop we are updating

        if (strcmp(argv[1], "v") == 0) {
            params = &CANMotorCFG::v2iParams[tuning_motor_id];
            is_a2v = false;
        }
        else if (strcmp(argv[1], "a") == 0) {
            params = &CANMotorCFG::a2vParams[tuning_motor_id];
            is_a2v = true;
        }

        if (params != nullptr) {
            if (strcmp(argv[2], "p") == 0) params->kp = val;
            else if (strcmp(argv[2], "i") == 0) params->ki = val;
            else if (strcmp(argv[2], "d") == 0) params->kd = val;

            CANMotorController::load_PID_params(tuning_motor_id, is_a2v, *params);

            chMtxLock(&uart_mtx);
            chprintf(chp, "PID %s loop updated: %s = %.4f\r\n", argv[1], argv[2], val);
            chMtxUnlock(&uart_mtx);
        }
    }
    else if (strcmp(argv[0], "get") == 0) {
        auto v_pid = CANMotorCFG::v2iParams[tuning_motor_id];
        auto a_pid = CANMotorCFG::a2vParams[tuning_motor_id];
        chMtxLock(&uart_mtx);
        // Format for easy Python parsing -> >pid:loop:p:i:d
        chprintf(chp, ">pid:v:%.4f:%.4f:%.4f\n", v_pid.kp, v_pid.ki, v_pid.kd);
        chprintf(chp, ">pid:a:%.4f:%.4f:%.4f\n", a_pid.kp, a_pid.ki, a_pid.kd);
        chMtxUnlock(&uart_mtx);
    }

    return true;
}

static const Shell::Command tuning_commands[] = {
    {"tune", "PID Tuning utility", cmd_tune, nullptr},
    {nullptr, nullptr, nullptr, nullptr}
};

int main(void) {
    halInit();
    System::init();
    LED::all_off();

    chMtxObjectInit(&uart_mtx);

    can1.start(NORMALPRIO);
    can2.start(NORMALPRIO + 1);
    CANMotorController::start(NORMALPRIO + 2, NORMALPRIO + 3, &can1, &can2);

    Shell::addCommands(tuning_commands);
    Shell::start(HIGHPRIO);

    tuningThread.start(NORMALPRIO + 4);
    LED::green_on();

#if CH_CFG_NO_IDLE_THREAD
    while (true) {}
#else
    chibios_rt::BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}