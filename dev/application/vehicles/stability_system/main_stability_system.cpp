//
// Created by Tianyi Han on 3/18/2026.
// main_stability_system.cpp
//

#include "main_stability_system.h"

using namespace chibios_rt;

// Motor homing configuration table
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
volatile float target_r = 272.61f;
volatile float target_h = 50.0f;
volatile bool new_target_available = false;

// SPI2 Configuration (Slave Mode)
static const SPIConfig spi2_slave_cfg = {
    false,                  // circular buffer mode
    nullptr,                // end callback function pointer
    nullptr,                // chip select line port
    0,                      // chip select line pad number
    SPI_CR1_DFF,            // CR1: 16-bit data frame
    0                       // CR2
};

void forceSpiSlave() {
    SPID2.spi->CR1 &= ~SPI_CR1_SPE;
    SPID2.spi->CR1 &= ~SPI_CR1_MSTR;
    SPID2.spi->CR1 &= ~SPI_CR1_SSI;
    SPID2.spi->CR1 &= ~SPI_CR1_SSM;
    SPID2.spi->CR1 |= SPI_CR1_SPE;
}

// Homing Thread
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

        // Move to offset (Back off by 5mm)
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

        for (int i = 0; i < 2; i++) {
            CANMotorIF::motor_feedback[motors[i]].reset_accumulate_angle();
            CANMotorCFG::enable_limits[motors[i]] = false;
            CANMotorCFG::enable_a2v[motors[i]] = false;
        }

        is_homed = true;
    }
};

HomingThread* homingThreads[HOMING_GROUP_COUNT];

// Inverse Kinematics Logic
bool solve_linkage_IK(float r_leg, float h_leg, float &x1, float &x2) {
    float tx = r_leg - 213.0f;
    float ty = h_leg - 221.2f;

    const float r1 = 43.4f;
    const float r2 = 82.09f;
    const float L_ext = 122.8f;
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

// UART IK Control Thread
class IKControlThread : public BaseStaticThread<1024> {
protected:
    void main() final {
        setName("IKControl");
        while (!shouldTerminate()) {
            if (new_target_available) {
                new_target_available = false;

                float x1, x2;
                if (solve_linkage_IK(target_r, target_h, x1, x2)) {
                    float target_angle_m0 = (x1 + 115.0f) * 180.0f;
                    float target_angle_m1 = (x2 + 5.0f) * -180.0f;

                    for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
                        CANMotorCFG::enable_a2v[homing_list[i].m0] = true;
                        CANMotorCFG::enable_a2v[homing_list[i].m1] = true;
                        CANMotorController::set_target_angle(homing_list[i].m0, target_angle_m0);
                        CANMotorController::set_target_angle(homing_list[i].m1, target_angle_m1);
                    }
                }
            }
            sleep(TIME_MS2I(20));
        }
    }
} ikControlThread;

// Shell Command Handler for IK
bool cmd_set_ik(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)chp;
    if (argc != 2) return false;

    float r_cmd = std::atof(argv[0]);
    float h_cmd = std::atof(argv[1]);

    if (r_cmd < IK_MIN_R || r_cmd > IK_MAX_R || h_cmd < IK_MIN_H || h_cmd > IK_MAX_H) {
        Shell::printf("IK Error: Bounds (R: 272.61 to 385.14, H: 10.0 to 60.0)." SHELL_NEWLINE_STR);
        return true;
    }

    target_r = r_cmd;
    target_h = h_cmd;
    new_target_available = true;
    return true;
}

static const Shell::Command system_commands[] = {
    {"ik", "Set Inverse Kinematics target (tx ty)", cmd_set_ik, nullptr},
    {nullptr, nullptr, nullptr, nullptr}
};

// Remote Cartesian Control Thread
class RemoteControlThread : public BaseStaticThread<1024> {
private:
    float current_r = 272.61f;
    float current_h = 50.0f;

    static constexpr float MAX_SPEED_MM_S = 10.0f;
    static constexpr float LOOP_DT_S = 0.01f;

    void main() final {
        setName("RemoteControl");

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
                float next_r = current_r + (joy_x * MAX_SPEED_MM_S * LOOP_DT_S);
                float next_h = current_h + (joy_y * MAX_SPEED_MM_S * LOOP_DT_S);

                if (next_r < IK_MIN_R) next_r = IK_MIN_R;
                if (next_r > IK_MAX_R) next_r = IK_MAX_R;
                if (next_h < IK_MIN_H) next_h = IK_MIN_H;
                if (next_h > IK_MAX_H) next_h = IK_MAX_H;

                float x1, x2;
                if (solve_linkage_IK(next_r, next_h, x1, x2)) {
                    current_r = next_r;
                    current_h = next_h;

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

// SPI Cartesian Control Thread
class SPIControlThread : public BaseStaticThread<1024> {
    void main() final {
        setName("SPIControl");
        uint16_t rx_buffer[5];

        while(!shouldTerminate()) {
            spiReceive(&SPID2, 5, rx_buffer);

            if (rx_buffer[0] == 0xAAAA) {
                float_int target_r;
                float_int target_h;

                target_r.parts[0] = rx_buffer[1];
                target_r.parts[1] = rx_buffer[2];
                target_h.parts[0] = rx_buffer[3];
                target_h.parts[1] = rx_buffer[4];

                float tr = target_r.value;
                float th = target_h.value;

                // Enforce physical bounding box limits
                if (tr < IK_MIN_R) tr = IK_MIN_R;
                if (tr > IK_MAX_R) tr = IK_MAX_R;
                if (th < IK_MIN_H) th = IK_MIN_H;
                if (th > IK_MAX_H) th = IK_MAX_H;

                float x1, x2;
                if (solve_linkage_IK(tr, th, x1, x2)) {
                    float target_angle_m0 = (x1 + 115.0f) * 180.0f;
                    float target_angle_m1 = (x2 + 5.0f) * -180.0f;

                    for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
                        CANMotorCFG::enable_a2v[homing_list[i].m0] = true;
                        CANMotorCFG::enable_a2v[homing_list[i].m1] = true;

                        CANMotorController::set_target_angle(homing_list[i].m0, target_angle_m0);
                        CANMotorController::set_target_angle(homing_list[i].m1, target_angle_m1);
                    }
                }
            } else {
                Shell::printf("SPI Sync Lost! RX[0]=0x%04X" SHELL_NEWLINE_STR, rx_buffer[0]);
                spiStop(&SPID2);
                spiStart(&SPID2, &spi2_slave_cfg);
                forceSpiSlave();
            }

            sleep(TIME_MS2I(5));
        }
    }
} spiControlThread;

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

    // Remote::start();

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

    // Start SPI Driver
    spiStart(&SPID2, &spi2_slave_cfg);
    forceSpiSlave();

    LED::red_off();
    LED::green_on();

    // Start Threads. Be cautious if running multiple threads commanding the same motors!
    // remoteControlThread.start(NORMALPRIO + 4);
    spiControlThread.start(NORMALPRIO + 5);
    // remoteWatchdogThread.start(NORMALPRIO - 1);

#if CH_CFG_NO_IDLE_THREAD
    while (true) {}
#else
    BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}