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
volatile bool stability_homed = false;
volatile bool homing_request_pending = false;
volatile bool homing_active = false;
volatile float spi_target_r[SPI_LEG_COUNT] = {272.61f, 272.61f, 272.61f, 272.61f};
volatile float spi_target_h[SPI_LEG_COUNT] = {50.0f, 50.0f, 50.0f, 50.0f};

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

bool spiSlaveConfigured() {
    const uint32_t cr1 = SPID2.spi->CR1;
    const uint32_t mismatch = SPI_CR1_MSTR | SPI_CR1_SSM |
                              SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_LSBFIRST;
    return (cr1 & mismatch) == 0U &&
           (cr1 & SPI_CR1_SPE) != 0U &&
           (cr1 & SPI_CR1_DFF) != 0U;
}

bool ensureSpiSlave() {
    if (spiSlaveConfigured()) {
        return true;
    }
    forceSpiSlave();
    return spiSlaveConfigured();
}

uint16_t spi_crc16_words(const uint16_t *words, size_t count) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < count; i++) {
        crc ^= words[i];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1U) ^ 0xA001U)
                             : static_cast<uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

// Homing Thread
class HomingThread : public chibios_rt::BaseStaticThread<512> {
public:
    volatile bool is_homed = false;

private:
    HomingConfig cfg;

public:
    HomingThread(HomingConfig config) : cfg(config) {}
    virtual ~HomingThread() = default;

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

static bool runHomingCycle() {
    HomingThread *threads[HOMING_GROUP_COUNT] = {};
    ThreadReference references[HOMING_GROUP_COUNT];
    bool all_started = true;

    for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
        references[i] = ThreadReference(nullptr);
        threads[i] = new HomingThread(homing_list[i]);
        if (threads[i] == nullptr) {
            all_started = false;
            continue;
        }
        references[i] = threads[i]->start(NORMALPRIO + 4);
        if (references[i].isNull()) {
            all_started = false;
        }
    }

    for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
        if (!references[i].isNull()) {
            references[i].wait();
        }
    }

    for (size_t i = 0; i < HOMING_GROUP_COUNT; i++) {
        delete threads[i];
    }
    return all_started;
}

static void requestHoming() {
    chSysLock();
    if (!homing_active) {
        stability_homed = false;
        homing_active = true;
        homing_request_pending = true;
    }
    chSysUnlock();
}

class HomingManagerThread : public BaseStaticThread<512> {
private:
    void main() final {
        setName("HomeManager");
        while (!shouldTerminate()) {
            if (homing_request_pending) {
                chSysLock();
                homing_request_pending = false;
                chSysUnlock();
                LED::green_off();
                LED::red_on();

                const bool homed = runHomingCycle();
                chSysLock();
                stability_homed = homed;
                homing_active = false;
                chSysUnlock();

                LED::red_off();
                if (homed) {
                    LED::green_on();
                } else {
                    LED::red_on();
                }
            }
            sleep(TIME_MS2I(10));
        }
    }
} homingManagerThread;

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

bool solve_linkage_FK(float x1, float x2, float &r_leg, float &h_leg) {
    const float r1 = 43.4f;
    const float r2 = 82.09f;
    const float L_ext = 122.8f;
    const float L_perp = 11.65f;
    const float A1_x = 48.0f;
    const float arm_long = r1 + L_ext;
    const float target_radius =
            std::sqrt((arm_long * arm_long) + (L_perp * L_perp));
    const float phi_offset = std::atan2(L_perp, arm_long);

    const float dx = A1_x;
    const float dy = x2 - x1;
    const float d_sq = (dx * dx) + (dy * dy);
    if (d_sq < 1.0e-6f) {
        return false;
    }

    const float d = std::sqrt(d_sq);
    const float a = ((r2 * r2) - (r1 * r1) + d_sq) / (2.0f * d);
    const float h_sq = (r2 * r2) - (a * a);
    if (h_sq < 0.0f) {
        return false;
    }

    const float h = std::sqrt(h_sq);
    const float p_x = (a * dx) / d;
    const float p_y = x1 + ((a * dy) / d);
    const float ax = p_x + ((h * dy) / d);
    const float ay = p_y - ((h * dx) / d);
    if (ax <= 0.0f) {
        return false;
    }

    const float theta_A1A2 = std::atan2(ay - x2, ax - A1_x);
    const float theta_A1A3 = theta_A1A2 + phi_offset;
    const float tx = A1_x + (target_radius * std::cos(theta_A1A3));
    const float ty = x2 + (target_radius * std::sin(theta_A1A3));
    if (!std::isfinite(tx) || !std::isfinite(ty)) {
        return false;
    }

    r_leg = tx + 213.0f;
    h_leg = ty + 221.2f;
    return true;
}

static bool read_current_position(size_t leg, float &r, float &h) {
    if (!stability_homed || leg >= HOMING_GROUP_COUNT ||
        leg >= SPI_LEG_COUNT) {
        return false;
    }
    const float angle0 =
            CANMotorIF::motor_feedback[homing_list[leg].m0].accumulate_angle();
    const float angle1 =
            CANMotorIF::motor_feedback[homing_list[leg].m1].accumulate_angle();
    return solve_linkage_FK((angle0 / 180.0f) - 115.0f,
                            (-angle1 / 180.0f) - 5.0f, r, h);
}

static bool apply_cartesian_target(size_t leg, float r, float h) {
    constexpr float WIRE_ROUNDING_TOLERANCE = 0.051f;
    if (leg >= HOMING_GROUP_COUNT || leg >= SPI_LEG_COUNT ||
        !std::isfinite(r) || !std::isfinite(h) ||
        r < (IK_MIN_R - WIRE_ROUNDING_TOLERANCE) ||
        r > (IK_MAX_R + WIRE_ROUNDING_TOLERANCE) ||
        h < IK_MIN_H || h > IK_MAX_H) {
        return false;
    }

    float x1, x2;
    if (!solve_linkage_IK(r, h, x1, x2)) {
        return false;
    }
    const float target_angle_m0 = (x1 + 115.0f) * 180.0f;
    const float target_angle_m1 = (x2 + 5.0f) * -180.0f;
    if (!std::isfinite(target_angle_m0) || !std::isfinite(target_angle_m1)) {
        return false;
    }

    CANMotorCFG::enable_a2v[homing_list[leg].m0] = true;
    CANMotorCFG::enable_a2v[homing_list[leg].m1] = true;
    CANMotorController::set_target_angle(homing_list[leg].m0, target_angle_m0);
    CANMotorController::set_target_angle(homing_list[leg].m1, target_angle_m1);
    spi_target_r[leg] = r;
    spi_target_h[leg] = h;
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
private:
    static uint16_t position_to_wire(float value) {
        return static_cast<uint16_t>(std::lround(value * 10.0f));
    }

    static size_t r_word(size_t leg) {
        return SPI_FIRST_POSITION_WORD + (2 * leg);
    }

    static void finalize_frame(uint16_t *frame) {
        frame[0] = SPI_FRAME_MAGIC;
        frame[SPI_FRAME_WORDS - 1] =
                spi_crc16_words(frame, SPI_FRAME_WORDS - 1);
    }

    static bool valid_frame(const uint16_t *frame) {
        return frame[0] == SPI_FRAME_MAGIC &&
               frame[SPI_FRAME_WORDS - 1] ==
                       spi_crc16_words(frame, SPI_FRAME_WORDS - 1);
    }

    void main() final {
        setName("SPIControl");
        uint16_t rx_frame[SPI_FRAME_WORDS] = {};
        uint16_t tx_frame[SPI_FRAME_WORDS] = {};
        uint16_t reply_seq = 0;
        uint16_t reply_command = SPI_CMD_NONE;
        SpiStatus reply_status = SPI_STATUS_UNHOMED;

        while (!shouldTerminate()) {
            if (!ensureSpiSlave()) {
                sleep(TIME_MS2I(20));
                continue;
            }

            tx_frame[0] = SPI_FRAME_MAGIC;
            tx_frame[1] = reply_seq;
            tx_frame[2] = static_cast<uint16_t>(
                    (reply_command << 8U) | static_cast<uint16_t>(reply_status));
            for (size_t leg = 0; leg < SPI_LEG_COUNT; leg++) {
                float current_r = spi_target_r[leg];
                float current_h = spi_target_h[leg];
                (void) read_current_position(leg, current_r, current_h);
                tx_frame[r_word(leg)] = position_to_wire(current_r);
                tx_frame[r_word(leg) + 1] = position_to_wire(current_h);
            }
            finalize_frame(tx_frame);

            spiExchange(&SPID2, SPI_FRAME_WORDS, tx_frame, rx_frame);
            if (!valid_frame(rx_frame)) {
                continue;
            }

            reply_seq = rx_frame[1];
            reply_command = (rx_frame[2] >> 8U) & 0xFFU;
            switch (reply_command) {
                case SPI_CMD_HOME:
                    requestHoming();
                    reply_status = SPI_STATUS_UNHOMED;
                    break;

                case SPI_CMD_QUERY:
                    reply_status = stability_homed
                                           ? SPI_STATUS_READY
                                           : SPI_STATUS_UNHOMED;
                    break;

                case SPI_CMD_SET_TARGET:
                    if (!stability_homed) {
                        reply_status = SPI_STATUS_UNHOMED;
                    } else {
                        bool all_valid = true;
                        for (size_t leg = 0; leg < SPI_LEG_COUNT; leg++) {
                            const float r =
                                    static_cast<float>(rx_frame[r_word(leg)]) / 10.0f;
                            const float h =
                                    static_cast<float>(rx_frame[r_word(leg) + 1]) / 10.0f;
                            if (!apply_cartesian_target(leg, r, h)) {
                                all_valid = false;
                            }
                        }
                        reply_status = all_valid
                                               ? SPI_STATUS_READY
                                               : SPI_STATUS_INVALID;
                    }
                    break;

                default:
                    reply_status = SPI_STATUS_INVALID;
                    break;
            }
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

static void waitForHomingButton() {
    palSetPadMode(GPIOA, GPIOA_USER_BUTTON, PAL_MODE_INPUT_PULLUP);

    bool stable_pressed = false;
    uint8_t debounce_count = 0;

    while (true) {
        const bool raw_pressed =
                palReadPad(GPIOA, GPIOA_USER_BUTTON) == PAL_LOW;

        if (raw_pressed == stable_pressed) {
            debounce_count = 0;
        } else if (++debounce_count >= 3U) {
            stable_pressed = raw_pressed;
            debounce_count = 0;
            if (stable_pressed) {
                return;
            }
        }

        chThdSleepMilliseconds(10);
    }
}

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

    waitForHomingButton();

    stability_homed = runHomingCycle();

    // Start SPI Driver
    spiStart(&SPID2, &spi2_slave_cfg);
    forceSpiSlave();

    LED::red_off();
    if (stability_homed) {
        LED::green_on();
    } else {
        LED::red_on();
    }

    // Start Threads. Be cautious if running multiple threads commanding the same motors!
    // remoteControlThread.start(NORMALPRIO + 4);
    homingManagerThread.start(NORMALPRIO + 4);
    spiControlThread.start(NORMALPRIO + 5);
    // remoteWatchdogThread.start(NORMALPRIO - 1);

#if CH_CFG_NO_IDLE_THREAD
    while (true) {}
#else
    BaseThread::setPriority(IDLEPRIO);
#endif
    return 0;
}
