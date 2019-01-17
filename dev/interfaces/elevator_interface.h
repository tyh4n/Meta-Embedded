//
// Created by Zhu Kerui on 2019/1/16.
//

#ifndef META_INFANTRY_RMDS108_INTERFACE_H
#define META_INFANTRY_RMDS108_INTERFACE_H

#include "ch.hpp"
#include "hal.h"
#include "can_interface.h"

class ElevatorInterface {
public:

    /**
     * @brief the SID of the four CANTxFrames for the four wheels
     */
    typedef enum {
        RX_FRONT_LEFT = 0x30B,  // The front-left wheel
        RX_FRONT_RIGHT = 0x31B,  // The front-right wheel
        RX_REAR_LEFT = 0x32B,  // The left-rear wheel
        RX_REAR_RIGHT = 0x33B  // The right-rear wheel
    } elevator_wheel_rx_id_t;

    /**
     * @brief the SID for command 5 (set position command) of the four CANRxFrames for the four wheels
     */
    typedef enum {
        COMMAND_5_FRONT_LEFT = 0x305,  // The front-left wheel
        COMMAND_5_FRONT_RIGHT = 0x315,  // The front-right wheel
        COMMAND_5_REAR_LEFT = 0x325,  // The left-rear wheel
        COMMAND_5_REAR_RIGHT = 0x335  // The right-rear wheel
    } elevator_wheel_tx_command_5_id_t;

    /**
     * @brief the SID for command 0 (reset command) of the four CANRxFrames for the four wheels
     */
    typedef enum {
        COMMAND_0_FRONT_LEFT = 0x300,  // The front-left wheel
        COMMAND_0_FRONT_RIGHT = 0x310,  // The front-right wheel
        COMMAND_0_REAR_LEFT = 0x320,  // The left-rear wheel
        COMMAND_0_REAR_RIGHT = 0x330  // The right-rear wheel
    } elevator_wheel_tx_command_0_id_t;

    /**
     * @brief the SID for command 1 (choose mode command) of the four CANRxFrames for the four wheels
     */
    typedef enum {
        COMMAND_1_FRONT_LEFT = 0x301,  // The front-left wheel
        COMMAND_1_FRONT_RIGHT = 0x311,  // The front-right wheel
        COMMAND_1_REAR_LEFT = 0x321,  // The left-rear wheel
        COMMAND_1_REAR_RIGHT = 0x331  // The right-rear wheel
    } elevator_wheel_tx_command_1_id_t;

    static bool enabled;  // if not enabled, 0 current will be sent in send_elevator_currents

    /**
     * @brief contains the target position of both the front wheels and the rear wheels
     * target_position[0] contains the target position of the front wheels
     * target_position[1] contains the target position of the rear wheels
     */
    static int32_t target_position[2];

    /**
     * @brief set the target position for both the front wheels and the rear wheels
     * @param front_wheel_position      the target position of the front wheels
     * @param rear_wheel_position       the target position of the rear wheels
     */
    static void set_position(int32_t front_wheel_position, int32_t rear_wheel_position);

    /**
     * @brief contains the command CANTxFrames for the four wheels
     * txFrames[0] and txFrames[1] are for the front_left and front_right wheels accordingly
     * txFrames[2] and txFrames[3] are for the left_rear and right_rear wheels accordingly
     */
    static CANTxFrame txFrames[4];

    /** Motor Interface **/
    typedef struct {

        /**
         * Basic Parameters
         */
        int32_t real_position; // the real position of the motor
        int16_t real_current;  // the real current in the motor
        int16_t real_velocity;  // the real velocity of the motor

    } elevator_wheel_t;

    static constexpr uint16_t PWM = 5000;  // the pwm of the current

    /**
     * @brief contains the real information of the four wheels
     * elevator_wheels[0] and elevator_wheels[1] are for the front_left and front_right wheels accordingly
     * elevator_wheels[2] and elevator_wheels[3] are for the left_rear and right_rear wheels accordingly
     */
    static elevator_wheel_t elevator_wheels[4];

    /**
     * @brief Get the feedback (real position, real velocity, real current) of each motor from the driver
     * @return true if success, false otherwise
     */
    static bool get_feedback(CANRxFrame *rxmsg);


    /**
     * @brief send message of each motor
     * @return true if success, false otherwise
     */
    bool send_message();

    /**
     * @brief set the CAN interface
     * @param can_interface
     */
    static void start(CANInterface* can_interface);

private:

    static CANInterface* can;

};

#endif //META_INFANTRY_RMDS108_INTERFACE_H
