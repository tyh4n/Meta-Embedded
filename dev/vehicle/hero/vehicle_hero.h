//
// Created by liuzikai on 2019-04-22.
// Edited by Qian Chen & Mo Kanya & Jing Tenjun on 2019-07-05
// This file contains common parameters for hero
//

#ifndef META_INFANTRY_VEHICLE_INFANTRY_H
#define META_INFANTRY_VEHICLE_INFANTRY_H


/// AHRS Configurations
#define ON_BOARD_AHRS_MATRIX {{0.0f, -1.0f, 0.0f}, \
                              {1.0f, 0.0f, 0.0f}, \
                              {0.0f, 0.0f, 1.0f}}
// Raw angle of yaw and pitch when gimbal points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts

#define GIMBAL_YAW_FRONT_ANGLE_RAW 0
#define GIMBAL_PITCH_FRONT_ANGLE_RAW (-200)
#define GIMBAL_SUB_PITCH_FRONT_ANGLE_RAW 3569


#define LOADER_SHOOT_DEGREE_PER_BULLET 72.0f

#define MPU6500_BIAS_DATA_ID 0x0001
#define MPU6500_STORED_GYRO_BIAS {0.408107906, -0.399448364, 0.547539412}


/// Gimbal and Shoot Installation Configurations

#define GIMBAL_YAW_CAN_CHANNEL      (GimbalIF::can_channel_2)
#define GIMBAL_PITCH_CAN_CHANNEL    (GimbalIF::can_channel_1)
#define GIMBAL_SUB_PITCH_CAN_CHANNEL    (GimbalIF::can_channel_1)
#define GIMBAL_BULLET_CAN_CHANNEL   (GimbalIF::can_channel_2)
#define GIMBAL_FW_LEFT_CAN_CHANNEL  (GimbalIF::can_channel_1)
#define GIMBAL_FW_RIGHT_CAN_CHANNEL (GimbalIF::can_channel_1)

#define GIMBAL_YAW_CAN_ID         1
#define GIMBAL_PITCH_CAN_ID       1
#define GIMBAL_SUB_PITCH_CAN_ID   1
#define GIMBAL_BULLET_CAN_ID      6
#define GIMBAL_FW_LEFT_CAN_ID     4
#define GIMBAL_FW_RIGHT_CAN_ID    3

#define GIMBAL_YAW_MOTOR_TYPE      (CANInterface::GM6020)
#define GIMBAL_PITCH_MOTOR_TYPE    (CANInterface::GM6020)
#define GIMBAL_SUB_PITCH_MOTOR_TYPE     (CANInterface::M3508)
#define SHOOT_BULLET_MOTOR_TYPE    (CANInterface::M3508)
#define GIMBAL_FW_LEFT_MOTOR_TYPE  (CANInterface::M3508)
#define GIMBAL_FW_RIGHT_MOTOR_TYPE (CANInterface::M3508)

#define GIMBAL_YAW_MOTOR_DR         (CANInterface::DEFAULT_DECELERATION_RATIO)
#define GIMBAL_PITCH_MOTOR_DR       (CANInterface::DEFAULT_DECELERATION_RATIO)
#define GIMBAL_SUB_PITCH_MOTOR_DR   (CANInterface::DEFAULT_DECELERATION_RATIO)
#define SHOOT_BULLET_MOTOR_DR       (CANInterface::M3508_WITH_DECELERATION_RATIO)
#define GIMBAL_FW_LEFT_MOTOR_DR     (CANInterface::DEFAULT_DECELERATION_RATIO)
#define GIMBAL_FW_RIGHT_MOTOR_DR    (CANInterface::DEFAULT_DECELERATION_RATIO)

#define GIMBAL_MOTOR_CONFIG \
{ {GIMBAL_YAW_CAN_CHANNEL,          GIMBAL_YAW_CAN_ID,          GIMBAL_YAW_MOTOR_TYPE,          GIMBAL_YAW_MOTOR_DR}, \
  {GIMBAL_PITCH_CAN_CHANNEL,        GIMBAL_PITCH_CAN_ID,        GIMBAL_PITCH_MOTOR_TYPE,        GIMBAL_PITCH_MOTOR_DR}, \
  {GIMBAL_SUB_PITCH_CAN_CHANNEL,    GIMBAL_SUB_PITCH_CAN_ID,    GIMBAL_SUB_PITCH_MOTOR_TYPE,    GIMBAL_SUB_PITCH_MOTOR_DR}, \
  {GIMBAL_BULLET_CAN_CHANNEL,       GIMBAL_BULLET_CAN_ID,       SHOOT_BULLET_MOTOR_TYPE,        SHOOT_BULLET_MOTOR_DR}, \
  {GIMBAL_FW_LEFT_CAN_CHANNEL,      GIMBAL_FW_LEFT_CAN_ID,      GIMBAL_FW_LEFT_MOTOR_TYPE,      GIMBAL_FW_LEFT_MOTOR_DR}, \
  {GIMBAL_FW_RIGHT_CAN_CHANNEL,     GIMBAL_FW_RIGHT_CAN_ID,     GIMBAL_FW_RIGHT_MOTOR_TYPE,     GIMBAL_FW_RIGHT_MOTOR_DR} }

#define CHASSIS_FR_CHANNEL      (ChassisIF::can_channel_2)
#define CHASSIS_FL_CHANNEL      (ChassisIF::can_channel_2)
#define CHASSIS_BL_CHANNEL      (ChassisIF::can_channel_2)
#define CHASSIS_BR_CHANNEL      (ChassisIF::can_channel_2)

#define CHASSIS_FR_CAN_ID      1
#define CHASSIS_FL_CAN_ID      2
#define CHASSIS_BL_CAN_ID      3
#define CHASSIS_BR_CAN_ID      4

#define CHASSIS_FR_MOTOR_TYPE  (CANInterface::M3508)
#define CHASSIS_FL_MOTOR_TYPE  (CANInterface::M3508)
#define CHASSIS_BL_MOTOR_TYPE  (CANInterface::M3508)
#define CHASSIS_BR_MOTOR_TYPE  (CANInterface::M3508)

#define SHOOT_DEGREE_PER_BULLET 40.0f  // rotation degree of bullet loader for each bullet

#define CHASSIS_FR_MOTOR_DR  (CANInterface::M3508_WITH_DECELERATION_RATIO)
#define CHASSIS_FL_MOTOR_DR  (CANInterface::M3508_WITH_DECELERATION_RATIO)
#define CHASSIS_BL_MOTOR_DR  (CANInterface::M3508_WITH_DECELERATION_RATIO)
#define CHASSIS_BR_MOTOR_DR  (CANInterface::M3508_WITH_DECELERATION_RATIO)

#define CHASSIS_MOTOR_CONFIG \
{ {CHASSIS_FR_CHANNEL, CHASSIS_FR_CAN_ID, CHASSIS_FR_MOTOR_TYPE, CHASSIS_FR_MOTOR_DR}, \
  {CHASSIS_FL_CHANNEL, CHASSIS_FL_CAN_ID, CHASSIS_FL_MOTOR_TYPE, CHASSIS_FL_MOTOR_DR}, \
  {CHASSIS_BL_CHANNEL, CHASSIS_BL_CAN_ID, CHASSIS_BL_MOTOR_TYPE, CHASSIS_BL_MOTOR_DR}, \
  {CHASSIS_BR_CHANNEL, CHASSIS_BR_CAN_ID, CHASSIS_BR_MOTOR_TYPE, CHASSIS_BR_MOTOR_DR} }

#define GIMBAL_YAW_INSTALL_DIRECTION    (GimbalSKD::NEGATIVE)
#define GIMBAL_PITCH_INSTALL_DIRECTION  (GimbalSKD::POSITIVE)
#define GIMBAL_SUB_PITCH_INSTALL_DIRECTION (GimbalSKD::NEGATIVE)
#define SHOOT_BULLET_INSTALL_DIRECTION  (ShootSKD::NEGATIVE)

#define GIMBAL_ANGLE_INSTALLATION_MATRIX {{1.0f, 0.0f, 0.0f}, \
                                          {0.0f, 1.0f, 0.0f}, \
                                          {0.0f, 0.0f, -1.0f}}


#define GIMBAL_GYRO_INSTALLATION_MATRIX {{0.0f,  1.0f, 0.0f}, \
                                         {0.0f,  0.0f,  1.0f}, \
                                         {1.0f, 0.0f,  0.0f}}

/// Gimbal and Shoot PID Parameters
#define GIMBAL_PID_YAW_A2V_KP 20.0f
#define GIMBAL_PID_YAW_A2V_KI 0.0f
#define GIMBAL_PID_YAW_A2V_KD 2.0f
#define GIMBAL_PID_YAW_A2V_I_LIMIT 8192.0f
#define GIMBAL_PID_YAW_A2V_OUT_LIMIT 8192.0f
#define GIMBAL_PID_YAW_A2V_PARAMS \
    {GIMBAL_PID_YAW_A2V_KP, GIMBAL_PID_YAW_A2V_KI, GIMBAL_PID_YAW_A2V_KD, \
    GIMBAL_PID_YAW_A2V_I_LIMIT, GIMBAL_PID_YAW_A2V_OUT_LIMIT}

#define GIMBAL_PID_YAW_V2I_KP 150.00f
#define GIMBAL_PID_YAW_V2I_KI 16.00f
#define GIMBAL_PID_YAW_V2I_KD 20.00f
#define GIMBAL_PID_YAW_V2I_I_LIMIT 8192.00f
#define GIMBAL_PID_YAW_V2I_OUT_LIMIT 8192.00f
#define GIMBAL_PID_YAW_V2I_PARAMS \
    {GIMBAL_PID_YAW_V2I_KP, GIMBAL_PID_YAW_V2I_KI, GIMBAL_PID_YAW_V2I_KD, \
    GIMBAL_PID_YAW_V2I_I_LIMIT, GIMBAL_PID_YAW_V2I_OUT_LIMIT}

#define GIMBAL_PID_PITCH_A2V_KP 8.5f
#define GIMBAL_PID_PITCH_A2V_KI 0.0f
#define GIMBAL_PID_PITCH_A2V_KD 0.1f
#define GIMBAL_PID_PITCH_A2V_I_LIMIT 70.0f
#define GIMBAL_PID_PITCH_A2V_OUT_LIMIT 90.0f
#define GIMBAL_PID_PITCH_A2V_PARAMS \
    {GIMBAL_PID_PITCH_A2V_KP, GIMBAL_PID_PITCH_A2V_KI, GIMBAL_PID_PITCH_A2V_KD, \
    GIMBAL_PID_PITCH_A2V_I_LIMIT, GIMBAL_PID_PITCH_A2V_OUT_LIMIT}

#define GIMBAL_PID_PITCH_V2I_KP 300.00f
#define GIMBAL_PID_PITCH_V2I_KI 7.0f
#define GIMBAL_PID_PITCH_V2I_KD 0.0f
#define GIMBAL_PID_PITCH_V2I_I_LIMIT 8192.00f
#define GIMBAL_PID_PITCH_V2I_OUT_LIMIT 8192.00f
#define GIMBAL_PID_PITCH_V2I_PARAMS \
    {GIMBAL_PID_PITCH_V2I_KP, GIMBAL_PID_PITCH_V2I_KI, GIMBAL_PID_PITCH_V2I_KD, \
    GIMBAL_PID_PITCH_V2I_I_LIMIT, GIMBAL_PID_PITCH_V2I_OUT_LIMIT}

#define GIMBAL_RESTRICT_YAW_MIN_ANGLE -90
#define GIMBAL_RESTRICT_YAW_MAX_ANGLE 90
#define GIMBAL_RESTRICT_YAW_VELOCITY 20

#define SHOOT_PID_BULLET_LOADER_A2V_KP 8.5f
#define SHOOT_PID_BULLET_LOADER_A2V_KI 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_KD 0.18f
#define SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT 720.0f
#define SHOOT_PID_BULLET_LOADER_A2V_OUT_LIMIT 720.0f
#define SHOOT_PID_BULLET_LOADER_A2V_PARAMS \
    {SHOOT_PID_BULLET_LOADER_A2V_KP, SHOOT_PID_BULLET_LOADER_A2V_KI, SHOOT_PID_BULLET_LOADER_A2V_KD, \
    SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT, SHOOT_PID_BULLET_LOADER_A2V_OUT_LIMIT}

#define CALIBRATE_PID_BULLET_LOADER_A2V_OUT_LIMIT 60.0f
#define CALIBRATE_PID_BULLET_LOADER_A2V_PARAMS \
    {SHOOT_PID_BULLET_LOADER_A2V_KP, SHOOT_PID_BULLET_LOADER_A2V_KI, SHOOT_PID_BULLET_LOADER_A2V_KD, \
    SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT, CALIBRATE_PID_BULLET_LOADER_A2V_OUT_LIMIT}

#define SHOOT_PID_BULLET_LOADER_V2I_KP 35.0f
#define SHOOT_PID_BULLET_LOADER_V2I_KI 2.1f
#define SHOOT_PID_BULLET_LOADER_V2I_KD 0.0f
#define SHOOT_PID_BULLET_LOADER_V2I_I_LIMIT 3000.0f
#define SHOOT_PID_BULLET_LOADER_V2I_OUT_LIMIT 8192.0f
#define SHOOT_PID_BULLET_LOADER_V2I_PARAMS \
    {SHOOT_PID_BULLET_LOADER_V2I_KP, SHOOT_PID_BULLET_LOADER_V2I_KI, SHOOT_PID_BULLET_LOADER_V2I_KD, \
    SHOOT_PID_BULLET_LOADER_V2I_I_LIMIT, SHOOT_PID_BULLET_LOADER_V2I_OUT_LIMIT}

#define GIMBAL_PID_SUB_PITCH_A2V_KP 7.8f
#define GIMBAL_PID_SUB_PITCH_A2V_KI 0.0f
#define GIMBAL_PID_SUB_PITCH_A2V_KD 0.08f
#define GIMBAL_PID_SUB_PITCH_A2V_I_LIMIT 1000.0f
#define GIMBAL_PID_SUB_PITCH_A2V_OUT_LIMIT 3000.0f
#define GIMBAL_PID_SUB_PITCH_A2V_PARAMS \
    {GIMBAL_PID_SUB_PITCH_A2V_KP, GIMBAL_PID_SUB_PITCH_A2V_KI, GIMBAL_PID_SUB_PITCH_A2V_KD, \
    GIMBAL_PID_SUB_PITCH_A2V_I_LIMIT, GIMBAL_PID_SUB_PITCH_A2V_OUT_LIMIT}

#define GIMBAL_PID_SUB_PITCH_V2I_KP 24.0f
#define GIMBAL_PID_SUB_PITCH_V2I_KI 0.21f
#define GIMBAL_PID_SUB_PITCH_V2I_KD 0.0f
#define GIMBAL_PID_SUB_PITCH_V2I_I_LIMIT 10000.0f
#define GIMBAL_PID_SUB_PITCH_V2I_OUT_LIMIT 29000.0f
#define GIMBAL_PID_SUB_PITCH_V2I_PARAMS \
    {GIMBAL_PID_SUB_PITCH_V2I_KP, GIMBAL_PID_SUB_PITCH_V2I_KI, GIMBAL_PID_SUB_PITCH_V2I_KD, \
    GIMBAL_PID_SUB_PITCH_V2I_I_LIMIT, GIMBAL_PID_SUB_PITCH_V2I_OUT_LIMIT}

//TODO: Need to revised to the newest pid params.
#define SHOOT_PID_FW_LEFT_V2I_KP 35.0f
#define SHOOT_PID_FW_LEFT_V2I_KI 0.1f
#define SHOOT_PID_FW_LEFT_V2I_KD 0.02f
#define SHOOT_PID_FW_LEFT_V2I_I_LIMIT 4000.0f
#define SHOOT_PID_FW_LEFT_V2I_OUT_LIMIT 8192.0f
#define SHOOT_PID_FW_LEFT_V2I_PARAMS \
    {SHOOT_PID_FW_LEFT_V2I_KP, SHOOT_PID_FW_LEFT_V2I_KI, SHOOT_PID_FW_LEFT_V2I_KD, \
    SHOOT_PID_FW_LEFT_V2I_I_LIMIT, SHOOT_PID_FW_LEFT_V2I_OUT_LIMIT}

//TODO: Need to revised to the newest pid params.
#define SHOOT_PID_FW_RIGHT_V2I_KP 35.0f
#define SHOOT_PID_FW_RIGHT_V2I_KI 0.1f
#define SHOOT_PID_FW_RIGHT_V2I_KD 0.02f
#define SHOOT_PID_FW_RIGHT_V2I_I_LIMIT 4000.0f
#define SHOOT_PID_FW_RIGHT_V2I_OUT_LIMIT 8192.0f
#define SHOOT_PID_FW_RIGHT_V2I_PARAMS \
    {SHOOT_PID_FW_RIGHT_V2I_KP, SHOOT_PID_FW_RIGHT_V2I_KI, SHOOT_PID_FW_RIGHT_V2I_KD, \
    SHOOT_PID_FW_RIGHT_V2I_I_LIMIT, SHOOT_PID_FW_RIGHT_V2I_OUT_LIMIT}

/// Chassis Mechanism Parameters
#define CHASSIS_WHEEL_BASE  550.0f                // distance between front axle and the back axle [mm]
#define CHASSIS_WHEEL_TREAD 500.0f                    // distance between left and right wheels [mm]
#define CHASSIS_WHEEL_CIRCUMFERENCE 478.0f             // circumference of wheels [mm]
#define CHASSIS_GIMBAL_OFFSET 0.0f                    // distance between center of gimbal and the center of chassis

/// Chassis PID Parameters
#define CHASSIS_PID_V2I_KP 50.0f
#define CHASSIS_PID_V2I_KI 0.3f
#define CHASSIS_PID_V2I_KD 0.3f
#define CHASSIS_PID_V2I_I_LIMIT 3000.0f
#define CHASSIS_PID_V2I_OUT_LIMIT 6000.0f
#define CHASSIS_PID_V2I_PARAMS \
    {CHASSIS_PID_V2I_KP, CHASSIS_PID_V2I_KI, CHASSIS_PID_V2I_KD, \
    CHASSIS_PID_V2I_I_LIMIT, CHASSIS_PID_V2I_OUT_LIMIT}


#define CHASSIS_CLIP_PID_V2I_KP 50.0f
#define CHASSIS_CLIP_PID_V2I_KI 0.3f
#define CHASSIS_CLIP_PID_V2I_KD 0.3f
#define CHASSIS_CLIP_PID_V2I_I_LIMIT 1000.0f
#define CHASSIS_CLIP_PID_V2I_OUT_LIMIT 2400.0f
#define CHASSIS_CLIP_PID_V2I_PARAMS \
    {CHASSIS_CLIP_PID_V2I_KP, CHASSIS_CLIP_PID_V2I_KI, CHASSIS_CLIP_PID_V2I_KD, \
    CHASSIS_CLIP_PID_V2I_I_LIMIT, CHASSIS_CLIP_PID_V2I_OUT_LIMIT}

#define CHASSIS_CLIP_PID_THETA2V_KP 9.5f
#define CHASSIS_CLIP_PID_THETA2V_KI 0.015f
#define CHASSIS_CLIP_PID_THETA2V_KD 0.0f
#define CHASSIS_CLIP_PID_THETA2V_I_LIMIT 180.0f
#define CHASSIS_CLIP_PID_THETA2V_OUT_LIMIT 540.0f
#define CHASSIS_CLIP_PID_THETA2V_PARAMS \
    {CHASSIS_CLIP_PID_THETA2V_KP, CHASSIS_CLIP_PID_THETA2V_KI, CHASSIS_CLIP_PID_THETA2V_KD, \
    CHASSIS_CLIP_PID_THETA2V_I_LIMIT, CHASSIS_CLIP_PID_THETA2V_OUT_LIMIT}

#define CHASSIS_DODGE_PID_THETA2V_KP 7.0f                   //TODO:
#define CHASSIS_DODGE_PID_THETA2V_KI 0.0f
#define CHASSIS_DODGE_PID_THETA2V_KD 0.0f
#define CHASSIS_DODGE_PID_THETA2V_I_LIMIT 180.0f
#define CHASSIS_DODGE_PID_THETA2V_OUT_LIMIT 540.0f
#define CHASSIS_DODGE_PID_THETA2V_PARAMS \
    {CHASSIS_DODGE_PID_THETA2V_KP, CHASSIS_DODGE_PID_THETA2V_KI, CHASSIS_DODGE_PID_THETA2V_KD, \
    CHASSIS_DODGE_PID_THETA2V_I_LIMIT, CHASSIS_DODGE_PID_THETA2V_OUT_LIMIT}

#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KP 30.0f
#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KI 15.0f
#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KD 0.0f
#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_I_LIMIT 720.0f
#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_OUT_LIMIT 720.0f
#define CHASSIS_LOGIC_DODGE_OMEGA2VOLT_PARAMS \
    {CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KP, CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KI, CHASSIS_LOGIC_DODGE_OMEGA2VOLT_KD,\
     CHASSIS_LOGIC_DODGE_OMEGA2VOLT_I_LIMIT, CHASSIS_LOGIC_DODGE_OMEGA2VOLT_OUT_LIMIT}

#define CHASSIS_FOLLOW_PID_THETA2V_KP 11.0f
#define CHASSIS_FOLLOW_PID_THETA2V_KI 0.0f
#define CHASSIS_FOLLOW_PID_THETA2V_KD 0.0f
#define CHASSIS_FOLLOW_PID_THETA2V_I_LIMIT 90.0f
#define CHASSIS_FOLLOW_PID_THETA2V_OUT_LIMIT 270.0f
#define CHASSIS_FOLLOW_PID_THETA2V_PARAMS \
    {CHASSIS_FOLLOW_PID_THETA2V_KP, CHASSIS_FOLLOW_PID_THETA2V_KI, CHASSIS_FOLLOW_PID_THETA2V_KD, \
    CHASSIS_FOLLOW_PID_THETA2V_I_LIMIT, CHASSIS_FOLLOW_PID_THETA2V_OUT_LIMIT}

#define CHASSIS_DODGE_MODE_THETA   25
#define CHASSIS_DODGE_MODE_INTERVAL 750        // TODO:
#define CHASSIS_BIASED_ANGLE 20

/// Vision
#define VISION_BASIC_CONTROL_DELAY      100   /* ms */
#define VISION_DEFAULT_BULLET_SPEED     15.0f /* mm/ms = m/s */
#define VISION_VELOCITY_UPDATE_FRACTION 1.0f
#define VISION_PREDICT_FORWARD_AMOUNT   300

/// Thread Priority List
#define THREAD_CAN1_RX_PRIO                        (HIGHPRIO - 1)
#define THREAD_CAN1_TX_PRIO                        (HIGHPRIO - 2)
#define THREAD_CAN2_RX_PRIO                        (HIGHPRIO - 3)
#define THREAD_CAN2_TX_PRIO                        (HIGHPRIO - 4)
#define THREAD_MPU_PRIO                            (HIGHPRIO - 3)
#define THREAD_IST_PRIO                            (HIGHPRIO - 4)
#define THREAD_AHRS_PRIO                           (HIGHPRIO - 5)
#define THREAD_GIMBAL_SKD_PRIO                     (NORMALPRIO + 3)
#define THREAD_CHASSIS_SKD_PRIO                    (NORMALPRIO + 2)
#define THREAD_SHOOT_SKD_PRIO                      (NORMALPRIO + 1)
#define THREAD_USER_PRIO                           (NORMALPRIO)
#define THREAD_USER_ACTION_PRIO                    (NORMALPRIO - 1)
#define THREAD_SHOOT_LG_VISION_PRIO                (NORMALPRIO - 1)
#define THREAD_GIMBAL_LG_VISION_PRIO               (NORMALPRIO - 1)
#define THREAD_GIMBAL_LG_SENTRY_PRIO               (NORMALPRIO - 1)
#define THREAD_CHASSIS_LG_DODGE_PRIO               (NORMALPRIO - 2)
#define THREAD_CHASSIS_POWER_SET_PRIO              (NORMALPRIO - 3)
#define THREAD_SHOOT_LG_STUCK_DETECT_PRIO          (NORMALPRIO - 4)
#define THREAD_REFEREE_SENDING_PRIO                (NORMALPRIO - 6)
#define THREAD_REFEREE_SKD_PRIO                    (NORMALPRIO - 7)
#define THREAD_SUPERCAP_INIT_PRIO                   (NORMALPRIO - 8)
#define THREAD_INSPECTOR_PRIO                      (NORMALPRIO - 10)
#define THREAD_INSPECTOR_REFEREE_PRIO              (NORMALPRIO - 11)
#define THREAD_SHOOT_BULLET_COUNTER_PRIO           (LOWPRIO + 7)
#define THREAD_USER_CLIENT_DATA_SEND_PRIO          (LOWPRIO + 6)
#define THREAD_SHELL_PRIO                          (LOWPRIO + 5)
#define THREAD_BUZZER_SKD_PRIO                         (LOWPRIO)

/// Dev Board LED Usage List
#define DEV_BOARD_LED_SYSTEM_INIT 1
#define DEV_BOARD_LED_CAN         2
#define DEV_BOARD_LED_AHRS        3
#define DEV_BOARD_LED_REMOTE      4
#define DEV_BOARD_LED_GIMBAL      5
#define DEV_BOARD_LED_CHASSIS     6
#define DEV_BOARD_LED_REFEREE     7
#define DEV_BOARD_LED_SD_CARD     8

/// User Client Usage List
#define USER_CLIENT_FW_STATE_LIGHT                  0
#define USER_CLIENT_DODGE_MODE_LIGHT                1
#define USER_CLIENT_SUPER_CAPACITOR_STATUS_LIGHT    2
#define USER_CLIENT_SPEED_LEVEL_3_LIGHT             3
#define USER_CLIENT_SPEED_LEVEL_2_LIGHT             4
#define USER_CLIENT_SPEED_LEVEL_1_LIGHT             5

//#define USER_CLIENT_FW_SPEED_NUM                    1
#define USER_CLIENT_REMAINING_HEAT_NUM              2
#define USER_CLIENT_ACQUIRED_BULLET_NUM             3
//#define USER_CLIENT_ACTUAL_POWER_NUM                2
//#define USER_CLIENT_ACTUAL_POWER_NUM                2
#define USER_CLIENT_SUPER_CAPACITOR_VOLTAGE_NUM     1

/// Super Capacitor Configurations
#define SUPER_CAPACITOR_WARNING_VOLTAGE   15

#endif //META_INFANTRY_VEHICLE_INFANTRY_H
