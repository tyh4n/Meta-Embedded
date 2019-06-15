//
// Created by zhukerui on 2019/5/18.
// Copy from liuzikai
//

#ifndef META_INFANTRY_VEHICLE_SENTRY_H
#define META_INFANTRY_VEHICLE_SENTRY_H

/** Installation Based Params **/

// Raw angle of yaw and pitch when gimbal points straight forward.
//   Note: the program will echo the raw angles of yaw and pitch as the program starts
#define GIMBAL_YAW_FRONT_ANGLE_RAW 119.7f
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 0.7f

// Relationship between MPU6500 data and gimbal angular velocities
#define GIMBAL_YAW_ACTUAL_VELOCITY (-MPU6500Controller::angle_speed.x)
#define GIMBAL_PITCH_ACTUAL_VELOCITY (MPU6500Controller::angle_speed.y)

/** Shooting Mechanism User Preference **/
#define GIMBAL_REMOTE_FRICTION_WHEEL_DUTY_CYCLE 0.5
#define GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE 0.5

/** Gimbal Motor PID Params **/

#define GIMBAL_PID_YAW_V2I_KP 9.4f
#define GIMBAL_PID_YAW_V2I_KI 0.22f
#define GIMBAL_PID_YAW_V2I_KD 0.21f
#define GIMBAL_PID_YAW_V2I_I_LIMIT 1000.0f
#define GIMBAL_PID_YAW_V2I_OUT_LIMIT 3000.0f
#define GIMBAL_YAW_V2I_PID_PARAMS \
    {GIMBAL_PID_YAW_V2I_KP, GIMBAL_PID_YAW_V2I_KI, GIMBAL_PID_YAW_V2I_KD, \
    GIMBAL_PID_YAW_V2I_I_LIMIT, GIMBAL_PID_YAW_V2I_OUT_LIMIT}

#define GIMBAL_PID_YAW_A2V_KP 6.5f
#define GIMBAL_PID_YAW_A2V_KI 0.0f
#define GIMBAL_PID_YAW_A2V_KD 0.4f
#define GIMBAL_PID_YAW_A2V_I_LIMIT 1000.0f
#define GIMBAL_PID_YAW_A2V_OUT_LIMIT 3000.0f
#define GIMBAL_YAW_A2V_PID_PARAMS \
    {GIMBAL_PID_YAW_A2V_KP, GIMBAL_PID_YAW_A2V_KI, GIMBAL_PID_YAW_A2V_KD, \
    GIMBAL_PID_YAW_A2V_I_LIMIT, GIMBAL_PID_YAW_A2V_OUT_LIMIT}

#define GIMBAL_PID_PITCH_V2I_KP 5.8f
#define GIMBAL_PID_PITCH_V2I_KI 0.43f
#define GIMBAL_PID_PITCH_V2I_KD 0.019f
#define GIMBAL_PID_PITCH_V2I_I_LIMIT 1000.0f
#define GIMBAL_PID_PITCH_V2I_OUT_LIMIT 3000.0f
#define GIMBAL_PITCH_V2I_PID_PARAMS \
    {GIMBAL_PID_PITCH_V2I_KP, GIMBAL_PID_PITCH_V2I_KI, GIMBAL_PID_PITCH_V2I_KD, \
    GIMBAL_PID_PITCH_V2I_I_LIMIT, GIMBAL_PID_PITCH_V2I_OUT_LIMIT}

#define GIMBAL_PID_PITCH_A2V_KP 6.5f
#define GIMBAL_PID_PITCH_A2V_KI 0.0f
#define GIMBAL_PID_PITCH_A2V_KD 0.45f
#define GIMBAL_PID_PITCH_A2V_I_LIMIT 1000.0f
#define GIMBAL_PID_PITCH_A2V_OUT_LIMIT 3000.0f
#define GIMBAL_PITCH_A2V_PID_PARAMS \
    {GIMBAL_PID_PITCH_A2V_KP, GIMBAL_PID_PITCH_A2V_KI, GIMBAL_PID_PITCH_A2V_KD, \
    GIMBAL_PID_PITCH_A2V_I_LIMIT, GIMBAL_PID_PITCH_A2V_OUT_LIMIT}

// TODO: select a better params
#define GIMBAL_PID_BULLET_LOADER_V2I_KP 20.0f
#define GIMBAL_PID_BULLET_LOADER_V2I_KI 0.0f
#define GIMBAL_PID_BULLET_LOADER_V2I_KD 0.0f
#define GIMBAL_PID_BULLET_LOADER_V2I_I_LIMIT 0.0f
#define GIMBAL_PID_BULLET_LOADER_V2I_OUT_LIMIT 2000.0f
#define GIMBAL_BL_V2I_PID_PARAMS \
    {GIMBAL_PID_BULLET_LOADER_V2I_KP, GIMBAL_PID_BULLET_LOADER_V2I_KI, GIMBAL_PID_BULLET_LOADER_V2I_KD, \
    GIMBAL_PID_BULLET_LOADER_V2I_I_LIMIT, GIMBAL_PID_BULLET_LOADER_V2I_OUT_LIMIT}

/*** Chassis PID Params ***/

#define SENTRY_CHASSIS_PID_V2I_KP 40.0f
#define SENTRY_CHASSIS_PID_V2I_KI 0.25f
#define SENTRY_CHASSIS_PID_V2I_KD 0.22f
#define SENTRY_CHASSIS_PID_V2I_I_LIMIT 1000.0f
#define SENTRY_CHASSIS_PID_V2I_OUT_LIMIT 5000.0f
#define SENTRY_CHASSIS_PID_V2I_PARAMS \
    {SENTRY_CHASSIS_PID_V2I_KP, SENTRY_CHASSIS_PID_V2I_KI, SENTRY_CHASSIS_PID_V2I_KD, \
    SENTRY_CHASSIS_PID_V2I_I_LIMIT, SENTRY_CHASSIS_PID_V2I_OUT_LIMIT}

#define CURVE_1_RIGHT 30.0f
#define CURVE_1_LEFT 40.0f
#define STRAIGHTWAY_RIGHT 50.0f
#define STRAIGHTWAY_LEFT 60.0f
#define CURVE_2_RIGHT 70.0f
#define CURVE_2_LEFT 80.0f
#define CRUISING_SPEED 80.0f
#define ESCAPE_SPEED 110.0f
#define CRUISING_PID_A2V_PARAMS {20, 0, 0, 0, CRUISING_SPEED}
#define ESCAPE_PID_A2V_PARAMS {20, 0, 0, 0, ESCAPE_SPEED}

#endif //META_INFANTRY_VEHICLE_SENTRY_H
