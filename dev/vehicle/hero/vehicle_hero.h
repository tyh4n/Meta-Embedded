//
// Created by 404 on 2019-05-18.
//

#ifndef META_INFANTRY_VEHICLE_HERO_H
#define META_INFANTRY_VEHICLE_HERO_H

/** Mechanism Parameters */
#define CHASSIS_WHEEL_BASE 492.4f                      // distance between front axle and the back axle [mm]
#define CHASSIS_WHEEL_TREAD 472.76f                     // distance between left and right wheels [mm]
#define CHASSIS_WHEEL_CIRCUMFERENCE 478.0f             // [mm]

#define SHOOT_DEGREE_PER_BULLER 72.0f                 // rotation degree of bullet loader for each bullet
#define SHOOT_DEGREE_PER_BULLER_PLATE 36.0f           // rotation degree of bullet plate for each bullet

/** Shooting Mechanism User Preference **/
#define GIMBAL_REMOTE_FRICTION_WHEEL_DUTY_CYCLE 0.5
#define GIMBAL_PC_FRICTION_WHEEL_DUTY_CYCLE 0.5

/** Initial Gimbal Angle Raw **/
#define GIMBAL_YAW_FRONT_ANGLE_RAW 5372
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 4128

/** Gimbal Motor PID Params **/
#define GIMBAL_PID_YAW_V2I_KP 22.0f
#define GIMBAL_PID_YAW_V2I_KI 0.0006f
#define GIMBAL_PID_YAW_V2I_KD 0.0f
#define GIMBAL_PID_YAW_V2I_I_LIMIT 30000.0f
#define GIMBAL_PID_YAW_V2I_OUT_LIMIT 30000.0f
#define GIMBAL_YAW_V2I_PID_PARAMS \
    {GIMBAL_PID_YAW_V2I_KP, GIMBAL_PID_YAW_V2I_KI, GIMBAL_PID_YAW_V2I_KD, \
    GIMBAL_PID_YAW_V2I_I_LIMIT, GIMBAL_PID_YAW_V2I_OUT_LIMIT}

#define GIMBAL_PID_YAW_A2V_KP 8.5f
#define GIMBAL_PID_YAW_A2V_KI -0.0018f
#define GIMBAL_PID_YAW_A2V_KD 0.03f
#define GIMBAL_PID_YAW_A2V_I_LIMIT 30000.0f
#define GIMBAL_PID_YAW_A2V_OUT_LIMIT 30000.0f
#define GIMBAL_YAW_A2V_PID_PARAMS \
    {GIMBAL_PID_YAW_A2V_KP, GIMBAL_PID_YAW_A2V_KI, GIMBAL_PID_YAW_A2V_KD, \
    GIMBAL_PID_YAW_A2V_I_LIMIT, GIMBAL_PID_YAW_A2V_OUT_LIMIT}

#define GIMBAL_PID_PITCH_V2I_KP 11.0f
#define GIMBAL_PID_PITCH_V2I_KI 0.11f
#define GIMBAL_PID_PITCH_V2I_KD 0.0f
#define GIMBAL_PID_PITCH_V2I_I_LIMIT 1000.0f
#define GIMBAL_PID_PITCH_V2I_OUT_LIMIT 3000.0f
#define GIMBAL_PITCH_V2I_PID_PARAMS \
    {GIMBAL_PID_PITCH_V2I_KP, GIMBAL_PID_PITCH_V2I_KI, GIMBAL_PID_PITCH_V2I_KD, \
    GIMBAL_PID_PITCH_V2I_I_LIMIT, GIMBAL_PID_PITCH_V2I_OUT_LIMIT}

#define GIMBAL_PID_PITCH_A2V_KP 10.0f
#define GIMBAL_PID_PITCH_A2V_KI 0.0f
#define GIMBAL_PID_PITCH_A2V_KD 0.1f
#define GIMBAL_PID_PITCH_A2V_I_LIMIT 1000.0f
#define GIMBAL_PID_PITCH_A2V_OUT_LIMIT 3000.0f
#define GIMBAL_PITCH_A2V_PID_PARAMS \
    {GIMBAL_PID_PITCH_A2V_KP, GIMBAL_PID_PITCH_A2V_KI, GIMBAL_PID_PITCH_A2V_KD, \
    GIMBAL_PID_PITCH_A2V_I_LIMIT, GIMBAL_PID_PITCH_A2V_OUT_LIMIT}

#define GIMBAL_PID_BULLET_LOADER_V2I_KP 19.0f
#define GIMBAL_PID_BULLET_LOADER_V2I_KI 0.15f
#define GIMBAL_PID_BULLET_LOADER_V2I_KD 0.0f
#define GIMBAL_PID_BULLET_LOADER_V2I_I_LIMIT 3000.0f
#define GIMBAL_PID_BULLET_LOADER_V2I_OUT_LIMIT 5000.0f
#define GIMBAL_BL_V2I_PID_PARAMS \
    {GIMBAL_PID_BULLET_LOADER_V2I_KP, GIMBAL_PID_BULLET_LOADER_V2I_KI, GIMBAL_PID_BULLET_LOADER_V2I_KD, \
    GIMBAL_PID_BULLET_LOADER_V2I_I_LIMIT, GIMBAL_PID_BULLET_LOADER_V2I_OUT_LIMIT}


/*** Bullet Plate PID Params ***/
#define GIMBAL_PID_BULLET_PLATE_V2I_KP 20.0f
#define GIMBAL_PID_BULLET_PLATE_V2I_KI 0.13f
#define GIMBAL_PID_BULLET_PLATE_V2I_KD 0.0f
#define GIMBAL_PID_BULLET_PLATE_V2I_I_LIMIT 10000.0f
#define GIMBAL_PID_BULLET_PLATE_V2I_OUT_LIMIT 10000.0f
#define GIMBAL_PID_BULLET_PLATE_V2I_PARAMS \
    {GIMBAL_PID_BULLET_PLATE_V2I_KP, GIMBAL_PID_BULLET_PLATE_V2I_KI, GIMBAL_PID_BULLET_PLATE_V2I_KD, \
    GIMBAL_PID_BULLET_PLATE_V2I_I_LIMIT, GIMBAL_PID_BULLET_PLATE_V2I_OUT_LIMIT}

/*** Chassis PID Params ***/
#define CHASSIS_PID_V2I_KP 26.0f
#define CHASSIS_PID_V2I_KI 0.1f
#define CHASSIS_PID_V2I_KD 0.02f
#define CHASSIS_PID_V2I_I_LIMIT 2000.0f
#define CHASSIS_PID_V2I_OUT_LIMIT 6000.0f
#define CHASSIS_PID_V2I_PARAMS \
    {CHASSIS_PID_V2I_KP, CHASSIS_PID_V2I_KI, CHASSIS_PID_V2I_KD, \
    CHASSIS_PID_V2I_I_LIMIT, CHASSIS_PID_V2I_OUT_LIMIT}
#define GIMBAL_YAW_ACTUAL_VELOCITY (-MPU6500::angle_speed.z)
#define GIMBAL_PITCH_ACTUAL_VELOCITY (MPU6500::angle_speed.x)

#endif //META_INFANTRY_VEHICLE_HERO_H

