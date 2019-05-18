//
// Created by zhukerui on 2019/4/29.
//

#include "sentry_chassis_calculator.h"

bool SentryChassisController::enable;
SentryChassisController::sentry_mode_t SentryChassisController::running_mode;
bool SentryChassisController::change_speed;
time_msecs_t SentryChassisController::start_time;
float SentryChassisController::target_position;
float SentryChassisController::target_velocity;
float SentryChassisController::radius;
int SentryChassisController::const_current;
PIDController SentryChassisController::motor_right_pid;
PIDController SentryChassisController::motor_left_pid;


void SentryChassisController::init_controller(CANInterface* can_interface) {
    SentryChassis::init(can_interface);
    Referee::init();
    enable = false;
    running_mode = STOP_MODE;
    clear_position();
    radius = 30.0f;
    const_current = 1000;
    change_speed = false;
}

void SentryChassisController::clear_position() {
    for(int i = 0; i < MOTOR_COUNT; i++){
        motor[i].actual_angle = 0;
        motor[i].round_count = 0;
        motor[i].present_position = 0;
        motor[i].present_velocity = 0;
    }
    target_position = 0;
    target_velocity = 0;
}

void SentryChassisController::set_destination(float dist) {
    target_position = dist;
    // Every time a new target position is set, a new target velocity should be decided
    if (target_position > motor[MOTOR_RIGHT].present_position){
        target_velocity = maximum_speed;
    } else if (target_position < motor[MOTOR_RIGHT].present_position){
        target_velocity = - maximum_speed;
    } else{
        target_velocity = 0;
    }
}

void SentryChassisController::update_target_current() {

    switch (running_mode){
        case (CONST_CURRENT_MODE):
            motor[MOTOR_LEFT].target_current = motor[MOTOR_RIGHT].target_current = const_current;
            return;
        case (ONE_STEP_MODE):
            // If we are in the ONE_STEP_MODE
            if((motor[MOTOR_LEFT].present_position >= target_position-5 && motor[MOTOR_LEFT].present_position <= target_position+5)
            || (motor[MOTOR_RIGHT].present_position >= target_position-5 && motor[MOTOR_RIGHT].present_position <= target_position+5)) {
                // If the sentry is in the "stop area", we stop the sentry by simply set the target velocity to 0
                target_velocity = 0;
            }
            break;
        case (AUTO_MODE):
            // If we are in the AUTO MODE
            if((motor[MOTOR_LEFT].present_position >= radius || motor[MOTOR_LEFT].present_position <= -radius)
               || (motor[MOTOR_RIGHT].present_position >= radius || motor[MOTOR_RIGHT].present_position <= -radius)) {
                // If the sentry is in the "stop area", we change the destination according to the rule we set in set_auto_destination()
                if(motor[MOTOR_RIGHT].present_position > 0){
                    set_destination(-radius);
                } else{
                    set_destination(radius);
                }
            }
            break;
        case (STOP_MODE):
        default:
            // If we are in the STOP_MODE, then the sentry now is not movable
            target_velocity = 0;
    }
    // Set the target current
    if (enable){
        if(change_speed){
            // If the change_speed is true, then the speed should change variously from time to time
            time_msecs_t present_time = SYSTIME - start_time;
            motor[MOTOR_RIGHT].target_current = (int)(motor_right_pid.calc(motor[MOTOR_RIGHT].present_velocity, (abs(cos(3.1415f*present_time/2000.0f)))*target_velocity));
            motor[MOTOR_LEFT].target_current = (int)(motor_left_pid.calc(motor[MOTOR_LEFT].present_velocity, (abs(cos(3.1415f*present_time/2000.0f)))*target_velocity));
        }else{
            motor[MOTOR_RIGHT].target_current = (int)(motor_right_pid.calc(motor[MOTOR_RIGHT].present_velocity, target_velocity));
            motor[MOTOR_LEFT].target_current = (int)(motor_left_pid.calc(motor[MOTOR_LEFT].present_velocity, target_velocity));
        }
        if(Referee::power_heat_data.chassis_power > 20) LOG("power overload: %.2f", Referee::power_heat_data.chassis_power);
    }else {
        motor[MOTOR_LEFT].target_current = motor[MOTOR_RIGHT].target_current = 0;
    }
}

void SentryChassisController::set_mode(SentryChassisController::sentry_mode_t target_mode, float index) {
    running_mode = target_mode;
    clear_position();
    if(running_mode == AUTO_MODE){
        radius = index;
        set_destination(radius);
    } else if(running_mode == CONST_CURRENT_MODE){
        const_current = (int)index;
    }
}