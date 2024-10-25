#include "auto_funcs.h"
#include "pid.h"
#include <algorithm>

// float wall_pid_cycle(wall_state_enum state, float target, float prev_error, int cap, float kp, float kd, float deadzone) {
//     int reading = wall_rot.get_position() % 36000;
//     printf("%d\n", reading);
//     if (reading > cap) {
//         reading = reading - 36000;
//     }
//     float error = target - ((float)reading / 100.0);
//     // if (fabs(error) < deadzone) {
//     //     wall.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     //     wall.move_velocity(0);
//     //     return ;
//     // }
//     // float delta = error - prev_error;
//     // printf("Target: %f, Pos: %d, Error: %f, Delta: %f, Vel: %f\n", target, reading, error, delta, kp*error - kd*delta);
//     // wall.move_velocity(kp*error - kd*delta);
//     return error;
// }

void drive(float distance, float dir) {
	printf("RUNNING\n");
    float kp = 0.7;
    float ki = 0.0; 
    float kd = 0.1;
    if (abs(dir - 1.0) < 0.01) {
        kp = 0.7;
        ki = 0.0; 
        kd = 0.1;
    } else {
        kp = 0.5;
        ki = 0.0; 
        kd = 0.1;
    }

    PID left_pid = PID(kp, ki, kd);
    PID right_pid = PID(kp, ki, kd);
    PID turn_pid = PID(0.8, 0.0, 0.0);

    left_pid.set_prev(distance);
    right_pid.set_prev(distance);

    lmotor.set_zero_position_all(0.0);
    rmotor.set_zero_position_all(0.0);
    
    float deadzone = 1;

    float left;
    float right;
    float turn;

    float imu_reading;

    int prev_time = pros::millis();
    int cur_time;

    // printf("Left: %f, Right: %f\n", lmotor.get_position(), rmotor.get_position());
    while (fabs(lmotor.get_position() - distance) > deadzone && fabs(rmotor.get_position() - distance) > deadzone) {
        cur_time = pros::millis();
        float delta_time = (float)(pros::millis() - prev_time) / 1000.0;
        left = left_pid.cycle(distance, dir*lmotor.get_position(), delta_time, 100000000.0, true);
        right = right_pid.cycle(distance, dir*rmotor.get_position(), delta_time, 100000000.0, true);

        imu_reading = imu.get_heading();
        if (imu_reading > 250) {
            imu_reading = imu_reading - 360;
        }

        turn = turn_pid.cycle(0, imu_reading, delta_time, 10.0);
        // printf("Left: %f, Right: %f, Heading: %f, delta %f, LPID: %f, RPID: %f, turn: %f\n", left_motors.get_position(), right_motors.get_position(), imu.get_heading(), delta_time, left, right, turn);
        pros::lcd::print(1, "Left %f", lmotor.get_position());
        pros::lcd::print(2, "Right %f", rmotor.get_position());
        left_motors.move_velocity(left*dir);
        right_motors.move_velocity(right*dir);
        prev_time = cur_time;
        pros::delay(20);

    }
	left_motors.move_velocity(0);
	right_motors.move_velocity(0);

}

float get_angle(float current, float desired, float right) {
    // Assumes current and desired are between 0 & 360
    float larger_a = std::max(current, desired);
    float smaller_a = std::min(current, desired);
    float dist = larger_a - smaller_a;
    float sign = 1.0;
    if (dist > 180) {
        dist = 360 - dist;
        sign *= -1.0;
    }
    if (abs(larger_a - current) < 0.01) {
        sign *= -1.0;
    }
    return dist * sign;


} 

void turn(float angle, bool right) {

    PID turn_pid = PID(2, 0.0, 0.2);
    
    float deadzone = 0.5;
    int prev_time = pros::millis();
    int cur_time;

    float power;

    float err = get_angle(imu.get_heading(), angle, right);
    printf("headuing: %f\n" , imu.get_heading());
    turn_pid.set_prev(-err);
    while (fabs(err) > deadzone) {
        err = get_angle(imu.get_heading(), angle, right);
        printf("Err: %f\n" , err);
        cur_time = pros::millis();
        float delta_time = (float)(pros::millis() - prev_time) / 1000.0;

        power = turn_pid.cycle(0, err, delta_time, 10000000.0);
        printf("Power: %f", power);
        left_motors.move_velocity(-power);
        right_motors.move_velocity(power);

        prev_time = cur_time;
        pros::lcd::print(0, "Heading: %f", imu.get_heading());
        pros::delay(20);

    }
    left_motors.move_velocity(0);
    right_motors.move_velocity(0);
}