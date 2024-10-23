#include "auto_funcs.h"
#include "pid.h"

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

void drive(float distance, bool dir) {
	printf("RUNNING\n");
    float kp = 0.07;
    float ki = 0.0;
    float kd = 0.0;
    PID left_pid = PID(kp, ki, kd);
    PID right_pid = PID(kp, ki, kd);
    PID turn_pid = PID(0.5, 0.0, 0.0);

    left_motors.set_zero_position_all(0.0);
    right_motors.set_zero_position_all(0.0);
    
    float deadzone = 1;

    float left;
    float right;
    float turn;

    float imu_reading;

    int prev_time = pros::millis();
    int cur_time;
    printf("Left: %f, Right: %f\n", left_motors.get_position(), right_motors.get_position());
    while (fabs(left_motors.get_position() - distance) > deadzone && fabs(right_motors.get_position() - distance) > deadzone) {
        cur_time = pros::millis();
        float delta_time = (float)(pros::millis() - prev_time) / 1000.0;
        left = left_pid.cycle(distance, left_motors.get_position(), delta_time, 10.0);
        right = right_pid.cycle(distance, right_motors.get_position(), delta_time, 10.0);

        imu_reading = imu.get_heading();
        if (imu_reading > 250) {
            imu_reading = imu_reading - 360;
        }

        turn = turn_pid.cycle(0, imu_reading, delta_time, 10.0);
        printf("Left: %f, Right: %f, Heading: %f, delta %f, LPID: %f, RPID: %f, turn: %f\n", left_motors.get_position(), right_motors.get_position(), imu.get_heading(), delta_time, left, right, turn);
        pros::lcd::print(1, "Left %f", left_motors.get_position());
        pros::lcd::print(2, "Right %f", right_motors.get_position());
        left_motors.move_velocity(left + turn);
        right_motors.move_velocity(right - turn);
        prev_time = cur_time;
        pros::delay(20);

    }
	left_motors.move_velocity(0);
	right_motors.move_velocity(0);

}