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
    PID left_pid = PID(1.0, 0, 1.0);
    PID right_pid = PID(1.0, 0, 1.0);
    PID turn_pid = PID(1.0, 0, 1.0);

    left_motors.set_zero_position_all(0.0);
    right_motors.set_zero_position_all(0.0);
    
    float deadzone = 1;

    float left;
    float right;

    int start_time = pros::millis();

    while (((left_motors.get_position() + right_motors.get_position())/2 - distance) > fabs(deadzone)) {
        float delta_time = (float)(pros::millis() - start_time) / 1000.0;
        left = left_pid.cycle(left_motors.get_position(), distance, delta_time, 10.0);
        right = right_pid.cycle(right_motors.get_position(), distance, delta_time, 10.0);

        left_motors.move_velocity(left);
        right_motors.move_velocity(right);

    }

}