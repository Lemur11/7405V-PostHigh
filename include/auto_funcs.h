#include "devices.h"
#include "main.h"

float wall_pid_cycle(wall_state_enum state, float target, float prev_error, int cap, float kp, float kd, int next_state, float deadzone=5.0);

void drive(float distance, float dir);

void turn(float angle, bool right);

float get_angle(float current, float desired, float right);