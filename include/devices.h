#include "main.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

extern pros::Controller controller;

extern pros::v5::MotorGroup left_motors;
extern pros::v5::MotorGroup right_motors;

extern pros::Motor rmotor;
extern pros::Motor lmotor;

extern pros::Motor wall;

extern pros::Motor roller;
extern pros::Motor hooks;

extern pros::Rotation wall_rot;
extern pros::Distance hook_dist;
extern pros::Optical ring_col;
extern pros::Imu imu;

extern pros::adi::Pneumatics mogo;
extern pros::adi::Pneumatics doinker;