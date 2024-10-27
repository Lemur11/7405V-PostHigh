#include "devices.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::v5::MotorGroup left_motors = pros::MotorGroup({-17,16,-13});
pros::v5::MotorGroup right_motors = pros::MotorGroup({8,-9,10});

pros::Motor lmotor = pros::Motor(-13);
pros::Motor rmotor = pros::Motor(10);

pros::Motor wall = pros::Motor(14);

pros::Motor roller = pros::Motor(-19);
pros::Motor hooks = pros::Motor(-4);

pros::Rotation wall_rot = pros::Rotation(11);
pros::Distance hook_dist = pros::Distance(18);
pros::Optical ring_col = pros::Optical(20);
pros::Imu imu = pros::Imu(6);

pros::adi::Pneumatics mogo = pros::adi::Pneumatics('e', true);
pros::adi::Pneumatics doinker = pros::adi::Pneumatics('h', true);