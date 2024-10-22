#include "devices.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::v5::MotorGroup left_motors = pros::MotorGroup({-17,5,-13});
pros::v5::MotorGroup right_motors = pros::MotorGroup({8,-9,10});

pros::Motor wall = pros::Motor(15);

pros::Motor roller = pros::Motor(-19);
pros::Motor hooks = pros::Motor(-6);

pros::Rotation wall_rot = pros::Rotation(11);
pros::Distance hook_dist = pros::Distance(18);
pros::Optical ring_col = pros::Optical(20);

pros::adi::Pneumatics mogo = pros::adi::Pneumatics(1, false);
