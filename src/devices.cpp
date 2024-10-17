#include "devices.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::v5::MotorGroup left_motors = pros::MotorGroup({-7,19,-2});
pros::v5::MotorGroup right_motors = pros::MotorGroup({16,-14,20});

pros::Motor wall = pros::Motor(13);

pros::Motor roller = pros::Motor(-1);
pros::Motor hooks = pros::Motor(-17);

pros::Rotation wall_rot = pros::Rotation(12);
pros::Distance hook_dist = pros::Distance(18);

pros::adi::Pneumatics mogo = pros::adi::Pneumatics(1, false);
