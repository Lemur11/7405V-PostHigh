#include "wall.h"
#include "constants.h"

pros::Task* wall_update = nullptr;

wall_mech::wall_mech() :
    wall_pid(0.015, 0, 0.0001),
    prev_time(pros::millis())
    {
    wall_update = new pros::Task {[=] {
        while (true) {
            update();
            pros::delay(20);
            };
        }
    };
 }

void wall_mech::set_target(int reference, bool blocking) {
    target.store(reference);
    done = false;
    if (blocking) {
        while (!done) {
            pros::delay(20);
        }
    }
}

int wall_mech::get_rotation() {
    return wall_rot.get_position() % 36000;
}

void wall_mech::update() {
    int reading = wall_rot.get_position() % 36000;
    if (reading > wall_cap) {
        reading = reading - 36000;
    }
    pros::lcd::print(1, "Reading: %d", reading);
    cur_time = pros::millis();
    float power = wall_pid.cycle(target.load(), reading, (float)(cur_time - prev_time) / 1000.0, 10000, true);
    wall.move_velocity(power);
    if ((target.load() - reading) < wall_deadzone) {
        wall.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        wall.move_velocity(0);
        done = true;
    }
    else {
        done = false;
    }
    prev_time = cur_time;
    pros::delay(20);
}
