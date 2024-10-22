#include "main.h"
#include "devices.h"
#include "auto_funcs.h"
#include <atomic>
#include "pid.h"

// String print_state(&state_enum s) {
// 	switch (s) {
// 		case NORMAL: return "NORMAL"
// 		case CATCH: return "CATCH"
// 		case STOPPED: return "STOPPED"
// 		case ARMED: return "ARMED"
// 		case MOVED: return "MOVED"
// 		case USED: return "USED"
// 	}
// }

// state vars
std::atomic<wall_state_enum> wall_state(NORMAL);
std::atomic<sort_state_enum> sort_state(OFF);

void color_sort() {
	int delay = 100;
	if (hook_dist.get() < 30 && wall_state == NORMAL) {
		double reading = ring_col.get_hue();
		switch (sort_state) {
			case RED:
			{
				if (300 < reading < 360 || reading < 120) {
					sort_state = SORTING;
					pros::delay(delay);
					hooks.move_velocity(0);
					pros::delay(100);
					sort_state = RED;
				}
			}
		}
	}
}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	wall_rot.reset_position();
	wall_rot.set_data_rate(5);
	pros::delay(500);
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// constants
	float target;
	float error;
	float prev_error;
	float kp = 4;
	float kd = 2;
	float delta;
	float intake_vel = 300;
	float deadzone = 500;
	int cap = 28000;

	// pid
	PID wall_pid = PID(0.1, 0, 0.1);

	// color sort
	// pros::Task sorting(color_sort);

	// coast brake mode
	left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	//init time
	int prev_time = pros::millis();
	int cur_time;

	while (true) {
		// Arcade control scheme
		int dir = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_motors.move(dir + turn);                      // Sets left motor voltage
		right_motors.move(dir - turn);                     // Sets right motor voltage

		// button logic
		// use toggle (on rising edge)
		if (controller.get_digital_new_press(DIGITAL_L1)) {
			mogo.extend();
		} else if (controller.get_digital_new_press(DIGITAL_L2)) {
			mogo.retract();
		}
		if (!(sort_state == SORTING)) {
			if (controller.get_digital(DIGITAL_R1)) {
				roller.move_velocity(intake_vel);
				hooks.move_velocity(intake_vel);
			} 
			else if (controller.get_digital(DIGITAL_R2)) {
				roller.move_velocity(-intake_vel);		
				hooks.move_velocity(-intake_vel);
			}
			else {
				roller.move_velocity(0);
				hooks.move_velocity(0);
			}
		}
		if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
			wall_pid.reset();
			if (wall_state == NORMAL) {
				wall_state = CATCH;
			}
			else if (wall_state == ARMED) {
				wall_state = TOGGLED;
			}
			else {
				wall_state = USED;
			}
		}

		// fish mech finite state machine 
		switch(wall_state) {
			case NORMAL:
			{
				pros::lcd::print(6, "NORMAL");
				// wall.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				break;
			}
			case CATCH:
			{
				hooks.move_velocity(100);
				roller.move_velocity(600);
				pros::lcd::print(6, "CATCH");
				if (hook_dist.get() < 30) {
					hooks.move_velocity(0);
					prev_error = 77;
					wall_state = STOPPED;
				}
				break;
			}
			case STOPPED:
			{
				pros::lcd::print(6, "STOPPED");
				int reading = wall_rot.get_position() % 36000;
    			if (reading > cap) {
        			reading = reading - 36000;
    			}
				cur_time = pros::millis();
				float power = wall_pid.cycle(7700, reading, (float)(cur_time - prev_time), 10000, true);
				wall.move_velocity(power);
				if (fabs(7700 - reading) < deadzone) {
        			wall.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        			wall.move_velocity(0);
					wall_state = ARMED;
				}
				prev_time = cur_time;
				break;
			}
			case ARMED:
			{
				pros::lcd::print(6, "ARMED");
				break;
			}
			case TOGGLED:
			{
				pros::lcd::print(6, "TOGGLED");
				hooks.move_velocity(50);
				int reading = wall_rot.get_position() % 36000;
    			if (reading > cap) {
        			reading = reading - 36000;
    			}
				cur_time = pros::millis();
				float power = wall_pid.cycle(20000, reading, (float)(cur_time - prev_time), 10000, true);
				wall.move_velocity(power);
				if (fabs(20000 - reading) < deadzone) {
        			wall.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        			wall.move_velocity(0);
					wall_state = USED;
				}
				prev_time = cur_time;
				break;
			}
			case USED:
			{
				pros::lcd::print(6, "USED");
				int reading = wall_rot.get_position() % 36000;
    			if (reading > cap) {
        			reading = reading - 36000;
    			}
				cur_time = pros::millis();
				float power = wall_pid.cycle(0, reading, (float)(cur_time - prev_time), 10000, true);
				wall.move_velocity(power);
				if (fabs(0 - reading) < deadzone) {
        			wall.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        			wall.move_velocity(0);
					wall_state = USED;
				}				
				prev_time = cur_time;
				break;
			}
		}

		pros::lcd::print(0, "%d", wall_state.load());
		pros::lcd::print(1, "%d", hook_dist.get());
		pros::lcd::print(2, "Error: %f", error);
		pros::lcd::print(3, "Prev: %f", prev_error);
		pros::lcd::print(4, "Rot: %f", (float)wall_rot.get_angle() / 100.0);

		pros::delay(20);                               // Run for 20 ms then update
	}
}