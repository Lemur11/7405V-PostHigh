#include "main.h"
#include "devices.h"

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


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
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
	// state var
	state_enum state = NORMAL;

	// constants
	float target;
	float error;
	float prev_error;
	float kp = 4;
	float kd = 2;
	float delta;
	float intake_vel = 600;
	int cap = 28000;

	// coast brake mode
	left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	// reset rotation
	wall_rot.reset();

	while (true) {
		// Arcade control scheme
		int dir = controller.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = controller.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_motors.move(dir + turn);                      // Sets left motor voltage
		right_motors.move(dir - turn);                     // Sets right motor voltage

		// button logic
		// use toggle (on rising edge)
		if (controller.get_digital_new_press(DIGITAL_R1)) {
			mogo.toggle();
		}
		if (controller.get_digital(DIGITAL_L1)) {
			roller.move_velocity(intake_vel);
			hooks.move_velocity(intake_vel);
		} 
		else if (controller.get_digital(DIGITAL_L2)) {
			roller.move_velocity(-intake_vel);		
			hooks.move_velocity(-intake_vel);
		}
		else {
			roller.move_velocity(0);
			hooks.move_velocity(0);
		}
		if (controller.get_digital_new_press(DIGITAL_A)) {
			if (state == NORMAL) {
				state = CATCH;
			}
			else if (state == ARMED) {
				state = TOGGLED;
			}
			else {
				state = USED;
			}
		}

		// fish mech finite state machine 
		switch(state) {
			case NORMAL:
			{
				pros::lcd::print(6, "NORMAL");
				wall.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				break;
			}
			case CATCH:
			{
				hooks.move_velocity(100);
				roller.move_velocity(600);
				pros::lcd::print(6, "CATCH");
				if (hook_dist.get() < 30) {
					hooks.move_velocity(0);
					state = STOPPED;
				}
				break;
			}
			case STOPPED:
			{
				pros::lcd::print(6, "STOPPED");
				target = 77.0;
				prev_error = target;
				error = target;
				int reading = wall_rot.get_angle();
				printf("%d\n", reading);
				if (reading > cap) {
					reading = reading - 36000;
				}
				error = target - ((float)reading / 100.0);
				if (abs(error) < 5) {
					wall.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
					wall.move_velocity(0);
					state = ARMED;
					break;
				}
				delta = error - prev_error;
				printf("Target: %f, Pos: %d, Error: %f, Delta: %f, Vel: %f\n", target, reading, error, delta, kp*error - kd*delta);
				prev_error = error;
				wall.move_velocity(kp*error - kd*delta);
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
				state = MOVED;
				break;
			}
			case MOVED:
			{
				pros::lcd::print(6, "MOVED");
				hooks.move_velocity(50);
				if (target != 200) {
					target = 200;
					prev_error = target;
					error = target;
				}
				int reading = wall_rot.get_angle();
				if (reading > cap) {
					reading = reading - 36000;
				}
				error = target - ((float)reading / 100.0);
				if (abs(error) < 5) {
					wall.move_velocity(0);
					state = USED;
					break;
				}
				delta = error - prev_error;
				printf("Target: %f, Pos: %d, Error: %f, Delta: %f, Vel: %f\n", target, reading, error, delta, kp*error - kd*delta);
				prev_error = error;
				wall.move_velocity(kp*error - kd*delta);
				break;
			}
			case USED:
			{
				pros::lcd::print(6, "USED");
				if (target != 0) {
					target = 0;
					prev_error = target;
					error = target;
				}
				int reading = wall_rot.get_angle();
				if (reading > cap) {
					reading = reading - 36000;
				}
				error = target - ((float)reading / 100.0);
				if (abs(error) < 4) {
					wall.move_velocity(0);
					state = NORMAL;
					break;
				}
				delta = error - prev_error;
				printf("Target: %f, Pos: %d, Error: %f, Delta: %f, Vel: %f\n", target, reading, error, delta, kp*error - kd*delta);
				prev_error = error;
				wall.move_velocity(kp*error - kd*delta);
				break;
			}
		}

		pros::lcd::print(0, "%d", state);
		pros::lcd::print(1, "%d", hook_dist.get());
		pros::lcd::print(2, "Error: %f", error);
		pros::lcd::print(3, "Prev: %f", prev_error);
		pros::lcd::print(4, "Rot: %f", (float)wall_rot.get_angle() / 100.0);

		pros::delay(20);                               // Run for 20 ms then update
	}
}