#include "main.h"
#include "devices.h"
#include "auto_funcs.h"
#include "pid.h"
#include "constants.h"
#include <atomic>


#define wall_move(pos) \
		prev_time = pros::millis(); \
		while (true) { \
			int reading = wall_rot.get_position() % 36000; \
			if (reading > wall_cap) { \
				reading = reading - 36000; \
			} \
			pros::lcd::print(1, "Reading: %d", reading); \
			cur_time = pros::millis(); \
			float power = wall_pid.cycle(pos, reading, (float)(cur_time - prev_time) / 1000.0, 10000, true); \
			wall.move_velocity(power); \
			if ((pos - reading) < wall_deadzone) { \
				wall.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); \
				wall.move_velocity(0); \
				break; \
			} \
			prev_time = cur_time; \
			pros::delay(20); \
		} \

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

PID wall_pid = PID(0.015, 0, 0.0001);

void wall_task() {
	int prev_time;
	int cur_time;
	while (true) {
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
				if (hook_dist.get() < 40) {
					hooks.move_velocity(0);
					wall_state = STOPPED;
				}
				break;
			}
			case STOPPED:
			{
				pros::lcd::print(6, "STOPPED");
				int reading = wall_rot.get_position() % 36000;
    			if (reading > wall_cap) {
        			reading = reading - 36000;
    			}
				cur_time = pros::millis();
				float power = wall_pid.cycle(7700, reading, (float)(cur_time - prev_time) / 1000.0, 10000, true);
				wall.move_velocity(power);
				if (fabs(7700 - reading) < wall_deadzone) {
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
    			if (reading > wall_cap) {
        			reading = reading - 36000;
    			}
				cur_time = pros::millis();
				float power = wall_pid.cycle(20000, reading, (float)(cur_time - prev_time) / 1000.0, 10000, true);
				wall.move_velocity(power);
				if (fabs(20000 - reading) < wall_deadzone) {
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
    			if (reading > wall_cap) {
        			reading = reading - 36000;
    			}
				cur_time = pros::millis();
				float power = wall_pid.cycle(0, reading, (float)(cur_time - prev_time) / 1000.0, 10000, true);
				wall.move_velocity(power);
				if (fabs(0 - reading) < wall_deadzone) {
        			wall.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        			wall.move_velocity(0);
					wall_state = NORMAL;
				}				
				prev_time = cur_time;
				break;
			}
		}
	}
}


void color_sort() {
	int delay = 40;
	while (true) {
		// printf("%d\n", hook_dist.get());
		if (hook_dist.get() < 40 && wall_state == NORMAL) {
			double reading = ring_col.get_hue();
			printf("PASSED\n");
			switch (sort_state) {
				case RED:
				{
					if (reading < 70) {
						printf("THROWING ring %f\n", reading);
						sort_state = SORTING;
						hooks.move_velocity(600);
						pros::delay(delay);
						hooks.move_velocity(-600);
						pros::delay(100);
						sort_state = RED;
					}
					break;
				}
				case BLUE:
					if (reading > 70) {
						printf("THROWING");
						sort_state = SORTING;
						hooks.move_velocity(600);
						pros::delay(delay);
						hooks.move_velocity(-600);
						pros::delay(100);
						sort_state = BLUE;
					}
					break;

			}
		}

		pros::delay(20);
	}
}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// llemu init
	pros::lcd::initialize();
	// reset motor encoders
	lmotor.tare_position();
	rmotor.tare_position();
	lmotor.set_gearing(pros::E_MOTOR_GEAR_600);
	rmotor.set_gearing(pros::E_MOTOR_GEAR_600);
	lmotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	rmotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	// wall rot reset
	wall_rot.reset_position();
	wall_rot.set_data_rate(5);
	// imu reset
	imu.reset(true);
	// color sort init
	pros::Task sorting(color_sort);
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
void autonomous() {
	bool r_b = true;
	int cur_time;
	int prev_time;
}

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
		left_motors.move(dir + turn_sensitivity*(float)turn);                      // Sets left motor voltage
		right_motors.move(dir - turn_sensitivity*(float)turn);                     // Sets right motor voltage

		// button logic
		// use toggle (on rising edge)
		if (controller.get_digital_new_press(DIGITAL_L1)) {
			mogo.retract();
		} else if (controller.get_digital_new_press(DIGITAL_L2)) {
			mogo.extend();
		}
		if (!(sort_state == SORTING)) {
			sort_state = OFF;
			if (controller.get_digital(DIGITAL_R1)) {
				roller.move_velocity(intake_vel);
				hooks.move_velocity(intake_vel);
			} 
			else if (controller.get_digital(DIGITAL_R2)) {
				roller.move_velocity(-intake_vel);		
				hooks.move_velocity(-intake_vel);
			}
			else if (controller.get_digital(DIGITAL_B)) {
				hooks.move_velocity(400);
				roller.move_velocity(600);
				sort_state = BLUE;
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

		if (controller.get_digital_new_press(DIGITAL_Y)) {
			doinker.toggle();
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
				if (hook_dist.get() < 40) {
					hooks.move_velocity(0);
					wall_state = STOPPED;
				}
				break;
			}
			case STOPPED:
			{
				pros::lcd::print(6, "STOPPED");
				int reading = wall_rot.get_position() % 36000;
    			if (reading > wall_cap) {
        			reading = reading - 36000;
    			}
				cur_time = pros::millis();
				float power = wall_pid.cycle(7700, reading, (float)(cur_time - prev_time) / 1000.0, 10000, true);
				wall.move_velocity(power);
				if (fabs(7700 - reading) < wall_deadzone) {
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
    			if (reading > wall_cap) {
        			reading = reading - 36000;
    			}
				cur_time = pros::millis();
				float power = wall_pid.cycle(20000, reading, (float)(cur_time - prev_time) / 1000.0, 10000, true);
				wall.move_velocity(power);
				if (fabs(20000 - reading) < wall_deadzone) {
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
    			if (reading > wall_cap) {
        			reading = reading - 36000;
    			}
				cur_time = pros::millis();
				float power = wall_pid.cycle(0, reading, (float)(cur_time - prev_time) / 1000.0, 10000, true);
				wall.move_velocity(power);
				if (fabs(0 - reading) < wall_deadzone) {
        			wall.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        			wall.move_velocity(0);
					wall_state = NORMAL;
				}				
				prev_time = cur_time;
				break;
			}
		}

		printf("%f\n", ring_col.get_hue());
		printf("%d\n", hook_dist.get());

		// std::vector<double> lm = left_motors.get_position_all();
		// std::vector<double> lr = right_motors.get_position_all();
		pros::lcd::print(0, "Left: %f", lmotor.get_position());
		pros::lcd::print(1, "Right: %f", rmotor.get_position());
		pros::lcd::print(2, "Heading: %f", imu.get_heading());
		pros::delay(20);                               // Run for 20 ms then update
	}
}