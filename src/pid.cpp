#include "pid.h"
#include <algorithm>

PID::PID(float init_kP, float init_kI, float init_kD) {
    kP = init_kP;
    kI = init_kI;
    kD = init_kD;

    error = 0;
    sum = 0;
    delta = 0;
    prev_error = 0;
    prev_output = 0;
}

float PID::cycle(float reference, float reading, float delta_time, float max_slew, bool log) {
    if (delta_time <= 0.0) {
        delta_time = 0.001;
    }
    error = reference - reading;
    delta = (error - prev_error) / delta_time;
    if (kI != 0.0) {
        sum += error * delta_time;
    }
    prev_error = error;

    float out = kP * error + kI * sum + kD * delta;
    if (out > prev_output) {
        out = std::min(prev_output + max_slew, out);
    } else {
        out = std::max(prev_output - max_slew, out);
    }
    if (log) {
        printf("Reading: %f ", reading);
        printf("Ref: %f ", reference);
        printf("Error: %f ", error);
        printf("Delta: %f ", delta);
        // printf("Sum: %f\n", sum);
        printf("Time: %f ", delta_time);
        printf("Prev Out: %f ", prev_output);
        printf("Out: %f\n", out);
    }
    prev_output = out;
    return out;
}

void PID::reset() {
    error = 0;
    sum = 0;
    delta = 0;
    prev_error = 0;
    prev_output = 0;
}

void PID::set_prev(float nset) {
    prev_error = nset;
}