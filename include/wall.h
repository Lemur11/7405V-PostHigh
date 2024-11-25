#include "devices.h"
#include "pid.h"
#include <atomic>


class wall_mech {
    public:
        wall_mech();
        void set_target(int reference, bool blocking=false);
        int get_rotation();
        bool done = true;
    private:
        void update();
        int prev_time;
        int cur_time;
        PID wall_pid;
        pros::Task* wall_update;
        std::atomic<int> target{0};
};