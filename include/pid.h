class PID {
    private:
        float kP;
        float kI;
        float kD;

        float error;
        float prev_error;
        float sum;
        float delta;
        float prev_output;
    public:
        PID(float init_kP, float init_kI, float init_kD);
        float cycle(float reference, float reading, float delta_time, float max_slew, bool log=false);
        void reset();
};