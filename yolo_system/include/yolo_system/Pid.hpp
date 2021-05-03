#pragma once
class Pid
{
    private:
        float kp_;
        float ki_;
        float kd_;
        float target_;
        float actual_;
        float e_;
        float e_pre_;
        float integral_;
    public:
        Pid();
        ~Pid() {};
        Pid(float p, float i, float d);
        float pid_core(float target, float actual);
};