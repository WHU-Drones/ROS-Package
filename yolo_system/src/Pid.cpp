#include "yolo_system/Pid.hpp"

Pid::Pid() :kp_(0), ki_(0), kd_(0), target_(0), actual_(0), integral_(0)
{
    e_ = target_ - actual_;
    e_pre_ = e_;
}
Pid::Pid(float p, float i, float d) : kp_(p), ki_(i), kd_(d), target_(0), actual_(0), integral_(0)
{
    e_ = target_ - actual_;
    e_pre_ = e_;
}
float Pid::pid_core(float target, float actual)
{
    float u;
    target_ = target;
    actual_ = actual;
    e_ = target_ - actual_;
    integral_ += e_;
    u = kp_ * e_ +ki_*integral_+kd_*(e_-e_pre_);
    e_pre_ = e_;
    return u;
}
