#ifndef PID_H
#define PID_H

#include <math.h>

namespace pid {

class Controller
{
private:

  bool angle_; // True if we're controlling an angle [-pi, pi]
  double target_ = 0;
  double prev_error_ = 0;
  double integral_ = 0;
  double Kp_;
  double Ki_;
  double Kd_;

public:

  // Standard constructor
  Controller(bool angle, double Kp, double Ki, double Kd)
  {
    angle_ = angle;
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
  }

  // Intuitive constructor
  Controller(bool angle, double damping_ratio, double natural_frequency)
  {
    angle_ = angle;
    Kp_ = natural_frequency * natural_frequency * (1 + 2 * damping_ratio);
    Ki_ = natural_frequency * natural_frequency * natural_frequency;
    Kd_ = natural_frequency * (1 + 2 * damping_ratio);
  }

  // Set target
  void set_target(double target)
  {
    target_ = target;
    prev_error_ = 0;
    integral_ = 0;
  }

  // Run one calculation
  double calc(double state, double dt, double bias)
  {
    double error = target_ - state;

    if (angle_)
    {
      // Deal with discontinuity
      while (error < -M_PI)
      {
        error += 2 * M_PI;
      }
      while (error > M_PI)
      {
        error -= 2 * M_PI;
      }
    }

    integral_ = integral_ + (error * dt);
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    return Kp_ * error + Ki_ * integral_ + Kd_ * derivative + bias;
  }
};

} // namespace pid

#endif // PID_H