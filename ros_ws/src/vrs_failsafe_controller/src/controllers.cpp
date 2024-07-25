#include "vrs_failsafe_controller.hpp"

void VrsFailsafeController::ThrottleController()
{
    // PID gains
    float kp = 0.1;
    float ki = 0.1;
    float kd = 0.1;
    float dt = 0.1;
    float max = 1.0;
    float min = 0.0;
    float integral = 0.0;
    float derivative = 0.0;
    float error = 0.0;
    float last_error = 0.0;
    float output = 0.0;
    PubPositionTarget(0, 0);
}