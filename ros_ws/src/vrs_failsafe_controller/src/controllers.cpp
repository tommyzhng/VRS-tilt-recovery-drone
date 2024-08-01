#include "vrs_failsafe_controller.hpp"

/*  calculate error between target published by px4 and current measured by imu */
void VrsFailsafeController::CalculateTargetError()
{
    // store  quaternino error in temp var and convert to euler angles
    Eigen::Quaternionf q_err = targetAttitude_ * curAttitude_.conjugate();
    errorAttitudeEuler_ = q_err.toRotationMatrix().eulerAngles(0, 1, 2);
}

/*  open loop control for majority of recovery, then switch to closed loop to stabilize
    because response is unknown at this stage since there is assumed to be a random sine wave applied to thrust */
void VrsFailsafeController::ThrottleController()
{
    if (curLocalVelocity_(2) < -1) {
        PubThrust(1.0);
    }
}
/*  response should be based on stability of the drone, in terms of roll pitch and yaw
    more instability should result in more servo angle */
void VrsFailsafeController::ServoController()
{
    // initially set to 45 degree angle to disrupt the cyclic circulation
    // go back to 0 angle with criterion to get maximum thrust in the down direction
    if (curAccell_(2) <= 0){
        servoAngleSetpoint_ = 45;
    }
}

void VrsFailsafeController::EstimateVRS()
{   
    // satisfy some critereons to change curState to vrsDetected
    if (curLocalVelocity_(2) < -0.28 * vh) {  // Xin and Gao criterion
        if (sqrt(curLocalVelocity_(1)**2 + curLocalVelocity_(2)**2) < 0.7) {  // see if velocity magnitude is less than 0.7 m/s
            if (errorAttitudeEuler_(0) < 6.5 && errorAttitudeEuler_(1) < 5) {  // check if roll and pitch are within 5 degrees
                curState_ = "vrsFailsafe";
            }

        }
    }

}