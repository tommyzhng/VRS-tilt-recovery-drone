#include "vrs_failsafe_controller.hpp"

void VrsFailsafeController::ThrottleController()
{

}

void VrsFailsafeController::ServoController()
{

}

void VrsFailsafeController::EstimateVRS()
{   
    // get response from service
    // satisfy some critereons to change curState to vrsDetected
    if (curLocalPosition_(2) > 0.28 * vh) {
        
    }

}