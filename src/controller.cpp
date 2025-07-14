#include "controller.h"



bool ArmRead()
{
    float InputArm = pulseIn(ARM_PIN, HIGH, 25000); // Read arm state with 25ms timeout
    if (InputArm > 1500) {
        return true; // Return true if armed
    } else {
        return false; // Return false if disarmed
    }

}