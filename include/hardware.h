#ifndef _HRDWARE_
#define _HRDWARE_

#include <Arduino.h>
#include <dcpwm.h>       // PWM
#include <ESP_Counter.h> // Encoder
#include <AutoPID.h>     // PID
#include <Eigen.h>       // Linear math
#include <Eigen/QR>      // Calls inverse, determinant, LU decomp., etc.
#include "parameter.h"
// Hardware
static ESP_Counter WheelEncoder[NumMotors];      // Hardware-Encoder-Units
static DCPWM MotorPWM[NumMotors];                // Hardware-PWM-Units
static AutoPID *speedController[NumMotors];      // PID-Units
static pidParam speedControllerParam[NumMotors]; // PID-Parameter

#endif