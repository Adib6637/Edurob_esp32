#ifndef _VARIABLES_
#define _VARIABLES_

#include <Arduino.h>
#include "parameter.h"
#include <Eigen.h> // Linear math
#include <Eigen/QR> // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;   // Eigen related statement; simplifies syntax for declaration of matrices

// Variables
int64_t encoderOld[NumMotors];       // Last encoder values
int64_t encoderNew[NumMotors];       // Current encoder values
const int windowSize = 5;            // Number of values in moving average window
float inputs[NumMotors][windowSize]; // Input values for moving average
float inputsum[NumMotors];           // Sum of values for moving average
int windowIndex = 0;                 // Currently accessed cell of inputs[motornum][x];

// Setpoints
static double setpointSpeed[NumMotors]; // Motorspeed setpoints

// Matrix
MatrixXd kinematik(4, 3);    // Kinematics matrix for differential
MatrixXd kinematikInv(3, 4); // Inverse Kinematics matrix for differential
Vector3d robotSpeedSetpoint; // Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector3d robotSpeed;         // Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector3d robotSpeedMax;      // Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector3d robotSpeedAcc;      // Vector with translationional and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector4d wheelSpeedSetpoint; // Wheel speeds in rad/s
Vector3d robotOdom;          // Odometry Position
Vector4d robotWheelSpeed;    // Current wheel speeds
Vector3d robotOdomSpeed;     // Odometry Speed
double rosQuaternion[4];     // Quaternion for ROS transform message

#endif