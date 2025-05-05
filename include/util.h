#ifndef _UTIL_
#define _UTIL_

#include <Arduino.h>
#include <ESP_Counter.h> // Encoder
#include <Eigen.h>       // Linear math
#include <Eigen/QR>      // Calls inverse, determinant, LU decomp., etc.
#include "parameter.h"
#include "hardware.h"

// Error function in case of unhandeld ros-error
void error_loop();
// Convert Eulerdegrees to quaternion
const void euler_to_quat(float x, float y, float z, double *q);
// checks the direction of the encoder
int checkEncoderDirection(ESP_Counter &encoder);
// Rotate 2D-Vector
const void rotate2D(float r, double &x, double &y);
// Print content of Eigen::MatrixXd
void print_mtxd(const Eigen::MatrixXd &X);

#endif