#include "util.h" 

// Error function in case of unhandeld ros-error
void error_loop()
{
  while (1)
  {
    Serial.println("ERR");
    delay(100);
  }
}

// Convert Eulerdegrees to quaternion
const void euler_to_quat(float x, float y, float z, double *q)
{
  float c1 = cos(y / 2.0);
  float c2 = cos(z / 2.0);
  float c3 = cos(x / 2.0);

  float s1 = sin(y / 2.0);
  float s2 = sin(z / 2.0);
  float s3 = sin(x / 2.0);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

// checks the direction of the encoder
int checkEncoderDirection(ESP_Counter &encoder)
{
  int count = encoder.getCount();
  delay(500);
  int count2 = encoder.getCount();
  if (count2 > count)
  {
    return 1;
  }
  else if (count2 < count)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

// Rotate 2D-Vector
const void rotate2D(float r, double &x, double &y)
{
  double temp = x * cos(r) - y * sin(r);
  y = x * sin(r) + y * cos(r);
  x = temp;
}

// Print content of Eigen::MatrixXd
void print_mtxd(const Eigen::MatrixXd &X)
{
  int i, j, nrow, ncol;

  nrow = X.rows();
  ncol = X.cols();

  Serial.print("nrow: ");
  Serial.println(nrow);
  Serial.print("ncol: ");
  Serial.println(ncol);
  Serial.println();

  for (i = 0; i < nrow; i++)
  {
    for (j = 0; j < ncol; j++)
    {
      Serial.print(X(i, j), 6); // print 6 decimal places
      Serial.print(", ");
    }
    Serial.println();
  }
  Serial.println();
}
