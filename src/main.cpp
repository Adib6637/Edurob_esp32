#include <Arduino.h>
#include <WiFi.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "math.h"

#include <dcpwm.h>       // PWM
#include <ESP_Counter.h> // Encoder
#include <AutoPID.h>     // PID Library https://github.com/r-downing/AutoPID
#include <Eigen.h>       // Linear math
#include <Eigen/QR>      // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;   // Eigen related statement; simplifies syntax for declaration of matrices

// Project specific headers
#include "parameter.h"
#include "kinematik.h"
#include "util.h"
#include "hardware.h"
// Variables // Setpoints // Matrix
#include "variables.h"
#include "connect.h"

// ROS2
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>// ROS Messages
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }
#define RX_PIN 1
#define TX_PIN 2
#define DEBUG 1
#define PRINT_DEBUG(statement)  if (DEBUG) { statement; }
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
geometry_msgs__msg__TransformStamped tfData[3];
geometry_msgs__msg__TransformStamped tfStaticData[3];
rcl_subscription_t twistSubscriber;
geometry_msgs__msg__Twist twistMessage;
tf2_msgs__msg__TFMessage messageTf;
rcl_publisher_t publisherTf;
tf2_msgs__msg__TFMessage messageTfStatic;
rcl_publisher_t publisherTfStatic;
rcl_publisher_t publisherSpeed;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rcl_subscription_t subscriber;
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void subscription_callback(const void * msgin);
void setTfData();
void setTfStaticData();
void check_wifi();

// Init Encoder- and PWM-Units
bool initHardware()
{
  pinMode(EnablePIN, OUTPUT);   // Motor enable signal
  digitalWrite(EnablePIN, LOW); // Disable motors
  for (int i = 0; i < NumMotors; i++)
  {
    if (!WheelEncoder[i].init(i, Enc_A[i], Enc_B[i]))
    {
      return false;
    }
  }

  int currentChannel = 0;
  for (int i = 0; i < NumMotors; i++)
  {
    if (!MotorPWM[i].init(30000, 10, currentChannel, currentChannel + 1, PWM_A[i], PWM_B[i]))
    {
      return false;
    }
    MotorPWM[i].setPWM(0); // Set initial speed to 0
    currentChannel += 2;
  }
  digitalWrite(EnablePIN, HIGH); // Enable motors
  for (int i = 0; i < NumMotors; i++)
  {
    MotorPWM[i].setPWM(100 * MotorDir[i]); // Start Motor
    EncoderDir[i] = checkEncoderDirection(WheelEncoder[i]);
    MotorPWM[i].setPWM(0);
  }
  delay(1000); // Wait for Encoder to stabilize
  for (int i = 0; i < NumMotors; i++)
  {
    WheelEncoder[i].resetCounter();
  }

  return true;
}
void initPID()
{
  for (int i = 0; i < NumMotors; i++)
  {
    speedControllerParam[i].input = 0;
    speedControllerParam[i].output = 0;
    speedControllerParam[i].setpoint = 0;
    speedControllerParam[i].p = 0.04;
    speedControllerParam[i].i = 0.08;
    speedControllerParam[i].d = 0.0;
    speedControllerParam[i].sampleTimeMs = sampleTime;
    speedControllerParam[i].outMin = -100;
    speedControllerParam[i].outMax = 100;

    speedController[i] = new AutoPID(&speedControllerParam[i].input, &speedControllerParam[i].setpoint, &speedControllerParam[i].output, speedControllerParam[i].outMin, speedControllerParam[i].outMax, speedControllerParam[i].p, speedControllerParam[i].i, speedControllerParam[i].d);
    speedController[i]->setTimeStep(speedControllerParam[i].sampleTimeMs);
    speedController[i]->setBangBang(0, 0); // Disable BangBang-Controll
  }
}
void initMatrix()
{

  mecanum_matrix(kinematik, kinematikInv, l1, l2); // sets the kinematik and kinematikInv to the desired values
  robotSpeedSetpoint << 0.0,
      0.0,
      0.0;

  robotSpeedAcc << maxTAccel, maxTAccel, maxRAccel;
  robotSpeedMax << maxTSpeed, maxTSpeed, maxRSpeed;
  wheelSpeedSetpoint = (1 / wheelRadius) * kinematik * robotSpeedSetpoint;
}

// Speed controller task
void speedControllerTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(sampleTime);

  for (int i = 0; i < NumMotors; i++)
  {
    inputsum[i] = 0;
  }

  for (int i = 0; i < NumMotors; i++)
  {
    for (int k = 0; k < windowSize; k++)
    {
      inputs[i][k] = 0;
    }
  }

  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    for (int i = 0; i < 3; i++)
    {
      if (robotSpeed[i] > robotSpeedSetpoint[i])
      {
        robotSpeed[i] = robotSpeed[i] - (robotSpeedAcc[i] * 0.005);
      }
      else if (robotSpeed[i] < robotSpeedSetpoint[i])
      {
        robotSpeed[i] = robotSpeed[i] + (robotSpeedAcc[i] * 0.005);
      }

      if (robotSpeed[i] > robotSpeedMax[i])
      {
        robotSpeed[i] = robotSpeedMax[i];
      }
      else if (robotSpeed[i] < -robotSpeedMax[i])
      {
        robotSpeed[i] = -robotSpeedMax[i];
      }

      if (robotSpeed[i] < 0.000001 && robotSpeed[i] > -0.000001)
      {
        robotSpeed[i] = 0;
      }
    }
    wheelSpeedSetpoint = (1 / wheelRadius) * kinematik * robotSpeed;
    for (int i = 0; i < NumMotors; i++)
    {
      if (abs(wheelSpeedSetpoint[i]) < 0.15)
      {
        wheelSpeedSetpoint[i] = 0;
      }
    }

    for (int i = 0; i < NumMotors; i++)
    {
      speedControllerParam[i].setpoint = wheelSpeedSetpoint(i, 0);
      encoderNew[i] = EncoderDir[i] * WheelEncoder[i].getCount();

      inputsum[i] -= inputs[i][windowIndex];
      inputs[i][windowIndex] = ((((encoderNew[i] - encoderOld[i]) * incrementsToRad) / sampleTime) * 1000);
      inputsum[i] += inputs[i][windowIndex];

      speedControllerParam[i].input = inputsum[i] / windowSize; // Filtered

      speedController[i]->run();

      MotorPWM[i].setPWM(MotorDir[i] * speedControllerParam[i].output * Rad2PWM); // Raw
      encoderOld[i] = encoderNew[i];
    }
    for (int i = 0; i < NumMotors; i++)
    {
      robotWheelSpeed[i] = inputs[i][windowIndex];
    }

    robotOdomSpeed = (wheelRadius * kinematikInv * robotWheelSpeed);

    robotOdomSpeed = robot_vel_to_world_vel(robotOdom[2], robotOdomSpeed); // Conversion Velocity in robotcoordinates to velocity in worldcoordinates

    robotOdom[0] = robotOdomSpeed[0] * sampleTime / 1000 + robotOdom[0];
    robotOdom[1] = robotOdomSpeed[1] * sampleTime / 1000 + robotOdom[1];
    robotOdom[2] = robotOdomSpeed[2] * sampleTime / 1000 + robotOdom[2];
    windowIndex = (windowIndex + 1) % windowSize;
  }
}

// Logger task
void loggerTask(void *pvParameters)
{
  while (true)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    if (false){
      Serial.print("Odometry: Xt= ");
      Serial.print(robotOdom[0]); // current x-position in meters
      Serial.print(", Yt= ");
      Serial.print(robotOdom[1]); // current y-position in meters
      Serial.print(", Zr= ");
      Serial.println(robotOdom[2]); // current z-rotation in radians

    }
    
  }
}

// Setup
void setup()
{
  Serial.begin(115200);
  initHardware();
  initPID();
  initMatrix();

  xTaskCreate(
      speedControllerTask,   /* Task function. */
      "speedControllerTask", /* String with name of task. */
      4096,                /* Stack size in bytes. */
      NULL,                  /* Parameter passed as input of the task */
      5,                     /* Priority of the task. */
      NULL);                 /* Task handle. */
  xTaskCreate(
      loggerTask,   /* Task function. */
      "loggerTask", /* String with name of task. */
      2048,        /* Stack size in bytes. */
      NULL,         /* Parameter passed as input of the task */
      1,            /* Priority of the task. */
      NULL);        /* Task handle. */

  // ROS2
  set_microros_wifi_transports(WIFI_SSD, WIFI_PASS, HOST_IP, HOST_PORT);
  //Serial.print(WiFi.localIP());
  allocator = rcl_get_default_allocator();
  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // Create node
  RCCHECK(rclc_node_init_default(&node, "Edurob_nv", "", &support));

  // create tf publisher
  //RCCHECK(rclc_publisher_init_default(
  //  &publisherTf,
  //  &node,
  //  ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
  //  "/tf"));

  // create tf publisher static
  // Set publisher QoS
  //rmw_qos_profile_t rmw_qos_profile_tfstatic = {RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10U, RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, {0ULL, 0ULL}, {0ULL, 0ULL}, RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT, {0ULL, 0ULL}, false};
  //RCCHECK(rclc_publisher_init(
  //    &publisherTfStatic,
  //    &node,
  //    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
  //    "/tf_static",
  //    &rmw_qos_profile_tfstatic));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &twistSubscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twistSubscriber, &twistMessage, &subscription_callback, ON_NEW_DATA));

  // Create publisher (foward cmd_vel) for testing
  //RCCHECK(rclc_publisher_init_best_effort(
  //  &publisher,
  //  &node,
  //  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  //  "edurob/pub_msg_test"));

  
  // Enable motors
  digitalWrite(EnablePIN, HIGH); // Enable motors
}

void loop()
{
  check_wifi(); 
  // #############-USER-CODE-START-#####################
  //double tx = 0.0, ty = 0.0, theta = 0.0;
  //robotSpeedSetpoint << tx, ty, theta;
  Serial.println("alive");
  // #############-USER-CODE-END-#####################
  rmw_uros_sync_session(100); // Synchronize time
  if (rmw_uros_epoch_synchronized())
  {
    setTfData();
    setTfStaticData();
    RCSOFTCHECK(rcl_publish(&publisherTf, &messageTf, NULL));
    RCSOFTCHECK(rcl_publish(&publisherTf, &messageTfStatic, NULL));
  }
  RCCHECK(rclc_executor_spin_some(&executor, 0));
  vTaskDelay(10);
}

void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msga = (const geometry_msgs__msg__Twist *)msgin;
  robotSpeedSetpoint << msga->linear.x, msga->linear.y, msga->angular.z;
  RCSOFTCHECK(rcl_publish(&publisher, msga, NULL));
}

void  check_wifi(){
  if (WiFi.status() != WL_CONNECTED) {
    //Serial.println("Wi-Fi disconnected, trying to reconnect...");
    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(1000);
    }
    //Serial.println("Reconnected to Wi-Fi.");
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

// Set tf data
void setTfData()
{
  tfData[0].header.frame_id =
      micro_ros_string_utilities_set(tfData[0].header.frame_id, "/odom");
  tfData[0].child_frame_id =
      micro_ros_string_utilities_set(tfData[0].child_frame_id, "/base_link");
  tfData[0].transform.translation.x = robotOdom[0];
  tfData[0].transform.translation.y = robotOdom[1];
  euler_to_quat(0, 0, robotOdom[2], rosQuaternion);
  tfData[0].transform.rotation.x = (double)rosQuaternion[1];
  tfData[0].transform.rotation.y = (double)rosQuaternion[2];
  tfData[0].transform.rotation.z = (double)rosQuaternion[3];
  tfData[0].transform.rotation.w = (double)rosQuaternion[0];
  tfData[0].header.stamp.nanosec = rmw_uros_epoch_millis() * 1000;
  tfData[0].header.stamp.sec = rmw_uros_epoch_millis() / 1000;

  messageTf.transforms.size = 1;
  messageTf.transforms.data = tfData;
}

// Set static tf data
void setTfStaticData()
{
  tfStaticData[0].header.frame_id =
      micro_ros_string_utilities_set(tfStaticData[0].header.frame_id, "/base_link");
  tfStaticData[0].child_frame_id =
      micro_ros_string_utilities_set(tfStaticData[0].child_frame_id, "/laser_link");
  tfStaticData[0].transform.translation.x = 0;
  tfStaticData[0].transform.translation.y = 0;
  tfStaticData[0].transform.translation.z = 0.08;
  tfStaticData[0].transform.rotation.x = 0;
  tfStaticData[0].transform.rotation.y = 0;
  tfStaticData[0].transform.rotation.z = 0;
  tfStaticData[0].transform.rotation.w = 1;
  tfStaticData[0].header.stamp.nanosec = rmw_uros_epoch_millis() * 1000;
  tfStaticData[0].header.stamp.sec = rmw_uros_epoch_millis() / 1000;

  messageTfStatic.transforms.size = 1;
  messageTfStatic.transforms.data = tfStaticData;
}