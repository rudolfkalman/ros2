#include <Arduino.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
sensor_msgs__msg__Imu imu_msg;

#include <sensor_msgs/msg/joy.h>
sensor_msgs__msg__Joy recv_joy;
int32_t joy_button_memory[20];
float joy_axes_memory[20];
char joy_frameid_memory[20];

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_GIGA) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL) && !defined(ARDUINO_UNOR4_WIFI) && !defined(ARDUINO_OPTA)
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi 
#endif

#include <Adafruit_BNO08x.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define CHECK_LED_PIN 25

Adafruit_BNO08x bno08x;
#define bno08X_RESET -1;

sh2_SensorValue_t sensorValue;

float r = 0;
float i = 0;
float j = 0;
float k = 0;
  
float wx = 0;
float wy = 0;
float wz = 0;
  
float ax = 0;
float ay = 0;
float az = 0;

void UpdateSensor(const sh2_SensorValue_t &v) {
  switch (v.sensorId) {

    case SH2_GAME_ROTATION_VECTOR:
      r = v.un.rotationVector.real;
      j = -v.un.rotationVector.i;
      i =  v.un.rotationVector.j;
      k =  v.un.rotationVector.k;
      break;

    case SH2_GYROSCOPE_CALIBRATED:
      wy = v.un.gyroscope.x;
      wx = -v.un.gyroscope.y;
      wz = -v.un.gyroscope.z;
      break;

    case SH2_ACCELEROMETER:
      ax = -v.un.accelerometer.x;
      ay = -v.un.accelerometer.y;
      az =  v.un.accelerometer.z;
      break;
  }
}

bool IsAllReady(){
  return !isnan(r) &&
         !isnan(i) &&
         !isnan(j) &&
         !isnan(k) &&
         !isnan(wx) &&
         !isnan(wy) &&
         !isnan(wz) &&
         !isnan(ax) &&
         !isnan(ay) &&
         !isnan(az);
}

void GetSensor(){
  while(bno08x.getSensorEvent(&sensorValue)){
    UpdateSensor(sensorValue);
  }
  imu_msg.orientation.w = r;
  imu_msg.orientation.x = i;
  imu_msg.orientation.y = j;
  imu_msg.orientation.z = k;
  imu_msg.angular_velocity.x = wx;
  imu_msg.angular_velocity.y = wy;
  imu_msg.angular_velocity.z = wz;
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
}

rcl_publisher_t imu_publisher;
rmw_qos_profile_t imu_qos = rmw_qos_profile_sensor_data;

rcl_subscription_t joy_subscriber;
rmw_qos_profile_t joy_qos = rmw_qos_profile_sensor_data;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t esp_node;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

const int timeout_ms = 1000;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void joy_callback(const void * msgin){
  const sensor_msgs__msg__Joy * msg = (const sensor_msgs__msg__Joy *)msgin;
  if(msg->buttons.data[0] == 1){
    digitalWrite(CHECK_LED_PIN, HIGH);
  }
  else{
    digitalWrite(CHECK_LED_PIN, LOW);
  }
}

unsigned long last_publish_time = 0;
const unsigned long PUBLISH_INTERVAL = 50;

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CHECK_LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("BNO08x not detected");
    while (1)
      delay(10);
  }
  Serial.println("BNO08x detected!");
 
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) {
    Serial.println("Failed to enable game rotation vector");
    while (1)
      delay(10);
  }
  Serial.println("Game Rotation Vector Report Enabled!");

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
    Serial.println("Failed to enable gyroscope report");
    while (1)
      delay(10);
  }
  Serial.println("Gyroscope Report Enabled!");

  if (!bno08x.enableReport(SH2_ACCELEROMETER, 10000)) {
    Serial.println("Failed to enable linear_acceleration report");
    while (1)
      delay(10);
  }
  Serial.println("Accelerometer Report Enabled!");

  //set_microros_transports();

  set_microros_wifi_transports("Synology-home", "Wi.LL'-ad6es4", "192.168.1.16", 8888);
  delay(2000);

  rmw_uros_sync_session(timeout_ms);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&esp_node, "esp_node", "", &support));

  sensor_msgs__msg__Joy__init(&recv_joy);

  recv_joy.buttons.data = joy_button_memory;
  recv_joy.buttons.capacity = 20;
  recv_joy.buttons.size = 0;

  recv_joy.axes.data = joy_axes_memory;
  recv_joy.axes.capacity = 20;
  recv_joy.axes.size = 0;

  recv_joy.header.frame_id.data = joy_frameid_memory;
  recv_joy.header.frame_id.capacity = 20;
  recv_joy.header.frame_id.size = 0;

  // create publisher
  /*RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &esp_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu"));*/

  //imu_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  //imu_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  imu_qos.depth = 1;

  RCCHECK(rclc_publisher_init(
    &imu_publisher,
    &esp_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu",
    &imu_qos));


  /*RCCHECK(rclc_subscription_init_default(
    &joy_subscriber,
    &esp_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "/joy"));*/
  
  joy_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  joy_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  joy_qos.depth = 1;

  RCCHECK(rclc_subscription_init(
    &joy_subscriber,
    &esp_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "/joy",
    &joy_qos));

  String frame_id = "imu";
  rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, frame_id.c_str());

  imu_msg.orientation_covariance[0] = 0.00001;
  imu_msg.orientation_covariance[4] = 0.00001;
  imu_msg.orientation_covariance[8] = 0.00001;

  imu_msg.angular_velocity_covariance[0] = 0.00001;
  imu_msg.angular_velocity_covariance[4] = 0.00001;
  imu_msg.angular_velocity_covariance[8] = 0.00001;

  imu_msg.linear_acceleration_covariance[0] = 0.00001;
  imu_msg.linear_acceleration_covariance[4] = 0.00001;
  imu_msg.linear_acceleration_covariance[8] = 0.00001;

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &joy_subscriber, &recv_joy, &joy_callback, ON_NEW_DATA));
}

void loop() {
  GetSensor();
  if(millis() - last_publish_time >= PUBLISH_INTERVAL){
    int64_t time_ns = rmw_uros_epoch_nanos();
    imu_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000);
    imu_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    last_publish_time = millis();
  }

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  vTaskDelay(0);
}
