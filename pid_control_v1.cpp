#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <vector>
#include <cmath>
#include <utility>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <config.h>
#include <motor.h>
#include <Utilize.h>

#define RCCHECK(fn)                  \
  {                                  \
    rcl_ret_t temp_rc = fn;          \
    if ((temp_rc != RCL_RET_OK))     \
    {                                \
      rclErrorLoop();                \
    }                                \
  }
#define RCSOFTCHECK(fn)              \
  {                                  \
    rcl_ret_t temp_rc = fn;          \
    if ((temp_rc != RCL_RET_OK))     \
    {                                \
    }                                \
  }
#define EXECUTE_EVERY_N_MS(MS, X)          \
  do                                       \
  {                                        \
    static volatile int64_t init = -1;     \
    if (init == -1)                        \
    {                                      \
      init = uxr_millis();                 \
    }                                      \
    if (uxr_millis() - init > MS)          \
    {                                      \
      X;                                   \
      init = uxr_millis();                 \
    }                                      \
  } while (0)

// ##############################################

// ROS variables
rcl_publisher_t status_publisher;
std_msgs__msg__Int32 status_msg;
rcl_publisher_t encoder_publisher;
std_msgs__msg__Float32MultiArray encoder_msg;

// ---- Subscribers ----
rcl_subscription_t robot_velocity_subscriber;
rcl_subscription_t robot_direction_subscriber;

std_msgs__msg__Float32 robot_velocity_msg;
geometry_msgs__msg__Twist robot_direction_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t encoder_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
static int disconnect_count = 0;

// Motor Pins
#define MOTOR_L_IN1_1 19
#define MOTOR_L_IN2_1 18
#define MOTOR_L_IN1_2 23
#define MOTOR_L_IN2_2 22

#define MOTOR_R_IN1_1 27
#define MOTOR_R_IN2_1 26
#define MOTOR_R_IN1_2 33
#define MOTOR_R_IN2_2 32

// Encoder Pins
#define ENCODER_L1_A 16
#define ENCODER_L1_B 4
#define ENCODER_L2_A 5
#define ENCODER_L2_B 17

#define ENCODER_R1_A 35
#define ENCODER_R1_B 34
#define ENCODER_R2_A 39
#define ENCODER_R2_B 36

volatile long int encoderTicks[4];      // 0: L1, 1: L2, 2: R1, 3: R2
volatile long int lastEncoderTicks[4];
float rpm[4];                            // <<< ใช้ float ชัดๆ
unsigned long lastEncoderTime = 0;

float max_rpm = 330.0;   // RPM สูงสุดของมอเตอร์
float max_pwm = 255.0;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//------------------------------ < Function Prototype > ------------------------------//
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void flashLED(unsigned int);
struct timespec getTime();

void publishData();
void publishEncoderData();
void update_motors_from_cmd();

// encoder interrupts
void encoderTickL1A();
void encoderTickL1B();
void encoderTickL2A();
void encoderTickL2B();
void encoderTickR1A();
void encoderTickR1B();
void encoderTickR2A();
void encoderTickR2B();

void setupEncoders();
void calculateRPM();

// ============================ PID (แก้ใหม่ทั้งชุด) ============================

struct PIDState {
  double Kp, Ki, Kd;
  double error, last_error;
  double integrate;
  unsigned long last_ms;
};

// ปรับค่าได้ตามต้องการ
PIDState pidL{1.2, 0.0001, 0.005, 0, 0, 0, 0};
PIDState pidR{1.2, 0.0001, 0.005, 0, 0, 0, 0};

// คืนค่า PWM 0..255 ตลอด
double pid_update(PIDState &pid, float setpoint, float feedback)
{
  unsigned long now = millis();
  double dt = (pid.last_ms == 0) ? 0.01 : (now - pid.last_ms) / 1000.0;
  if (dt <= 0) dt = 0.01;

  pid.error = (double)setpoint - (double)feedback;
  pid.integrate += pid.error * dt;

  double d_error = (pid.error - pid.last_error) / dt;

  double u = pid.Kp * pid.error + pid.Ki * pid.integrate + pid.Kd * d_error;

  // คุมเป็นขนาด PWM (ทิศทางไปใส่ทีหลัง)
  u = constrain(u, 0.0, (double)max_pwm);

  pid.last_error = pid.error;
  pid.last_ms = now;

  return u;
}

// รับ signed pwm (-255..255) แล้วสั่ง IN1/IN2
void set_motor_pwm(int in1_pin, int in2_pin, int pwm_signed)
{
  int pwm = abs(pwm_signed);
  pwm = constrain(pwm, 0, 255);

  if (pwm_signed > 0) {
    analogWrite(in1_pin, pwm);
    analogWrite(in2_pin, 0);
  } else if (pwm_signed < 0) {
    analogWrite(in1_pin, 0);
    analogWrite(in2_pin, pwm);
  } else {
    analogWrite(in1_pin, 0);
    analogWrite(in2_pin, 0);
  }
}

//------------------------------ < Main > -------------------------------------//

void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  setupEncoders();

  pinMode(MOTOR_L_IN1_1, OUTPUT);
  pinMode(MOTOR_L_IN2_1, OUTPUT);
  pinMode(MOTOR_L_IN1_2, OUTPUT);
  pinMode(MOTOR_L_IN2_2, OUTPUT);
  pinMode(MOTOR_R_IN1_1, OUTPUT);
  pinMode(MOTOR_R_IN2_1, OUTPUT);
  pinMode(MOTOR_R_IN1_2, OUTPUT);
  pinMode(MOTOR_R_IN2_2, OUTPUT);

  analogWriteResolution(8);
  analogWriteFrequency(5000);
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  switch (state)
  {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000,
        state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT)
      {
        destroyEntities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED)
      {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(300));
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      disconnect_count = 0;
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}

//------------------------------ < Encoder Functions > -------------------------------------//

void setupEncoders()
{
  pinMode(ENCODER_L1_A, INPUT);
  pinMode(ENCODER_L1_B, INPUT);
  pinMode(ENCODER_L2_A, INPUT);
  pinMode(ENCODER_L2_B, INPUT);
  pinMode(ENCODER_R1_A, INPUT);
  pinMode(ENCODER_R1_B, INPUT);
  pinMode(ENCODER_R2_A, INPUT);
  pinMode(ENCODER_R2_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L1_A), encoderTickL1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L1_B), encoderTickL1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L2_A), encoderTickL2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L2_B), encoderTickL2B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R1_A), encoderTickR1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R1_B), encoderTickR1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R2_A), encoderTickR2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R2_B), encoderTickR2B, CHANGE);

  for (int i = 0; i < 4; i++) {
    encoderTicks[i] = 0;
    lastEncoderTicks[i] = 0;
    rpm[i] = 0.0f;
  }

  lastEncoderTime = millis();
}

void encoderTickL1A() { encoderTicks[0] += (digitalRead(ENCODER_L1_A) == digitalRead(ENCODER_L1_B)) ? 1 : -1; }
void encoderTickL1B() { encoderTicks[0] += (digitalRead(ENCODER_L1_A) != digitalRead(ENCODER_L1_B)) ? 1 : -1; }
void encoderTickL2A() { encoderTicks[1] += (digitalRead(ENCODER_L2_A) == digitalRead(ENCODER_L2_B)) ? 1 : -1; }
void encoderTickL2B() { encoderTicks[1] += (digitalRead(ENCODER_L2_A) != digitalRead(ENCODER_L2_B)) ? 1 : -1; }
void encoderTickR1A() { encoderTicks[2] += (digitalRead(ENCODER_R1_A) == digitalRead(ENCODER_R1_B)) ? 1 : -1; }
void encoderTickR1B() { encoderTicks[2] += (digitalRead(ENCODER_R1_A) != digitalRead(ENCODER_R1_B)) ? 1 : -1; }
void encoderTickR2A() { encoderTicks[3] += (digitalRead(ENCODER_R2_A) == digitalRead(ENCODER_R2_B)) ? 1 : -1; }
void encoderTickR2B() { encoderTicks[3] += (digitalRead(ENCODER_R2_A) != digitalRead(ENCODER_R2_B)) ? 1 : -1; }

void calculateRPM()
{
  unsigned long currentTime = millis();
  if (currentTime - lastEncoderTime >= 10) // 10ms
  {
    double timeDifference = (currentTime - lastEncoderTime) / 1000.0;

    for (int i = 0; i < 4; i++)
    {
      long tickDifference = encoderTicks[i] - lastEncoderTicks[i];

      // ปรับตาม encoder ของคุณ
      // pulses_per_rev = 90, gear = 12 ตามที่คุณคอมเมนต์ไว้
      double rpm_real = (tickDifference * 60.0) / (timeDifference * 90.0 * 12.0);

      // ทำเป็น feedback 0..255 (ใช้ abs เพราะทิศเอาจากคำสั่ง)
      double pwm_fb = (fabs(rpm_real) / max_rpm) * max_pwm;
      rpm[i] = (float)constrain(pwm_fb, 0.0, (double)max_pwm);

      lastEncoderTicks[i] = encoderTicks[i];
    }

    lastEncoderTime = currentTime;
  }
}

//------------------------------ < ROS Callbacks > -------------------------------------//

void robot_velocity_callback(const void * msgin)
{
  robot_velocity_msg.data = ((const std_msgs__msg__Float32*)msgin)->data;
  // ไม่สั่งมอเตอร์ตรงนี้ เพื่อให้คุมคาบคงที่ใน timer
}

void robot_direction_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist*)msgin;
  robot_direction_msg = *msg;
  // ไม่สั่งมอเตอร์ตรงนี้ เพื่อให้คุมคาบคงที่ใน timer
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    calculateRPM();          // อัปเดต feedback ก่อน
    update_motors_from_cmd();// PID + สั่ง PWM
    publishData();
  }
}

void encoderTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    publishEncoderData();
  }
}

// ============================ ใช้ PID ที่นี่จริง ============================
void update_motors_from_cmd()
{
  float dir_linear  = robot_direction_msg.linear.x;
  float dir_angular = robot_direction_msg.angular.z;
  float speed       = robot_velocity_msg.data;   // 0..1

  float v = 0.0f, omega = 0.0f;
  if (dir_linear != 0.0f) {
    v = dir_linear * speed;      // -1..1
    omega = 0.0f;
  } else if (dir_angular != 0.0f) {
    v = 0.0f;
    omega = dir_angular * speed; // -1..1
  }

  float left_cmd  = (v + omega)*2;   
  float right_cmd = (v - omega)*2;   

  // ทำ setpoint เป็น “ขนาด” 0..255
  float left_sp_mag  = fabs(left_cmd);
  float right_sp_mag = fabs(right_cmd);

  // feedback 0..255 (ของคุณทำไว้แล้ว)
  float left_fb  = (rpm[0] + rpm[1]) * 0.5f;
  float right_fb = (rpm[2] + rpm[3]) * 0.5f;

  int left_pwm  = (int)pid_update(pidL, left_sp_mag, left_fb);
  int right_pwm = (int)pid_update(pidR, right_sp_mag, right_fb);

  // ใส่ทิศทางตามคำสั่ง
  if (left_cmd < 0)  left_pwm  = -left_pwm;
  if (right_cmd < 0) right_pwm = -right_pwm;
  right_pwm = -right_pwm;  // เดี๋ยวคุยข้อถัดไป

  set_motor_pwm(MOTOR_L_IN1_1, MOTOR_L_IN2_1, left_pwm);
  set_motor_pwm(MOTOR_L_IN1_2, MOTOR_L_IN2_2, left_pwm);

  set_motor_pwm(MOTOR_R_IN1_1, MOTOR_R_IN2_1, right_pwm);
  set_motor_pwm(MOTOR_R_IN1_2, MOTOR_R_IN2_2, right_pwm);
}

//------------------------------ < ROS Entities > -------------------------------------//

bool createEntities()
{
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  RCCHECK(rclc_node_init_default(&node, "esp32", "", &support));

  RCCHECK(rclc_publisher_init_best_effort(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/motor_status"));

  RCCHECK(rclc_publisher_init_best_effort(
    &encoder_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/encoder_data"));

  encoder_msg.data.capacity = 8;
  encoder_msg.data.size = 8;
  encoder_msg.data.data = (float*)malloc(encoder_msg.data.capacity * sizeof(float));

  RCCHECK(rclc_subscription_init_default(
    &robot_velocity_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/robot_velocity"));

  RCCHECK(rclc_subscription_init_default(
    &robot_direction_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/robot_direction"));

  const unsigned int control_timeout = 10;  // 10ms
  RCCHECK(rclc_timer_init_default(
    &control_timer,
    &support,
    RCL_MS_TO_NS(control_timeout),
    controlCallback));

  RCCHECK(rclc_timer_init_default(
    &encoder_timer,
    &support,
    RCL_MS_TO_NS(100),
    encoderTimerCallback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &encoder_timer));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &robot_velocity_subscriber,
    &robot_velocity_msg,
    &robot_velocity_callback,
    ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &robot_direction_subscriber,
    &robot_direction_msg,
    &robot_direction_callback,
    ON_NEW_DATA));

  syncTime();
  return true;
}

bool destroyEntities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&status_publisher, &node);
  rcl_publisher_fini(&encoder_publisher, &node);
  rcl_subscription_fini(&robot_velocity_subscriber, &node);
  rcl_subscription_fini(&robot_direction_subscriber, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&control_timer);
  rcl_timer_fini(&encoder_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  free(encoder_msg.data.data);
  return true;
}

void publishData()
{
  float speed = robot_velocity_msg.data;
  status_msg.data = (fabs(speed) > 0.1f) ? 1 : 0;
  rcl_publish(&status_publisher, &status_msg, NULL);
}

void publishEncoderData()
{
  for (int i = 0; i < 4; i++) {
    encoder_msg.data.data[i] = rpm[i];  // 0..255
  }
  for (int i = 0; i < 4; i++) {
    encoder_msg.data.data[i + 4] = (float)encoderTicks[i];
  }
  rcl_publish(&encoder_publisher, &encoder_msg, NULL);
}

void syncTime()
{
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
  struct timespec tp = {0};
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}

void rclErrorLoop()
{
  ESP.restart();
  while (true)
  {
    flashLED(3);
  }
}

void flashLED(unsigned int n_times)
{
  delay(1000);
}
