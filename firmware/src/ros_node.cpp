#include "ros_node.h"

rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t timer;

rcl_publisher_t motor_vel_pub, motor_pos_pub, stepper_pos_pub;
rcl_subscription_t motor_cmd_vel_sub, motor_cmd_pos_sub, servo_sub, led_sub;
rcl_service_t reset_pos_srv;

std_msgs__msg__Float64MultiArray cmd_vel_msg, cmd_pos_msg, vel_msg, pos_msg, stepper_pos_msg;
std_msgs__msg__Int32 servo_msg;
std_msgs__msg__String string_msg;

std_srvs__srv__Empty_Request reset_req;
std_srvs__srv__Empty_Response reset_res;

void setup_ros_node() {
  // Transport
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Init Node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

  // PUBLISHER
  RCCHECK(rclc_publisher_init_default(
    &stepper_pos_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/stepper/pos"
  ));

  RCCHECK(rclc_publisher_init_default(
    &motor_vel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/motor/vel"
  ));

  RCCHECK(rclc_publisher_init_default(
    &motor_pos_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/motor/pos"
  ));

  // SUBSCRIBER
  RCCHECK(rclc_subscription_init_default(
    &led_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/led_cmd"
  ));

  RCCHECK(rclc_subscription_init_default(
    &servo_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/servo_cmd"
  ));

  RCCHECK(rclc_subscription_init_default(
    &motor_cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/motor/cmd_vel"
  ));

  RCCHECK(rclc_subscription_init_default(
    &motor_cmd_pos_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/motor/cmd_pos"
  ));

  // SERVICE
  RCCHECK(rclc_service_init_default(
    &reset_pos_srv,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
    "/reset_pos"
  ));

  // Memory allocation
  allocate_array_msg(&cmd_vel_msg, 2);
  allocate_array_msg(&cmd_pos_msg, 2);

  allocate_array_msg(&vel_msg, 2);
  vel_msg.data.size = 2;
  allocate_array_msg(&pos_msg, 2);
  pos_msg.data.size = 2;
  allocate_array_msg(&stepper_pos_msg, 2);
  stepper_pos_msg.data.size = 2;

  // String allocation
  string_msg.data.capacity = MAX_STRING_LEN;
  string_msg.data.size = 0;
  string_msg.data.data = (char*) malloc(string_msg.data.capacity * sizeof(char));

  // TIMER
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // EXECUTOR
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));

  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &led_sub, &string_msg, &led_callback, ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &motor_cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &motor_cmd_pos_sub, &cmd_pos_msg, &cmd_pos_callback, ON_NEW_DATA
  )); 

  RCCHECK(rclc_executor_add_service(
    &executor, &reset_pos_srv, &reset_req, &reset_res, &reset_pos_callback
  ));
}

void spin_ros_node(){
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);  }
}

// CALLBACKS
void cmd_vel_callback(const void * msgin) {
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  
  if (msg->data.size >= 2) {
    stateL.targetVel = (long)msg->data.data[0];
    stateR.targetVel = (long)msg->data.data[1];
  }
}

void cmd_pos_callback(const void * msgin) {
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  
  if (msg->data.size >= 2) {
    stateL.targetPos = (long)msg->data.data[0];
    stateR.targetPos = (long)msg->data.data[1];
  }
}

void led_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  // If we receive "ON", turn LED on. If "OFF", turn it off.
  if (strcmp(msg->data.data, "ON") == 0) {
      blinking = true;
  } else if (strcmp(msg->data.data, "OFF") == 0) {
      blinking = false;
  }
}

void servo_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  if (msg->data >= 0 && msg->data <= 180) {
    servo_angle = msg->data;
    myServo.write(servo_angle);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Blink LED
    if (blinking) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
    }
    else {
      digitalWrite(LED_PIN, LOW); 
    }

    // Publish motor state
    pos_msg.data.data[0] = (double)stateL.currentPos;
    pos_msg.data.data[1] = (double)stateR.currentPos;

    vel_msg.data.data[0] = (double)stateL.currentVel;
    vel_msg.data.data[1] = (double)stateR.currentVel;

    stepper_pos_msg.data.data[0] = (double)stepperLeftPos;
    stepper_pos_msg.data.data[1] = (double)stepperRightPos;

    RCSOFTCHECK(rcl_publish(&motor_pos_pub, &pos_msg, NULL));
    RCSOFTCHECK(rcl_publish(&motor_vel_pub, &vel_msg, NULL));
    RCSOFTCHECK(rcl_publish(&stepper_pos_pub, &stepper_pos_msg, NULL));
  }
}

void reset_pos_callback(const void * req, void * res){
  stateL.currentPos = 0;
  stateR.currentPos = 0;
  stepperLeftPos = 0;
  stepperRightPos = 0;

  stateL.targetPos = 0;
  stateR.targetPos = 0;

  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
}

// Helper to allocate memory for Array messages
void allocate_array_msg(std_msgs__msg__Float64MultiArray * msg, size_t size) {
    msg->data.capacity = size;
    msg->data.size = 0; // Input starts empty
    msg->data.data = (double*) malloc(msg->data.capacity * sizeof(double));
}