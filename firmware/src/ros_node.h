#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include "config.h"
#include "stepper.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#include <std_srvs/srv/empty.h>

// ROS ENTITIES
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_timer_t timer;

extern rcl_publisher_t motor_vel_pub, motor_pos_pub;
extern rcl_subscription_t motor_cmd_vel_sub, motor_cmd_pos_sub, servo_sub, led_sub;
extern rcl_service_t reset_pos_srv;

extern std_msgs__msg__Float64MultiArray cmd_vel_msg, cmd_pos_msg, vel_msg, pos_msg;
extern std_msgs__msg__Int32 servo_msg;
extern std_msgs__msg__String string_msg;

extern std_srvs__srv__Empty_Request reset_req;
extern std_srvs__srv__Empty_Response reset_res;

// MACROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// FUNCTION PROTOTYPES
void setup_ros_node();
void spin_ros_node();
void error_loop();

// Callbacks
void cmd_vel_callback(const void * msgin);
void cmd_pos_callback(const void * msgin);
void servo_callback(const void * msgin);
void led_callback(const void * msgin);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void reset_pos_callback(const void * req, void * res);

// Helper
void allocate_array_msg(std_msgs__msg__Float64MultiArray * msg, size_t size);

#endif