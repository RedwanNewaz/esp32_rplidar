#pragma once 
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/laser_scan.h>
#include "rplidar.h"

#define LED_PIN 2
#define LOOP_DELAY 50 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define DEG2RAD(x) ((x)*M_PI/180.)

static rcl_publisher_t publisher;
static rcl_subscription_t subscriber;
static geometry_msgs__msg__Twist cmd_msg;

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;
static const char * sub_topic_name = "cmd_vel";
static const char * pub_topic_name = "rplidar/scan";
static rplidarHandler *rplidar = nullptr; 

// Constants for the car type and function
static const byte HEAD = 0xFF;        // refer to yahboom rosmater_lib
static const byte DEVICE_ID = 0xFC;   // refer to yahboom rosmater_lib
static const byte FUNC_MOTION = 0x12; // refer to yahboom rosmater_lib
static const byte CAR_TYPE = 1;    // refer to yahboom rosmater_lib
static const byte COMPLEMENT = 257 - DEVICE_ID;;  // refer to yahboom rosmater_lib

// Get message type support
static const rosidl_message_type_support_t * sub_type_support =
ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);

static const rosidl_message_type_support_t * pub_type_support =
ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan);

inline void error_loop(){
  while(1){
    digitalWrite(LED_PIN, LOW);
    delay(LOOP_DELAY);
  }
}

namespace airlab{

    class Node{

    public:
        Node();

        void init(rplidarHandler *lidar=nullptr, const char* nodeName = "microros_platformio_node");

        void loop();


    protected:
        static void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

        static void subscription_callback(const void * msgin);

        static void set_lidar_scan_msg();

        static void set_car_motion(float v_x, float v_y, float v_z); 

    private:
        rclc_executor_t executor_pub, executor_sub;

    };

    
}