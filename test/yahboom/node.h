#pragma once 
#include <micro_ros_platformio.h>
#include "credential.h"
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/twist.h>
#include "rplidar.h"
#include "motor_cmd.h"

#define LED_PIN 2
#define LOOP_DELAY 10 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


static rcl_publisher_t publisher;
static rcl_subscription_t subscriber;
static sensor_msgs__msg__LaserScan msg; 
static geometry_msgs__msg__Twist cmd_msg;

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rcl_timer_t timer;
const char * sub_topic_name = "cmd_vel";
const char * pub_topic_name = "rplidar/scan";
static rplidarHandler* rplidar_;

// Get message type support
const rosidl_message_type_support_t * sub_type_support =
ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);

const rosidl_message_type_support_t * pub_type_support =
ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan);


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, LOW);
    delay(LOOP_DELAY);
  }
}

namespace airlab{

    class Node{

    public:


        Node(rplidarHandler* rplidar, const char* nodeName = "microros_platformio_node")
        {
            rplidar_ = rplidar; 
            set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

            allocator = rcl_get_default_allocator();

            //create init_options
            RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

            // create node
            RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

            // create publisher
            RCCHECK(rclc_publisher_init_default(
                &publisher,
                &node,
                pub_type_support,
                pub_topic_name));

            
       
            // create subscriber a reliable subscriber
            RCCHECK(rclc_subscription_init_default(
            &subscriber, &node,
            sub_type_support, sub_topic_name));

            // create timer,
            RCCHECK(rclc_timer_init_default(
                &timer,
                &support,
                RCL_MS_TO_NS(LOOP_DELAY),
                timer_callback));
            

            // create executor
            RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
            RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

              // Add subscription to the executor
            RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
            RCCHECK(rclc_executor_add_subscription(
            &executor_sub, &subscriber, &cmd_msg,
            &subscription_callback, ON_NEW_DATA));

            // // msg.data = 0;
            // msg.data.data = new int[360];
            // msg.data.capacity = 360;
            // msg.data.size = 360;



            // float32 scan_time        # time between scans [seconds]
            msg.header.frame_id.data = "rplidar";

            const int data_size = 360;
            msg.ranges.data = new float[data_size]; 
            msg.ranges.size = data_size;
            msg.ranges.capacity = data_size;       // # range data [m] (Note: values < range_min or > range_max should be discarded)

           
            

        }


        void loop()  
        {
            RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(LOOP_DELAY)));
            RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(LOOP_DELAY)));
        }
    protected:

        static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
        {
            RCLC_UNUSED(last_call_time);
            if (timer != NULL) {

                if (rplidar_) {
                    if(rplidar_ ->is_measurement_ready() )
                    {
                        
                        msg.scan_time = 1.0 / rplidar_->get_fq();
                        digitalWrite(LED_PIN, !digitalRead(LED_PIN));


                        msg.angle_min = 2 * M_PI;      // start angle of the scan [rad]
                        msg.angle_max =  0;      // end angle of the scan [rad]
                        

                        msg.range_min = 0.15;        // minimum range value [m]
                        msg.range_max = 0.0;        // maximum range value [m]
                        
                        for(int i = 0; i < rplidar_->get_measurement_size(); ++i)
                        {
                            msg.ranges.data[i] = rplidar_->get_measurement(i, 1) /  1000;
                            msg.range_max = max(msg.range_max, msg.ranges.data[i]);
                            float theta = rplidar_->get_measurement(i, 0) / 180.0 * M_PI; 
                            msg.angle_min = min(msg.angle_min,  theta);
                            msg.angle_max = max(msg.angle_max, theta);
                        }

                        msg.angle_increment = (msg.angle_max - msg.angle_min) / rplidar_->get_measurement_size();

                        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
                        rplidar_->reset_measurement();
                    }
                }                
            }
        }

        static void subscription_callback(const void * msgin)
        {
            // Cast received message to used type
            const geometry_msgs__msg__Twist * cmd_msg_ptr = (const geometry_msgs__msg__Twist *)msgin;
            float vx = cmd_msg_ptr->linear.x;
            float vy = cmd_msg_ptr->linear.y;
            float vz = cmd_msg_ptr->angular.z;
            set_car_motion(vx, vy, vz);
          
        }

    private:
        rclc_executor_t executor_pub, executor_sub;
        


        

    };

    
}