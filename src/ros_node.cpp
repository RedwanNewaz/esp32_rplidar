#include "credential.h"
#include "ros_node.h"

namespace airlab{

    Node::Node()
    {

    }

    void Node::init(rplidarHandler *lidar, const char* nodeName )
    {
        set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

        allocator = rcl_get_default_allocator();

        rplidar = lidar; 

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
         
    }

    void Node::loop()
    {
        if(rplidar != nullptr)
            rplidar->loop();
        
        RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(LOOP_DELAY)));
        RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(LOOP_DELAY)));
    }


    void Node::timer_callback(rcl_timer_t * timer, int64_t last_call_time)
    {
        RCLC_UNUSED(last_call_time);
        if (timer == NULL || rplidar == nullptr || !rplidar->is_measurement_ready()) 
            return;
        
        set_lidar_scan_msg();
        
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        
    }

    void Node::set_lidar_scan_msg()
    {
        static sensor_msgs__msg__LaserScan scan_msg; 
        scan_msg.header.frame_id.data = "rplidar";
        float angle_max = 0;
        float angle_min = DEG2RAD(360.0);
        int N = rplidar->get_measurement_size();
    

        for(int i = 0; i < N; ++i)
        {
            auto data = rplidar->get_measurement(i);
            float angle = DEG2RAD(data.angle);
            angle_max = max(angle_max, angle);
            angle_min = min(angle_min, angle);
        }

        bool reversed = (angle_max > angle_min);
        
        if ( reversed ) {
            scan_msg.angle_min =  M_PI - angle_max;
            scan_msg.angle_max =  M_PI - angle_min;
        } else {
            scan_msg.angle_min =  M_PI - angle_min;
            scan_msg.angle_max =  M_PI - angle_max;
        }

        scan_msg.angle_increment =
        (scan_msg.angle_max - scan_msg.angle_min) / (float)(N);

        float scan_time = (rplidar->get_measurement(N-1).scan_time - rplidar->get_measurement(0).scan_time) / 1000.0;
        scan_msg.scan_time = scan_time;
        scan_msg.time_increment = scan_time / (float)(N);
        scan_msg.range_min = 0.15;
        scan_msg.range_max = 0;//;

        // compute ranges and distance 
        if(scan_msg.ranges.data != nullptr)
            delete[] scan_msg.ranges.data;

        if(scan_msg.intensities.data != nullptr)
            delete[] scan_msg.intensities.data;
        
        scan_msg.ranges.data = new float[N];
        scan_msg.intensities.data = new float[N];
        scan_msg.ranges.size = scan_msg.intensities.size = N;
        scan_msg.ranges.capacity = scan_msg.intensities.capacity = N;

        bool inverted = false; 
        bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
        if (!reverse_data) {
            for (size_t i = 0; i < N; i++) {
                auto data = rplidar->get_measurement(i);
                // float read_value = data.dist_mm_q2/4.0f/1000;
                float read_value = data.dist_mm_q2/1000;
                scan_msg.ranges.data[i] = read_value;
                scan_msg.intensities.data[i] = (float)(data.quality >> 2);
                scan_msg.range_max = max(scan_msg.range_max, read_value);
            }
        } else {
            for (size_t i = 0; i < N; i++) {
                auto data = rplidar->get_measurement(i);
                // float read_value = data.dist_mm_q2/4.0f/1000;
                float read_value = data.dist_mm_q2/1000;
             
                scan_msg.ranges.data[N-1-i] = read_value;
                scan_msg.intensities.data[N-1-i] = (float) (data.quality >> 2);
                scan_msg.range_max = max(scan_msg.range_max, read_value);
            }
        }       


        RCSOFTCHECK(rcl_publish(&publisher, &scan_msg, NULL));
    }

    void Node::subscription_callback(const void * msgin)
    {
        // Cast received message to used type
        const geometry_msgs__msg__Twist * cmd_msg_ptr = (const geometry_msgs__msg__Twist *)msgin;
        float vx = cmd_msg_ptr->linear.x;
        float vy = cmd_msg_ptr->linear.y;
        float vz = cmd_msg_ptr->angular.z;
        set_car_motion(vx, vy, vz);

    }

    void Node::set_car_motion(float v_x, float v_y, float v_z)
    {
      byte vx_parms[2], vy_parms[2], vz_parms[2];
      int16_t vx = int16_t(v_x * 1000), vy = int16_t(v_y * 1000), vz = int16_t(v_z * 1000);

      vx_parms[0] = lowByte(vx);
      vx_parms[1] = highByte(vx);
      vy_parms[0] = lowByte(vy);
      vy_parms[1] = highByte(vy);
      vz_parms[0] = lowByte(vz);
      vz_parms[1] = highByte(vz);

      byte cmd[] = {HEAD, DEVICE_ID, 0x00, FUNC_MOTION, CAR_TYPE, 
                    vx_parms[0], vx_parms[1], vy_parms[0], vy_parms[1], vz_parms[0], vz_parms[1], 0};
      cmd[2] = 10;
      byte checksum = 0;
      for (byte i = 0; i < 11; i++) {
        checksum += cmd[i];
      }
      checksum += COMPLEMENT;
      checksum &= 0xFF;
      cmd[11] = checksum;
      Serial.write(cmd, sizeof(cmd));
    }



}