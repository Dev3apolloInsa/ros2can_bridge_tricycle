// Licensed to the Apache Software Foundation (ASF) under one
// or more contributor license agreements.  See the NOTICE file
// distributed with this work for additional information
// regarding copyright ownership.  The ASF licenses this file
// to you under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
// 
//   http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing,
// software distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.

/** @file ros2socketcan.h
 *
 *  @ingroup ROS2CAN_bridge
 *  @author Philipp Wuestenberg
 *  @brief  bidirectional ROS2 to CAN interface with topics and service
 */

#ifndef __ros2_socketcan_H__
#define __ros2_socketcan_H__

#include <linux/can/raw.h>
#include <boost/asio.hpp>
#include <mutex>
#include <thread>
#include <cstdio>
#include <memory>
#include <string>
#include <fcntl.h>              
#include <stdio.h> 
#include <unistd.h>                  
#include <iostream>
#include <bitset>
#include <sstream>
#include <linux/joystick.h> 
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "can_msgs/msg/tricycle_status.hpp"
#include "can_msgs/msg/tricycle_control_cmd.hpp"
//#include "can_msgs/srv/can_request.hpp"

#include "log.h"

const std::string version = "1.00 from: " + std::string(__DATE__) + " " + std::string(__TIME__);
const std::string programdescr = "ROS 2 to CAN-Bus Bridge\nVersion: " + version;

/**
 * @brief The ros2socketcan bridge connects a canbus with the ROS2 topic system. 
 * @details A nodes is provided, which provides the bridge from a ROS topic to the CAN bus and from the CAN bus to a ROS topic. The node functions as a bidirectional bridge and provides a service to publish a message and receive the answer with the fitting message id. 
 * 
 */
class ros2socketcan : public rclcpp::Node
{
    public:
        /**
         * @brief constructor for ros2socketcan class
         * @details Within the constructor the topic and service naming is done. 
         */
        ros2socketcan(std::string can_socket2 = "vcan0");//boost::asio::io_service& ios);
        
        /**
         * @brief Within the Init() fucntin the ROS and CAN setup is done.
         * @details Within the Init() function the ROS2 publisher, subscriber and the service server is initialized. In addition the socketcan interface is configured and assigned to the socket. The Init function is necessary as the topics need a fully constructed node class to be added to.
         */
        void Init(const char* can_socket = "vcan0");//boost::asio::io_service& ios);
        /**
         * @brief destructor
         */
        ~ros2socketcan();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
        rclcpp::Publisher<can_msgs::msg::TricycleStatus>::SharedPtr tricyclepublisher_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr test_pub_;
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
        rclcpp::Publisher<can_msgs::msg::TricycleControlCmd>::SharedPtr tricycletransmitcmd_;     
        //rclcpp::service::Service<can_msgs::srv::CanRequest>::SharedPtr server_ros2can_;
        
        can_msgs::msg::Frame current_frame;
        
        /** @brief The CanSendConfirm function is needed by the .async_write_some function and is called as confirmation for a successfull send process.
         */
        void CanSendConfirm();
        
        /** @brief The CanPublisher is listening to a ROS2 Topic and calls the CanSend Method.
         */
        void CanPublisher(const can_msgs::msg::Frame::SharedPtr msg);
        
        /** @brief The CanSend method sends a ROS message to the CAN bus.
         * @details The CanSend function is Called by the CanPublisher and ther ros2can_srv. It converts the ROS message to a can_frame and adds the CAN Flags to the message ID.  
         */
        void CanSend(const can_msgs::msg::Frame msg);
        
        /** @brief The CanListener listens to the CAN Bus and publishes the message to a ROS2 Topic.
         * @details The CanListener function is Called by the .async_read_some when a Message is received on the Can Socket. 
         * It converts the message to a ROS Message and publishes it to a ROS2 Topic. Afterwards .async_read_some must be called again to wait for further CAN Messages.
         */
        void CanListener(struct can_frame& rec_frame, boost::asio::posix::basic_stream_descriptor<>& stream);
        
        /** @brief The ros2can_service provides the possibility to send a can message and wait for a specific can message with a give CAN Message ID.
         */
        /*void ros2can_srv(
        const std::shared_ptr<rmw_request_id_t> //request_header,
        const std::shared_ptr<can_msgs::srv::CanRequest::Request> request,
        std::shared_ptr<can_msgs::srv::CanRequest::Response> response);*/
        

        double GetCommandThrottle(double val, double inc);

        double GetCommandSteer(double val, double inc);

        /** @Reads a joystick event from the joystick device ==> Returns 0 on success.Otherwise -1 is returned.
        */
        int read_event(int fd, struct js_event *event);

        /** Returns the number of axes on the controller or 0 if an error occurs.
        */
        size_t get_axis_count(int fd);

        /** @Returns the number of buttons on the controller or 0 if an error occurs.
        */
        size_t get_button_count(int fd);

        /** Current state of an axis.
        */
        struct axis_state {
            short x, y;
        };

        /** Keeps track of the current axis state.
        *
        * NOTE: This function assumes that axes are numbered starting from 0, and that
        * the X axis is an even number, and the Y axis is an odd number. However, this
        * is usually a safe assumption.
        *
        * Returns the axis that the event indicated.
        */
        size_t get_axis_state(struct js_event *event, struct axis_state axes[3]);

        /** Thread function definition for joystick*/
        void JoystickLoopThreadFunc();
        
        /** @biref The Stop method is needed as the interuped handler must be configered to the asio libary.
         */
        void stop();
        
        boost::asio::io_service ios;
        boost::asio::posix::basic_stream_descriptor<> stream;
        boost::asio::signal_set signals;

        struct state {
            bool tab[15] = {false};
        } state_button;

        struct datacan {
            unsigned int candata[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        } state_can_data;

        int js;  
        const char *device;                                            
        struct js_event event;
        struct axis_state axes[3] = {0};
        size_t axis;
        std::unique_ptr<std::thread> keyboard_thread_;

        struct sockaddr_can addr;
        struct can_frame frame;
        struct can_frame framebis;
        struct can_frame controlcmd;
        struct can_frame rec_frame;
        struct ifreq ifr;
        float FLAGS_throttle_inc_delta = 0.0;
        float FLAGS_steering_inc_delta = 2.0;
        bool var_brake;
        int cmd_acc;
        int cmd_sterr;
        float cmd_sterr1;

        int natsock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        std::stringstream topicname_receive;
        std::stringstream topicname_transmit;
        std::stringstream topicname_transmitbis;
        std::stringstream topicname_receivebis;
        std::stringstream servername;
};
#endif
