/**
 * \file	provider_underwater_com_node.h
 * \author	Francis Alonzo <francisalonzo29@gmail.com
 * \date	02/09/2021
 * 
 * \copyright Copyright (c) 2021 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROVIDER_UNDERWATER_COM_NODE
#define PROVIDER_UNDERWATER_COM_NODE

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <future>
#include <std_msgs/UInt64.h>

#include "Configuration.h"
#include "drivers/serial.h"
#include <sonia_common/Modem_Definitions.h>
#include <sonia_common/ModemSendCmd.h>

#define BUFFER_SIZE 256
#define MODEM_M64_PAYLOAD 8

namespace provider_underwater_com {

class ProviderUnderwaterComNode
{
    public:

        ProviderUnderwaterComNode(const ros::NodeHandlePtr &_nh);
        ~ProviderUnderwaterComNode();

        void Spin();
    
    private:

        void UnderwaterComCallback(const std_msgs::UInt64 &msg);
        bool UnderwaterComService(sonia_common::ModemSendCmd::Request &req, sonia_common::ModemSendCmd::Response &res);

        uint8_t Calculate_Checksum(const char (&buffer)[BUFFER_SIZE], const size_t size); // Prevent decay
        uint8_t Append_Checksum(char (&buffer)[BUFFER_SIZE], const size_t size); // Prevent decay
        bool Confirm_Checksum(char (&buffer)[BUFFER_SIZE], const size_t size); // Prevent decay

        void Queue_Packet(const char cmd, const char (&packet)[MODEM_M64_PAYLOAD] = {}, const size_t size_packet = 0); // Prevent decay
        bool Transmit_Packet(bool pop_packet);
        bool Send_CMD_To_Sensor(char *buffer, char cmd, const char (&packet)[MODEM_M64_PAYLOAD] = {}, size_t size = 0); // Prevent decay
        bool Check_CMD(const char *cmd);
        void Append_Packet(char (&buffer)[BUFFER_SIZE], const size_t index, const char (&packet)[MODEM_M64_PAYLOAD], const size_t size_packet); // Prevent decay
        uint8_t Find_Character(const char (&buffer)[BUFFER_SIZE], const char to_find, const size_t size); // Prevent decay
        void Copy_Array(const char (&buffer)[BUFFER_SIZE], char (&buffer_returned)[BUFFER_SIZE], const size_t size, const size_t start = 0); // Prevent decay

        void Manage_Write();
        void Export_To_ROS(const char (&buffer)[BUFFER_SIZE], const ssize_t size); // Prevent decay
        void Read_Packet();

        void Set_Sensor();
        void Init_Function();
        bool Verify_Version();
        bool Get_Payload_Load();
        bool Set_Configuration(const char role, const uint8_t channel);
        bool Flush_Queue();

        ros::NodeHandlePtr nh_;
        Configuration configuration_;
        Serial serialConnection_;
        
        ros::Subscriber underwaterComSubscriber_;
        ros::Publisher underwaterComPublisher_;
        ros::ServiceServer underwaterComService_;

        std::thread manage_write_thread;
        std::thread read_packet_thread;
        std::thread init_function_thread;
        std::promise <bool>init_complete;

        std::mutex write_mutex;
        std::mutex response_mutex;
        std::mutex parse_mutex;

        std::condition_variable write_cond;
        std::condition_variable response_cond;
        std::condition_variable parse_cond;

        std::string write_string = "";
        std::string response_string = "";
        std::string parse_string = "";

        std::atomic_bool stop_write_thread = {false};
        std::atomic_bool stop_read_thread = {false};


        char writeBuffer[BUFFER_SIZE] = {};
        uint8_t writeSize = 0;       
        uint8_t payload_;
        bool init_error_ = true;
};
}

#endif //PROVIDER_UNDERWATER_COM_NODE