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
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <fstream>
#include <sstream>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "Configuration.h"
#include "drivers/serial.h"
#include "Modem_Definitions.h"
//#include <sonia_common/Modem_Definitions.h>
#include <sonia_common/ModemPacket.h>

namespace provider_underwater_com {

class ProviderUnderwaterComNode
{
    public:

        ProviderUnderwaterComNode(const ros::NodeHandlePtr &_nh);
        ~ProviderUnderwaterComNode();

        void Spin();
    
    private:

        void UnderwaterComCallback(const std_msgs::String &msg);
        bool UnderwaterComService(sonia_common::ModemPacket::Request &req, sonia_common::ModemPacket::Response &res);

        uint8_t CalculateChecksum(const std::string &sentence, uint8_t length);
        void AppendChecksum(std::string &sentence);
        bool ConfirmChecksum(const std::string &sentence);

        void Queue_Packet(const std::string &cmd, const std::string &packet = "");
        bool Transmit_Packet(bool pop_packet);
        void Send_CMD_To_Sensor(char *buffer, char cmd, const std::string &packet = "");
        bool Check_CMD(const char *cmd);

        void Manage_Write();
        void Manage_Response();
        void Export_To_ROS(std::string buffer);
        void Read_Packet();

        void Set_Sensor(const char role, const uint8_t channel = 4);
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
        std_msgs::String msg_received;

        std::thread manage_write_thread;
        std::thread manage_response_thread;
        std::thread read_packet_thread;

        std::mutex write_mutex;
        std::mutex response_mutex;
        std::mutex parse_mutex;

        std::condition_variable write_cond;
        std::condition_variable response_cond;
        std::condition_variable parse_cond;

        std::string write_string = "";
        std::string response_string = "";
        std::string parse_string = "";
       
        uint8_t payload_;
        bool init_error_ = true;
};
}

#endif //PROVIDER_UNDERWATER_COM_NODE