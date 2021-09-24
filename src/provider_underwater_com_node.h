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
#include <string>
#include <std_msgs/String.h>
#include <fstream>
#include <sstream>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <math.h>
#include <time.h>

#include "Configuration.h"
#include "drivers/serial.h"
#include <sonia_common/ModemM64_definitions.h>
//#include "ModemM64_definitions.h"
#include <sonia_common/ModemPacket.h>
#include "sharedQueue.h"

#define MALFORMED '!'

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
        bool Read_for_Packet(char *buffer);
        bool Check_CMD(const std::string &cmd);

        void Manage_Packet_Master();
        void Manage_Packet_Slave();

        void Manage_Packet();
        void Export_To_ROS(std::string buffer);
        void Read_for_Packet_Slave();

        void Set_Sensor(std::string &role, uint8_t channel = 4);
        void Verify_Version();
        void Get_Payload_Load();
        void Set_Configuration(const char &role, uint8_t channel);
        void Flush_Queue();

        ros::NodeHandlePtr nh_;
        Configuration configuration_;
        Serial serialConnection_;
        
        ros::Subscriber underwaterComSubscriber_;
        ros::Publisher underwaterComPublisher_;
        ros::ServiceServer underwaterComService_;
        std_msgs::String msg_received;

        std::thread manage_thread;
        std::thread read_for_packet_slave;

        std::mutex writerQueue_mutex;

        char role_;
        uint8_t channel_;        
        uint8_t payload_;
        bool init_error_ = true;
        bool resend_ = true;

        SharedQueue<std::string> writerQueue;
        SharedQueue<std::string> readerQueue;

        ros::Duration sleeptime;

        float_t timeout_ = 10.0;
};

}

#endif //PROVIDER_UNDERWATER_COM_NODE