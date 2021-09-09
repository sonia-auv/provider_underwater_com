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
#include <std_msgs/UInt8.h>
#include <fstream>
#include <sstream>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "Configuration.h"
#include "drivers/serial.h"
#include "ModemM64_definitions.h"

namespace provider_underwater_com {

class ProviderUnderwaterComNode
{
    public:

        ProviderUnderwaterComNode(const ros::NodeHandlePtr &_nh);
        ~ProviderUnderwaterComNode();

        void Spin();
    
    private:

        void UnderwaterComCallback(const std_msgs::UInt8 &msg);

        uint8_t CalculateChecksum(const std::string &sentence, uint8_t length);
        void AppendChecksum(std::string &sentence);
        bool ConfirmChecksum(const std::string &sentence);

        void Queue_Packet(const std::string &direction, const std::string &cmd, const std::string &packet = "");
        void Read_Packet();
        void Set_Sensor(const char &role = ROLE_MASTER, uint8_t channel = 4);
        bool Verify_Version();


        ros::NodeHandlePtr nh_;
        Configuration configuration_;
        Serial serialConnection_;
        
        ros::Subscriber underwaterComSubscriber_;
        ros::Publisher underwaterComPublisher_;

        std::thread reader_thread;
        std::mutex response_mutex;
        std::condition_variable response_cond;
        std::string response_str = "";

        uint8_t payload_;
        
};

}

#endif //PROVIDER_UNDERWATER_COM_NODE