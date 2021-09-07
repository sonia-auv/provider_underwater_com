/**
 * \file	provider_underwater_com_node.cc
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

#include "provider_underwater_com_node.h"

namespace provider_underwater_com 
{

    //Node Construtor
    ProviderUnderwaterComNode::ProviderUnderwaterComNode(const ros::NodeHandlePtr &_nh)
        : nh_(_nh)//, configuration_(_nh), serialConnection_(configuration_.getTtyPort())
    {
        underwaterComSubscriber_ = nh_->subscribe("/proc_underwater_com/send_msgs", 100, &ProviderUnderwaterComNode::UnderwaterComCallback, this);
        underwaterComPublisher_ = nh_->advertise<std_msgs::UInt8>("/provider_underwater_com/receive_msgs", 100);
    }

    //Node Destructor
    ProviderUnderwaterComNode::~ProviderUnderwaterComNode(){}

    //Node Spin
    void ProviderUnderwaterComNode::Spin()
    {
        ros::Rate r(1);

        while(ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    void ProviderUnderwaterComNode::UnderwaterComCallback(const std_msgs::UInt8 &msg)
    {
        std::string packet = std::to_string(msg.data);
        std::string dir = DIR_CMD;
        std::string cmd = CMD_QUEUE_PACKET;

        Queue_Packet(dir, cmd, packet);
    }

    uint8_t ProviderUnderwaterComNode::CalculateChecksum(const std::string sentence)
    {
        uint8_t check = 0;

        for(unsigned int i = 1; i < sentence.size(); i++)
            check ^= sentence[i];
        
        return check;
    }

    void ProviderUnderwaterComNode::AppendChecksum(std::string &sentence)
    {
        std::stringstream ss;
        uint8_t checksum = CalculateChecksum(sentence);
        ss << sentence << std::string("*") << std::hex << checksum;
        sentence = ss.str();
    }

    bool ProviderUnderwaterComNode::ConfirmChecksum(const std::string &sentence)
    {
        try
        {
            std::string checksumData = sentence.substr(0, sentence.find("*", 0));
            uint8_t calculatedChecksum = CalculateChecksum(checksumData);
            uint8_t originalChecksum = std::stoi(sentence.substr(sentence.find("*", 0)+1, 2), nullptr, 16);
            return originalChecksum == calculatedChecksum;
        }
        catch(...)
        {
            ROS_INFO_STREAM("Underwater Com: bad packet checksum");
            return false;
        }
    }

    void ProviderUnderwaterComNode::Queue_Packet(const std::string &direction, const std::string &cmd, const std::string &packet)
    {
        std::stringstream ss;
        std::string sentence;

        if(cmd != CMD_QUEUE_PACKET && cmd != CMD_SET_SETTINGS)
        {
            ss << SOP << direction << cmd;
        }
        else
        {
            ss << SOP << direction << cmd << std::string(",") << packet;
        }

        sentence = ss.str();
        AppendChecksum(sentence);

        

    }
}