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

#define BUFFER_SIZE 256

namespace provider_underwater_com 
{

    //Node Construtor
    ProviderUnderwaterComNode::ProviderUnderwaterComNode(const ros::NodeHandlePtr &_nh)
        : nh_(_nh), configuration_(_nh), serialConnection_(configuration_.getTtyPort())
    {
        serialConnection_.flush();

        underwaterComSubscriber_ = nh_->subscribe("/proc_underwater_com/send_msgs", 100, &ProviderUnderwaterComNode::UnderwaterComCallback, this);
        underwaterComPublisher_ = nh_->advertise<std_msgs::UInt8>("/provider_underwater_com/receive_msgs", 100);

        reader_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Read_Packet, this));

        Set_Sensor(ROLE_MASTER, 4);
    }

    //Node Destructor
    ProviderUnderwaterComNode::~ProviderUnderwaterComNode()
    {
        underwaterComSubscriber_.shutdown();
        reader_thread.~thread();
    }

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
        std::string packet = ",8," + std::to_string(msg.data); // Payload for the CMD to send, always 8
        std::string dir = std::to_string(DIR_CMD);
        std::string cmd = std::to_string(CMD_QUEUE_PACKET);

        Queue_Packet(dir, cmd, packet);
    }

    uint8_t ProviderUnderwaterComNode::CalculateChecksum(const std::string sentence)
    {
        uint8_t check = 0;

        for(unsigned int i = 0; i < sentence.size(); i++)
            check ^= sentence[i];
        
        return check;
    }

    void ProviderUnderwaterComNode::AppendChecksum(std::string &sentence)
    {
        std::stringstream ss;
        uint8_t checksum = CalculateChecksum(sentence);
        ss << sentence << std::to_string(CHECKSUM) << std::hex << checksum;
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
            ROS_INFO_STREAM("Underwater Com: bad checksum");
            return false;
        }
    }

    void ProviderUnderwaterComNode::Queue_Packet(const std::string &direction, const std::string &cmd, const std::string &packet)
    {
        std::stringstream ss;
        std::string sentence;

        if(cmd != std::to_string(CMD_QUEUE_PACKET) && cmd != std::to_string(CMD_SET_SETTINGS))
        {
            ss << SOP << direction << cmd << EOP;
        }
        else
        {
            ss << SOP << direction << cmd << std::string(",") << packet << EOP;
        }

        sentence = ss.str();
        //AppendChecksum(sentence);
        serialConnection_.transmit(sentence);

        ROS_DEBUG("Packet sent to Modem");
    }

    void ProviderUnderwaterComNode::Read_Packet()
    {
        ROS_INFO_STREAM("Reader thread started");
        char buffer[BUFFER_SIZE];

            while(!ros::isShuttingDown())
            {
                do 
                {
                    serialConnection_.readOnce(buffer, 0);
                } 
                while (buffer[0] != SOP);
                
                uint8_t i;

                for(i = 1; buffer[i-1] != EOP && i < BUFFER_SIZE; ++i)
                {
                    serialConnection_. readOnce(buffer, i);
                }

                if(i >= BUFFER_SIZE)
                {
                    continue;
                }

                buffer[i] = 0;

                if(buffer[1] != DIR_RESP)
                {
                    ROS_INFO_STREAM("Error on the response");
                }

                ROS_INFO_STREAM(std::string(buffer));
            }
    }

    bool ProviderUnderwaterComNode::Set_Sensor(const char &role, uint8_t channel)
    {
        Verify_Version();
    }

    void ProviderUnderwaterComNode::Verify_Version()
    {
        Queue_Packet(std::to_string(DIR_CMD),std::to_string(CMD_GET_VERSION));
    }
}