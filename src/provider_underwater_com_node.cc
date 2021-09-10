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
        underwaterComPublisher_ = nh_->advertise<std_msgs::String>("/provider_underwater_com/receive_msgs", 100);

        reader_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Read_Packet, this));
        export_to_ros_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Export_To_ROS, this));

        Set_Sensor(ROLE_MASTER, 4);
    }

    //Node Destructor
    ProviderUnderwaterComNode::~ProviderUnderwaterComNode()
    {
        underwaterComSubscriber_.shutdown();
        reader_thread.~thread();
        export_to_ros_thread.~thread();
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

    void ProviderUnderwaterComNode::UnderwaterComCallback(const std_msgs::String &msg)
    {
        std::string packet = "," + std::to_string(payload_) + "," + msg.data; // TODO add a size check before transmit
        std::string dir = std::string(1, DIR_CMD);
        std::string cmd = std::string(1, CMD_QUEUE_PACKET);

        Queue_Packet(cmd, packet);
    }

    uint8_t ProviderUnderwaterComNode::CalculateChecksum(const std::string &sentence, uint8_t length)
    {
        uint8_t check = 0;

        for(uint8_t i = 0; i < length; ++i)
        {
            check = crc_table[(uint8_t)sentence.at(i) ^ check];
        }
        
        return check;
    }

    void ProviderUnderwaterComNode::AppendChecksum(std::string &sentence)
    {
        std::stringstream ss;
        char buffer[2];

        uint8_t checksum = CalculateChecksum(sentence, sentence.size());
        sprintf(buffer, "%x", checksum);

        ss << sentence << std::string(1, CHECKSUM) << buffer << std::string(1, EOP);
        sentence = ss.str();
    }

    bool ProviderUnderwaterComNode::ConfirmChecksum(const std::string &sentence)
    {      
        try
        {
            std::string checksumData = sentence.substr(0, sentence.find("*", 0));
            uint8_t calculatedChecksum = CalculateChecksum(checksumData, checksumData.size());
            uint8_t originalChecksum = std::stoi(sentence.substr(sentence.find("*", 0)+1, 2), nullptr, 16);
            return originalChecksum == calculatedChecksum;
        }
        catch(...)
        {
            ROS_INFO_STREAM("Underwater Com: bad checksum");
            return false;
        }
    }

    void ProviderUnderwaterComNode::Queue_Packet(const std::string &cmd, const std::string &packet)
    {
        std::stringstream ss;
        std::string sentence;

        ss << SOP << DIR_CMD << cmd;

        if(cmd == std::string(1, CMD_QUEUE_PACKET) || cmd == std::string(1, CMD_SET_SETTINGS))
        {
            ss << packet;
        }

        sentence = ss.str();
        AppendChecksum(sentence);
        serialConnection_.transmit(sentence);

        ROS_DEBUG("Packet sent to Modem");
    }

    void ProviderUnderwaterComNode::Read_Packet()
    {
        ROS_INFO_STREAM("Read thread started");
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

                if(buffer[1] != DIR_RESP || buffer[2] == RETURN_ERROR)
                {
                    ROS_INFO_STREAM("Error on the response");
                }
                else if(buffer[2] == RESP_GOT_PACKET)
                {
                    std::unique_lock<std::mutex> mlock(export_to_ros_mutex);
                    export_to_ros_str = std::string(buffer);
                    export_to_ros_cond.notify_one();
                }
                else if(buffer[2] != CMD_FLUSH && ConfirmChecksum(buffer))
                {
                    std::unique_lock<std::mutex> mlock(response_mutex);
                    response_str = std::string(buffer);
                    response_cond.notify_one();
                }
            }
    }

    void ProviderUnderwaterComNode::Export_To_ROS()
    {
        std::string size;
        std::string msg;

        while(!ros::isShuttingDown())
        {
            msg_received.data.clear();
            std::unique_lock<std::mutex> mlock(export_to_ros_mutex);
            export_to_ros_cond.wait(mlock);

            std::stringstream ss(export_to_ros_str);
            std::getline(ss, size, ','); // TODO add a size check before publish
            std::getline(ss, size, ',');
            std::getline(ss, msg, '*');

            msg_received.data = msg;

            underwaterComPublisher_.publish(msg_received);
        }
    }

    void ProviderUnderwaterComNode::Set_Sensor(const char &role, uint8_t channel)
    {
        Verify_Version();   
        Get_Payload_Load();
        Set_Configuration(role, channel);
    }

    void ProviderUnderwaterComNode::Verify_Version()
    {
        std::string major_version = "";
        
        Queue_Packet(std::string(1, CMD_GET_VERSION));

        std::unique_lock<std::mutex> mlock(response_mutex);
        response_cond.wait(mlock);

        std::stringstream ss(response_str);
        std::getline(ss, major_version, ',');
        std::getline(ss, major_version, ',');

        if(std::stoi(major_version) == 1)
        {
            ROS_DEBUG("Major version is 1");
        }
        else
        {
            ROS_INFO_STREAM("Major version isn't 1");
            init_error_ = true;
        }
        
    }

    void ProviderUnderwaterComNode::Get_Payload_Load()
    {
        std::string payload = "";

        Queue_Packet(std::string(1, CMD_GET_PAYLOAD_SIZE));

        std::unique_lock<std::mutex> mlock(response_mutex);
        response_cond.wait(mlock);

        std::stringstream ss(response_str);
        std::getline(ss, payload, ',');
        std::getline(ss, payload, '*');

        payload_ = std::stoi(payload);

        ROS_DEBUG("Payload set");
    }

    void ProviderUnderwaterComNode::Set_Configuration(const char &role, uint8_t channel)
    {
        std::string acknowledge;
        char buffer[2];

        sprintf(buffer, "%d", channel);
        std::string packet = "," + std::string(1, role) + "," + buffer;

        Queue_Packet(std::string(1, CMD_SET_SETTINGS), packet);
        
        std::unique_lock<std::mutex> mlock(response_mutex);
        response_cond.wait(mlock);

        std::stringstream ss(response_str);
        std::getline(ss, acknowledge, ',');
        std::getline(ss, acknowledge, '*');

        if(acknowledge == std::string(1, NAK))
        {
            ROS_WARN_STREAM("Could not set the configuration");
        }
        else
        {
            ROS_DEBUG("Configuration set");
        }
    }
}