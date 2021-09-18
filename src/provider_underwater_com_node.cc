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
        underwaterComService_ = nh_->advertiseService("/provider_underwater_com/request", &ProviderUnderwaterComNode::UnderwaterComService, this);

        manage_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Manage_Packet, this));
        export_to_ros_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Export_To_ROS, this));

        std::string role_sensor = configuration_.getRole();
        Set_Sensor(role_sensor, std::stoi(configuration_.getChannel()));
    }

    //Node Destructor
    ProviderUnderwaterComNode::~ProviderUnderwaterComNode()
    {
        manage_thread.~thread();
        export_to_ros_thread.~thread();
        underwaterComSubscriber_.shutdown();
    }

    //Node Spin
    void ProviderUnderwaterComNode::Spin()
    {
        ros::Rate r(1); // 1 Hz

        while(ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    void ProviderUnderwaterComNode::UnderwaterComCallback(const std_msgs::String &msg)
    {
        std::string packet = "," + std::to_string(payload_) + "," + msg.data; // TODO add a size check before transmit
        Queue_Packet(std::string(1, CMD_QUEUE_PACKET), packet);
    }

    bool ProviderUnderwaterComNode::UnderwaterComService(sonia_common::ModemPacket::Request &req, sonia_common::ModemPacket::Response &res)
    {   
        Queue_Packet(std::string(1, req.cmd));

        std::unique_lock<std::mutex> mlock(response_mutex);
        response_cond.wait(mlock);

        char cmd_rec = response_str.at(2);
        std::stringstream ss(response_str);

        switch (cmd_rec)
        {
            case CMD_GET_BUFFER_LENGTH:
            {    
                std::string queue_length;
                
                std::getline(ss, queue_length, ',');
                std::getline(ss, queue_length, '*');

                res.queue_length = std::stoi(queue_length);
                break;
            }
            case CMD_GET_SETTINGS:
            {
                std::string role;
                std::string channel;

                std::getline(ss, role, ',');
                std::getline(ss, role, ',');
                std::getline(ss, channel, '*');

                res.role = std::stoi(role);
                res.channel = std::stoi(channel);
                break;
            }
            case CMD_GET_DIAGNOSTIC:
            {
                std::string link_up;
                std::string packet_count;
                std::string packet_loss_count;
                std::string bit_error_rate;

                std::getline(ss, link_up, ',');
                std::getline(ss, link_up, ',');
                std::getline(ss, packet_count, ',');
                std::getline(ss, packet_loss_count, ',');
                std::getline(ss, bit_error_rate, '*');

                res.link = std::stoi(link_up);
                res.packet_count = std::stoi(packet_count);
                res.packet_count_loss = std::stoi(packet_loss_count);
                res.bit_error_rate = std::stof(bit_error_rate);
            }
            default:
            {
                ROS_ERROR("CMD received isn't working with the service.");
                return false;
            }
        }
        
        return true;
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

        if(Check_CMD(cmd))
        {
            ss << SOP << DIR_CMD << cmd;

            if(cmd == std::string(1, CMD_QUEUE_PACKET) || cmd == std::string(1, CMD_SET_SETTINGS))
            {
                ss << packet;
            }

            sentence = ss.str();
            AppendChecksum(sentence);
            writerQueue.push_back(sentence);

            ROS_DEBUG_STREAM("Packet sent to Modem");
        }
        else
        {
            ROS_INFO_STREAM("CMD unknow. Can't queue packet");
        }
    }

    bool ProviderUnderwaterComNode::Read_for_Packet(char *buffer)
    {
        clock_t start = clock();
        clock_t now = clock();
        
        while((now-start) / CLOCKS_PER_SEC <= timeout_)
        {
            serialConnection_.readOnce(buffer, 0);

            if(buffer[0] == SOP)
            {
                uint8_t i;

                for(i = 1; buffer[i-1] != EOP && i < BUFFER_SIZE; ++i)
                {
                    serialConnection_. readOnce(buffer, i);
                }

                buffer[i] = 0;
                return true;
            }

            now = clock();
        }
        ROS_WARN_STREAM("No response from the other sensor");
        return false;
    }

    uint8_t ProviderUnderwaterComNode::Verify_Packet_Size(const std::string &packet)
    {
        return packet.size();
    }
    
    size_t ProviderUnderwaterComNode::Split_Packet(std::string *packet_array, uint8_t size_array, const std::string &msg)
    {
        size_t size_packet = ceil(msg.size()/8.0);
        char buffer[7];

        for(uint8_t i = 0; i < size_packet && i < size_array; ++i)
        {
            try
            {
                packet_array[i] = msg.substr(i*8, 8);
            }
            catch(const std::out_of_range& e)
            {
                msg.copy(buffer, msg.size()-(i*8),i*8);
                packet_array[i] = std::string(buffer);
            }
            
        }
        
        return size_packet;
    }

    bool ProviderUnderwaterComNode::Check_CMD(const std::string &cmd)
    {
        const char *c_cmd = cmd.data();

        for(uint8_t i = 0; i < all_valid_size; ++i)
        {
            if(c_cmd[0] == all_valid[i])
            {
                return true;
            }
        }
        return false;
    }

    void ProviderUnderwaterComNode::Manage_Packet_Master()
    {
        char buffer[BUFFER_SIZE];
        bool resend = true;
        bool new_packet;
        
        if(resend) serialConnection_.transmit(writerQueue.front());

        resend = true;
        
        new_packet = Read_for_Packet(buffer);

        if(ConfirmChecksum(buffer) && new_packet)
        {
            if(buffer[2] == RESP_GOT_PACKET)
            {
                std::unique_lock<std::mutex> mlock(export_to_ros_mutex);
                export_to_ros_str = std::string(buffer);
                export_to_ros_cond.notify_one();
                writerQueue.get_n_pop_front();
            }
            else if(buffer[2] == CMD_QUEUE_PACKET && buffer[4] == ACK)
            {
                resend = false;
            }
            else if(buffer[2] == RETURN_ERROR || buffer[2] == MALFORMED)
            {
                ROS_WARN_STREAM("Resquest not made properly");
                writerQueue.get_n_pop_front();
            }
            else
            {
                std::unique_lock<std::mutex> mlock(response_mutex);
                response_str = std::string(buffer);
                response_cond.notify_one();
                writerQueue.get_n_pop_front();
            }
        }
    }

    void ProviderUnderwaterComNode::Manage_Pakcet_Slave()
    {
        
    }

    void ProviderUnderwaterComNode::Manage_Packet()
    {
        ROS_INFO_STREAM("Manage thread started");

            while(!ros::isShuttingDown())
            {
                ros::Duration(0.1).sleep();

                while(!writerQueue.empty() && role_ == ROLE_MASTER)
                {
                    ROS_INFO_STREAM("I am sending");
                    Manage_Packet_Master();
                }

                while(!readerQueue.empty() && role_ == ROLE_SLAVE)
                {
                    ROS_INFO_STREAM("I am listening");
                    Manage_Pakcet_Slave();
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
            std::getline(ss, size, ','); // TODO add a size check before publish and check if message good
            std::getline(ss, size, ',');
            std::getline(ss, msg, '*');

            msg_received.data = msg;

            underwaterComPublisher_.publish(msg_received);
        }
    }

    void ProviderUnderwaterComNode::Set_Sensor(std::string &role, uint8_t channel)
    {
        ROS_ASSERT_MSG(role == "master" || role == "slave", "Set the role as 'master' or 'slave'. Error in config");

        if(role == "master")
        {
            role_ = ROLE_MASTER;
        }
        else
        {
            role_ = ROLE_SLAVE;
        }

        uint8_t i = 0;
        
        while(i < 3 && init_error_ == true)
        {
            init_error_ = false;
            Verify_Version();   
            Get_Payload_Load();
            Set_Configuration(role_, channel);
            Flush_Queue();
            ++i;
        }

        if(i == 3)
        {
            ROS_ERROR("Problem with the init. Node shutting down.");
            ros::shutdown();
        }
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

        if(major_version != "1")
        {
            ROS_ERROR("Major Version isn't 1. Restarting init");
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

        if(payload >= "0" || payload <= "9")
        {
            payload_ = std::stoi(payload);
            ROS_DEBUG("Payload set");
        }
        else
        {
            ROS_ERROR("Payload isn't a integer. Restarting init");
            init_error_ = true;
        }
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
            ROS_ERROR("Could not set the configuration. Restarting init");
            init_error_ = true;
        }
    }

    void ProviderUnderwaterComNode::Flush_Queue()
    {
        std::string acknowledge;

        Queue_Packet(std::string(1, CMD_FLUSH));

        std::unique_lock<std::mutex> mlock(response_mutex);
        response_cond.wait(mlock);

        std::stringstream ss(response_str);
        std::getline(ss, acknowledge, ',');
        std::getline(ss, acknowledge, '*');

        if(acknowledge == std::string(1, NAK))
        {
            ROS_ERROR("Couldn't flush the queue. Restarting init");
            init_error_ = true;
        }
    }
}