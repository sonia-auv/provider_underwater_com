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
        
        manage_write_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Manage_Write, this));
        manage_response_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Manage_Response, this));
        read_packet_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Read_Packet, this));
        
        ROS_INFO_STREAM("Setting the sensor");

        Set_Sensor(configuration_.getRole().at(0), std::stoi(configuration_.getChannel()));

        underwaterComService_ = nh_->advertiseService("/provider_underwater_com/request", &ProviderUnderwaterComNode::UnderwaterComService, this);
    }

    //Node Destructor
    ProviderUnderwaterComNode::~ProviderUnderwaterComNode()
    {
        manage_write_thread.~thread();
        manage_response_thread.~thread();
        read_packet_thread.~thread();
    }

    //Node Spin
    void ProviderUnderwaterComNode::Spin()
    {
        ros::Rate r(5); // 5 Hz

        while(ros::ok() && init_error_ == false)
        {
            ros::spinOnce();
            r.sleep();
        }
        ros::shutdown();
    }

    void ProviderUnderwaterComNode::UnderwaterComCallback(const std_msgs::String &msg)
    {
        std::string packet = "," + std::to_string(payload_) + "," + msg.data; // TODO add a size check before transmit
        Queue_Packet(std::string(1, CMD_QUEUE_PACKET), packet);
    }

    bool ProviderUnderwaterComNode::UnderwaterComService(sonia_common::ModemPacket::Request &req, sonia_common::ModemPacket::Response &res)
    {   
        char buffer[BUFFER_SIZE];

        switch ((char)req.cmd)
        {
            case CMD_GET_BUFFER_LENGTH:
            {    
                Send_CMD_To_Sensor(buffer, CMD_GET_BUFFER_LENGTH);
                std::stringstream ss(buffer);
                
                std::string queue_length;
                
                std::getline(ss, queue_length, ',');
                std::getline(ss, queue_length, '*');

                res.queue_length = std::stoi(queue_length);
                break;
            }
            case CMD_GET_SETTINGS:
            {
                Send_CMD_To_Sensor(buffer, CMD_GET_SETTINGS);
                std::stringstream ss(buffer);

                std::string role;
                std::string channel;

                std::getline(ss, role, ',');
                std::getline(ss, role, ',');
                std::getline(ss, channel, '*');

                res.role = (uint8_t)role.at(0);
                res.channel = std::stoi(channel);
                break;
            }
            case CMD_GET_DIAGNOSTIC:
            {
                Send_CMD_To_Sensor(buffer, CMD_GET_DIAGNOSTIC);
                std::stringstream ss(buffer);

                std::string link_up;
                std::string packet_count;
                std::string packet_loss_count;
                std::string bit_error_rate;

                std::getline(ss, link_up, ',');
                std::getline(ss, link_up, ',');
                std::getline(ss, packet_count, ',');
                std::getline(ss, packet_loss_count, ',');
                std::getline(ss, bit_error_rate, '*');

                res.link = (uint8_t)link_up.at(0);
                res.packet_count = std::stoi(packet_count);
                res.packet_count_loss = std::stoi(packet_loss_count);
                res.bit_error_rate = std::stof(bit_error_rate);
                break;
            }
            case CMD_SET_SETTINGS:
            {
                if(Set_Configuration((char)req.role, req.channel)) return false;
                res.role = req.role;
                res.channel = req.channel;
                break;
            }
            case CMD_FLUSH:
            {
                ROS_INFO_STREAM("Flushed queue");
                if(Flush_Queue()) return false;
                break;
            }
            default:
            {
                ROS_ERROR_STREAM("CMD received isn't working with this service");
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
        sprintf(buffer, "%02x", checksum);

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
            ROS_WARN_STREAM("Underwater Com: bad checksum");
            return false;
        }
    }

    void ProviderUnderwaterComNode::Queue_Packet(const std::string &cmd, const std::string &packet)
    {
        std::stringstream ss;
        std::string sentence;

        if(Check_CMD(cmd.data()))
        {
            ss << SOP << DIR_CMD << cmd;

            if(cmd == std::string(1, CMD_QUEUE_PACKET) || cmd == std::string(1, CMD_SET_SETTINGS))
            {
                ss << packet;
            }

            sentence = ss.str();
            AppendChecksum(sentence);

            std::unique_lock<std::mutex> mlock(write_mutex);
            write_string = sentence;
            write_cond.notify_one();

            ROS_DEBUG_STREAM("Packet sent to Modem");
        }
        else
        {
            ROS_WARN_STREAM("CMD unknow. Can't queue packet");
        }
    }

    bool ProviderUnderwaterComNode::Transmit_Packet(bool pop_packet)
    {
        if(!write_string.empty())
        {
            serialConnection_.transmit(write_string);
            if(pop_packet) write_string.clear();
            return true;
        }
        else
        {
            ROS_WARN_STREAM("Packet isn't queue");
            return false;
        }
    }

    void ProviderUnderwaterComNode::Send_CMD_To_Sensor(char *buffer, char cmd, const std::string &packet)
    {
        Queue_Packet(std::string(1, cmd), packet);        
        std::unique_lock<std::mutex> mlock(parse_mutex);
        parse_cond.wait(mlock);

        if(!parse_string.empty())
        {        
            std::string tmp = parse_string;

            for(uint8_t i = 0; i < tmp.size(); ++i)
            {
                buffer[i] = tmp.at(i);
            }

            parse_string.clear();
        }
    }

    bool ProviderUnderwaterComNode::Check_CMD(const char *cmd)
    {
        for(uint8_t i = 0; i < all_valid_size; ++i)
        {
            if(cmd[0] == all_valid[i])
            {
                return true;
            }
        }
        return false;
    }

    void ProviderUnderwaterComNode::Manage_Write()
    { 
        ROS_INFO_STREAM("Manage write thread started");

            while(!ros::isShuttingDown())
            {
                std::unique_lock<std::mutex> mlock(write_mutex);
                write_cond.wait(mlock);
                Transmit_Packet(true);
                ROS_INFO_STREAM("Packet is sent");
            }
    }

    void ProviderUnderwaterComNode::Manage_Response()
    {
        ROS_INFO_STREAM("Manage response thread started");

        while (!ros::isShuttingDown())
        {
            std::unique_lock<std::mutex> mlock(response_mutex);
            response_cond.wait(mlock);
            Export_To_ROS(response_string);
            response_string.clear();
        }
    }

    void ProviderUnderwaterComNode::Export_To_ROS(std::string buffer)
    {
        std::string size;
        std::string msg;

        msg_received.data.clear();
        std::stringstream ss(buffer);
        std::getline(ss, size, ','); // TODO add a size check before publish and check if message good
        std::getline(ss, size, ',');
        std::getline(ss, msg, '*');

        msg_received.data = msg;

        underwaterComPublisher_.publish(msg_received);
    }

    void ProviderUnderwaterComNode::Read_Packet()
    {
        uint8_t i;
        char buffer[BUFFER_SIZE];

        while(!ros::isShuttingDown())
        {          
            do
            {
                serialConnection_.readOnce(buffer, 0);
            } while(buffer[0] != SOP);

            for(i = 1; buffer[i-1] != EOP && i < BUFFER_SIZE; ++i)
            {
                serialConnection_. readOnce(buffer, i);
            }

            buffer[i] = 0;

            if(i > BUFFER_SIZE)
            {
                continue;
            }

            ROS_INFO_STREAM("New Data received");

            if(ConfirmChecksum(buffer))
            {                
                if(buffer[2] == RESP_GOT_PACKET)
                {
                    std::unique_lock<std::mutex> mlock(response_mutex);
                    response_string = std::string(buffer);
                    response_cond.notify_one();
                }
                else if(buffer[2] == CMD_QUEUE_PACKET && buffer[4] == ACK)
                {
                    ROS_INFO_STREAM("Packet queue");
                }
                else if(buffer[2] == RETURN_ERROR || buffer[2] == MALFORMED)
                {
                    ROS_ERROR_STREAM("Resquest not made properly");
                    std::unique_lock<std::mutex> mlock(parse_mutex);
                    parse_cond.notify_one();
                }
                else
                {
                    std::unique_lock<std::mutex> mlock(parse_mutex);
                    parse_string = std::string(buffer);
                    parse_cond.notify_one();
                }
            }
        }
    }

    void ProviderUnderwaterComNode::Set_Sensor(const char role, const uint8_t channel)
    {
        uint8_t i = 0;
        
        while(i < 3 && init_error_ == true)
        {
            init_error_ = false;
            init_error_ = Verify_Version() | Get_Payload_Load() | Set_Configuration(role, channel) | Flush_Queue();  
            ++i;
        }

        if(init_error_)
        {
            ROS_ERROR_STREAM("Problem with the init. Node shutting down.");
        }
        else
        {
            ROS_INFO_STREAM("Initialisation completed");
        }
    }

    bool ProviderUnderwaterComNode::Verify_Version()
    {
        std::string major_version = "";
        char buffer[BUFFER_SIZE];
        
        Send_CMD_To_Sensor(buffer, CMD_GET_VERSION);

        std::stringstream ss(buffer);
        std::getline(ss, major_version, ',');
        std::getline(ss, major_version, ',');

        if(major_version != "1")
        {
            ROS_ERROR_STREAM("Major Version isn't 1.");
            return true;
        }
        ROS_INFO_STREAM("Major version is 1");
        return false;
    }

    bool ProviderUnderwaterComNode::Get_Payload_Load()
    {
        std::string payload = "";
        char buffer[BUFFER_SIZE];

        Send_CMD_To_Sensor(buffer, CMD_GET_PAYLOAD_SIZE);

        std::stringstream ss(buffer);
        std::getline(ss, payload, ',');
        std::getline(ss, payload, '*');

        if((payload >= "0" || payload <= "9"))
        {
            payload_ = std::stoi(payload);
            ROS_INFO_STREAM("Payload set");
            return false;
        }
        else
        {
            ROS_ERROR_STREAM("Payload isn't a integer.");
            return true;
        }
    }

    bool ProviderUnderwaterComNode::Set_Configuration(const char role, const uint8_t channel)
    {
        std::string acknowledge;
        char buffer[BUFFER_SIZE];

        sprintf(buffer, "%d", channel);
        std::string packet = "," + std::string(1, role) + "," + buffer;

        Send_CMD_To_Sensor(buffer, CMD_SET_SETTINGS, packet);

        std::stringstream ss(buffer);
        std::getline(ss, acknowledge, ',');
        std::getline(ss, acknowledge, '*');

        if(acknowledge == std::string(1, NAK))
        {
            ROS_ERROR_STREAM("Could not set the configuration.");
            return true;
        }
        ROS_INFO_STREAM("Configuration set");
        return false;
    }

    bool ProviderUnderwaterComNode::Flush_Queue()
    {
        std::string acknowledge;
        char buffer[BUFFER_SIZE];

        Send_CMD_To_Sensor(buffer, CMD_FLUSH);

        std::stringstream ss(buffer);
        std::getline(ss, acknowledge, ',');
        std::getline(ss, acknowledge, '*');

        if(acknowledge == std::string(1, NAK))
        {
            ROS_ERROR_STREAM("Couldn't flush the queue.");
            return true;
        }
        ROS_INFO_STREAM("Queue flushed");
        return false;
    }
}