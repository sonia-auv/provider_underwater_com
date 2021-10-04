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
        //serialConnection_.flush();

        underwaterComSubscriber_ = nh_->subscribe("/proc_underwater_com/send_msgs", 100, &ProviderUnderwaterComNode::UnderwaterComCallback, this);
        underwaterComPublisher_ = nh_->advertise<std_msgs::String>("/provider_underwater_com/receive_msgs", 100);
        
        read_for_packet_slave = std::thread(std::bind(&ProviderUnderwaterComNode::Read_for_Packet_Slave, this));

        char *auv;        
        auv = getenv("AUV");

        if(strcmp(auv, "AUV7") == 0)
        {
            role_ = ROLE_SLAVE;
        }
        else
        {
            role_ = ROLE_MASTER;
        }

        Set_Sensor(std::stoi(configuration_.getChannel()));

        /*if(role_ == ROLE_SLAVE)
        {
            read_for_packet_slave = std::thread(std::bind(&ProviderUnderwaterComNode::Read_for_Packet_Slave, this));
        }*/
        manage_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Manage_Packet, this));

        underwaterComService_ = nh_->advertiseService("/provider_underwater_com/request", &ProviderUnderwaterComNode::UnderwaterComService, this);
    }

    //Node Destructor
    ProviderUnderwaterComNode::~ProviderUnderwaterComNode()
    {
        manage_thread.~thread();
        read_for_packet_slave.~thread();
        underwaterComSubscriber_.shutdown();
    }

    //Node Spin
    void ProviderUnderwaterComNode::Spin()
    {
        ros::Rate r(1); // 1 Hz

        while(ros::ok() && init_completed_ == true)
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

        Send_CMD_To_Sensor(buffer, (char)req.cmd);
        
        if(ConfirmChecksum(buffer))
        {
            char cmd_rec = buffer[2];
            char tmp;
            std::stringstream ss(buffer);

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

                    tmp = role.at(0);
                    res.role = (uint8_t)tmp;
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

                    tmp = link_up.at(0);
                    res.link = (uint8_t)tmp;
                    res.packet_count = std::stoi(packet_count);
                    res.packet_count_loss = std::stoi(packet_loss_count);
                    res.bit_error_rate = std::stof(bit_error_rate);
                    break;
                }
                case CMD_FLUSH:
                {
                    ROS_DEBUG_STREAM("Flushed queue");
                    break;
                }
                default:
                {
                    ROS_ERROR("CMD received isn't working with the service. CMD received is %c", cmd_rec);
                    return false;
                }
            }
            return true;
        }
        return false;
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
            ROS_WARN_STREAM("CMD unknow. Can't queue packet");
        }
    }

    bool ProviderUnderwaterComNode::Transmit_Packet(bool pop_packet)
    {
        if(!writerQueue.empty())
        {
            serialConnection_.transmit(writerQueue.front());
            if(pop_packet) writerQueue.pop_front();
            return true;
        }
        else
        {
            ROS_WARN_STREAM("Packet isn't queue");
            return false;
        }
    }

    bool ProviderUnderwaterComNode::Read_for_Packet(char *buffer)
    {        
        ros::Rate r(2); // 2 Hz
        uint8_t cycles = 0;

        while(cycles < timeout_)
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
            ++cycles;
            r.sleep();
        }
        ROS_DEBUG_STREAM("No response after 10 secs.");
        return false;
    }

    void ProviderUnderwaterComNode::Send_CMD_To_Sensor(char *buffer, char cmd)
    {
        //writerQueue_mutex.lock();
        //readerQueue_mutex.lock();

        Queue_Packet(std::string(1, cmd));
        Transmit_Packet(true);

        std::unique_lock<std::mutex> mlock(parseQueue_mutex);
        parseQueue_cond.wait(mlock);

        if(!parseQueue.empty())
        {        
            std::string tmp = parseQueue.get_n_pop_front();

            for(uint8_t i = 0; i < tmp.size(); ++i)
            {
                buffer[i] = tmp.at(i);
            }
        }

        //writerQueue_mutex.unlock();
        //readerQueue_mutex.unlock();
    }

    void ProviderUnderwaterComNode::Send_CMD_To_Sensor(char *buffer, char cmd, std::string &packet)
    {
        //writerQueue_mutex.lock();
        //readerQueue_mutex.lock();

        Queue_Packet(std::string(1, cmd), packet);
        Transmit_Packet(true);
        
        std::unique_lock<std::mutex> mlock(parseQueue_mutex);
        parseQueue_cond.wait(mlock);

        if(!parseQueue.empty())
        {        
            std::string tmp = parseQueue.get_n_pop_front();

            for(uint8_t i = 0; i < tmp.size(); ++i)
            {
                buffer[i] = tmp.at(i);
            }
        }
        
        //writerQueue_mutex.unlock();
        //readerQueue_mutex.unlock();
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
        
        if(!writerQueue.empty())
        {
            if(send_) Transmit_Packet(false);

            send_ = false;

            if(!readerQueue.empty())
            {
                std::string tmp = readerQueue.get_n_pop_front();

                for(uint8_t i = 0; i < tmp.size(); ++i)
                {
                    buffer[i] = tmp.at(i);
                }

                if(buffer[2] == RESP_GOT_PACKET)
                {
                    Export_To_ROS(buffer);
                }

                writerQueue.pop_front();
                send_ = true;
            }
        }
    }

    void ProviderUnderwaterComNode::Manage_Packet_Slave()
    {
        if(!readerQueue.empty())
        {
            Export_To_ROS(readerQueue.get_n_pop_front());

            while(writerQueue.empty())
            {
                ros::Duration(1).sleep();
            }

            Transmit_Packet(true);
        }
    }

    void ProviderUnderwaterComNode::Manage_Packet()
    {
        ros::Rate r(1); // 1 Hz
        
        ROS_INFO_STREAM("Manage thread started");

            while(!ros::isShuttingDown())
            {
                //writerQueue_mutex.lock();

                if(role_ == ROLE_MASTER)
                {
                    Manage_Packet_Master();
                }
                else if(role_ == ROLE_SLAVE)
                {
                    Manage_Packet_Slave();
                }

                //writerQueue_mutex.unlock();
                r.sleep();
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

    void ProviderUnderwaterComNode::Read_for_Packet_Slave()
    {
        ros::Rate r(1); // 1 Hz
        char buffer[BUFFER_SIZE];
        bool new_packet = false;

        while(!ros::isShuttingDown())
        {          
            //readerQueue_mutex.lock();

            new_packet = Read_for_Packet(buffer);

            if(new_packet && ConfirmChecksum(buffer))
            {
                if(buffer[2] == RESP_GOT_PACKET)
                {
                    readerQueue.push_back(buffer);
                }
                else if(buffer[2] == CMD_QUEUE_PACKET && buffer[4] == ACK)
                {
                    ROS_INFO_STREAM("Packet queue");
                }
                else if(buffer[2] == RETURN_ERROR || buffer[2] == MALFORMED)
                {
                    ROS_ERROR_STREAM("Resquest not made properly");
                    std::unique_lock<std::mutex> mlock(parseQueue_mutex);
                    parseQueue_cond.notify_one();
                }
                else
                {
                    std::unique_lock<std::mutex> mlock(parseQueue_mutex);
                    parseQueue.push_back(buffer);
                    parseQueue_cond.notify_one();
                }
            }
            //readerQueue_mutex.unlock();
            r.sleep();
        }
    }

    void ProviderUnderwaterComNode::Set_Sensor(uint8_t channel)
    {
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
            ROS_ERROR_STREAM("Problem with the init. Node shutting down.");
            init_completed_ = false;
        }
        else
        {
            init_completed_ = true;
            ROS_INFO_STREAM("Initialisation completed");
        }
    }

    void ProviderUnderwaterComNode::Verify_Version()
    {
        std::string major_version = "";
        char buffer[BUFFER_SIZE];
        
        Send_CMD_To_Sensor(buffer, CMD_GET_VERSION);

        std::stringstream ss(buffer);
        std::getline(ss, major_version, ',');
        std::getline(ss, major_version, ',');

        if(major_version != "1" && !ConfirmChecksum(buffer))
        {
            ROS_ERROR_STREAM("Major Version isn't 1. Restarting init");
            init_error_ = true;
        }
    }

    void ProviderUnderwaterComNode::Get_Payload_Load()
    {
        std::string payload = "";
        char buffer[BUFFER_SIZE];

        Send_CMD_To_Sensor(buffer, CMD_GET_PAYLOAD_SIZE);

        std::stringstream ss(buffer);
        std::getline(ss, payload, ',');
        std::getline(ss, payload, '*');

        if((payload >= "0" || payload <= "9") && ConfirmChecksum(buffer))
        {
            payload_ = std::stoi(payload);
            ROS_DEBUG_STREAM("Payload set");
        }
        else
        {
            ROS_ERROR_STREAM("Payload isn't a integer. Restarting init");
            init_error_ = true;
        }
    }

    void ProviderUnderwaterComNode::Set_Configuration(const char &role, uint8_t channel)
    {
        std::string acknowledge;
        char buffer[BUFFER_SIZE];

        sprintf(buffer, "%d", channel);
        std::string packet = "," + std::string(1, role) + "," + buffer;

        Send_CMD_To_Sensor(buffer, CMD_SET_SETTINGS, packet);

        std::stringstream ss(buffer);
        std::getline(ss, acknowledge, ',');
        std::getline(ss, acknowledge, '*');

        if(acknowledge == std::string(1, NAK) && !ConfirmChecksum(buffer))
        {
            ROS_ERROR_STREAM("Could not set the configuration. Restarting init");
            init_error_ = true;
        }
    }

    void ProviderUnderwaterComNode::Flush_Queue()
    {
        std::string acknowledge;
        char buffer[BUFFER_SIZE];

        Send_CMD_To_Sensor(buffer, CMD_FLUSH);

        std::stringstream ss(buffer);
        std::getline(ss, acknowledge, ',');
        std::getline(ss, acknowledge, '*');

        if(acknowledge == std::string(1, NAK) && !ConfirmChecksum(buffer))
        {
            ROS_ERROR_STREAM("Couldn't flush the queue. Restarting init");
            init_error_ = true;
        }
    }
}