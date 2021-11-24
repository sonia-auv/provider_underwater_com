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
#include <fcntl.h>

namespace provider_underwater_com
{

    //Node Construtor
    ProviderUnderwaterComNode::ProviderUnderwaterComNode(const ros::NodeHandlePtr &_nh)
        : nh_(_nh), configuration_(_nh), serialConnection_(configuration_.getTtyPort(), O_RDWR | O_NOCTTY)
    {
        underwaterComSubscriber_ = nh_->subscribe("/proc_underwater_com/send_msgs", 100, &ProviderUnderwaterComNode::UnderwaterComCallback, this);
        underwaterComPublisher_ = nh_->advertise<sonia_common::IntersubCom>("/provider_underwater_com/receive_msgs", 100);
        
        manage_write_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Manage_Write, this));
        read_packet_thread = std::thread(std::bind(&ProviderUnderwaterComNode::Read_Packet, this));
        
        ros::Duration(1).sleep();

        ROS_INFO_STREAM("Setting the sensor");
        Set_Sensor(configuration_.getRole().at(0), std::stoi(configuration_.getChannel()));

        underwaterComService_ = nh_->advertiseService("/provider_underwater_com/request", &ProviderUnderwaterComNode::UnderwaterComService, this);
    }

    //Node Destructor
    ProviderUnderwaterComNode::~ProviderUnderwaterComNode()
    {
        manage_write_thread.~thread();
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

    void ProviderUnderwaterComNode::UnderwaterComCallback(const sonia_common::IntersubCom &msg)
    {
        uint64_t packet = 0;
        char packet_array[MODEM_M64_PAYLOAD];

        // Add function for multiple paquet support
        modem_data.header.endOfPacket = 0b1;
        modem_data.header.packetId = 0b0;
        modem_data.header.packetNumber = 0b1;

        modem_data.killSwitchState = msg.kill_switch_state;
        modem_data.missionSwitchState = msg.mission_switch_state;
        modem_data.depth = msg.depth;
        modem_data.missionId = msg.mission_id;
        modem_data.missionState = (uint8_t) msg.mission_state;
        modem_data.torpedosState = msg.torpedos_state;
        modem_data.droppersState = msg.droppers_state;

        packet = *((uint64_t *)&modem_data);

        for(uint8_t i = 0; i < MODEM_M64_PAYLOAD; ++i)
        {
            packet_array[i] = (char) (packet >> i * MODEM_M64_PAYLOAD);
        }

        Queue_Packet(CMD_QUEUE_PACKET, packet_array, MODEM_M64_PAYLOAD);
    }

    bool ProviderUnderwaterComNode::UnderwaterComService(sonia_common::ModemSendCmd::Request &req, sonia_common::ModemSendCmd::Response &res)
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

    uint8_t ProviderUnderwaterComNode::Calculate_Checksum(const char (&buffer)[BUFFER_SIZE], const size_t size)
    {
        uint8_t check = 0;

        for(uint8_t i = 0; i < size; ++i)
        {
            check = crc_table[(uint8_t)buffer[i] ^ check];
        }

        return check;
    }

    uint8_t ProviderUnderwaterComNode::Append_Checksum(char (&buffer)[BUFFER_SIZE], const size_t size)
    {
        char checksum_buffer[2];

        uint8_t checksum = Calculate_Checksum(buffer, size);
        sprintf(checksum_buffer, "%02x", checksum);

        buffer[size] = CHECKSUM;
        buffer[size + 1] = checksum_buffer[0];
        buffer[size + 2] = checksum_buffer[1];
        buffer[size + 3] = EOP;

        return size + 4;
    }

    bool ProviderUnderwaterComNode::Confirm_Checksum(char (&buffer)[BUFFER_SIZE], const size_t size)
    {    
        char checksumData[BUFFER_SIZE];

        try
        {
            uint8_t position = Find_Character(buffer, CHECKSUM, size);
            Copy_Array(buffer, checksumData, position);
            uint8_t calculatedChecksum = Calculate_Checksum(checksumData, position);
            std::string checksumSentence {buffer[position + 1], buffer[position + 2]};
            uint8_t originalChecksum = std::stoi(checksumSentence, nullptr, 16);
            return originalChecksum == calculatedChecksum;
        }
        catch(...)
        {
            ROS_WARN_STREAM("Underwater Com: bad checksum");
            return false;
        }
    }

    void ProviderUnderwaterComNode::Queue_Packet(const char cmd, const char (&packet)[MODEM_M64_PAYLOAD], const size_t size_packet)
    {
        char send_msg[BUFFER_SIZE];
        uint8_t index = 0;

        if(Check_CMD(&cmd))
        {
            send_msg[0] = SOP;
            send_msg[1] = DIR_CMD;
            send_msg[2] = cmd;
            index = 3;

            if(cmd == CMD_QUEUE_PACKET)
            {
                send_msg[3] = ',';
                send_msg[4] = std::to_string(payload_).at(0);
                send_msg[5] = ',';
                index = 6;
                Append_Packet(send_msg, index, packet, size_packet);
            }
            else if(cmd == CMD_SET_SETTINGS)
            {
                send_msg[3] = ',';
                index = 4;
                Append_Packet(send_msg, index, packet, size_packet);
            }

            index = Append_Checksum(send_msg, index + size_packet);      
            std::unique_lock<std::mutex> mlock(write_mutex); 
            std::copy(std::begin(send_msg), std::end(send_msg), std::begin(writeBuffer));
            writeSize = index;
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
        if(writeBuffer[0] != 0)
        {
            serialConnection_.transmit(writeBuffer, writeSize);
            if(pop_packet)
            {
                writeBuffer[0] = 0;
                writeSize = 0;
            }
            return true;
        }
        else
        {
            ROS_WARN_STREAM("Packet isn't queue");
            return false;
        }
    }

    bool ProviderUnderwaterComNode::Send_CMD_To_Sensor(char *buffer, char cmd, const char (&packet)[MODEM_M64_PAYLOAD], size_t size)
    {
        Queue_Packet(cmd, packet, size);
        std::unique_lock<std::mutex> mlock(parse_mutex);
        parse_cond.wait(mlock);

        if(!parse_string.empty())
        {        
            std::string tmp = parse_string;
            parse_string.erase();

            for(uint8_t i = 0; i < tmp.size(); ++i)
            {
                buffer[i] = tmp.at(i);
            }
            return false;
        }
        return true;
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

    void ProviderUnderwaterComNode::Append_Packet(char (&buffer)[BUFFER_SIZE], const size_t index, const char (&packet)[MODEM_M64_PAYLOAD], const size_t size_packet)
    {
        for(uint8_t i = 0; i < size_packet; ++i)
        {
            buffer[index+i] = packet[i];
        }
    }

    uint8_t ProviderUnderwaterComNode::Find_Character(const char (&buffer)[BUFFER_SIZE], const char to_find, const size_t size)
    {
        uint8_t i;

        for(i = 0; i < size; ++i)
        {
            if(buffer[i] == to_find) break;
        }
        return i;
    }

    void ProviderUnderwaterComNode::Copy_Array(const char (&buffer)[BUFFER_SIZE], char (&buffer_returned)[BUFFER_SIZE], const size_t size, const size_t start)
    {
        for(uint8_t i = 0; i < size; ++i)
        {
            buffer_returned[i+start] = buffer[i];
        }
    }

    void ProviderUnderwaterComNode::Manage_Write()
    { 
        ros::Rate r(1); // 1 Hz
        ROS_INFO_STREAM("Manage write thread started");

        while(!ros::isShuttingDown())
        {
            std::unique_lock<std::mutex> mlock(write_mutex);
            write_cond.wait(mlock);
            Transmit_Packet(true);
        }
    }

    void ProviderUnderwaterComNode::Export_To_ROS(const char (&buffer)[BUFFER_SIZE], const ssize_t size)
    {
        Modem_M64_t packet;
        uint64_t data = 0;
        char tmp[BUFFER_SIZE];
        // TODO add a size check before publish and check if message good

        uint8_t first = Find_Character(buffer, ',', size);
        Copy_Array(buffer, tmp, MODEM_M64_PAYLOAD, first + 1);

        for(uint8_t i = 0; i < MODEM_M64_PAYLOAD; ++i)
        {
            if(i != 0) data += tmp[i] * (i * 8);
            else data += tmp[i];
        }
        packet = *((Modem_M64_t *)&data);

        msg.depth = packet.depth;
        msg.kill_switch_state = packet.killSwitchState;
        msg.mission_switch_state = packet.missionSwitchState;
        msg.mission_id = packet.missionId;
        msg.mission_state = (int8_t) packet.missionState;
        msg.torpedos_state = packet.torpedosState;
        msg.droppers_state = packet.droppersState;  

        underwaterComPublisher_.publish(msg);
    }

    void ProviderUnderwaterComNode::Read_Packet()
    {
        uint8_t i;
        char buffer[BUFFER_SIZE];

        ROS_INFO_STREAM("Read Packet thread started");

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

            if(Confirm_Checksum(buffer, i-1))
            {
                if(buffer[2] == RESP_GOT_PACKET)
                {
                    Export_To_ROS(buffer, i-1);
                }
                else if(buffer[2] == CMD_QUEUE_PACKET && buffer[4] == ACK)
                {
                    ROS_DEBUG_STREAM("Packet queue");
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
        
        if(Send_CMD_To_Sensor(buffer, CMD_GET_VERSION)) return true;

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

        if(Send_CMD_To_Sensor(buffer, CMD_GET_PAYLOAD_SIZE)) return true;

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
        char packet[MODEM_M64_PAYLOAD] = {role, ','}; 

        sprintf(buffer, "%d", channel);
        packet[2] = buffer[0];

        if(Send_CMD_To_Sensor(buffer, CMD_SET_SETTINGS, packet, 3)) return true;

        std::stringstream ss(buffer);
        std::getline(ss, acknowledge, ',');
        std::getline(ss, acknowledge, '*');

        if(acknowledge != std::string(1, ACK))
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

        if(Send_CMD_To_Sensor(buffer, CMD_FLUSH)) return true;

        std::stringstream ss(buffer);
        std::getline(ss, acknowledge, ',');
        std::getline(ss, acknowledge, '*');

        if(acknowledge != std::string(1, ACK))
        {
            ROS_ERROR_STREAM("Couldn't flush the queue.");
            return true;
        }
        ROS_INFO_STREAM("Queue flushed");
        return false;
    }
}