/**
 * \file	serial.cc
 * \author	Lucas Mongrain
 * \date	26/10/2017
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

#include "serial.h"
#include <fcntl.h>
#include <ros/ros.h>
#include <sys/ioctl.h>

Serial::Serial(std::string port, int flags)
{
    fd = open(port.c_str(), flags);
    if(fd == -1)
    {
        ROS_ERROR("unable to connect to %s", port.c_str());
        ros::shutdown();
    }
    else
    {
        ROS_INFO("connection to %s succeed", port.c_str());
    }

    tcgetattr(fd, &options);

    // setup le baudrate
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_cflag &= ~(PARENB | PARODD);
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    //Input Flags
    options.c_iflag &= ~IGNBRK;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    //Local Flags
    options.c_lflag  = 0;

    //Output Flags
    options.c_oflag  = 0;


    tcsetattr(fd, TCSANOW, &options);
}

Serial::~Serial()
{
    close(fd);
}

std::string Serial::receive(size_t count)
{
    ROS_DEBUG("provider_underwater_com receive data");
    char data[1024];
    data[0] = 0;

    read(fd, data, count);
    return std::string(data);
}

void Serial::readOnce(char* data, int offset)
{
    ROS_DEBUG("provider_underwater_com receive Once");
    read(fd, (data+offset), 1);
}

void Serial::flush()
{
    ROS_DEBUG("provider_underwater_com flush data");
    tcflush(fd,TCIOFLUSH);
}

ssize_t Serial::transmit(const std::string data)
{
    ROS_DEBUG("provider_underwater_com transmit data");
    return write(fd, data.c_str(), data.size());
}

ssize_t Serial::transmit(const void *data, const ssize_t size)
{
    ROS_DEBUG("serial transmit data");
    return write(fd, data, size);
}