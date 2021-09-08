/**
 * \file	serial.h
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

#ifndef INTERFACE_RS485_SERIAL_H
#define INTERFACE_RS485_SERIAL_H

#include <string>
#include <string.h>
#include <termios.h>
#include <unistd.h>


class Serial{
public:
    Serial(std::string port);
    ~Serial();

    std::string receive(size_t count);
    void readOnce(char* data, int offset);
    void flush();
    ssize_t transmit(const std::string data);

private:

    struct termios options;
    int fd;
};

#endif //INTERFACE_RS485_SERIAL_H
