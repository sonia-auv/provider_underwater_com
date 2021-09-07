//
// Created by dev on 10/26/17.
//

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
