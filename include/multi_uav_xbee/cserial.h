#ifndef CSERIAL_H
#define CSERIAL_H

#include <string>
#include <sstream>
#include <iostream>
#include <ctime>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

namespace multi_uav_xbee{

class CSerial {
private:
    int fd;

public:
    CSerial();
    ~CSerial();

    bool openPort(std::string port, int baud = 9600);
    bool closePort();

    std::string readData();
    bool writeData(std::string data);

};

}

#endif // CSERIAL_H
