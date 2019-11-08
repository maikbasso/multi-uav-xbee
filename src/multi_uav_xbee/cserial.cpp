#include "multi_uav_xbee/cserial.h"

namespace multi_uav_xbee{

CSerial::CSerial(){

}

CSerial::~CSerial(){
    CSerial::closePort();
}

bool CSerial::openPort(std::string port, int baud){

    this->fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (this->fd != -1){
        // Read the configureation of the port

          struct termios options;
          tcgetattr( this->fd, &options );

          /* SEt Baud Rate */

          switch (baud) {
          case 57600:
              cfsetispeed( &options, B57600 );
              cfsetospeed( &options, B57600 );
              break;
          case 9600:
              cfsetispeed( &options, B9600 );
              cfsetospeed( &options, B9600 );
              break;
          case 115200:
              cfsetispeed( &options, B115200 );
              cfsetospeed( &options, B115200 );
              break;
          default:
              cfsetispeed( &options, B9600 );
              cfsetospeed( &options, B9600 );
              break;
          }

          //I don't know what this is exactly

          options.c_cflag |= ( CLOCAL | CREAD );

          // Set the Charactor size

          options.c_cflag &= ~CSIZE; /* Mask the character size bits */
          options.c_cflag |= CS8;    /* Select 8 data bits */

          // Set parity - No Parity (8N1)

          options.c_cflag &= ~PARENB;
          options.c_cflag &= ~CSTOPB;
          options.c_cflag &= ~CSIZE;
          options.c_cflag |= CS8;

          // Disable Hardware flowcontrol

          //  options.c_cflag &= ~CNEW_RTSCTS;  -- not supported

          // Enable Raw Input

          options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

          // Disable Software Flow control

          options.c_iflag &= ~(IXON | IXOFF | IXANY);

          // Chose raw (not processed) output

          options.c_oflag &= ~OPOST;

          if ( tcsetattr(this->fd, TCSANOW, &options ) == -1 )
            std::cout << "Error with tcsetattr!" << std::endl;

          fcntl(this->fd, F_SETFL, FNDELAY);

        return true;
    }
    else{
        fcntl(fd, F_SETFL, FNDELAY);
        return false;
    }
}

bool CSerial::closePort(){

    close(this->fd);

    return true;
}

std::string CSerial::readData(){

  char data = 0;
  std::stringstream ss;
  int dwBytesRead = 0;

  do
  {
      // Read data from the COM-port
      dwBytesRead = read(this->fd, &data, 1);
      if ((dwBytesRead > 0) && (data != '\n')) ss << data;
  }
  while (data != '\n');

  return ss.str();

}

bool CSerial::writeData(std::string data){
    data = data + '\n';
    if(write(this->fd, data.c_str(), data.size()) < 0){
        return false;
    }
    else{
        return true;
    }
}


}
