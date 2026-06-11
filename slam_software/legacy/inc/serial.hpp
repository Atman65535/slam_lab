#include <cstring>
#include <termios.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <iostream>


class Serial {
public:
    // ~Serial() {close(fd);}
    Serial(std::string const serial_name,
           int baudrate) {
     fd = open(serial_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
     if (fd < 0) {
         throw "Error opening serial";
     }
     else {
         std::cout << "Open fd" << fd << std::endl;
     }
     // struct termios tty;
     // if (tcgetattr(fd, &tty) != 0) {
        
     // }
     // if (baudrate == 921600) {
     //     cfsetospeed(&tty, B921600);
     //     cfsetispeed(&tty, B921600);
     // }
     // else if (baudrate == 460800) {
     //     cfsetospeed(&tty, B460800);
     //     cfsetispeed(&tty, B460800);
     // }
     // else {
     //     std::cout <<"no implementation" << std::endl;
     //     exit(1);
     // }

     // tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // CSIZE the character size mask
     // tty.c_cflag &= ~IGNBRK; // disable break processing
    
     // tty.c_lflag = 0;
    
     // tty.c_oflag = 0;
    
     // tty.c_cc[VMIN] = 0; // read doesn't block
     // tty.c_cc[VTIME] = 1; // 0.5 second read timeout

     // tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

     // tty.c_cflag |= (CLOCAL | CREAD); // ignore moden controls, enable reading;
     // tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
     // tty.c_cflag &= ~CSTOPB;
     // tty.c_cflag &= ~CRTSCTS;
     // if (tcsetattr(fd, TCSANOW, &tty) == 0) {
     //     std::cout << "set " + serial_name << "succey" << std::endl;
     // }
        fd = open(serial_name.c_str(),
                  O_RDWR | O_NOCTTY | O_SYNC);
        if (fd == -1)
            std::cout << "Error opening device " + serial_name << std::endl;
        tcgetattr(fd, &tty);

        cfsetospeed(&tty, B921600);
        cfsetispeed(&tty, B921600);

        tty.c_iflag |= IGNBRK; // ignore 0x00 break
        tty.c_iflag &= ~(INPCK | ISTRIP | INLCR);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // special byte command ? Fuck off

        tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
        tty.c_oflag = 0; // no output, but still 

        // non block mode thus the two are aborted
        tty.c_cc[VMIN] = 0; 
        tty.c_cc[VTIME] = 5; 


        tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls, enable receive
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit 
        tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
        tty.c_cflag &= ~CSTOPB;                     // one stop b
        tty.c_cflag &= ~CRTSCTS;                    // no hardware ctl
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0)
            std::cout << "Error save config" << std::endl;
    }
    size_t r(void* buf, size_t nbytes) {
        return read(fd, buf, nbytes);
    }
private:
    int fd;
    struct termios tty;
};
