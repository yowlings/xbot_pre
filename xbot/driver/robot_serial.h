#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include "robot_frame.h"
#ifndef ROBOT_SERIAL_H
#define ROBOT_SERIAL_H
class RSerial
{
 public:
    int fd;
    RSerial();
    int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
    int open_port(int fd,int comport);
    int initial(char *dev= "/dev/ttyUSB0");
    int swrite(Frame &f);
    int sread(Frame &f);
};








#endif // ROBOT_SERIAL_H
