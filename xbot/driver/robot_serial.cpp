#include "robot_serial.h"
#include "robot_frame.h"
#include <queue>
RSerial::RSerial()
{

}
int RSerial::initial(char *dev)
{
        fd = open(dev, O_RDWR|O_NOCTTY|O_NDELAY); //dev
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ");
            printf("%s\n",dev);
        }

    if(fcntl(fd, F_SETFL, 0)<0)
    {
        printf("fcntl failed!\n");
    }
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    }
    if(isatty(STDIN_FILENO)==0)
    {
        printf("standard input is not a terminal device\n");
    }
    else
    {
        printf("isatty success!\n");
    }
    printf("fd-open=%d\n",fd);

    if(set_opt(fd,115200,8,'N',1)<0)
    {
        perror("set_opt error");
        return -1;
    }
    printf("fd=%d\n",fd);


    return fd;
}

int RSerial::set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if  ( tcgetattr( fd,&oldtio)  !=  0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':                     //奇校验
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':                     //偶校验
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':                    //无校验
        newtio.c_cflag &= ~PARENB;
        break;
    }

switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
    {
        newtio.c_cflag &=  ~CSTOPB;
    }
    else if ( nStop == 2 )
    {
        newtio.c_cflag |=  CSTOPB;
    }
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}

int RSerial::swrite(Frame &f)
{
    int ncount=write(fd, f.get_frame(), f.get_frame_len());
    if(ncount==-1)
    {
        printf("Wirte sbuf error./n");
        return -1;
    }
    else
    {
//        printf("write %d bytes \n",ncount);
//        printf("writebuff is ");
//        for(int i=0;i<ncount;i++)
//        {
//            printf("%02x ",*( f.get_frame()+i));
//        }
//        printf("\n");
    }
    return 0;
}

int RSerial::sread(Frame &f)//data->frame->process
{
    char readbuff[512];
    char buffer[512];
  //  f.set_rbuff_len(readbuff_len);
    int nread=read(fd,readbuff,512);
    if(nread>0)
    {
//            printf("Readbuff length is %d\n",nread);
            for(int i=0;i<nread;i++)
            {
//                printf("%02x",readbuff[i]);
                f.rbuff_q.push(readbuff[i]);
            }
           f.update_sensor_data();
    }
     if(nread==-1)
    {
        printf("read error\n");
    }
 //   f.set_rbuff_len(buffer_len);
}
