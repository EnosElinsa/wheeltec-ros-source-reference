/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: serial
@filename: lsiosr.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           21-8-21     fu          new
*******************************************************/
#include "lsn10/lsiosr.h"

namespace ls {

LSIOSR * LSIOSR::instance(std::string name, int speed, int fd)
{
  static LSIOSR obj(name, speed, fd);
  return &obj;
}

LSIOSR::LSIOSR(std::string port, int baud_rate, int fd):port_(port), baud_rate_(baud_rate), fd_(fd)
{
  printf("port = %s, baud_rate = %d\n", port.c_str(), baud_rate);
}

LSIOSR::~LSIOSR()
{
  close();
}

/* Function of serial port configuration  */
int LSIOSR::setOpt()
{
  struct termios newtio, oldtio;

  if (tcgetattr(fd_, &oldtio) == -1)
  {
    perror("SetupSerial 1");
    return -1;
  }
  bzero(&newtio, sizeof(newtio));

	newtio.c_cflag |= (tcflag_t)(CLOCAL | CREAD | CS8 | CRTSCTS);
    newtio.c_cflag &= (tcflag_t) ~(CSTOPB | PARENB | PARODD);
    newtio.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                    ISIG | IEXTEN);  
    newtio.c_oflag &= (tcflag_t) ~(OPOST);
    newtio.c_iflag &= (tcflag_t) ~(IXON | IXOFF | INLCR | IGNCR | ICRNL | IGNBRK);

    newtio.c_cc[VMIN] = 0;
    newtio.c_cc[VTIME] = 0;
	
  /*Set the baud rate */
  switch (baud_rate_)
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
  case 230400:
    cfsetispeed(&newtio, B230400);
   cfsetospeed(&newtio, B230400);
    break;
  case 460800:
    cfsetispeed(&newtio, B460800);
   cfsetospeed(&newtio, B460800);
    break;
  default:
    cfsetispeed(&newtio, B9600);
   cfsetospeed(&newtio, B9600);
    break;
  }


  if ((tcsetattr(fd_, TCSANOW, &newtio)) == -1)
  {
    perror("serial set error");
    return -1;
  }

  tcflush(fd_, TCIFLUSH);

  return 0;
}

void LSIOSR::flushinput() {
  tcflush(fd_, TCIFLUSH);
}

/* Read data from the serial port  */
int LSIOSR::read(char *buffer, int length, int timeout)
{
  memset(buffer, 0, length);

  int	totalBytesRead = 0;
  int rc;
  int unlink = 0;
  char* pb = buffer;

  if (timeout > 0)
  {
    rc = waitReadable(timeout);

    if (rc <= 0)
    {
      return (rc == 0) ? 0 : -1;
    }

    int	retry = 3;
    while (length > 0)
    {
      rc = ::read(fd_, pb, (size_t)length);

      if (rc > 0)
      {	
          return rc;
      }
      else if (rc < 0)
      {
        printf("error \n");
        retry--;
        if (retry <= 0)
        {
          break;
        }
      }
	  unlink++;
      rc = waitReadable(20);

	  if(unlink > 10)
		  return -1;
	  
      if (rc <= 0)
      {
        break;
      }
    }
  }
  else
  {
    rc = ::read(fd_, pb, (size_t)length);

    if (rc > 0)
    {
      totalBytesRead += rc;
    }
    else if ((rc < 0) && (errno != EINTR) && (errno != EAGAIN))
    {
      printf("read error\n");
      return -1;
    }
  }

  return totalBytesRead;
}

int LSIOSR::waitReadable(int millis)
{
  if (fd_ < 0)
  {
    return -1;
  }
  int serial = fd_;
  
  fd_set fdset;
  struct timeval tv;
  int rc = 0;
  
  while (millis > 0)
  {
    if (millis < 5000)
    {
      tv.tv_usec = millis % 1000 * 1000;
      tv.tv_sec  = millis / 1000;

      millis = 0;
    }
    else
    {
      tv.tv_usec = 0;
      tv.tv_sec  = 5;

      millis -= 5000;
    }

    FD_ZERO(&fdset);
    FD_SET(serial, &fdset);
    
    rc = select(serial + 1, &fdset, NULL, NULL, &tv);
    if (rc > 0)
    {
      rc = (FD_ISSET(serial, &fdset)) ? 1 : -1;
      break;
    }
    else if (rc < 0)
    {
      rc = -1;
      break;
    }
  }

  return rc;
}


int LSIOSR::waitWritable(int millis)
{
  if (fd_ < 0)
  {
    return -1;
  }
  int serial = fd_;

  fd_set fdset;
  struct timeval tv;
  int rc = 0;

  while (millis > 0)
  {
    if (millis < 5000)
    {
      tv.tv_usec = millis % 1000 * 1000;
      tv.tv_sec  = millis / 1000;

      millis = 0;
    }
    else
    {
      tv.tv_usec = 0;
      tv.tv_sec  = 5;

      millis -= 5000;
    }

    FD_ZERO(&fdset);
    FD_SET(serial, &fdset);

    rc = select(serial + 1, NULL, &fdset, NULL, &tv);
    if (rc > 0)
    {
      rc = (FD_ISSET(serial, &fdset)) ? 1 : -1;
      break;
    }
    else if (rc < 0)
    {
      rc = -1;
      break;
    }
  }

  return rc;
}

/* Send data to the serial port  */
int LSIOSR::send(const char* buffer, int length, int timeout)
{
  if (fd_ < 0)
  {
    return -1;
  }

  if ((buffer == 0) || (length <= 0))
  {
    return -1;
  }

  int	totalBytesWrite = 0;
  int rc;
  char* pb = (char*)buffer;


  if (timeout > 0)
  {
    rc = waitWritable(timeout);
    if (rc <= 0)
    {
      return (rc == 0) ? 0 : -1;
    }

    int	retry = 3;
    while (length > 0)
    {
      rc = write(fd_, pb, (size_t)length);
      if (rc > 0)
      {
        length -= rc;
        pb += rc;
        totalBytesWrite += rc;

        if (length == 0)
        {
          break;
        }
      }
      else
      {
        retry--;
        if (retry <= 0)
        {
          break;
        }
      }

      rc = waitWritable(50);
      if (rc <= 0)
      {
        break;
      }
    }
  }
  else
  {
    rc = write(fd_, pb, (size_t)length);
    if (rc > 0)
    {
      totalBytesWrite += rc;
    }
    else if ((rc < 0) && (errno != EINTR) && (errno != EAGAIN))
    {
      return -1;
    }
  }

  if(0)
  {
    printf("Serial Tx: ");
    for(int i = 0; i < totalBytesWrite; i++)
    {
      printf("%02X ", (buffer[i]) & 0xFF);
    }
    printf("\n\n");
  }

  return totalBytesWrite;
}

int LSIOSR::init(int type)
{
	int error_code = 0;

	fd_ = open(port_.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);
	if (0 < fd_)
	{
		error_code = 0;
		setOpt();
		printf("open_port %s , fd %d OK !\n", port_.c_str(), fd_);
	}
	else
	{
		error_code = -1;
		if(type == 1)
			printf("open_port %s ERROR !\n", port_.c_str());
	}
  return error_code;
}

int LSIOSR::close()
{
  ::close(fd_);
}

}
