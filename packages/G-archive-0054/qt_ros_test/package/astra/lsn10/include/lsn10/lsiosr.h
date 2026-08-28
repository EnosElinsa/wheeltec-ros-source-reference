/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: serial
@filename: lsiosr.h
@brief:
@version:       date:       author:     comments:
@v1.0           21-8-21     fu          new
*******************************************************/
#ifndef LSIOSR_H
#define LSIOSR_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <fstream>
#include <iostream>

#define     BAUD_2400       2400
#define     BAUD_4800       4800
#define     BAUD_9600       9600
#define     BAUD_57600     	57600
#define     BAUD_115200     115200
#define		  BAUD_230400		  230400
#define     BAUD_460800     460800

namespace ls
{
class LSIOSR{
public:
  static LSIOSR* instance(std::string name, int speed, int fd = 0);

  ~LSIOSR();

  /* Read data from the serial port  */
  int read(char *buffer, int length, int timeout = 30);

  /* Send data to the serial port  */
  int send(const char* buffer, int length, int timeout = 30);

  /* Empty serial port input buffer */
  void flushinput();

  /* Serial port initialization  */
  int init(int type = 2);

  int close();

private:
  LSIOSR(std::string name, int speed, int fd);

  int waitWritable(int millis);
  int waitReadable(int millis);

  /* Serial port configuration  */
  int setOpt();

  std::string port_;

  int baud_rate_;

  int fd_;
};
}
#endif

