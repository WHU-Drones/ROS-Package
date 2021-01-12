/*
 * serialport.h
 *
 *  Created on: 18-12-01
 *      Author: hailiang
 */

#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H

class SerialPort {

public:
    SerialPort(const char *dev, int baudrate, int databits, char check, int stopbits);
    SerialPort(const char *dev, int baudrate);
    ~SerialPort();

public:
    int openPort();
    int closePort();

    int readLine(char *buff);
    int readBytes( void *buff, int nbytes);

    int writeLine(const char *buff);
    int writeBytes(const void *buff, int nbytes);

private:
    int fd_;

    char dev_[32];
    int  baudrate_;
    int  databits_;
    char check__;
    int  stopbits_;
};

#endif /* _SERIAL_PORT_H */
