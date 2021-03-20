/*
 * serial.c
 *
 *  Created on: 18-4-12
 *      Author: hailiang
 */

#include "serialport.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

SerialPort::SerialPort(const char *dev, int baudrate, int databits, char check, int stopbits) {
    strcpy(dev_, dev);
    baudrate_ = baudrate;
    databits_ = databits;
    check__    = check;
    stopbits_ = stopbits;
}

SerialPort::SerialPort(const char *dev, int baudrate) {
    strcpy(dev_, dev);
    baudrate_ = baudrate;
}

SerialPort::~SerialPort() {
    if (fd_) {
        close(fd_);
    }
}

int SerialPort::openPort() {

    /* 打开串行设备,读写方式,阻塞模式 */
    if ((fd_ = open(dev_, O_RDWR | O_NOCTTY)) < 0)
        return -1;

    /* 串口配置 */
    struct termios opt = {0};

    /* 获取串口配置 */
    tcgetattr(fd_, &opt);
    bzero(&opt, sizeof(opt));

    opt.c_cflag |= CLOCAL | CREAD;
    opt.c_cflag &= ~CSIZE;   /* 清除数据位掩码 */
    opt.c_cflag &= ~CRTSCTS; /* 关闭硬件流控制 */

    /* 数据位 */
    switch (databits_) {
        case 7:
            opt.c_cflag |= CS7;
            break;
        case 8:
            opt.c_cflag |= CS8;
            break;
        default:
            opt.c_cflag |= CS8;
            break;
    }

    /* 奇偶校验 */
    switch (check__) {
        case 'O':
            /* 奇校验 */
            opt.c_cflag |= PARENB;
            opt.c_cflag |= PARODD;
            opt.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
            /* 偶校验 */
            opt.c_iflag |= (INPCK | ISTRIP);
            opt.c_cflag |= PARENB;
            opt.c_cflag &= ~PARODD;
            break;
        case 'N':
            /* 无校验 */
            opt.c_cflag &= ~PARENB;
            break;
        default:
            /* 无校验 */
            opt.c_cflag &= ~PARENB;
            break;
    }

    /* 波特率 */
    switch (baudrate_) {
        case 2400:
            cfsetispeed(&opt, B2400);
            cfsetospeed(&opt, B2400);
            break;
        case 4800:
            cfsetispeed(&opt, B4800);
            cfsetospeed(&opt, B4800);
            break;
        case 9600:
            cfsetispeed(&opt, B9600);
            cfsetospeed(&opt, B9600);
            break;
        case 115200:
            cfsetispeed(&opt, B115200);
            cfsetospeed(&opt, B115200);
            break;
        default:
            cfsetispeed(&opt, B9600);
            cfsetospeed(&opt, B9600);
            break;
    }

    /* 停止位 */
    if (stopbits_ == 1) {
        opt.c_cflag &= ~CSTOPB;
    } else if (stopbits_ == 2) {
        opt.c_cflag |= CSTOPB;
    } else {
        opt.c_cflag &= ~CSTOPB;
    }

    /* 阻塞状态有效 */
    opt.c_cc[VTIME] = 0; /* 等待的零到几百毫秒的值(单位百毫秒) */
    opt.c_cc[VMIN]  = 1; /* 等待的最小字节数 */

    /* 关闭软件控制 */
    opt.c_iflag &= ~IXON;  /* 输出软件控制 */
    opt.c_iflag &= ~IXOFF; /* 输入软件控制 */

    /* 关闭换行回车字符 */
    opt.c_iflag &= ~INLCR; /* 字符NL(0A)映射到CR(0D) */
    opt.c_iflag &= ~ICRNL; /* CR(0D)映射成字符NR(0A) */

    /* 重新设置串口 */
    tcsetattr(fd_, TCSANOW, &opt);
    /* 清空缓冲区 */
    tcflush(fd_, TCIFLUSH);

    return 0;
}

int SerialPort::closePort() {
    int err;

    err = close(fd_);
    fd_ = 0;
    return err;
}

int SerialPort::readLine(char *buff) {
    int nbytes = 0;

    while (true) {
        if (read(fd_, buff + nbytes, 1) < 0)
            return -1;

        if (buff[nbytes++] == '\n'){
            buff[nbytes] = '\0';
            break;
        }
    }

    return nbytes - 1;
}

int SerialPort::readBytes(void *buff, int nbytes) {
    return (int) read(fd_, buff, (size_t) nbytes);
}

int SerialPort::writeLine(const char *buff) {
    int k = 0;

    while (buff[k] != '\0') {
        if (write(fd_, buff + k++, 1) < 0)
            return -1;
    }

    return 0;
}

int SerialPort::writeBytes(const void *buff, int nbytes) {
    return (int) write(fd_, buff, (size_t) nbytes);
}
