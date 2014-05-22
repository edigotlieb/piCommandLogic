/* 
 * File:   wrapperFunctions.h
 * Author: udi
 *
 * Created on May 20, 2014, 4:09 PM
 */

#ifndef WRAPPERFUNCTIONS_H
#define	WRAPPERFUNCTIONS_H
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>
#include <sstream>

#include "opencv2/opencv.hpp"

#define PICTUERS_TO_CALIB 4
#define BLUE_LED 15
#define RED_LED 14
using std::string;
using namespace cv;
using namespace std;

std::string numberToString(int num) {
    ostringstream strout;
    string str;
    strout << num;
    str = strout.str();
    return str;
}

int init_pin(int pin) {
    string pin_str = numberToString(pin);
    string dir = "/sys/class/gpio/export";
    ofstream file(dir.c_str());
    if (file < 0) {
        //error
        return -1;
    }
    file << pin;
    file.close();
    dir = "/sys/class/gpio/gpio" + pin_str + "/direction";
    cout << dir << endl;
    file.open(dir.c_str());
    if (file < 0) {
        //error
        return -1;
    }
    file << "out";
    file.close();
    return 1;
}

int write_pin(int pin, int value) {
    if (value != 0 && value != 1) {
        //error
        return -1;
    }
    string value_str = numberToString(value);
    string pin_str = numberToString(pin);
    string dir = "/sys/class/gpio/gpio" + pin_str + "/value";
    ofstream file(dir.c_str());
    if (file < 0) {
        return -1;
    }
    file << value_str;
    file.close();
    return 1;
}

int read_pin(int pin){
    string pin_str = numberToString(pin);
    string dir = "/sys/class/gpio/gpio" + pin_str + "/value";
    ifstream file(dir.c_str());
    if (file < 0) {
        return -1;
    }
    string value;
    file >> value;
    return atoi(value.c_str());
}

void blink(int pin) {
    
    write_pin(pin, 1);
    usleep(250000);
    write_pin(pin, 0);
    usleep(250000);
}

void blink(int pin,int sleep_time) {
    int value = read_pin(pin);
    cout <<"old value "<< value<<endl;
    if (value!=0 && value!=1){
    value=0;
    }
    value = (value+1)%2;
    cout << "new value " <<value<<endl;
    write_pin(value,1);
    usleep(sleep_time);
    write_pin((value+1)%2,0);
    usleep(sleep_time);
}

/**
 *
 *
 * Returns the file descriptor on success or -1 on error.
 */
int open_port(const char* port) {
    int fd; /* File descriptor for the port */

    // Open serial port
    // O_RDWR - Read and write
    // O_NOCTTY - Ignore special chars like CTRL-C
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        /* Could not open the port. */
        return (-1);
    } else {
        fcntl(fd, F_SETFL, 0);
    }

    return (fd);
}

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control) {
    //struct termios options;

    struct termios config;
    if (!isatty(fd)) {
        fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
        return false;
    }
    if (tcgetattr(fd, &config) < 0) {
        fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
        return false;
    }
    //
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
            INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
            ONOCR | OFILL | OPOST);

#ifdef OLCUC
    config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
    config.c_oflag &= ~ONOEOT;
#endif

    //
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    //
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    //
    // One input byte is enough to return from read()
    // Inter-character timer off
    //
    config.c_cc[VMIN] = 1;
    config.c_cc[VTIME] = 10; // was 0

    // Get the current options for the port
    //tcgetattr(fd, &options);

    switch (baud) {
        case 1200:
            if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 1800:
            cfsetispeed(&config, B1800);
            cfsetospeed(&config, B1800);
            break;
        case 9600:
            cfsetispeed(&config, B9600);
            cfsetospeed(&config, B9600);
            break;
        case 19200:
            cfsetispeed(&config, B19200);
            cfsetospeed(&config, B19200);
            break;
        case 38400:
            if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 57600:
            if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 115200:
            if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;

            // These two non-standard (by the 70'ties ) rates are fully supported on
            // current Debian and Mac OS versions (tested since 2010).
        case 460800:
            if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 921600:
            if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0) {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        default:
            fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
            return false;

            break;
    }

    //
    // Finally, apply the configuration
    //
    if (tcsetattr(fd, TCSAFLUSH, &config) < 0) {
        fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
        return false;
    }
    return true;
}

void close_port(int fd) {
    close(fd);
}
/**
 * Take PICTURES_TO_CALIB and save them to files.
 * @return 
 */
int take_pictures_for_calib(){
    cv::VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    if (!cap.isOpened()){
	cout <<"can't open camera" <<endl;
        return -1;
    }
    Mat mat;
    string filename;
    for (int i=0;i<PICTUERS_TO_CALIB;i++){
        blink(RED_LED);
        cap >> mat;
        filename = "calib_"+numberToString(i)+".jpg";
        imwrite(filename,mat);
	usleep(5000000);
    }
    return 0;
}


#endif	/* WRAPPERFUNCTIONS_H */

