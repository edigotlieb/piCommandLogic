/**
 * Calib Test - the 1st test.
 *
 */
#include "mavlink/v1.0/common/mavlink.h"
#include "mavlink/v1.0/ardupilotmega/mavlink_msg_pi_trigger.h"
#include "mavlink/v1.0/ardupilotmega/mavlink_msg_pi_cam_data.h"
// Standard includes
#include <iostream>
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
#include "wrapperFunctions.h"
#include "mavlink/v1.0/pixhawk/pixhawk.h"
#define CALIBRATION_MESSAGE "piCalibrat"
#define TRIGGER "piTrigger"
#define BLUE_LED 15
#define RED_LED 14
#define MAX_NUM_OF_MESSAGES 10
#define START 1
#define STOP 0

typedef enum read_results {
    IMAGE_SENT,
    CALIB_START,
    CALIB_STOP,
    TRIGGER_RECIEVED,
    NOTHING
} read_results;

using std::string;
using namespace std;
uint16_t trigger_id;
struct timeval tv; ///< System time
int loggingLevel;
// Settings
int sysid = 42; ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 110;
int serial_compid = 0;
bool silent = false; ///< Wether console output should be enabled
bool verbose = false; ///< Enable verbose output
bool debug = false; ///< Enable debug functions and output
int fd;
ofstream logFile;



void log(int level, string message){
    if (loggingLevel>=level){
        logFile<<message;
    }
}

int sendTrigger(int serial_fd) {
    int fd = serial_fd;
    char buf[300];
    mavlink_message_t message;
    mavlink_pi_trigger_t trigger;
    trigger.trigger_id = trigger_id;
    trigger.timestamp_pi = 0; // TODO
    mavlink_msg_pi_trigger_encode(255, 1, &message, &trigger);

    unsigned len = mavlink_msg_to_send_buffer((uint8_t*) buf, &message);
    //   printf("before write\n");
    /* write packet via serial link */
    write(fd, buf, len);

    /* wait until all data has been written
    tcdrain(fd);
     */
    //     printf("after write , sys_id = %d\n",requestParam.target_system);


    return 0;

}

/**
 * @brief Serial function
 *
 * This function blocks waiting for serial data in it's own thread
 * The Main Loop - reads MAX_NUM_OF_MESSAGES messages, operates the commands from it and saves the last angle recieved
 */
read_results serial_readMSG(int serial_fd) {
    int fd = serial_fd;
    // Blocking wait for new data
    uint8_t msgReceived = false;
    mavlink_message_t message;
    uint8_t num_of_messages = 0;
    float last_angle = -181;
    while (!msgReceived){
        uint8_t cp;
        mavlink_status_t status;
        if (read(fd, &cp, 1) > 0) {
            // Check if a message could be decoded, return the message in case yes
            msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
        } else {
            //   if (!silent) fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
        }
    }
    if (msgReceived){
        switch(message.msgid){
            case MAVLINK_MSG_ID_NAMED_VALUE_INT:
                cout << "got named value"<<endl;
                mavlink_named_value_int_t named_int;
                mavlink_msg_named_value_int_decode(&message,&named_int);
                named_int.name[10]='\0';
                cout<<named_int.name<<endl;
                int rv;
                if (!strcmp(named_int.name, CALIBRATION_MESSAGE)){
                    if (named_int.value==START){
                        write_pin(BLUE_LED,1);
                        rv = take_pictures_for_calib();
                        if (rv!=0){
                            blink(RED_LED);
                            blink(RED_LED);
                            blink(RED_LED);
                        } else {
                            write_pin(RED_LED,1);
                        }
                    }
                    if (named_int.value==STOP){
                        write_pin(BLUE_LED,0);
                        write_pin(RED_LED,0);
                    }
                }
        }
    }
    return NOTHING;
}

int setupPort(char* uart_name, int baudrate) {
    // SETUP SERIAL PORT

    // Exit if opening port failed
    // Open the serial port.
    if (!silent) printf("Trying to connect to %s.. ", uart_name);
    fflush(stdout);
    fd = open_port(uart_name);
    if (fd == -1) {
        if (!silent) printf("failure, could not open port.\n");
        return (EXIT_FAILURE);
    } else {
        if (!silent) printf("success.\n");
    }
    if (!silent) printf("Trying to configure %s.. ", uart_name);
    bool setup = setup_port(fd, baudrate, 8, 1, false, false);
    if (!setup) {
        if (!silent) printf("failure, could not configure port.\n");
        return (EXIT_FAILURE);
    } else {
        if (!silent) printf("success.\n");
    }

    int noErrors = 0;
    if (fd == -1 || fd == 0) {
        if (!silent) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
        return (EXIT_FAILURE);
    } else {
        if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
    }

    if (fd < 0) {
        return (noErrors);
    }
    return 2;
}

/**
 * 1 to start, 0 to stop
 */
int sendRequestDataStream(int serial_fd, int start_stop_flag) {
    int fd = serial_fd;

    char buf[300];


    mavlink_message_t message;

    mavlink_request_data_stream_t requestDS;

    requestDS.start_stop = start_stop_flag;
    requestDS.req_stream_id = MAV_DATA_STREAM_EXTRA2;
    requestDS.req_message_rate = 10;
    requestDS.target_component = 1;
    requestDS.target_system = 1;

    mavlink_msg_request_data_stream_encode(255, 1, &message, &requestDS);
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*) buf, &message);
    //   printf("before write\n");
    /* write packet via serial link */
    write(fd, buf, len);
    /* wait until all data has been written
    tcdrain(fd);
     */

    //     printf("after write , sys_id = %d\n",requestParam.target_system);


    return 0;

}

int main(int argc, char **argv) {

    /* default values for arguments */
    char *uart_name = (char*) "/dev/ttyACM0";
    int baudrate = 115200;
    trigger_id = 0;
    loggingLevel=1; // temp
    init_pin(BLUE_LED);
    init_pin(RED_LED);
    printf("starting serial port\n");
    logFile.open("log.txt");
    int status = setupPort(uart_name, baudrate);

    if (status == EXIT_FAILURE || status == 0) {
        exit(status);
    }

    // Run indefinitely while the serial loop handles data
    printf("\nREADY, waiting for serial data.\n");
    for (int i = 0; i < 4; i++) {
        if (i % 2 == 0)
            blink(BLUE_LED);
        else
            blink(RED_LED);
    }
    read_results rv;
    while (true) {
        rv = serial_readMSG(fd);
        // check rv TODO

    }


    close_port(fd);

    return 0;
}
