/** This example is public domain. */





/**
 * @file
 *   @brief The serial interface process
 *
 *   This process connects any external MAVLink UART device and prints data
 *
 *   @author Lorenz Meier, <lm@inf.ethz.ch>
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
#include "ImageProcessing.h"
#define CALIBRATION_MESSAGE "piCalibrat"
#define TRIGGER "piTrigger"
#define BLUE_LED 15
#define RED_LED 14
#define MAX_NUM_OF_MESSAGES 10
#define MAX_NUM_OF_TREES 10
#define MAX_ANGLE_TO_PROCESS 10

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
FilterApplier fa;
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
//flags
bool triggerFlag;


void log(int level, string message) {
    if (loggingLevel >= level) {
        logFile << message;
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

int sendCamData(int serial_fd, int trees[][]) {
    int fd = serial_fd;
    char buf[300];
    mavlink_message_t message;
    mavlink_pi_cam_data_t data;
    data.start_tree_1 = trees[0][0];
    data.end_tree_1 = trees[0][1];
    data.start_tree_2 = trees[1][0];
    data.end_tree_2 = trees[1][1];
    data.start_tree_3 = trees[2][0];
    data.end_tree_3 = trees[2][1];
    data.start_tree_4 = trees[3][0];
    data.end_tree_4 = trees[3][1];
    data.start_tree_5 = trees[4][0];
    data.end_tree_5 = trees[4][1];
    data.start_tree_6 = trees[5][0];
    data.end_tree_6 = trees[5][1];
    data.start_tree_7 = trees[6][0];
    data.end_tree_7 = trees[6][1];
    data.start_tree_8 = trees[7][0];
    data.end_tree_8 = trees[7][1];
    data.start_tree_9 = trees[8][0];
    data.end_tree_9 = trees[8][1];
    data.start_tree_10 = trees[9][0];
    data.end_tree_10 = trees[9][1];
    data.data_id = trigger_id;
    mavlink_msg_pi_cam_data_encode(255, 1, &message, &data);

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
    while (num_of_messages < MAX_NUM_OF_MESSAGES) {
        //if (debug) printf("Checking for new data on serial port\n");
        // Block until data is available, read only one byte to be able to continue immediately
        //char buf[MAVLINK_MAX_PACKET_LEN];
        uint8_t cp;
        mavlink_status_t status;
        if (read(fd, &cp, 1) > 0) {
            // Check if a message could be decoded, return the message in case yes
            msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
            if (msgReceived) {
                num_of_messages++;
            }
        } else {
            //   if (!silent) fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
        }
        if (msgReceived) {
            //printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
            /* decode and print */

            switch (message.msgid) {
                case MAVLINK_MSG_ID_ATTITUDE: // new angle
                {
                    mavlink_attitude_t attitude;
                    mavlink_msg_attitude_decode(&message, &attitude);
                    if (triggerFlag){
                    last_angle = attitude.roll;
                    }

                    //        printf("Got message ATTITUDE \n");
                    //        printf("\t YAW:\t% f\tPITCH:\t% f\tROLL:\t% f \n", attitude.yaw, attitude.pitch, attitude.roll);
                    //return IMAGE_SENT;
                }
                    break;
                case MAVLINK_MSG_ID_NAMED_VALUE_INT:
                {
                    int rv;
                    printf("got named value\n");
                    mavlink_named_value_int_t named_int;
                    mavlink_msg_named_value_int_decode(&message, &named_int);
                    named_int.name[10] = '\0';
                    printf("value: %s\n", named_int.name);
                    cout << named_int.value << endl;
                    if (!strcmp(named_int.name, CALIBRATION_MESSAGE)) {
                        if (named_int.value == 1) {
                            cout << "Calib Stop" << endl;
                            ~fa;
                            fa = FilterApplier();
                            if(!fa.isOpened()){
                                blink(RED_LED);
                                blink(BLUE_LED);
                                blink(RED_LED);
                                blink(BLUE_LED);
                                blink(RED_LED);
                                blink(BLUE_LED);
                                blink(RED_LED);
                                blink(BLUE_LED);
                            }
                            write_pin(BLUE_LED, 0);
                            write_pin(RED_LED,0);
                        } else {
                            cout << "Calib Start" << endl;
                            write_pin(BLUE_LED, 1);
                            rv = take_pictures_for_calib();
                            if (rv != 0) {
                                blink(RED_LED);
                                blink(RED_LED);
                                blink(RED_LED);
                            } else {
                                write_pin(RED_LED, 1);
                            }
                            
                        }
                    } else if (!strcmp(named_int.name, TRIGGER)) {
                        if (named_int.value == 1) {
                            // flag
                            printf("*** Starting Trig ***\n");
                            write_pin(RED_LED, 1);
                            triggerFlag = true;
                            //return TRIGGER_RECIEVED;
                        } else {
                            // same flag
                            cout << "*** Stopping Trig ***" << endl;
                            write_pin(RED_LED, 0);
                            int trees[MAX_NUM_OF_TREES][2];
                            for (int i=0;i<MAX_NUM_OF_TREES;i++){
                                trees[i][0]=trees[i][1]=0;
                            }
                            trees[0][0]=4;
                            sendCamData(fd,trees);
                            //return TRIGGER_RECIEVED;TRIGGER
                        }
                    }
                }
                default:
                {
                    //return NOTHING;
                }
            }
            msgReceived = false;
        }
    }
    // If a message could be decoded, handle it
    if (last_angle != -181 && abs(last_angle) < MAX_ANGLE_TO_PROCESS) {
        // send trigger
        int rv = sendTrigger(fd);
        // capture & process
        int trees[MAX_NUM_OF_TREES][2];
        rv = fa.findTrees(trees);
        // send data
        sendCamData(fd, trees);
        trigger_id++;

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
    triggerFlag=false;
    loggingLevel = 1; // temp
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

