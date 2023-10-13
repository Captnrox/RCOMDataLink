// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>

typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

typedef struct
{
    bool stuffed;
    unsigned char byte1;
    unsigned char byte2;

} StuffingAux;

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer

// 1000 - 3 from PA means each image chunk will have 997 bytes
#define MAX_PAYLOAD_SIZE 1000

// MISC
#define FALSE 0
#define TRUE 1

// UA
#define FLAG 0x7E
#define A_UA 0x03
#define C_UA 0x07
#define BCC1_UA A_UA ^ C_UA

// SET
#define A_SET 0x03
#define C_SET 0x03
#define BCC1_SET A_SET ^ C_SET

// STATE MACHINE
#define START_ST 0
#define FLAG_RCV 1
#define A_RCV 2
#define C_RCV 3
#define BCC_OK 4

// INFORMATION FRAMES
#define A_SENT_BY_TX 0x03

// Open a connection using the "port" parameters defined in struct linkLayer.
// Return "1" on success or "-1" on error.
int llopen(LinkLayer connectionParameters);

// Open a connection using the "port" parameters defined in struct linkLayer - Transmitter Side
// Return "1" on success or "-1" on error.
int llopen_transmitter(LinkLayer connectionParameters);

// Open a connection using the "port" parameters defined in struct linkLayer - Receiver Side
// Return "1" on success or "-1" on error.
int llopen_transmitter(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or "-1" on error.
int llwrite(const unsigned char *buf, int bufSize);

// Receive data in packet.
// Return number of chars read, or "-1" on error.
int llread(unsigned char *packet);

// Close previously opened connection.
// if showStatistics == TRUE, link layer should print statistics in the console on close.
// Return "1" on success or "-1" on error.
int llclose(int showStatistics);

// Escapes flags and escape characters in byte
// Returns a struct containg a bool, that indicates if the byte required stuffing or not, and the corresponding stuffed sequence
StuffingAux stuff_byte(unsigned char byte);

// Translates escape sequences in the received bytes
// Return a struct containing a bool, that indicates if the bytes required destuffing ot not, and the corresponding destuffed sequence
StuffingAux destuff_byte(unsigned char byte1, unsigned char byte2);

// Creates information frame with the data passed as parameter and the corresponding frame number
// Places the created frame in the result array
void create_inf_frame(unsigned char *data, unsigned n, bool frame_num, unsigned char *result);

// Destuffs the bytes in a frame
// Places the result in destuffed_frame, an array corresponding to the frame information after translation
void destuff_frame(unsigned char *frame, unsigned n, unsigned char *destuffed_frame);

//Handles alarm interrupts
void alarmHandler(int signal);

#endif // _LINK_LAYER_H_
