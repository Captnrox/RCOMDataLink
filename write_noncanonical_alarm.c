// Write to serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

//States

#define START_ST 0
#define FLAG_RCV 1
#define A_RCV 2
#define C_RCV 3
#define BCC_OK 4

#define FLAG 0x7E
#define A_UA 0x03
#define C_UA 0x07
#define BCC1_UA A_UA ^ C_UA

#define FALSE 0
#define TRUE 1

#define BUF_SIZE 5

volatile int STOP = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

int main(int argc, char *argv[])
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // the read() func will return immediately, with either the number of bytes currently available in the receiver buffer, or the number of bytes requested

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    // Create string to send
    unsigned char A_SEND = 0x03;
    unsigned char C_SEND = 0x03;
    unsigned char BCC1 = A_SEND ^ C_SEND;

    unsigned char buf[BUF_SIZE] = {FLAG, A_SEND, C_SEND, BCC1, FLAG};
    unsigned char received[BUF_SIZE] = {0};

    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.
    // buf[5] = '\n';

    for (int i = 0; i < BUF_SIZE; i++)
    {
        printf("0x%02X\n", buf[i]);
    }

    // Wait until all bytes have been written to the serial port
    // sleep(1);

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    unsigned char count = 0;
    unsigned char input[1] = {0};
    unsigned char state = START_ST;

    while (alarmCount < 3 && STOP == FALSE)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(3); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;

            int write_bytes = write(fd, buf, BUF_SIZE);
            printf("%d bytes written\n", write_bytes);
        }

        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);
        if (read_bytes)
        {
            received[count] = input[0];
            count++;
        }

        switch (state)
        {
        case START_ST:
        {
            if (received[count] == FLAG)
                state = FLAG_RCV;
            break;
        }
        case FLAG_RCV:
        {
            if (received[count] == A_UA) //This is specific state machine for receiving UA that exits as soon as a wrong char is read, in a more general state machine, we don't verify what we receive here
                state = A_RCV;
            else if (received[count] != FLAG)
                state = START_ST;
            break;
        }
        case A_RCV:
        {
            if (received[count] == C_UA)
                state = C_RCV;
            else if (received[count] == FLAG)
                state = FLAG_RCV;
            else
                state = START_ST;
            break;
        }
        case C_RCV:
        {
            if (received[count] == BCC1_UA) //In a more general state machine, where we don't know what we are receiving, we compare the received BCC1 to the A (received) ^ C (received)
                state = BCC_OK;
            if (received[count] == FLAG)
                state = FLAG_RCV;
            else
                state = START_ST;
            break;
        }

        case BCC_OK:
        {
            if (received[count] == FLAG)
            {
                printf(":%u:%d\n", received, count);
                printf("Read UA\n");

                STOP = TRUE;
                alarm(0);
            }
            else
                state = START_ST;
            break;
        }
        }
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
