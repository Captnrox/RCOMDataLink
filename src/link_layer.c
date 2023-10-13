// Link layer protocol implementation
#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

volatile int STOP = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;
int fd;
struct termios oldtio;
struct termios newtio;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("[llopen] Alarm #%d\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    switch (connectionParameters.role)
    {
    case LlTx:
        return llopen_transmitter(connectionParameters);
    case LlRx:
        break;
    }
}

int llopen_transmitter(LinkLayer connectionParameters)
{
    // Open serial port device for reading and writing, and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("[llopen] tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
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
        perror("[llopen] tcsetattr");
        exit(-1);
    }

    printf("[llopen] New termios structure set\n");

    // Create string to send
    unsigned char A_SEND = 0x03;
    unsigned char C_SEND = 0x03;
    unsigned char BCC1 = A_SEND ^ C_SEND;

    unsigned char buf[5] = {FLAG, A_SEND, C_SEND, BCC1, FLAG};
    unsigned char received[5] = {0};

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    unsigned char input[1] = {0};
    unsigned char state = START_ST;

    while (alarmCount < 3 && STOP == FALSE)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(3); // Set alarm to be triggered in 3s
            alarmEnabled = TRUE;

            int write_bytes = write(fd, buf, 5);
            printf("[llopen] %d bytes written\n", write_bytes);

            for (int i = 0; i < 5; i++)
            {
                printf("[llopen] 0x%02X\n", buf[i]);
            }
        }

        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);
        if (read_bytes)
        {
            printf("[llopen] Read %u\n", input[0]);
        }

        switch (state)
        {
        case START_ST:
        {
            if (input[0] == FLAG)
                state = FLAG_RCV;
            break;
        }
        case FLAG_RCV:
        {
            if (input[0] == A_UA) // This is specific state machine for receiving UA that exits as soon as a wrong char is read, in a more general state machine, we don't verify what we receive here
                state = A_RCV;
            else if (input[0] != FLAG)
                state = START_ST;
            break;
        }
        case A_RCV:
        {
            if (input[0] == C_UA)
                state = C_RCV;
            else if (input[0] == FLAG)
                state = FLAG_RCV;
            else
                state = START_ST;
            break;
        }
        case C_RCV:
        {
            if (input[0] == BCC1_UA) // In a more general state machine, where we don't know what we are receiving, we compare the received BCC1 to the A (received) ^ C (received)
                state = BCC_OK;
            else if (input[0] == FLAG)
                state = FLAG_RCV;
            else
                state = START_ST;
            break;
        }

        case BCC_OK:
        {
            if (input[0] == FLAG)
            {
                printf("[llopen] Read UA\n");

                STOP = TRUE;
                alarm(0);
            }
            else
                state = START_ST;
            break;
        }
        }
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
}
