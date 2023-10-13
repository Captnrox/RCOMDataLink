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
            printf("[llopen_transmitter] %d bytes written\n", write_bytes);

            for (int i = 0; i < 5; i++)
            {
                printf("[llopen_transmitter] 0x%02X\n", buf[i]);
            }
        }

        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);
        if (read_bytes)
        {
            printf("[llopen_transmitter] Read %u\n", input[0]);
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
                printf("[llopen_transmitter] Read UA\n");

                STOP = TRUE;
                alarm(0);
            }
            else
                state = START_ST;
            break;
        }
        }
    }
    return 1;
}

int llopen_receiver(LinkLayer connectionParameters)
{
    // Loop for input
    unsigned char buf[5] = {0};

    volatile int Error = FALSE;

    unsigned char input[1] = {0};
    unsigned char state = START_ST;

    while (STOP == FALSE)
    {
        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);

        if (read_bytes)
        {
            printf("[llopen_receiver] Read %u\n", input[0]);
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
            if (input[0] == A_SET)
                state = A_RCV;
            else if (input[0] != FLAG)
                state = START_ST;
            break;
        }
        case A_RCV:
        {
            if (input[0] == C_SET)
                state = C_RCV;
            else if (input[0] == FLAG)
                state = FLAG_RCV;
            else
                state = START_ST;
            break;
        }
        case C_RCV:
        {
            if (input[0] == BCC1_SET)
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
                printf("[llopen_receiver] Read Message\n");

                unsigned char UA[5] = {FLAG, 0x03, 0x07, 0x03 ^ 0x07, FLAG};
                int bytes = write(fd, UA, 5);
                printf("[llopen_receiver] %d UA bytes written\n", bytes);
                sleep(1);

                STOP = TRUE;
            }
            else
            {
                state = START_ST;
            }
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

StuffingAux stuff_byte(unsigned char byte)
{
    StuffingAux result;

    switch (byte)
    {
    case 0x7E:
    {
        result.stuffed = true;
        result.byte1 = 0x7D;
        result.byte1 = 0x5E;
        return result;
    }
    case 0x7D:
    {
        result.stuffed = true;
        result.byte1 = 0x7D;
        result.byte2 = 0x5D;
        return result;
    }
    default:
    {
        result.stuffed = false;
        result.byte1 = byte;
        return result;
    }
    }
}

StuffingAux destuff_byte(unsigned char byte1, unsigned char byte2)
{
    StuffingAux result;

    // Is not stuffed
    if (byte1 != 0x7D)
    {
        result.stuffed = false;
        result.byte1 = byte1;
        result.byte2 = byte2;
        return result;
    }

    // Is stuffed
    result.stuffed = true;

    switch (byte2)
    {
    case 0x5E:
    {
        result.byte1 = 0x7E;
        return result;
    }
    case 0x5D:
    {
        result.byte1 = 0x7D;
        return result;
    }
    }
}

void create_inf_frame(unsigned char *data, unsigned n, bool frame_num, unsigned char *result)
{
    // TODO: Make a way to alter A by a parameter
    unsigned char header[5] = {FLAG,
                               A_SENT_BY_TX,
                               frame_num,
                               A_SENT_BY_TX ^ frame_num};

    unsigned result_idx = 4; // Next idx on which to write data
    memcpy(result, header, 4);

    unsigned char bcc2 = data[0];

    // Stuffing Data
    StuffingAux stuffData;

    for (int i = 0; i < n; i++)
    {
        stuffData = stuff_byte(data[i]);

        if (stuffData.stuffed)
        {
            result[result_idx++] = stuffData.byte1;
            result[result_idx] = stuffData.byte2;
        }
        else
        {
            result[result_idx] = data[i];
        }
        result_idx++;

        if (i)
        { // If it is not the first chunk of data, XOR it with the rest
            bcc2 = bcc2 ^ data[i];
        }
    }

    StuffingAux stuffBCC2 = stuff_byte(bcc2);

    if (stuffBCC2.stuffed)
    {
        result[result_idx++] = stuffBCC2.byte1;
        result[result_idx++] = stuffBCC2.byte2;
    }
    else
    {
        result[result_idx++] = bcc2;
    }

    result[result_idx] = FLAG;
}

void destuff_frame(unsigned char *frame, unsigned n, unsigned char *destuffed_frame)
{
    StuffingAux destuffData;
    unsigned destuffed_idx = 0;

    for (int i = 0; i < n; i++)
    {
        // We will only analyse the last byte if the previous one wasn't stuffed, that means the last byte will alawyas be destuffed
        if (i == n - 1)
        {
            destuffed_frame[destuffed_idx] = frame[i];
        }
        else
        {
            destuffData = destuff_byte(frame[i], frame[i + 1]);

            if (destuffData.stuffed)
            {
                destuffed_frame[destuffed_idx++] = destuffData.byte1;
                i++; // Skip the next frame because it was part of the stuffing
            }
            else
            {
                destuffed_frame[destuffed_idx++] = frame[i];
            }
        }
    }
}
