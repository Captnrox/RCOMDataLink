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

bool frameCountTx = 0;
bool frameCountRx = 0;

int numRetransmissions;
int timeout;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

int sendReceiverResponse(unsigned char C)
{
    switch (C)
    {
    case C_RR0:
        printf("[sendReceiverResponse] Sent response RR0\n");
        break;
    case C_RR1:
        printf("[sendReceiverResponse] Sent response RR1\n");
        break;
    case C_REJ0:
        printf("[sendReceiverResponse] Sent response REJ0\n");
        break;
    case C_REJ1:
        printf("[sendReceiverResponse] Sent response REJ1\n");
        break;
    }

    unsigned char frame[5] = {FLAG, ANS_RX, C, ANS_RX ^ C, FLAG};
    return write(fd, frame, 5);
}

int llopenTransmitter(LinkLayer connectionParameters)
{
    printf("Reached llopenTransmitter\n");
    // Create string to send
    unsigned char A_SEND = 0x03;
    unsigned char C_SEND = 0x03;
    unsigned char BCC1 = A_SEND ^ C_SEND;

    unsigned char buf[5] = {FLAG, A_SEND, C_SEND, BCC1, FLAG};

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    unsigned char input[1] = {0};
    unsigned char state = START_ST;
    alarmEnabled = FALSE;
    alarmCount = 0;

    while (alarmCount < numRetransmissions)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(timeout); // Set alarm to be triggered in timeout seconds
            alarmEnabled = TRUE;

            int write_bytes = write(fd, buf, 5);
            sleep(1);
            printf("[llopenTransmitter] %d bytes written\n", write_bytes);

            for (int i = 0; i < 5; i++)
            {
                printf("[llopenTransmitter] 0x%02X\n", buf[i]);
            }
        }

        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);
        if (read_bytes)
        {
            printf("[llopenTransmitter] Read %u\n", input[0]);
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
            if (input[0] == ANS_RX) // This is specific state machine for receiving UA that exits as soon as a wrong char is read, in a more general state machine, we don't verify what we receive here
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
            if (input[0] == (ANS_RX ^ C_UA)) // In a more general state machine, where we don't know what we are receiving, we compare the received BCC1 to the A (received) ^ C (received)
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
                printf("[llopenTransmitter] Read UA\n");

                alarm(0);
                return 1;
            }
            else
                state = START_ST;
            break;
        }
        }
    }
    return -1;
}

int llopenReceiver(LinkLayer connectionParameters)
{
    printf("Reached llopenReceiver\n");

    // Loop for input
    unsigned char input[1] = {0};
    unsigned char state = START_ST;
    STOP = FALSE;

    while (STOP == FALSE)
    {
        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);

        if (read_bytes)
        {
            printf("[llopenReceiver] Read %x\n", input[0]);
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
            if (input[0] == CMD_TX)
                state = A_RCV;
            else if (input[0] != FLAG)
                state = START_ST;
            break;
        }
        case A_RCV:
        {
            if (input[0] == CMD_TX)
                state = C_RCV;
            else if (input[0] == FLAG)
                state = FLAG_RCV;
            else
                state = START_ST;
            break;
        }
        case C_RCV:
        {
            if (input[0] == (CMD_TX ^ C_SET))
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
                printf("[llopenReceiver] Read SET\n");

                unsigned char UA[5] = {FLAG, 0x03, 0x07, 0x03 ^ 0x07, FLAG};
                int bytes = write(fd, UA, 5);
                sleep(1);
                printf("[llopenReceiver] %d UA bytes written\n", bytes);

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
    return 1;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    printf("Reached llopen\n");
    printf("[llopen] Serial port: %s\n", connectionParameters.serialPort);

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

    numRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    switch (connectionParameters.role)
    {
    case LlTx:
        return llopenTransmitter(connectionParameters);
    case LlRx:
        return llopenReceiver(connectionParameters);
    }
    return -1;
}

int llwriteSendFrame(unsigned char *frame, int frameSize)
{
    unsigned char expectedResponse = frameCountTx ? C_RR0 : C_RR1;
    unsigned char rejection = frameCountTx ? C_REJ1 : C_REJ0;

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    unsigned char input[1] = {0};
    unsigned char state = START_ST;
    alarmEnabled = FALSE;
    alarmCount = 0;

    while (alarmCount < numRetransmissions)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(timeout); // Set alarm to be triggered in timeout seconds
            alarmEnabled = TRUE;

            int write_bytes = write(fd, frame, frameSize);
            sleep(1);
            printf("[llWriteSendFrame] %d bytes written\n", write_bytes);

            /*
            for (int i = 0; i < bufSize + 6; i++)
            {
                printf("[llwrite] 0x%02X\n", infFrame[i]);
            }*/
        }

        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);
        if (read_bytes)
        {
            //printf("[llWriteSendFrame] Read 0x%x\n", input[0]);

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
                if (input[0] == ANS_RX)
                    state = A_RCV;
                else if (input[0] != FLAG)
                    state = START_ST;
                break;
            }
            case A_RCV:
            {
                if (input[0] == expectedResponse)
                    state = C_RCV;
                else if (input[0] == FLAG)
                    state = FLAG_RCV;
                else if (input[0] == rejection)
                {
                    printf("[llWriteSendFrame] Read RJ\n");
                    alarm(0);
                    return 0;
                }
                else
                    state = START_ST;
                break;
            }
            case C_RCV:
            {
                if (input[0] == (ANS_RX ^ expectedResponse))
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
                    printf("[llWriteSendFrame] Read RR%d\n", !frameCountTx);
                    alarm(0);
                    frameCountTx = !frameCountTx;
                    return 1;
                }
                else
                    state = START_ST;
                break;
            }
            }
        }
    }
    printf("[llWriteSendFrame] Obtained no appropriate response after numRetransmission alarms\n");
    alarm(0);
    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char infFrame[6 + 2 * bufSize];
    createInfFrame(buf, bufSize, frameCountTx, CMD_TX, infFrame);

    int result = llwriteSendFrame(infFrame, 2 * bufSize + 6);

    while (result == 0)
    {
        result = llwriteSendFrame(infFrame, 2 * bufSize + 6); // Resend frame while REJs are being read
    }
    return result == 1 ? bufSize + 6 : -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char input[1] = {0};
    unsigned char state = START_ST;
    STOP = FALSE;

    bool receivedFrame;

    unsigned int packetIdx = 0;
    unsigned char bcc2;

    while (STOP == FALSE)
    {
        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);
        if (read_bytes)
        {
            // printf("[llread] Read %u\n", input[0]);
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
                if (input[0] == CMD_TX)
                    state = A_RCV;
                else if (input[0] != FLAG)
                    state = START_ST;
                break;
            }
            case A_RCV:
            {
                if (input[0] == C_FRAME0)
                {
                    receivedFrame = 0;
                    state = C_RCV;
                }
                else if (input[0] == C_FRAME1)
                {
                    receivedFrame = 1;
                    state = C_RCV;
                }
                else if (input[0] == FLAG)
                    state = FLAG_RCV;
                else
                    state = START_ST;
                break;
            }
            case C_RCV:
            {
                unsigned int receivedC = receivedFrame ? 0x40 : 0x00;
                if (input[0] == (CMD_TX ^ receivedC))
                    state = READING_DATA;
                else if (input[0] == FLAG)
                    state = FLAG_RCV;
                else
                    state = START_ST;
                break;
            }
            case READING_DATA:
            {
                if (input[0] == ESC)
                    state = DESTUFFING;
                else if (input[0] == FLAG)
                {
                    printf("[llread] Read whole input\n");
                    unsigned char receivedBcc2 = packet[packetIdx - 1];
                    bcc2 = packet[0];

                    packet[packetIdx - 1] = '\0'; // Remove read bcc2 because the application layer only wants the data packets

                    for (int i = 1; i < packetIdx - 1; i++) // Start in index 1 because index 0 is THE value that initializes bcc2
                    {
                        bcc2 = bcc2 ^ packet[i];
                    }

                    if (bcc2 == receivedBcc2) // Frame data has NO errors
                    {
                        if (receivedFrame == frameCountRx)
                        {
                            frameCountRx = !frameCountRx; // Updated value with what frame receiver expects next
                            sendReceiverResponse(frameCountRx ? C_RR1 : C_RR0);
                            printf("[llread] Read expected frame\n");
                            return packetIdx;
                        }
                        else // Duplicate Frame
                        {

                            sendReceiverResponse(frameCountRx ? C_RR1 : C_RR0); // Receiver is still expecting the same frame, because it just got a duplicate
                            printf("[llread] Read duplicate frame\n");
                            return -1;
                        }
                    }
                    else // Frame data HAS errors
                    {
                        if (receivedFrame == frameCountRx)
                        {
                            sendReceiverResponse(frameCountRx ? C_REJ1 : C_REJ0); // Receiver is still expecting the same frame, because it just got a frame with errors
                            printf("[llread] Read expected frame with errors\n");
                            return -1;
                        }
                        else // Duplicate Frame
                        {
                            sendReceiverResponse(frameCountRx ? C_RR1 : C_RR0); // Receiver is still expecting the same frame, because it just got a duplicate
                            printf("[llread] Read duplicate frame with errors\n");
                            return -1;
                        }
                    }
                }
                else
                    packet[packetIdx++] = input[0];
                break;
            }
            case DESTUFFING:
            {
                if (input[0] == 0x5E)
                {
                    packet[packetIdx++] = 0x7E;
                    state = READING_DATA;
                }
                else if (input[0] == 0x5D)
                {
                    packet[packetIdx++] = 0x7D;
                    state = READING_DATA;
                }
                else
                {
                    printf("[llread] An ESC that wasn't stuffed\n");
                    state = READING_DATA;
                }
                break;
            }
            }
        }
    }
    return -1;
}

int llcloseTransmitter(LinkLayer connectionParameters)
{
    // Create the supervision frame
    unsigned char supervA = 0x03;
    unsigned char supervBCC1 = supervA ^ C_DISC;

    unsigned char buf[5] = {FLAG, supervA, C_DISC, supervBCC1, FLAG};

    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    unsigned char input[1] = {0};
    unsigned char state = START_ST;
    alarmEnabled = FALSE;
    alarmCount = 0;

    while (alarmCount < numRetransmissions)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(timeout); // Set alarm to be triggered in timeout seconds
            alarmEnabled = TRUE;

            int write_bytes = write(fd, buf, 5);
            sleep(1);
            printf("[llCloseTransmitter] %d bytes written\n", write_bytes);
        }

        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);
        if (read_bytes)
        {
            // printf("[llCloseTransmitter] Read 0x%x\n", input[0]);

            switch (state)
            {
            case START_ST:
                if (input[0] == FLAG)
                    state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (input[0] == 0x01)
                    state = A_RCV;
                else if (input[0] != FLAG)
                    state = START_ST;
                break;
            case A_RCV:
                if (input[0] == C_DISC)
                    state = C_RCV;
                else if (input[0] == FLAG)
                    state = FLAG_RCV;
                else
                    state = START_ST;
                break;
            case C_RCV:
                if (input[0] == (C_DISC ^ 0x01))
                    state = BCC_OK;
                else if (input[0] == FLAG)
                    state = FLAG_RCV;
                else
                    state = START_ST;
                break;
            case BCC_OK:
                if (input[0] == FLAG)
                {
                    printf("[llCloseTransmitter] Read C_DISC\n");

                    unsigned char UA[5] = {FLAG, 0x01, C_UA, 0x01 ^ C_UA, FLAG};
                    int bytes = write(fd, UA, 5);
                    sleep(1);
                    printf("[llCloseTransmitter] %d UA bytes written\n", bytes);

                    alarm(0);
                    return 1;
                }
                else
                    state = START_ST;
                break;
            }
        }
    }
    return -1;
}

int llcloseReceiver(LinkLayer connectionParameters)
{
    unsigned char state = START_ST;
    unsigned char input[1] = {0};

    STOP = FALSE;

    while (STOP == FALSE)
    {
        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);
        if (read_bytes)
        {
            // printf("[llCloseReceiver] Read 0x%x\n", input[0]);

            switch (state)
            {
            case START_ST:
                if (input[0] == FLAG)
                    state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (input[0] == 0x03)
                    state = A_RCV;
                else if (input[0] != FLAG)
                    state = START_ST;
                break;
            case A_RCV:
                if (input[0] == C_DISC)
                    state = C_RCV;
                else if (input[0] == FLAG)
                    state = FLAG_RCV;
                else
                    state = START_ST;
                break;
            case C_RCV:
                if (input[0] == (C_DISC ^ 0x03))
                    state = BCC_OK;
                else if (input[0] == FLAG)
                    state = FLAG_RCV;
                else
                    state = START_ST;
                break;
            case BCC_OK:
                if (input[0] == FLAG)
                {
                    printf("[llCloseReceiver] Read C_DISC\n");
                    STOP = true;
                }
                else
                    state = START_ST;
            }
        }
    }

    // Create the C_C_DISC frame
    unsigned char supervA = 0x01;
    unsigned char supervBCC1 = supervA ^ C_DISC;

    unsigned char buf[5] = {FLAG, supervA, C_DISC, supervBCC1, FLAG};

    (void)signal(SIGALRM, alarmHandler);

    input[0] = 0;
    state = START_ST;
    alarmEnabled = FALSE;
    alarmCount = 0;

    // Write C_C_DISC and wait for UA from transmitter
    while (alarmCount < numRetransmissions)
    {
        if (alarmEnabled == FALSE)
        {
            alarm(timeout); // Set alarm to be triggered in timeout seconds
            alarmEnabled = TRUE;

            int write_bytes = write(fd, buf, 5);
            sleep(1);
            printf("[llCloseReceiver] %d bytes written\n", write_bytes);
        }

        // Returns after a char has been input
        int read_bytes = read(fd, input, 1);
        if (read_bytes)
        {
            // printf("[llCloseReceiver] Read 0x%x\n", input[0]);

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
                if (input[0] == 0x01) // This is specific state machine for receiving UA that exits as soon as a wrong char is read, in a more general state machine, we don't verify what we receive here
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
                if (input[0] == (0x01 ^ C_UA)) // In a more general state machine, where we don't know what we are receiving, we compare the received BCC1 to the A (received) ^ C (received)
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
                    printf("[llcloseReceiver] Read UA\n");
                    alarm(0);
                    return 1;
                }
                else
                    state = START_ST;
                break;
            }
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////

int llclose(int showStatistics, LinkLayer connectionParameters)
{
    switch (connectionParameters.role)
    {
    case LlTx:
        if (llcloseTransmitter(connectionParameters) == -1)
            return -1;
        break;
    case LlRx:
        if (llcloseReceiver(connectionParameters) == -1)
            return -1;
        break;
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);
    return 1;
}

StuffingAux stuffByte(unsigned char byte)
{
    StuffingAux result;

    switch (byte)
    {
    case 0x7E:
    {
        result.stuffed = true;
        result.byte1 = 0x7D;
        result.byte2 = 0x5E;
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
        result.stuffed = FALSE;
        result.byte1 = byte;
        return result;
    }
    }
}

StuffingAux destuffByte(unsigned char byte1, unsigned char byte2)
{
    StuffingAux result;

    // Is not stuffed
    if (byte1 != 0x7D)
    {
        result.stuffed = FALSE;
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
    return result;
}

void createInfFrame(const unsigned char *data, unsigned n, bool frameNum, AddressFieldType addressType, unsigned char *result)
{
    unsigned char header[4] = {FLAG,
                               addressType,
                               frameNum ? 0x40 : 0x00,
                               addressType ^ (frameNum ? 0x40 : 0x00)};

    unsigned result_idx = 4; // Next idx on which to write data
    memcpy(result, header, 4);

    unsigned char bcc2 = data[0];

    // Stuffing Data
    StuffingAux stuffData;

    for (int i = 0; i < n; i++)
    {
        stuffData = stuffByte(data[i]);

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

    StuffingAux stuffBCC2 = stuffByte(bcc2);

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

void destuffFrame(unsigned char *frame, unsigned n, unsigned char *destuffedFrame)
{
    StuffingAux destuffData;
    unsigned destuffedIdx = 0;

    for (int i = 0; i < n; i++)
    {
        // We will only analyse the last byte if the previous one wasn't stuffed, that means the last byte will alawyas be destuffed
        if (i == n - 1)
        {
            destuffedFrame[destuffedIdx] = frame[i];
        }
        else
        {
            destuffData = destuffByte(frame[i], frame[i + 1]);

            if (destuffData.stuffed)
            {
                destuffedFrame[destuffedIdx++] = destuffData.byte1;
                i++; // Skip the next frame because it was part of the stuffing
            }
            else
            {
                destuffedFrame[destuffedIdx++] = frame[i];
            }
        }
    }
}
