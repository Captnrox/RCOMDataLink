// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

unsigned int bytesToRepresent(long int n)
{
    return ceil(log2f((float)n) / 8.0); // log2 get the highest bit set, and division by 8 finds how many bytes it takes to represent it
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    printf("Reached applicationLayer\n");

    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.role = strcmp(role, "tx") ? LlRx : LlTx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    llopen(connectionParameters);

    // Send Start Control Packet
    switch (connectionParameters.role)
    {
    case LlTx:
    {
        // Create start control packet
        FILE *file = fopen(filename, "rb");
        if (file == NULL)
        {
            perror("[application_layer] File not found\n");
            exit(-1);
        }

        fseek(file, 0L, SEEK_END);
        long int fileSize = ftell(file); // File size in bytes
        fclose(file);

        printf("File size: %ld\n", fileSize);

        unsigned char L1 = bytesToRepresent(fileSize);
        unsigned char L2 = strlen(filename);

        printf("L1: %d L2: %d\n", L1, L2);

        unsigned char startControl[3 + L1 + 2 + L2];

        startControl[0] = (unsigned char)CTRL_START;
        startControl[1] = (unsigned char)T_SIZE;
        startControl[2] = L1;
        unsigned int idx = 3;

        for (int i = L1; i >= 0; i--, idx++)
        {
            startControl[idx] = (fileSize >> (8 * i)) & 0xff;
        }

        startControl[idx++] = T_NAME;
        startControl[idx++] = L2;

        for (int i = 0; i < L2; i++, idx++)
        {
            startControl[idx] = filename[i];
        }

        llwrite(startControl, 3 + L1 + 2 + L2);
         break;
    }
    case LlRx:
        unsigned char packet[MAX_PAYLOAD_SIZE];
        llread(packet);
        break;
    }

    llclose(false, connectionParameters);

    // Transmissor: llwrite() -> antes de escrever dá stuff and bytes (feito no PROTOCOL) (997 bytes por chunk)
    // Primeiro manda um control  packet (que é uma information frame) com o tamanho e nome do ficheiro
    // Recetor: llread() -> depois de ler os dados são destuffed (feito no PROTOCOL)

    // unsigned char result[2 * n + 8]; // If we hypothetically have to escape every byte of the array, it will take up 2n. The header and ending take 6 bytes, or 8 if we have to stuff the BCCs
}
