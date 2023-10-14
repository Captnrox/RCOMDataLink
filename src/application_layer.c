// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

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
    printf("Finished llopen, calling llclose\n");
    llclose(false, connectionParameters);

    // Send Start Control Packet
    switch (connectionParameters.role)
    {
    case LlTx:
    {
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

        int L1 = ceil(log2f((float)fileSize) / 8.0); // log2 get the highest bit set, and division by 8 finds how many bytes it takes to represent it
        int L2 = 1 //placeholder

        char *start_control[3 + L1 + 2 + L2];

        start_control[0] = CTRL_START;
        start_control[1] = T_SIZE;
        start_control[2] = L1;

        for (int i = L1; i >= 0; i--)
        {
            start_control[2 + i] = fileSize >>  (L1 >> (8 * i)) & 0xff;
        }

        break;
    }
    case LlRx:

        break;
    }

    // Transmissor: llwrite() -> antes de escrever dá stuff and bytes (feito no PROTOCOL) (997 bytes por chunk)
    // Primeiro manda um control  packet (que é uma information frame) com o tamanho e nome do ficheiro
    // Recetor: llread() -> depois de ler os dados são destuffed (feito no PROTOCOL)

    // unsigned char result[2 * n + 8]; // If we hypothetically have to escape every byte of the array, it will take up 2n. The header and ending take 6 bytes, or 8 if we have to stuff the BCCs
}
