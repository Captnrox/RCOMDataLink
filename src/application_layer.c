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

        while (llwrite(startControl, 3 + L1 + 2 + L2) == -1)
            ;
        printf("Wrote control packet\n");

        while (fileSize > 0)
        {
            printf("fileSize: %d\n", fileSize);
            unsigned char data[MAX_PAYLOAD_SIZE] = {0};
            unsigned int numBytes = fileSize < MAX_PAYLOAD_SIZE - 3 ? fileSize : MAX_PAYLOAD_SIZE - 3;
            data[0] = 1;

            data[1] = (numBytes >> 8) & 0xff;
            data[2] = numBytes & 0xff;
            fread(data + 3, sizeof(unsigned char), numBytes, file);

           /*  for (int i = 0; i < MAX_PAYLOAD_SIZE; i++)
            {
                printf("Data being sent: %x\n", data[i]);
            } */

            while (llwrite(data, 3 + numBytes) == -1)
                ;
            printf("Wrote frame\n");
            fileSize -= numBytes;
        }

        startControl[0] = 3; // TODO: Change this
        while (llwrite(startControl, 3 + L1 + 2 + L2) == -1)
            ;
        fclose(file);

        break;
    }
    case LlRx:
        unsigned char packet[MAX_PAYLOAD_SIZE];
        while (llread(packet) == -1) // Read start control packet
            ;
        printf("Read control packet\n");
        // QUESTION: Depois de ler os dados do control packet, o que fazer com eles? Não nos parece ser preciso para nada
        FILE *outputFile = fopen(filename, "wb+");

        while (packet[0] != 3) // While we haven't received a control end packet
        {
            while (llread(packet) == -1)
                ;
            printf("Read packet\n");

            unsigned char *dataStart = packet + 3;
            fwrite(dataStart, sizeof(unsigned char), MAX_PAYLOAD_SIZE - 3, outputFile); // TODO: The file might not be a multiple of 997 so the last packet will have a lot of empty space
        }
        break;
    }

    llclose(false, connectionParameters);

    // Transmissor: llwrite() -> antes de escrever dá stuff and bytes (feito no PROTOCOL) (997 bytes por chunk)
    // Primeiro manda um control  packet (que é uma information frame) com o tamanho e nome do ficheiro
    // Recetor: llread() -> depois de ler os dados são destuffed (feito no PROTOCOL)

    // unsigned char result[2 * n + 8]; // If we hypothetically have to escape every byte of the array, it will take up 2n. The header and ending take 6 bytes, or 8 if we have to stuff the BCCs
}
