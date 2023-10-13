// Application layer protocol implementation

#include "application_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // llopen()

    // Transmissor: llwrite() -> antes de escrever dá stuff and bytes (feito no PROTOCOL) (997 bytes por chunk)
    // Primeiro manda um control  packet (que é uma information frame) com o tamanho e nome do ficheiro
    // Recetor: llread() -> depois de ler os dados são destuffed (feito no PROTOCOL)

    unsigned char result[2 * n + 8]; // If we hypothetically have to escape every byte of the array, it will take up 2n. The header and ending take 6 bytes, or 8 if we have to stuff the BCCs
}
