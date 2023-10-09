// Application layer protocol implementation

#include "application_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // llopen()

    // Transmissor: llwrite() -> antes de escrever dá stuff and bytes (feito no PROTOCOL) (997 bytes por chunk)
        //Primeiro manda um control  packet (que é uma information frame) com o tamanho e nome do ficheiro
    // Recetor: llread() -> depois de ler os dados são destuffed (feito no PROTOCOL)
}
