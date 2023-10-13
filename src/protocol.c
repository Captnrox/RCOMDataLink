
#include <stdbool.h>

#define FLAG 0x07
#define A_SENT_BY_TX 0x03

typedef struct
{
    bool stuffed;
    unsigned char byte1;
    unsigned char byte2;

} StuffingAux;

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

    unsigned result_idx; // Next idx on which to write data

    //  Stuffing BCC1

    StuffingAux stuffBCC1 = stuff_byte(header[3]);

    if (stuffBCC1.stuffed)
    {
        header[3] = stuffBCC1.byte1;
        header[4] = stuffBCC1.byte2;
        memcpy(result, header, 5);
        result_idx = 5;
    }
    else
    {
        memcpy(result, header, 4);
        result_idx = 4;
    }

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
