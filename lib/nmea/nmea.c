
//----------------------------------------------------------------------------
// AIS Decoder
//
// NMEA.c - Send out message using VDM NMEA Sentences
//
//----------------------------------------------------------------------------


//************************************************************************
//
// MODULES USED
//
//************************************************************************


#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "nmea.h"


//************************************************************************
//STa
// DEFINITIONS AND MACROS
//
//************************************************************************

//************************************************************************
//
// LOCAL VARIABLES
//
//************************************************************************

static char encoded_string[170];
static uint16_t encoded_length;
static uint8_t fill_bits;
static uint8_t seq_id = 0;

//************************************************************************
//
// LOCAL FUNCTION PROTOTYPES
//
//************************************************************************

void VDM_Send(uint8_t num_sentences, uint8_t sentence, uint8_t seq_id,
              char *src, uint8_t fill_bits);


//************************************************************************
//                                             
// EXPORTED FUNCTIONS
//
//************************************************************************


void NMEA_Send(unsigned char *pkt, uint8_t pkt_size)
{
    char *src;
    char *dest;
    uint16_t i;
    uint16_t j;
    uint8_t char8bit=0x00;
	uint8_t char6bit=0x00;
    uint16_t bitcount;
    uint8_t num_sentences, sentence;

    // Encode packet into 6-bit string
    src = (char *)pkt;
    dest = encoded_string;
    bitcount = 0;
    encoded_length = 0;
    for (i=0; i < pkt_size; i++)
    {
        char8bit = *src++;
        for (j=0; j < 8; j++)
        {
            if (char8bit & 0x80)
                char6bit |= 0x01;
            else
                char6bit &= 0xfe;
            if (++bitcount == 6)
            {
                // We have 6 bits
                bitcount = 0;
                char6bit &= 0x3f;

                // Convert to ASCII character
                if (char6bit < 0x28)
                    char6bit += 0x30;
                else
                    char6bit += 0x38;

                // write to destination string
                *dest++ = char6bit;
                ++encoded_length;
            }
            else
            {
                char6bit <<= 1;
            }
            char8bit <<= 1;
        }
    }

    // Add any fill bits required to the last character
    if (bitcount != 0)
    {
        // Fill bits required
        fill_bits = 6 - bitcount;
        char6bit <<= fill_bits;
        char6bit &= 0x3f;

        // Convert to ASCII character
        if (char6bit < 0x28)
            char6bit += 0x30;
        else
            char6bit += 0x38;

        // write to destination string
        *dest++ = char6bit;
        ++encoded_length;
    }
    else
        fill_bits = 0;

    // Terminate the string
    *dest = '\0';

    // Work out how many sentences required to send
    if (encoded_length <= 63)
    {
        // Only one sentence required
        num_sentences = 1;
    }
    else
    {
        num_sentences = encoded_length / 62;
        if (encoded_length % 62)
            num_sentences ++;
    }

    // Send the packet out using VDM sentences
    src = encoded_string;
    for (sentence = 1; sentence <= num_sentences; sentence++)
    {
        VDM_Send(num_sentences, sentence, seq_id, src, fill_bits);
        src += 62;
    }

    // If it was a multi-sentence message then advance the sequential ID
    if (num_sentences > 1){
        if (seq_id >= 9)
            seq_id = 0;
        else
            seq_id++;
	}
}// NMEA_Send



//************************************************************************
//
// LOCAL FUNCTIONS
//
//************************************************************************

void VDM_Send(uint8_t num_sentences, uint8_t sentence, uint8_t seq_id,
              char *src, uint8_t fill_bits)
{
    char vdmbuf[100];
    char str[100];
    char checksum;
    char *ptr;

    // Print start of sentence to buffer
    sprintf(vdmbuf, "AIVDM,%d,%d,", num_sentences, sentence);

    // If multi-sentence message then include the sequential ID
    if (num_sentences > 1)
    {
        sprintf(str, "%d", seq_id);
        strcat (vdmbuf, str);
    }

    // Assume AIS channels 1
    sprintf(str, ",A,");
    strcat (vdmbuf, str);

    // Append the 6-bit encoded message
    if (sentence == num_sentences)
        strcat (vdmbuf, src);
    else
    {
        strncpy (str, src, 62);
        str[62] = '\0';
        strcat (vdmbuf, str);
    }

    // Append number of fill bits
    if (sentence == num_sentences)
    {
        sprintf(str, ",%d", fill_bits);
        strcat(vdmbuf,str);
    }
    else
    {
        strcat(vdmbuf, ",0");
    }

    // Calculate checksum
    ptr = vdmbuf;
    checksum = *ptr++;
    while (*ptr)
    {
        checksum ^= *ptr++;
    }

    // Send sentence out using UART
    printf("!%s",vdmbuf);
    printf("*%02X", checksum);
    printf("\r\n");

}// VDM_Send



//************************************************************************
//
// EOF
//
//************************************************************************
