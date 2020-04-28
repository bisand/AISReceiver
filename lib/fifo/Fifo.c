//************************************************************************
//
// MODULES USED
//
//************************************************************************
#include <stdlib.h>
#include <stdio.h>
#include "fifo.h"
//************************************************************************
//
// DEFINITIONS AND MACROS
//
//************************************************************************
// Define size of circular buffer - must be a power of 2
#define     BUFFER_SIZE     256
// Define bitmask to wrap buffer address		(3FF for 1024)
#define     BUFFER_MASK     0xFF

// Define size of buffer index - must be a power of 2
#define     INDEX_SIZE      4
// Define bitmask to wrap index address
#define     INDEX_MASK      0x03
//************************************************************************
//
// LOCAL VARIABLES
//
//************************************************************************
static unsigned char buffer[BUFFER_SIZE];
static uint8_t pkt_index[INDEX_SIZE];
static uint8_t in_index;
static uint8_t out_index;
static uint8_t pkt_offset;
//************************************************************************
//
// EXPORTED FUNCTIONS
//
//************************************************************************

// This function checks if there is a packet waiting in the FIFO.
// If there is up to bufsize bytes of the next packet are transferred
// to the nominated buffer.
// Returns the number of bytes copied, zero if no packet
//
unsigned short FIFO_Get_Packet(unsigned char *buf, uint8_t bufsize)
{
    uint8_t pkt_start, pkt_end, pkt_size, bytes_copied;

    // Check if there's a packet available
    if (in_index == out_index)
        return 0;
    
    // Find the start and end of the packet
    pkt_start = pkt_index[out_index];
    pkt_end = pkt_index[(out_index + 1) & INDEX_MASK];

    // Calculate the size of the packet
    if (pkt_end > pkt_start)
        pkt_size = pkt_end - pkt_start;
    else
        pkt_size = (BUFFER_SIZE - pkt_start) + pkt_end;

    // Copy the packet out into the target buffer
    bytes_copied = 0;
    while ((bytes_copied < pkt_size) && (bytes_copied < bufsize))
    {
        *buf++ = buffer[pkt_start];
        pkt_start = (pkt_start+1) & BUFFER_MASK;
        ++bytes_copied;
    }
        
    // Advance the out pointer
    out_index = (out_index+1) & INDEX_MASK;

    // Return the number of bytes copied
    return bytes_copied;

}// FIFO_Get_Packet


//
// Put a received byte into the FIFO
//
void FIFO_Put(unsigned char c)
{
    uint8_t pos;
    
    pos = (pkt_index[in_index] + pkt_offset) & BUFFER_MASK;
    buffer[pos] = c;
    pkt_offset++;   
}

//
// Release a complete packet into the FIFO
//
void FIFO_Write_Packet(void)
{
    uint8_t pos;
 
    if (pkt_offset == 0)
        return;

    pos = (pkt_index[in_index] + pkt_offset) & BUFFER_MASK;

    in_index = (in_index+1) & INDEX_MASK;
    pkt_index[in_index] = pos;

}


//
// Reset the buffer pointer
//
void FIFO_Reset_Packet(void)
{
    pkt_offset = 0;
}


//
// Initialise the FIFO system
//
void FIFO_Init(void)
{
    in_index = 0;
    out_index = 0;
    pkt_index[0] = 0;

}// FIFO_Init


//************************************************************************
//
// EOF
//
//************************************************************************

