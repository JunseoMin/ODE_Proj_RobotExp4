/*
 * GccApplication1.c
 *
 * Created: 2023-11-21 오후 10:15:10
 * Author : minjs
 */ 

#include <avr/io.h>
#include "dataType.h"

volatile unsigned char checkSize;
volatile unsigned char g_buf[256], g_BufWriteCnt, g_BufReadCnt;

volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID = 1;


int main(void)
{
    /* Replace with your application code */
    while (1) 
    {
    }
}

