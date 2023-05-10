#include "serial.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define FOSC 16000000 //clock frequency
#define BAUD 9600   //Baud rate used
#define MYUBRR (FOSC/16/BAUD-1) //Value for UBRR0
volatile unsigned char remoteValidDataFlag = 0;
volatile unsigned char dataReceiveStarted=0;
char remoteData[7];
volatile int indexCharsReceived=0;

void serial_init()
{
    DDRC |= (1<<PC4);
    PORTC &= ~(1<<PC4);
    UBRR0 = MYUBRR;
    UCSR0B |= (1<<TXEN0 | 1<<RXEN0); //enable RX and TX
    UCSR0C= (3<<UCSZ00); //Async., no parity, 1stop bit, 8 data bits
    UCSR0B |=(1<<RXCIE0);
}
void tx_char(char next_character)
{
    while ((UCSR0A & (1 << UDRE0)) == 0) { }
    UDR0 = next_character;
}
void transmitString(char* str)
{
    for(int i=0; i<=6; i++) // 16 characters, the 17th character is NULL
    {
        tx_char(str[i]);
    }
}
ISR(USART_RX_vect)
{
    char ch = UDR0;
    
    if(ch=='@' && dataReceiveStarted==0) //if this is the first character
    {
        dataReceiveStarted=1;
        indexCharsReceived++;
    }
    else if((ch=='+' || ch=='-') && dataReceiveStarted==1 && indexCharsReceived!=1) //if +/- is recieved in a wrong spot, reset
    {
        dataReceiveStarted=0;
        indexCharsReceived=0;
    }
    else if((ch=='+' || ch=='-') && dataReceiveStarted==1 && indexCharsReceived==1)//if +/- is received in the right spot, increment the index for next char
    {
        indexCharsReceived++;
        remoteData[indexCharsReceived-2]=ch;
    }
    else if(dataReceiveStarted==1)
    {
        if(ch>='0' && ch<='9' && indexCharsReceived<=4)
        {
            indexCharsReceived++;
            remoteData[indexCharsReceived-2]=ch;
        }
        else if(ch =='#' && (indexCharsReceived==4 || indexCharsReceived==5))
        {
            indexCharsReceived++;
            remoteValidDataFlag=1;
            remoteData[indexCharsReceived-2] ='\0';
        }
        else //if invalid data is received
        {
            dataReceiveStarted=0;
            indexCharsReceived=0;
        }
    }
}
