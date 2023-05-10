#include "encoder.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

volatile unsigned char changed=0 ;  // Flag for state change
volatile int count ;        // Count to display
volatile unsigned char new_state, old_state; //states of the rotary encoder
volatile unsigned char a, b;

void encoder_init()
{
    
    PORTD |= ((1<<PD2)|(1<<PD3)); //enable pull-up resistors for rotary encoder
    /*enable local interrupts*/
    PCICR |= (1<<PCIE2); //for local and remote pin change on Port D
    PCMSK2 |= ((1<<PCINT18)|(1<<PCINT19)); //for rotary encoder
    char buttons= PIND; //save what PIND contains now
    a= buttons &(1<<PD2);
    b= buttons &(1<<PD3);
    count=0;
    if (!b && !a)
    old_state = 0;
    else if (!b && a)
    old_state = 1;
    else if (b && !a)
    old_state = 2;
    else
    old_state = 3;
    new_state = old_state;
}

//for rotary encoder
ISR(PCINT2_vect)
{
    // Read the input bits and determine A and B.
        char buttons= PIND;
        a= buttons &(1<<PD2);
        b= buttons &(1<<PD3);

    // For each state, examine the two input bits to see if state
    // has changed, and if so set "new_state" to the new state,
    // and adjust the count value.
    if (old_state == 0) {

        // Handle A and B inputs for state 0
        if( a!=0 && b==0 )
        {
            new_state=1;
            count++;
        }
        else if( a==0 && b!=0)
        {
            new_state=2;
            count--;
        }

    }
    else if (old_state == 1) {

        // Handle A and B inputs for state 1
        if( a!=0 && b!=0 )
        {
            new_state=3;
            count++;
        }
        else if( a==0 && b==0)
        {
            new_state=0;
            count--;
        }

    }
    else if (old_state == 2) {

        // Handle A and B inputs for state 2
        if( a!=0 && b!=0 )
        {
            new_state=3;
            count--;
        }
        else if( a==0 && b==0)
        {
            new_state=0;
            count++;
        }
    }
    else {   // old_state = 3

        // Handle A and B inputs for state 3
        if( a!=0 && b==0 )
        {
            new_state=1;
            count--;
        }
        else if( a==0 && b!=0)
        {
            new_state=2;
            count++;
        }
    }

    /* add code to limit count value to stay between 50 and 90 */
    if(count>90)
    {
        count=90;
    }
    else if( count<50)
    {
        count= 50;
    }
    // If state changed, update the value of old_state,
    // and set a flag that the state has changed.
    if (new_state != old_state) {
        changed = 1;
        old_state = new_state;
    }

}
