#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "lcd.h"
#include "ds18b20.h"
#include "project.h"
#include "encoder.h"
#include "serial.h"


//Global variables

volatile unsigned char remoteLocalChanged=0; //for remote/local button presses

/*temperature variables*/
volatile unsigned char tempState; // state0=OK state1=WARM state2=HOT
volatile unsigned char oldTempState;
volatile unsigned char ledBlinkState=0; // state0=off state1=on
volatile unsigned char ledTimerFlag=0;
volatile unsigned char local,remote;
volatile int _PWM_width= 23;
volatile int tempInFahrenheit;
volatile int tempInCelsius;
volatile unsigned char decimal=0;
volatile unsigned char modeState;//local state0=local, state1=remote
volatile unsigned short freq=523;
volatile unsigned char arrowFlag=0;

//timer for servo motor
void timer2_init(void)
{
    TCCR2A |= (0b11 << WGM00);  // Fast PWM mode, modulus = 256
    TCCR2A |= (0b10 << COM2A0); // Turn D11 (PB3) on at 0x00 and off at OCR2A
    OCR2A = 23;   //midway between 0.75ms and 2.25ms, max=35 min=11
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024 for 16ms period
}

//timer for LEDs
void timer1_init(unsigned short m)
{
    TCCR1B |= (1<<WGM12); //set the CTC mode
    TIMSK1 |=(1<<OCIE1A); //enable Output Compare A Match Interrupt
    OCR1A=m; //load the MAX count w prescalar=256
    TCCR1B |= (1<<CS12); //set the prescalar to 256 and start counting
    TCCR1B &= ~(1<<CS12);
}

//timer for buzzer
void timer0_init(unsigned short m)
{
    TCCR0A |= (1<<WGM01); //set the CTC mode
    TIMSK0 |=(1<<OCIE0A); //enable Output Compare A Match Interrupt
    OCR0A=m; //load the MAX count w prescalar=256
}

int main(void) {

    
    //initialize timers
    timer2_init();
    timer1_init(31250); // 0.5 a second w/ 256 prescalar
    timer0_init(59); // 0.5 a second w/ 256 prescalar, count= (1/(2*523))*(16000000/256)

    //initializee rotary encoder
    encoder_init();
    
    //Serial interface initializations
    serial_init();
    
    PCICR |= (1<<PCIE1); //for local and remote pin change on Port C
    PCMSK1 |=  ((1<<PCINT9)|(1<<PCINT10)); //for local and remote pin change
    sei();

    // Initialize DDR and PORT registers and LCD
    DDRB |=(1<<PB3); //set PORTB PB3 as output for servo
    DDRB|= (1<<PB4); // set PORTB PB4 as output for RED LED
    DDRB|= (1<<PB5); // set PORTB PB5 as output for GREEN LED
    DDRC |= (1<<PC5); // set PORTC PC5 as output for Buzzer
    
    PORTC |= ((1<<PC1)|(1<<PC2)); //enable pull-up resistors for blue and yellow buttons

    lcd_init(); //initialize the screen
    lcd_writecommand(1);

    // Write a spash screen to the LCD
    char name[11]="Nemsiss Sh";
    lcd_stringout(name);
    char text[30];
    unsigned char  courseCode;
    courseCode=109;
    lcd_moveto(1,3);
    snprintf(text, 30,"EE%d Project", courseCode);
    lcd_stringout(text);
    _delay_ms(1000);
    lcd_writecommand(1);

    modeState=0; //default is set to state0 which is the local state

    char threshold_data[17];
    unsigned char eeprom_threshold=50;
    /*temprature readings*/
    unsigned char tempInBytes[2];
    if(ds_init()==0)
    {
        lcd_moveto(1,0);
        lcd_stringout("Sensor not responding");
    }
    ds_convert();
    int num1;  //to store the temperature received from remote
    while (1)
    {                 // Loop forever

        /*CODE FOR TEMPERATURE CONVERSION*/
        char buff[17];
        char transmitBuff[7];
        if(ds_temp(tempInBytes)) //if temperature has changed
        {
            tempInCelsius=0; //temp in celsius
            tempInCelsius|= tempInBytes[1];
            tempInCelsius=tempInCelsius<<8;
            tempInCelsius|=tempInBytes[0];
            tempInCelsius=tempInCelsius*10; //to get 0.1 accuracy;
            tempInCelsius=tempInCelsius>>4;
            tempInFahrenheit= (9*tempInCelsius/5);
            decimal= tempInFahrenheit%10; //to capture the decimal part
            tempInFahrenheit=tempInFahrenheit/10; //to display the whole number part only
            tempInFahrenheit=tempInFahrenheit+32;

            /*transmit data to remote if local temperature has changed*/
            if(tempInFahrenheit<0)
            {
                snprintf(transmitBuff,7, "@-%d#", tempInFahrenheit );
            }
            else if( tempInFahrenheit>=0)
            {
                snprintf(transmitBuff,7, "@+%d#", tempInFahrenheit );
            }
            transmitString(transmitBuff);
            
            if(tempInFahrenheit )
            {
                if(arrowFlag==0)
                {
                    snprintf(buff,17, ">Local: %d.%d  ", tempInFahrenheit,decimal );
                }
                else
                {
                    snprintf(buff,17, " Local: %d.%d  ", tempInFahrenheit,decimal );
                }
                lcd_moveto(1,0);
                lcd_stringout(buff);
            }
            if(remoteValidDataFlag )
            {
                remoteValidDataFlag=0;
                sscanf(remoteData,"%d", &num1);
                if(arrowFlag==1)
                {
                    snprintf(buff,17, ">Remote: %d ",num1);
                }
                else
                {
                    snprintf(buff,17, " Remote: %d ",num1);
                }
                lcd_moveto(0,0);
                lcd_stringout(buff);
            }
  
            ds_convert();
        }

        /*CODE FOR SETTING LOCAL/REMOTE MODE*/
        if(remoteLocalChanged)
        {
            remoteLocalChanged=0;
            if(remote==0)
            {
                modeState=1;
            }
            else if(local==0)
            {
                modeState=0;
            }
        }
        if(modeState==0 && tempInFahrenheit) //local state
        {
            arrowFlag=0;
            
            //set limits for temperature dial i.e. servo motor
            if(((-4*tempInFahrenheit)/10 +51)<11)
            {
                OCR2A= 11;
            }
            else if(((-4*tempInFahrenheit)/10 +51)>35)
            {
                OCR2A= 35;
            }
            else
            {
                OCR2A= (-4*tempInFahrenheit)/10 +51;        //update the OCR2A value w/ the new local temperature
            }
        }
        else if(modeState==1 ) //remote state
        {
            arrowFlag=1;
            
            //set limits for temperature dial i.e. servo motor
            if(((-4*num1)/10 +51)<11)
            {
                OCR2A= 11;
            }
            else if(((-4*num1)/10 +51)>35)
            {
                OCR2A= 35;
            }
            else
            {
                OCR2A= (-4*num1)/10 +51;        //update the OCR2A value w/ the new remote temperature
            }
        }
        /*CODE FOR TEMPRATURE THRESHOLD*/
        if(changed) //check if temperature threshold has changed
        {
            changed=0;
            eeprom_update_byte((void *)200, count); //count is guaranteed to be in the range of [50,90]
        }
        if(eeprom_read_byte((void *)200)>=50 && eeprom_read_byte((void *)200)<=90 ) //if eeprom data is valid
        {
            eeprom_threshold= eeprom_read_byte((void *)200);
        }
        count= eeprom_threshold;
        snprintf(threshold_data,17, "%d", eeprom_threshold);
        lcd_moveto(1,14);
        lcd_stringout(threshold_data);

        /*CODE FOR TEMPRATURE STATES AND LEDS*/
        lcd_moveto(0,12);
        oldTempState=tempState;
        if(eeprom_threshold< tempInFahrenheit   && tempInFahrenheit<eeprom_threshold+3)
        {
            tempState=1; //state1= WARM
        }
        else if( tempInFahrenheit>= eeprom_threshold+3 )
        {
            tempState=2; //state2= HOT
        }
        else
        {
            tempState=0; //state0= OK
        }
        if(tempState==2)
        {
            if(oldTempState==0)
            {
                PORTB &= ~(1<<PB5);
            }
            else if(oldTempState==1)
            {
                PORTB &= ~(1<<PB4);
                TCCR1B &= ~(1<<CS12); //stop counting
            }
            if(oldTempState==1 || oldTempState==0)
            {
                TCCR0B |= (1<<CS12); // start counting
            }
            PORTB |= (1<<PB4);
            lcd_stringout(" HOT");

        }
        else if(tempState==1)
        {
            if(oldTempState==0)
            {
                PORTB &= ~(1<<PB5);
            }
            else if(oldTempState==2)
            {
                PORTB &= ~(1<<PB4);
            }
            lcd_stringout("WARM");
            TCCR1B |= (1<<CS12); //start counting and blinking LED
            if(ledTimerFlag==1)
            {
                ledTimerFlag=0;
            }
        }
        else if(tempState==0)
        {
            if(oldTempState==1)
            {
                PORTB &= ~(1<<PB4);
                TCCR1B &= ~(1<<CS12); //stop counting
            }
            PORTB|= (1<<PB5); //turn on the green LED
            lcd_stringout("  OK");
        }
    }
}


 //for local/remote pin changes
ISR(PCINT1_vect)
{
    char remote_local_buttons= PINC;
    local = remote_local_buttons &(1<<PC1); //blue button
    remote = remote_local_buttons &(1<<PC2); //yellow button
    remoteLocalChanged=1;
}

//for LEDs
ISR(TIMER1_COMPA_vect)
{
    if(ledBlinkState==0) //if led is off
    {
        ledBlinkState=1;
        PORTB |= (1<<PB4); //turn it on
    }
    else if(ledBlinkState==1) //if led is on
    {
        ledBlinkState=0;
        PORTB &= ~(1<<PB4); //turn it off
    }
    ledTimerFlag=1;
}

//for buzzer
ISR(TIMER0_COMPA_vect)
{
    if(freq>=1)
    {
        freq--;
        PORTC ^=(1<<PC5);//flip the bit
    }
    else
    {
        PORTC &= ~(1<<PC5);
        TCCR0B &= ~(1<<CS12);
        freq=523;
    }
}
