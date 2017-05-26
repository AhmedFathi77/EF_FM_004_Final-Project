
#include <avr/io.h>
#include <util/delay.h>
#include "DC-Motors.c"
#include "Proto.h"
#include <avr/interrupt.h>
#include <stdlib.h>

#include <avr/interrupt.h>
#include <util/delay.h>



int main(void)
{

    DDRA = 0xFF;
    DDRD &= ~(1<<0);
    DDRD &= ~(1<<1);
    DDRC = 0xFF;
    DDRB  = 0x0F;

int Distance_in_cm = 0;
    _delay_ms(50);

    EF_send_command(0x28);
    EF_send_command(0x0c);
    EF_send_command(0x06);
    EF_send_command(0x80);


    intiate_Sensor();


    while(1)
    {

        Distance_in_cm = read_sonar() ;




        if( (PIND & (1 << Left_Sensor)) && (PIND & (1 << Right_Sensor)) )
        {
        	EF_Move_Forward();


        }else if( !(PIND & (1<<Left_Sensor)) && !(PIND & (1 << Right_Sensor)) )
        {
        	EF_Move_Backward();

        }
        else if( (PIND & (1 << Left_Sensor)) && !(PIND & (1 << Right_Sensor)) )
        { //move left
            EF_Move_Left();

         }
        else if( !(PIND & (1 << Left_Sensor)) && (PIND & (1 << Right_Sensor)))
        { //move right
              EF_Move_Right();

        }


    }
}



void intiate_Sensor(){
	DDRD &= ~(1<<Left_Sensor);
	DDRD &= ~(1<<Right_Sensor);
}



void EF_Send_char(char Value){
	PORTB = (Value&0xf0);
	PORTC |=(LCD_ENABLE_PIN);
	_delay_ms(2);
	PORTC &= ~(LCD_ENABLE_PIN);
	// second 4 bit
	PORTB = ((Value<<4)&0xf0);
	PORTC |=(LCD_ENABLE_PIN);
	_delay_ms(2);
	PORTC &= ~(LCD_ENABLE_PIN);
	_delay_us(100);
}
void EF_send_command(char comand){
	PORTC &=~(1<<LCD_RW_PIN);
	PORTC &=~(1<<LCD_RS_PIN);
	EF_Send_char(comand);
}
void EF_send_Data(char data){
	PORTC &=~(1<<LCD_RW_PIN);
	PORTC |= (1<<LCD_RS_PIN);
	EF_Send_char(data);
}
void EF_CLEAR_LCD(){
	EF_send_command(0x01);
	_delay_ms(20);
}
void EF_void_LCD_goto(char y, char x)
{
	char firstAddress[] = {0x80,0xC0,0x94,0xD4};

	EF_send_command(firstAddress[y-1] + x-1);
	_delay_ms(10);
}
void EF_void_LCD_print(char *string)
{
	int i = 0;

	while(string[i]!=0)
	{
		 (string[i]);
		i++;
	}
}
void EF_send_a_string(char *string_of_characters)
{
    while(*string_of_characters > 0)
    {
        EF_Send_char(*string_of_characters++);
    }
}
void init_sonar(){
	DDRD |= (1<<TRIG_BIT);     // Set Trigger pin as output
    DDRD &= ~(1<<ECHO_BIT);      // Set Echo pin as input
}


void trigger_sonar(){
	 PORTD &= ~(1<<TRIG_BIT);            // Clear pin before setting it high
    _delay_us(1);           // Clear to zero and give time for electronics to set
    PORTD |=(1<<TRIG_BIT);          // Set pin high
    _delay_us(12);          // Send high pulse for minimum 10us
    PORTD &= ~(1<<TRIG_BIT);             // Clear pin
    _delay_us(1);           // Delay not required, but just in case...
}


ISR(TIMER1_OVF_vect){   // Timer1 overflow interrupt
    overFlowCounter++;
    TCNT1=0;
}


unsigned int read_sonar(){
    int  dist_in_cm = 0;
    init_sonar();                       // Setup pins and ports
    trigger_sonar();                    // send a 10us high pulse

    while(!(PIND & (1<<	ECHO_BIT))){   // while echo pin is still low
        trig_counter++;
         uint32_t max_response_time = SONAR_TIMEOUT;
        if (trig_counter > max_response_time){   // SONAR_TIMEOUT
            return TRIG_ERROR;
        }
    }

    TCNT1=0;                            // reset timer
    TCCR1B |= (1<<CS10);              // start 16 bit timer with no prescaler
    TIMSK |= (1<<TOIE1);             // enable overflow interrupt on timer1
    overFlowCounter=0;                  // reset overflow counter
    sei();                              // enable global interrupts

    while((PIND & (1<<ECHO_BIT))){    // while echo pin is still high
        if (((overFlowCounter*TIMER_MAX)+TCNT1) > SONAR_TIMEOUT){
            return ECHO_ERROR;          // No echo within sonar range
        }
    };

    TCCR1B = 0x00;                      // stop 16 bit timer with no prescaler
    cli();                              // disable global interrupts
    no_of_ticks = ((overFlowCounter*TIMER_MAX)+TCNT1);  // counter count
    dist_in_cm = (no_of_ticks/(CONVERT_TO_CM*CYCLES_PER_US));   // distance in cm
    return (dist_in_cm );
}



