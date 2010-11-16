/*
* Hackaday.com AVR Tutorial firmware
* written by: Mike Szczys (@szczys)
* 11/15/2010
*
* ATmega168
* Blinks 8 LEDs conneced on PORT D
*
* place URL here
*/

#include <avr/io.h>
#include <avr/interrupt.h>

int main(void)
{

  //Setup the clock
  cli();			//Disable global interrupts
  TCCR1B |= 1<<CS11 | 1<<CS10;	//Divide by 64
  OCR1A = 15624;		//Count 15624 cycles for 1 second interrupt
  TCCR1B |= 1<<WGM12;		//Put Timer/Counter1 in CTC mode
  TIMSK1 |= 1<<OCIE1A;		//enable timer compare interrupt
  sei();			//Enable global interrupts

  //Setup the I/O for the LEDs

  DDRD |= 0xFF;			//Set PortD pins as an outputs
  PORTD |= 0xFF;		//Set PortD pins high to turn on LEDs

  while(1) { }			//Loop forever, interrupts do the rest
}

ISR(TIMER1_COMPA_vect)		//Interrupt Service Routine
{
  PORTD ^= 0xFF;		//Use xor to toggle the LEDs
}
