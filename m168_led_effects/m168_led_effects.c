/*
* Hackaday.com AVR Tutorial firmware
*
* m168_led_effects
*
* written by: Mike Szczys (@szczys)
* 11/15/2010
*
* ATmega168
* Produces different light patterns using:
*  - 8 LEDs conneced on PORT D
*
* http://hackaday.com/2010/11/19/avr-programming-04-writing-code-etc/
*/
#define F_CPU 1000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/******************************
* Pin and Setting Definitions *
******************************/

#define ledPort		PORTD
#define ledDDR		DDRD

//Milliseconds delay in sweep mode
#define sweepDelay	30

//Milliseconds delay in flasher mode
#define flasherDelay	110

/************
* Variables *
************/

//Used to count in binary. This must be 'volatile'
//   becaused it is accessed in both ISR and main
volatile unsigned char binary_counter = 0;


/*************
* Prototypes *
*************/

void initIO(void);
void timer1_1Hz(void);
void timer1_stop(void);
void delay_ms(int cnt);
void sweep(unsigned char times);
void flasher(unsigned char times);

/************
* Functions *
************/

//Setup the I/O for the LEDs
void initIO(void)
{
  ledDDR |= 0xFF;			//Set PortD pins as an outputs
  ledPort |= 0xFF;		//Set PortD pins high to turn on LEDs
}

//Setup a 1 Hz timer
void timer1_1Hz(void)
{
  cli();			//Disable global interrupts
  TCCR1B |= 1<<CS11 | 1<<CS10;	//Divide by 64
  OCR1A = 15624;		//Count 15624 cycles for 1 second interrupt
  TCCR1B |= 1<<WGM12;		//Put Timer/Counter1 in CTC mode
  TIMSK1 |= 1<<OCIE1A;		//enable timer compare interrupt
  sei();			//Enable global interrupts
}

//Stop timer 1
void timer1_stop(void)
{
  //Set TCCR1B back to defaults to clear timer source and mode
  TCCR1B &= ~( (WGM12) | (1<<CS12) | (1<<CS11) | (1<<CS10) );
}

//Delay a number of milliseconds
void delay_ms(int cnt)
{
  while (cnt-- > 0) _delay_ms(1);
}

//Sweep one lighted LED back and forth
void sweep(unsigned char times)
{
  //Start with Least Significant Bit illuminated
  unsigned char tracker = 0x01;	//Setup starting value
  ledPort = tracker;		//Output new values to LEDs			
  delay_ms(sweepDelay);		//wait a bit

  while(times--)
  {
    //Shift bits left until 0b10000000 is reached
    while(tracker < 0x80)
    {
      tracker <<= 1;		//Shift bits
      ledPort = tracker;	//Output new values to LEDs
      delay_ms(sweepDelay);	//wait a bit
    }

    //Shift bits right until 0b00000001 is reached
    while(tracker > 0x01)
    {
      tracker >>= 1;		//Shift bits
      ledPort = tracker;	//Output new values to LEDs
      delay_ms(sweepDelay);	//wait a bit
    }
  }
}

//Flash in an XOR pattern
void flasher(unsigned char times)
{
  unsigned char tracker = 0xAA;	//Setup starting value
  ledPort = tracker;		//Lights every-other LED
  delay_ms(flasherDelay);	//wait a bit

  while (times--)
  {
    tracker ^= 0xFF;		//Toggle all values
    ledPort = tracker;		//Output new values to LEDs
    delay_ms(flasherDelay);	//wait a bit
  }
}

int main(void)
{
 
  initIO();		//Setup LED pins
  delay_ms(1000);	//Wait ~1s to see LEDs lit

  while(1) 
  { 
    sweep(10);		//Show sweeping pattern

    flasher(200);	//Show flashing pattern
  
    ledPort = 0;	//Turn off all LEDs
    binary_counter = 0;
    timer1_1Hz();	//Count in binary

    //Trap main function in a loop. ISR will do the incrementing
    while(binary_counter < 0x10) { }

    timer1_stop();	//Counting finished, stop the counter
    ledPort = 0;	//Turn off all LEDs
  }
}

/*********************
* Interrupt Handling *
*********************/

ISR(TIMER1_COMPA_vect)		//Interrupt Service Routine
{

  ledPort = ++binary_counter; //Display and increment the count value
}
