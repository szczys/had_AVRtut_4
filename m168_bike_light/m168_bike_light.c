/*
* Hackaday.com AVR Tutorial firmware
*
* m168_bike_light
*
* written by: Mike Szczys (@szczys)
* 11/15/2010
*
* ATmega168
* Bicycle tail light using
*  - 8 LEDs conneced on PORT D
*  - A button on PC0
*
* place URL here
*/
#define F_CPU 1000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/******************************
* Pin and Setting Definitions *
******************************/

//LED deffinitions
#define ledPort		PORTD
#define ledDDR		DDRD

//Button definitions
#define KEY_PORT	PORTC
#define KEY_DDR		DDRC
#define KEY_PIN		PINC	//Needed for debounce ISR
#define KEY0		0 	//PC0

//Milliseconds delay in sweep mode
#define sweepDelay	30

//Milliseconds delay in flasher mode
#define flasherDelay	110

//State definitions
#define sweep		0
#define xor		1
#define flash		2
#define	solid		3
#define sleep		4

/************
* Variables *
************/

//State machine variable
unsigned char state = sweep;

//Debounce
unsigned char debounce_cnt = 0;
volatile unsigned char key_press;
unsigned char key_state;

/*************
* Prototypes *
*************/

void initIO(void);
void timer0_overflow(void);
void delay_ms(int cnt);
unsigned char get_key_press( unsigned char key_mask );
void sweep(void);
void flasher(void);

/************
* Functions *
************/

//Setup the I/O for the LEDs
void initIO(void)
{
  ledDDR |= 0xFF;		//Set PortD pins as an outputs
  ledPort |= 0xFF;		//Set PortD pins high to turn on LEDs

  KEY_DDR &= ~(1<<KEY0);	//Set PC0 as an input
  KEY_PORT |= (1<<KEY0);	//Enable internal pull-up resistor of PC0
}

//Setup a timer for button debounce
void timer0_overflow(void)
{
  cli();
  //Timer0 for buttons
  TCCR0B |= 1<<CS02 | 1<<CS00;		//Divide by 1024
  TIMSK0 |= 1<<TOIE0;			//enable timer overflow interrupt
  sei();
}

//Delay a number of milliseconds
void delay_ms(int cnt)
{
  while (cnt-- > 0) _delay_ms(1);
}

//Danni Debounce Function
unsigned char get_key_press( unsigned char key_mask )
{
  cli();               // read and clear atomic !
  key_mask &= key_press;                        // read key(s)
  key_press ^= key_mask;                        // clear key(s)
  sei();
  return key_mask;
}

//Sweep one lighted LED back and forth
void sweep(void)
{
  //Start with Least Significant Bit illuminated
  unsigned char tracker = 0x01;	//Setup starting value
  ledPort = tracker;		//Output new values to LEDs			
  delay_ms(sweepDelay);		//wait a bit

  while(1)	//Loop forever
  {
    //Shift bits left until 0b10000000 is reached
    while(tracker < 0x80)
    {
      tracker <<= 1;		//Shift bits
      ledPort = tracker;	//Output new values to LEDs
      delay_ms(sweepDelay);	//wait a bit
      if( get_key_press( 1<<KEY0 )) return;	//Leave function on button press
    }

    //Shift bits right until 0b00000001 is reached
    while(tracker > 0x01)
    {
      tracker >>= 1;		//Shift bits
      ledPort = tracker;	//Output new values to LEDs
      delay_ms(sweepDelay);	//wait a bit
      if( get_key_press( 1<<KEY0 )) return;	//Leave function on button press
    }
  }
}

//Flash in an XOR pattern
void flasher(void)
{
  unsigned char tracker = 0xAA;	//Setup starting value
  ledPort = tracker;		//Lights every-other LED
  delay_ms(flasherDelay);	//wait a bit

  while (1)	//Loop forever
  {
    tracker ^= 0xFF;		//Toggle all values
    ledPort = tracker;		//Output new values to LEDs
    delay_ms(flasherDelay);	//wait a bit
    if( get_key_press( 1<<KEY0 )) return;	//Leave function on button press
  }
}

int main(void)
{
 
  initIO();		//Setup LED and Button pins
  timer0_overflow();	//Setup the button debounce timer

  while(1) 
  {
    switch (state)
    {
      case sweep:
        
        break;

    //sweep();		//Show sweeping pattern
    //flasher();	//Show flashing pattern
  
    {
  }
}

/*********************
* Interrupt Handling *
*********************/

//Button debounce ISR
ISR(TIMER0_OVF_vect)           // every 10ms
{
  static unsigned char ct0, ct1;
  unsigned char i;

  TCNT0 = (unsigned char)(signed short)-(F_CPU / 1024 * 10e-3 + 0.5);   // preload for 10ms

  i = key_state ^ ~KEY_PIN;    // key changed ?
  ct0 = ~( ct0 & i );          // reset or count ct0
  ct1 = ct0 ^ (ct1 & i);       // reset or count ct1
  i &= ct0 & ct1;              // count until roll over ?
  key_state ^= i;              // then toggle debounced state
  key_press |= key_state & i;  // 0->1: key press detect
}