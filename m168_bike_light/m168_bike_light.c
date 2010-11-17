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
* Timer0 used for debounce
* Timer1 takes place of delay functions
* Button cycles through modes
* Final mode is sleep
* Pin interrupt wakes from sleep
*
* place URL here
*/
#define F_CPU 1000000L

#include <avr/io.h>
#include <avr/interrupt.h>

/******************************
* Pin and Setting Definitions *
******************************/

//LED definitions
#define ledPort		PORTD
#define ledDDR		DDRD

//Button definitions
#define KEY_PORT	PORTC
#define KEY_DDR		DDRC
#define KEY_PIN		PINC	//Needed for debounce ISR
#define KEY0		0 	//PC0

//Delay values used with Timer1
//  1 ms = 15.625
#define sweepDelay	469	//About 30ms
#define xorDelay	3125	//About 200ms
#define flashDelay	1719	//About 110ms

//State definitions
#define sweep		0
#define xor		1
#define flash		2
#define	alternate	3
#define sleep		4

/************
* Variables *
************/

//State machine variable
unsigned char state;

//Delay flag used by interrupt
volatile unsigned char timer = 0;

//Sweep variables
unsigned char tracker;
unsigned char direction;


//Debounce
unsigned char debounce_cnt = 0;
volatile unsigned char key_press;
unsigned char key_state;

/*************
* Prototypes *
*************/

void initIO(void);
void initTimer1_modes(void);
void initState(unsigned char state);
void timer0_overflow(void);
void start_timer1_compare(unsigned int cycle_count);
void timer1_stop(void);
unsigned char get_key_press( unsigned char key_mask );
void toggle_led(void);
//void sweep(void);

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

void initTimer1_modes(void)
{
  cli();			//Disable global interrupts
  TIMSK1 |= 1<<OCIE1A;		//enable timer compare interrupt
  TCCR1B |= 1<<WGM12;		//Put Timer/Counter1 in CTC mode
  sei();			//Enable global interrupts
}

//Initialize the state passed in as a variable
void initState(unsigned char state)
{
  switch (state)
  {
    case sweep:
      tracker = 0x01;	//Starting bitmask
      direction = 1;	//Starting direction (1=ascending, 0=descending)
      start_timer1_compare(sweepDelay);
      break;

    case xor:
      ledPort = 0x0F;
      start_timer1_compare(xorDelay);
      break;

    case flash:
      ledPort = 0xFF;
      start_timer1_compare(flashDelay);
      break;

    case alternate:
      break;

    case sleep:
      break;
  }
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

//Setup a 1 Hz timer
void start_timer1_compare(unsigned int cycle_count)
{
  cli();			//Disable global interrupts
  OCR1A = cycle_count;		//Count cycles for timed interrupt
  TCCR1B |= 1<<CS11 | 1<<CS10;	//Divide by 64
  sei();			//Enable global interrupts
}

//Stop timer 1
void timer1_stop(void)
{
  //Set TCCR1B back to defaults to clear timer source and mode

  cli();
  TCCR1B &= ~( (1<<CS11) | (1<<CS10) );
  TCNT1 = 0;			//Reset counter so we start counting at 0 next time
  sei();
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

//Toggles pins on ledPort
void toggle_led(void)
{
  ledPort ^= 0xFF;
}

/*Sweep one lighted LED back and forth
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
*/

int main(void)
{
 
  initIO();		//Setup LED and Button pins
  timer0_overflow();	//Setup the button debounce timer
  initTimer1_modes();	//Setup the delay timer

  state = sweep;	//Set default state
  initState(state);	//Setup initial state

  while(1) 
  {
    //If interrupt set the timer flag act on that
    if (timer) {
      switch (state)
      {
        case sweep:
          if (direction) tracker <<= 1;
          else tracker >>= 1;

          if ((tracker == 0x01) | (tracker == 0x80)) direction ^= 1;

          ledPort = tracker;
          break;

        case xor:
          toggle_led();
          break;

        case flash:
          toggle_led();
          break;

        case alternate:
          break;

        case sleep:
          break;
      }
      timer = 0;
    }

    if( get_key_press( 1<<KEY0 )) {
      timer1_stop();	//Halt the delay timer
      timer = 0;	//Clear delay timer flag
      
      //Increment the state machine
      if (++state > flash) state = sweep;
      initState(state);
    }
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

ISR(TIMER1_COMPA_vect)		//Interrupt Service Routine
{
  timer = 1;
}
