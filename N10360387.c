#define BAUD 9600

// Standard c headers
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// AVR header file 
#include <util/setbaud.h>
#include <util/delay.h>
#include <avr/io.h>

//graphics headers
#include <lcd.h>
#include <macros.h>
#include <ascii_font.h>
#include <graphics.h>

//sensor header
#include "hcsr04.h"

//interrupt
#include <avr/interrupt.h>

/*
 *  Setting data directions in a data direction register (DDR)
 *
 *
 *  Setting, clearing, and reading bits in registers.
 *	reg is the name of a register; pin is the index (0..7)
 *  of the bit to set, clear or read.
 *  (WRITE_BIT is a combination of CLEAR_BIT & SET_BIT)
 */
#define SET_BIT(reg, pin)			(reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin)			(reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value)	(reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin)			(((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin)		(BIT_VALUE((reg),(pin))==1)

//***** UART for sensor and LCD
int uart_putch(char c, FILE *unused) {
	(void) unused;

	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;

	return 0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putch, NULL, _FDEV_SETUP_WRITE);

void uart_init() {
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

	UCSR0A &= ~(1 << U2X0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	stdout = &uart_output;
}

// ****** Function declarations ******
// timers & interrupt
void uart_init1(unsigned int ubrr); 
char uart_getchar(void);
void uart_putchar(unsigned char data);
void uart_putstring(unsigned char* s);

#define MYF_CPU (16000000UL)
#define MYUBRR (MYF_CPU/16/BAUD-1)

//timer definitions
#define FREQ (16000000.0)
#define PRESCALE (1024.0)

//****** Function declarations *******
//debouncing
void uart_init2(unsigned int ubrr);
char uart_getchar2(void);
void uart_putchar2(unsigned char data);
void uart_putstring2(unsigned char* s);

#define DEBOUNCE_MS (50)

//To store button press count 
uint16_t counter = 0;

//UPDATE: Timer overflow counter
volatile int overflow_counter = 0;

// -- SETUP CODE --
void setup()
{
	// UPDATE: initialise uart for timer & interrupt
      uart_init1(MYUBRR);
	
	// UPDATE: Timer 0 in normal mode, with pre-scaler 1024 ==> ~60Hz overflow.
	// Timer overflow on.
	TCCR0A = 0;
	TCCR0B = 5;
	TIMSK0 = 1;
	
	
    // UPDATE: Enable B5 as output, led on C1
	SET_BIT(DDRC,1);

	// UPDATE: Enable timer overflow, and turn on interrupts.
	sei();
	
	//init uart for debouncing
	uart_init2(MYUBRR);
	
	
	SET_BIT(DDRC,2); // set LED in  PORTB pin 3 for output "Safe signal led" to C2
	SET_BIT(DDRB,2); //set LED in PORTB pin 2 for output "Warning signal led
	
    CLEAR_BIT(DDRB,0); // set Button in PORTB pin 0 for input "turn on app button"
	CLEAR_BIT(DDRB,1); //set Button in PORTB pin 1 for input "Detect distance button"
	CLEAR_BIT(DDRC,0); //set Button in PORTC pin 0 for input "Exit program button"
	
	//initialise screen with low contrast. see lcd.h for options
	lcd_init(LCD_LOW_CONTRAST);
	//zero screen buffer (memory)
	clear_screen();
	//write buffer to LCD
	show_screen();
}


// -- LCD CODE --

//Display welcome message to screen
void welcome_message()
{
	char *welcome= "WELCOME!";
	char *hello= "Hello CAB202";
	char *intro = "I am a COVID-19";
	char *purpose = "social distance";
	char *purpose1 = "checker. :) ";
	
  clear_screen();
  //draw string using foreground or background options
  //line 1-6
  draw_string(0, 0, welcome, BG_COLOUR); 
  draw_string(0, 8, hello, FG_COLOUR);
  draw_string(0, 16, intro, FG_COLOUR);
  draw_string(0, 24, purpose,FG_COLOUR);
  draw_string(0, 32,purpose1, FG_COLOUR);
  
  show_screen();
  
  _delay_ms(100);
}

//Display detecting message to screen
void detecting_message()
{
	char* press_start = "Press";
	char* press_message = "button to detect";
	char* press_distance_message = "person within 12";
	char* radius = "inch.";

  clear_screen();
  draw_string(0, 0, press_start, FG_COLOUR); 
  draw_string(0, 8, press_message, FG_COLOUR);
  draw_string(0, 16, press_distance_message, FG_COLOUR);
  draw_string(0, 24, radius ,FG_COLOUR);

  show_screen();
  
  _delay_ms(100);
}

//Display social distancing reminder on screen
void too_close_meesage()
{
	char* warning = "PERSON TOO CLOSE!!";
	char* distance_message1 = "Practice social";
	char* distance_message2= "distancing";

  clear_screen();
  draw_string(0, 8, warning, FG_COLOUR); 
  draw_string(0, 16, distance_message1, FG_COLOUR);
  draw_string(0, 24, distance_message2, FG_COLOUR);

  show_screen();
  
  _delay_ms(100);
}

//Display safe message on screen
void safe_message()
{
	char* safe = "ALL GOOD!!";
	char* social_distancing1 = "Social";
	char* social_distancing2 = "distancing ";
	char* social_distancing3 = "is practiced. :)";

  clear_screen();
  draw_string(0, 8, safe, FG_COLOUR); 
  draw_string(0, 16, social_distancing1, FG_COLOUR);
  draw_string(0, 24, social_distancing2, FG_COLOUR);
  draw_string(0, 32, social_distancing3, FG_COLOUR);

  show_screen();
  
  _delay_ms(100);
}

//Display activating sensor message on screen
void standby_sensor()
{
	char* sensor_act1= "Sensor on";
	char* sensor_act2= "standby...";

  clear_screen();
  draw_string(0, 8, sensor_act1, FG_COLOUR); 
  draw_string(0, 16, sensor_act2, FG_COLOUR); 

  show_screen();
  
  _delay_ms(100);
}

void bye_message()
{
	char* bye = "Goodbye CAB202";

  clear_screen();
  draw_string(0, 8,bye, FG_COLOUR); 

  show_screen();
  
  _delay_ms(100);
}

// -- LED CODE --

void turn_on_safe_LED()
{
    SET_BIT(PORTC,2); // turn ON LEDs by sending '1' to pin 3 B2 TO C2
}

void turn_on_warning_LED()
{
	SET_BIT(PORTB,2);//turn ON LEDs by sending '1' to pin 2  
}

// Turn on blinking safe_led
void safe_led()
{
     turn_on_safe_LED(); // Turn ON long LED
     _delay_ms(1000);  // Set voltage back to 0
     CLEAR_BIT(PORTB,2); // Turn OFF long LED
     _delay_ms(1000);
}

//turn on blinking warning led
void warning_led()
{
	 turn_on_warning_LED(); // Turn ON long LED
     _delay_ms(1000);  // Set voltage back to 0
     CLEAR_BIT(PORTC,2); // Turn OFF long LED 
     _delay_ms(1000);
}

// -- SPEAKER CODE --
void warning_signal_speaker(void)
{
	// set PB3 as an output
    DDRB|= (1 << PB3);
 
 	//make the timer overflow at 128 (we'll use Fast PWM mode, TOP=OCRA)
    OCR2A = 128;
    //50% duty cycle (50% of 128)
    OCR2B = 64;
 
    // set none-inverting mode
    TCCR2A |= (1 << COM2A1);
 
    // TinkerCAD Errata: timer clocking must be enabled before WGM
    // set prescaler to 1024 (overflow ~61hz) and starts PWM
    TCCR2B = (1 << CS22) | (1 << CS20);
 
 	 // set fast PWM Mode, overflowing at OCR0A
    TCCR2A |= (1 << WGM21) | (1 << WGM20);
    TCCR2B |= (1 << WGM22);//WGM02 is in a different control register for some reason

    
    	//Don't make the frequency sweep too fast
    	_delay_ms(2);

    	//increment the TOP value, lowering the frequency
    	OCR2A++;
    	//if it overflowed, set it to a minimum value
    	if (OCR2A < 64) OCR2A = 64;

    	//keep it at 50% duty cycle
    	OCR2B = OCR2A/2;

}

//-- SENSOR CODE ---
void sensor()
{
	//start serial comms
	uart_init();
	//initialise LCD
	lcd_init(LCD_LOW_CONTRAST);
	clear_screen();
	show_screen();

	// setting up Distance env_sensor
	// ECHO on B4
	// TRIG on B5
	hcsr04 distance_sensor;
	hcsr04_setup(&distance_sensor, &PORTB, 5, &PORTB, 4);

	while(1){

		char src[20];
		char dest[20];
		clear_screen();

		// reading distance
		double distance;

		////send values to UART
		if (hcsr04_read(&distance_sensor, &distance)) {
			printf("Distance read: %d mm\n", (int) (distance * 1000));
		} else {
			printf("Failed to read distance\n");
		}
		dtostrf((1000.0*distance),7, 3, src);
		strcpy(dest, "D=");
		strcat(dest, src);
		
		
		if((int) (distance * 1000) == -1000)
		{
			standby_sensor();
		}
		else if ((int) (distance * 1000) < 244)
		{
			too_close_meesage();
			warning_led();
			CLEAR_BIT(PORTC,2);
			warning_signal_speaker();
		}
		else if((int) (distance * 1000) > 244)
		{
			safe_message();
			safe_led();
			CLEAR_BIT(PORTB,2);
		}
				
		draw_string(0, 0, dest, FG_COLOUR);
	
		show_screen();
		_delay_ms(2000);
	}
}

// Function that prompts detection
void start_detect()
{
	for(;;)
	{
		sensor();
	}
}




//*************** timers, interrupt, and debouncing*****************

//define a buffer to be sent for debouncing
unsigned char temp_buf_debounce[64];
char temp_debounce_label[64];
char temp_interrupt_label[64];

// Start application
void start_app()
{
	//UPDATE: buffer for timers & interrupt
	char temp_buf_interrupt[64];

    //UPDATE: compute elapsed time for timers & interrupt
	double time = ( overflow_counter * 256.0 + TCNT0 ) * PRESCALE  / FREQ;
	
	//UPDATE: convert float to a string for timer and interrupt
	dtostrf(time,7,3,temp_buf_interrupt); //7,3
	strcpy(temp_interrupt_label, "Time I.=");
	strcat(temp_interrupt_label,temp_buf_interrupt);

	//UPDATE: send serial data for timer and interrupt
	uart_putstring((unsigned char *) temp_buf_interrupt);
	uart_putchar('\n');
	
	 //UPDATE:flash LED every cycle for timer and interrupt 
	PORTC^=(1<<PINC1);
	 
	 // display
	 draw_string(6,38,temp_interrupt_label, FG_COLOUR);
	 show_screen();
	 _delay_ms(2000);
	 clear_screen();

    //convert number to string fr debouncing
     itoa(counter,(char*)temp_buf_debounce,10);
	 strcpy(temp_debounce_label, "Press count = ");
	 strcat(temp_debounce_label,(char*)temp_buf_debounce);
	
	//display debounce buffer
	 draw_string(0,38,(char *)temp_debounce_label, FG_COLOUR);
	 show_screen();
	 _delay_ms(2000);
	 clear_screen();
	 
}

//Interrupt Service Routine
ISR(TIMER0_OVF_vect) {
	overflow_counter ++;
	
	if(BIT_IS_SET(PINB,0))
	{
		_delay_ms(DEBOUNCE_MS);
		
		while(BIT_IS_SET(PINB,0)) // start program when PB0 button is pressed
		{
			//clear_screen();
			welcome_message();
			_delay_ms(4000);
			detecting_message();
		}
		
		counter++;
	}
	else if(BIT_IS_SET(PINB,1)) //Start detecting when PB1 button is pressed
	{
		uart_putstring(temp_buf_debounce);
		
		while(BIT_IS_SET(PINB,1))
		{
			start_detect();
		}
	}
	else if (BIT_IS_SET(PINC,0)) // End Program, NOT PROPERLY ASSIGNED YET!!
	{
		bye_message();
		_delay_ms(4000);
		exit(1);
	}
}

//-- MAIN CODE--
int main() 
{		
	setup();
	for(;;)
	{
		start_app();
		_delay_ms(1000);
	}
	
	return 0;
}

/********* serial definitions for timers and interrupts****************/

// Initialize the UART
void uart_init1(unsigned int ubrr) {

	UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)(ubrr);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);

}

// Transmit a data
void uart_putchar(unsigned char data) {

    while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer*/

  	UDR0 = data;            /* Put data into buffer, sends the data */

}

// Receive data
char uart_getchar(void) {

  /* Wait for data to be received */
  while ( !(UCSR0A & (1<<RXC0)) );


/* Get and return received data from buffer */
return UDR0;

}

// Transmit a string
void uart_putstring(unsigned char* s)
{
    // transmit character until NULL is rea
}

/********* serial definitions for debouncing****************/
// Initialize the UART
void uart_init2(unsigned int ubrr) {

	UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)(ubrr);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00);

}

// Transmit a data
void uart_putchar2(unsigned char data) {

    while (!( UCSR0A & (1<<UDRE0))); /* Wait for empty transmit buffer*/

  	UDR0 = data;            /* Put data into buffer, sends the data */

}

// Receive data
char uart_getchar2(void) {

/* Wait for data to be received */
  while ( !(UCSR0A & (1<<RXC0)));


/* Get and return received data from buffer */
return UDR0;

}

// Transmit a string
void uart_putstring2(unsigned char* s)
{
    // transmit character until NULL is reached
    while(*s > 0) uart_putchar(*s++);
}


/*************************************************************************
REFERENCE: some codes from lecture examples were used in this assignment. *
**************************************************************************/

