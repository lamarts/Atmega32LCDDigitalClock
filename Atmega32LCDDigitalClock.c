/*
 * ****************************************************************************************************************
 * Atmega32LCDDigitalClock.c
 * (1) This program demonstrates the use of the HD74480 LCD with an Atmega32 8-bit microcontroller
 * (2)This program utilizes the LCD in 4-bit mode - to set, reset (buttons) and display time in minutes, hours and seconds
 * (3) program utilizes the Timer interrupt feature in the Atmega32 microcontroller
 * The interrupt timer onboard the chip was used to generate the 1-second timer using the internal clock set @ 4MHz
 *
 * AtmelStudio 6 was used to develop code, compile and upload firmware to the chip using a usbasp ISP programmer
 *
 * The Atmega32 datasheet (doc2503.pdf) can be found at www.atmel.com and is used throughout the code as reference material
 *
 *
 * The UART functions will be added as part of next version of this program to demonstrate control from a PC
 *
 * This code was tested and passed all tests
 *
 * Created: 9/22/2015 6:04:59 PM
 *  Author: lamarts
 *******************************************************************************************************************
 */  

#define F_CPU 4000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// GPIO PORT Initialization
#define LCDCONTROL			PORTA   // PA0 through PA2 for RS, RW and Enable for LCD control
#define LCDDATA				PORTA     // PA4 through PA7 for LCD data...D4, D5, D6 and D7
 
#define LCD_DIR				DDRA

#define TIMEPORT			PORTD
#define TIME_DIR			DDRD
#define TIME_PIND			PIND


#define RegisterSel		PINA0     // Pin for Port A
#define ReadWrite			PINA1	    // Pin for Port A
#define Enable				PINA2	    // Pin for Port A
#define SETHOUR				PIND3     // Pin for Port D
#define SETMINUTE			PIND4     // Pin for Port D


// LCD Instructions - Can be used for MACROS...but I did not use these in this code...just lazy
#define LCD_Clear			    0x01
#define LCD_Home			    0x02
#define LCD_EntryMode			0x06
#define LCD_DisplayOff		Ox08
#define LCD_DisplayOn			Ox0C
#define LCD_FunctReset		0x30
#define LCD_FunctSet4bit	0x28
#define LCD_SetCursor			0x80

#define LCDMaxLines			  2
#define LCDMaxCharacters	16
#define LCDLineOne			  0x80
#define LCDLineTwo			  0xC0


ISR(TIMER1_COMPA_vect);			// Timer Interrupt

// Global variables
static volatile int HOU = 00;		// Initial Hour
static volatile int MIN = 00;		// Initial Minute
static volatile int SEC = 00;   // Initial Second

char displaySec[2];
char displayMin[2];
char displayHou[2];

// Function Prototypes
// LCD Control functions
void LCD_Init(void);				              // Initializes the LCD
void LCD_SendCommand(unsigned char cmd);	// Sends command to the LCD
void LCD_SendData(unsigned char data);		// Sends data to the LCD
void LCD_SendString(char *string);		    // Sends string to the	LCD
void LCD_CustomCharacters(void);		      // Sends customized characters to the LCD
void LCD_DisplayTime();				            // Display time
void LCD_SetTime(void);				            // Set time
void LCD_EnterTime(void);			            // Complete adjusting time - changes display from "Set Time"

// To be used for UART to PC communication - not used in this version of program
void UART_Transmit(unsigned char sendData);	


int main(void)
{
	// initialization code
	// Button pressed after desired time is entered
	DDRB &= ~(1<<PINB0);		// Data direction is for input
	PORTB |= (1<<PINB0);		// Set for "high" when not pressed...low (ground) when button is pressed...See SetTime() function
	
	// Timer/Counter1 Control Register B - page 119 of the datasheet
	// Clock Select Bit and Waveform Generation Mode
	TCCR1B &= ~(1<<CS12|1<<WGM12);  // No clock source - starts clock at "00:00:00"...will turn on later after setting time 
	// Output Compare Register 1
	OCR1A = 15625-1;        // formula for 1 second timer
	

	// Timer/Counter Interrupt Mask Register - page 112 of the datasheet
	TIMSK |= 1<<OCIE1A;			// Output Compare A Match Interrupt Enable
	
	// Enable global interrupts
	sei();
	
	// Initialization UART communication - - page 146  - this is for UART to PC
	// Not implemented in this version of the program
	// Set baud rate - 9600
	uint16_t ubrr_value = 25;   // See Table 69 on page 166 of the datasheet
	
	UBRRH = (ubrr_value>>8);
	UBRRL = ubrr_value;
	
	// Enable receiver and transmitter 
	UCSRB = (1<<RXEN) | (1<<TXEN);
	
	// Set frame format: 8 bit data, 1 stop bit - page 162
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
	
	//////////////////////////////////////////////////////////////////////////////

	// Set Data direction for outputs: LCD
	LCD_DIR |= (1<<RegisterSel | 1<<ReadWrite | 1<<Enable);

	// Set Data direction for inputs: (2) pushbuttons
	TIME_DIR &= ~(1<<SETHOUR | 1<<SETMINUTE);	// inputs for buttons, set hour, set minute
	TIMEPORT |= (1<<SETHOUR | 1<<SETMINUTE);
	
	LCD_Init();
	LCD_SendString("Set Current Time:");		// LCD info on first line
	LCD_SendCommand(0xC0);                  // LCD go to second line, first position
	LCD_SendString("00:00:00");             // LCD info on second line
	_delay_ms(15);
			
    	while(1)
    	{ 
		LCD_SetTime();
    	}
}
// UART Function to be used for next part of code - NOT USED HERE
void UART_Transmit(unsigned char sendData)
{
	while(!(UCSRA & (1<<UDRE)))
	{
				
	}
	UDR = sendData;
}
// Initialize LCD
void LCD_Init()
{
	LCD_DIR = 0xff;
	LCD_SendCommand(0x02);	// initialize the LCD in 4 bit mode
	_delay_us(50);
	LCD_SendCommand(0x28);  // LCD 2 lines, 5x7 matrix
	_delay_us(50);
	LCD_SendCommand(0x0C);  // Display On, cursor On
	_delay_us(50);	
	LCD_SendCommand(0x01);  // LCD clear
	_delay_ms(2);
	LCD_SendCommand(0x80);  // Go to 1st line 1st column
}

void LCD_DisplayTime()
{
	// Display hours
	LCD_SendCommand(0xC0);			    // LCD cursor on second line first position
	itoa(HOU/10, displayHou, 10);		// integer to ascii function - can be found in stdlib.h
	LCD_SendString(displayHou);
	itoa(HOU%10, displayHou, 10);
	LCD_SendString(displayHou);
	LCD_SendString(":");
	LCD_SendCommand(0xC0 +3);
	
	// Display minutes
	itoa(MIN/10, displayMin, 10);
	LCD_SendString(displayMin);
	itoa(MIN%10, displayMin, 10);
	LCD_SendString(displayMin);
	LCD_SendCommand(0xC0 +5);
	LCD_SendString(":");
	
	// Display seconds
	LCD_SendCommand(0xC0 +6);
	itoa(SEC/10, displaySec, 10);
	LCD_SendString(displaySec);
	itoa(SEC%10, displaySec, 10);
	LCD_SendString(displaySec);
	LCD_SendString("  ");
	_delay_ms(250);
}

// Function Definitions
void LCD_SetTime()
{
	// Set HOURS using button
	if(bit_is_clear(PIND, SETHOUR))
	{
		HOU++;
		_delay_ms(250);
		if(HOU>23)
		HOU=0;
		LCD_SendCommand(0xC0);
		LCD_DisplayTime();
	}
	// Set MINUTES using button
	if(bit_is_clear(PIND, SETMINUTE))
	{
		MIN++;
		_delay_ms(250);
		if(MIN>59)
		MIN=0;
		LCD_SendCommand(0xC0);
		LCD_DisplayTime();
	}
	// Start Digital Clock 
	if(bit_is_clear(PINB, 0))
	{
		TCCR1B = (1<<CS12|1<<WGM12);		// Clock source is now set after setting time
		LCD_SendCommand(0x80);
		LCD_SendString("Time:           ");	// LCD info on first line
		LCD_SendCommand(0xC0+10);		        // LCD info on second line 10 digits to the right
		LCD_CustomCharacters();			        // Added smiley face on the second line
	}

}

ISR(TIMER1_COMPA_vect)					// Timer Interrupt on Table 18
{
	if(SEC<60)
	{
		SEC++;
	}
	if(SEC>59)
	{
		if(MIN<60)
		{
			MIN++;
		}
		SEC=0;
	}
	if(MIN>59)
	{
		HOU++;
		MIN=0;
		if(HOU>23)
		{
			HOU = 0;
		}
	}
	LCD_DisplayTime();

}

void LCD_SendCommand(unsigned char cmd)
{
	LCDDATA = (cmd & 0xF0);			// used for 4-bit mode...
	LCDCONTROL &= ~(1<<RegisterSel);
	LCDCONTROL &= ~(1<<ReadWrite);
	LCDCONTROL |= 1<<Enable;
	_delay_us(1);
	LCDCONTROL &= ~(1<<Enable);
	_delay_us(10);
	
	LCDDATA = ((cmd << 4) & 0xF0);
	LCDCONTROL &= ~(1<<RegisterSel);
	LCDCONTROL &= ~(1<<ReadWrite);
	LCDCONTROL |= 1<<Enable;
	_delay_us(1);
	LCDCONTROL &= ~(1<<Enable);
	_delay_ms(10);
}

void LCD_SendData(unsigned char data)
{
	LCDDATA = (data & 0xF0);
	LCDCONTROL |= 1<<RegisterSel;
	LCDCONTROL &= ~(1<<ReadWrite);
	LCDCONTROL |= 1<<Enable;
	_delay_us(1);
	LCDCONTROL &= ~(1<<Enable);
	_delay_us(10);
	
	LCDDATA = ((data << 4) & 0xF0);
	LCDCONTROL |= 1<<RegisterSel;
	LCDCONTROL &= ~(1<<ReadWrite);
	LCDCONTROL |= 1<<Enable;
	_delay_us(1);
	LCDCONTROL &= ~(1<<Enable);
	_delay_ms(1);	
}

void LCD_SendString(char *string)
{
	while(*string)
	LCD_SendData(*string++);
}

void LCD_CustomCharacters()
{
	// Address where customized character is stored
	LCD_SendCommand(120);	
	
	// Definition of first custom character - smiley face :-)
	LCD_SendData(0);			// first byte
	LCD_SendData(0);			// second byte
	LCD_SendData(10);			// third byte
	LCD_SendData(0);			// fourth byte
	LCD_SendData(4);			// fifth byte
	LCD_SendData(17);			// sixth byte
	LCD_SendData(14);			// seventh byte
	LCD_SendData(0);			// eighth byte
	
	// Location of LCD where first character is to be displayed
	LCD_SendCommand(0xC0+10);
	LCD_SendData(7);
	_delay_ms(10);	
}
