/*
 * ece295_I2C_tutorial.c
 *
 * Created: 9/2/2021 5:38:25 PM
 * Author : amack
 */ 

#define F_CPU		8000000UL // Clock speed

#include <avr/io.h>
#include <util/delay.h> //for delay function
#include <stdio.h>

#include "i2c.h"
#include "screen_cmds.h"

int input_length = 0;
int current_line = COMMAND_SET_CURSOR_LINE_1;
void screen_init(void)
{
	I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);
	
	I2Csendcmd(SCREEN_ADDR, COMMAND_8BIT_4LINES_NORMAL_RE1_IS0);
	I2Csendcmd(SCREEN_ADDR, COMMAND_NW);
	I2Csendcmd(SCREEN_ADDR, COMMAND_SEGMENT_BOTTOM_VIEW);
	I2Csendcmd(SCREEN_ADDR, COMMAND_BS1_1);
	I2Csendcmd(SCREEN_ADDR, COMMAND_8BIT_4LINES_RE0_IS1);
	I2Csendcmd(SCREEN_ADDR, COMMAND_BS0_1);
	I2Csendcmd(SCREEN_ADDR, COMMAND_FOLLOWER_CONTROL);
	I2Csendcmd(SCREEN_ADDR, COMMAND_POWER_BOOSTER_CONTRAST);
	I2Csendcmd(SCREEN_ADDR, COMMAND_SET_CONTRAST_1010);
	I2Csendcmd(SCREEN_ADDR, COMMAND_8BIT_4LINES_RE0_IS0);
	I2Csendcmd(SCREEN_ADDR, COMMAND_DISPLAY_ON_CURSOR_ON_BLINK_ON);
	
	I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);
}

void screen_write_string(char string_to_write[])
{
	int letter=0;
	
	//I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);
	//I2Csendcmd(SCREEN_ADDR, COMMAND_SET_CURSOR_LINE_1);
	
	
	while(string_to_write[letter]!='\0')
	{
		if ((input_length % LINE_LENGTH == 0 ))
		{
			if (current_line == COMMAND_SET_CURSOR_LINE_4){
				current_line = COMMAND_SET_CURSOR_LINE_1;// We've gone past the end of the line, go to the next one
				I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);
			}
			else
			current_line = current_line+0x20;
			I2Csendcmd(SCREEN_ADDR, current_line); // We've gone past the end of the line, go to the next one
		}
		
		I2Csenddatum(SCREEN_ADDR, string_to_write[letter]);
		letter++;
		
		/*I2Csendcmd(SCREEN_ADDR, COMMAND_SET_CURSOR_LINE_4);
		I2Csenddatum(SCREEN_ADDR, char(input_length));*/
	}
}


int main(void)
{
	DDRA = 0x0f; //PortA as output (only need PA0 for display)
	//char string_to_write1[15] = "pinb4 is on";
	//char string_to_write2[15] = "1";
	
	_delay_ms(5);
	PORTA = PORTA | (0<<PA0); // turn off
	_delay_ms(200);
	PORTA = PORTA | (1<<PA0); // turn on display
	_delay_ms(5);
	
	//Set up I2C
	I2Cinit();
	
	//Initialize display
	// ...
	screen_init();
	
    //Write some data
	//screen_write_string(string_to_write2);
	
	
	DDRB = 0x0f;
	uint8_t pin_value [4] = {1,1,1,1};
	
	_delay_ms(10);
    while (1) 
    {	
		
		PORTB = 0xfe;
		_delay_ms(5);
		if (~(pin_value[0]) & (PINB>>PINB4 & 0b1)){pin_value[0] = 1;}
		else if (pin_value[0] & ~(PINB>>PINB4 & 0b1)){input_length++;screen_write_string("D");pin_value[0] = 0;}
		
		if (~(pin_value[1]) & (PINB>>PINB5 & 0b1)){pin_value[1] = 1;}
		else if (pin_value[1] & ~(PINB>>PINB5 & 0b1)){input_length++;screen_write_string("#");pin_value[1] = 0;}
	
		if (~(pin_value[2]) & (PINB>>PINB6 & 0b1)){pin_value[2] = 1;}
		else if (pin_value[2] & ~(PINB>>PINB6 & 0b1)){input_length++;screen_write_string("0");pin_value[2] = 0;}
		
		if (~(pin_value[3]) & (PINB>>PINB7 & 0b1)){pin_value[3] = 1;}
		else if (pin_value[3] & ~(PINB>>PINB7 & 0b1)){input_length++;screen_write_string("*");pin_value[3] = 0;}
	
	
		
	//0b11111101	
		PORTB = 0xfb;
		_delay_ms(5);
		if (~(pin_value[0]) & (PINB>>PINB4 & 0b1)){pin_value[0] = 1;}
		else if (pin_value[0] & ~(PINB>>PINB4 & 0b1)){input_length++;screen_write_string("B");pin_value[0] = 0;}
		
		if (~(pin_value[1]) & (PINB>>PINB5 & 0b1)){pin_value[1] = 1;}
		else if (pin_value[1] & ~(PINB>>PINB5 & 0b1)){input_length++;screen_write_string("6");pin_value[1] = 0;}
	
		if (~(pin_value[2]) & (PINB>>PINB6 & 0b1)){pin_value[2] = 1;}
		else if (pin_value[2] & ~(PINB>>PINB6 & 0b1)){input_length++;screen_write_string("5");pin_value[2] = 0;}
		
		if (~(pin_value[3]) & (PINB>>PINB7 & 0b1)){pin_value[3] = 1;}
		else if (pin_value[3] & ~(PINB>>PINB7 & 0b1)){input_length++;screen_write_string("4");pin_value[3] = 0;}
	
	
	//0b11111011
		PORTB = 0xfc;
		_delay_ms(5);	
		if (~(pin_value[0]) & (PINB>>PINB4 & 0b1)){pin_value[0] = 1;}
		else if (pin_value[0] & ~(PINB>>PINB4 & 0b1)){I2Csendcmd(SCREEN_ADDR, COMMAND_CLEAR_DISPLAY);pin_value[0] = 0;}
		
		if (~(pin_value[1]) & (PINB>>PINB5 & 0b1)){pin_value[1] = 1;}
		else if (pin_value[1] & ~(PINB>>PINB5 & 0b1)){input_length++;screen_write_string("9");pin_value[1] = 0;}
	
		if (~(pin_value[2]) & (PINB>>PINB6 & 0b1)){pin_value[2] = 1;}
		else if (pin_value[2] & ~(PINB>>PINB6 & 0b1)){input_length++;screen_write_string("8");pin_value[2] = 0;}
		
		if (~(pin_value[3]) & (PINB>>PINB7 & 0b1)){pin_value[3] = 1;}
		else if (pin_value[3] & ~(PINB>>PINB7 & 0b1)){input_length++;screen_write_string("7");pin_value[3] = 0;}
		
	//b11110111
		
		PORTB = 0xf7;
		_delay_ms(5);
		
		if (~(pin_value[0]) & (PINB>>PINB4 & 0b1)){pin_value[0] = 1;}
		else if (pin_value[0] & ~(PINB>>PINB4 & 0b1)){input_length++;screen_write_string("A");pin_value[0] = 0;}
		
		if (~(pin_value[1]) & (PINB>>PINB5 & 0b1)){pin_value[1] = 1;}
		else if (pin_value[1] & ~(PINB>>PINB5 & 0b1)){input_length++;screen_write_string("3");pin_value[1] = 0;}
	
		if (~(pin_value[2]) & (PINB>>PINB6 & 0b1)){pin_value[2] = 1;}
		else if (pin_value[2] & ~(PINB>>PINB6 & 0b1)){input_length++;screen_write_string("2");pin_value[2] = 0;}
		
		if (~(pin_value[3]) & (PINB>>PINB7 & 0b1)){pin_value[3] = 1;}
		else if (pin_value[3] & ~(PINB>>PINB7 & 0b1)){input_length++;screen_write_string("1");pin_value[3] = 0;}
		
		
		
		
    }
	
}

