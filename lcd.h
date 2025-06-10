/*
 * AVR ATmega32A Characteristic LCD library 
 *
 * Created: 10/9/2017 2:28:23 AM
 * Author: AVINASH SHARMA
 */ 

#include <avr/io.h>
#include <util/delay.h>

#define LCD_PRT PORTD
#define LCD_DDR DDRD	
#define LCD_PIN PIND	
#define LCD_RS 3
#define LCD_RW 1
#define LCD_EN 2
#define LCD_D7 7

#define DELAY 200
#define Upper_Nibble 0xF0
#define Lower_Nibble 0x0F
#define Data_pin 4

#define SET_E() (LCD_PRT |= (1 << LCD_EN))
#define SET_RS() (LCD_PRT |= (1 << LCD_RS))
#define SET_RW() (LCD_PRT |= (1 << LCD_RW))

#define CLEAR_E() (LCD_PRT &=~ (1 << LCD_EN))
#define CLEAR_RS() (LCD_PRT &=~ (1 << LCD_RS))
#define CLEAR_RW() (LCD_PRT &=~ (1 << LCD_RW))

void lcd_busy()
{
	uint8_t busy,status=0x00,temp;
	
	LCD_DDR&=(~(Upper_Nibble));
	
	SET_RW();		//Read mode
	CLEAR_RS();		//Read status
	
	_delay_us(0.5);		//tAS
	
	do{
		SET_E();

		//Wait tDA for data to become available
		_delay_us(0.5);

		status=(LCD_PIN>>4);
		status=status<<4;

		_delay_us(0.5);

		//Pull E low
		CLEAR_E();
		_delay_us(1);	//tEL

		SET_E();
		_delay_us(0.5);

		temp=(LCD_PIN>>4);
		temp&=0x0F;

		status=status|temp;

		busy=status & 0b10000000;

		_delay_us(0.5);
		CLEAR_E();
		_delay_us(1);	//tEL
	}while(busy);

	CLEAR_RW();		//write mode
	
	LCD_DDR |= 0xFF;
}

void lcdCommand(unsigned char cmnd)
{
	
	CLEAR_RS();
	CLEAR_RW();
	_delay_us(1);
	
	SET_E();
	LCD_PRT = (LCD_PRT & Lower_Nibble) | (cmnd & Upper_Nibble);
	_delay_us(1);
	
	CLEAR_E();
	
	_delay_us(1);
	
	SET_E();
	LCD_PRT = (LCD_PRT & Lower_Nibble) | ((cmnd << Data_pin) & Upper_Nibble);
	_delay_us(1);
	
	CLEAR_E();
	_delay_us(1);
	lcd_busy();
}

void lcdData(unsigned char data) {
	
	
	SET_RS();
	CLEAR_RW();
	_delay_us(1);
	
	SET_E();
	LCD_PRT = (LCD_PRT & Lower_Nibble) | (data & Upper_Nibble);
	_delay_us(1);
	
	CLEAR_E();
	
	_delay_us(1);

	SET_E();
	LCD_PRT = (LCD_PRT & Lower_Nibble) | ((data << Data_pin) & Upper_Nibble);
	_delay_us(1);
	
	CLEAR_E();
	_delay_us(1);
	
	lcd_busy();
}
 




void lcd_init() {
	// Set all pins of LCD port as output
	//while(lcd_busy());
	_delay_ms(100);
	
	LCD_PRT&=(~(0xFF));

	LCD_DDR |= 0xFF;
	
	_delay_us(1);
	
	// Ensure EN pin is initially low
	CLEAR_E();
	_delay_ms(20);  // Wait for power-on initialization (at least 15ms)

	// Send initialization commands to LCD
	lcdCommand(0x33);   // Initialization sequence as per HD44780 datasheet
	_delay_ms(5);       // Wait for >4.1ms

	lcdCommand(0x32);   // Initialization sequence as per HD44780 datasheet
	_delay_ms(5);       // Wait for >100us

	lcdCommand(0x28);   // 4-bit mode, 2 lines, 5x8 font
	_delay_ms(5);       // Wait for >100us

	lcdCommand(0x0C);   // Display ON, Cursor OFF, Blink OFF
	_delay_ms(5);       // Wait for >100us

	lcdCommand(0x01);   // Clear display
	_delay_ms(2);       // Wait for >2ms

	lcdCommand(0x06);   // Entry mode set: Increment cursor, No display shift
	_delay_ms(5);       // Wait for >100us
}


void lcd_clear()
{
	lcdCommand(0x01);
	_delay_ms(20);                             // Time Delay - 20 micro Second
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{
	unsigned char firstCharAdr[] = {0x80,0xC0,0x94,0xD4};
	
	lcdCommand(firstCharAdr[y-1] + x-1);
	_delay_ms(10);                            // Time Delay - 100 micro Second
}

void lcd_print(char *str)
{
	unsigned char i=0;
	while(str[i] !=0)
	{
	  lcdData(str[i]);
	  i++;
	}
}


void lcd_print_num(unsigned val, unsigned length)
{
	char str[3]={0,0,0};
	int i=2,j=0;
	
	while(val)
	{
		str[i]=val%10;
		val=val/10;
		i--;
	}
	
	j=3-length;

	for(i=j;i<3;i++)
	{
		lcdData('0'+str[i]);
	}
}

uint8_t lcd_read_byte() 
{
	uint8_t data,data_h,data_l;

	// Set the LCD data pins as inputs
	LCD_DDR&=(~(Upper_Nibble));

	SET_RW();		//Read mode
	SET_RS();		//Read status

	// Enable LCD to latch data
	SET_E();
	
	_delay_us(1); // Delay to ensure data is latched

	data_h=(LCD_PIN>>4);
	data_h=data_h<<4;

	_delay_us(0.5);

		//Pull E low
	CLEAR_E();
	_delay_us(1);	//tEL

	SET_E();
	_delay_us(0.5);

	data_l=(LCD_PIN>>Data_pin);
	data_l&=Lower_Nibble;

	data=data_h|data_l;

	_delay_us(0.5);
	CLEAR_E();
	_delay_us(1);	//tEL


	CLEAR_RW();		//write mode
	_delay_us(1);
	
	lcd_busy();

	return data;
}

 
