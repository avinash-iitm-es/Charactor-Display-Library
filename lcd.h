/*
 * AVR ATmega32A Characteristic LCD library 
 *
 * Created: 10/9/2017 2:28:23 AM
 * Author: AVINASH
 
 * Recreated: 09-03-2019 
 * 
 *
 * Author: AVINASH
 * AK Mechatronics 
 * akmechatronicssytems@gmail.com
 * avinash62096@gmail.com
 */ 

// #define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>


#define LCD_PRT PORTD
#define LCD_DDR DDRD	
#define LCD_PIN PIND	
#define LCD_RS 3
#define LCD_RW 1
#define LCD_EN 2
#define LCD_D7 7

#define SET_E() (LCD_PRT |= (1 << LCD_EN))
#define SET_RS() (LCD_PRT |= (1 << LCD_RS))
#define SET_RW() (LCD_PRT |= (1 << LCD_RW))

#define CLEAR_E() (LCD_PRT &=~ (1 << LCD_EN))
#define CLEAR_RS() (LCD_PRT &=~ (1 << LCD_RS))
#define CLEAR_RW() (LCD_PRT &=~ (1 << LCD_RW))

#define DELAY 200
// #define DELAY-2

void lcd_busy()
{
	uint8_t busy,status=0x00,temp;
	
	// 	LCD_DDR&=(~(0XF0));
	LCD_DDR&=(~(0xF0));
	
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
	LCD_PRT = (LCD_PRT & 0x0F) | (cmnd & 0xF0);
	_delay_us(1);
	
	CLEAR_E();
	
	_delay_us(1);
	
	SET_E();
	LCD_PRT = (LCD_PRT & 0x0F) | ((cmnd << 4) & 0xF0);
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
	LCD_PRT = (LCD_PRT & 0x0F) | (data & 0xF0);
	_delay_us(1);
	
	CLEAR_E();
	
	_delay_us(1);

	SET_E();
	LCD_PRT = (LCD_PRT & 0x0F) | ((data << 4) & 0xF0);
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
	
/***************************************************************
	This function writes a integer type value to LCD module

	Arguments:
	1)int val	: Value to print

	2)unsigned int field_length :total length of field in which the value is printed
	must be between 1-5 if it is -1 the field length is no of digits in the val

	****************************************************************/
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
	LCD_DDR&=(~(0xF0));

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

	data_l=(LCD_PIN>>4);
	data_l&=0x0F;

	data=data_h|data_l;

	_delay_us(0.5);
	CLEAR_E();
	_delay_us(1);	//tEL


	CLEAR_RW();		//write mode
	_delay_us(1);
	
	lcd_busy();

	return data;
}

/*

  void lcdCommand(unsigned char cmnd) {
	  // Send higher nibble of command
	  //while(lcd_busy());
	  
	  LCD_PRT = (LCD_PRT & 0x0F) | (cmnd & 0xF0);
	  
	  // 	LCD_PRT &=~ (1 << LCD_RS);  // RS low for command mode
	  CLEAR_RS();
	  // 	LCD_PRT &=~ (1 << LCD_RW);  // RW low for write mode
	  CLEAR_RW();
	  // 	LCD_PRT |= (1 << LCD_EN);   // Enable LCD
	  SET_E();
	  _delay_us(5);               // Minimum delay of 450ns
	  // 	LCD_PRT &=~ (1 << LCD_EN);  // Disable LCD
	  CLEAR_E();
	  _delay_us(DELAY);             // Delay for command execution 100uS

	  // Send lower nibble of command
	  LCD_PRT = (LCD_PRT & 0x0F) | ((cmnd << 4) & 0xF0);
	  
	  // 	LCD_PRT |= (1 << LCD_EN);   // Enable LCD
	  SET_E();
	  _delay_us(5);               // Minimum delay of 450ns
	  // 	LCD_PRT &=~ (1 << LCD_EN);  // Disable LCD
	  CLEAR_E();
	  _delay_us(DELAY);             // Delay for command execution
  }

  void lcdData(unsigned char data) {
	  // Send higher nibble of data
	  //while(lcd_busy());
	  
	  LCD_PRT = (LCD_PRT & 0x0F) | (data & 0xF0);
	  
	  // 	LCD_PRT |= (1 << LCD_RS);   // RS high for data mode
	  SET_RS();
	  // 	LCD_PRT &=~ (1 << LCD_RW);  // RW low for write mode
	  CLEAR_RW();
	  // 	LCD_PRT |= (1 << LCD_EN);   // Enable LCD
	  SET_E();
	  _delay_us(5);               // Minimum delay of 450ns
	  // 	LCD_PRT &=~ (1 << LCD_EN);  // Disable LCD
	  CLEAR_E();
	  _delay_us(DELAY);             // Delay for data write

	  // Send lower nibble of data
	  LCD_PRT = (LCD_PRT & 0x0F) | ((data << 4) & 0xF0);
	  
	  // 	LCD_PRT |= (1 << LCD_EN);   // Enable LCD
	  SET_E();
	  _delay_us(5);               // Minimum delay of 450ns
	  // 	LCD_PRT &=~ (1 << LCD_EN);  // Disable LCD
	  CLEAR_E();
	  _delay_us(DELAY);             // Delay for data write
  }
  
void lcdCommand(unsigned char cmnd)
{
	LCD_PRT = (LCD_PRT & 0x0F) | (cmnd & 0xF0);
    LCD_PRT &=~ (1<<LCD_RS);
	LCD_PRT &=~ (1<<LCD_RW);
	LCD_PRT |= (1<<LCD_EN);
	_delay_us(800);                                 // Time Delay - 25 micro Second
	LCD_PRT &=~ (1<<LCD_EN);
	_delay_us(240);                                 // Time Delay - 25 micro Second
	
	LCD_PRT = (LCD_PRT & 0x0F) | (cmnd << 4);
	LCD_PRT |= (1<<LCD_EN);
	_delay_us(240);                                 // Time Delay - 25 micro Second
	LCD_PRT &=~ (1<<LCD_EN);
}

void lcdData(unsigned char data)
{
	LCD_PRT = (LCD_PRT & 0x0F) | (data & 0xF0);
	LCD_PRT |= (1<<LCD_RS);
	LCD_PRT &=~ (1<<LCD_RW);
	LCD_PRT |= (1<<LCD_EN);
	_delay_us(800);                                // Time Delay - 25 micro Second
	LCD_PRT &=~ (1<<LCD_EN);
	LCD_PRT = (LCD_PRT & 0x0F) | (data <<4);
	LCD_PRT |= (1<<LCD_EN);
	_delay_us(240);                                // Time Delay - 25 micro Second
	LCD_PRT &=~ (1<<LCD_EN);
}

void lcd_init()
{
	LCD_DDR = 0xFF;
	
	LCD_PRT &=~ (1<<LCD_EN);
	_delay_ms(20);                            // Time Delay -  2 milli Second
	
	lcdCommand(0x33); _delay_ms(10);          // Time Delay - 100 micro Second
	lcdCommand(0x32); _delay_ms(10);          // Time Delay - 100 micro Second
	lcdCommand(0x28); _delay_ms(10);          // Time Delay - 100 micro Second
	lcdCommand(0x0c); _delay_ms(10);          // Time Delay - 100 micro Second
	lcdCommand(0x01); _delay_ms(20);         // Time Delay - 2 milli Second
	lcdCommand(0x06); _delay_ms(10);	       // Time Delay - 100 micro Second
}

void LCDByte(uint8_t c, uint8_t isdata)
{
	
	uint8_t hn,ln;			//Nibbles
	uint8_t temp;
	
	hn=c>>4;
	ln=(c & 0x0F);
	
	if(isdata==0)
	CLEAR_RS();
	else
	SET_RS();
	
	_delay_us(1);
	
	LCD_PRT = (LCD_PRT & 0x0F) | (c & 0xF0);

	CLEAR_RW();
	SET_E();
	_delay_us(5);
	
	CLEAR_E();
	_delay_us(DELAY);
	
	// Send lower nibble of command
	LCD_PRT = (LCD_PRT & 0x0F) | ((c << 4) & 0xF0);
	SET_E();
	_delay_us(5);
	CLEAR_E();
	_delay_us(DELAY);
}

void lcd_busy()
{
	uint8_t busy,status=0x00,temp;
	
	
	
	LCD_DDR = 0x00;         // Set data direction of LCD port as input
	// 	LCD_PRT &=~ (1 << LCD_RS);  // RS low for command mode
	CLEAR_RS();
	LCD_PRT |= (1 << LCD_RW);   // RW high for read mode
	// 	LCD_PRT &=~ (1 << LCD_EN);  // Enable LCD
	CLEAR_E();
	_delay_us(5);               // Minimum delay of 450ns

	// 	LCD_PRT |= (1 << LCD_EN);   // Enable pulse
	SET_E();
	_delay_us(5);               // Minimum delay of 450ns
	lcd_status = (LCD_PIN & (1 << LCD_D7)); // Read busy flag
	// 	LCD_PRT &=~ (1 << LCD_EN);  // Disable LCD
	CLEAR_E();
	_delay_us(5);               // Minimum delay of 450ns

	LCD_DDR = 0xFF;          // Restore data direction of LCD port as output
}

*/