#define LCD_PORTC6 26
#define LCD_PORTC7 27
#define LCD_PORTD4 34
#define LCD_PORTD5 35
#define LCD_PORTD6 36
#define LCD_PORTD7 37

#ifndef D0
#define D0 LCD_PORTA0
#define D1 LCD_PORTA1
#define D2 LCD_PORTA2
#define D3 LCD_PORTA3
#endif

#include<util/delay.h>

void pinChange(int a, int b)
{
	if(b == 0)
	{
		if(a == LCD_PORTC6)
		PORTC &= ~(1<<PC6);
		else if(a == LCD_PORTC7)
		PORTC &= ~(1<<PC7);
		else if(a == LCD_PORTD4)
		PORTD &= ~(1<<PD4);
		else if(a == LCD_PORTD5)
		PORTD &= ~(1<<PD5);
		else if(a == LCD_PORTD6)
		PORTD &= ~(1<<PD6);
		else if(a == LCD_PORTD7)
		PORTD &= ~(1<<PD7);
	}
	else
	{
		if(a == LCD_PORTC6)
		PORTC |= (1<<PC6);
		else if(a == LCD_PORTC7)
		PORTC |= (1<<PC7);
		else if(a == LCD_PORTD4)
		PORTD |= (1<<PD4);
		else if(a == LCD_PORTD5)
		PORTD |= (1<<PD5);
		else if(a == LCD_PORTD6)
		PORTD |= (1<<PD6);
		else if(a == LCD_PORTD7)
		PORTD |= (1<<PD7);
	}
}

void Port(char a)
{
	if(a & 1)
	pinChange(D4,1);
	else
	pinChange(D4,0);
	
	if(a & 2)
	pinChange(D5,1);
	else
	pinChange(D5,0);
	
	if(a & 4)
	pinChange(D6,1);
	else
	pinChange(D6,0);
	
	if(a & 8)
	pinChange(D7,1);
	else
	pinChange(D7,0);
}
void Cmd(char a)
{
	pinChange(RS,0);             // => RS = 0
	Port(a);
	pinChange(EN,1);            // => E = 1
	_delay_ms(1);
	pinChange(EN,0);             // => E = 0
	_delay_ms(1);
}

void Clear(){
	Cmd(0);
	Cmd(1);
}

void Set_Cursor(char a, char b){
	char temp,z,y;
	if(a == 1)
	{
		temp = 0x80 + b;
		z = temp>>4;
		y = (0x80+b) & 0x0F;
		Cmd(z);
		Cmd(y);
	}
	else if(a == 2)
	{
		temp = 0xC0 + b;
		z = temp>>4;
		y = (0xC0+b) & 0x0F;
		Cmd(z);
		Cmd(y);
	}
}

void Init()
{
	Port(0x00);
	_delay_ms(20);
	Cmd(0x03);
	_delay_ms(5);
	Cmd(0x03);
	_delay_ms(11);
	Cmd(0x03);
	Cmd(0x02);
	Cmd(0x02);
	Cmd(0x08);
	Cmd(0x00);
	Cmd(0x0C);
	Cmd(0x00);
	Cmd(0x06);
}

void Write_Char(char a)
{
	char temp,y;
	temp = a&0x0F;
	y = a&0xF0;
	pinChange(RS,1);             // => RS = 1
	Port(y>>4);             //Data transfer
	pinChange(EN,1);
	_delay_ms(1);
	pinChange(EN,0);
	_delay_ms(1);
	Port(temp);
	pinChange(EN,1);
	_delay_ms(1);
	pinChange(EN,0);
	_delay_ms(1);
}

void Write_String(char *a){
	int i;
	for(i=0;a[i]!='\0';i++)
	Write_Char(a[i]);
}

void Shift_Right(){
	Cmd(0x01);
	Cmd(0x0C);
}
void Shift_Left(){
	Cmd(0x01);
	Cmd(0x08);
}

