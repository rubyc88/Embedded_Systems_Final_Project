#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz clock speed
#endif
#define D4 LCD_PORTD4
#define D5 LCD_PORTD5
#define D6 LCD_PORTD6
#define D7 LCD_PORTD7
#define RS LCD_PORTC6
#define EN LCD_PORTC7
#define SCL LCD_PORTC0
#define SDA LCD_PORTC1

#define TCS34725_Read_Mode 0xD1u
#define TCS34725_Write_Mode 0xD0u
#define sensor 0x29
#define bus_id (0x00)
#define readbit (0x01)

// Registers for TCS342725
#define ENABLE                  (0x00) // Enables states and interrupts
#define ATIME                   (0x01) // RGBC time
#define WTIME    				(0x03) // Wait time
#define AILTL					(0x04) // Clear interrupt low threshold low byte
#define AILTH					(0x05) // Clear interrupt low threshold high byte
#define AIHTL					(0x06) // Clear interrupt high threshold low byte
#define AIHTH					(0x07) // Clear interrupt high threshold high byte
#define PERS					(0x0C) // Interrupt persistence filter
#define CONFIG					(0x0D) // Configuration
#define CONTROL                 (0x0F) // Control
#define ID                      (0x12) // Device ID
#define STATUS					(0x13) // Device Status
#define CDATAL		            (0x14) // Clear data low byte
#define CDATAH                  (0x15) // Clear data high byte
#define RDATAL		            (0x16) // Red data low byte
#define RDATAH                  (0x17) // Red data high byte
#define GDATAL                  (0x18) // Green data low byte
#define GDATAH                  (0x19) // Green data high byte
#define BDATAL                  (0x1A) // Blue data low byte
#define BDATAH                  (0x1B) // Blue data high byte

//Configuration Bits
#define ENABLE_AEN              (0x02)  /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
#define ENABLE_PON              (0x01)  /* Power on - Writing 1 activates the internal oscillator, 0 disables it */
#define INTEGRATIONTIME_2_4MS    0xFF   /**<  2.4ms - 1 cycle    - Max Count: 1024  */
#define INTEGRATIONTIME_24MS     0xF6   /**<  24ms  - 10 cycles  - Max Count: 10240 */
#define INTEGRATIONTIME_101MS    0xD5   /**<  101ms - 42 cycles  - Max Count: 43008 */
#define INTEGRATIONTIME_154MS    0xC0   /**<  154ms - 64 cycles  - Max Count: 65535 */
#define INTEGRATIONTIME_700MS    0x00   /**<  700ms - 256 cycles - Max Count: 65535 */



// Gains
#define TCS34725_GAIN_1X                  0x00   /**<  No gain  */
#define TCS34725_GAIN_4X                  0x01   /**<  4x gain  */
#define TCS34725_GAIN_16X                 0x02   /**<  16x gain */
#define TCS34725_GAIN_60X                 0x03   /**<  60x gain */



#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h" 
#include "stdio.h"
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include <util/delay.h>
#include <util/twi.h>

#define BUADRATE 51

void TWIinit();
void Wait_n_Check_Error(uint8_t expected);
void    TWIwrite(uint8_t slaveid, uint8_t addr, uint8_t data);
uint8_t TWIread(uint8_t slaveid, uint8_t addr);
void light_sensor_init();
int16_t red_sensor_read();
int16_t green_sensor_read();
int16_t blue_sensor_read();

//// Global Buffers and Vars ////
char strbuf[100];
uint8_t err;
uint8_t TRACE=0;

int main(void)
{
	DDRD = 0xFF;
	DDRC = 0xFF;
//	int i;
//	int blue_random;
//	int red_random;
//	int green_random;
	int16_t  red;
	int16_t green;
	int16_t blue;
	char snum[16];

	Init();
	Clear();
	//Set_Cursor(1,2);
	//Write_String("COLOR SENSOR");	
	
	TWIinit();
	light_sensor_init();
	
	Set_Cursor(2,0);
	Write_String("R:");

	
	Set_Cursor(2,6);
	Write_String("G:");

	
	Set_Cursor(2,12);
	Write_String("B:");
	
	
	uint8_t num;
	

	while(1){
		red = red_sensor_read();
		Set_Cursor(2,0);
		Write_String("R");

		/*
		red= red_sensor_read();
		green = green_sensor_read();
		blue = blue_sensor_read();
		Write_String(itoa(red,snum,10));
		Write_String(itoa(green,snum,10));
		Write_String(itoa(blue,snum,10));
		*/
		//Write_String(red);
		//Write_String(blue);
		//Write_String(green);
		/*
		// Red Value
		red_random = rand() % 255;
		Set_Cursor(2,2);
		if(red_random <100){
			Write_String("0");
			Set_Cursor(2,3);
		}
		Write_String(itoa(red_random,snum,10));
		
		// Green Value
		green_random = rand() % 255;
		Set_Cursor(2,8);
		if(green_random <100){
			Write_String("0");
			Set_Cursor(2,9);
		}
		Write_String(itoa(green_random,snum,10));
		
		// Blue Value
		blue_random = rand() % 255;
		Set_Cursor(2,14);
		
		if (blue_random < 100 ){
			Write_String("0");
			Set_Cursor(2,15);			
		}
		Write_String(itoa(blue_random,snum,10));
		//Set_Cursor(1,2);
		//Write_String("            ");
		if (red_random < 128 && green_random < 128 && blue_random<128){
					//Set_Cursor(1,6);
					//Write_String("BLACK");
		}else if (red_random>128 && green_random >128 && blue_random> 128){
					//Set_Cursor(1,6);
					//Write_String("WHITE");
		}else if (red_random>128 && green_random <128 && blue_random< 128){
					//Set_Cursor(1,6);
					//Write_String("RED");
		}else if (red_random<128 && green_random >128 && blue_random< 128){
					//Set_Cursor(1,6);
					//Write_String("GREEN");
		}else if (red_random<128 && green_random <128 && blue_random> 128){
					//Set_Cursor(1,6);
					//Write_String("BLUE");
		}else if (red_random>128 && green_random >128 && blue_random< 128){
					//Set_Cursor(1,6);
					//Write_String("YELLOW");
		}else if (red_random<128 && green_random >128 && blue_random> 128){
					//Set_Cursor(1,6);
					//Write_String("CYAN");
		}else if (red_random>128 && green_random <128 && blue_random> 128){
					//Set_Cursor(1,6);
					//Write_String("MAGENTA");
		}	
		
		
		for(i=0;i<1;i++){
			_delay_ms(50);
			Shift_Left();
		}
		_delay_ms(100);
		for(i=0;i<1;i++){
			_delay_ms(10);
			Shift_Right();
		}
		_delay_ms(100);
	*/
	}
	
}
void TWIinit(){
	DDRC |= (1 << PORTC0) | (1 << PORTC1);    // SDA and SCL are set to be outputs	$$$$$$$$
	TWSR = (0b00 << TWPS0);   // TWPS=00 --> prescale =0 --> div by 1				$$$$$$$$
	TWBR = 8;   // SCLfreq = clkCPU/(16+2*(TWBR).(TWPS))							$$$$$$$$
	// SCLfreq = 8000_000/(16+2*(12).(1)) = 200KHz    if use TWBR=12
	// SCLfreq = 8000_000/(16+2*(8).(1))  = 250KHz    if use TWBR=8
	// SCLfreq = 8000_000/(16+2*(2).(1))  = 400KHz    if use TWBR=2 (fastest)
}

void Wait_n_Check_Error(uint8_t expected){
	while (!(TWCR & (1 << TWINT)));   // Wait for TWINT to set										$$$$$$$$
	
	if ((TWSR & 0xF8) != expected){
		switch(expected){
			case TW_START		         : sprintf(strbuf, "Expecting TW_START		           but got 0x%x\r\n", TWSR);   break;
			case TW_REP_START		     : sprintf(strbuf, "Expecting TW_REP_START		       but got 0x%x\r\n", TWSR);   break;
			case TW_MT_SLA_ACK		     : sprintf(strbuf, "Expecting TW_MT_SLA_ACK		       but got 0x%x\r\n", TWSR);   break;
			case TW_MT_SLA_NACK		     : sprintf(strbuf, "Expecting TW_MT_SLA_NACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_MT_DATA_ACK		     : sprintf(strbuf, "Expecting TW_MT_DATA_ACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_MT_DATA_NACK		 : sprintf(strbuf, "Expecting TW_MT_DATA_NACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_MT_ARB_LOST		     : sprintf(strbuf, "Expecting TW_MT/R_ARB_LOST		   but got 0x%x\r\n", TWSR);   break;
			case TW_MR_SLA_ACK		     : sprintf(strbuf, "Expecting TW_MR_SLA_ACK		       but got 0x%x\r\n", TWSR);   break;
			case TW_MR_SLA_NACK		     : sprintf(strbuf, "Expecting TW_MR_SLA_NACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_MR_DATA_ACK		     : sprintf(strbuf, "Expecting TW_MR_DATA_ACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_MR_DATA_NACK		 : sprintf(strbuf, "Expecting TW_MR_DATA_NACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_ST_SLA_ACK		     : sprintf(strbuf, "Expecting TW_ST_SLA_ACK		       but got 0x%x\r\n", TWSR);   break;
			case TW_ST_ARB_LOST_SLA_ACK	 : sprintf(strbuf, "Expecting TW_ST_ARB_LOST_SLA_ACK   but got 0x%x\r\n", TWSR);   break;
			case TW_ST_DATA_ACK		     : sprintf(strbuf, "Expecting TW_ST_DATA_ACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_ST_DATA_NACK		 : sprintf(strbuf, "Expecting TW_ST_DATA_NACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_ST_LAST_DATA		 : sprintf(strbuf, "Expecting TW_ST_LAST_DATA		   but got 0x%x\r\n", TWSR);   break;
			case TW_SR_SLA_ACK		     : sprintf(strbuf, "Expecting TW_SR_SLA_ACK		       but got 0x%x\r\n", TWSR);   break;
			case TW_SR_ARB_LOST_SLA_ACK	 : sprintf(strbuf, "Expecting TW_SR_ARB_LOST_SLA_ACK   but got 0x%x\r\n", TWSR);   break;
			case TW_SR_GCALL_ACK		 : sprintf(strbuf, "Expecting TW_SR_GCALL_ACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_SR_ARB_LOST_GCALL_ACK: sprintf(strbuf, "Expecting TW_SR_ARB_LOST_GCALL_ACK but got 0x%x\r\n", TWSR);   break;
			case TW_SR_DATA_ACK		     : sprintf(strbuf, "Expecting TW_SR_DATA_ACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_SR_DATA_NACK		 : sprintf(strbuf, "Expecting TW_SR_DATA_NACK		   but got 0x%x\r\n", TWSR);   break;
			case TW_SR_GCALL_DATA_ACK	 : sprintf(strbuf, "Expecting TW_SR_GCALL_DATA_ACK	   but got 0x%x\r\n", TWSR);   break;
			case TW_SR_GCALL_DATA_NACK	 : sprintf(strbuf, "Expecting TW_SR_GCALL_DATA_NACK	   but got 0x%x\r\n", TWSR);   break;
			case TW_SR_STOP		         : sprintf(strbuf, "Expecting TW_SR_STOP		       but got 0x%x\r\n", TWSR);   break;
			default                      : sprintf(strbuf, "Expecting 0x%x but got 0x%x\r\n", expected,           TWSR);   break;
		}
	}
}

void TWIwrite(uint8_t slaveid, uint8_t addr, uint8_t data){
	
	// Send START condition, ACK is not expected
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWSTA);							// $$$$$$$$
	Wait_n_Check_Error(TW_START);

	// Setup the slave ID (7bits) along with the intended operation (1bit)
	TWDR = slaveid|TW_WRITE;										// $$$$$$$$
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWEA);  // TWINT bit in TWCR to start transmission of address,
	Wait_n_Check_Error(TW_MT_SLA_ACK); // expecting ACK				// $$$$$$$$

	// Send the 1st byte which will be interpreted by the device as the register address,
	TWDR = addr;
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWEA);  // expecting ACK		// $$$$$$$$
	Wait_n_Check_Error(TW_MT_DATA_ACK);								// $$$$$$$$
	
	// Send the 2nd byte which will be interpreted by the device as the register data,
	TWDR = data;
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWEA);  // expecting ACK		// $$$$$$$$
	Wait_n_Check_Error(TW_MT_DATA_ACK);								// $$$$$$$$

	// Send STOP condition, Nothing expected. Note that TWINT isn't set after STOP
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWSTO);							// $$$$$$$$
	
	// Wait for STOP to be executed. TWINT is not set after a stop condition!
	while(TWCR & _BV(TWSTO));
}
uint8_t TWIread(uint8_t slaveid, uint8_t addr){
	// Send START condition, ACK is not expected
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWSTA);		
	// Setup the slave ID (7bits) along with the intended operation (1bit)
	TWDR = slaveid|TW_WRITE;
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWEA);  // TWINT bit in TWCR to start transmission of address,
	// Send the 1st byte which will be interpreted by the device as the register address,
	TWDR = addr;
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWEA);// expecting ACK
	// Send Repeat START
	TWCR = _BV(TWINT)|_BV(TWSTA)|_BV(TWEN);
	// Tell the device that you need to read the data for the address sent before
	TWDR = slaveid|TW_READ;                  // Setup the slave ID + READ						// $$$$$$$$
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWEA);    // TWINT bit in TWCR to start transmission of address.
	// Asking the slave to send the data byte,
	TWCR = _BV(TWINT)|_BV(TWEN);            // TWINT bit in TWCR to start transmission of data. // $$$$$$$$
	uint8_t data = TWDR;	                       // Read the received data
	// Send STOP condition, Nothing expected. Note that TWINT isn't set after STOP
	TWCR = _BV(TWINT)|_BV(TWEN)|_BV(TWSTO);
	// Wait for STOP to be executed. TWINT is not set after a stop condition!
	while(TWCR & _BV(TWSTO));
	return data;
	}

void light_sensor_init(){
	TWIwrite(sensor , 0x80, 0b11);
	Set_Cursor(1,2);
	Write_String("I Enabled");
	
}

int16_t red_sensor_read(){
	int16_t res;
	res  = TWIread(sensor, 0x96)<<8;		// $$$$$$$$
	res |= TWIread(sensor, 0x97);		// $$$$$$$$
	return res;
	
}

int16_t green_sensor_read(){
	int16_t res;
	res  = TWIread(ID, GDATAH)<<8;		// $$$$$$$$
	res |= TWIread(ID, GDATAL);		// $$$$$$$$
	return res;
	
}

int16_t blue_sensor_read(){
	int16_t res;
	res  = TWIread(ID, BDATAH)<<8;		// $$$$$$$$
	res |= TWIread(ID, BDATAL);		// $$$$$$$$
	return res;
	
}




