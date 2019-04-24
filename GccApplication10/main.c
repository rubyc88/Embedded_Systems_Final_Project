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
#define address (0x29)
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

void TWIinit();
void Wait_n_Check_Error(uint8_t expected);
void TWIwrite(uint8_t slaveid, uint8_t addr, uint8_t data);
uint8_t TWIread(uint8_t slaveid, uint8_t addr);
void TCS34725init();


int main(void)
{
	DDRD = 0xFF;
	DDRC = 0xFF;
	int i;
	int blue_random;
	int red_random;
	int green_random;
	char snum[5];
	Init();
	Clear();
	Set_Cursor(1,2);
	Write_String("COLOR SENSOR");	
	
	Set_Cursor(2,0);
	Write_String("R:");

	
	Set_Cursor(2,6);
	Write_String("G:");

	
	Set_Cursor(2,12);
	Write_String("B:");

	while(1){
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
		Set_Cursor(1,2);
		Write_String("            ");
		if (red_random < 128 && green_random < 128 && blue_random<128){
					Set_Cursor(1,6);
					Write_String("BLACK");
		}else if (red_random>128 && green_random >128 && blue_random> 128){
					Set_Cursor(1,6);
					Write_String("WHITE");
		}else if (red_random>128 && green_random <128 && blue_random< 128){
					Set_Cursor(1,6);
					Write_String("RED");
		}else if (red_random<128 && green_random >128 && blue_random< 128){
					Set_Cursor(1,6);
					Write_String("GREEN");
		}else if (red_random<128 && green_random <128 && blue_random> 128){
					Set_Cursor(1,6);
					Write_String("BLUE");
		}else if (red_random>128 && green_random >128 && blue_random< 128){
					Set_Cursor(1,6);
					Write_String("YELLOW");
		}else if (red_random<128 && green_random >128 && blue_random> 128){
					Set_Cursor(1,6);
					Write_String("CYAN");
		}else if (red_random>128 && green_random <128 && blue_random> 128){
					Set_Cursor(1,6);
					Write_String("MAGENTA");
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

	}
}
void TWIinit(){
  // Sets SCL to 400kHz
  TWSR = 0x00;
  TWBR = 0x0C;
  // enable TWI
  TWCR = (1<<TWEN); 
}
void TWIStart(void){
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
}
void TWIStop(void){
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void TWIWrite(uint8_t u8data){
	TWDR = u8data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
}
uint8_t TWIReadACK(void){
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

uint8_t TWIReadNACK(void){
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

uint8_t TWIGetStatus(void){
	uint8_t status;
	//mask status
	status = TWSR & 0xF8;
	return status;
}






