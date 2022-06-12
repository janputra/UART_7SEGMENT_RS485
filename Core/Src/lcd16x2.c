#include "lcd16x2.h"
//extern unsigned char f_timer_100us =0;

void send_to_lcd(char data)
{

	//writing data to pin PE0~PE3
	GPIOE->ODR =  (GPIOE->ODR & 0xFFFFFFF0) | data;

/*
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ((data>>3)&0x01));
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ((data>>2)&0x01));
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ((data>>1)&0x01));
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ((data>>0)&0x01));
*/

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);
	HAL_Delay(1);
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 0);
    HAL_Delay(1);
}



void lcd_cmd(char cmd)
{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, 0);

	send_to_lcd((cmd>>4)&0x0f);
	send_to_lcd(cmd&0x0f);

}

void lcd_data(char data)
{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, 1);

	send_to_lcd((data>>4)&0x0f);
	send_to_lcd(data&0x0f);
}

void lcd_write_string(char *s)
{
	for(int i=0;s[i]!=0;i++) lcd_data(s[i]);

}

void lcd_init(void)
{
    // 4 bit initialization
  HAL_Delay(50);  // wait for >40ms
 /*lcd_cmd (0x30);
  HAL_Delay(5);  // wait for >4.1ms
  lcd_cmd (0x30);
  HAL_Delay(1);  // wait for >100us
  lcd_cmd (0x30);
  HAL_Delay(1);
  lcd_cmd (0x20);  // 4bit mode
  HAL_Delay(1);
*/
// dislay initialization
  lcd_cmd (0x2);
  lcd_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
  HAL_Delay(1);
  lcd_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
  HAL_Delay(1);
  lcd_cmd (0x01);  // clear display
  HAL_Delay(1);
  lcd_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
  HAL_Delay(1);
  lcd_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bitss)
}

void lcd_set_pos(int row, int col)
{

	if (row==0)
	{
		col |= 0x80;
	}
	else
	{
		col |= 0xC0;
	}

	lcd_cmd(col);

}

void lcd_clear(void)
{
	 lcd_cmd(0x01);
}
