/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd16x2.h"
#include "buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define RAND __TIME__
uint8_t f_lcd_item =0;
uint8_t f_TX_ID=0;
uint8_t f_busy;
uint8_t f_querry=0;
uint8_t f_read_msg;
uint8_t tx_timeout = 0;
unsigned char f_timer_TX = 0;
unsigned char f_seg_timer_500ms;
unsigned char f_timer_10ms = 0;
unsigned char f_timer_update_digit = 0;
unsigned char f_waiting_rx = 0;
unsigned char f_timer_30ms = 0;
unsigned char f_timer_50ms = 0;
unsigned char n_querry = 0;
unsigned char flag_digit_1 = 1;

unsigned char tx2_buffer[10] = {"123456789"};
unsigned char tx1_buffer[6] = {"abcdef"};
unsigned char d_timer_30ms;
unsigned char d_timer_50ms;
unsigned char d_timer_update_digit;
unsigned char d_timer_TX1;
unsigned char TX1_delay_val = 250;
unsigned char d_timer_TX2;
unsigned char TX2_delay_val = 250;
unsigned char key1_data, key2_data;
unsigned char state, old_state, event, error;
unsigned char num_slave=0;
uint8_t key1_ID,key2_ID;

unsigned char digit1, digit2;
uint8_t old_num_slave, old_digit1,old_digit2;
// flag for LCD
unsigned char is_EN = 0;

unsigned char digit1_update = 0;
unsigned char digit2_update = 0;
unsigned char cmd, data;
unsigned char lcd_process = 4;

char digit_table[17] = {"0123456789abcdef-"};

uint8_t rx_temp;
uint8_t x=1;
circular_buffer digit1_buf;
circular_buffer digit2_buf;
circular_buffer rx_buffer;
circular_buffer event_buffer;
uint8_t ID;
uint8_t ID_list[10];
uint8_t TX_msg[6];
uint8_t RX_msg[4];
uint8_t *pRX_msg;
uint8_t *pTX_msg;
uint8_t temp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RS485_RX_Task(void);
void RS485_TX_Task(void);
void task_timer(void);
void counting_task(void);
void uart_TX2_task(void);
void uart_TX1_task(void);
void uart_RX1_task(void);
void uart_RX2_task(void);
void segment_display_task(void);

void key_read_task(void);
void main_task(void);

void seven_segment_driver(char input, char select_digit);
void TX1_delay_update(void);
void TX2_delay_update(void);
void m_send_to_lcd(char data);
void Set_Transmitter_Port1(void);
void Set_Transmitter_Port2(void);
void Set_Receiver_Port1(void);
void Set_Receiver_Port2(void);

void RS485_Send_Message(uint8_t addr, uint8_t func_code, uint8_t data);
void RS485_Read_Message(void);
void check_slave(void);
void update_lcd(uint8_t* itm);

void display_state_slave(uint8_t slave, uint8_t state);
/*
unsigned char m_send_to_lcd(char data);
unsigned char m_lcd_cmd(char cmd);
unsigned char m_lcd_data(char cmd);
unsigned char m_lcd_set_pos(int row, int col);
*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM4_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim4);

	HAL_GPIO_WritePin(TX1_EN_GPIO_Port, TX1_EN_Pin, 0); // Enable Receiver 1
	// HAL_GPIO_WritePin(TX2_EN_GPIO_Port, TX2_EN_Pin, 0);	// Enable Receiver 2

	HAL_UART_Receive_IT(&huart4, &rx_temp, 1);
	// HAL_UART_Receive_IT(&huart5, &rx2_buffer.data[rx2_buffer.head], 1);

	digit1 = 16;
	digit2 = 16;
	num_slave=0;
	ID=0;
	ID_list[0]= 0xFF;
	ID_list[SLAVE1]= 0;
	ID_list[SLAVE2]= 0;
	//ID_list[2]= 0x12;
	state = STATE_IDLE;
	/*
	if (num_slave>0){
		state = STATE_OPERATION;
	}else{
		state=STATE_ASSIGNED_ADDR;
	}*/
	HAL_Delay(1000);
	lcd_init();
	lcd_clear();
	lcd_set_pos(0, 0);
	lcd_write_string("NUM SLAVE: ");
	lcd_set_pos(0, 0xC);
	lcd_data(digit_table[num_slave]);
	lcd_set_pos(1, 0);
	lcd_write_string("S2-00:");
	lcd_set_pos(1, 0x6);
	lcd_data(digit_table[digit2]);
	lcd_set_pos(1, 0x9);
	lcd_write_string("S1-00:");
	lcd_set_pos(1, 0xF);
	lcd_data(digit_table[digit1]);
	old_digit1 = digit1;
	old_digit2=old_digit2;
	old_num_slave= num_slave;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		task_timer();
		/// segment_display_task();
		
		key_read_task();
		RS485_RX_Task();
		main_task();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */


void update_lcd(uint8_t* itm){

	//if (lcd_process<4)return;
	if (itm == &digit2){
		lcd_set_pos(1,0x6);
		lcd_data(digit_table[digit2]);
		
	}else if(itm ==&digit1){
		lcd_set_pos(1,0xF);
		lcd_data(digit_table[digit1]);
	}else if(itm ==&num_slave){
		lcd_set_pos(0,0xC);
		lcd_data(digit_table[num_slave]);
		
	}

}


void task_timer(void)
{
	if (!f_timer_10ms)
		return;		  // checking if 10 ms timer interrupt is set (10 ms), if set then do timer task
	f_timer_10ms = 0; // clear the flag to wait next interupt

	d_timer_30ms++; // count timer for 30 ms interval

	if (d_timer_30ms == 3) // checking if the count reached 30 ms
	{
		d_timer_30ms = 0; // assign "0" to repeat counting
		f_timer_30ms = 1; // Set flag to inform 30 ms timer is done counting
	}

	d_timer_update_digit++;
	if (d_timer_update_digit > 125)
	{

		d_timer_update_digit = 0;
		f_timer_update_digit = 1;
	}
	d_timer_50ms++;			// for querry 
	if (d_timer_50ms == 5)
	{

		d_timer_50ms = 0;
		f_querry=1;
	}

}

void key_read_task(void)
{
	if (!f_timer_30ms) return;		  // Checking if 30 ms counting is done
	f_timer_30ms = 0; // clear the flag to wait next counting

	uint8_t key_pindata = (uint8_t)(GPIOG->IDR & (KEY1_Pin | KEY2_Pin));

	key1_data = key1_data << 1; // Preparing to read KEY1 Input
	key1_data &= 0b00001110;
	key1_data |= ((key_pindata >> 2) & 0x1); // Read KEY1 Input

	key2_data = key2_data << 1; // Preparing to read KEY2 Input
	key2_data &= 0b00001110;
	key2_data |= (key_pindata >> 3); // Read KEY2 Input

	if (key1_data == KEY_PRESSED) // Checking if KEY1 is pressed
	{
		f_querry =0;
		state = STATE_IDLE;
		buffer_push(&event_buffer, EVENT_KEY1_PRESSED); // Store the event in buffer
	}else if(key1_data == KEY_RELEASED){
		buffer_push(&event_buffer, EVENT_KEY1_RELEASED); 
	}

	if (key2_data == KEY_PRESSED) // Checking if KEY2 is pressed
	{
		
		f_querry =0;
		state = STATE_IDLE;
		buffer_push(&event_buffer, EVENT_KEY2_PRESSED); // Store the event in buffer
	}else if(key2_data == KEY_RELEASED){
		buffer_push(&event_buffer, EVENT_KEY2_RELEASED);
	}

}

void main_task(void)
{

	if (event_buffer.head != event_buffer.tail)
	{
		
		event = buffer_pop(&event_buffer); // if there is event then get the event from buffer

	}

	switch (state)
	{
	case STATE_IDLE:

		switch (event)
		{
		case EVENT_KEY1_PRESSED:

			f_TX_ID = SLAVE1;
			f_timer_update_digit=1;
			//d_timer_update_digit = 250;			
			event = EVENT_RESET;
			state = STATE_TX;
			
			break;

		case EVENT_KEY2_PRESSED:
			
			f_TX_ID = SLAVE2;
			f_timer_update_digit=1;
			//d_timer_update_digit = 250;	
			event = EVENT_RESET;
			state = STATE_TX;
			
			break;
				
		default:
			if (!f_querry) break;
			f_querry =0;
					
			if(ID>0){
				
				RS485_Send_Message(ID_list[ID],FUNC_READ, '0');
			}else{
				RS485_Send_Message(ID_list[ID],FUNC_FIND_SLAVE,'0');
			}
						
			ID = (ID+1)== 3 ?0:ID+1;
			
			state = STATE_WAITING_RX;
			old_state = STATE_IDLE;
			break;

		}
		break;

	case STATE_TX:
		switch (event)
		{
		case EVENT_KEY1_RELEASED:

			f_timer_update_digit =1;
			state = STATE_IDLE;

			break;

		case EVENT_KEY2_RELEASED:

			f_timer_update_digit =1;
			state = STATE_IDLE;	

			break;

		default:
			/// timer tx by key
			if (!f_timer_update_digit) break;
			f_timer_update_digit =0;

			if(f_TX_ID == SLAVE1){
				digit1=(digit1+1)>9? 0 :digit1+1;
				update_lcd(&digit1);
				old_digit1 = digit1;
				//buffer_push(&digit1_buf,digit1);
				RS485_Send_Message(ID_list[SLAVE1],FUNC_WRITE,(digit1+'0'));
			}
			else if (f_TX_ID == SLAVE2)
			{
				digit2=(digit2+1)>9? 0 :digit2+1;
				update_lcd(&digit2);
				old_digit2 = digit2;
				//buffer_push(&digit1_buf,digit1);

				RS485_Send_Message(ID_list[SLAVE2],FUNC_WRITE,(digit2+'0'));
			}
			event = EVENT_RESET;
			state = STATE_WAITING_RX;
			old_state = STATE_TX;
			break;
		}
		break;
	case STATE_WAITING_RX:
		
		if (event==EVENT_RX_COMPLETE){
			RS485_Read_Message();
			
			if (old_state==STATE_TX){
				d_timer_update_digit=0;
				f_timer_update_digit =0;
			}
			state=old_state;
			event = EVENT_RESET;
		

		}else{
			
			if (!f_querry) break;
			f_querry =0;
			
			RS485_Send_Message(TX_msg[1],TX_msg[2],TX_msg[3]);
			tx_timeout++;
			
			if (tx_timeout==2){
				state = old_state;
				event = EVENT_RESET;	
				tx_timeout=0;
				//check_slave();
				break;

			}
			
		}
		
		break;

	}


}

void check_slave(void){

	if ((TX_msg[1]==ID_list[0])||(TX_msg[1]==0)||(TX_msg[2]==FUNC_WRITE))return;
	
	if (TX_msg[1]==ID_list[SLAVE1]){
		display_state_slave(SLAVE1, DISCONNECTED);
		ID_list[SLAVE1] = 0;
		digit1 = 16;
	}else if (TX_msg[1]== ID_list[SLAVE2]){
		display_state_slave(SLAVE2, DISCONNECTED);
		ID_list[SLAVE2]	= 0;
		digit2 =16;
	}
		
	num_slave--;
	update_lcd(&num_slave);

	
}


void RS485_RX_Task(void)
{
	
	if (rx_buffer.tail==rx_buffer.head) return;
	
	
	if (rx_buffer.data[rx_buffer.tail]==SOF)
	{
		f_read_msg =1;
		pRX_msg = RX_msg;

	}else if(rx_buffer.data[rx_buffer.tail]==EOF)
	{
		f_read_msg =0;
    	buffer_push(&event_buffer,EVENT_RX_COMPLETE);
		//RS485_Read_Message();  //event msg ready 
	}else{

		if (f_read_msg){
			*pRX_msg++=rx_buffer.data[rx_buffer.tail];
		}	
	}
	rx_buffer.tail = (rx_buffer.tail+1)%BUFFER_SIZE;

}

void RS485_Read_Message(void)
{
	//digit2 = 10;
	//uint8_t * digit;
	uint8_t checksum = 0;

	checksum = checksum^RX_msg[0]^RX_msg[1]^RX_msg[2];
	if (!(checksum== RX_msg[3]))
	{
		return;

	}

	if ((RX_msg[0] == ID_list[SLAVE1]))
	{
		if (RX_msg[1] == FUNC_READ)
		{	
			digit1 = (RX_msg[2] - '0');
			
			if (digit1!= old_digit1){
				update_lcd(&digit1);
				old_digit1 = digit1;
			}
			
		}
		else if (RX_msg[1] == FUNC_WRITE)
		{
			
		}

	}else if ((RX_msg[0] == ID_list[SLAVE2]))
	{
		if (RX_msg[1] == FUNC_READ)
		{	
			digit2 = (RX_msg[2] - '0');
			
			if(digit2!= old_digit2)
			{
				update_lcd(&digit2);
				old_digit2 =digit2;
			}
			
		}
		else if (RX_msg[1] == FUNC_WRITE)
		{
			
	}
	}else if(RX_msg[0]==ID_list[0]){

		if (RX_msg[1]== FUNC_FIND_SLAVE){

			if(ID_list[SLAVE1]==0){
				ID_list[SLAVE1]=RX_msg[2];
				display_state_slave(SLAVE1, CONNECTED);
			}else if (ID_list[SLAVE2]==0){
				ID_list[SLAVE2]= RX_msg[2];
				display_state_slave(SLAVE2, CONNECTED);
				}
			else{
				return;
			}
			num_slave++;
			update_lcd(&num_slave);
			
		}
		
	}
	else{
		return;
	}

	
}

void display_state_slave(uint8_t slave, uint8_t state){
	
	
	lcd_clear();
	
	if (state == DISCONNECTED)
	{
			lcd_set_pos(0, 0x1);
			lcd_write_string("DISCONNECTED");
	}
	else{
			lcd_set_pos(0, 0x3);
			lcd_write_string("CONNECTED");
	}
	
	if (slave ==SLAVE1)
	{
		lcd_set_pos(1, 0x2);
		lcd_write_string("S1 ADDR:");
	
		lcd_set_pos(1, 0xC);
		temp = ID_list[SLAVE1]/16;
		lcd_data(digit_table[temp]);
		lcd_set_pos(1, 0xD);
		temp = ID_list[SLAVE1]%16;
		lcd_data(digit_table[temp]);
	}
	else if(slave == SLAVE2){
		lcd_set_pos(1, 0x2);
		lcd_write_string("S2 ADDR:");
		
		lcd_set_pos(1, 0xC);
		temp = ID_list[SLAVE2]/16;
		lcd_data(digit_table[temp]);
		lcd_set_pos(1, 0xD);
		temp = ID_list[SLAVE2]%16;
		lcd_data(digit_table[temp]);
	}

	HAL_Delay(1000);
	
	lcd_clear();
	lcd_set_pos(0, 0);
	lcd_write_string("NUM SLAVE: ");
	lcd_set_pos(0, 0xC);
	lcd_data(digit_table[num_slave]);
	lcd_set_pos(1, 0);
	lcd_write_string("S2-");

	lcd_set_pos(1, 0x3);
	temp = ID_list[SLAVE2]/16;
	lcd_data(digit_table[temp]);
	lcd_set_pos(1, 0x4);
	temp = ID_list[SLAVE2]%16;
	lcd_data(digit_table[temp]);
	lcd_set_pos(1, 0x5);
	lcd_data(':');
	lcd_set_pos(1, 0x6);
	lcd_data(digit_table[digit2]);


	lcd_set_pos(1, 0x9);
	lcd_write_string("S1-");

	lcd_set_pos(1, 0xC);
	temp = ID_list[SLAVE1]/16;
	lcd_data(digit_table[temp]);
	lcd_set_pos(1, 0xD);
	temp = ID_list[SLAVE1]%16;
	lcd_data(digit_table[temp]);
	lcd_set_pos(1, 0xE);
	lcd_data(':');
	lcd_set_pos(1, 0xF);
	lcd_data(digit_table[digit1]);
}

void RS485_Send_Message(uint8_t addr, uint8_t func_code, uint8_t data)
{
	
	TX_msg[0] = SOF;
	TX_msg[1] = addr;
	TX_msg[2] = func_code;
	TX_msg[3] = data;
	TX_msg[4] = (((0x00^TX_msg[1])^TX_msg[2])^TX_msg[3]);    // checksum
	TX_msg[5] = EOF;

	// uint8_t *pbuf_tx = (uint8_t *)&msg;
	HAL_GPIO_WritePin(TX1_EN_GPIO_Port, TX1_EN_Pin, 1); /// Enable Transmitter Mode
	HAL_UART_Transmit(&huart4, TX_msg, sizeof(TX_msg), 10);
	HAL_GPIO_WritePin(TX1_EN_GPIO_Port, TX1_EN_Pin, 0); /// Enable Receiver Mode
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Check which version of the timer triggered this callback and toggle LED

	if (htim == &htim4)
	{
		f_timer_10ms = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	if (huart == &huart4)
	{
		buffer_push(&rx_buffer, rx_temp);
		HAL_UART_Receive_IT(&huart4, &rx_temp, 1);
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
