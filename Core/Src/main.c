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
#include "message.h"
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
unsigned char f_timer_TX=0;
unsigned char f_seg_timer_500ms;
unsigned char f_timer_10ms=0;
unsigned char f_timer_20ms=0;

unsigned char f_timer_30ms=0;
unsigned char i,p_tx1,p_tx2;
unsigned char flag_digit_1=1;

unsigned char tx2_buffer[10]={"123456789"};
unsigned char tx1_buffer[6]={"abcdef"};
unsigned char d_timer_30ms;
unsigned char d_timer_20ms;
unsigned char d_timer_TX1;
unsigned char TX1_delay_val =250;
unsigned char d_timer_TX2;
unsigned char TX2_delay_val =250;
unsigned char key1_data, key2_data;
unsigned char state,event;

unsigned char lcd_disp_lock=0;

unsigned char uart_tx1_flag,uart_tx2_flag;
unsigned char flag_state_tx1, flag_state_tx;

unsigned char digit1,digit2;
//flag for LCD
unsigned char is_EN=0;
unsigned char lcd_digit1_f=0;
unsigned char digit1_update = 0;
unsigned char digit2_update = 0;
unsigned char cmd, data;
unsigned char lcd_process=0;

unsigned char digit_table[17]={"0123456789abcdef-"};

uint8_t rx_temp;
uint8_t transmission_f;
uint8_t start_cmd=0x2;
uint8_t stop_cmd=0x3;
circular_buffer rx1_buffer;
circular_buffer rx2_buffer;
circular_buffer event_buffer;
uint8_t ID;
uint8_t TX_msg[4];
uint8_t RX_msg[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void task_timer(void);
void counting_task(void);
void uart_TX2_task(void);
void uart_TX1_task(void);
void uart_RX1_task(void);
void uart_RX2_task(void);
void segment_display_task(void);
void lcd_display_task(void);
void key_read_task(void);
void main_task(void);
void setEvent(unsigned char event);
unsigned char getEvent(void);
void seven_segment_driver(char input,char select_digit);
void TX1_delay_update(void);
void TX2_delay_update(void);
void m_send_to_lcd(char data);
void Set_Transmitter_Port1(void);
void Set_Transmitter_Port2(void);
void Set_Receiver_Port1(void);
void Set_Receiver_Port2(void);
void RS485_Send_Message(void);
void RS485_Read_Message(void);
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

  	
  HAL_GPIO_WritePin(TX1_EN_GPIO_Port, TX1_EN_Pin, 0);   // Enable Receiver 1
  HAL_GPIO_WritePin(TX2_EN_GPIO_Port, TX2_EN_Pin, 0);	// Enable Receiver 2

  HAL_UART_Receive_IT(&huart4, &rx1_buffer.data[rx1_buffer.head], 1);
  HAL_UART_Receive_IT(&huart5, &rx2_buffer.data[rx2_buffer.head], 1);

  ID=1;


  lcd_init();
  lcd_clear();
  lcd_set_pos(0, 3);
  lcd_write_string("TETRADYNE");
  lcd_set_pos(1, 0);
  lcd_write_string("D2:");
  lcd_set_pos(1, 0xC);
  lcd_write_string("D1:");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	  	 task_timer();
	  	     ///segment_display_task();
	  	     lcd_display_task();
	  	     key_read_task();

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

void lcd_display_task(void){


	if(!lcd_digit1_f){

		 cmd = 0xC3;
		 data = digit_table[digit2];

	}else{

		cmd = 0xCF;
        data = digit_table[digit1];

	}
	switch(lcd_process){

			case 0:
					HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, 0);
					m_send_to_lcd((cmd>>4)&0x0f);
					break;
			case 1:
					HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, 0);
					m_send_to_lcd((cmd)&0x0f);
					break;
			case 2:
					HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, 1);
					m_send_to_lcd((data>>4)&0x0f);
					break;
			case 3:
					HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, 1);
					m_send_to_lcd(data&0x0f);
					break;
	}
}


void m_send_to_lcd(char data)
{

	//writing data to pin PE0~PE3
	GPIOE->ODR =  (GPIOE->ODR & 0xFFFFFFF0) | data;

	if(!f_timer_20ms) return ;
	f_timer_20ms =0;

	if (!is_EN){
		HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 1);
		is_EN=1;

	}else{

		HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, 0);
		is_EN=0;
		lcd_process++;
		if (lcd_process>3){
			lcd_process=0;
			lcd_digit1_f= (~lcd_digit1_f)&0x1;
		}
	}
}


void task_timer(void)
{
	if(!f_timer_10ms) return;       // checking if 10 ms timer interrupt is set (10 ms), if set then do timer task
	f_timer_10ms =0;		// clear the flag to wait next interupt

	d_timer_30ms++;			// count timer for 30 ms interval
			
	if(d_timer_30ms==3)		// checking if the count reached 30 ms
	{
		d_timer_30ms =0;	// assign "0" to repeat counting
		f_timer_30ms=1;		// Set flag to inform 30 ms timer is done counting
	}

	d_timer_20ms++;
	if (d_timer_20ms==2){

		d_timer_20ms =0;
		f_timer_20ms=1;
	}
	

	d_timer_TX1++;
		if(d_timer_TX1>=TX1_delay_val)     // checking if the count reached LED interval
		{
			d_timer_TX1=0;		// assign "0" to repeat counting
			buffer_push(&event_buffer,EVENT_TX1_UPDATE);

		}
/*
	d_timer_TX2++;
		if(d_timer_TX2>=TX2_delay_val)     // checking if the count reached LED interval
		{
			d_timer_TX2=0;		// assign "0" to repeat counting
			buffer_push(&event_buffer,EVENT_TX2_UPDATE);

		}
*/

}

void key_read_task(void)
{
	if(!f_timer_30ms) return;  		 // Checking if 30 ms counting is done
	f_timer_30ms =0;          		 // clear the flag to wait next counting

	uint8_t key_pindata = (uint8_t)(GPIOG->IDR & (KEY1_Pin|KEY2_Pin));

	key1_data = key1_data<<1;      		 //Preparing to read KEY1 Input
	key1_data &= 0b00001110;
	key1_data |= ((key_pindata>>2) & 0x1);			 // Read KEY1 Input

	key2_data = key2_data<<1;		 //Preparing to read KEY2 Input
	key2_data &= 0b00001110;
	key2_data |= (key_pindata>>3);    		 // Read KEY2 Input

	if(key1_data == KEY_PRESSED)    	 // Checking if KEY1 is pressed
	{
		buffer_push(&event_buffer,EVENT_KEY1_PRESSED) ; // Store the event in buffer

	}else if(key1_data == KEY_RELEASED)		//  Checking if KEY1 is released
	{
		buffer_push(&event_buffer,EVENT_KEY1_RELEASED); // Store the event in buffer
	}

	if(key2_data == KEY_PRESSED)		// Checking if KEY2 is pressed
	{
		buffer_push(&event_buffer,EVENT_KEY2_PRESSED); // Store the event in buffer

	}else if(key2_data == KEY_RELEASED)		//  Checking if KEY2 is released
	{
		buffer_push(&event_buffer,EVENT_KEY2_RELEASED); // Store the event in buffer
	}

}


void main_task(void)
{

	if (event_buffer.head!=event_buffer.tail){
		event = buffer_pop(&event_buffer);   // if there is event then get the event from buffer
	}

	switch(state)
	{
		case STATE_IDLE:

			switch(event)
				{
					case EVENT_KEY1_PRESSED:
						
							
						TX1_delay_val=250;
						d_timer_TX1=250;
						///Set_Transmitter_Port1();
						state = STATE_TX1;
												
						break;
					case EVENT_RX_COMPLETE:
						RS485_Read_Message();
												
						break;
						/*
					case EVENT_KEY2_PRESSED:
						TX2_delay_val=250;
						d_timer_TX2=250;
						Set_Transmitter_Port2();
						state = STATE_TX2;
						
						break;*/
				}
			break;

		case STATE_TX1:

			switch (event){

				case EVENT_TX1_UPDATE:
						TX_msg[0] = 0x2; 
						TX_msg[1] = FUNC_WRITE;
						TX_msg[2] = tx2_buffer[p_tx2++];
						if (p_tx2>8){
							p_tx2=0;
						}

						RS485_Send_Message();
					
						TX1_delay_update();
						event=0;
						break;
				case EVENT_RX_COMPLETE:
						RS485_Read_Message();
												
						break;
					/*
				case EVENT_KEY2_PRESSED:
					TX2_delay_val=250;
					d_timer_TX2=250;

					state = STATE_TX_ALL;
					break;*/

				case EVENT_KEY1_RELEASED:
					//Set_Receiver_Port1();
					state = STATE_IDLE;


					break;

			}

			break;

	
	}

}

/*
void TX1_delay_update(void){

	if (TX1_delay_val==50) return;

	TX1_delay_val-=50;
}

void TX2_delay_update(void){

	if (TX2_delay_val==50) return;

	TX2_delay_val-=50;
}

*/

void RS485_Read_Message(void){

uint8_t*digit;
  //if (transmission_f) return;
  if(rx2_buffer.tail==rx2_buffer.head) return;

  buffer_to_message(&rx2_buffer, RX_msg);

 // if (check_checksum(&RX_msg)==CHECKSUM_ERROR) return;
  if (RX_msg[0]== 0x1) {
	digit=&digit1;

  }else if(RX_msg[0]== 0x2){
	digit=&digit2

  }
/*
  if (RX_msg.function_code == FUNC_READ)
  {

  }
  */

   if (RX_msg[1] == FUNC_WRITE)
  { 
    	*digit= RX_msg[2]-'0';
	
  }
  
}

void RS485_Send_Message(void)
{

   //uint8_t *pbuf_tx = (uint8_t *)&msg; 
   HAL_GPIO_WritePin(TX1_EN_GPIO_Port, TX1_EN_Pin, 1); /// Enable Transmitter Mode
   HAL_UART_Transmit(&huart4,&start_cmd,1,10);
  
   HAL_UART_Transmit(&huart4,TX_msg,sizeof(TX_msg),10);

   HAL_UART_Transmit(&huart4,&stop_cmd,1,10);
  
   HAL_GPIO_WritePin(TX1_EN_GPIO_Port, TX1_EN_Pin, 0); /// Enable Receiver Mode

}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED

   if(htim == &htim4)
  {
	  f_timer_10ms=1;

  }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	 if(huart == &huart5)
	{
		if (rx_temp==0x2)
		{
				transmission_f=1;
		}
		else if (rx_temp==0x3)
		{
				transmission_f=0;
				buffer_push(&event_buffer,EVENT_RX_COMPLETE);
		}
		else{

			if (transmission_f)
			{
				 buffer_push(&rx2_buffer,rx_temp);
			}
		}

		HAL_UART_Receive_IT(&huart5, &rx_temp, 1);
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

#ifdef  USE_FULL_ASSERT
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
