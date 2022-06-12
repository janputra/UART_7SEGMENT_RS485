#ifndef __LCD16x2_H__
#define __LCD16x2_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */
void send_to_lcd(char data);
void lcd_cmd(char cmd);
void lcd_data(char data);
void lcd_init(void);
void lcd_set_pos(int row, int col);
void lcd_clear(void);
void lcd_write_string(char *s);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ LCD16x2_H__ */
