/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define SMARTCARD_HANDLE hsmartcard1
extern SMARTCARD_HandleTypeDef hsmartcard1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define SC_VOLTAGE_5V GPIO_PIN_SET
#define SC_VOLTAGE_3V GPIO_PIN_RESET

#define SC_VoltageConfig5V()  HAL_GPIO_WritePin(SCARD_3_5V_GPIO_Port, SCARD_3_5V_Pin, GPIO_PIN_SET)
#define SC_VoltageConfig3V()  HAL_GPIO_WritePin(SCARD_3_5V_GPIO_Port, SCARD_3_5V_Pin, GPIO_PIN_RESET)

#define SC_PowerCmdHigh()     HAL_GPIO_WritePin(SCARD_VCC_GPIO_Port, SCARD_VCC_Pin, GPIO_PIN_SET)
#define SC_PowerCmdLow()      HAL_GPIO_WritePin(SCARD_VCC_GPIO_Port, SCARD_VCC_Pin, GPIO_PIN_RESET)

#define SC_ResetHigh()        HAL_GPIO_WritePin(SCARD_RESET_GPIO_Port, SCARD_RESET_Pin, GPIO_PIN_SET)
#define SC_ResetLow()         HAL_GPIO_WritePin(SCARD_RESET_GPIO_Port, SCARD_RESET_Pin, GPIO_PIN_RESET)

#define GREEN_LED_ON() HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET )
#define GREEN_LED_OFF() HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET )
#define RED_LED_ON() HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET )
#define RED_LED_OFF() HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET )
#define BLUE_LED_ON() HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET )
#define BLUE_LED_OFF() HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET )

static int8_t SC_Detect(void)
{
   return (HAL_GPIO_ReadPin(SCARD_OFF_GPIO_Port, SCARD_OFF_Pin) == GPIO_PIN_SET );
}
void SC_PowerOff(void)
{
   SC_ResetLow();
   /* Disable CMDVCC */
   SC_PowerCmdHigh();
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int state = 0;
uint8_t ATR[16];
uint8_t ATR_length;
#define ATR_MAX_LENGTH 16
uint8_t command[5] = { //// CLA, INS, P1, P2, LEN
   0,
   0xB6, // read system
   0,    // add High
   0x0C, // address Low
   4  // read 4 bytes
};
uint8_t manu[7]; 
static void SC_SendByteCard(uint8_t SCData)
{
   unsigned char ch;
   while(SC_Detect()) {
      HAL_SMARTCARD_Transmit(&SMARTCARD_HANDLE, &SCData, 1, 300);      
      if ( __HAL_SMARTCARD_GET_FLAG(&SMARTCARD_HANDLE, SMARTCARD_FLAG_FE) == SET) {
         ch = SMARTCARD_HANDLE.Instance->RDR;
         continue;
      }
      /* If a Noise error is signaled by the card */
      do {
         /* Clear the UART1 Frame noise flag */
         ch = SMARTCARD_HANDLE.Instance->RDR;
         if ( !SC_Detect()) break;
      } while (__HAL_SMARTCARD_GET_FLAG(&SMARTCARD_HANDLE, SMARTCARD_FLAG_NE) == SET);                     
      break;
   }
}
void ISO_send_command(uint8_t *command, uint8_t len )
{
   uint16_t i;
   for ( i = 0; i < len; i ++ ) {
      SC_SendByteCard( *command++ );
   }
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART1_SMARTCARD_Init();

  /* USER CODE BEGIN 2 */
   SC_PowerOff();
   /* Select 3 V for smart card */
   SC_VoltageConfig3V();
  
  GREEN_LED_ON();
  RED_LED_ON();
  BLUE_LED_ON();
  HAL_Delay(1000);
  GREEN_LED_OFF();
  RED_LED_OFF();
  BLUE_LED_OFF();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_StatusTypeDef status;
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
     switch( state ) {
       case 0:
           
         if( SC_Detect()) { // smart card detected 
            uint32_t manu_code;
            state = 1;
            BLUE_LED_ON();
            HAL_Delay(300); 
            /* Enable CMDVCC */
            SC_PowerCmdLow();
            /* reset card sequence */
            SC_ResetLow();
            HAL_Delay(200);
            SC_ResetHigh();
            status = HAL_SMARTCARD_Receive(&SMARTCARD_HANDLE, ATR, ATR_MAX_LENGTH, 1500 );
            ATR_length = SMARTCARD_HANDLE.RxXferSize- SMARTCARD_HANDLE.RxXferCount;
            if ( status != HAL_OK ) 
               ATR_length--;
            if ( ATR_length != 8 ) break;
            // ATR_length == 8
            HAL_Delay(200);
            // veriy MANUFACTURE code in configuration, address 0, C
            
            ISO_send_command(command, 5 );
            
            status = HAL_SMARTCARD_Receive(&SMARTCARD_HANDLE, manu, 7, 1000 ); 
            // Send data:    00 B6 00 0C 04: CLA INS P1 P2 LEN
            // receive data: B6 7F 94 89 10 90 00
            // where 
            // [B6] ACK of INS
            // [7F 94 89 10] are return bytes
            // [90 00] is status 
            if ( status != HAL_OK ) {
                RED_LED_ON();
                BLUE_LED_OFF();
                break;
            }
            // 
#define MANUCODE 0x7F948910            
            manu_code = manu[1];
            manu_code = (manu_code << 8) + manu[2];
            manu_code = (manu_code << 8) + manu[3];
            manu_code = (manu_code << 8) + manu[4];
            if ( manu_code == MANUCODE ) {
               GREEN_LED_ON();
            } else 
               RED_LED_ON();
            
         }
         break;
       case 1:
         if( !SC_Detect()) { // smart card not detected 
            __HAL_SMARTCARD_CLEAR_FEFLAG(&SMARTCARD_HANDLE); 
            __HAL_SMARTCARD_CLEAR_NEFLAG(&SMARTCARD_HANDLE); 
            __HAL_SMARTCARD_CLEAR_OREFLAG(&SMARTCARD_HANDLE); 
            BLUE_LED_OFF();
            GREEN_LED_OFF();
            RED_LED_OFF();
            
            SC_PowerOff(); // power off card controls
            state = 0;
         }
         break;
      }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
