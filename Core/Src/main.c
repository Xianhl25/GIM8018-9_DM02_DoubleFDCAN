/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fdcan.h"
#include "memorymap.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "math.h"
#include "lcd.h"
#include "pic.h"
#include "can_comm.h"
#include "motor_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

char YAWCMD[3] = {0XFF,0XAA,0X52};
char ACCCMD[3] = {0XFF,0XAA,0X67};
char SLEEPCMD[3] = {0XFF,0XAA,0X60};
char UARTMODECMD[3] = {0XFF,0XAA,0X61};
char IICMODECMD[3] = {0XFF,0XAA,0X62};

struct SAcc ACC;
struct SAngle Angle;
struct SGyro Gyro;
struct SHeight Height;

float angle_x;
float angle_y;
float angle_z;

float acc_x;
float acc_y;
float acc_z;

float gyro_x;
float gyro_y;
float gyro_z;

float height;

uint8_t rx_buff1;
uint8_t data[8];

volatile float Can1MotorCurVelocity = 0;
volatile float Can1MotorCurTorque = 0;
volatile float Can2MotorCurVelocity = 0;
volatile float Can2MotorCurTorque = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI1_Init();
  MX_UART7_Init();
  MX_USART10_UART_Init();
  /* USER CODE BEGIN 2 */
  
  LCD_Init();
  LCD_Fill(0,0,LCD_W, LCD_H,BLACK);
  
  MX_FDCAN1_FilterInit();
  MX_FDCAN2_FilterInit();
  
  HAL_Delay(100);
  
	sendcmd((uint8_t*)ACCCMD);
	sendcmd((uint8_t*)YAWCMD);
	int count = 0;
	HAL_UART_Receive_IT(&huart7,&rx_buff1,1);
	
	Can1MotorControl_Start();
	HAL_Delay(100);
	
	Can1Comm_SendControlPara(0.0f, 0.0f, 0.0f, 0.0f, 0.5f);
	HAL_Delay(100);
	
	Can2MotorControl_Start();
	HAL_Delay(100);
	
	Can2Comm_SendControlPara(0.0f, 0.0f, 0.0f, 0.0f, 0.5f);
	HAL_Delay(100);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	/* IMU数据接收测试 */
//	sendcmd((uint8_t*)ACCCMD);
//	sendcmd((uint8_t*)YAWCMD);
//	HAL_UART_Receive_IT(&huart7,&rx_buff1,1);
//	get_imu_data();
	  
//	/* CAN数据发送测试 */
//	uint8_t num = 0xff;
//	FDCAN1_Send_Msg(&num, 0x001);
//	FDCAN2_Send_Msg(&num, 0x001);
	  
	/* CAN1电机测试 */
	Can1Comm_SendControlPara(0.0f, 0.0f, 0.0f, 0.0f, 0.5f);
	  
	/* CAN2电机测试 */

	  
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

float getFloat(uint32_t value) 
{
	/* 基于IEEE743协议将十六进制数据转换为单精度浮点数 */
	
    /* 提取符号位 (S) */
    unsigned int value_s = (value >> 31) & 0x01;
    
    /* 提取指数位 (E) */
    unsigned int value_e = (value >> 23) & 0xFF;
    
    /* 计算尾数位 (M) */
    float value_M = 0.0f;
	
    for (int i = 0; i < 23; i++) 
	{
        int binaryData = (value >> i) & 1;
        
        float bitX = powf(2, (-23 + i));
		
        value_M += binaryData * bitX;
    }

    /* 计算 (E - 127) */
    float E_127 = (float)value_e - 127.0f;

    /* 计算最终的浮点数值 */
    float res = powf(-1, value_s) * (1.0f + value_M) * powf(2, E_127);

    return res;
}

void sendcmd(uint8_t data[3])
{
	static uint8_t tx_buff;
	for(int i=0;i<3;i++)
	{
		tx_buff = data[i];
		HAL_UART_Transmit(&huart7,&tx_buff,1,0Xff);
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) // 如果是USART1
    {
		CopeSerial2Data(rx_buff1);
		HAL_UART_Receive_IT(&huart7,&rx_buff1,1);	
	}
}

void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55)
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x51:	memcpy(&ACC,&ucRxBuffer[2],8);break;    /* 加速度输出 */
			case 0x52:	memcpy(&Gyro,&ucRxBuffer[2],8);break;   /* 角速度输出 */
			case 0x53:	memcpy(&Angle,&ucRxBuffer[2],8);break;  /* 角度输出 */
			case 0x56:  memcpy(&Height,&ucRxBuffer[2],8);break; /* 气压高度输出 */

		}
		ucRxCnt=0;
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	
	if (hfdcan == &hfdcan1)
	{
		uint16_t tmp_value;
		
		FDCAN1_Receive_Msg((uint8_t *)data);
		
		if(data[0] == CAN_SLAVE_ID)
		{
//			tmp_value = (data[3]<<4)|(data[4]>>4);
//			Can1MotorCurVelocity = uint_to_float(tmp_value, V_MIN, V_MAX, 12);
			tmp_value = ((data[4] & 0x0F) << 8) | data[5];
			Can1MotorCurTorque = uint_to_float(tmp_value, T_MIN, T_MAX, 12);
		}
	}
	
	if (hfdcan == &hfdcan2)
	{
		uint16_t tmp_value;
		
		
		FDCAN2_Receive_Msg((uint8_t *)data);
		
		if(data[0] == CAN_SLAVE_ID)
		{
			tmp_value = (data[3]<<4)|(data[4]>>4);
			Can2MotorCurVelocity = uint_to_float(tmp_value, V_MIN, V_MAX, 12);
		}
	}
}

void get_imu_data()
{		
		angle_x = (float)Angle.Angle[1]/32768*180;
		angle_y = (float)Angle.Angle[0]/32768*180;
		angle_z = (float)Angle.Angle[2]/32768*180;
			
		gyro_x = (float)Gyro.w[1]/32768*2000;
		gyro_y = (float)Gyro.w[0]/32768*2000;
		gyro_z = (float)Gyro.w[2]/32768*2000;
		
		acc_x = (float)ACC.a[1]/32768*16;
		acc_y = (float)ACC.a[0]/32768*16;
		acc_z = (float)ACC.a[2]/32768*16;
	
		height = (float)((Height.Height[7]<<24)|(Height.Height[6]<<16)|(Height.Height[5]<<8)|Height.Height[4]);
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
