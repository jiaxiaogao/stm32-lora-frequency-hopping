/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under BSD 3-Clause license,
* the "License"; You may not use this file except in compliance with the
* License. You may obtain a copy of the License at:
*                        opensource.org/licenses/BSD-3-Clause
*
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//lora
#include "platform.h"
#include "radio.h"
#include "sx1276-Hal.h"
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BUFFERSIZE 5

#define SLAVENUM 1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t RegVersion = 0;

uint8_t data_frame_rev[BUFFERSIZE] = "";//收到数据暂存数组
uint8_t data_frame_send[BUFFERSIZE] = "2";//发送数据暂存数组
uint16_t sizeof_data_frame = BUFFERSIZE;

//lora设备初始化的返回值
tRadioDriver *Radio = NULL;





//FDMA相关参数
//2^32 = 4,294,967,296，fre[0]（0号频率）是公共频道
uint32_t fre[] = {470000000, 472000000, 474000000, 476000000};//目前只设置3个频道
uint8_t fre_used = 0x01;//二进制第n位标识fre[n]是否被占用，1标识占用
//
////占用频道指令 0字节：标志位0 1字节：占用频率  2字节：接收方地址0
////接收方地址0: 对应从机设备号，这个是定死的！一对设备只有唯一一个（可以用Lora的设备号来做）
//uint8_t cmd_zy[BUFFERSIZE] = "000";
//uint16_t sizeof_cmd_zy = BUFFERSIZE;//SX1276SetTxPacket这个函数参数是这样规定的，避免warning
//
////释放频道指令 0字节：标志位0 1字节：释放频率
//uint8_t cmd_sf[BUFFERSIZE] = "10";
//uint16_t sizeof_cmd_sf = BUFFERSIZE;
//
//
//uint8_t data_frame[BUFFERSIZE] = "";//收到的数据暂存在这，第一个字节为2标识是通信数据
//uint16_t sizeof_data_frame = BUFFERSIZE;

//控制指令码 1:前进 2：后退 3：左转    4：右转
uint8_t cmd_control[BUFFERSIZE] = "";

//本机地址
uint8_t slave_num = SLAVENUM;

//当前使用的频率
uint8_t fre_now = 0;

//extern tLoRaSettings LoRaSettings;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



//传入参数i为0，表示关灯，1表示开灯
void led(uint8_t i)
{
  if(i)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);//LED_R
  else
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

void slave_task()
{
  switch(Radio->Process())
  {
  case RF_RX_DONE:
    
    Radio->GetRxPacket(data_frame_rev, &sizeof_data_frame);
    
    if(data_frame_rev[0] == '0')//占用频率指令 0字节：0标志位 1字节：占用频率  2字节：接收方地址
    {
      //只需要更改自己的频率使用表，其他的不管（因为这里是主机，主机不可能被当作请求的接收方的）
      led(0);
      printf("SLAVE send cmd_zy---cmd_code: %c, fre: %c, request_slave_addr: %c---\r\n", data_frame_rev[0], data_frame_rev[1], data_frame_rev[2]);;
      printf("---Local addr: %d\r\n", slave_num);
      fre_used = fre_used | (0x01 << (data_frame_rev[1] - '0'));//更新频率使用表
      
      if(data_frame_rev[2] == (slave_num + '0'))
      {
        LoRaSettings.RFFrequency = fre[ (data_frame_rev[1] - '0') ];//更改自己的频率
        
//        LoRaSettings.RFFrequency = 472000000;
        Radio = RadioDriverInit();
        Radio->Init();

        
        fre_now = data_frame_rev[1] - '0';
        
        HAL_Delay(500);//延时，防止出现主机还没有准备好的情况
        data_frame_send[1] = '1';
        Radio->SetTxPacket(data_frame_send, sizeof_data_frame);//回复数据
//        HAL_Delay(5000);
////          并不能用，这个用了反而会导致主机收不到返回的数据，具体内部细节就先不去追究了
//        Radio->Process();//强制刷新
//        Radio->StartRx();//发完之后立马进入接受状态
        printf("SLAVE send cmd_zy_back---cmd_code:%c, back_num:%c(1:yes,0:no)---\r\n", data_frame_send[0], data_frame_send[1]);
        
        
      }
      
    }
    else if(data_frame_rev[0] == '1')//释放频率指令  0字节：0标志位 1字节：释放频率
    {
      
      fre_used = fre_used & (~(0x01 << (data_frame_rev[1] - '0')));//更新频率使用表
      Radio->StartRx();//继续回到接收状态
      
    }
    else if(data_frame_rev[0] == '3')//主机发来的数据
    {
      if(data_frame_rev[1] == '1')//命令1：前进
      {
        printf("SLAVE rev cmd_control---cmd_code: %c, control: %c(1:qian,2:hou)---\r\n",data_frame_rev[0], data_frame_rev[1]);
        //使用led闪亮2下，标识前进
        led(0);
        HAL_Delay(500);//延时
        led(1);
        HAL_Delay(500);//延时
        led(0);
        HAL_Delay(500);//延时
        led(1);
        HAL_Delay(500);//延时
        
        
        
        //不需要接收主机的释放指令了
        //释放当前使用的频率
        fre_used = fre_used & (~(0x01 << fre_now));
        
        //更改当前频率为公共频率
        LoRaSettings.RFFrequency = fre[ 0 ];
        Radio = RadioDriverInit();
        Radio->Init();
        
        Radio->StartRx();//此次通信结束，进入接收状态等待下一次通信
        printf("---End of this communication---\r\n");//结束本次通信
        
      }
    }
    
    break;
    
  case RF_TX_DONE:
    
    Radio->StartRx();//发完之后立马进入接受状态
    led(1);//等待接收的时候led亮
    
    break;
    
    
  default:
    
    break;
  }
}

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  printf("Hello World!\r\n");
  
  SX1276Read( REG_LR_VERSION, &RegVersion );
  if(RegVersion != 0x12)
  {
    printf("get RegVersion error, RegVersion = %#x\r\n", RegVersion);
  }
  else
  {
    printf("get RegVersion success, RegVersion = %#x\r\n", RegVersion);
  }
  
  //sx1278初始化
  Radio = RadioDriverInit();
  Radio->Init();
  
  //初始化之后默认进入接收状态
  Radio->StartRx();
  led(1);//等待接收的时候led亮
  printf("SLAVE start rev \r\n");
  
 
  
  while (1)
  {

    slave_task();
    
    
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
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
  
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


//重定向串口
int fputc(int c, FILE *stream)
{
  while(!(USART1->SR & (1 << 7)));//等待数据发送完成
  USART1->DR = c;//将c赋给串口1的DR寄存器，即重定向到串口，也可以是其他的接口
  return c;
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
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
