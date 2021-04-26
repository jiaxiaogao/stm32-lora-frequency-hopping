/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

//#include "esp8266.h"
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
//#define MAXSLAVENUM 2

//#define REQUESTNUM 0//因为使用了求余，所以从零开始计数


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RegVersion = 0;

//char request_num = REQUESTNUM;
//uint16_t sizeof_request_num = sizeof(request_num);
//uint8_t data_frame[BUFFERSIZE] = "";
//uint16_t sizeof_data_frame = BUFFERSIZE;

//lora设备初始化的返回值
tRadioDriver *Radio = NULL;


//————————声明一下：这些数组里面存储的都是字符，使用的时候请注意，里面是‘1’，不是1，若用作下表需要减‘0’

//FDMA相关参数          
//2^32 = 4,294,967,296，fre[0]（0号频率）是公共频道
uint32_t fre[] = {470000000, 472000000, 474000000, 476000000};//目前只设置3个频道
uint8_t fre_used = 0x01;//二进制第n位标识fre[n]是否被占用，1标识占用

//占用频道指令 0字节：标志位0 1字节：占用频率  2字节：接收方地址1
//接收方地址0: 对应从机设备号，这个是定死的！一对设备只有唯一一个（可以用Lora的设备号来做）
uint8_t cmd_zy[BUFFERSIZE] = "001";
uint16_t sizeof_cmd_zy = BUFFERSIZE;//SX1276SetTxPacket这个函数参数是这样规定的，避免warning

//释放频道指令 0字节：标志位0 1字节：释放频率
uint8_t cmd_sf[BUFFERSIZE] = "10";
uint16_t sizeof_cmd_sf = BUFFERSIZE;



uint8_t data_frame[BUFFERSIZE] = "";//收到的数据暂存在这，第一个字节为2标识是通信数据
uint16_t sizeof_data_frame = BUFFERSIZE;

//控制指令码 1:前进 2：后退 3：左转    4：右转
uint8_t cmd_control[BUFFERSIZE] = "3";




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void led_init()//LED初始化为三色
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//LED_B
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);//LED_R
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);//LED_G
}
void led_send()//发送时：蓝灯
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void led_rev()//接收时：绿灯
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}
void led_debug()//debug：红灯
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

void master_task()
{
  switch(Radio->Process())
  {
  case RF_RX_DONE:
    
    Radio->GetRxPacket(data_frame, &sizeof_data_frame);
    if(data_frame[0] == '0')//占用频率指令 0字节：0标志位 1字节：占用频率  2字节：接收方地址
    {
      led_debug();//debug：红灯
      //只需要更改自己的频率使用表，其他的不管（因为这里是主机，主机不可能被当作请求的接收方的）
      
      fre_used = fre_used | (0x01 << (data_frame[1] - '0'));//更新频率使用表
      Radio->StartRx();//继续回到接收状态
      
    }
    else if(data_frame[0] == '1')//释放频率指令  0字节：0标志位 1字节：释放频率
    {
      led_debug();//debug：红灯
      fre_used = fre_used & (~(0x01 << (data_frame[1] - '0')));//更新频率使用表
      Radio->StartRx();//继续回到接收状态
      
    }
    else if(data_frame[0] == '2')//从机响应的数据
    {
      if(data_frame[1] == '1')//标识从机准备好接收数据了,直接发送命令就行了，不用管对方是否接收到，直接断开了
      {
        led_send();//发送时：蓝灯
        HAL_Delay(500);//延时，防止出现从机还没有准备好的情况
        Radio->SetTxPacket(cmd_control, sizeof_data_frame);//发送控制指令，前导码先不弄
        printf("MASTER sent cmd_control---cmd_code: %c, control: %c(1:qian,2:hou)---\r\n",cmd_control[0], cmd_control[1]);
        
        Radio->Process();//为了发出上面这个tx指令
        
        //释放当前使用的频率
        fre_used = fre_used & (~(0x01 << (cmd_zy[1] - '0' )));
        
        //更改当前频率为公共频率
        LoRaSettings.RFFrequency = fre[ 0 ];
        Radio = RadioDriverInit();
        Radio->Init();
        
        printf("---End of this communication---\r\n");//结束本次通信
        
        HAL_Delay(500);//延时，防止出现其他设备还没有准备好的情况
        cmd_sf[1] = '0';//数字1转换为字符1
        Radio->SetTxPacket(cmd_sf, sizeof_cmd_sf);//发送释放频率指令
        printf("MASTER sent cmd_sf---cmd_code: %c, fre: %c---\r\n",cmd_sf[0], cmd_sf[1]);
        
      }
    }
    break;
    
  case RF_TX_DONE:
    led_rev();
    Radio->StartRx();
    break;
  default:
    break;
  }
}

//返回是否检测到了按键：1是
uint8_t wait_key()
{
  //  HAL_Delay(5000);//延时5s，模拟等待按键
  if(HAL_GPIO_ReadPin( GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET)
  {
    HAL_Delay(100);//防抖
    if(HAL_GPIO_ReadPin( GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET)
    {
      cmd_control[1] = '1';//默认为前进
      led_debug();//debug：红灯——如果一直按着就一直是红色，松开之后就变回三色
      while(HAL_GPIO_ReadPin( GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET);//防止连按
      led_init();
      return 1;
    }
  }
  return 0;
}
void choose_fre()
{
  uint8_t i = 1;
  while(1)
  {
    if((fre_used & (0x01 << i)) == 0)//频率i没有被占用 0000 0001
    {
      cmd_zy[1] = i + '0';//选择到的频率(只是下标)
//      printf("i = %d", i);
      return;
    }
    else
    {
      i++;
      //目前先不考虑健壮性，目前只有2对设备，不会出现频率不够用的情况的
    }
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  led_init();
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
  LoRaSettings.RFFrequency = fre[0];
  Radio = RadioDriverInit();
  Radio->Init();
  
  //////	//esp初始化  
  //////    printf("-----111\r\n");
  //////    if(esp8266_init_test())
  //////        printf("esp8266_init OK!\r\n");
  //////    else
  //////        printf("esp8266_init ERROR!\r\n");
  
  
  //    Radio->SetTxPacket(&request_num, sizeof_request_num);	//初始化之后就开始轮询了
  //    printf("MASTER sent request_num000\r\n");
  
  Radio->StartRx();//进入接收状态
  printf("MASTER start rev \r\n");

  while (1)
  {
    
    if(wait_key())//检测到有按键按下
    {
      choose_fre();//选择当前可用的频率
      Radio->SetTxPacket(cmd_zy, sizeof_cmd_zy);//发送占用频率请求
      printf("MASTER send cmd_zy---cmd_code: %c, fre: %c, request_slave_addr: %c---\r\n", cmd_zy[0], cmd_zy[1], cmd_zy[2]);
      
      Radio->Process();//为了发出上面这个指令
      
      LoRaSettings.RFFrequency = fre[ cmd_zy[1] - '0' ];//更改自己的频率
//      LoRaSettings.RFFrequency = 472000000;
      Radio = RadioDriverInit();
      Radio->Init();
      
//      led_rev();//手动进入接收状态，重新初始化之后不能检测到RF_TX_DONE了
      Radio->StartRx();
//      printf("MASTER's free choose OK\r\n");
      
    }
    
    master_task();
    
    
    /* USER CODE END WHILE */
  }    
  /* USER CODE BEGIN 3 */
  
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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

//int _write (int fd, char *pBuffer, int size) 
//{ 
//for (int i = 0; i < size; i++) 
//{ 
//while((USART1->SR&0X40)==0);//等待上一次串口数据发送完成 
//USART1->DR = (uint8_t) pBuffer[i]; //写DR,串口1将发送数据
//} 
//return size; 
//}

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
