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

#define SLAVENUM 0


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t RegVersion = 0;

uint8_t data_frame_rev[BUFFERSIZE] = "";//�յ������ݴ�����
uint8_t data_frame_send[BUFFERSIZE] = "2";//���������ݴ�����
uint16_t sizeof_data_frame = BUFFERSIZE;

//lora�豸��ʼ���ķ���ֵ
tRadioDriver *Radio = NULL;





//FDMA��ز���
//2^32 = 4,294,967,296��fre[0]��0��Ƶ�ʣ��ǹ���Ƶ��
uint32_t fre[] = {470000000, 472000000, 474000000, 476000000};//Ŀǰֻ����3��Ƶ��
uint8_t fre_used = 0x01;//�����Ƶ�nλ��ʶfre[n]�Ƿ�ռ�ã�1��ʶռ��
//
////ռ��Ƶ��ָ�� 0�ֽڣ���־λ0 1�ֽڣ�ռ��Ƶ��  2�ֽڣ����շ���ַ0
////���շ���ַ0: ��Ӧ�ӻ��豸�ţ�����Ƕ����ģ�һ���豸ֻ��Ψһһ����������Lora���豸��������
//uint8_t cmd_zy[BUFFERSIZE] = "000";
//uint16_t sizeof_cmd_zy = BUFFERSIZE;//SX1276SetTxPacket������������������涨�ģ�����warning
//
////�ͷ�Ƶ��ָ�� 0�ֽڣ���־λ0 1�ֽڣ��ͷ�Ƶ��
//uint8_t cmd_sf[BUFFERSIZE] = "10";
//uint16_t sizeof_cmd_sf = BUFFERSIZE;
//
//
//uint8_t data_frame[BUFFERSIZE] = "";//�յ��������ݴ����⣬��һ���ֽ�Ϊ2��ʶ��ͨ������
//uint16_t sizeof_data_frame = BUFFERSIZE;

//����ָ���� 1:ǰ�� 2������ 3����ת    4����ת
uint8_t cmd_control[BUFFERSIZE] = "";

//������ַ
uint8_t slave_num = SLAVENUM;

//��ǰʹ�õ�Ƶ��
uint8_t fre_now = 0;

//extern tLoRaSettings LoRaSettings;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



//�������iΪ0����ʾ�صƣ�1��ʾ����
void led(uint8_t i)
{
  if(i)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void slave_task()
{
  switch(Radio->Process())
  {
  case RF_RX_DONE:
    
    Radio->GetRxPacket(data_frame_rev, &sizeof_data_frame);
    
    if(data_frame_rev[0] == '0')//ռ��Ƶ��ָ�� 0�ֽڣ�0��־λ 1�ֽڣ�ռ��Ƶ��  2�ֽڣ����շ���ַ
    {
      //ֻ��Ҫ�����Լ���Ƶ��ʹ�ñ������Ĳ��ܣ���Ϊ���������������������ܱ���������Ľ��շ��ģ�
      led(0);
      printf("SLAVE send cmd_zy---cmd_code: %c, fre: %c, request_slave_addr: %c---\r\n", data_frame_rev[0], data_frame_rev[1], data_frame_rev[2]);;
      printf("---Local addr: %d\r\n", slave_num);
      fre_used = fre_used | (0x01 << (data_frame_rev[1] - '0'));//����Ƶ��ʹ�ñ�
      
      if(data_frame_rev[2] == (slave_num + '0'))
      {
        LoRaSettings.RFFrequency = fre[ (data_frame_rev[1] - '0') ];//�����Լ���Ƶ��
        
//        LoRaSettings.RFFrequency = 472000000;
        Radio = RadioDriverInit();
        Radio->Init();

        
        fre_now = data_frame_rev[1] - '0';
        
        HAL_Delay(500);//��ʱ����ֹ����������û��׼���õ����
        data_frame_send[1] = '1';
        Radio->SetTxPacket(data_frame_send, sizeof_data_frame);//�ظ�����
//        HAL_Delay(5000);
////          �������ã�������˷����ᵼ�������ղ������ص����ݣ������ڲ�ϸ�ھ��Ȳ�ȥ׷����
//        Radio->Process();//ǿ��ˢ��
//        Radio->StartRx();//����֮������������״̬
        printf("SLAVE send cmd_zy_back---cmd_code:%c, back_num:%c(1:yes,0:no)---\r\n", data_frame_send[0], data_frame_send[1]);
        
        
      }
      
    }
    else if(data_frame_rev[0] == '1')//�ͷ�Ƶ��ָ��  0�ֽڣ�0��־λ 1�ֽڣ��ͷ�Ƶ��
    {
      
      fre_used = fre_used & (~(0x01 << (data_frame_rev[1] - '0')));//����Ƶ��ʹ�ñ�
      Radio->StartRx();//�����ص�����״̬
      
    }
    else if(data_frame_rev[0] == '3')//��������������
    {
      if(data_frame_rev[1] == '1')//����1��ǰ��
      {
        printf("SLAVE rev cmd_control---cmd_code: %c, control: %c(1:qian,2:hou)---\r\n",data_frame_rev[0], data_frame_rev[1]);
        //ʹ��led����2�£���ʶǰ��
        led(0);
        HAL_Delay(500);//��ʱ
        led(1);
        HAL_Delay(500);//��ʱ
        led(0);
        HAL_Delay(500);//��ʱ
        led(1);
        HAL_Delay(500);//��ʱ
        
        
        
        //����Ҫ�����������ͷ�ָ����
        //�ͷŵ�ǰʹ�õ�Ƶ��
        fre_used = fre_used & (~(0x01 << fre_now));
        
        //���ĵ�ǰƵ��Ϊ����Ƶ��
        LoRaSettings.RFFrequency = fre[ 0 ];
        Radio = RadioDriverInit();
        Radio->Init();
        
        Radio->StartRx();//�˴�ͨ�Ž������������״̬�ȴ���һ��ͨ��
        printf("---End of this communication---\r\n");//��������ͨ��
        
      }
    }
    
    break;
    
  case RF_TX_DONE:
    
    Radio->StartRx();//����֮������������״̬
    led(1);//�ȴ����յ�ʱ��led��
    
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
  
  //sx1278��ʼ��
  Radio = RadioDriverInit();
  Radio->Init();
  
  //��ʼ��֮��Ĭ�Ͻ������״̬
  Radio->StartRx();
  led(1);//�ȴ����յ�ʱ��led��
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


//�ض��򴮿�
int fputc(int c, FILE *stream)
{
  while(!(USART1->SR & (1 << 7)));//�ȴ����ݷ������
  USART1->DR = c;//��c��������1��DR�Ĵ��������ض��򵽴��ڣ�Ҳ�����������Ľӿ�
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
