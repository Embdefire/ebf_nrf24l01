/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   无线模块/nrf24l01+/双板测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火  STM32767 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "main.h"  
#include "stm32f7xx.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_usart.h"
#include "./NRF24L01/bsp_nrf24l01.h"
#include "./key/bsp_key.h"

uint8_t status;	               // 用于判断接收/发送状态
uint8_t status2;                // 用于判断接收/发送状态
uint8_t txbuf[32]={0,1,2,3,6,6,8,88,9};	   // 发送缓冲
uint8_t rxbuf[32];			         // 接收缓冲
int i=0;

void Self_Test(void);

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{
	  
    /* 配置系统时钟为216 MHz */
    SystemClock_Config();
	
	  SCB_EnableICache();//使能I-Cache
    SCB_EnableDCache();//使能D-Cache  
  
    /* 初始化RGB彩灯 */
    LED_GPIO_Config();

    /* 初始化USART1 配置模式为 115200 8-N-1 */
    UARTx_Config();
	  
	  /*初始化NRF2*/
	  SPI_NRF2_Init();
	
	  Self_Test();
   
}
void Self_Test(void)
{

	/*检测 NRF2 模块与 MCU 的连接*/
  status = NRF2_Check();
	
	 /*判断连接状态*/  
  if(status == SUCCESS)	   
    printf("\r\n      NRF2与MCU连接成功！ 按 K2 发送数据\r\n");  
  else	  
    printf("\r\n  NRF2与MCU连接失败，请重新检查接线。\r\n");
	
  NRF2_RX_Mode();    // NRF2 进入接收模式

  while(1)
  {
  
    /* 等待 NRF2 接收数据 */
    status2 = NRF2_Rx_Dat(rxbuf);

    /* 判断接收状态 */
    if(status2 == RX_DR)
    {
      for(i=0;i<32;i++)
      {	
        printf("\r\n NRF2[%d] 接收数据为：%d \r\n",i,rxbuf[i]); 
      }
      
      printf("\r\n按 K1 使用NRF1 发送数据\n"); 
      printf("\r\n按 K2 使用NRF2 发送数据\r\n"); 
    }
    
    /* NRF2 发送数据 */
    if (Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)    // 按键按下，开始送数据
    { 
			LED1_TOGGLE;
      /* 发送数据 */
      NRF2_TX_Mode();
           
      status2 = NRF2_Tx_Dat(txbuf);
      
      /* 发送数据的状态 */
       if(status2 == TX_DS)
      {
        printf("\r\nNRF2 发送数据成功\r\n");
      }
      else
      {
        printf("\r\nNRF2 发送数据失败  %d\r\n", status);
      }
      
      printf("\r\nNRF2 进入接收模式\r\n"); 

      NRF2_RX_Mode();
    }
  }
}


/**
  * @brief  System Clock 配置
  *         system Clock 配置如下 : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  无
  * @retval 无
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* 使能HSE，配置HSE为PLL的时钟源，配置PLL的各种分频因子M N P Q 
	 * PLLCLK = HSE/M*N/P = 25M / 25 *432 / 2 = 216M
	 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* 激活 OverDrive 模式以达到216M频率  */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* 选择PLLCLK作为SYSCLK，并配置 HCLK, PCLK1 and PCLK2 的时钟分频因子 
	 * SYSCLK = PLLCLK     = 216M
	 * HCLK   = SYSCLK / 1 = 216M
	 * PCLK2  = SYSCLK / 2 = 108M
	 * PCLK1  = SYSCLK / 4 = 54M
	 */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }  
}
void Delay(__IO uint32_t nCount)	 //简单的延时函数
{
	for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/

