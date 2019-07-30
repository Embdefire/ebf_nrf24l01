/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ����ģ��/nrf24l01+/˫�����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����  STM32767 ������
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "main.h"  
#include "stm32f7xx.h"
#include "./led/bsp_led.h" 
#include "./usart/bsp_usart.h"
#include "./NRF24L01/bsp_nrf24l01.h"
#include "./key/bsp_key.h"

uint8_t status;	               // �����жϽ���/����״̬
uint8_t status2;                // �����жϽ���/����״̬
uint8_t txbuf[32]={0,1,2,3,6,6,8,88,9};	   // ���ͻ���
uint8_t rxbuf[32];			         // ���ջ���
int i=0;

void Self_Test(void);

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
	  
    /* ����ϵͳʱ��Ϊ216 MHz */
    SystemClock_Config();
	
	  SCB_EnableICache();//ʹ��I-Cache
    SCB_EnableDCache();//ʹ��D-Cache  
  
    /* ��ʼ��RGB�ʵ� */
    LED_GPIO_Config();

    /* ��ʼ��USART1 ����ģʽΪ 115200 8-N-1 */
    UARTx_Config();
	  
	  /*��ʼ��NRF2*/
	  SPI_NRF2_Init();
	
	  Self_Test();
   
}
void Self_Test(void)
{

	/*��� NRF2 ģ���� MCU ������*/
  status = NRF2_Check();
	
	 /*�ж�����״̬*/  
  if(status == SUCCESS)	   
    printf("\r\n      NRF2��MCU���ӳɹ��� �� K2 ��������\r\n");  
  else	  
    printf("\r\n  NRF2��MCU����ʧ�ܣ������¼����ߡ�\r\n");
	
  NRF2_RX_Mode();    // NRF2 �������ģʽ

  while(1)
  {
  
    /* �ȴ� NRF2 �������� */
    status2 = NRF2_Rx_Dat(rxbuf);

    /* �жϽ���״̬ */
    if(status2 == RX_DR)
    {
      for(i=0;i<32;i++)
      {	
        printf("\r\n NRF2[%d] ��������Ϊ��%d \r\n",i,rxbuf[i]); 
      }
      
      printf("\r\n�� K1 ʹ��NRF1 ��������\n"); 
      printf("\r\n�� K2 ʹ��NRF2 ��������\r\n"); 
    }
    
    /* NRF2 �������� */
    if (Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)    // �������£���ʼ������
    { 
			LED1_TOGGLE;
      /* �������� */
      NRF2_TX_Mode();
           
      status2 = NRF2_Tx_Dat(txbuf);
      
      /* �������ݵ�״̬ */
       if(status2 == TX_DS)
      {
        printf("\r\nNRF2 �������ݳɹ�\r\n");
      }
      else
      {
        printf("\r\nNRF2 ��������ʧ��  %d\r\n", status);
      }
      
      printf("\r\nNRF2 �������ģʽ\r\n"); 

      NRF2_RX_Mode();
    }
  }
}


/**
  * @brief  System Clock ����
  *         system Clock �������� : 
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
  * @param  ��
  * @retval ��
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* ʹ��HSE������HSEΪPLL��ʱ��Դ������PLL�ĸ��ַ�Ƶ����M N P Q 
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
  
  /* ���� OverDrive ģʽ�Դﵽ216MƵ��  */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* ѡ��PLLCLK��ΪSYSCLK�������� HCLK, PCLK1 and PCLK2 ��ʱ�ӷ�Ƶ���� 
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
void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/

