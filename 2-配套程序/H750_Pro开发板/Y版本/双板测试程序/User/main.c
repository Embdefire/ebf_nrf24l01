/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ����ģ��/nrf24l01+/˫�����
  ******************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32H750 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************
  */  
#include "stm32h7xx.h"
#include "main.h"
#include "./led/bsp_led.h"
#include "./delay/core_delay.h" 
#include "./mpu/bsp_mpu.h" 
#include "./usart/bsp_usart.h"
#include "./NRF24L01/bsp_nrf24l01.h"
#include "./key/bsp_key.h"


uint8_t status;	               // �����жϽ���/����״̬
uint8_t status2;                // �����жϽ���/����״̬
uint8_t txbuf[32]={0,1,2,3,4,5,6};	   // ���ͻ���
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
    
	/* ϵͳʱ�ӳ�ʼ����400MHz */
	SystemClock_Config();
  
  SCB_EnableICache();    // ʹ��ָ�� Cache
  SCB_EnableDCache();    // ʹ������ Cache

	/* LED �˿ڳ�ʼ�� */
	LED_GPIO_Config();	
    
    /* ���ô���1Ϊ��115200 8-N-1 */
	UARTx_Config();

	/* ��ʼ��NRF2 */
	SPI_NRF2_Init();

  Key_GPIO_Config();
	printf("\r\n ����һ�� NRF24L01 ���ߴ���ʵ�� \r\n");
  printf("\r\n �������ߴ��� ������ �ķ�����Ϣ\r\n");
  printf("\r\n   ���ڼ��NRF��MCU�Ƿ��������ӡ�����\r\n");
  
	/*�շ����ݲ���*/
	Self_Test();	
		
}

/**
  * @brief  NRFģ����Ժ�����NRF1��NRF2֮��ѭ����������
  * @param  ��
  * @retval ��
  */
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
        printf("\r\n NRF2 ��������Ϊ��%d \r\n",rxbuf[i]); 
      }
      
      printf("\r\n�� K1 ʹ��NRF1 ��������\n"); 
      printf("\r\n�� K2 ʹ��NRF2 ��������\r\n"); 
    }
    
    /* NRF2 �������� */
    if (Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)    // ���� 2 ���£���ʼ������
    { 
			LED2_TOGGLE;
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
  *         system Clock ��������: 
	*            System Clock source  = PLL (HSE)
	*            SYSCLK(Hz)           = 400000000 (CPU Clock)
	*            HCLK(Hz)             = 200000000 (AXI and AHBs Clock)
	*            AHB Prescaler        = 2
	*            D1 APB3 Prescaler    = 2 (APB3 Clock  100MHz)
	*            D2 APB1 Prescaler    = 2 (APB1 Clock  100MHz)
	*            D2 APB2 Prescaler    = 2 (APB2 Clock  100MHz)
	*            D3 APB4 Prescaler    = 2 (APB4 Clock  100MHz)
	*            HSE Frequency(Hz)    = 25000000
	*            PLL_M                = 5
	*            PLL_N                = 160
	*            PLL_P                = 2
	*            PLL_Q                = 4
	*            PLL_R                = 2
	*            VDD(V)               = 3.3
	*            Flash Latency(WS)    = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;
  
  /*ʹ�ܹ������ø��� */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

  /* ��������ʱ��Ƶ�ʵ������ϵͳƵ��ʱ����ѹ���ڿ����Ż����ģ�
		 ����ϵͳƵ�ʵĵ�ѹ����ֵ�ĸ��¿��Բο���Ʒ�����ֲᡣ  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
 
  /* ����HSE������ʹ��HSE��ΪԴ����PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
 
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
//  if(ret != HAL_OK)
//  {

//    while(1) { ; }
//  }
  
	/* ѡ��PLL��Ϊϵͳʱ��Դ����������ʱ�ӷ�Ƶ�� */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK  | \
																 RCC_CLOCKTYPE_HCLK    | \
																 RCC_CLOCKTYPE_D1PCLK1 | \
																 RCC_CLOCKTYPE_PCLK1   | \
                                 RCC_CLOCKTYPE_PCLK2   | \
																 RCC_CLOCKTYPE_D3PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}
/****************************END OF FILE***************************/
