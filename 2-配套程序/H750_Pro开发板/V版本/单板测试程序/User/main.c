/**
  ******************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ����ģ��/nrf24l01+/�������
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
    
    /* ϵͳʱ�ӳ�ʼ����480MHz */
    SystemClock_Config();

    SCB_EnableICache();    // ʹ��ָ�� Cache
    SCB_EnableDCache();    // ʹ������ Cache

    /* LED �˿ڳ�ʼ�� */
    LED_GPIO_Config();	

    /* ���ô���1Ϊ��115200 8-N-1 */
    UARTx_Config();

        /* ��ʼ��NRF1 */
    SPI_NRF1_Init();

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
  /*��� NRF ģ���� MCU ������*/
  status = NRF1_Check(); 

  /*�ж�����״̬*/  
  if(status == SUCCESS)	   
    printf("\r\n      NRF1��MCU���ӳɹ��� �� K1 ��������\r\n");  
  else	  
    printf("\r\n  NRF1��MCU����ʧ�ܣ������¼����ߡ�\r\n");
	
	
	/*��� NRF2 ģ���� MCU ������*/
  status = NRF2_Check();
	
	 /*�ж�����״̬*/  
  if(status == SUCCESS)	   
    printf("\r\n      NRF2��MCU���ӳɹ��� �� K2 ��������\r\n");  
  else	  
    printf("\r\n  NRF2��MCU����ʧ�ܣ������¼����ߡ�\r\n");

  NRF1_RX_Mode();     // NRF1 �������ģʽ
  NRF2_RX_Mode();    // NRF2 �������ģʽ

  while(1)
  {
    /* �ȴ� NRF1 �������� */
    status = NRF1_Rx_Dat(rxbuf);

    /* �жϽ���״̬ */
    if(status == RX_DR)
    {
      for(i=0;i<32;i++)
      {	
        printf("\r\n NRF1 ��������Ϊ��%d \r\n",rxbuf[i]); 
      }
      
      printf("\r\n�� K1 ʹ��NRF1 ��������\n"); 
      printf("\r\n�� K2 ʹ��NRF2 ��������\r\n"); 
    }
    
    /* NRF1 �������� */
    if (Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)    // �������£���ʼ������
    { 
			LED1_TOGGLE;
      /* �������� */
      NRF1_TX_Mode();
           
      status = NRF1_Tx_Dat(txbuf);
      
      /* �������ݵ�״̬ */
       if(status == TX_DS)
      {
        printf("\r\nNRF1 �������ݳɹ�\r\n");
      }
      else
      {
        printf("\r\nNRF1 ��������ʧ��  %d\r\n", status);
      }
      
      printf("\r\nNRF1 �������ģʽ\r\n"); 

      NRF1_RX_Mode();
    }
    
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
	*            SYSCLK(Hz)           = 480000000 (CPU Clock)
	*            HCLK(Hz)             = 240000000 (AXI and AHBs Clock)
	*            AHB Prescaler        = 2
	*            D1 APB3 Prescaler    = 2 (APB3 Clock  120MHz)
	*            D2 APB1 Prescaler    = 2 (APB1 Clock  120MHz)
	*            D2 APB2 Prescaler    = 2 (APB2 Clock  120MHz)
	*            D3 APB4 Prescaler    = 2 (APB4 Clock  120MHz)
	*            HSE Frequency(Hz)    = 25000000
	*            PLL_M                = 5
	*            PLL_N                = 192
	*            PLL_P                = 2
	*            PLL_Q                = 4
	*            PLL_R                = 2
	*            VDD(V)               = 3.3
	*            Flash Latency(WS)    = 4
  * @param  None
  * @retval None
  */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** ���õ�Դ���ø���
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** ����������ѹ�������ѹ
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** ��ʼ��CPU��AHB��APB����ʱ��
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
		while(1);
  }
  /** ��ʼ��CPU��AHB��APB����ʱ��
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
		while(1);
  }
}
/****************************END OF FILE***************************/
