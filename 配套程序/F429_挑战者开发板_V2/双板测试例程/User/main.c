/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   2.4g����ģ��/nrf24l01+/���� ����  ֻʹ��һ�鿪���壬
	*					 ��2��NRFģ������շ�����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 F429 ��ս�߿�����
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "stm32f4xx.h"
#include "./usart/bsp_usart.h"
#include "./nrf/bsp_spi_nrf.h"
#include "./key/bsp_key.h"

u8 status;	               // �����жϽ���/����״̬
u8 txbuf[32]={0,1,2,3};	   // ���ͻ���
u8 rxbuf[32];			         // ���ջ���
int i=0;

void Self_Test(void);

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)                  
{   
	/* ��ʼ��NRF1 */
  SPI_NRF_Init();

  /* ����1��ʼ�� */
  USARTx_Config();
  
  /* ������ʼ�� */
  Key_GPIO_Config();

  printf("\r\n ����һ�� NRF24L01 ���ߴ���ʵ�� \r\n");
  printf("\r\n �������ߴ��� ������ �ķ�����Ϣ\r\n");
  printf("\r\n   ���ڼ��NRF��MCU�Ƿ��������ӡ�����\r\n");

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
  status = NRF_Check(); 

  /*�ж�����״̬*/  
  if(status == SUCCESS)	   
    printf("\r\n      NRF��MCU���ӳɹ��� �� K1 ��������\r\n");  
  else	  
    printf("\r\n  NRF��MCU����ʧ�ܣ������¼����ߡ�\r\n");

  NRF_RX_Mode();     // NRF1 �������ģʽ

  while(1)
  {
    /* �ȴ� NRF1 �������� */
    status = NRF_Rx_Dat(rxbuf);

    /* �жϽ���״̬ */
    if(status == RX_DR)
    {
      for(i=0;i<32;i++)
      {	
        printf("\r\n NRF ��������Ϊ��%d \r\n",rxbuf[i]); 
      }
      
      printf("\r\n�� K1 ʹ��NRF ��������\n");  
    }
    
    /* NRF1 �������� */
    if (Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)    // �������£���ʼ������
    { 
      /* �������� */
      NRF_TX_Mode();
           
      status = NRF_Tx_Dat(txbuf);
      
      /* �������ݵ�״̬ */
       if(status == TX_DS)
      {
        printf("\r\nNRF �������ݳɹ�\r\n");
      }
      else
      {
        printf("\r\nNRF ��������ʧ��  %d\r\n", status);
      }
      
      printf("\r\nNRF �������ģʽ\r\n"); 

      NRF_RX_Mode();
    }
  }
}



/*********************************************END OF FILE**********************/

