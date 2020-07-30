/**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   ����ģ��/nrf24l01+/�������
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��STMH743������  
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "./NRF24L01/bsp_nrf24l01.h"
#include "./delay/core_delay.h"   

SPI_HandleTypeDef NRF1_Handler;  
SPI_HandleTypeDef NRF2_Handler;  

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ

/**
  * @brief  SPI_NRF1�� I/O����
  * @param  ��
  * @retval ��
  */
void SPI_NRF1_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

	  /* ʹ����Ӧ�˿ڵ�ʱ�� */
    NRF1_CE_GPIO_CLK_ENABLE();
	  NRF1_CSN_GPIO_CLK_ENABLE();
    NRF1_CLK_GPIO_CLK_ENABLE();
    NRF1_MISO_GPIO_CLK_ENABLE();
	  NRF1_MOSI_GPIO_CLK_ENABLE();
	  NRF1_IRQ_GPIO_CLK_ENABLE();
	  /* ʹ��SPI2ʱ�� */
    NRF1_SPI_CLOCK();                  
    
    /* ��ʼ��CLK���ŵ����� */
    GPIO_Initure.Pin=NRF1_CLK_PIN;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              
    GPIO_Initure.Pull=GPIO_PULLUP;                 
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;           
    GPIO_Initure.Alternate=NRF1_GPIO_AF;          
    HAL_GPIO_Init(NRF1_CLK_GPIO_PORT,&GPIO_Initure);
	
	  /* ��ʼ��MISO���ŵ����� */
	  GPIO_Initure.Pin=NRF1_MISO_PIN;
	  HAL_GPIO_Init(NRF1_MISO_GPIO_PORT,&GPIO_Initure);
		
	  /* ��ʼ��MOSI���ŵ����� */
	  GPIO_Initure.Pin=NRF1_MOSI_PIN;
	  HAL_GPIO_Init(NRF1_MOSI_GPIO_PORT,&GPIO_Initure);
		
    /* ��ʼ��CSN���ŵ����� */
    GPIO_Initure.Pin=NRF1_CSN_PIN; 
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP; 
    GPIO_Initure.Pull=GPIO_PULLUP;         
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;   
    HAL_GPIO_Init(NRF1_CSN_GPIO_PORT,&GPIO_Initure);
		
		/* ��ʼ��CE���ŵ����� */
		GPIO_Initure.Pin=NRF1_CE_PIN;
	  HAL_GPIO_Init(NRF1_CE_GPIO_PORT,&GPIO_Initure);
		
    /* ��ʼ��IRQ���ŵ����� */		
    GPIO_Initure.Pin=NRF1_IRQ_PIN;          
    GPIO_Initure.Mode=GPIO_MODE_INPUT;    
    HAL_GPIO_Init(NRF1_IRQ_GPIO_PORT,&GPIO_Initure);     
    
    /* ��ʼ��SPI������ */
    NRF1_Handler.Instance=NRF1_SPI;                      //SPI2
    NRF1_Handler.Init.Mode=SPI_MODE_MASTER;          //����SPI����ģʽ������Ϊ��ģʽ
    NRF1_Handler.Init.Direction=SPI_DIRECTION_2LINES;//SPI����Ϊ˫��ģʽ
    NRF1_Handler.Init.DataSize=SPI_DATASIZE_8BIT;    //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    NRF1_Handler.Init.CLKPolarity=SPI_POLARITY_LOW;  //����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
    NRF1_Handler.Init.CLKPhase=SPI_PHASE_1EDGE;      //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    NRF1_Handler.Init.NSS=SPI_NSS_SOFT;              //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
		NRF1_Handler.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;//NSS�ź�����ʧ��
    NRF1_Handler.Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;  //SPI��ģʽIO״̬����ʹ��
    NRF1_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
    NRF1_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;     //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    NRF1_Handler.Init.CRCPolynomial=7;               //CRCֵ����Ķ���ʽ
    HAL_SPI_Init(&NRF1_Handler); 
    __HAL_SPI_ENABLE(&NRF1_Handler);                 //ʹ��SPI2
		
		NRF1_CE_0(); 			         
		/*�õ�CSN��ʹ��SPI����*/
		NRF1_CSN_1();			         
}


/**
  * @brief   ������NRF1��/дһ�ֽ�����
  * @param   д�������
  *	@arg     TxData 
  * @retval  ��ȡ�õ�����
  */
uint8_t SPI_NRF1_RW(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&NRF1_Handler,&TxData,&Rxdata,1, 1000);       
 	  return Rxdata;          		    //�����յ�������		
}


/**
  * @brief  ��Ҫ����NRF1��MCU�Ƿ���������
  * @param  ��
  * @retval SUCCESS/ERROR ��������/����ʧ��
  */
uint8_t NRF1_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t buf1[5];
	uint8_t i;	 
	SPI_NRF1_WriteBuf(NRF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	SPI_NRF1_ReadBuf(TX_ADDR,buf1,5); //����д��ĵ�ַ  
		printf("NRF1:%x\n",buf1[1]);
	for(i=0;i<5;i++)
	{
		if(buf1[i]!=0XA5)
		break;
	}		
	if(i!=5)return 0;//���24L01����	
	return 1;		     //��⵽24L01
}	


/**
  * @brief   ������NRF1�ض��ļĴ���д������
  * @param   
  *	@arg     reg:NRF������+�Ĵ�����ַ
  *	@arg     dat:��Ҫ��Ĵ���д�������
  * @retval  NRF��status�Ĵ�����״̬
  */
uint8_t SPI_NRF1_WriteReg(uint8_t reg,uint8_t value)
{
	  uint8_t status;	
		/*�õ�CSN��ʹ��SPI����*/
    NRF1_CSN_0();  
	
	  /*��������Ĵ����� */
  	status =SPI_NRF1_RW(reg);
	
    /*��Ĵ���д������*/
  	SPI_NRF1_RW(value);
	
	  /*CSN���ߣ����*/	   	
  	NRF1_CSN_1();  
	
    /*����״̬�Ĵ�����ֵ*/	
  	return(status);       	
}



/**
  * @brief   ���ڴ�NRF1�ض��ļĴ�����������
  * @param   
  *		@arg   reg:NRF������+�Ĵ�����ַ
  * @retval  �Ĵ����е�����
  */
uint8_t SPI_NRF1_ReadReg(uint8_t reg)
{
		uint8_t reg_val;	
	  /*�õ�CSN��ʹ��SPI����*/	
		NRF1_CSN_0();     
	
  	/*���ͼĴ����� */
  	SPI_NRF1_RW(reg);  
	
	  /*��ȡ�Ĵ�����ֵ */
  	reg_val=SPI_NRF1_RW(NOP);
	
	 /*CSN���ߣ����*/	
  	NRF1_CSN_1();   
	 
  	return(reg_val);    
}	


/**
  * @brief   ������NRF1�ļĴ�����д��һ������
  * @param   
  *		@arg   reg : NRF������+�Ĵ�����ַ
  *		@arg   pBuf�����ڴ洢���������ļĴ������ݵ����飬�ⲿ����
  * 	@arg   bytes: pBuf�����ݳ���
  * @retval  NRF��status�Ĵ�����״̬
  */
uint8_t SPI_NRF1_ReadBuf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	  uint8_t status,uint8_t_ctr;
    /*�õ�CSN��ʹ��SPI����*/		
  	NRF1_CSN_0();    

    /*���ͼĴ����� */	
  	status=SPI_NRF1_RW(reg);
    
    /*��ȡ����������*/ 	
 	  for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
	  pBuf[uint8_t_ctr]=SPI_NRF1_RW(NOP);
	
	  /*CSN���ߣ����*/
  	NRF1_CSN_1();  
	  
	  /*����״̬�Ĵ�����ֵ*/	
  	return status;             
}


/**
  * @brief    ������NRF1�ļĴ�����д��һ������
  * @param   
  *	@arg      reg : NRF������+�Ĵ�����ַ
  *	@arg      pBuf���洢�˽�Ҫд��д�Ĵ������ݵ����飬�ⲿ����
  * @arg      bytes: pBuf�����ݳ���
  * @retval   NRF��status�Ĵ�����״̬
  */
uint8_t SPI_NRF1_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	  uint8_t status,uint8_t_ctr;	 

    /*�õ�CSN��ʹ��SPI����*/			
 	  NRF1_CSN_0();      

    /*���ͼĴ����� */		
  	status = SPI_NRF1_RW(reg);
	
	  /*�򻺳���д������*/
  	for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
	  SPI_NRF1_RW(*pBuf++);
  
    /*CSN���ߣ����*/
	  NRF1_CSN_1();              
  	return status;             
}		
 


/**
  * @brief   ������NRF1�ķ��ͻ�������д������
  * @param   
  *		@arg txBuf���洢�˽�Ҫ���͵����ݵ����飬�ⲿ����	
  * @retval  ���ͽ�����ɹ�����TXDS,ʧ�ܷ���MAXRT��ERROR
  */
uint8_t NRF1_Tx_Dat(uint8_t *txbuf)
{
	uint8_t state;  

	 /*ceΪ�ͣ��������ģʽ1*/
	NRF1_CE_0();

	/*д���ݵ�TX BUF ��� 32���ֽ�*/						
   SPI_NRF1_WriteBuf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);

      /*CEΪ�ߣ�txbuf�ǿգ��������ݰ� */   
 	 NRF1_CE_1();
	  	
	  /*�ȴ���������ж� */                            
	while(NRF1_Read_IRQ()!=0); 	
	
	/*��ȡ״̬�Ĵ�����ֵ */                              
	state = SPI_NRF1_ReadReg(STATUS);

	 /*���TX_DS��MAX_RT�жϱ�־*/                  
	SPI_NRF1_WriteReg(NRF_WRITE_REG+STATUS,state); 	

	SPI_NRF1_WriteReg(FLUSH_TX,NOP);    //���TX FIFO�Ĵ��� 

	 /*�ж��ж�����*/    
	if(state&MAX_TX)                     //�ﵽ����ط�����
			 return MAX_TX; 

	else if(state&TX_DS)                  //�������
		 	return TX_DS;
	 else						  
			return ERROR;                 //����ԭ����ʧ��
} 

/**
  * @brief   ���ڴ�NRF1�Ľ��ջ������ж�������
  * @param   
  *		@arg rxBuf �����ڽ��ո����ݵ����飬�ⲿ����	
  * @retval 
  *		@arg ���ս��
  */
uint8_t NRF1_Rx_Dat(uint8_t *rxbuf)
{
	uint8_t state; 
	NRF1_CE_1();	 //�������״̬
	 /*�ȴ������ж�*/
	while(NRF1_Read_IRQ()==0)
  {
    NRF1_CE_0();  	 //�������״̬
    /*��ȡstatus�Ĵ�����ֵ  */               
    state=SPI_NRF1_ReadReg(STATUS);
     
    /* ����жϱ�־*/      
    SPI_NRF1_WriteReg(NRF_WRITE_REG+STATUS,state);

    /*�ж��Ƿ���յ�����*/
    if(state&RX_DR)                                 //���յ�����
    {
      SPI_NRF1_ReadBuf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
         SPI_NRF1_WriteReg(FLUSH_RX,NOP);          //���RX FIFO�Ĵ���
      return RX_DR; 
    }
    else    
      return ERROR;                    //û�յ��κ�����
  }
  
  return ERROR;                    //û�յ��κ�����
}



/**
  * @brief  NRF1���ò��������ģʽ
  * @param  ��
  * @retval ��
  */   
void NRF1_RX_Mode(void)
{
	  NRF1_CE_0();	  
  	SPI_NRF1_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	  
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+EN_AA,0x01);       //ʹ��ͨ��0���Զ�Ӧ��    
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01);   //ʹ��ͨ��0�Ľ��յ�ַ  	 
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);	        //����RFͨ��Ƶ��		  
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);    //����TX�������,0db����,2Mbps,���������濪��   
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);     //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	NRF1_CE_1(); //CEΪ��,�������ģʽ 
}		



/**
  * @brief  NRF1���÷���ģʽ
  * @param  ��
  * @retval ��
  */	 
void NRF1_TX_Mode(void)
{														 
	  NRF1_CE_0();	    
  	SPI_NRF1_WriteBuf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	SPI_NRF1_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  	SPI_NRF1_WriteReg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);       //����RFͨ��Ϊ40
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	  NRF1_CE_1();
	  CPU_TS_Tmr_Delay_US(130);                        //CEҪ����һ��ʱ��Ž��뷢��ģʽ
}



/**
  * @brief  SPI_NRF2�� I/O����
  * @param  ��
  * @retval ��
  */
void SPI_NRF2_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

	  /* ʹ��SPI2ʱ�� */
    NRF2_SPI_CLOCK();   	
	  /* ʹ����Ӧ�˿ڵ�ʱ�� */
    NRF2_CE_GPIO_CLK_ENABLE();
	  NRF2_CSN_GPIO_CLK_ENABLE();
    NRF2_CLK_GPIO_CLK_ENABLE();
    NRF2_MISO_GPIO_CLK_ENABLE();
	  NRF2_MOSI_GPIO_CLK_ENABLE();
	  NRF2_IRQ_GPIO_CLK_ENABLE();
               
    
    /* ��ʼ��CLK���ŵ����� */
    GPIO_Initure.Pin=NRF2_CLK_PIN;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              
    GPIO_Initure.Pull=GPIO_PULLUP;                 
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;           
    GPIO_Initure.Alternate=NRF2_GPIO_AF;          
    HAL_GPIO_Init(NRF2_CLK_GPIO_PORT,&GPIO_Initure);
	
	  /* ��ʼ��MISO���ŵ����� */
	  GPIO_Initure.Pin=NRF2_MISO_PIN;
	  HAL_GPIO_Init(NRF2_MISO_GPIO_PORT,&GPIO_Initure);
		
	  /* ��ʼ��MOSI���ŵ����� */
	  GPIO_Initure.Pin=NRF2_MOSI_PIN;
	  HAL_GPIO_Init(NRF2_MOSI_GPIO_PORT,&GPIO_Initure);
		
    /* ��ʼ��CSN���ŵ����� */
    GPIO_Initure.Pin=NRF2_CSN_PIN; 
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP; 
    GPIO_Initure.Pull=GPIO_PULLUP;         
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;   
    HAL_GPIO_Init(NRF2_CSN_GPIO_PORT,&GPIO_Initure);
		
		/* ��ʼ��CE���ŵ����� */
		GPIO_Initure.Pin=NRF2_CE_PIN;
	  HAL_GPIO_Init(NRF2_CE_GPIO_PORT,&GPIO_Initure);
		
    /* ��ʼ��IRQ���ŵ����� */ 
    GPIO_Initure.Pin=NRF2_IRQ_PIN;  		
    GPIO_Initure.Mode=GPIO_MODE_INPUT;    
    HAL_GPIO_Init(NRF2_IRQ_GPIO_PORT,&GPIO_Initure);     
    
    /* ��ʼ��SPI������ */
    NRF2_Handler.Instance=NRF2_SPI;                      //SPI1
    NRF2_Handler.Init.Mode=SPI_MODE_MASTER;          //����SPI����ģʽ������Ϊ��ģʽ
    NRF2_Handler.Init.Direction=SPI_DIRECTION_2LINES;//SPI����Ϊ˫��ģʽ
    NRF2_Handler.Init.DataSize=SPI_DATASIZE_8BIT;    //����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
    NRF2_Handler.Init.CLKPolarity=SPI_POLARITY_LOW;  //����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
    NRF2_Handler.Init.CLKPhase=SPI_PHASE_1EDGE;      //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    NRF2_Handler.Init.NSS=SPI_NSS_SOFT;              //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
		NRF2_Handler.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;//NSS�ź�����ʧ��
    NRF2_Handler.Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;  //SPI��ģʽIO״̬����ʹ��
    NRF2_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ16
    NRF2_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;     //ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    NRF2_Handler.Init.CRCPolynomial=7;               //CRCֵ����Ķ���ʽ
    HAL_SPI_Init(&NRF2_Handler); 
    __HAL_SPI_ENABLE(&NRF2_Handler);                 //ʹ��SPI1
		
		NRF2_CE_0(); 			         
		/*�õ�CSN��ʹ��SPI����*/
		NRF2_CSN_1();			         
}


/**
  * @brief   ������NRF2��/дһ�ֽ�����
  * @param   д�������
  *	@arg     TxData 
  * @retval  ��ȡ�õ�����
  */
uint8_t SPI_NRF2_RW(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&NRF2_Handler,&TxData,&Rxdata,1, 1000);       
 	  return Rxdata;          		    //�����յ�������		
}


/**
  * @brief  ��Ҫ����NRF2��MCU�Ƿ���������
  * @param  ��
  * @retval SUCCESS/ERROR ��������/����ʧ��
  */
uint8_t NRF2_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t buf1[5];
	uint8_t i;	 
	SPI_NRF2_WriteBuf(NRF_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	SPI_NRF2_ReadBuf(TX_ADDR,buf1,5); //����д��ĵ�ַ  
	printf("NRF2:%x\n",buf1[1]);
	for(i=0;i<5;i++)
	{
		if(buf1[i]!=0XA5)
		break;
	}		
	if(i!=5)return 0;//���24L01����	
	return 1;		     //��⵽24L01
}	


/**
  * @brief   ������NRF2�ض��ļĴ���д������
  * @param   
  *	@arg     reg:NRF������+�Ĵ�����ַ
  *	@arg     dat:��Ҫ��Ĵ���д�������
  * @retval  NRF��status�Ĵ�����״̬
  */
uint8_t SPI_NRF2_WriteReg(uint8_t reg,uint8_t value)
{
	  uint8_t status;	
		/*�õ�CSN��ʹ��SPI����*/
    NRF2_CSN_0();  
	
	  /*��������Ĵ����� */
  	status =SPI_NRF2_RW(reg);
	
    /*��Ĵ���д������*/
  	SPI_NRF2_RW(value);
	
	  /*CSN���ߣ����*/	   	
  	NRF2_CSN_1();  
	
    /*����״̬�Ĵ�����ֵ*/	
  	return(status);       	
}



/**
  * @brief   ���ڴ�NRF2�ض��ļĴ�����������
  * @param   
  *		@arg   reg:NRF������+�Ĵ�����ַ
  * @retval  �Ĵ����е�����
  */
uint8_t SPI_NRF2_ReadReg(uint8_t reg)
{
		uint8_t reg_val;	
	  /*�õ�CSN��ʹ��SPI����*/	
		NRF2_CSN_0();     
	
  	/*���ͼĴ����� */
  	SPI_NRF2_RW(reg);  
	
	  /*��ȡ�Ĵ�����ֵ */
  	reg_val=SPI_NRF2_RW(NOP);
	
	 /*CSN���ߣ����*/	
  	NRF2_CSN_1();   
	 
  	return(reg_val);    
}	


/**
  * @brief   ������NRF2�ļĴ�����д��һ������
  * @param   
  *		@arg   reg : NRF������+�Ĵ�����ַ
  *		@arg   pBuf�����ڴ洢���������ļĴ������ݵ����飬�ⲿ����
  * 	@arg   bytes: pBuf�����ݳ���
  * @retval  NRF��status�Ĵ�����״̬
  */
uint8_t SPI_NRF2_ReadBuf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	  uint8_t status,uint8_t_ctr;
    /*�õ�CSN��ʹ��SPI����*/		
  	NRF2_CSN_0();    

    /*���ͼĴ����� */	
  	status=SPI_NRF2_RW(reg);
    
    /*��ȡ����������*/ 	
 	  for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
	  pBuf[uint8_t_ctr]=SPI_NRF2_RW(NOP);
	
	  /*CSN���ߣ����*/
  	NRF2_CSN_1();  
	  
	  /*����״̬�Ĵ�����ֵ*/	
  	return status;             
}


/**
  * @brief    ������NRF2�ļĴ�����д��һ������
  * @param   
  *	@arg      reg : NRF������+�Ĵ�����ַ
  *	@arg      pBuf���洢�˽�Ҫд��д�Ĵ������ݵ����飬�ⲿ����
  * @arg      bytes: pBuf�����ݳ���
  * @retval   NRF��status�Ĵ�����״̬
  */
uint8_t SPI_NRF2_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	  uint8_t status,uint8_t_ctr;	 

    /*�õ�CSN��ʹ��SPI����*/			
 	  NRF2_CSN_0();      

    /*���ͼĴ����� */		
  	status = SPI_NRF2_RW(reg);
	
	  /*�򻺳���д������*/
  	for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
	  SPI_NRF2_RW(*pBuf++);
  
    /*CSN���ߣ����*/
	  NRF2_CSN_1();              
  	return status;             
}		
 


/**
  * @brief   ������NRF2�ķ��ͻ�������д������
  * @param   
  *		@arg txBuf���洢�˽�Ҫ���͵����ݵ����飬�ⲿ����	
  * @retval  ���ͽ�����ɹ�����TXDS,ʧ�ܷ���MAXRT��ERROR
  */
uint8_t NRF2_Tx_Dat(uint8_t *txbuf)
{
	uint8_t state;  

	 /*ceΪ�ͣ��������ģʽ1*/
	NRF2_CE_0();

	/*д���ݵ�TX BUF ��� 32���ֽ�*/						
   SPI_NRF2_WriteBuf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);

      /*CEΪ�ߣ�txbuf�ǿգ��������ݰ� */   
 	 NRF2_CE_1();
	  	
	  /*�ȴ���������ж� */                            
	while(NRF2_Read_IRQ()!=0); 	
	
	/*��ȡ״̬�Ĵ�����ֵ */                              
	state = SPI_NRF2_ReadReg(STATUS);

	 /*���TX_DS��MAX_RT�жϱ�־*/                  
	SPI_NRF2_WriteReg(NRF_WRITE_REG+STATUS,state); 	

	SPI_NRF2_WriteReg(FLUSH_TX,NOP);    //���TX FIFO�Ĵ��� 

	 /*�ж��ж�����*/    
	if(state&MAX_TX)                     //�ﵽ����ط�����
			 return MAX_TX; 

	else if(state&TX_DS)                  //�������
		 	return TX_DS;
	 else						  
			return ERROR;                 //����ԭ����ʧ��
} 

/**
  * @brief   ���ڴ�NRF2�Ľ��ջ������ж�������
  * @param   
  *	@arg     rxBuf �����ڽ��ո����ݵ����飬�ⲿ����	
  * @retval 
  *	@arg    ���ս��
  */
uint8_t NRF2_Rx_Dat(uint8_t *rxbuf)
{
	uint8_t state; 
	NRF2_CE_1();	 //�������״̬
	 /*�ȴ������ж�*/
	while(NRF2_Read_IRQ()==0)
  {
    NRF2_CE_0();  	 //�������״̬
    /*��ȡstatus�Ĵ�����ֵ  */               
    state=SPI_NRF2_ReadReg(STATUS);
     
    /* ����жϱ�־*/      
    SPI_NRF2_WriteReg(NRF_WRITE_REG+STATUS,state);

    /*�ж��Ƿ���յ�����*/
    if(state&RX_DR)                                 //���յ�����
    {
      SPI_NRF2_ReadBuf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
         SPI_NRF2_WriteReg(FLUSH_RX,NOP);          //���RX FIFO�Ĵ���
      return RX_DR; 
    }
    else    
      return ERROR;                    //û�յ��κ�����
  }
  
  return ERROR;                    //û�յ��κ�����
}



/**
  * @brief  NRF2���ò��������ģʽ
  * @param  ��
  * @retval ��
  */   
void NRF2_RX_Mode(void)
{
	  NRF2_CE_0();	  
  	SPI_NRF2_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	  
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+EN_AA,0x01);       //ʹ��ͨ��0���Զ�Ӧ��    
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01);   //ʹ��ͨ��0�Ľ��յ�ַ  	 
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);	        //����RFͨ��Ƶ��		  
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);    //����TX�������,0db����,2Mbps,���������濪��   
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);     //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	NRF2_CE_1(); //CEΪ��,�������ģʽ 
}		



/**
  * @brief  NRF2���÷���ģʽ
  * @param  ��
  * @retval ��
  */	 
void NRF2_TX_Mode(void)
{														 
	  NRF2_CE_0();	    
  	SPI_NRF2_WriteBuf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	SPI_NRF2_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  	SPI_NRF2_WriteReg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);       //����RFͨ��Ϊ40
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	  NRF2_CE_1();
	  CPU_TS_Tmr_Delay_US(130);                        //CEҪ����һ��ʱ��Ž��뷢��ģʽ
}


