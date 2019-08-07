/**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   无线模块/nrf24l01+/单板测试
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火STMH743开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "./NRF24L01/bsp_nrf24l01.h"
#include "./delay/core_delay.h"   

SPI_HandleTypeDef NRF1_Handler;  
SPI_HandleTypeDef NRF2_Handler;  

const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址

/**
  * @brief  SPI_NRF1的 I/O配置
  * @param  无
  * @retval 无
  */
void SPI_NRF1_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

	  /* 使能相应端口的时钟 */
    NRF1_CE_GPIO_CLK_ENABLE();
	  NRF1_CSN_GPIO_CLK_ENABLE();
    NRF1_CLK_GPIO_CLK_ENABLE();
    NRF1_MISO_GPIO_CLK_ENABLE();
	  NRF1_MOSI_GPIO_CLK_ENABLE();
	  NRF1_IRQ_GPIO_CLK_ENABLE();
	  /* 使能SPI2时钟 */
    NRF1_SPI_CLOCK();                  
    
    /* 初始化CLK引脚的配置 */
    GPIO_Initure.Pin=NRF1_CLK_PIN;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              
    GPIO_Initure.Pull=GPIO_PULLUP;                 
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;           
    GPIO_Initure.Alternate=NRF1_GPIO_AF;          
    HAL_GPIO_Init(NRF1_CLK_GPIO_PORT,&GPIO_Initure);
	
	  /* 初始化MISO引脚的配置 */
	  GPIO_Initure.Pin=NRF1_MISO_PIN;
	  HAL_GPIO_Init(NRF1_MISO_GPIO_PORT,&GPIO_Initure);
		
	  /* 初始化MOSI引脚的配置 */
	  GPIO_Initure.Pin=NRF1_MOSI_PIN;
	  HAL_GPIO_Init(NRF1_MOSI_GPIO_PORT,&GPIO_Initure);
		
    /* 初始化CSN引脚的配置 */
    GPIO_Initure.Pin=NRF1_CSN_PIN; 
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP; 
    GPIO_Initure.Pull=GPIO_PULLUP;         
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;   
    HAL_GPIO_Init(NRF1_CSN_GPIO_PORT,&GPIO_Initure);
		
		/* 初始化CE引脚的配置 */
		GPIO_Initure.Pin=NRF1_CE_PIN;
	  HAL_GPIO_Init(NRF1_CE_GPIO_PORT,&GPIO_Initure);
		
    /* 初始化IRQ引脚的配置 */		
    GPIO_Initure.Pin=NRF1_IRQ_PIN;          
    GPIO_Initure.Mode=GPIO_MODE_INPUT;    
    HAL_GPIO_Init(NRF1_IRQ_GPIO_PORT,&GPIO_Initure);     
    
    /* 初始化SPI的配置 */
    NRF1_Handler.Instance=NRF1_SPI;                      //SPI2
    NRF1_Handler.Init.Mode=SPI_MODE_MASTER;          //设置SPI工作模式，设置为主模式
    NRF1_Handler.Init.Direction=SPI_DIRECTION_2LINES;//SPI设置为双线模式
    NRF1_Handler.Init.DataSize=SPI_DATASIZE_8BIT;    //设置SPI的数据大小:SPI发送接收8位帧结构
    NRF1_Handler.Init.CLKPolarity=SPI_POLARITY_LOW;  //串行同步时钟的空闲状态为低电平
    NRF1_Handler.Init.CLKPhase=SPI_PHASE_1EDGE;      //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    NRF1_Handler.Init.NSS=SPI_NSS_SOFT;              //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
		NRF1_Handler.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;//NSS信号脉冲失能
    NRF1_Handler.Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;  //SPI主模式IO状态保持使能
    NRF1_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16;//定义波特率预分频的值:波特率预分频值为16
    NRF1_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;     //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    NRF1_Handler.Init.CRCPolynomial=7;               //CRC值计算的多项式
    HAL_SPI_Init(&NRF1_Handler); 
    __HAL_SPI_ENABLE(&NRF1_Handler);                 //使能SPI2
		
		NRF1_CE_0(); 			         
		/*置低CSN，使能SPI传输*/
		NRF1_CSN_1();			         
}


/**
  * @brief   用于向NRF1读/写一字节数据
  * @param   写入的数据
  *	@arg     TxData 
  * @retval  读取得的数据
  */
uint8_t SPI_NRF1_RW(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&NRF1_Handler,&TxData,&Rxdata,1, 1000);       
 	  return Rxdata;          		    //返回收到的数据		
}


/**
  * @brief  主要用于NRF1与MCU是否正常连接
  * @param  无
  * @retval SUCCESS/ERROR 连接正常/连接失败
  */
uint8_t NRF1_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t buf1[5];
	uint8_t i;	 
	SPI_NRF1_WriteBuf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	SPI_NRF1_ReadBuf(TX_ADDR,buf1,5); //读出写入的地址  
		printf("NRF1:%x\n",buf1[1]);
	for(i=0;i<5;i++)
	{
		if(buf1[i]!=0XA5)
		break;
	}		
	if(i!=5)return 0;//检测24L01错误	
	return 1;		     //检测到24L01
}	


/**
  * @brief   用于向NRF1特定的寄存器写入数据
  * @param   
  *	@arg     reg:NRF的命令+寄存器地址
  *	@arg     dat:将要向寄存器写入的数据
  * @retval  NRF的status寄存器的状态
  */
uint8_t SPI_NRF1_WriteReg(uint8_t reg,uint8_t value)
{
	  uint8_t status;	
		/*置低CSN，使能SPI传输*/
    NRF1_CSN_0();  
	
	  /*发送命令及寄存器号 */
  	status =SPI_NRF1_RW(reg);
	
    /*向寄存器写入数据*/
  	SPI_NRF1_RW(value);
	
	  /*CSN拉高，完成*/	   	
  	NRF1_CSN_1();  
	
    /*返回状态寄存器的值*/	
  	return(status);       	
}



/**
  * @brief   用于从NRF1特定的寄存器读出数据
  * @param   
  *		@arg   reg:NRF的命令+寄存器地址
  * @retval  寄存器中的数据
  */
uint8_t SPI_NRF1_ReadReg(uint8_t reg)
{
		uint8_t reg_val;	
	  /*置低CSN，使能SPI传输*/	
		NRF1_CSN_0();     
	
  	/*发送寄存器号 */
  	SPI_NRF1_RW(reg);  
	
	  /*读取寄存器的值 */
  	reg_val=SPI_NRF1_RW(NOP);
	
	 /*CSN拉高，完成*/	
  	NRF1_CSN_1();   
	 
  	return(reg_val);    
}	


/**
  * @brief   用于向NRF1的寄存器中写入一串数据
  * @param   
  *		@arg   reg : NRF的命令+寄存器地址
  *		@arg   pBuf：用于存储将被读出的寄存器数据的数组，外部定义
  * 	@arg   bytes: pBuf的数据长度
  * @retval  NRF的status寄存器的状态
  */
uint8_t SPI_NRF1_ReadBuf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	  uint8_t status,uint8_t_ctr;
    /*置低CSN，使能SPI传输*/		
  	NRF1_CSN_0();    

    /*发送寄存器号 */	
  	status=SPI_NRF1_RW(reg);
    
    /*读取缓冲区数据*/ 	
 	  for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
	  pBuf[uint8_t_ctr]=SPI_NRF1_RW(NOP);
	
	  /*CSN拉高，完成*/
  	NRF1_CSN_1();  
	  
	  /*返回状态寄存器的值*/	
  	return status;             
}


/**
  * @brief    用于向NRF1的寄存器中写入一串数据
  * @param   
  *	@arg      reg : NRF的命令+寄存器地址
  *	@arg      pBuf：存储了将要写入写寄存器数据的数组，外部定义
  * @arg      bytes: pBuf的数据长度
  * @retval   NRF的status寄存器的状态
  */
uint8_t SPI_NRF1_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	  uint8_t status,uint8_t_ctr;	 

    /*置低CSN，使能SPI传输*/			
 	  NRF1_CSN_0();      

    /*发送寄存器号 */		
  	status = SPI_NRF1_RW(reg);
	
	  /*向缓冲区写入数据*/
  	for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
	  SPI_NRF1_RW(*pBuf++);
  
    /*CSN拉高，完成*/
	  NRF1_CSN_1();              
  	return status;             
}		
 


/**
  * @brief   用于向NRF1的发送缓冲区中写入数据
  * @param   
  *		@arg txBuf：存储了将要发送的数据的数组，外部定义	
  * @retval  发送结果，成功返回TXDS,失败返回MAXRT或ERROR
  */
uint8_t NRF1_Tx_Dat(uint8_t *txbuf)
{
	uint8_t state;  

	 /*ce为低，进入待机模式1*/
	NRF1_CE_0();

	/*写数据到TX BUF 最大 32个字节*/						
   SPI_NRF1_WriteBuf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);

      /*CE为高，txbuf非空，发送数据包 */   
 	 NRF1_CE_1();
	  	
	  /*等待发送完成中断 */                            
	while(NRF1_Read_IRQ()!=0); 	
	
	/*读取状态寄存器的值 */                              
	state = SPI_NRF1_ReadReg(STATUS);

	 /*清除TX_DS或MAX_RT中断标志*/                  
	SPI_NRF1_WriteReg(NRF_WRITE_REG+STATUS,state); 	

	SPI_NRF1_WriteReg(FLUSH_TX,NOP);    //清除TX FIFO寄存器 

	 /*判断中断类型*/    
	if(state&MAX_TX)                     //达到最大重发次数
			 return MAX_TX; 

	else if(state&TX_DS)                  //发送完成
		 	return TX_DS;
	 else						  
			return ERROR;                 //其他原因发送失败
} 

/**
  * @brief   用于从NRF1的接收缓冲区中读出数据
  * @param   
  *		@arg rxBuf ：用于接收该数据的数组，外部定义	
  * @retval 
  *		@arg 接收结果
  */
uint8_t NRF1_Rx_Dat(uint8_t *rxbuf)
{
	uint8_t state; 
	NRF1_CE_1();	 //进入接收状态
	 /*等待接收中断*/
	while(NRF1_Read_IRQ()==0)
  {
    NRF1_CE_0();  	 //进入待机状态
    /*读取status寄存器的值  */               
    state=SPI_NRF1_ReadReg(STATUS);
     
    /* 清除中断标志*/      
    SPI_NRF1_WriteReg(NRF_WRITE_REG+STATUS,state);

    /*判断是否接收到数据*/
    if(state&RX_DR)                                 //接收到数据
    {
      SPI_NRF1_ReadBuf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
         SPI_NRF1_WriteReg(FLUSH_RX,NOP);          //清除RX FIFO寄存器
      return RX_DR; 
    }
    else    
      return ERROR;                    //没收到任何数据
  }
  
  return ERROR;                    //没收到任何数据
}



/**
  * @brief  NRF1配置并进入接收模式
  * @param  无
  * @retval 无
  */   
void NRF1_RX_Mode(void)
{
	  NRF1_CE_0();	  
  	SPI_NRF1_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	  
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+EN_AA,0x01);       //使能通道0的自动应答    
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01);   //使能通道0的接收地址  	 
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);	        //设置RF通信频率		  
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);    //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);     //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	NRF1_CE_1(); //CE为高,进入接收模式 
}		



/**
  * @brief  NRF1配置发送模式
  * @param  无
  * @retval 无
  */	 
void NRF1_TX_Mode(void)
{														 
	  NRF1_CE_0();	    
  	SPI_NRF1_WriteBuf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	SPI_NRF1_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  	SPI_NRF1_WriteReg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);       //设置RF通道为40
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	SPI_NRF1_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	  NRF1_CE_1();
	  CPU_TS_Tmr_Delay_US(130);                        //CE要拉高一段时间才进入发送模式
}



/**
  * @brief  SPI_NRF2的 I/O配置
  * @param  无
  * @retval 无
  */
void SPI_NRF2_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

	  /* 使能SPI2时钟 */
    NRF2_SPI_CLOCK();   	
	  /* 使能相应端口的时钟 */
    NRF2_CE_GPIO_CLK_ENABLE();
	  NRF2_CSN_GPIO_CLK_ENABLE();
    NRF2_CLK_GPIO_CLK_ENABLE();
    NRF2_MISO_GPIO_CLK_ENABLE();
	  NRF2_MOSI_GPIO_CLK_ENABLE();
	  NRF2_IRQ_GPIO_CLK_ENABLE();
               
    
    /* 初始化CLK引脚的配置 */
    GPIO_Initure.Pin=NRF2_CLK_PIN;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              
    GPIO_Initure.Pull=GPIO_PULLUP;                 
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;           
    GPIO_Initure.Alternate=NRF2_GPIO_AF;          
    HAL_GPIO_Init(NRF2_CLK_GPIO_PORT,&GPIO_Initure);
	
	  /* 初始化MISO引脚的配置 */
	  GPIO_Initure.Pin=NRF2_MISO_PIN;
	  HAL_GPIO_Init(NRF2_MISO_GPIO_PORT,&GPIO_Initure);
		
	  /* 初始化MOSI引脚的配置 */
	  GPIO_Initure.Pin=NRF2_MOSI_PIN;
	  HAL_GPIO_Init(NRF2_MOSI_GPIO_PORT,&GPIO_Initure);
		
    /* 初始化CSN引脚的配置 */
    GPIO_Initure.Pin=NRF2_CSN_PIN; 
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP; 
    GPIO_Initure.Pull=GPIO_PULLUP;         
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;   
    HAL_GPIO_Init(NRF2_CSN_GPIO_PORT,&GPIO_Initure);
		
		/* 初始化CE引脚的配置 */
		GPIO_Initure.Pin=NRF2_CE_PIN;
	  HAL_GPIO_Init(NRF2_CE_GPIO_PORT,&GPIO_Initure);
		
    /* 初始化IRQ引脚的配置 */ 
    GPIO_Initure.Pin=NRF2_IRQ_PIN;  		
    GPIO_Initure.Mode=GPIO_MODE_INPUT;    
    HAL_GPIO_Init(NRF2_IRQ_GPIO_PORT,&GPIO_Initure);     
    
    /* 初始化SPI的配置 */
    NRF2_Handler.Instance=NRF2_SPI;                      //SPI1
    NRF2_Handler.Init.Mode=SPI_MODE_MASTER;          //设置SPI工作模式，设置为主模式
    NRF2_Handler.Init.Direction=SPI_DIRECTION_2LINES;//SPI设置为双线模式
    NRF2_Handler.Init.DataSize=SPI_DATASIZE_8BIT;    //设置SPI的数据大小:SPI发送接收8位帧结构
    NRF2_Handler.Init.CLKPolarity=SPI_POLARITY_LOW;  //串行同步时钟的空闲状态为低电平
    NRF2_Handler.Init.CLKPhase=SPI_PHASE_1EDGE;      //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    NRF2_Handler.Init.NSS=SPI_NSS_SOFT;              //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
		NRF2_Handler.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;//NSS信号脉冲失能
    NRF2_Handler.Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;  //SPI主模式IO状态保持使能
    NRF2_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16;//定义波特率预分频的值:波特率预分频值为16
    NRF2_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;     //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    NRF2_Handler.Init.CRCPolynomial=7;               //CRC值计算的多项式
    HAL_SPI_Init(&NRF2_Handler); 
    __HAL_SPI_ENABLE(&NRF2_Handler);                 //使能SPI1
		
		NRF2_CE_0(); 			         
		/*置低CSN，使能SPI传输*/
		NRF2_CSN_1();			         
}


/**
  * @brief   用于向NRF2读/写一字节数据
  * @param   写入的数据
  *	@arg     TxData 
  * @retval  读取得的数据
  */
uint8_t SPI_NRF2_RW(uint8_t TxData)
{
    uint8_t Rxdata;
    HAL_SPI_TransmitReceive(&NRF2_Handler,&TxData,&Rxdata,1, 1000);       
 	  return Rxdata;          		    //返回收到的数据		
}


/**
  * @brief  主要用于NRF2与MCU是否正常连接
  * @param  无
  * @retval SUCCESS/ERROR 连接正常/连接失败
  */
uint8_t NRF2_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t buf1[5];
	uint8_t i;	 
	SPI_NRF2_WriteBuf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	SPI_NRF2_ReadBuf(TX_ADDR,buf1,5); //读出写入的地址  
	printf("NRF2:%x\n",buf1[1]);
	for(i=0;i<5;i++)
	{
		if(buf1[i]!=0XA5)
		break;
	}		
	if(i!=5)return 0;//检测24L01错误	
	return 1;		     //检测到24L01
}	


/**
  * @brief   用于向NRF2特定的寄存器写入数据
  * @param   
  *	@arg     reg:NRF的命令+寄存器地址
  *	@arg     dat:将要向寄存器写入的数据
  * @retval  NRF的status寄存器的状态
  */
uint8_t SPI_NRF2_WriteReg(uint8_t reg,uint8_t value)
{
	  uint8_t status;	
		/*置低CSN，使能SPI传输*/
    NRF2_CSN_0();  
	
	  /*发送命令及寄存器号 */
  	status =SPI_NRF2_RW(reg);
	
    /*向寄存器写入数据*/
  	SPI_NRF2_RW(value);
	
	  /*CSN拉高，完成*/	   	
  	NRF2_CSN_1();  
	
    /*返回状态寄存器的值*/	
  	return(status);       	
}



/**
  * @brief   用于从NRF2特定的寄存器读出数据
  * @param   
  *		@arg   reg:NRF的命令+寄存器地址
  * @retval  寄存器中的数据
  */
uint8_t SPI_NRF2_ReadReg(uint8_t reg)
{
		uint8_t reg_val;	
	  /*置低CSN，使能SPI传输*/	
		NRF2_CSN_0();     
	
  	/*发送寄存器号 */
  	SPI_NRF2_RW(reg);  
	
	  /*读取寄存器的值 */
  	reg_val=SPI_NRF2_RW(NOP);
	
	 /*CSN拉高，完成*/	
  	NRF2_CSN_1();   
	 
  	return(reg_val);    
}	


/**
  * @brief   用于向NRF2的寄存器中写入一串数据
  * @param   
  *		@arg   reg : NRF的命令+寄存器地址
  *		@arg   pBuf：用于存储将被读出的寄存器数据的数组，外部定义
  * 	@arg   bytes: pBuf的数据长度
  * @retval  NRF的status寄存器的状态
  */
uint8_t SPI_NRF2_ReadBuf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	  uint8_t status,uint8_t_ctr;
    /*置低CSN，使能SPI传输*/		
  	NRF2_CSN_0();    

    /*发送寄存器号 */	
  	status=SPI_NRF2_RW(reg);
    
    /*读取缓冲区数据*/ 	
 	  for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
	  pBuf[uint8_t_ctr]=SPI_NRF2_RW(NOP);
	
	  /*CSN拉高，完成*/
  	NRF2_CSN_1();  
	  
	  /*返回状态寄存器的值*/	
  	return status;             
}


/**
  * @brief    用于向NRF2的寄存器中写入一串数据
  * @param   
  *	@arg      reg : NRF的命令+寄存器地址
  *	@arg      pBuf：存储了将要写入写寄存器数据的数组，外部定义
  * @arg      bytes: pBuf的数据长度
  * @retval   NRF的status寄存器的状态
  */
uint8_t SPI_NRF2_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	  uint8_t status,uint8_t_ctr;	 

    /*置低CSN，使能SPI传输*/			
 	  NRF2_CSN_0();      

    /*发送寄存器号 */		
  	status = SPI_NRF2_RW(reg);
	
	  /*向缓冲区写入数据*/
  	for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
	  SPI_NRF2_RW(*pBuf++);
  
    /*CSN拉高，完成*/
	  NRF2_CSN_1();              
  	return status;             
}		
 


/**
  * @brief   用于向NRF2的发送缓冲区中写入数据
  * @param   
  *		@arg txBuf：存储了将要发送的数据的数组，外部定义	
  * @retval  发送结果，成功返回TXDS,失败返回MAXRT或ERROR
  */
uint8_t NRF2_Tx_Dat(uint8_t *txbuf)
{
	uint8_t state;  

	 /*ce为低，进入待机模式1*/
	NRF2_CE_0();

	/*写数据到TX BUF 最大 32个字节*/						
   SPI_NRF2_WriteBuf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);

      /*CE为高，txbuf非空，发送数据包 */   
 	 NRF2_CE_1();
	  	
	  /*等待发送完成中断 */                            
	while(NRF2_Read_IRQ()!=0); 	
	
	/*读取状态寄存器的值 */                              
	state = SPI_NRF2_ReadReg(STATUS);

	 /*清除TX_DS或MAX_RT中断标志*/                  
	SPI_NRF2_WriteReg(NRF_WRITE_REG+STATUS,state); 	

	SPI_NRF2_WriteReg(FLUSH_TX,NOP);    //清除TX FIFO寄存器 

	 /*判断中断类型*/    
	if(state&MAX_TX)                     //达到最大重发次数
			 return MAX_TX; 

	else if(state&TX_DS)                  //发送完成
		 	return TX_DS;
	 else						  
			return ERROR;                 //其他原因发送失败
} 

/**
  * @brief   用于从NRF2的接收缓冲区中读出数据
  * @param   
  *	@arg     rxBuf ：用于接收该数据的数组，外部定义	
  * @retval 
  *	@arg    接收结果
  */
uint8_t NRF2_Rx_Dat(uint8_t *rxbuf)
{
	uint8_t state; 
	NRF2_CE_1();	 //进入接收状态
	 /*等待接收中断*/
	while(NRF2_Read_IRQ()==0)
  {
    NRF2_CE_0();  	 //进入待机状态
    /*读取status寄存器的值  */               
    state=SPI_NRF2_ReadReg(STATUS);
     
    /* 清除中断标志*/      
    SPI_NRF2_WriteReg(NRF_WRITE_REG+STATUS,state);

    /*判断是否接收到数据*/
    if(state&RX_DR)                                 //接收到数据
    {
      SPI_NRF2_ReadBuf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
         SPI_NRF2_WriteReg(FLUSH_RX,NOP);          //清除RX FIFO寄存器
      return RX_DR; 
    }
    else    
      return ERROR;                    //没收到任何数据
  }
  
  return ERROR;                    //没收到任何数据
}



/**
  * @brief  NRF2配置并进入接收模式
  * @param  无
  * @retval 无
  */   
void NRF2_RX_Mode(void)
{
	  NRF2_CE_0();	  
  	SPI_NRF2_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	  
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+EN_AA,0x01);       //使能通道0的自动应答    
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01);   //使能通道0的接收地址  	 
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);	        //设置RF通信频率		  
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);    //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);     //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	NRF2_CE_1(); //CE为高,进入接收模式 
}		



/**
  * @brief  NRF2配置发送模式
  * @param  无
  * @retval 无
  */	 
void NRF2_TX_Mode(void)
{														 
	  NRF2_CE_0();	    
  	SPI_NRF2_WriteBuf(NRF_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	SPI_NRF2_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  	SPI_NRF2_WriteReg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+RF_CH,CHANAL);       //设置RF通道为40
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	SPI_NRF2_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	  NRF2_CE_1();
	  CPU_TS_Tmr_Delay_US(130);                        //CE要拉高一段时间才进入发送模式
}


