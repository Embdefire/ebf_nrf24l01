#ifndef __24L01_H
#define __24L01_H
#include "stm32h7xx.h"


/* NRF���Ӧ��SPI��� */
extern SPI_HandleTypeDef NRF1_Handler; 
extern SPI_HandleTypeDef NRF2_Handler; 


#define TX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5   	//5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  32  	//32�ֽڵ��û����ݿ��

#define CHANAL 40	//Ƶ��ѡ�� 

// SPI(NRF14L01) commands ,	NRF��SPI����궨�壬���NRF����ʹ���ĵ�
#define NRF_READ_REG    0x00  // Define read command to register
#define NRF_WRITE_REG   0x20  // Define write command to register
#define RD_RX_PLOAD 0x61  // Define RX payload register address
#define WR_TX_PLOAD 0xA0  // Define TX payload register address
#define FLUSH_TX    0xE1  // Define flush TX register command
#define FLUSH_RX    0xE2  // Define flush RX register command
#define REUSE_TX_PL 0xE3  // Define reuse TX payload register command
#define NOP         0xFF  // Define No Operation, might be used to read status register

// SPI(NRF14L01) registers(addresses) ��NRF14L01 ��ؼĴ�����ַ�ĺ궨��
#define CONFIG      0x00  // 'Config' register address
#define EN_AA       0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR   0x02  // 'Enabled RX addresses' register address
#define SETUP_AW    0x03  // 'Setup address width' register address
#define SETUP_RETR  0x04  // 'Setup Auto. Retrans' register address
#define RF_CH       0x05  // 'RF channel' register address
#define RF_SETUP    0x06  // 'RF setup' register address
#define STATUS      0x07  // 'Status' register address
#define OBSERVE_TX  0x08  // 'Observe TX' register address
#define CD          0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0  0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1  0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2  0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3  0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4  0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5  0x0F  // 'RX address pipe5' register address
#define TX_ADDR     0x10  // 'TX address' register address
#define RX_PW_P0    0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1    0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2    0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3    0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4    0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5    0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS 0x17  // 'FIFO Status Register' register address

															
#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_DS   		0x20  //TX��������ж�
#define RX_DR   		0x40  //���յ������ж�



//NRF2���Ŷ���
/*******************************************************/
/*CE����*/
#define NRF2_CE_PIN                     GPIO_PIN_2             
#define NRF2_CE_GPIO_PORT               GPIOC                 
#define NRF2_CE_GPIO_CLK_ENABLE()        __GPIOC_CLK_ENABLE()

/*CSN����*/
#define NRF2_CSN_PIN                     GPIO_PIN_7                
#define NRF2_CSN_GPIO_PORT               GPIOC                      
#define NRF2_CSN_GPIO_CLK_ENABLE()       __GPIOC_CLK_ENABLE()

/*SCK����*/
#define NRF2_CLK_PIN                      GPIO_PIN_5             
#define NRF2_CLK_GPIO_PORT                GPIOA                       
#define NRF2_CLK_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

/*MISO����*/
#define NRF2_MISO_PIN                     GPIO_PIN_6                
#define NRF2_MISO_GPIO_PORT               GPIOA                       
#define NRF2_MISO_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()


/*MOSI����*/
#define NRF2_MOSI_PIN                    GPIO_PIN_7                
#define NRF2_MOSI_GPIO_PORT              GPIOA                       
#define NRF2_MOSI_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

/*IRQ����*/
#define NRF2_IRQ_PIN                     GPIO_PIN_12            
#define NRF2_IRQ_GPIO_PORT               GPIOA                      
#define NRF2_IRQ_GPIO_CLK_ENABLE()       __GPIOA_CLK_ENABLE()

/*ѡ��SPI*/
#define NRF2_SPI                         SPI1
/*���ù���*/
#define NRF2_GPIO_AF                     GPIO_AF5_SPI1
/*SPIʱ��*/
#define NRF2_SPI_CLOCK()                 __HAL_RCC_SPI1_CLK_ENABLE(); 
/************************************************************/


#define NRF2_CE_0()         HAL_GPIO_WritePin(NRF2_CE_GPIO_PORT,NRF2_CE_PIN,GPIO_PIN_RESET)
#define NRF2_CE_1()         HAL_GPIO_WritePin(NRF2_CE_GPIO_PORT,NRF2_CE_PIN,GPIO_PIN_SET)    

#define NRF2_CSN_0()        HAL_GPIO_WritePin(NRF2_CSN_GPIO_PORT,NRF2_CSN_PIN,GPIO_PIN_RESET)
#define NRF2_CSN_1()        HAL_GPIO_WritePin(NRF2_CSN_GPIO_PORT,NRF2_CSN_PIN,GPIO_PIN_SET)    

#define NRF2_CLK_0()        HAL_GPIO_WritePin(NRF2_CLK_GPIO_PORT,NRF2_CLK_PIN,GPIO_PIN_RESET)
#define NRF2_CLK_1()        HAL_GPIO_WritePin(NRF2_CLK_GPIO_PORT,NRF2_CLK_PIN,GPIO_PIN_SET)    

#define NRF2_MOSI_0()       HAL_GPIO_WritePin(NRF2_MOSI_GPIO_PORT,NRF2_MOSI_PIN,GPIO_PIN_RESET)
#define NRF2_MOSI_1()       HAL_GPIO_WritePin(NRF2_MOSI_GPIO_PORT,NRF2_MOSI_PIN,GPIO_PIN_SET)    

#define NRF2_Read_IRQ()	    HAL_GPIO_ReadPin(NRF2_IRQ_GPIO_PORT, NRF2_IRQ_PIN)  //�ж�����
/************************************************************/

//NRF2���ͽ������ݿ�ȶ���								   	   
uint8_t SPI_NRF2_RW(uint8_t TxData);
void SPI_NRF2_Init(void);//��ʼ��
void NRF2_RX_Mode(void);//����Ϊ����ģʽ
void NRF2_TX_Mode(void);//����Ϊ����ģʽ
uint8_t SPI_NRF2_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts);//д������
uint8_t SPI_NRF2_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t uint8_ts);//��������		  
uint8_t SPI_NRF2_ReadReg(uint8_t reg);			//���Ĵ���
uint8_t SPI_NRF2_WriteReg(uint8_t reg, uint8_t value);//д�Ĵ���
uint8_t NRF2_Check(void);//���24L01�Ƿ����
uint8_t NRF2_Tx_Dat(uint8_t *txbuf);//����һ����������
uint8_t NRF2_Rx_Dat(uint8_t *rxbuf);//����һ����������

#endif
