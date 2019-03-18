

#ifndef __SPI_NRF_H
#define __SPI_NRF_H

#include "stm32f4xx.h"



#define TX_ADR_WIDTH 	5  	//�����ַ���
#define TX_PLOAD_WIDTH  32   //��������ͨ����Ч���ݿ��0~32Byte 

#define RX_ADR_WIDTH    5
#define RX_PLOAD_WIDTH  32



#define CHANAL 40	//Ƶ��ѡ�� 

// SPI(nRF24L01) commands ,	NRF��SPI����궨�壬���NRF����ʹ���ĵ�
#define NRF_READ_REG    0x00  // Define read command to register
#define NRF_WRITE_REG   0x20  // Define write command to register
#define RD_RX_PLOAD 0x61  // Define RX payload register address
#define WR_TX_PLOAD 0xA0  // Define TX payload register address
#define FLUSH_TX    0xE1  // Define flush TX register command
#define FLUSH_RX    0xE2  // Define flush RX register command
#define REUSE_TX_PL 0xE3  // Define reuse TX payload register command
#define NOP         0xFF  // Define No Operation, might be used to read status register

// SPI(nRF24L01) registers(addresses) ��NRF24L01 ��ؼĴ�����ַ�ĺ궨��
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

#define MAX_RT      0x10 //�ﵽ����ط������жϱ�־λ
#define TX_DS		0x20 //��������жϱ�־λ	  // 

#define RX_DR		0x40 //���յ������жϱ�־λ

/* NRF1 �ܽ� */

#define NRF_SPI                           SPI1
#define NRF_SPI_CLK                       RCC_APB2Periph_SPI1
#define NRF_SPI_CLK_INIT                  RCC_APB2PeriphClockCmd

#define NRF_SPI_SCK_PIN                   GPIO_Pin_3                 
#define NRF_SPI_SCK_GPIO_PORT             GPIOB                      
#define NRF_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define NRF_SPI_SCK_PINSOURCE             GPIO_PinSource3
#define NRF_SPI_SCK_AF                    GPIO_AF_SPI1

#define NRF_SPI_MISO_PIN                  GPIO_Pin_4               
#define NRF_SPI_MISO_GPIO_PORT            GPIOB                  
#define NRF_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define NRF_SPI_MISO_PINSOURCE            GPIO_PinSource4
#define NRF_SPI_MISO_AF                   GPIO_AF_SPI1

#define NRF_SPI_MOSI_PIN                  GPIO_Pin_5               
#define NRF_SPI_MOSI_GPIO_PORT            GPIOB                      
#define NRF_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define NRF_SPI_MOSI_PINSOURCE            GPIO_PinSource5
#define NRF_SPI_MOSI_AF                   GPIO_AF_SPI1


#define NRF_CSN_GPIO_PORT    GPIOE
#define NRF_CSN_PIN          GPIO_Pin_15
#define NRF_CSN_GPIO_CLK     RCC_AHB1Periph_GPIOE

#define NRF_CE_GPIO_PORT    GPIOE
#define NRF_CE_PIN          GPIO_Pin_13
#define NRF_CE_GPIO_CLK     RCC_AHB1Periph_GPIOE

#define NRF_IRQ_GPIO_PORT    GPIOE
#define NRF_IRQ_PIN          GPIO_Pin_11
#define NRF_IRQ_GPIO_CLK     RCC_AHB1Periph_GPIOE

#define NRF_CSN_HIGH()      GPIO_SetBits(NRF_CSN_GPIO_PORT, NRF_CSN_PIN)
#define NRF_CSN_LOW()       GPIO_ResetBits(NRF_CSN_GPIO_PORT, NRF_CSN_PIN)		        //csn�õ�
#define NRF_CE_HIGH()	      GPIO_SetBits(NRF_CE_GPIO_PORT,NRF_CE_PIN)
#define NRF_CE_LOW()	      GPIO_ResetBits(NRF_CE_GPIO_PORT,NRF_CE_PIN)			      //CE�õ�
#define NRF_Read_IRQ()		  GPIO_ReadInputDataBit(NRF_IRQ_GPIO_PORT, NRF_IRQ_PIN)  //�ж�����

/* NRF2 �ܽ� */

#define NRF2_SPI                           SPI3
#define NRF2_SPI_CLK                       RCC_APB1Periph_SPI3
#define NRF2_SPI_CLK_INIT                  RCC_APB1PeriphClockCmd

#define NRF2_SPI_SCK_PIN                   GPIO_Pin_10                
#define NRF2_SPI_SCK_GPIO_PORT             GPIOC                      
#define NRF2_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOC
#define NRF2_SPI_SCK_PINSOURCE             GPIO_PinSource10
#define NRF2_SPI_SCK_AF                    GPIO_AF_SPI3

#define NRF2_SPI_MISO_PIN                  GPIO_Pin_11            
#define NRF2_SPI_MISO_GPIO_PORT            GPIOC                  
#define NRF2_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOC
#define NRF2_SPI_MISO_PINSOURCE            GPIO_PinSource11
#define NRF2_SPI_MISO_AF                   GPIO_AF_SPI3

#define NRF2_SPI_MOSI_PIN                  GPIO_Pin_12              
#define NRF2_SPI_MOSI_GPIO_PORT            GPIOC                     
#define NRF2_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOC
#define NRF2_SPI_MOSI_PINSOURCE            GPIO_PinSource12
#define NRF2_SPI_MOSI_AF                   GPIO_AF_SPI3

#define NRF2_CSN_GPIO_PORT    GPIOA
#define NRF2_CSN_PIN          GPIO_Pin_4
#define NRF2_CSN_GPIO_CLK     RCC_AHB1Periph_GPIOA

#define NRF2_CE_GPIO_PORT    GPIOA
#define NRF2_CE_PIN          GPIO_Pin_6
#define NRF2_CE_GPIO_CLK     RCC_AHB1Periph_GPIOA

#define NRF2_IRQ_GPIO_PORT    GPIOA
#define NRF2_IRQ_PIN          GPIO_Pin_8
#define NRF2_IRQ_GPIO_CLK     RCC_AHB1Periph_GPIOA


#define NRF2_CSN_HIGH()     GPIO_SetBits(NRF2_CSN_GPIO_PORT, NRF2_CSN_PIN)
#define NRF2_CSN_LOW()      GPIO_ResetBits(NRF2_CSN_GPIO_PORT, NRF2_CSN_PIN)		        //csn�õ�
#define NRF2_CE_HIGH()	    GPIO_SetBits(NRF2_CE_GPIO_PORT,NRF2_CE_PIN)
#define NRF2_CE_LOW()	      GPIO_ResetBits(NRF2_CE_GPIO_PORT,NRF2_CE_PIN)			      //CE�õ�
#define NRF2_Read_IRQ()		  GPIO_ReadInputDataBit ( NRF2_IRQ_GPIO_PORT, NRF2_IRQ_PIN)  //�ж�����

void SPI_NRF_Init(void);
u8 SPI_NRF_RW(u8 dat);
u8 SPI_NRF_ReadReg(u8 reg );
u8 SPI_NRF_WriteReg(u8 reg,u8 dat);

u8 SPI_NRF_ReadBuf(u8 reg,u8 *pBuf,u8 bytes);
u8 SPI_NRF_WriteBuf(u8 reg ,u8 *pBuf,u8 bytes);	

void NRF_TX_Mode(void);
void NRF_RX_Mode(void);
u8 NRF_Rx_Dat(u8 *rxbuf);
u8 NRF_Tx_Dat(u8 *txbuf);
u8 NRF_Check(void); 


void SPI_NRF2_Init(void);
u8 SPI_NRF2_RW(u8 dat);
u8 SPI_NRF2_ReadReg(u8 reg );
u8 SPI_NRF2_WriteReg(u8 reg,u8 dat);

u8 SPI_NRF2_ReadBuf(u8 reg,u8 *pBuf,u8 bytes);
u8 SPI_NRF2_WriteBuf(u8 reg ,u8 *pBuf,u8 bytes);	

void NRF2_TX_Mode(void);
void NRF2_RX_Mode(void);
u8 NRF2_Rx_Dat(u8 *rxbuf);
u8 NRF2_Tx_Dat(u8 *txbuf);
u8 NRF2_Check(void); 

#endif /* __SPI_NRF_H */
