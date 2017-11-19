#include "stm32f10x.h"
#include "stm32f10x_mfrc522.h"


int main(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	//Port A, B, C clock system enabling
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	// USART 1 and SPI 1 clock system enabling
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Configure PC13 (board led) in output push pull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure USART1 Tx pin as alternate function open-drain */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 Rx pin as alternate function in-floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure); // USART initialization
	USART_Cmd(USART1, ENABLE); // USART enabling
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // Interruption enabling

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//	 SPI system initialization
//   MOSI & CSK & NSS pins configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//MISO pin configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction 		= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode 			  	= SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize 		  	= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL 			  	= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 			  	= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 			  	= SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;

	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);

	SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);

//	MFRC522_Reset(SPI1);
//	MFRC522_Write(SPI1, MFRC522_TModeReg, 0x8D);
//	MFRC522_Write(SPI1, MFRC522_TPrescalerReg, 0x3E);
//	MFRC522_Write(SPI1, MFRC522_TReloadRegL, 30);
//	MFRC522_Write(SPI1, MFRC522_TReloadRegH, 0);
//	MFRC522_Write(SPI1, MFRC522_RFCfgReg, 0x70);			// 48dB gain
//	MFRC522_Write(SPI1, MFRC522_TxASKReg, 0x40);
//	MFRC522_Write(SPI1, MFRC522_ModeReg, 0x3D);
//	MFRC522_AntennaOn(SPI1);

	GPIO_SetBits(GPIOC, GPIO_Pin_13);

	USART_SendData(USART1, 'A');

	SPI_I2S_SendData(SPI1, 0xFF);
	while (1) {

	}
}

void USART1_IRQHandler ()
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
		static uint8_t ledStatus = SET;
		uint16_t receivedData = USART_ReceiveData(USART1);
		USART_SendData(USART1, receivedData);
		SPI_I2S_SendData(SPI1, receivedData);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		ledStatus = !ledStatus;
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, ledStatus);
	}
}
