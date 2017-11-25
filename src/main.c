#include "stm32f10x.h"
#include "stm32f10x_mfrc522.h"

void USART_MultiSendData(USART_TypeDef* USARTx, uint8_t* Data);
void MFRC522_SelfTest(MFRC522_Transport* MFRC522_Transport);

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
	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

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

	// SPI system initialization
	// MOSI & CSK & NSS pins configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// NSS & RST pins configuration
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
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

	EXTI_InitTypeDef   EXTI_InitStructure;

	/* Configure PA.00 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect EXTI0 Line to PA.00 pin */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_SetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_3);

	MFRC522_Transport MFRC522_Transport;
	MFRC522_Transport.SPIx = SPI1;
	MFRC522_Transport.GPIOx = GPIOA;
	MFRC522_Transport.GPIO_Pin = GPIO_Pin_4;

	MFRC522_SelfTest(&MFRC522_Transport);

	MFRC522_Reset(&MFRC522_Transport);
	MFRC522_Write(&MFRC522_Transport, MFRC522_TModeReg, 0x8D);
	MFRC522_Write(&MFRC522_Transport, MFRC522_TPrescalerReg, 0x3E);
	MFRC522_Write(&MFRC522_Transport, MFRC522_TReloadRegL, 30);
	MFRC522_Write(&MFRC522_Transport, MFRC522_TReloadRegH, 0);
	MFRC522_Write(&MFRC522_Transport, MFRC522_RFCfgReg, 0x70);			// 48dB gain
	MFRC522_Write(&MFRC522_Transport, MFRC522_TxASKReg, 0x40);
	MFRC522_Write(&MFRC522_Transport, MFRC522_ModeReg, 0x3D);
	MFRC522_AntennaOn(&MFRC522_Transport);

	GPIO_SetBits(GPIOC, GPIO_Pin_13);

	USART_MultiSendData(USART1, (uint8_t*) "Initialized");
	while (1);
}

void MFRC522_SelfTest(MFRC522_Transport* MFRC522_Transport)
{
	MFRC522_Write(MFRC522_Transport, MFRC522_CommandReg, PCD_RESETPHASE);
	MFRC522_Write(MFRC522_Transport, MFRC522_FIFOLevelReg, 0x80);
	for (uint8_t i = 0; i < 25; i++) {
		MFRC522_Write(MFRC522_Transport, MFRC522_FIFODataReg, 0x00);
	}

	MFRC522_Write(MFRC522_Transport, MFRC522_CommandReg, 0x01);
	MFRC522_Write(MFRC522_Transport, MFRC522_AutoTestReg, 0x09);
	MFRC522_Write(MFRC522_Transport, MFRC522_FIFODataReg, 0x00);
	MFRC522_Write(MFRC522_Transport, MFRC522_CommandReg, PCD_CALCCRC);
	for (uint8_t i = 0; i < 0xff; i++) {
		if (MFRC522_Read(MFRC522_Transport, MFRC522_FIFOLevelReg) >= 64) {
			break;
		}
	}
	MFRC522_Write(MFRC522_Transport, MFRC522_CommandReg, PCD_IDLE);

	for (uint8_t i = 0; i < 64; i++) {
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		USART_SendData(USART1, MFRC522_Read(MFRC522_Transport, MFRC522_FIFODataReg));
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	}

	MFRC522_Write(MFRC522_Transport, MFRC522_AutoTestReg, 0x00);
}
void USART_MultiSendData(USART_TypeDef* USARTx, uint8_t* Data)
{
	while (*Data) {
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		USART_SendData(USARTx, *Data);
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		Data++;
	}
}

void USART1_IRQHandler ()
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
		uint16_t receivedData = USART_ReceiveData(USART1);

		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		SPI_I2S_SendData(SPI1, receivedData);
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
		GPIO_SetBits(GPIOA, GPIO_Pin_4);

		uint16_t spiResponse = SPI_I2S_ReceiveData(SPI1);
		USART_SendData(USART1, spiResponse);

		EXTI_GenerateSWInterrupt(EXTI_Line2);

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET) {
		static uint8_t ledStatus = SET;
		ledStatus = !ledStatus;
		GPIO_WriteBit(GPIOC, GPIO_Pin_13, ledStatus);

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}
