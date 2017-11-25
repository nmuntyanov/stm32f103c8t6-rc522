/*
 * stm32f10x_mfrc522.c
 *
 *  Created on: Nov 16, 2017
 *      Author: nick
 */

#include "stm32f10x_mfrc522.h"

void MFRC522_CS_Enable(MFRC522_Transport* MFRC522_Transport)
{
	while (SPI_I2S_GetFlagStatus(MFRC522_Transport->SPIx, SPI_I2S_FLAG_BSY) == SET);
	GPIO_ResetBits(MFRC522_Transport->GPIOx, MFRC522_Transport->GPIO_Pin);
}

void MFRC522_CS_Disable(MFRC522_Transport* MFRC522_Transport)
{
	while (SPI_I2S_GetFlagStatus(MFRC522_Transport->SPIx, SPI_I2S_FLAG_BSY) == SET);
	GPIO_SetBits(MFRC522_Transport->GPIOx, MFRC522_Transport->GPIO_Pin);
}

uint8_t MFRC522_Check(MFRC522_Transport* MFRC522_Transport, uint8_t* id) {
	uint8_t status;
	status = MFRC522_Request(MFRC522_Transport, PICC_REQIDL, id);	// Find cards, return card type
	if (status == MI_OK) {
		status = MFRC522_Anticoll(MFRC522_Transport, id);	// Card detected. Anti-collision, return card serial number 4 bytes
	}
	MFRC522_Halt(MFRC522_Transport);		// Command card into hibernation

	return status;
}

uint8_t MFRC522_Compare(uint8_t * CardID, uint8_t * CompareID) {
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if (CardID[i] != CompareID[i]) return MI_ERR;
	}

	return MI_OK;
}

void MFRC522_Write(MFRC522_Transport* MFRC522_Transport, uint8_t addr, uint8_t val)
{
	assert_param(IS_RC522_REG(addr));
	// Address format：0XXXXXX0
	addr = (addr << 1) & 0x7E;
	MFRC522_CS_Enable(MFRC522_Transport);
	SPI_I2S_SendData(MFRC522_Transport->SPIx, (uint16_t) addr);
	while(SPI_I2S_GetFlagStatus(MFRC522_Transport->SPIx, SPI_I2S_FLAG_BSY) == SET);
	SPI_I2S_SendData(MFRC522_Transport->SPIx, (uint16_t) val);
	while(SPI_I2S_GetFlagStatus(MFRC522_Transport->SPIx, SPI_I2S_FLAG_BSY) == SET);
	MFRC522_CS_Disable(MFRC522_Transport);
}


uint8_t MFRC522_Read(MFRC522_Transport* MFRC522_Transport, uint8_t addr)
{
	assert_param(IS_RC522_REG(addr));

	uint8_t val;
	// Address format：1XXXXXX0
	addr = ((addr << 1) & 0x7E) | 0x80;
	MFRC522_CS_Enable(MFRC522_Transport);
	SPI_I2S_SendData(MFRC522_Transport->SPIx, (uint16_t) addr);
	while(SPI_I2S_GetFlagStatus(MFRC522_Transport->SPIx, SPI_I2S_FLAG_BSY) == SET);

	SPI_I2S_ReceiveData(MFRC522_Transport->SPIx);
	while(SPI_I2S_GetFlagStatus(MFRC522_Transport->SPIx, SPI_I2S_FLAG_BSY) == SET);

	SPI_I2S_SendData(MFRC522_Transport->SPIx, 0xFF);
	while(SPI_I2S_GetFlagStatus(MFRC522_Transport->SPIx, SPI_I2S_FLAG_BSY) == SET);

	val = SPI_I2S_ReceiveData(MFRC522_Transport->SPIx);
	while(SPI_I2S_GetFlagStatus(MFRC522_Transport->SPIx, SPI_I2S_FLAG_BSY) == SET);
	MFRC522_CS_Disable(MFRC522_Transport);

	return val;
}

void MFRC522_SetBitMask(MFRC522_Transport* MFRC522_Transport, uint8_t reg, uint8_t mask)
{
	uint8_t tmp;
	tmp = MFRC522_Read(MFRC522_Transport, reg);
	MFRC522_Write(MFRC522_Transport, reg, tmp | mask);  // set bit mask
}

void MFRC522_ClearBitMask(MFRC522_Transport* MFRC522_Transport, uint8_t reg, uint8_t mask)
{
	uint8_t tmp;
	tmp = MFRC522_Read(MFRC522_Transport, reg);
	MFRC522_Write(MFRC522_Transport, reg, tmp & (~mask));
}

void MFRC522_AntennaOn(MFRC522_Transport* MFRC522_Transport)
{
	uint8_t temp;

	temp = MFRC522_Read(MFRC522_Transport, MFRC522_TxControlReg);
	if (!(temp & 0x03)) {
		MFRC522_SetBitMask(MFRC522_Transport, MFRC522_TxControlReg, 0x03);
	}
}

void MFRC522_AntennaOff(MFRC522_Transport* MFRC522_Transport)
{
	MFRC522_ClearBitMask(MFRC522_Transport, MFRC522_TxControlReg, 0x03);
}

void MFRC522_Reset(MFRC522_Transport* MFRC522_Transport)
{
	MFRC522_Write(MFRC522_Transport, MFRC522_CommandReg, PCD_RESETPHASE);
}

uint8_t MFRC522_Request(MFRC522_Transport* MFRC522_Transport, uint8_t reqMode, uint8_t *TagType)
{
	uint8_t status;
	uint16_t backBits;	// The received data bits

	MFRC522_Write(MFRC522_Transport, MFRC522_BitFramingReg, 0x07);	// TxLastBists = BitFramingReg[2..0]
	TagType[0] = reqMode;
	status = MFRC522_ToCard(MFRC522_Transport, PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);
	if ((status != MI_OK) || (backBits != 0x10)) {
		status = MI_ERR;
	}

	return status;
}

uint8_t MFRC522_ToCard(MFRC522_Transport* MFRC522_Transport, uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen)
{
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint16_t i;

	switch (command) {
		case PCD_AUTHENT: {
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE: {
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
		break;
	}

	MFRC522_Write(MFRC522_Transport, MFRC522_ComlEnReg, irqEn | 0x80);
	MFRC522_ClearBitMask(MFRC522_Transport, MFRC522_ComIrqReg, 0x80);
	MFRC522_SetBitMask(MFRC522_Transport, MFRC522_FIFOLevelReg, 0x80);
	MFRC522_Write(MFRC522_Transport, MFRC522_CommandReg, PCD_IDLE);

	// Writing data to the FIFO
	for (i = 0; i < sendLen; i++) {
		MFRC522_Write(MFRC522_Transport, MFRC522_FIFODataReg, sendData[i]);
	}

	// Execute the command
	MFRC522_Write(MFRC522_Transport, MFRC522_CommandReg, command);
	if (command == PCD_TRANSCEIVE) MFRC522_SetBitMask(MFRC522_Transport, MFRC522_BitFramingReg, 0x80);					// StartSend=1,transmission of data starts

	// Waiting to receive data to complete
	i = 2000;	// i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms
	do {
		// CommIrqReg[7..0]
		// Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = MFRC522_Read(MFRC522_Transport, MFRC522_ComIrqReg);
		i--;
	} while ((i!=0) && !(n & 0x01) && !(n & waitIRq));

	MFRC522_ClearBitMask(MFRC522_Transport, MFRC522_BitFramingReg, 0x80);	// StartSend=0

	if (i == 0) {
		return MI_ERR;
	}

	if ((MFRC522_Read(MFRC522_Transport, MFRC522_ErrorReg) & 0x1B)) {
		return MI_ERR;
	}

	uint8_t status = MI_OK;

	if (n & irqEn & 0x01) {
		status = MI_NOTAGERR;
	}
	if (command == PCD_TRANSCEIVE) {
		n = MFRC522_Read(MFRC522_Transport, MFRC522_FIFOLevelReg);
		lastBits = MFRC522_Read(MFRC522_Transport, MFRC522_ControlReg) & 0x07;
		if (lastBits) {
			*backLen = (n - 1) * 8 + lastBits;
		} else {
			*backLen = n * 8;
		}
		if (n == 0) {
			n = 1;
		}
		if (n > MFRC522_MAX_LEN) {
			n = MFRC522_MAX_LEN;
		}
		for (i = 0; i < n; i++) {
			backData[i] = MFRC522_Read(MFRC522_Transport, MFRC522_FIFODataReg);		// Reading the received data in FIFO
		}
	}

	return status;
}

uint8_t MFRC522_Anticoll(MFRC522_Transport* MFRC522_Transport, uint8_t *serNum)
{
	uint8_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;

	MFRC522_Write(MFRC522_Transport, MFRC522_BitFramingReg, 0x00);												// TxLastBists = BitFramingReg[2..0]
	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = MFRC522_ToCard(MFRC522_Transport, PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);
	if (status == MI_OK) {
		// Check card serial number
		for (i = 0; i < 4; i++) {
			serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i]) {
			status = MI_ERR;
		}
	}

	return status;
}

void MFRC522_CalculateCRC(MFRC522_Transport* MFRC522_Transport, uint8_t *pIndata, uint8_t len, uint8_t *pOutData)
{
	uint8_t i, n;

	MFRC522_ClearBitMask(MFRC522_Transport, MFRC522_DivIrqReg, 0x04);	// CRCIrq = 0
	MFRC522_SetBitMask(MFRC522_Transport, MFRC522_FIFOLevelReg, 0x80);	// Clear the FIFO pointer
	// Write_MFRC522(CommandReg, PCD_IDLE);

	// Writing data to the FIFO
	for (i = 0; i < len; i++) MFRC522_Write(MFRC522_Transport, MFRC522_FIFODataReg, *(pIndata+i));
	MFRC522_Write(MFRC522_Transport, MFRC522_CommandReg, PCD_CALCCRC);

	// Wait CRC calculation is complete
	i = 0xFF;
	do {
		n = MFRC522_Read(MFRC522_Transport, MFRC522_DivIrqReg);
		i--;
	} while ((i!=0) && !(n & 0x04));	// CRCIrq = 1

	// Read CRC calculation result
	pOutData[0] = MFRC522_Read(MFRC522_Transport, MFRC522_CRCResultRegL);
	pOutData[1] = MFRC522_Read(MFRC522_Transport, MFRC522_CRCResultRegH);
}

uint8_t MFRC522_SelectTag(MFRC522_Transport* MFRC522_Transport, uint8_t *serNum)
{
	uint8_t i;
	uint8_t status;
	uint8_t size;
	uint16_t recvBits;
	uint8_t buffer[9];

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for (i = 0; i < 5; i++) {
		buffer[i+2] = *(serNum+i);
	}
	MFRC522_CalculateCRC(MFRC522_Transport, buffer, 7, &buffer[7]);
	status = MFRC522_ToCard(MFRC522_Transport, PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
	if ((status == MI_OK) && (recvBits == 0x18)) {
		size = buffer[0];
	} else {
		size = 0;
	}

	return size;
}

uint8_t MFRC522_Auth(MFRC522_Transport* MFRC522_Transport, uint8_t authMode, uint8_t BlockAddr, uint8_t *Sectorkey, uint8_t *serNum)
{
	uint8_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[12];

	// Verify the command block address + sector + password + card serial number
	buff[0] = authMode;
	buff[1] = BlockAddr;
	for (i = 0; i < 6; i++) buff[i+2] = *(Sectorkey+i);
	for (i=0; i<4; i++) buff[i+8] = *(serNum+i);
	status = MFRC522_ToCard(MFRC522_Transport, PCD_AUTHENT, buff, 12, buff, &recvBits);
	if (status != MI_OK || !(MFRC522_Read(MFRC522_Transport, MFRC522_Status2Reg) & 0x08)) {
		status = MI_ERR;
	}

	return status;
}

uint8_t MFRC522_ReadBlock(MFRC522_Transport* MFRC522_Transport, uint8_t blockAddr, uint8_t *recvData)
{
	uint8_t status;
	uint16_t unLen;

	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;
	MFRC522_CalculateCRC(MFRC522_Transport, recvData,2, &recvData[2]);
	status = MFRC522_ToCard(MFRC522_Transport, PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);
	if ((status != MI_OK) || (unLen != 0x90)) {
		status = MI_ERR;
	}

	return status;
}

uint8_t MFRC522_WriteBlock(MFRC522_Transport* MFRC522_Transport, uint8_t blockAddr, uint8_t *writeData)
{
	uint8_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[18];

	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	MFRC522_CalculateCRC(MFRC522_Transport, buff, 2, &buff[2]);
	status = MFRC522_ToCard(MFRC522_Transport, PCD_TRANSCEIVE, buff, 4, buff, &recvBits);
	if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
		status = MI_ERR;
	}
	if (status == MI_OK) {
		// Data to the FIFO write 16Byte
		for (i = 0; i < 16; i++) {
			buff[i] = *(writeData+i);
		}
		MFRC522_CalculateCRC(MFRC522_Transport, buff, 16, &buff[16]);
		status = MFRC522_ToCard(MFRC522_Transport, PCD_TRANSCEIVE, buff, 18, buff, &recvBits);
		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
			status = MI_ERR;
		}
	}
	return status;
}


void MFRC522_Halt(MFRC522_Transport* MFRC522_Transport)
{
	uint16_t unLen;
	uint8_t buff[4];

	buff[0] = PICC_HALT;
	buff[1] = 0;
	MFRC522_CalculateCRC(MFRC522_Transport, buff, 2, &buff[2]);
	MFRC522_ToCard(MFRC522_Transport, PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}
