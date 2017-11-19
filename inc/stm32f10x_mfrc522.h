/*
 * stm32f10x_mfrc522.h
 *
 *  Created on: Nov 16, 2017
 *      Author: nick
 */

#ifndef STM32F10X_MFRC522_H_
#define STM32F10X_MFRC522_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

 typedef enum
 {
	 // Command and status
	 MFRC522_CommandReg = 0x01, // starts and stops command execution
	 MFRC522_ComlEnReg = 0x02, // enable and disable interrupt request control bits
	 MFRC522_DivlEnReg = 0x03, // enable and disable interrupt request control bits
	 MFRC522_ComIrqReg = 0x04, // interrupt request bits
	 MFRC522_DivIrqReg = 0x05, // interrupt request bits
	 MFRC522_ErrorReg = 0x06, // error bits showing the error status of the last command executed
	 MFRC522_Status1Reg = 0x07, // communication status bits
	 MFRC522_Status2Reg = 0x08, // receiver and transmitter status bits
	 MFRC522_FIFODataReg = 0x09, // input and output of 64 byte FIFO buffer
	 MFRC522_FIFOLevelReg = 0x0A, // number of bytes stored in the FIFO buffer
	 MFRC522_WaterLevelReg = 0x0B, // level for FIFO underflow and overflow warning
	 MFRC522_ControlReg = 0x0C, // miscellaneous control registers
	 MFRC522_BitFramingReg = 0x0D, // adjustments for bit-oriented frames
	 MFRC522_CollReg = 0x0E, // bit position of the first bit-collision detected on the RF interface

	 // Command
	 MFRC522_ModeReg = 0x11, // defines general modes for transmitting and receiving
	 MFRC522_TxModeReg = 0x12, //  defines transmission data rate and framing
	 MFRC522_RxModeReg = 0x13, // defines reception data rate and framing
	 MFRC522_TxControlReg = 0x14, // controls the logical behavior of the antenna driver pins TX1 and TX2
	 MFRC522_TxASKReg = 0x15, // controls the setting of the transmission modulation
	 MFRC522_TxSelReg = 0x16, // selects the internal sources for the antenna driver
	 MFRC522_RxSelReg = 0x17, // selects internal receiver settings
	 MFRC522_RxThresholdReg = 0x18, // selects thresholds for the bit decoder
	 MFRC522_DemodReg = 0x19, // defines demodulator settings
	 MFRC522_MfTxReg = 0x1C, // controls some MIFARE communication transmit parameters
	 MFRC522_MfRxReg = 0x1D, // controls some MIFARE communication receive parameters
	 MFRC522_SerialSpeedReg = 0x1F, // selects the speed of the serial UART interface

	 // Configuration
	 MFRC522_CRCResultRegH = 0x21, // shows the MSB and LSB values of the CRC calculation
	 MFRC522_CRCResultRegL = 0x22, // shows the MSB and LSB values of the CRC calculation
	 MFRC522_ModWidthReg = 0x24, // controls the ModWidth setting
	 MFRC522_RFCfgReg = 0x26, // reserved for future use
	 MFRC522_GsNReg = 0x27, // configures the receiver gain
	 MFRC522_CWGsPReg = 0x28, // selects the conductance of the antenna driver pins TX1 and TX2 for modulation
	 MFRC522_ModGsPReg = 0x29, //  defines the conductance of the p-driver output during periods of no modulation
	 MFRC522_TModeReg = 0x2A, //  defines the conductance of the p-driver output during periods of modulation
	 MFRC522_TPrescalerReg = 0x2B, // defines settings for the internal timer
	 MFRC522_TReloadRegL = 0x2C, // defines the 16-bit timer reload value
	 MFRC522_TReloadRegH = 0x2D, // defines the 16-bit timer reload value
	 MFRC522_TCounterValRegH = 0x2E, // shows the 16-bit timer value
	 MFRC522_TCounterValRegL = 0x2E, // shows the 16-bit timer value

	 // Test register
	 MFRC522_TestSel1Reg = 0x31, // general test signal configuration
	 MFRC522_TestSel2Reg = 0x32, //  general test signal configuration and PRBS control
	 MFRC522_TestPinEnReg = 0x33, // enables pin output driver on pins D1 to D7
	 MFRC522_TestPinValueReg = 0x34, //  defines the values for D1 to D7 when it is used as an I/O bus
	 MFRC522_TestBusReg = 0x35, //  shows the status of the internal test bus
	 MFRC522_AutoTestReg = 0x36, // controls the digital self test
	 MFRC522_VersionReg = 0x37, // shows the software version
	 MFRC522_AnalogTestReg = 0x38, //  controls the pins AUX1 and AUX2
	 MFRC522_TestDAC1Reg = 0x39, // defines the test value for TestDAC1
	 MFRC522_TestDAC2Reg = 0x3A, // defines the test value for TestDAC2
	 MFRC522_TestADCReg = 0x3B, // shows the value of ADC I and Q channels
 } MFRC522Regs_TypeDef;

#define MFRC522_DUMMY					0x00			// Dummy byte
#define MFRC522_MAX_LEN					16				// Buf len byte

 // Status enumeration, Used with most functions
 #define MI_OK			0
 #define MI_NOTAGERR		1
 #define MI_ERR			2

 // MFRC522 Commands
 #define PCD_IDLE						0x00   // NO action; Cancel the current command
 #define PCD_AUTHENT					0x0E   // Authentication Key
 #define PCD_RECEIVE					0x08   // Receive Data
 #define PCD_TRANSMIT					0x04   // Transmit data
 #define PCD_TRANSCEIVE					0x0C   // Transmit and receive data,
 #define PCD_RESETPHASE					0x0F   // Reset
 #define PCD_CALCCRC					0x03   // CRC Calculate

 // Mifare_One card command word
 #define PICC_REQIDL					0x26   // find the antenna area does not enter hibernation
 #define PICC_REQALL					0x52   // find all the cards antenna area
 #define PICC_ANTICOLL					0x93   // anti-collision
 #define PICC_SElECTTAG					0x93   // election card
 #define PICC_AUTHENT1A					0x60   // authentication key A
 #define PICC_AUTHENT1B					0x61   // authentication key B
 #define PICC_READ						0x30   // Read Block
 #define PICC_WRITE						0xA0   // write block
 #define PICC_DECREMENT					0xC0   // debit
 #define PICC_INCREMENT					0xC1   // recharge
 #define PICC_RESTORE					0xC2   // transfer block data to the buffer
 #define PICC_TRANSFER					0xB0   // save the data in the buffer
 #define PICC_HALT						0x50   // Sleep

#define IS_RC522_REG(REG) (((REG) == MFRC522_CommandReg) || ((REG) == MFRC522_ComlEnReg) || \
		((REG) == MFRC522_DivlEnReg) || ((REG) == MFRC522_ComIrqReg) || \
		((REG) == MFRC522_DivIrqReg) || ((REG) == MFRC522_ErrorReg) || \
		((REG) == MFRC522_Status1Reg) || ((REG) == MFRC522_Status2Reg) || \
		((REG) == MFRC522_FIFODataReg) || ((REG) == MFRC522_FIFOLevelReg) || \
		((REG) == MFRC522_WaterLevelReg) || ((REG) == MFRC522_ControlReg) || \
		((REG) == MFRC522_BitFramingReg) || ((REG) == MFRC522_CollReg) || \
		((REG) == MFRC522_ModeReg) || ((REG) == MFRC522_TxModeReg) || \
		((REG) == MFRC522_RxModeReg) || ((REG) == MFRC522_TxControlReg) || \
		((REG) == MFRC522_TxASKReg) || ((REG) == MFRC522_TxSelReg) || \
		((REG) == MFRC522_RxSelReg) || ((REG) == MFRC522_RxThresholdReg) || \
		((REG) == MFRC522_DemodReg) || ((REG) == MFRC522_MfTxReg) || \
		((REG) == MFRC522_MfRxReg) || ((REG) == MFRC522_SerialSpeedReg) || \
		((REG) == MFRC522_CRCResultRegH) || ((REG) == MFRC522_ModWidthReg) || \
		((REG) == MFRC522_RFCfgReg) || ((REG) == MFRC522_GsNReg) || \
		((REG) == MFRC522_CWGsPReg) || ((REG) == MFRC522_ModGsPReg) || \
		((REG) == MFRC522_TModeReg) || ((REG) == MFRC522_TPrescalerReg) || \
		((REG) == MFRC522_TReloadRegH) || ((REG) == MFRC522_TReloadRegL) || \
		((REG) == MFRC522_TCounterValRegH) || ((REG) == MFRC522_TCounterValRegL) || \
		((REG) == MFRC522_TestSel1Reg) || ((REG) == MFRC522_TestSel2Reg) || \
		((REG) == MFRC522_TestPinEnReg) || ((REG) == MFRC522_TestPinValueReg) || \
		((REG) == MFRC522_TestBusReg) || ((REG) == MFRC522_AutoTestReg) || \
		((REG) == MFRC522_VersionReg) || ((REG) == MFRC522_AnalogTestReg) || \
		((REG) == MFRC522_TestDAC1Reg) || ((REG) == MFRC522_TestDAC2Reg) || \
		((REG) == MFRC522_TestADCReg) || ((REG) == MFRC522_CRCResultRegL))

 uint8_t MFRC522_Check(SPI_TypeDef* SPIx, uint8_t * id);
 uint8_t MFRC522_Compare(uint8_t * CardID, uint8_t * CompareID);
 void MFRC522_Write(SPI_TypeDef* SPIx, uint8_t addr, uint8_t val);
 uint8_t MFRC522_Read(SPI_TypeDef* SPIx, uint8_t addr);
 void MFRC522_SetBitMask(SPI_TypeDef* SPIx, uint8_t reg, uint8_t mask);
 void MFRC522_ClearBitMask(SPI_TypeDef* SPIx, uint8_t reg, uint8_t mask);
 void MFRC522_AntennaOn(SPI_TypeDef* SPIx);
 void MFRC522_AntennaOff(SPI_TypeDef* SPIx);
 void MFRC522_Reset(SPI_TypeDef* SPIx);
 void MFRC522_Init(SPI_TypeDef* SPIx);
 uint8_t MFRC522_Request(SPI_TypeDef* SPIx, uint8_t reqMode, uint8_t *TagType);
 uint8_t MFRC522_ToCard(SPI_TypeDef* SPIx, uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen);
 uint8_t MFRC522_Anticoll(SPI_TypeDef* SPIx, uint8_t *serNum);
 void MFRC522_CalculateCRC(SPI_TypeDef* SPIx, uint8_t *pIndata, uint8_t len, uint8_t *pOutData);
 uint8_t MFRC522_SelectTag(SPI_TypeDef* SPIx, uint8_t *serNum);
 uint8_t MFRC522_Auth(SPI_TypeDef* SPIx, uint8_t authMode, uint8_t BlockAddr, uint8_t *Sectorkey, uint8_t *serNum);
 uint8_t MFRC522_ReadBlock(SPI_TypeDef* SPIx, uint8_t blockAddr, uint8_t *recvData);
 uint8_t MFRC522_WriteBlock(SPI_TypeDef* SPIx, uint8_t blockAddr, uint8_t *writeData);
 void MFRC522_Halt(SPI_TypeDef* SPIx);

#ifdef __cplusplus
}
#endif

#endif /* STM32F10X_MFRC522_H_ */
