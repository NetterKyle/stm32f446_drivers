/*
 * stm32f446xx_spi_drivers.h
 *
 *  Created on: Aug 20, 2024
 *      Author: kyle
 */

#ifndef INC_STM32F446XX_SPI_DRIVERS_H_
#define INC_STM32F446XX_SPI_DRIVERS_H_

#include "stm32f446xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;


typedef struct
{
	SPI_RegDef_t *pSPIx; // Holds base address of SPI port
	SPI_Config_t SPIConfig; // Holds SPI pin configuration settings
} SPI_Handle_t;

/*
 * @SPI mode
 */
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_MONITOR				0

/*
 * @SPI bus config
 */

#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 * @SPI clock speed
 */

#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					0
#define SPI_SCLK_SPEED_DIV8					0
#define SPI_SCLK_SPEED_DIV16				0
#define SPI_SCLK_SPEED_DIV32				0
#define SPI_SCLK_SPEED_DIV64				0
#define SPI_SCLK_SPEED_DIV128				0
#define SPI_SCLK_SPEED_DIV256				0

/*
 * @SPI data frame format
 */

#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

#define SPI_TXE_FLAG						(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG						(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG						(1 << SPI_SR_BSY)

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t pRxBuffer);

/*
 * IRW Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

#endif /* INC_STM32F446XX_SPI_DRIVERS_H_ */
