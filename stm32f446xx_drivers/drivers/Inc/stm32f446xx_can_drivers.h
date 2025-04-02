/*
 * stm32f446xx_can_drivers.h
 *
 *  Created on: Mar 26, 2025
 *      Author: kyle
 */

#ifndef INC_STM32F446XX_CAN_DRIVERS_H_
#define INC_STM32F446XX_CAN_DRIVERS_H_

typedef struct
{
	uint8_t CAN_BaudRate;
} CAN_Config_t;

typedef struct
{
	CAN_RegDef_t *pCANx; // Holds base address of CAN port
	CAN_Config_t CANConfig; // Holds CAN pin configuration settings
} CAN_Handle_t;

void CAN_Init(CAN_Handle_t *pCANHandle);


#endif /* INC_STM32F446XX_CAN_DRIVERS_H_ */
