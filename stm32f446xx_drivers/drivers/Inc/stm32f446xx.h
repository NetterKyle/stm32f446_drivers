/*
 * stm32f446.h
 *
 *  Created on: Aug 12, 2024
 *      Author: kyle
 */

#ifndef INC_STM32F446_H_
#define INC_STM32F446_H_

#include<stdint.h>
#define __vo volatile
/*
 * Memory base addresses
 */

#define FLASH_BASE_ADDR                                 0x08000000U // Base address of flash (main) memory (7 sectors comprising 512 Kbytes)
#define SRAM1_BASEADDR                                  0x20000000U // Base address of SRAM1 (112 Kbytes)
#define SRAM2_BASEADDR                                  (SRAM1_BASEADDR + 112 * 1024) // Base address of SRAM2 is base address of SRAM1 plus the size of SRAM1 (16 Kbytes)
#define ROM_BASEADDR                                    0x1FFF0000  // Base address of system memory (30 Kbytes)
#define SRAM                                            SRAM1_BASEADDR // Use SRAM1 base address for SRAM

/*
 * Peripheral bus base addresses
 */

#define PERIPH_BASEADDR                                         0x40000000U // Peripheral base address
#define APB1PERIPH_BASEADDR                                     PERIPH_BASE // APB1 peripheral base address is the same as peripheral base address
#define APB2PERIPH_BASEADDR                                     0x40010000U // APB2 peripheral base address
#define AHB1PERIPH_BASEADDR                                     0x40020000U // AHB1 peripheral base address
#define AHB2PERIPH_BASEADDR                                     0x5000 0000U // AHB2 peripheral base address

/*
 * AHB1 peripheral base addresses
 */

#define GPIOA_BASEADDR                                  (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                                  (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                                  (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                                  (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR                                  (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR                                  (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR                                  (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR                                  (AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR                                    (AHB1PERIPH_BASEADDR + 0x3800)
/*
 * APB1 peripheral base addresses
 */
#define SPI2_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR                                 (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR                                 (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR                                  (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR                                  (APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x5C00)

/*
 * APB2 peripheral base addresses
 */

#define USART1_BASEADDR                                 (APB2PERIPH_BASEADDR + 0X1000)
#define USART6_BASEADDR                                 (APB2PERIPH_BASEADDR + 0X1400)
#define SPI1_BASEADDR                                   (APB2PERIPH_BASEADDR + 0X3000)
#define SYSCFG_BASEADDR                                 (APB2PERIPH_BASEADDR + 0X3800)
#define EXTI_BASEADDR                                   (APB2PERIPH_BASEADDR + 0X3C00)

/*
 * Generic GPIO struct
 */
typedef struct
{
        __vo uint32_t MODER; // Port mode register
        __vo uint32_t OTYPER; // Port output type register
        __vo uint32_t OSPEEDR; // Port output speed register
        __vo uint32_t PUPDR; // Port pull-up/pull-down register
        __vo uint32_t IDR; // Port input data register
        __vo uint32_t ODR; // Port output data register
        __vo uint32_t BSRR; // Port bit set/reset register
        __vo uint32_t BSRRH; // Port configuration
        __vo uint32_t LCKR; // Port configuration lock register
        __vo uint32_t AFR[2]; // Alternate function register (AFR[0]: Low register, AFR[1]: High register)
} GPIO_RegDef_t;



typedef struct
{
        __vo uint32_t CR;
        __vo uint32_t PLLCFGR;
        __vo uint32_t CFGR;
        __vo uint32_t CIR;
        __vo uint32_t AHB1RSTR;
        __vo uint32_t AHB2RSTR;
        __vo uint32_t AHB3RSTR;
        uint32_t RESERVED0;
        __vo uint32_t APB1RSTR;
        __vo uint32_t APB2RSTR;
        uint32_t RESERVED1[2];
        __vo uint32_t AHB1ENR;
        __vo uint32_t AHB2ENR;
        __vo uint32_t AHB3ENR;
        uint32_t RESERVED2;
        __vo uint32_t APB1ENR;
        __vo uint32_t APB2ENR;
        uint32_t RESERVED3[2];
        __vo uint32_t AHB1LPENR;
        __vo uint32_t AHB2LPENR;
        __vo uint32_t AHB3LPENR;
        uint32_t RESERVED4;
        __vo uint32_t APB1LPENR;
        __vo uint32_t APB2LPENR;
        uint32_t RESERVED5[2];
        __vo uint32_t BDCR;
        __vo uint32_t CSR;
        uint32_t RESERVED6[2];
        __vo uint32_t SSCGR;
        __vo uint32_t PLLI2SCFGR;
        __vo uint32_t PLLSAICFGR;
        __vo uint32_t PLLDCKCFGR;
        __vo uint32_t PLLCKGATENR;
        __vo uint32_t PLLDCKCFGR2;
} RCC_RegDef_t;

/*
 * Peripheral definitions
 */
#define GPIOA                           ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                           ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                           ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                           ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE                           ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF                           ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG                           ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH                           ((GPIO_RegDef_t*) GPIOH_BASEADDR)

/*
 * Clock definition
 */
#define RCC                     ((RCC_RegDef_t*) RCC_BASEADDR)

/*
 * GPIO clock enable registers
 */
#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()         (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()         (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= (1 << 7))

/*
 * I2C clock enable registers
 */
#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= (1 << 23))

/*
 * SPI clock enable registers
 */
#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))

/*
 * USART clock enable registers
 */
#define USART1_PCLK_EN()                (RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()                (RCC->APB2ENR |= (1 << 5))

/*
 * SYSCFG clock enable register
 */
#define SYSCFG_PCLK_EN()                (RCC->APB2ENR |= 1 << 14)

/*
 * GPIO clock disable registers
 */
#define GPIOA_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 7))

/*
 * I2C clock disable registers
 */
#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 23))

/*
 * SPI clock disable registers
 */
#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))

/*
 * USART clock disable registers
 */
#define USART1_PCLK_DI()                (RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()                (RCC->APB2ENR &= ~(1 << 5))

/*
 * SYSCFG clock disable register
 */
#define SYSCFG_PCLK_DI()                (RCC->APB2ENR &= ~(1 << 14))

#define GPIOA_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7));}while(0)

#define ENABLE 			0
#define DISABLE 		1
#define SET 			ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET

#include "stm32f446xx_gpio_driver.h"

#endif /* INC_STM32F446_H_ */
