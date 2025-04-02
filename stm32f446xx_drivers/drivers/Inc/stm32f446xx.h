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
 * Processor specific details
 */

/*
 * Processor NVIC set interrupt register addresses
 */
#define NVIC_ISER0			((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*) 0xE000E10C)

/*
 * Processor NVIC clear interrupt register addresses
 */
#define NVIC_ICER0										((__vo uint32_t*) 0xE000E180)
#define NVIC_ICER1										((__vo uint32_t*) 0xE000E184)
#define NVIC_ICER2										((__vo uint32_t*) 0xE000E188)
#define NVIC_ICER3										((__vo uint32_t*) 0xE000E18C)

#define NVIC_PR_BASE_ADDR								((__vo uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED							4
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
#define APB1PERIPH_BASEADDR                                     PERIPH_BASEADDR // APB1 peripheral base address is the same as peripheral base address
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
#define CAN1_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x6800)

/*
 * APB2 peripheral base addresses
 */

#define USART1_BASEADDR                                 (APB2PERIPH_BASEADDR + 0X1000)
#define USART6_BASEADDR                                 (APB2PERIPH_BASEADDR + 0X1400)
#define SPI1_BASEADDR                                   (APB2PERIPH_BASEADDR + 0X3000)
#define SPI4_BASEADDR                                   (APB2PERIPH_BASEADDR + 0X3400)
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
        __vo uint32_t LCKR; // Port configuration lock register
        __vo uint32_t AFR[2]; // Alternate function register (AFR[0]: Low register, AFR[1]: High register)
} GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR1; // Control register 1
	__vo uint32_t CR2; // Control register 2
	__vo uint32_t SR; // Status register
	__vo uint32_t DR; // Data register
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;

typedef struct
{
	__vo uint32_t MCR; // Master control register
	__vo uint32_t MSR; // Master status register
	__vo uint32_t TSR; // Transmit status register
	__vo uint32_t RF0R; // Receive FIFO 0 register
	__vo uint32_t RF1R; // Receive FIFO 1 register
	__vo uint32_t IER; // Interrupt enable register
	__vo uint32_t ESR; // Error status register
	__vo uint32_t BTR; // Bit timing register
	__vo uint32_t TI0R; // CAN Tx mailbox 0 identifier register
	__vo uint32_t TDT0R; // CAN mailbox 0 data length control and time stamp register
	__vo uint32_t TDL0R; // CAN mailbox 0 data low register
	__vo uint32_t TDH0R; // CAN mailbox 0 data high register
	__vo uint32_t TI1R; // CAN Tx mailbox 1 identifier register
	__vo uint32_t TDT1R; // CAN mailbox 1 data length control and time stamp register
	__vo uint32_t TDL1R; // CAN mailbox 1 data low register
	__vo uint32_t TDH1R; // CAN mailbox 1 data high register
	__vo uint32_t TI2R; // CAN Tx mailbox 2 identifier register
	__vo uint32_t TDT2R; // CAN mailbox 2 data length control and time stamp register
	__vo uint32_t TDL2R; // CAN mailbox 2 data low register
	__vo uint32_t TDH2R; // CAN mailbox 2 data high register
	__vo uint32_t RI0R; //  CAN receive FIFO mailbox 0 identifier register
	__vo uint32_t RDT0R; // CAN receive FIFO mailbox 0 data length control and time stamp register
	__vo uint32_t RDL0R; // CAN receive FIFO mailbox 0 data low register
	__vo uint32_t RDH0R; // CAN receive FIFO mailbox 0 data high register
	__vo uint32_t RI1R; // CAN receive FIFO mailbox 1 identifier register
	__vo uint32_t RDT1R; // CAN receive FIFO mailbox 1 data length control and time stamp register
	__vo uint32_t RDL1R; // CAN receive FIFO mailbox 1 data low register
	__vo uint32_t RDH1R; // CAN receive FIFO mailbox 1 data high register
	uint32_t Reserved0[15];
	__vo uint32_t FS1R; // Filter scale register
	uint32_t Reserved1;
	__vo uint32_t FFA1R; // Filter FIFO assignment register
	uint32_t Reserved2;
	__vo uint32_t FA1R; // Filter activation register
	uint32_t Reserved3[9];
	__vo uint32_t F0R1; //
	__vo uint32_t F0R2; //
	__vo uint32_t F1R1; // Filter bank 1 register 1
	__vo uint32_t F1R2; // Filter bank 1 register 2
	__vo uint32_t F2R1; // Filter bank 2 register 1
	__vo uint32_t F2R2; // Filter bank 2 register 2
} CAN_RegDef_t;



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
 * EXTI (external interrupt event) struct
 */
typedef struct
{
        __vo uint32_t IMR; // Interrupt mask register
        __vo uint32_t EMR; // Event mask register
        __vo uint32_t RTSR; // Rising trigger selection register
        __vo uint32_t FTSR; // Falling trigger selection register
        __vo uint32_t SWIER; // Software interrupt event register
        __vo uint32_t PR; // Pending register
} EXTI_RegDef_t;

/*
 * SYSCFG struct
 */
typedef struct
{
        __vo uint32_t MEMRMP;
        __vo uint32_t PMC;
        __vo uint32_t EXTICR[4];
        uint32_t RESERVED1[2]; // Not mentioned in the documentation??
        __vo uint32_t CMPCR;
        uint32_t RESERVED2[2]; // Also not mentioned in the documentation?????
        __vo uint32_t CFGR; // Configuration register
} SYSCFG_RegDef_t;

/*
 * GPIO peripheral definitions
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
 * SPI peripheral definitions
 */
#define SPI1                           ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                           ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                           ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4                           ((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 * CAN peripheral definitions
 */
#define CAN1							((CAN_RegDef_t*)CAN1_BASE_ADDR)
#define CAN2							((CAN_RegDef_t*)CAN2_BASE_ADDR)


/*
 * Clock definition
 */
#define RCC                     ((RCC_RegDef_t*) RCC_BASEADDR)

#define EXTI 					((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)

/*
 * GPIO clock enable
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
 * SPI clock enable
 */
#define SPI1_PCLK_EN()         (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()         (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()         (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()         (RCC->APB2ENR |= (1 << 13))

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
#define SYSCFG_PCLK_EN()                (RCC->APB2ENR |= (1 << 14))

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

#define GPIO_BASE_ADDR_TO_CODE(x)		((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 : 0 )
/*
 * IRQ external interrupt position numbers
 */


#define IRQ_NO_EXTI0					6
#define IRQ_NO_EXTI1					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI9_5					23
#define IRQ_NO_EXTI15_10				40

/*
 * NVIC interrrupt priority levels
 */
#define NVIC_IRQ_PRI0 					0
#define NVIC_IRQ_PRI15					15



#define ENABLE 					0
#define DISABLE 				1
#define SET 					ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET 			SET
#define GPIO_PIN_RESET 			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET

#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_SSI				8 // Slave select internal
#define SPI_CR1_SSM				9 // Software slave management
#define SPI_CR1_RX_ONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_BIDI_MODE		15

#define SPI_CR2_SSOE			2

#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_BSY				7

#define CAN_MCR_INRQ			0 //Initialization request
#define CAN_MCR_SLEEP			1 // Sleep mode request
#define CAN_MCR_TXFP			2 // Transmit FIFO priority
#define CAN_MCR_RFLM			3 // Receive FIFO locked mode
#define CAN_MCR_NART			4 // No automatic retransmission
#define CAN_MCR_AWUM			5 // Automatic wakeup mode
#define CAN_MCR_ABOM			6 // Automatic bus-off management
#define CAN_MCR_TTCM			7 // Time triggered communication mode
#define CAN_MCR_RESET			15 // bxCAN software master reset
#define CAN_MCR_DBF				16 // Debug freeze

#define CAN_MSR_INAK			0 // Initialization acknowledge
#define CAN_MSR_SLAK			1 // Sleep acknowledge
#define CAN_MSR_ERRI			2 // Error interrupt
#define CAN_MSR_WKUI			3 // Wakeup interrupt
#define CAN_MSR_SLAKI			4 // Sleep acknowledge interrupt
#define CAN_MSR_TXM				8 // Transmit mode
#define CAN_MSR_RXM				9 // Receive mode
#define CAN_MSR_SAMP			10 // Last sample point
#define CAN_MSR_RX				11 // CAN Rx signal

#define CAN_TSR_RQCP0 			0 // Request complete mailbox 0
#define CAN_TSR_TXOK0			1 // Transmission OK of mailbox 0
#define CAN_TSR_ALST0			2 // Arbitration lost for mailbox 0
#define CAN_TSR_TERR0			3 // Transmission error of mailbox 0
#define CAN_TSR_ABRQ0			7 // Abort request for mailbox 0
#define CAN_TSR_RQCP1			8 // Request completed mailbox 1
#define CAN_TSR_TXOK1			9 // Transmission OK of mailbox 1
#define CAN_TSR_ALST1			10 // Arbitration lost for mailbox 1
#define CAN_TSR_TERR1			11 // Transmission error of mailbox 1
#define CAN_TSR_ABRQ1			15 // Abort request for mailbox 1
#define CAN_TSR_RQCP2			16 // Request completed mailbox 2
#define CAN_TSR_TXOK2			17 // Transmission OK of mailbox 2
#define CAN_TSR_ALST2			18 // Arbitration lost for mailbox 2
#define CAN_TSR_TERR2			19 // Transmission error of mailbox 2
#define CAN_TSR_ABRQ2			23 // Abort request for mailbox 2
#define CAN_TSR_CODE			24 // Mailbox code
#define CAN_TSR_TME0 			26 // Transmit mailbox 0 empty
#define CAN_TSR_TME1			27 // Transmit mailbox 1 empty
#define CAN_TSR_TME2			28 // Transmit mailbox 2 empty
#define CAN_TSR_LOW0			29 // Lowest priority flag for mailbox 0
#define CAN_TSR_LOW1			30 // Lowest priority flag for mailbox 1
#define CAN_TSR_LOW2			31 // Lowest priority flag for mailbox 2

#define CAN_RF0R_FMP0			0 // FIFO 0 message pending
#define CAN_RF0R_FULL0			3 // FIFO 0 full
#define CAN_RF0R_FOVR0			4 // FIFO 0 overrun
#define CAN_RF0R_RFOM0			5 // Release FIFO 0 output mailbox

#define CAN_RF0R_FMP0			0 // FIFO 0 message pending
#define CAN_RF0R_FULL0			3 // FIFO 0 full
#define CAN_RF0R_FOVR0			4 // FIFO 0 overrun
#define CAN_RF0R_RFOM0			5 // Release FIFO 0 output mailbox

#define CAN_RF1R_FMP0			0 // FIFO 1 message pending
#define CAN_RF1R_FULL0			3 // FIFO 1 full
#define CAN_RF1R_FOVR0			4 // FIFO 1 overrun
#define CAN_RF1R_RFOM0			5 // Release FIFO 1 output mailbox

#define CAN_IER_TMEIE			0 // Transmit mailbox empty interrupt enable
#define CAN_IER_FMPIE0			1 // FIFO message pending interrupt enable
#define CAN_IER_FFIE0			2 // FIFO full interrupt enable
#define CAN_IER_FOVIE0			3 // FIFO overrun interrupt enable
#define CAN_IER_FMPIE1			4 // FIFO message pending interrupt enable
#define CAN_IER_FFIE1			5 // FIFO full interrupt enable
#define CAN_IER_FOVIE1			6 // FIFO overrun interrupt enable
#define CAN_IER_EWGIE			8 // Error warning interrupt enable
#define CAN_IER_EPVIE			9 // Error passive interrupt enable
#define CAN_IER_BOFIE			10 // Bus-off interrupt enable
#define CAN_IER_LECIE			11 // Last error code interrupt enable
#define CAN_IER_ERRIE			15 // Error interrupt enable
#define CAN_IER_WKUIE			16 // Wakeup interrupt enable
#define CAN_IER_SLKIE			17 // Sleep interrupt enable

#define CAN_ESR_EWGF			0 // Error warning flag
#define CAN_ESR_EPVF			1 // Error passive flag
#define CAN_ESR_BOFF			2 // Bus-off flag
#define CAN_ESR_LEC				4 // Last error code
#define CAN_ESR_TEC				16 // Least significant bit of the 9-bit transmit error counter
#define CAN_ESR_REC				24 // Receive error counter

#define CAN_BTR_BRP				0 // Baud rate prescaler
#define CAN_BTR_TS1				16 // Time segment 1
#define CAN_BTR_TS2				20 // Time segment 2
#define CAN_BTR_SJW				24 //  Resynchronization jump width
#define CAN_BTR_LBKM			30 // Loop back mode (debug)
#define CAN_BTR_SILM			31 // Silent mode (debug)

#define CAN_RIxR_RTR			1 // Remote transmission request CAN receive FIFO x
#define CAN_RIxR_IDE			2 // Identifier extension CAN receive FIFO x
#define CAN_RIxR_EXID			3 // Extended identifier CAN receive FIFO x
#define CAN_RIxR_STID			21 // Standard identifier or MSBs of extended identifier CAN receive FIFO x



#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_drivers.h"

#endif /* INC_STM32F446_H_ */
