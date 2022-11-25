#ifndef __INTERRUPTSHANDLER_H__
#define __INTERRUPTSHANDLER_H__

#include "stm32f10x.h"

#define AFIO_EXTICR_PA                ((uint16_t)0x0000)            /*!< PA[0] pin */
#define AFIO_EXTICR_PB                ((uint16_t)0x0001)            /*!< PB[0] pin */
#define AFIO_EXTICR_PC                ((uint16_t)0x0002)            /*!< PC[0] pin */
#define AFIO_EXTICR_PD                ((uint16_t)0x0003)            /*!< PD[0] pin */
#define AFIO_EXTICR_PE                ((uint16_t)0x0004)            /*!< PE[0] pin */
#define AFIO_EXTICR_PF                ((uint16_t)0x0005)            /*!< PF[0] pin */
#define AFIO_EXTICR_PG                ((uint16_t)0x0006)            /*!< PG[0] pin */

const uint16_t AFIO_EXTICR[4] = {((uint16_t)0x000F), ((uint16_t)0x00F0), ((uint16_t)0x0F00), ((uint16_t)0xF000)};

class InterruptsHandler
{
	public:
		InterruptsHandler();
		void configPin(uint8_t pin, GPIO_TypeDef *port);
	private:
		
};

#endif
