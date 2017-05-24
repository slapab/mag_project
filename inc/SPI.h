/*
 * SPI.h
 *
 *  Created on: 19 May 2017
 *      Author: Marcin
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx_hal.h"

#define SPIx                             SPI5
#define SPIxCLK_ENABLE()                 __HAL_RCC_SPI5_CLK_ENABLE();
#define SPIxCLK_DISABLE()                __HAL_RCC_SPI5_CLK_DISABLE();
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE();
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOF_CLK_ENABLE();
#define SPIxMOSI_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOF_CLK_ENABLE();
#define WRX_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOD_CLK_ENABLE();
#define CS_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOC_CLK_ENABLE();
#define RST_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOD_CLK_ENABLE();

#define SPIx_FORCE_RESET()               __SPI5_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __SPI5_RELEASE_RESET

#define SPIx_SCK_PIN                     GPIO_PIN_7
#define SPIx_SCK_GPIO_PORT               GPIOF
#define SPIx_SCK_AF                      GPIO_AF5_SPI5
#define SPIx_MISO_PIN                    GPIO_PIN_8
#define SPIx_MISO_GPIO_PORT              GPIOF
#define SPIx_MISO_AF                     GPIO_AF5_SPI5
#define SPIx_MOSI_PIN                    GPIO_PIN_9
#define SPIx_MOSI_GPIO_PORT              GPIOF
#define SPIx_MOSI_AF                     GPIO_AF5_SPI5

#define WRX_PIN							 GPIO_PIN_13
#define WRX_GPIO_PORT                    GPIOD
#define CS_PIN                           GPIO_PIN_2
#define CS_GPIO_PORT                     GPIOC
#define RST_PIN                          GPIO_PIN_12
#define RST_GPIO_PORT                    GPIOD

#define DMAx                             DMA2
#define DMAx_CLOCK_ENABLE()              __HAL_RCC_DMA2_CLK_ENABLE();
#define DMAx_RX_STREAM                   DMA2_Stream3
#define DMAx_RX_CHANNEL                  DMA_CHANNEL_2
#define DMAx_TX_STREAM                   DMA2_Stream4
#define DMAx_TX_CHANNEL                  DMA_CHANNEL_2
#define DMAx_CLOCK_ENABLE()              __HAL_RCC_DMA2_CLK_ENABLE();


#define BUFFERSIZE                      (COUNTOF(aTxBuffer)-1)
#define COUNTOF(__BUFFER__)             (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

void GPIO_SPI_DMA_Init(void);
void SPI_Init(void);
void DMA_Init(void);
void GPIO_Init(void);
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi);














#endif /* SPI_H_ */
