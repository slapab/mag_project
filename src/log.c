#include <assert.h>
#include <string.h>
#include <signal.h> // sigatomic_t
#include "log.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "stm32f4xx_hal.h"

#define USART_TX_PIN   GPIO_PIN_9
#define USART_PORT     GPIOA
#define USART_PERIPH   USART1
#define USART_BAUDRATE 115200


static SemaphoreHandle_t logMutex;
static USART_HandleTypeDef usartLogHandle;

static void logTask(void* context);

// Ring buffer size
// Size of ring buffer must be power of 2!
#define LOG_RINGBUFFER_SIZE 128

// Ring buffer forward declarations
static size_t ring_buffer_fullness(void);
static size_t ring_buffer_freespace(void);
static bool ring_buffer_get(uint8_t * const data);
static bool ring_buffer_isfull(void);
static bool ring_buffer_isempty(void);
static bool ring_buffer_appendBuff(const uint8_t * const data, const size_t len);




static void logTask(void* context) {
    uint8_t charToSend = 0;

    while(true) {
        if (pdTRUE == xSemaphoreTakeRecursive(logMutex, 1000)) {
            if (true == ring_buffer_get(&charToSend)) {
                xSemaphoreGiveRecursive(logMutex);

                while (0 == (usartLogHandle.Instance->SR & USART_SR_TXE_Msk)); // wait for empty buffer
                usartLogHandle.Instance->DR = charToSend;
            } else {
                xSemaphoreGiveRecursive(logMutex);
                taskYIELD();
            }
        } // mutex taken
    }

    vTaskDelete(NULL);
}


void initLog(void) {
#ifdef DEBUG

    logMutex = xSemaphoreCreateRecursiveMutex();
    assert(NULL != logMutex);

    if (NULL != logMutex) {
        __GPIOA_CLK_ENABLE();
        __USART1_CLK_ENABLE();

        GPIO_InitTypeDef usartGPIOInit = {
            .Pin = USART_TX_PIN,
            .Mode = GPIO_MODE_AF_OD/*GPIO_MODE_AF_PP*/,
            .Pull = GPIO_PULLUP/*GPIO_NOPULL*/,
            .Speed = GPIO_SPEED_FREQ_MEDIUM,
            .Alternate = GPIO_AF7_USART1,
        };
        HAL_GPIO_Init(USART_PORT, &usartGPIOInit);

        usartLogHandle.Instance = USART_PERIPH,
        usartLogHandle.Init.BaudRate = USART_BAUDRATE,
        usartLogHandle.Init.WordLength = USART_WORDLENGTH_8B,
        usartLogHandle.Init.StopBits = USART_STOPBITS_2,
        usartLogHandle.Init.Parity = USART_PARITY_NONE,
        usartLogHandle.Init.Mode = USART_MODE_TX,
        usartLogHandle.Init.CLKLastBit = USART_LASTBIT_DISABLE,
        assert(HAL_OK == HAL_USART_Init(&usartLogHandle));

        assert(pdPASS == xTaskCreate(logTask, "logTask", 64, (void*)&logMutex, 1, NULL));
    }

#endif
}

bool logLock(void) {
    return (pdTRUE == xSemaphoreTakeRecursive(logMutex, 1000)) ? true : false;
}


void logUnlock(void) {
    xSemaphoreGiveRecursive(logMutex);
}


void logWriteMsg(const char* const msg) {
    ring_buffer_appendBuff((uint8_t*)msg, strnlen(msg, LOG_RINGBUFFER_SIZE));
}


void logWriteInt(int32_t val, uint_fast8_t base) {
    enum {BUFF_SIZE = 16};
    char buff[BUFF_SIZE] = {0};

    if (16 == base) {
        snprintf(buff, BUFF_SIZE, "0x%lX", val);
    } else {
        snprintf(buff, BUFF_SIZE, "%ld", val);
    }

    ring_buffer_appendBuff((uint8_t*)buff, strnlen(buff, BUFF_SIZE));
}


void logWriteUInt(uint32_t val, uint_fast8_t base) {
    enum {BUFF_SIZE = 16};
    char buff[BUFF_SIZE] = {0};

    if (16 == base) {
        snprintf(buff, BUFF_SIZE, "0x%lX", val);
    } else {
        snprintf(buff, BUFF_SIZE, "%lu", val);
    }

    ring_buffer_appendBuff((uint8_t*)buff, strnlen(buff, BUFF_SIZE));
}




// -----------------------------------------------------------------------------
// Simple ring buffer implementation
// -----------------------------------------------------------------------------



// the ring buffer structure
struct ft80x_it_ring_buffer {
    volatile uint8_t buf[LOG_RINGBUFFER_SIZE];
    volatile sig_atomic_t head;
    volatile sig_atomic_t tail;
} ringBuffer;

static size_t ring_buffer_fullness(void) {
    size_t fullness = (ringBuffer.head - ringBuffer.tail) & (LOG_RINGBUFFER_SIZE - 1);
    return fullness;
}

static size_t ring_buffer_freespace(void) {
    size_t fullness = ring_buffer_fullness();
    return LOG_RINGBUFFER_SIZE - fullness - 1; // -1 to handle this implementation of ring buffer
}

// get form ring buffer
static bool ring_buffer_get(uint8_t * const data) {
    if (ring_buffer_isempty())
        return false;

    sig_atomic_t tail_idx = ringBuffer.tail & ( LOG_RINGBUFFER_SIZE - 1);
    uint8_t tmp = ringBuffer.buf[tail_idx];
    *data = tmp;

    ++ringBuffer.tail;

    return true;
}

// full if head+1 == tail
// empty if tail == head
static bool ring_buffer_isfull(void) {
    // add 1 to handle this implementation of ringBuffer
    sig_atomic_t head_idx = (ringBuffer.head + 1) & (LOG_RINGBUFFER_SIZE - 1);
    sig_atomic_t tail_idx = ringBuffer.tail & (LOG_RINGBUFFER_SIZE - 1);

    return (head_idx == tail_idx) ? true : false;
}

static bool ring_buffer_isempty(void) {
    return (ringBuffer.head == ringBuffer.tail) ? true : false;
}

// api to handle the ring buffer
static bool ring_buffer_appendBuff(const uint8_t * const data, const size_t len) {

    if (!ring_buffer_isfull()) {
        const size_t free_space = ring_buffer_freespace();
        size_t copySize = (free_space < len) ? free_space : len;

        // instead of modulo operation -> but the size must be power of 2
        sig_atomic_t head_idx = ringBuffer.head & (LOG_RINGBUFFER_SIZE - 1);

        for (size_t i = 0; i < copySize; ++i) {
            ringBuffer.buf[head_idx] = data[i];
            head_idx = (head_idx + 1) & (LOG_RINGBUFFER_SIZE - 1);
        }

        // update index
        ringBuffer.head += len;
        return true;
    } else {
        return false;
    }

}
