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
#define LOG_RINGBUFFER_SIZE 256

// Ring buffer forward declarations

/// Returns how many data is currently inside the internal buffer
static size_t ring_buffer_fullness(void);
/// Returns total free space inside of ring buffer
static size_t ring_buffer_freespace(void);
/// Copies one element to data
//static bool ring_buffer_get(uint8_t* const data);
/// Copies elements up to len to destination place. Number of copied elements can be less
/// than requested length and that number is returned.
static size_t ring_buffer_get_multi(uint8_t* const destination, const uint8_t len);
static bool ring_buffer_isfull(void);
static bool ring_buffer_isempty(void);
static bool ring_buffer_appendBuff(const uint8_t * const data, const size_t len);




static void logTask(void* context) {
    enum {BUFF_SIZE = 16};
    uint8_t tempBuff[BUFF_SIZE] = {0};

    while(true) {
        if (pdTRUE == xSemaphoreTakeRecursive(logMutex, 1000)) {

            const size_t dataNoToSend = ring_buffer_get_multi(tempBuff, BUFF_SIZE);
            xSemaphoreGiveRecursive(logMutex);

            if (dataNoToSend > 0) {
                for (size_t el = 0; el < dataNoToSend; ++el) {
                    while (0 == (usartLogHandle.Instance->SR & USART_SR_TXE_Msk)); // wait for empty buffer
                    usartLogHandle.Instance->DR = tempBuff[el];
                }
            } else {
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
//static bool ring_buffer_get(uint8_t * const data) {
//    if (ring_buffer_isempty())
//        return false;
//
//    sig_atomic_t tail_idx = ringBuffer.tail & ( LOG_RINGBUFFER_SIZE - 1);
//    uint8_t tmp = ringBuffer.buf[tail_idx];
//    *data = tmp;
//
//    ++ringBuffer.tail;
//
//    return true;
//}

static size_t ring_buffer_get_multi(uint8_t* const destination, const uint8_t len) {
    if ((NULL != destination) && (len > 0) && (true == ring_buffer_isempty())) {
        return 0;
    }

    const sig_atomic_t tail_idx = ringBuffer.tail & (LOG_RINGBUFFER_SIZE - 1);
    const sig_atomic_t head_idx = ringBuffer.head & (LOG_RINGBUFFER_SIZE - 1);

    // Evaluate how many data can be copied using memcpy()
    size_t copySize = len;
    if (head_idx > tail_idx) {
        // simple case, whole valid data are placed in spaces before the end of the internal buffer
        if (ring_buffer_fullness() < len) {
            copySize = ring_buffer_fullness();
        }
    } else {
        // head_idx < tail_idx
        // head is behind the tail - copy only from the tail to the end of the internal buffer
        const size_t dataLenToTheEndOfBuffer = LOG_RINGBUFFER_SIZE - tail_idx;
        if (dataLenToTheEndOfBuffer < len) {
            copySize = dataLenToTheEndOfBuffer;
        }
    }

    // copy data to destination
    memcpy(destination, (void*)&ringBuffer.buf[tail_idx], copySize);
    ringBuffer.tail += copySize;

    return copySize;
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

// appending one element at the time -> not good performance
//static bool ring_buffer_appendBuff(const uint8_t * const data, const size_t len) {
//
//    if (!ring_buffer_isfull()) {
//        // copy not more than length of current free space
//        const size_t dataSize = (len > ring_buffer_freespace()) ? ring_buffer_freespace() : len;
//
//        // instead of modulo operation -> but the size must be power of 2
//        sig_atomic_t head_idx = ringBuffer.head & (LOG_RINGBUFFER_SIZE - 1);
//        for (size_t i = 0; i < dataSize; ++i) {
//            ringBuffer.buf[head_idx] = data[i];
//            head_idx = (head_idx + 1) & (LOG_RINGBUFFER_SIZE - 1);
//        }
//
//        // update index
//        ringBuffer.head += len;
//        return true;
//    } else {
//        return false;
//    }
//
//}

static bool ring_buffer_appendBuff(const uint8_t * const data, const size_t len) {

    if (!ring_buffer_isfull() && NULL != data && len > 0) {
        // copy not more than length of current free space
        const size_t dataSize = (len > ring_buffer_freespace()) ? ring_buffer_freespace() : len;

        const sig_atomic_t head_idx = ringBuffer.head & (LOG_RINGBUFFER_SIZE - 1);
        const sig_atomic_t tail_idx = ringBuffer.tail & (LOG_RINGBUFFER_SIZE - 1);
        // - evaluate the free space from the head index to the end of the internal buffer
        const size_t freeSpaceAtTheEnd = LOG_RINGBUFFER_SIZE - head_idx;

        if ((head_idx < tail_idx) /*Simple case, the whole free memory is within linear addresses*/
                || ((head_idx > tail_idx) && (0 == tail_idx)) /* Still the only free memory is placed before the end of internal buffer*/
                || ((head_idx >= tail_idx) && (dataSize <= freeSpaceAtTheEnd))) /* There is also free space from the beginning but data size is less than free space before the end of the internal buffer */
        {
            memcpy((void*) &ringBuffer.buf[head_idx], data, dataSize);
            // updates the head index
            ringBuffer.head += dataSize;
        } else {
            // head_idx > tail_idx and dataSize > freeSpaceAtTheEnd - this requires two calls to memcpy()
            // copy from head_idx to the end of internal buffer
            memcpy((void*)&ringBuffer.buf[head_idx], data, freeSpaceAtTheEnd);

            const size_t sizeOfRemainingData = dataSize - freeSpaceAtTheEnd;
            // copy from 0 to remaining free space - copies the rest of the message
            memcpy((void*)&ringBuffer.buf[0], &data[freeSpaceAtTheEnd], sizeOfRemainingData);
            // update index
            ringBuffer.head += freeSpaceAtTheEnd + sizeOfRemainingData;
        }

        return true;
    } else {
        return false;
    }

}
