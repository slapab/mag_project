#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "queue.h"
#include "mag3110.h"
#include "LCD.h"
#include "SPI.h"
#include "fonts.h"
#include "sdcard.h"
#include "fatfs.h"

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;
FATFS SDFatFs;
FIL myFile;
/// Uncomment one of below macros to select appropriate clocks frequencies
#define USE_MAIN_CLOCK_180MHZ 1
//#define USE_MAIN_CLOCK_72MHZ 1

static void SystemClock_Config(void);

void toggleLedTask(void* params) {
    APP_LOG_LOCK();
    APP_LOG_MSG("System core clock: "); APP_LOG_UINT(SystemCoreClock, 10);
    APP_LOG_UNLOCK();

    for( ;; ) {
        BSP_LED_Toggle(LED4);
        vTaskDelay(300);
    }

    vTaskDelete(NULL);
}

const char* template = "X:%d Y:%d Z:%d T:%d\n";
char log_buff[30];

void consumerTask(void *p)
{
    mag3110_data_t str = {0};
    mag3110_start();
    for( ;; ) {
        if(true == mag3110_get(&str))
        {
			sprintf(log_buff, template, str.x, str.y, str.z, str.temp);
			APP_LOG_LU_MSG(log_buff);
        }
        else
        {
        	APP_LOG_LU_MSG("No data -> check sensor connection!");
        }
        vTaskDelay(1000);
    }

    vTaskDelete(NULL);
}


int main(void) {
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();
    SystemCoreClockUpdate();
    LCD_Init();
    SysTick_Config(SystemCoreClock / 1000);
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    sdcard_init( &hsd );
    sdcard_mount( &SDFatFs, ""  );
    sdcard_open_file( &myFile, "maggg.csv" );
    sdcard_save2file( &myFile, 1, 2, 3 );
    sdcard_close_file( &myFile );
    initLog();
    BSP_LED_Init(LED4);


//---INIT I2C and MAGNEIC FIELD DETECTOR------------//
    bool x = false;
    x =  init_i2c();
    x = false;
    x = init_mag3110(SAMPLING_80HZ, 1000);
//--------------------------------------------------//

    xTaskCreate(toggleLedTask, "toggleLED", 128, NULL, 1, NULL);
    //consumer task shows how to read data form sensor in another task
    xTaskCreate(consumerTask, "consumerTask", 128, NULL, 1, NULL);
    /* Start the RTOS scheduler, this function should not return as it causes the
    execution context to change from main() to one of the created tasks. */
    vTaskStartScheduler();

    for(;;)
    {
    	//
    }

    return 0;
}


/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 180000000
 *            HCLK(Hz)                       = 180000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 360
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 *         The LTDC Clock is configured as follow :
 *            PLLSAIN                        = 192
 *            PLLSAIR                        = 4
 *            PLLSAIDivR                     = 8
 * @param  None
 * @retval None
 *
 * COPYRIGHT(c) 2014 STMicroelectronics
 */

#if 1 == USE_MAIN_CLOCK_180MHZ
static void SystemClock_Config(void) {
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /* Enable Power Control clock */
    __PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /*##-1- System Clock Configuration #########################################*/
    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    HAL_PWREx_ActivateOverDrive();

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

    /*##-2- LTDC Clock Configuration ###########################################*/
    /* LCD clock configuration */
    /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
    /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
    /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/4 = 48 Mhz */
    /* LTDC clock frequency = PLLLCDCLK / RCC_PLLSAIDIVR_8 = 48/8 = 6 Mhz */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
    PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
    PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}
#endif



/**
 * SYSCLK = 72
 * HCLK = 72
 * Cortex Timer 72
 * FCLK 72
 * APB1 Peripheral (PCLK1) 36
 * APB1 Timer 72
 * APB2 Peripheral (PCLK2) 72
 * APB2 Timer 72
 *
 */
#if 1 == USE_MAIN_CLOCK_72MHZ
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);


    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

//    /**Configure the Systick interrupt time
//    */
//  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
//
//    /**Configure the Systick
//    */
//  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
//
//  /* SysTick_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
#endif
