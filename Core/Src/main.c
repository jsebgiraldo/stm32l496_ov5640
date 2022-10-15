/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"

#include "user_ring_buffer.h"

#include "ff_gen_drv.h"
#include "sd_diskio.h"

#include "ov5640.h"

#include "stm32l496g_discovery.h"
#include "stm32l496g_discovery_io.h"
#include "stm32l496g_discovery_camera.h"
#include "stm32l496g_discovery_lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  E_RED  ,
  E_GRN  ,
  E_YEL  ,
  E_BLU  ,
  E_MAG  ,
  E_CYN  ,
  E_WHT  ,
  E_RESET,
  E_LAST,
}printf_color_t;

const char *str_color[E_LAST] =
{
    "\x1B[31m",
    "\x1B[32m",
    "\x1B[33m",
    "\x1B[34m",
    "\x1B[35m",
    "\x1B[36m",
    "\x1B[37m",
    "\x1B[0m"
};


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static const char *TAG = " [MAIN] ";
#define printf_dbg(format, ...)     	\
    do                                 	\
    {                                 	\
        printf(str_color[E_MAG]);		\
        printf(TAG);					\
        printf(str_color[E_RESET]);   	\
        printf(format, ##__VA_ARGS__); 	\
    } while (0)

#define PROJECT_NAME "STM32L496"
#define VERSION "1.0"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

DMA2D_HandleTypeDef hdma2d;

SD_HandleTypeDef hsd1;

UART_HandleTypeDef huart2;

SRAM_HandleTypeDef hsram1;
SRAM_HandleTypeDef hsram2;

/* USER CODE BEGIN PV */

OV5640_Object_t   OV5640Obj;

HAL_StatusTypeDef hal_status = HAL_OK;

#define XQUEUE_BUFFER_SIZE 20
ring_buffer_handle_t xQueue_IRQ_Events;
static uint16_t xQueue_IRQ_buffer[XQUEUE_BUFFER_SIZE];


#define   jpeg_buf_size     31*1024
uint32_t  jpeg_data_buf[jpeg_buf_size];

uint16_t pBuffer[240 * 240];


FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
//char SDPath[4]; /* SD card logical drive path */

typedef enum {
  APPLICATION_IDLE = 0,
  APPLICATION_RUNNING,
  APPLICATION_SD_UNPLUGGED,
}FS_FileOperationsTypeDef;

FS_FileOperationsTypeDef Appli_state = APPLICATION_IDLE;

static uint8_t isInitialized = 0;
static uint8_t isCreated = 0;
uint8_t workBuffer[_MAX_SS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DCMI_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FMC_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Error_Handler(void);

int _write(int file, char *ptr, int len) {
	//CDC_Transmit_FS((uint8_t*) ptr, len);
	return HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len,100);
}

static void print_startup_msg(void)
{
  printf("******************************************\r\n");
  printf("*	Project :\t %s       *\r\n", PROJECT_NAME);
  printf("*	Version :\t %s             *\r\n", VERSION);
  printf("*	Date    :\t %s     *\r\n", __DATE__);
  printf("******************************************\r\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t currentMillis = 0;
	static uint32_t previusMillis = 0;

	currentMillis = HAL_GetTick();
	if(currentMillis - previusMillis > 250)
	{
		ring_buffer_put(xQueue_IRQ_Events, GPIO_Pin);
		previusMillis = currentMillis;
	}

}

/**
  * @brief LCD image preparation.
  * @note  Configure image position and size for Discovery STM32L496 LCD
  *        and set LCD in pixel writing mode.
  *        This API must be followed by LCD_Write() call.
  * @param  x0: first pixel x position
  * @param  y0: first pixel y position
  * @param  xSize: image width (in pixels)
  * @param  ySize: image height (in pixels)
  * @retval None
  */


void jpeg_dcmi_frame_callback(DMA_HandleTypeDef *_hdma)
{
        uint8_t *p;
        uint32_t i=0,jpgstart=0,jpglen=0;
        uint8_t  head=0;



        HAL_DCMI_Stop(&hdcmi);

        p=(uint8_t*)jpeg_data_buf;


        FRESULT res; /* FatFs function common result code */
          uint32_t byteswritten; /* File write/read counts */


          /* Register the file system object to the FatFs module */
          if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
          {
        	printf_dbg("f_mount!\r\n");
        #if 1
            if (isCreated == 0)
            {
              res = f_mkfs(SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));

              if (res != FR_OK)
              {
                Error_Handler();
                while(1);
              }
            }
            isCreated = 1;
        #endif
            /* Create and Open a new text file object with write access */
            if(f_open(&MyFile, "STM32.jpeg", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
            {
              printf_dbg("f_open!\r\n");
              /* Write data to the text file */

              for(i=0;i<jpeg_buf_size * 4; i++) //search for 0XFF 0XD8 and 0XFF 0XD9, get size of JPG
			  {
					  if((p[i]==0XFF)&&(p[i+1]==0XD8))
					  {
							  jpgstart=i;
							  head=1;	// Already found  FF D8
					  }
					  if((p[i]==0XFF)&&(p[i+1]==0XD9)&&head)  //search for FF D9
					  {
							  jpglen=i-jpgstart+2;
							  break;
					  }
			  }
			  if(jpglen)
			  {
					  p+=jpgstart;	// move to FF D8
					  //HAL_UART_Transmit(&huart2, p, jpglen, 5000);
					  res = f_write(&MyFile, p, jpglen, (void *)&byteswritten);

					  // printf("jpg_size :  %d \r\n" , jpglen);
					  //printf("jpgstart :  %d \r\n" , jpgstart);
			  }

              res = f_write(&MyFile, jpeg_data_buf, jpeg_buf_size, (void *)&byteswritten);



              printf_dbg("f_write!\r\n");

              if((byteswritten > 0) && (res == FR_OK))
              {
                /* Close the open text file */
                f_close(&MyFile);
                printf_dbg("f_close!\r\n");
              }
            }
          }

        HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)jpeg_data_buf, jpeg_buf_size/4);
}

/**
  * @brief LCD image write-up.
  * @note  Resort to DMA to feed image pixels to STM32L496 Discovery LCD.
  *        LCD is configured in RGB565: one pixel is coded over 16 bits.
  * @param  OrigAddress: Image address
  * @param  DestAddress: LCD RAM address
  * @param  TransferSize: image size (in pixels)
  * @retval HAL status
  */


/**
  * @brief  Initialize the SD card.
  * @param  None
  * @retval None
  */
static void SD_Initialize(void)
{
  if (isInitialized == 0)
  {
    if (BSP_SD_Init() == MSD_OK)
    {
      BSP_SD_ITConfig();
      isInitialized = 1;
      printf_dbg("SD Initialize Okey!\r\n");
    }
    else
    {
      BSP_LED_On(LED1);
      printf_dbg("SD Initialize ERROR!!\r\n");
      /* wait until the uSD is plugged */
      while (BSP_SD_IsDetected() != SD_PRESENT)
      {}
      BSP_LED_Off(LED1);
    }
  }
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	printf_dbg("HAL_DCMI_FrameEventCallback\r\n");


	jpeg_dcmi_frame_callback(&hdma_dcmi);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);   // OFF

	__HAL_DCMI_DISABLE_IT(hdcmi, DCMI_IT_FRAME|DCMI_IT_OVR|DCMI_IT_ERR|DCMI_IT_VSYNC|DCMI_IT_LINE);

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DCMI_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  //MX_DMA2D_Init();
  MX_FMC_Init();
  MX_SDMMC1_SD_Init();
  //MX_FATFS_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  xQueue_IRQ_Events = ring_buffer_init(xQueue_IRQ_buffer, XQUEUE_BUFFER_SIZE);

  print_startup_msg();

  /* Initialize the IO functionalities */
  uint8_t ret;
  ret = BSP_IO_Init();
  if(ret == IO_OK)  printf_dbg("Expander OK\r\n");


  BSP_CAMERA_Init(RESOLUTION_R320x240);
  if(ret == CAMERA_OK) printf_dbg("Camera OK\r\n");

  /* Wait 1s to let auto-loops in the camera module converge and lead to correct exposure */
  HAL_Delay(1000);

  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_LINE | DCMI_IT_VSYNC);
  __HAL_DCMI_DISABLE_IT(&hdcmi, DCMI_IT_FRAME|DCMI_IT_OVR|DCMI_IT_ERR|DCMI_IT_VSYNC|DCMI_IT_LINE);

  hal_status = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)jpeg_data_buf, jpeg_buf_size/4 );
  if(hal_status != HAL_OK)Error_Handler();


  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
	SD_Initialize();

	if(BSP_SD_IsDetected())
	{
		printf_dbg("SD Card OK\r\n");
	}

   }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static uint16_t IRQ_event;
	  if(ring_buffer_get(xQueue_IRQ_Events, &IRQ_event))
	  {

		  switch (IRQ_event) {
			case GPIO_PIN_8:
				printf_dbg("JOY_UP was pressed!\r\n");
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);   // ON

				break;
			case GPIO_PIN_9:
				printf_dbg("JOY_LEFT was pressed!\r\n");

				break;
			case GPIO_PIN_10:
				printf_dbg("JOY_DOWN was pressed!\r\n");
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);   // OFF

				break;
			case GPIO_PIN_11:
				printf_dbg("JOY_RIGHT was pressed!\r\n");


				break;
			case GPIO_PIN_13:
				printf_dbg("JOY_SEL was pressed!\r\n");

				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);   // ON

				__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME|DCMI_IT_OVR|DCMI_IT_ERR|DCMI_IT_VSYNC|DCMI_IT_LINE);

				break;
			default:
				break;
		}

	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_1);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DCMI_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DCMI_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_ENABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}


/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK2;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_DISABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the SRAM2 memory initialization sequence
  */
  hsram2.Instance = FMC_NORSRAM_DEVICE;
  hsram2.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram2.Init */
  hsram2.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram2.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram2.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram2.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram2.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram2.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram2.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram2.Init.WriteOperation = FMC_WRITE_OPERATION_DISABLE;
  hsram2.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram2.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram2.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram2.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram2.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram2.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram2.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram2, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, OV5640_PWDN_Pin|OV5640_SIOC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, MFX_WAKEUP_Pin|LCD_PWR_ON_Pin|MIC_VDD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Audio_RST_GPIO_Port, Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OV5640_SIOD_Pin|LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : JOY_DOWN_Pin JOY_LEFT_Pin JOY_UP_Pin */
  GPIO_InitStruct.Pin = JOY_DOWN_Pin|JOY_LEFT_Pin|JOY_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : OV5640_PWDN_Pin */
  GPIO_InitStruct.Pin = OV5640_PWDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OV5640_PWDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USART1_TX_Pin */
  GPIO_InitStruct.Pin = USART1_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(USART1_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI1_FSA_Pin SAI1_SDB_Pin SAI1_MCKA_Pin SAI1_SDA_Pin */
  GPIO_InitStruct.Pin = SAI1_FSA_Pin|SAI1_SDB_Pin|SAI1_MCKA_Pin|SAI1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D5_Pin */
  GPIO_InitStruct.Pin = ARD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(ARD_D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UART1_RX_Pin UART1_CTS_Pin UART1_RTS_Pin */
  GPIO_InitStruct.Pin = UART1_RX_Pin|UART1_CTS_Pin|UART1_RTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_TE_Pin */
  GPIO_InitStruct.Pin = LCD_TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTGFS_DP_Pin USB_OTGFS_DM_Pin USB_OTGFS_ID_Pin */
  GPIO_InitStruct.Pin = USB_OTGFS_DP_Pin|USB_OTGFS_DM_Pin|USB_OTGFS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : JOY_SEL_Pin */
  GPIO_InitStruct.Pin = JOY_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(JOY_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MFX_WAKEUP_Pin LCD_PWR_ON_Pin MIC_VDD_Pin */
  GPIO_InitStruct.Pin = MFX_WAKEUP_Pin|LCD_PWR_ON_Pin|MIC_VDD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_RST_Pin */
  GPIO_InitStruct.Pin = Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Audio_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_INT_Pin */
  GPIO_InitStruct.Pin = CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTP_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DATIN3_Pin DF_CKOUT_Pin */
  GPIO_InitStruct.Pin = DATIN3_Pin|DF_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_A3_Pin */
  GPIO_InitStruct.Pin = ARD_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_A3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A0_Pin ARD_A5_Pin PC1 ARD_A2_Pin */
  GPIO_InitStruct.Pin = ARD_A0_Pin|ARD_A5_Pin|GPIO_PIN_1|ARD_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_BK1_NCS_Pin QSPI_BK1_IO1_Pin QSPI_BK1_IO0_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_NCS_Pin|QSPI_BK1_IO1_Pin|QSPI_BK1_IO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D0_Pin ARD_D1_Pin */
  GPIO_InitStruct.Pin = ARD_D0_Pin|ARD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : MFX_IRQ_OUT_Pin */
  GPIO_InitStruct.Pin = MFX_IRQ_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MFX_IRQ_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D13_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(ARD_D13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OV5640_SIOC_Pin */
  GPIO_InitStruct.Pin = OV5640_SIOC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(OV5640_SIOC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STMOD_ADC_Pin ARD_A4_Pin */
  GPIO_InitStruct.Pin = STMOD_ADC_Pin|ARD_A4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_BK1_IO2_Pin QSPI_CLK_Pin QSPI_BK1_IO3_Pin */
  GPIO_InitStruct.Pin = QSPI_BK1_IO2_Pin|QSPI_CLK_Pin|QSPI_BK1_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : JOY_RIGHT_Pin */
  GPIO_InitStruct.Pin = JOY_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(JOY_RIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI2_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SPI2_MOSI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OV5640_SIOD_Pin */
  GPIO_InitStruct.Pin = OV5640_SIOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(OV5640_SIOD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STMOD_RESET_Pin */
  GPIO_InitStruct.Pin = STMOD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STMOD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DFDATIN1_Pin */
  GPIO_InitStruct.Pin = DFDATIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(DFDATIN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	printf_dbg("!!!!!!!!Error_Handler!!!!!!!");
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
