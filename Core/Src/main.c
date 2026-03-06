/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>  // 引入printf所需头文件
#include "stm32f1xx_ll_fsmc.h"  // FSMC核心头文件（F1系列）
#include "fatfs.h"  // FATFS核心头文件（工程已包含）
#include "stm32f1xx_hal.h"  // 工程必有的HAL库头文件（包含SDIO基础定义）
#include "stm32f1xx_hal_rtc.h"  // RTC HAL库头文件

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 新增：TF卡作为USB MSC的块设备参数
#define TF_BLOCK_SIZE     512    // SD卡默认块大小（固定512字节）
#define TF_MAX_BLOCK_NUM  (1024*1024*4) // 4GB TF卡最大块数（可根据实际卡调整）
extern SD_HandleTypeDef hsd;     // 引用SDIO的SD卡句柄

uint32_t g_tf_block_num = 0;
uint16_t g_tf_block_size = 0;

// RTC句柄定义
RTC_HandleTypeDef hrtc;

// 按键定义：KEY_CANCEL（对应PC5，外部上拉，低电平有效）
#define KEY_CANCEL_PIN    CANCEL_Pin
#define KEY_CANCEL_PORT   GPIOC
// LED定义：LEDG（对应PB0的LED_G_Pin）
#define LEDG_PIN          LED_G_Pin
#define LEDG_PORT         GPIOB

// ********** AM29LV320 FSMC 配置 **********
#define FSMC_NOR_BASE_ADDR    ((uint16_t *)0x60000000)  // FSMC BANK1基地址(半字访问，AM29LV320为16位宽)
#define AM29LV320_CMD_READ_ID 0x90                      // JEDEC ID读取指令
#define AM29LV320_ADDR_CMD    0x0000                    // 指令写入地址
// AM29LV320 ID存储变量
uint16_t am29lv320_mfg_id = 0;    // 制造商ID
uint16_t am29lv320_dev_id = 0;    // 设备ID
uint16_t am29lv320_cap_id = 0;    // 容量ID

// 新增：AM29LV320数据读取相关定义
#define AM29LV320_RESET_CMD   0xFFFF                    // 复位指令（退出ID模式）
#define AM29LV320_ADDR_0x0000 0x0000                    // 0x0000字节地址对应的半字偏移

// 新增：AM29LV320 CFI容量读取相关定义
#define AM29LV320_CFI_ENTRY_ADDR  0x0055                // 进入CFI模式的半字偏移地址（对应字节0x00AA）
#define AM29LV320_CFI_ENTRY_DATA  0x0098                // 进入CFI模式的写入数据
#define AM29LV320_CFI_CAP_ADDR    0x0027                // 读取CFI容量的半字偏移地址（对应字节0x004E）

// 定义RY/BY#引脚（PD6）相关宏
#define RY_BY_PIN        GPIO_PIN_6
#define RY_BY_PORT       GPIOD

#define SYSTEM_CORE_CLOCK 72000000UL  // 系统主频（需与工程配置一致）
#define CYCLE_PER_LOOP    3UL         // 单循环指令数（实测校准）
#define MIN_DELAY_NS      10          // 最小可靠延迟（ns）

/* 全局静态变量：存储初始化后的固定参数（仅初始化一次） */
static uint32_t g_ns_per_cycle = 0;   // 单指令周期耗时（ns）
static uint32_t g_tick_step_us = 0;   // Tick递增步长（us，仅作为备用参数）

// 新增：文件信息结构体（存储.bin文件名称+大小）
typedef struct {
    char filename[256];  // 文件名（兼容长文件名）
    uint32_t filesize;   // 文件大小（字节）
} AM29_File_Info_t;

// 新增：全局文件列表缓存（最多存储32个.bin文件，可根据需求调整）
#define MAX_BIN_FILE_CNT 32
static AM29_File_Info_t g_am29_bin_files[MAX_BIN_FILE_CNT] = {0};
static uint8_t g_am29_bin_file_count = 0;  // 实际找到的.bin文件数量

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

NOR_HandleTypeDef hnor1;
NAND_HandleTypeDef hnand2;

/* USER CODE BEGIN PV */
HAL_SD_CardInfoTypeDef card_info;  // 新增：用于存储TF卡信息

// 添加USB设备句柄的外部声明
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void AM29_Get_Id_And_Read10(void);
static void UART_Menu_Show(void);
static uint8_t UART_Read_User_Input(void);
static void AM29_Erase_And_Verify(void);
static void AM29_Write_And_Verify(void);
static void switch_to_udisk(void);

uint8_t KEY_CANCEL_Detect(void);
uint32_t AM29LV320_Read_ID(void);  // 读取AM29的ID

static void MX_RTC_Init(void); //RTC初始化声明

void AM29LV320_Read_Data(uint16_t addr_halfword, uint16_t len, uint16_t *buf);
uint16_t AM29LV320_Read_0x0000(void); // 快速读取0x0000地址内容

// 新增：CFI容量读取函数声明
uint16_t AM29LV320_Read_CFI_Capacity(void); // 读取CFI容量原始值
uint32_t AM29LV320_Parse_CFI_Capacity(uint16_t cfi_value); // 解析CFI容量值为实际字节数


uint8_t AM29LV320_Chip_Erase(void); // 整芯片擦除函数声明

void delayns_init(void);
void delayns(uint32_t ns);
void delaycmd(void);

AM29_File_Info_t* AM29_List_Bin_Files(uint8_t *file_count);

uint8_t AM29LV320_Write_Data(uint32_t addr, uint16_t data);
uint8_t AM29LV320_Write_Data_From_File(const char *filename);
uint8_t AM29LV320_Write_Data_From_File_Little_End_For_MD(const char *filename);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 手动定义SDIO所需的核心常量和句柄（替代sdio.h）
SD_HandleTypeDef hsd; // SDIO句柄（全局，和工程其他部分兼容）

// SDIO时钟分频、总线宽度等常量（和HAL库定义一致）
#define SDIO_CLOCK_EDGE_RISING         0x00000000U
#define SDIO_CLOCK_BYPASS_DISABLE      0x00000000U
#define SDIO_CLOCK_POWER_SAVE_DISABLE  0x00000000U
#define SDIO_BUS_WIDE_1B               0x00000000U
#define SDIO_HARDWARE_FLOW_CONTROL_DISABLE 0x00000000U
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
	delayns_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  //MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	MX_RTC_Init();

  // 启动时强制将USB D+引脚拉低，防止主机检测到设备
  printf("初始化USB引脚为低电平，防止提前检测...\r\n");
  
  // 将PA12配置为输出推挽，并输出低电平
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_12;  // 只配置D+引脚
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);  // 拉低D+
  
  // PA11配置为输入浮空（D-）
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // 启动时通过USART1发送hello（波特率115200）
  printf("AM29 Programmer By motozilog V1.0\r\n");
	
/* TF START */
// 初始化后检测TF卡状态
printf("SDIO状态：%d\r\n", HAL_SD_GetState(&hsd));

// 读取真实卡信息
if(HAL_SD_GetCardInfo(&hsd, &card_info) == HAL_OK)
{
    // 优先使用逻辑块（兼容USB MSC）
    if(card_info.LogBlockNbr > 0 && card_info.LogBlockSize > 0)
    {
        g_tf_block_num = card_info.LogBlockNbr;
        g_tf_block_size = card_info.LogBlockSize;
    }
    // 备用：逻辑块为0时用物理块
    else if(card_info.BlockNbr > 0 && card_info.BlockSize > 0)
    {
        g_tf_block_num = card_info.BlockNbr;
        g_tf_block_size = card_info.BlockSize;
    }
    // 打印真实容量（验证）
    uint64_t real_bytes = (uint64_t)g_tf_block_num * g_tf_block_size;
    uint32_t real_mb = (uint32_t)(real_bytes / 1024 / 1024);
    printf("SDIO读取真实容量：%u MB\r\n", real_mb);
}
else
{
    printf("读取TF卡信息失败，使用默认容量（谨慎！）\r\n");
    // 设置默认容量以防万一
    g_tf_block_num = 0;
    g_tf_block_size = 0;
}
/* TF END */


	
		// 替换原有调用：获取文件列表并打印
		uint8_t bin_file_cnt = 0;
		AM29_File_Info_t* bin_file_list = AM29_List_Bin_Files(&bin_file_cnt);

		// 可选：遍历返回的文件列表（验证功能）
		if(bin_file_list != NULL && bin_file_cnt > 0) {
				printf("===== 验证返回的文件列表 =====\r\n");
				for(uint8_t i=0; i<bin_file_cnt; i++) {
						printf("列表第%d项：%s | %lu字节\r\n", 
									 i+1, bin_file_list[i].filename, (unsigned long)bin_file_list[i].filesize);
				}
		}	


		
		//写入测试
//		uint8_t write_ret = AM29LV320_Write_Data(0x60000000, 0x1234);
//		if (write_ret == 0)
//		{
//				printf("写入测试成功！\r\n");
//		}
//		else
//		{
//				printf("写入测试失败，错误码：%d\r\n", write_ret);
//		}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
		    uint8_t user_choice = 0;
    
    // 仅在首次进入/操作完成后显示一次菜单
    UART_Menu_Show();
    
    // 循环读取输入，直到获取有效选择
    while(user_choice == 0)
    {
        user_choice = UART_Read_User_Input();
			  UART_Menu_Show();
    }
    
    // 根据选择执行对应操作
    switch(user_choice)
    {
        case 1:
            AM29_Get_Id_And_Read10();
            break;			
        case 2:
            AM29_Erase_And_Verify();
            break;            
        case 3:
            AM29_Write_And_Verify();
            break;
				case 4:
            switch_to_udisk();
            break;
        default:
            printf("未知选项，请重新选择\r\n");
            break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */
// 确保SDIO电源稳定
__HAL_RCC_SDIO_CLK_ENABLE();
__HAL_RCC_GPIOC_CLK_ENABLE();  // SDIO数据线使用的GPIO

// 配置SDIO引脚为高速度
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_PULLUP;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

  /* USER CODE BEGIN SDIO_Init 2 */
  hsd.Init.ClockDiv = 8;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
		printf("TF卡未检测到（未插入？）\r\n");
    //Error_Handler();
  }
	else
	{
		printf("TF卡己插入\r\n");
	}
	
	// 尝试切换到4位模式（可选）
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) == HAL_OK) {
      printf("切换到4位模式成功\r\n");
    } else {
      printf("保持1位模式\r\n");
    }
	
  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_G_Pin|LED_R_Pin|A27_Pin|A28_Pin
                          |A29_Pin|A30_Pin|A31_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A26_GPIO_Port, A26_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : UP_Pin DOWN_Pin LEFT_Pin RIGHT_Pin
                           OK_Pin CANCEL_Pin */
  GPIO_InitStruct.Pin = UP_Pin|DOWN_Pin|LEFT_Pin|RIGHT_Pin
                          |OK_Pin|CANCEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_G_Pin LED_R_Pin A27_Pin A28_Pin
                           A29_Pin A30_Pin A31_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_R_Pin|A27_Pin|A28_Pin
                          |A29_Pin|A30_Pin|A31_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SDIO_CD_Pin */
  GPIO_InitStruct.Pin = SDIO_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDIO_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A26_Pin */
  GPIO_InitStruct.Pin = A26_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A26_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};
  FSMC_NAND_PCC_TimingTypeDef ComSpaceTiming = {0};
  FSMC_NAND_PCC_TimingTypeDef AttSpaceTiming = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the NOR1 memory initialization sequence
  */
  hnor1.Instance = FSMC_NORSRAM_DEVICE;
  hnor1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hnor1.Init */
  hnor1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hnor1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hnor1.Init.MemoryType = FSMC_MEMORY_TYPE_NOR;
  hnor1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hnor1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hnor1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hnor1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hnor1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hnor1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hnor1.Init.WaitSignal = FSMC_WAIT_SIGNAL_ENABLE;
  hnor1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hnor1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_ENABLE;
  hnor1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_NOR_Init(&hnor1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Perform the NAND2 memory initialization sequence
  */
  hnand2.Instance = FSMC_NAND_DEVICE;
  /* hnand2.Init */
  hnand2.Init.NandBank = FSMC_NAND_BANK3;
  hnand2.Init.Waitfeature = FSMC_NAND_PCC_WAIT_FEATURE_ENABLE;
  hnand2.Init.MemoryDataWidth = FSMC_NAND_PCC_MEM_BUS_WIDTH_8;
  hnand2.Init.EccComputation = FSMC_NAND_ECC_DISABLE;
  hnand2.Init.ECCPageSize = FSMC_NAND_ECC_PAGE_SIZE_256BYTE;
  hnand2.Init.TCLRSetupTime = 0;
  hnand2.Init.TARSetupTime = 0;
  /* hnand2.Config */
  hnand2.Config.PageSize = 0;
  hnand2.Config.SpareAreaSize = 0;
  hnand2.Config.BlockSize = 0;
  hnand2.Config.BlockNbr = 0;
  hnand2.Config.PlaneNbr = 0;
  hnand2.Config.PlaneSize = 0;
  hnand2.Config.ExtraCommandEnable = DISABLE;
  /* ComSpaceTiming */
  ComSpaceTiming.SetupTime = 252;
  ComSpaceTiming.WaitSetupTime = 252;
  ComSpaceTiming.HoldSetupTime = 252;
  ComSpaceTiming.HiZSetupTime = 252;
  /* AttSpaceTiming */
  AttSpaceTiming.SetupTime = 252;
  AttSpaceTiming.WaitSetupTime = 252;
  AttSpaceTiming.HoldSetupTime = 252;
  AttSpaceTiming.HiZSetupTime = 252;

  if (HAL_NAND_Init(&hnand2, &ComSpaceTiming, &AttSpaceTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

// 串口菜单交互逻辑（放在BEGIN 4：函数定义区）
static void UART_Menu_Show(void)
{
    printf("\r\n==================== AM29LV320 操作菜单 ====================\r\n");
    printf("1 - 获取AM29 ID并读取前10字\r\n");
		printf("2 - 整芯片擦除并查空验证\r\n");
    printf("3 - 写入BIN文件并校验\r\n");
    printf("4 - 切换到U盘模式（作为U盘连接电脑）\r\n");

    printf("============================================================\r\n");
    printf("请输入操作编号，按回车确认：");
}

static void switch_to_udisk(void)
{
    uint8_t dummy;
    uint32_t tickstart;
    USBD_StatusTypeDef status;
    
    printf("\r\n===== 切换到U盘模式 =====\r\n");
    
    // 检查USB时钟状态
    printf("检查USB时钟状态...\r\n");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY)) {
        printf("HSE 就绪\r\n");
    }
    
    // 使能USB时钟
    printf("使能USB时钟...\r\n");
    __HAL_RCC_USB_CLK_ENABLE();
    
    // 配置USB中断优先级
    printf("配置USB中断...\r\n");
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 1, 0);
    
    // 配置USB引脚
    printf("配置USB引脚...\r\n");
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // USB DP (PA12) 和 DM (PA11)
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    printf("正在初始化USB设备...\r\n");
    
    // 先反初始化USB（如果之前初始化过）
    // USBD_DeInit(&hUsbDeviceFS);
    // HAL_Delay(100);
    
    // 调用USB初始化前的准备工作
    printf("准备调用 MX_USB_DEVICE_Init...\r\n");
    
    // 禁用中断，防止初始化过程中的中断干扰
    __disable_irq();
    
    // 初始化USB设备
    MX_USB_DEVICE_Init();
    
    // 重新使能中断
    __enable_irq();
    
    printf("MX_USB_DEVICE_Init 调用完成\r\n");
    
    // 使能USB中断
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    
    printf("等待USB枚举完成...\r\n");
    tickstart = HAL_GetTick();
    
    while ((HAL_GetTick() - tickstart) < 10000) {  // 等待10秒
        HAL_Delay(500);
        printf("等待中... (状态: %d)\r\n", hUsbDeviceFS.dev_state);
        
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            printf("USB设备已配置！\r\n");
            break;
        }
    }
    
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        printf("USB设备已初始化，现在可以作为U盘连接到电脑\r\n");
    } else {
        printf("USB设备状态异常: %d\r\n", hUsbDeviceFS.dev_state);
    }
    
    
    printf("请在电脑上操作U盘，弹出U盘时将重启...\r\n");
    
    // ========== 修正的弹出检测逻辑 ==========
    uint32_t start_time = HAL_GetTick();
    uint32_t last_usb_state = hUsbDeviceFS.dev_state;
    uint32_t suspend_detected_time = 0;
    uint8_t eject_detected = 0;
    
    // USB状态定义 (来自 usbd_def.h)
    // #define USBD_STATE_DEFAULT     1
    // #define USBD_STATE_ADDRESSED   2
    // #define USBD_STATE_CONFIGURED  3
    // #define USBD_STATE_SUSPENDED   4
    
    while(!eject_detected) {
        // 检查是否有按键输入
//        if (HAL_UART_Receive(&huart1, &dummy, 1, 100) == HAL_OK) {
//            printf("检测到按键输入，返回主菜单\r\n");
//            break;
//        }
        
        // 检查USB状态变化
        uint32_t current_state = hUsbDeviceFS.dev_state;
        
        if (current_state != last_usb_state) {
            printf("USB状态变化: %u -> %u\r\n", last_usb_state, current_state);
            
            // 检测到挂起状态 (3 -> 4)
            if (last_usb_state == USBD_STATE_CONFIGURED && 
                current_state == USBD_STATE_SUSPENDED) {
                printf("检测到USB挂起，可能是Windows弹出操作\r\n");
                suspend_detected_time = HAL_GetTick();
            }
            
            // 检测到断开 (从任何状态变为0或1)
            if (last_usb_state >= USBD_STATE_CONFIGURED && 
                current_state < USBD_STATE_ADDRESSED) {
                printf("检测到USB断开连接！\r\n");
                eject_detected = 1;
                break;
            }
            
            last_usb_state = current_state;
        }
        
        // 如果在挂起状态持续超过2秒，认为是弹出操作
        if (suspend_detected_time > 0) {
            uint32_t suspend_duration = HAL_GetTick() - suspend_detected_time;
            
            // 检查是否从挂起恢复
            if (current_state == USBD_STATE_CONFIGURED) {
                printf("USB从挂起恢复，继续等待\r\n");
                suspend_detected_time = 0;
            }
            // 挂起持续2秒后，触发重启
            else if (suspend_duration > 2000) {
                printf("USB挂起超过2秒，确认为弹出操作\r\n");
                eject_detected = 1;
                break;
            }
        }
        
        // 超时检查（60秒无操作自动重启）- 只在配置状态且无挂起时计时
//        if (!suspend_detected_time && current_state == USBD_STATE_CONFIGURED) {
//            if ((HAL_GetTick() - start_time) > 60000) { // 60秒无操作自动重启
//                printf("U盘模式超时，自动重启...\r\n");
//                eject_detected = 1;
//                break;
//            }
//        }
        
        HAL_Delay(100);
    }
    
    // 触发系统重启
    if (eject_detected) {
        printf("检测到弹出操作或无操作超时，系统将在3秒后重启...\r\n");
        
        // 关闭所有外设
        printf("关闭外设...\r\n");
        HAL_UART_DeInit(&huart1);
        HAL_SD_DeInit(&hsd);
        
        // 等待3秒，让用户看到提示
        HAL_Delay(3000);
        
        printf("系统重启中...\r\n");
        HAL_Delay(100);
        
        // 执行系统重启
        NVIC_SystemReset();
    }
    
    printf("返回主菜单，USB连接保持\r\n");
}

// 串口读取用户输入（阻塞式，直到回车）
static uint8_t UART_Read_User_Input(void)
{
    uint8_t input_buf[16] = {0};
    uint8_t input_len = 0;
    uint8_t ch = 0;
    
    while(1)
    {
        if(HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY) == HAL_OK)
        {
            // 处理退格键（BackSpace，ASCII码：0x08 或 0x7F）
            if((ch == 0x08) || (ch == 0x7F)) 
            {
                if(input_len > 0) // 有字符可删时才处理
                {
                    input_len--;                  // 输入长度减1
                    input_buf[input_len] = 0;     // 清空最后一个字符
                    
                    // 串口回显退格+空格+退格（实现“删除”视觉效果）
                    uint8_t backspace_seq[3] = {0x08, 0x20, 0x08};
                    HAL_UART_Transmit(&huart1, backspace_seq, 3, HAL_MAX_DELAY);
                }
                continue; // 跳过后续逻辑，继续读取字符
            }
            // 处理回车确认（换行/回车都识别）
            else if(ch == '\r' || ch == '\n') 
            {
                printf("\r\n");
                break;
            }
            // 处理有效字符（仅数字1/2，限制输入长度）
            else if(input_len < sizeof(input_buf)-1 && (ch == '1' || ch == '2' || ch == '3' || ch == '4')) 
            {
                input_buf[input_len++] = ch;
                HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY); // 回显有效字符
            }
            // 其他无效字符（不回显、不存储）
            else
            {
                continue;
            }
        }
    }
    
    // 解析输入
     if(input_len == 1)
    {
        if(input_buf[0] == '1') return 1;
        if(input_buf[0] == '2') return 2;
        if(input_buf[0] == '3') return 3;
        if(input_buf[0] == '4') return 4;
        if(input_buf[0] == '5') return 5;
        if(input_buf[0] == '6') return 6;
    }
    
    // 无效输入提示
    printf("输入无效！请重新输入并按回车确认：");
    return 0;
}

// 擦除并查空验证（独立函数封装）
static void AM29_Get_Id_And_Read10(void)
{
	  uint32_t am29lv320_id_32bit = 0;

		    // 可选：读取0x0000开始的多个数据（示例读取前5个半字）
    uint16_t data_buf[5] = {0};
    AM29LV320_Read_Data(0, 5, data_buf);
    printf("AM29LV320 0x0000~0x0009 Data: ");
    for(int i=0; i<5; i++)
    {
      printf("0x%04X ", data_buf[i]);
    }
    printf("\r\n");

			// 读取AM29LV320 ID
			am29lv320_id_32bit = AM29LV320_Read_ID();
			printf("AM29LV320 ID (32bit): 0x%08X\r\n", am29lv320_id_32bit);

		// 读取并打印CFI容量信息
		uint16_t cfi_cap_val = AM29LV320_Read_CFI_Capacity();
		uint32_t actual_cap = AM29LV320_Parse_CFI_Capacity(cfi_cap_val);
		printf("AM29LV320 CFI Capacity Raw Value: 0x%04X\r\n", cfi_cap_val);
		if(actual_cap > 0)
		{
				printf("AM29LV320 Actual Capacity: %lu Bytes (%lu MB)\r\n", 
							 (unsigned long)actual_cap, (unsigned long)((unsigned long)actual_cap / 1024 / 1024));
		}
		else
		{
				printf("AM29LV320 CFI Capacity: Unknown\r\n");
		}
}


// 擦除并查空验证（独立函数封装）
static void AM29_Erase_And_Verify(void)
{
    uint8_t erase_ret = AM29LV320_Chip_Erase();
    if(erase_ret == 0)
    {
        // 擦除成功后，读取0x0000地址验证（擦除后应为0xFFFF）
        uint16_t data = AM29LV320_Read_0x0000();
        printf("Erase Verify 0x0000: 0x%04X\r\n", data);
    }    
}

// 写入BIN文件并校验（独立函数封装）
static void AM29_Write_And_Verify(void)
{
		uint8_t file_write_ret = AM29LV320_Write_Data_From_File("M04401~1.BIN");
		if (file_write_ret == 0)
		{
				printf("===== M04401~1.BIN 写入成功 =====\r\n");
		}
		else
		{
				printf("===== M04401~1.BIN 写入失败，错误码：%d =====\r\n", file_write_ret);
		}
}

static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  // 启用PWR时钟和BKP时钟
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();

  // 取消写保护（必须解锁才能配置RTC）
  HAL_PWR_EnableBkUpAccess();

  // 配置RTC时钟源为内部RC（LSI）
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // 将RTC时钟源设置为LSI
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // 启用RTC时钟
  __HAL_RCC_RTC_ENABLE();

  // 初始化RTC
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;  // 自动分频，1秒计数
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  // 初始化RTC时间（初始时间设为00:00:00）
  sTime.Hours = 0x00;
  sTime.Minutes = 0x00;
  sTime.Seconds = 0x00;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  // 初始化RTC日期（初始日期设为2024-01-01）
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x01;
  DateToUpdate.Year = 0x24;
  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
}

// 辅助函数：判断闰年（用于计算日期总天数）
static uint8_t is_leap_year(uint16_t year)
{
    // 闰年规则：能被4整除但不能被100整除，或能被400整除
    if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
        return 1;
    else
        return 0;
}

// 每个月的天数（非闰年）
static const uint8_t month_days[] = {31,28,31,30,31,30,31,31,30,31,30,31};

// 核心函数：将RTC时间（日期+时间）转换为Unix时间戳（秒）
uint32_t rtc_to_unix_timestamp(RTC_DateTypeDef *date, RTC_TimeTypeDef *time)
{
    uint32_t total_seconds = 0;
    uint16_t year = 2000 + date->Year; // 假设RTC的Year是两位（如24=2024），根据实际配置调整
    uint8_t month = date->Month;
    uint8_t day = date->Date;
    
    // 1. 计算从1970年到当前年份前一年的总秒数
    for (uint16_t y = 1970; y < year; y++)
    {
        total_seconds += is_leap_year(y) ? 31622400 : 31536000; // 闰年366天=31622400秒，平年365天=31536000秒
    }
    
    // 2. 计算当前年份到当前月份前一个月的总秒数
    for (uint8_t m = 1; m < month; m++)
    {
        total_seconds += month_days[m-1] * 86400; // 每个月的天数×86400秒/天
        // 闰年且月份>2，额外加1天的秒数
        if (m == 2 && is_leap_year(year))
            total_seconds += 86400;
    }
    
    // 3. 计算当前日期的秒数（天数-1，因为1号是0天）
    total_seconds += (day - 1) * 86400;
    
    // 4. 计算当天的时分秒转换为秒
    total_seconds += time->Hours * 3600 + time->Minutes * 60 + time->Seconds;
    
    return total_seconds;
}

// HAL RTC底层初始化（必须实现）
void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{
  if(rtcHandle->Instance==RTC)
  {
    __HAL_RCC_RTC_ENABLE();
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{
  if(rtcHandle->Instance==RTC)
  {
    __HAL_RCC_RTC_DISABLE();
  }
}

// 重定向fputc函数，使printf输出到USART1
int fputc(int ch, FILE *f)
{
  // 发送一个字节数据到USART1
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

uint8_t KEY_CANCEL_Detect(void)
{
  uint8_t key_flag = 0;
  // 1. 检测按键是否为低电平（按下）
  if(HAL_GPIO_ReadPin(KEY_CANCEL_PORT, KEY_CANCEL_PIN) == GPIO_PIN_RESET)
  {
    // 2. 10ms消抖（过滤物理抖动）
    HAL_Delay(10);
    if(HAL_GPIO_ReadPin(KEY_CANCEL_PORT, KEY_CANCEL_PIN) == GPIO_PIN_RESET)
    {
      key_flag = 1;
      // 3. 等待按键释放（避免长按重复触发）
      while(HAL_GPIO_ReadPin(KEY_CANCEL_PORT, KEY_CANCEL_PIN) == GPIO_PIN_RESET);
    }
  }
  return key_flag;
}

// 读取AM29LV320的JEDEC ID，返回32位数据：[31:16]=制造商ID，[15:0]=设备ID
uint32_t AM29LV320_Read_ID(void)  
{
  // 步骤1：强制复位芯片，退出所有特殊模式（确保初始状态）
  *(FSMC_NOR_BASE_ADDR + 0x00) = 0xFFFF;
  HAL_Delay(1);

  // 步骤2：发送读ID指令的解锁序列（严格按字节地址→16位偏移转换）
  // 解锁序列1：字节地址0x555 → 16位偏移0x2AA → 写0x00AA（低字节AA，高字节00）
  *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00AA;
  delaycmd();
  // 解锁序列2：字节地址0x2AA → 16位偏移0x155 → 写0x0055（低字节55，高字节00）
  *(FSMC_NOR_BASE_ADDR + 0x2AA) = 0x0055;
  delaycmd();
  // 解锁序列3：字节地址0x555 → 16位偏移0x2AA → 写0x0090（读ID指令）
  *(FSMC_NOR_BASE_ADDR + 0x555) = 0x0090;
  delaycmd();

  // 步骤3：读取ID（16位总线偏移对应字节地址）
  am29lv320_mfg_id = *(FSMC_NOR_BASE_ADDR + 0x00); // 字节0x0000 → 制造商ID (0x0122)
  am29lv320_dev_id = *(FSMC_NOR_BASE_ADDR + 0x01); // 字节0x0001 → 设备ID (0x00A4)
  am29lv320_cap_id = *(FSMC_NOR_BASE_ADDR + 0x02); // 字节0x0002 → 容量ID (0x0080)

  // 步骤4：发送复位指令，退出ID模式，回到正常数据读取模式
  *(FSMC_NOR_BASE_ADDR + 0x00) = 0xFFFF;
  HAL_Delay(1);

  // 组合32位ID返回
  return ((uint32_t)am29lv320_mfg_id << 16) | am29lv320_dev_id;
}

// 新增：读取AM29LV320指定地址的内容（默认读取0x0000地址）
// 参数：addr_halfword - 半字偏移地址（0x0000字节地址对应0，0x0002对应1，以此类推）
//       len          - 读取的半字数量（1个半字=2字节）
//       buf          - 存储读取数据的缓冲区（uint16_t类型）
void AM29LV320_Read_Data(uint16_t addr_halfword, uint16_t len, uint16_t *buf)
{
  if (buf == NULL || len == 0) return; // 入参校验

  // 步骤1：发送复位指令，确保芯片处于正常读模式（避免ID模式干扰）
  *(FSMC_NOR_BASE_ADDR + 0x00) = 0xFFFF;
  HAL_Delay(1);

  // 步骤2：读取指定地址的数据（16位总线直接访问）
  for (uint16_t i = 0; i < len; i++)
  {
    // 半字偏移累加，直接读取FSMC映射地址
    buf[i] = *(FSMC_NOR_BASE_ADDR + addr_halfword + i);
    // 可选：添加短延时，适配低速芯片（如AM29LV320读时序要求）
    //HAL_Delay(1);
  }
}

// 封装：快速读取0x0000地址的单个半字数据（简化调用）
uint16_t AM29LV320_Read_0x0000(void)
{
  uint16_t data;
  // 0x0000字节地址 → 半字偏移0
  AM29LV320_Read_Data(0, 1, &data);
  return data;
}


// 新增：从CFI中读取AM29LV320的容量原始值
uint16_t AM29LV320_Read_CFI_Capacity(void)
{
    uint16_t cfi_capacity = 0;
    
    // 步骤1：复位芯片，退出所有特殊模式（确保初始状态）
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29LV320_RESET_CMD;
    HAL_Delay(1);  // 确保复位指令生效
    
    // 步骤2：发送进入CFI模式的命令（ADDR:0x0055 DATA:0x0098）
    *(FSMC_NOR_BASE_ADDR + AM29LV320_CFI_ENTRY_ADDR) = AM29LV320_CFI_ENTRY_DATA;
    delaycmd();
    
    // 步骤3：读取0x0027地址的容量值
    cfi_capacity = *(FSMC_NOR_BASE_ADDR + AM29LV320_CFI_CAP_ADDR);
    delaycmd();
    
    // 步骤4：复位芯片，退出CFI模式，恢复正常读模式
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29LV320_RESET_CMD;
    HAL_Delay(1);
    
    return cfi_capacity;
}

// 新增：解析CFI容量值为实际字节容量
uint32_t AM29LV320_Parse_CFI_Capacity(uint16_t cfi_value)
{
 if (cfi_value == 0) return 0; // 异常值保护
    return (uint32_t)1 << cfi_value; // 2^exponent 计算（位运算高效）
}


/**
 * @brief  AM29LV320 芯片整片擦除函数
 * @note   严格遵循AM29LV320数据手册的擦除时序和状态检测逻辑
 * @retval uint8_t 擦除结果
 *         0 - 擦除成功
 *         1 - 擦除超时
 *         2 - 进入擦除状态失败/状态错误
 *         3 - 擦除验证失败
 *         4 - 擦除过程状态异常
 */
uint8_t AM29LV320_Chip_Erase(void)
{
	
		// 定义变量：存储开始/结束的Unix时间戳，以及日期时间结构体
    uint32_t erase_start_unix = 0;
    uint32_t erase_end_unix = 0;
    uint32_t erase_total_seconds = 0;
    
    RTC_TimeTypeDef erase_start_time = {0};
    RTC_DateTypeDef erase_start_date = {0};
    RTC_TimeTypeDef erase_end_time = {0};
    RTC_DateTypeDef erase_end_date = {0};

    // 1. 获取擦除开始的RTC时间+日期，并转换为Unix时间戳
    if (HAL_RTC_GetTime(&hrtc, &erase_start_time, RTC_FORMAT_BIN) != HAL_OK)
    {
        printf("获取擦除开始时间失败！\r\n");
        return 5;
    }
    HAL_RTC_GetDate(&hrtc, &erase_start_date, RTC_FORMAT_BIN); // 必须调用GetDate
    erase_start_unix = rtc_to_unix_timestamp(&erase_start_date, &erase_start_time);
    printf("擦除开始时间（Unix时间戳）：%lu 秒\r\n", (unsigned long)erase_start_unix);
    printf("擦除开始时间（本地）：%04d-%02d-%02d %02d:%02d:%02d\r\n",
           2000+erase_start_date.Year, erase_start_date.Month, erase_start_date.Date,
           erase_start_time.Hours, erase_start_time.Minutes, erase_start_time.Seconds);

	  
    // 超时计数器(单位:秒, 最大30秒超时保护)
    uint32_t timeout_count = 0;
		uint32_t max_timeout = 65535;
    // 擦除完成标志
    uint8_t erase_ok = 0;
    // 状态寄存器数据
    uint16_t status_data = 0;
    // 状态位(Q5:超时标志 Q6:翻转标志 Q7:完成标志)
    uint8_t q5 = 0, q6 = 0, q7 = 0;
    // Q6上一次状态(检测是否停止翻转)
    uint8_t q6_prev = 0;
    // RY/BY#引脚状态(低=忙 高=就绪)
    GPIO_PinState ry_by_state = GPIO_PIN_RESET;

		uint16_t cap = AM29LV320_Read_CFI_Capacity();
	  if(cap==0)
		{
			//获取容量失败
			printf("Get Chip Size failed\r\n");
			return 99;
		} 
		else if(cap<=0x14)
		{
			//4M
			max_timeout = 60;
		}
	
	
    /************************** 1. 初始化RY/BY#引脚 **************************/
    // 使能GPIOD时钟
    __HAL_RCC_GPIOD_CLK_ENABLE();
    
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin       = RY_BY_PIN;          // PD6引脚
    gpio_init.Mode      = GPIO_MODE_INPUT;    // 输入模式
    gpio_init.Pull      = GPIO_PULLUP;        // 启用内部上拉(外部无上拉时关键)
    gpio_init.Speed     = GPIO_SPEED_FREQ_LOW;// 低速模式
    HAL_GPIO_Init(RY_BY_PORT, &gpio_init);

    // 复位芯片, 确保退出之前的异常状态
    *(FSMC_NOR_BASE_ADDR + 0x0000) = 0xFFFF;
    HAL_Delay(1);

    /************************** 2. 发送擦除命令序列 **************************/
    // 严格按照AM29LV320手册的擦除命令时序
    *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00AA;   // 解锁序列1
    delaycmd();
    *(FSMC_NOR_BASE_ADDR + 0x2AA) = 0x0055;   // 解锁序列2
    delaycmd();
    *(FSMC_NOR_BASE_ADDR + 0x555) = 0x0080;   // 擦除解锁1
    delaycmd();
    *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00AA;   // 擦除解锁2
    delaycmd();
    *(FSMC_NOR_BASE_ADDR + 0x2AA) = 0x0055;   // 擦除解锁3
    delaycmd();
    *(FSMC_NOR_BASE_ADDR + 0x555) = 0x0010;   // 触发整芯片擦除
    HAL_Delay(10);                             // 等待芯片进入擦除状态

    // 读取并检查擦除状态
    ry_by_state = HAL_GPIO_ReadPin(RY_BY_PORT, RY_BY_PIN);
    printf("Check is Enter Erase\r\n");

    // 验证是否成功进入擦除状态
    if (ry_by_state == GPIO_PIN_RESET)
    {
        printf("RY/BY# is 0, Enter Success\r\n");
    }
    else
    {
        printf("RY/BY# is 1, Enter Failed\r\n");
        return 2; // 进入擦除状态失败
    }

    /************************** 3. 循环检测擦除状态 **************************/
    while (1)
    {
        // 超时保护
        if (timeout_count > max_timeout)
        {
            printf("Erase Timeout! (Count: %lu)\r\n", (unsigned long)timeout_count);
            // 复位芯片退出擦除状态
            *(FSMC_NOR_BASE_ADDR + 0x0000) = 0xFFFF;
            HAL_Delay(1);
            return 1; // 超时错误
        }

        // 读取RY/BY#引脚状态并打印
        ry_by_state = HAL_GPIO_ReadPin(RY_BY_PORT, RY_BY_PIN);
        printf("Time:%ds, RY/BY#:%d\r\n", timeout_count, 
               (ry_by_state == GPIO_PIN_SET) ? 1 : 0);

        // 仅当RY/BY#为高(就绪)时, 读取状态寄存器进行验证
        if (ry_by_state == GPIO_PIN_SET)
        {
            // 读取状态寄存器(0x0000地址)
            status_data = *(FSMC_NOR_BASE_ADDR + 0x0000);
            
            // 解析状态位
            q5 = (status_data >> 5) & 0x01; // Q5: 超时标志(0=正常, 1=超时)
            q6 = (status_data >> 6) & 0x01; // Q6: 翻转标志(擦除中持续翻转)
            q7 = (status_data >> 7) & 0x01; // Q7: 擦除完成标志(1=完成)

            // 打印详细状态信息
            printf("Status: Q7=%d, Q6=%d, Q5=%d, RY/BY#=%d\r\n", 
                   q7, q6, q5, (ry_by_state == GPIO_PIN_SET) ? 1 : 0);

            // 擦除完成判断条件
            if (ry_by_state == 1 && q7 == 1 && q6 == 1 && q5 == 1)
            {
                erase_ok = 1;
							
							
							
                break; // 擦除完成, 退出循环
            }
            // 异常状态判断
            else if (ry_by_state == 1 && q7 == 1 && q5 == 0)
            {
                return 4; // 擦除过程状态异常
            }
        }

        // 保存当前Q6状态, 用于下一次翻转检测
        q6_prev = q6;

        // 每1秒检测一次, 翻转LED提示擦除中
        HAL_Delay(1000);
        HAL_GPIO_TogglePin(LEDG_PORT, LEDG_PIN);
        timeout_count++;
    }
		
				// 2. 获取擦除结束的RTC时间+日期，并转换为Unix时间戳
    if (HAL_RTC_GetTime(&hrtc, &erase_end_time, RTC_FORMAT_BIN) != HAL_OK)
    {
        printf("获取擦除结束时间失败！\r\n");
    }
		else 
		{
				HAL_RTC_GetDate(&hrtc, &erase_end_date, RTC_FORMAT_BIN);
				erase_end_unix = rtc_to_unix_timestamp(&erase_end_date, &erase_end_time);
				printf("擦除结束时间（Unix时间戳）：%lu 秒\r\n", (unsigned long)erase_end_unix);
				printf("擦除结束时间（本地）：%04d-%02d-%02d %02d:%02d:%02d\r\n",
							 2000+erase_end_date.Year, erase_end_date.Month, erase_end_date.Date,
							 erase_end_time.Hours, erase_end_time.Minutes, erase_end_time.Seconds);

				// 3. 计算擦除耗时（直接相减，无需处理跨时段）
				erase_total_seconds = erase_end_unix - erase_start_unix;
				printf("AM29LV320 芯片擦除耗时：%lu 秒\r\n", (unsigned long)erase_total_seconds);
		}


    /************************** 4. 复位芯片并验证擦除结果 **************************/
    // 复位芯片, 退出状态查询模式
    *(FSMC_NOR_BASE_ADDR + 0x0000) = 0xFFFF;
    HAL_Delay(10); // 确保复位生效

		// 全芯片擦除验证：逐地址读取并检查是否为0xFFFF
		uint32_t error_count = 0;          // 统计非0xFFFF的地址数量
		uint32_t total_check_addr = 0;     // 已检查的地址总数
		uint32_t chip_total_halfwords = 0; // 芯片总半字数量（4MB = 2M 半字）

		// 根据CFI容量计算芯片总半字数（4MB = 4*1024*1024 / 2 = 2097152 半字）
		if (chip_total_halfwords == 0) {
			chip_total_halfwords = 2097152; // TODO:获取正确的容量
		}

		printf("开始全芯片擦除验证，总校验半字数：%lu\r\n", (unsigned long)chip_total_halfwords);

		// 逐半字地址读取并验证（16位总线，直接访问FSMC映射地址）
		for (total_check_addr = 0; total_check_addr < chip_total_halfwords; total_check_addr++)
		{
				// 直接读取当前半字地址数据（不调用封装函数，避免依赖）
				uint16_t current_data = *(FSMC_NOR_BASE_ADDR + total_check_addr);
				
				// 检查是否为擦除后的预期值0xFFFF
				if (current_data != 0xFFFF)
				{
						error_count++;
						// 每1000个错误打印一次（避免串口刷屏）
						if (error_count % 1000 == 0) {
								printf("验证异常：地址0x%08X 数据0x%04X | 累计错误数：%lu\r\n",
											 (uint32_t)(FSMC_NOR_BASE_ADDR + total_check_addr),
											 current_data, (unsigned long)error_count);
						}
				}
				
				// 每校验100000个地址打印进度（提升交互性）
				if (total_check_addr % 100000 == 0) {
					  HAL_GPIO_TogglePin(LEDG_PORT, LEDG_PIN);
						printf("验证进度：%lu/%lu 半字 | 当前错误数：%lu\r\n",
									 (unsigned long)total_check_addr,
									 (unsigned long)chip_total_halfwords,
									 (unsigned long)error_count);
				}
		}

		// 打印最终验证结果
		printf("全芯片验证完成：共校验%lu个半字 | 非0xFFFF地址数：%lu\r\n",
					 (unsigned long)total_check_addr, (unsigned long)error_count);

		// 根据错误数更新擦除结果标志
		if (error_count > 0)
		{
				erase_ok = 0; // 存在未擦除干净的地址，置为错误
				printf("擦除验证失败！共检测到%lu个地址未擦除干净\r\n", (unsigned long)error_count);
		}
		else
		{
				printf("擦除验证通过！全芯片所有地址均为0xFFFF\r\n");
		}


    /************************** 5. 返回擦除结果 **************************/
    if (erase_ok)
    {
        return 0; // 擦除成功
    }
    else
    {
        return 2; // 状态错误
    }
}

uint8_t AM29LV320_Write_Data_From_File(const char *filename)
{
    FRESULT res;
    FIL file;
    FATFS fs;
    const TCHAR* vol = _T("0:");
    char file_path[260] = {0};
    uint8_t write_ret = 0;
    uint32_t file_size = 0;
    uint32_t written_bytes = 0;
    const uint32_t AM29_MAX_CAPACITY = 0x400000; // AM29LV320最大容量4MB(字节)
    
						uint32_t error_count = 0;        // 统计不一致的Word数量

				// 定义变量：存储开始/结束的Unix时间戳，以及日期时间结构体
    uint32_t erase_start_unix = 0;
    uint32_t erase_end_unix = 0;
    uint32_t erase_total_seconds = 0;
    
    RTC_TimeTypeDef erase_start_time = {0};
    RTC_DateTypeDef erase_start_date = {0};
    RTC_TimeTypeDef erase_end_time = {0};
    RTC_DateTypeDef erase_end_date = {0};

		
    // 1. 入参校验
    if (filename == NULL || strlen(filename) == 0)
    {
        printf("写入失败：文件名参数为空\r\n");
        return 1;
    }
    sprintf(file_path, "0:/AM29/%s", filename); // 拼接完整文件路径

    // 2. 挂载TF卡
    res = f_mount(&fs, vol, 1);
    if (res != FR_OK)
    {
        printf("写入失败：TF卡挂载失败，错误码：%d\r\n", res);
        return 2;
    }

    // 3. 打开BIN文件（只读模式）
    res = f_open(&file, (const TCHAR*)file_path, FA_READ);
    if (res != FR_OK)
    {
        printf("写入失败：打开文件%s失败，错误码：%d\r\n", filename, res);
        f_mount(NULL, vol, 1);
        return 3;
    }

    // 4. 获取文件大小并校验
    file_size = f_size(&file);
    printf("打开文件成功：%s | 大小：%lu 字节\r\n", filename, (unsigned long)file_size);
    if (file_size > AM29_MAX_CAPACITY)
    {
        printf("写入失败：文件大小(%lu字节)超过芯片最大容量(%lu字节)\r\n",
               (unsigned long)file_size, (unsigned long)AM29_MAX_CAPACITY);
        f_close(&file);
        f_mount(NULL, vol, 1);
        return 4;
    }
    if (file_size == 0)
    {
        printf("写入失败：文件为空\r\n");
        f_close(&file);
        f_mount(NULL, vol, 1);
        return 1;
    }

    // 6. 芯片初始化（仅执行一次复位）
    printf("芯片初始化（仅复位一次）...\r\n");
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29LV320_RESET_CMD;
    HAL_Delay(1);

    // 7. 分块读取文件并写入芯片（16位对齐，每次写2字节）
    uint8_t read_buf[2] = {0};  // 改为8位缓冲区，精准读取大端字节
    uint16_t write_data = 0;    // 存储翻转后的16位数据
    UINT br = 0;                // 实际读取字节数
    printf("开始写入文件到芯片（适配大端模式）...\r\n");

				    // 1. 获取擦除开始的RTC时间+日期，并转换为Unix时间戳
    if (HAL_RTC_GetTime(&hrtc, &erase_start_time, RTC_FORMAT_BIN) != HAL_OK)
    {
        printf("获取写入开始时间失败！\r\n");
        return 5;
    }
    HAL_RTC_GetDate(&hrtc, &erase_start_date, RTC_FORMAT_BIN); // 必须调用GetDate
    erase_start_unix = rtc_to_unix_timestamp(&erase_start_date, &erase_start_time);
    printf("写入开始时间（Unix时间戳）：%lu 秒\r\n", (unsigned long)erase_start_unix);
    printf("写入开始时间（本地）：%04d-%02d-%02d %02d:%02d:%02d\r\n",
           2000+erase_start_date.Year, erase_start_date.Month, erase_start_date.Date,
           erase_start_time.Hours, erase_start_time.Minutes, erase_start_time.Seconds);

		
    while (written_bytes < file_size)
    {
        // 7.1 读取2字节大端数据（不足2字节时补0）
        br = 0;
        memset(read_buf, 0, sizeof(read_buf));
        res = f_read(&file, read_buf, 2, &br);
        if (res != FR_OK)
        {
            printf("写入失败：读取文件数据失败，错误码：%d\r\n", res);
            write_ret = 5;
            break;
        }
        if (br == 0) break; // 无数据可读，退出循环

        // 7.2 大端字节序转16位数据（关键修改）
        // 第1字节=高8位，第2字节=低8位 → 0x11 0x22 → 0x1122
        write_data = ((uint16_t)read_buf[0] << 8) | read_buf[1];

        // 7.3 计算当前写入地址（芯片本地字节地址 → 半字偏移）
        uint32_t am29_local_addr = written_bytes;
        uint32_t addr_halfword = am29_local_addr >> 1; // 字节地址转半字偏移
        uint32_t fsmc_addr = (uint32_t)FSMC_NOR_BASE_ADDR + (addr_halfword * 2); // 调试用FSMC地址

        // 7.4 执行写入命令序列（循环执行，不嵌套调用）
        *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00AA; // 命令1：0x0555半字偏移 → 0x00AA
        delaycmd();
        *(FSMC_NOR_BASE_ADDR + 0x2AA) = 0x0055; // 命令2：0x02AA半字偏移 → 0x0055
        delaycmd();
        *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00A0; // 命令3：0x0555半字偏移 → 0x00A0
        delaycmd();

        // 7.5 写入目标地址+数据（使用本地半字偏移访问）
        *(FSMC_NOR_BASE_ADDR + addr_halfword) = write_data;
        delaycmd(); // 确保数据写入完成

        // 7.6 等待写入完成（检测RY/BY#引脚，低电平=忙，高电平=就绪）
        uint32_t timeout = 1000; // 10ms超时保护
        while (HAL_GPIO_ReadPin(RY_BY_PORT, RY_BY_PIN) == GPIO_PIN_RESET)
        {
            delaycmd();
            if (--timeout == 0)
            {
                printf("写入超时：地址0x%08X (半字偏移0x%06X)\r\n", fsmc_addr, addr_halfword);
                *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29LV320_RESET_CMD; // 超时复位
                write_ret = 5;
                break;
            }
        }
        if (write_ret != 0) break; // 超时则退出循环

        // 7.7 更新已写入字节数
        written_bytes += br;

				// 7.8 进度打印（每写入64KB打印一次）
				if (written_bytes % (64 * 1024) == 0 || written_bytes == file_size)
				{
						// 复用工程已有RTC相关函数/变量，计算耗时
						static uint32_t write_start_unix = 0; // 静态变量：仅初始化一次
						RTC_TimeTypeDef sTime = {0};
						RTC_DateTypeDef sDate = {0};
						uint32_t current_unix = 0;
						uint32_t elapsed_sec = 0;
						float total_est_sec = 0.0f;

						// 首次进入时记录开始时间
						if (write_start_unix == 0)
						{
								if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK)
								{
										HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
										write_start_unix = rtc_to_unix_timestamp(&sDate, &sTime);
								}
								else
								{
										write_start_unix = HAL_GetTick() / 1000; // 备用：系统Tick转秒
								}
						}

						// 获取当前时间并计算已耗时
						if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK)
						{
								HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
								current_unix = rtc_to_unix_timestamp(&sDate, &sTime);
						}
						else
						{
								current_unix = HAL_GetTick() / 1000;
						}
						elapsed_sec = current_unix - write_start_unix;

						// 计算预计总耗时（防除0）
						if (written_bytes > 0 && elapsed_sec > 0)
						{
								float progress = (float)written_bytes / file_size;
								total_est_sec = (float)elapsed_sec / progress;
						}

						// 打印（新增已用时/预计耗时）
						printf("写入进度：%lu/%lu 字节 (%.1f%%) | 已用时：%lu 秒 | 预计总耗时：%.1f 秒\r\n",
									 (unsigned long)written_bytes, (unsigned long)file_size,
									 (float)written_bytes / file_size * 100,
									 (unsigned long)elapsed_sec, total_est_sec);
						
						//LED闪耀提示
						HAL_GPIO_TogglePin(LEDG_PORT, LEDG_PIN);
				}
    }

				//显示时间
				if (HAL_RTC_GetTime(&hrtc, &erase_end_time, RTC_FORMAT_BIN) != HAL_OK)
    {
        printf("获取写入结束时间失败！\r\n");
        
    }
		else 
		{
				HAL_RTC_GetDate(&hrtc, &erase_end_date, RTC_FORMAT_BIN);
				erase_end_unix = rtc_to_unix_timestamp(&erase_end_date, &erase_end_time);
				printf("写入结束时间（Unix时间戳）：%lu 秒\r\n", (unsigned long)erase_end_unix);
				printf("写入结束时间（本地）：%04d-%02d-%02d %02d:%02d:%02d\r\n",
							 2000+erase_end_date.Year, erase_end_date.Month, erase_end_date.Date,
							 erase_end_time.Hours, erase_end_time.Minutes, erase_end_time.Seconds);

				// 3. 计算写入耗时（直接相减，无需处理跨时段）
				erase_total_seconds = erase_end_unix - erase_start_unix;
				printf("AM29LV320 芯片写入耗时：%lu 秒\r\n", (unsigned long)erase_total_seconds);
		}


		// 8. 全量逐Word对比验证（8192字节分块读取优化）
if (write_ret == 0)
{
    printf("开始全量逐Word验证（文件VS芯片）...\r\n");
    uint32_t total_verify_words = 0; // 总验证Word数
    uint32_t verify_addr = 0;        // 字节地址偏移
    uint8_t file_buf[8192] = {0};    // 8192字节分块缓冲区（提升读取效率）
    UINT br = 0;
    #define VERIFY_BUF_SIZE 8192     // 分块大小定义

    // 重置文件指针到起始位置
    f_lseek(&file, 0);

    // 分块读取+逐Word遍历整个文件
    while (verify_addr < file_size)
    {
        // 8.1 计算当前块需要读取的字节数（不足8192时读取剩余部分）
        uint32_t read_len = (file_size - verify_addr) > VERIFY_BUF_SIZE ? 
                            VERIFY_BUF_SIZE : (file_size - verify_addr);
        
        // 8.2 从TF卡文件读取1块数据（8192字节）
        FRESULT res = f_read(&file, file_buf, read_len, &br);
        if (res != FR_OK || br != read_len)
        {
            printf("文件分块读取异常！起始地址0x%08X, 期望读取%lu字节, 实际读取%lu字节\r\n",
                   verify_addr, (unsigned long)read_len, (unsigned long)br);
            write_ret = 6;
            break;
        }

        // 8.3 遍历当前块的所有Word（逐2字节处理）
        for (uint32_t buf_idx = 0; buf_idx < br; buf_idx += 2)
        {
            // 8.3.1 处理文件大小奇数的边界情况（最后1字节补0）
            uint8_t byte0 = file_buf[buf_idx];
            uint8_t byte1 = (buf_idx + 1 < br) ? file_buf[buf_idx + 1] : 0x00;

            // 8.3.2 转换文件数据为大端格式16位Word（和原逻辑一致）
            uint16_t data_file = ((uint16_t)byte0 << 8) | byte1;
            
            // 8.3.3 读取芯片对应地址的Word（字节地址转半字偏移）
            uint32_t chip_halfword = verify_addr >> 1;
            uint16_t data_chip = *(FSMC_NOR_BASE_ADDR + chip_halfword);

            // 8.3.4 对比并统计错误
            if (data_chip != data_file)
            {
                error_count++;
                // 每1000个错误打印一次（避免串口刷屏）
                if (error_count % 1000 == 0)
                {
                    printf("验证异常：地址0x%08X | 文件0x%04X ≠ 芯片0x%04X | 累计错误：%lu\r\n",
                            (uint32_t)FSMC_NOR_BASE_ADDR + verify_addr, data_file, data_chip, (unsigned long)error_count);
                }
            }

            // 8.3.5 进度打印（每100000个Word打印一次）
            total_verify_words++;
            if (total_verify_words % 100000 == 0)
            {
                printf("验证进度：%lu/%lu Words | 当前错误数：%lu\r\n",
                        (unsigned long)total_verify_words, 
                        (unsigned long)((file_size + 1) / 2), // 总Word数（向上取整）
                        (unsigned long)error_count);
            }

            // 8.3.6 全局地址偏移递增（按字节）
            verify_addr += 2;
        }
    }

    // 8.4 验证结果汇总
    printf("全量验证完成 | 总验证Word数：%lu | 不一致Word数：%lu\r\n",
            (unsigned long)total_verify_words, (unsigned long)error_count);
    
    if (error_count > 0)
    {
        write_ret = 6; // 标记验证失败
    }
    else
    {
        printf("全量逐Word校验通过！\r\n");
    }
}

    // 9. 资源释放 + 芯片复位
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29LV320_RESET_CMD;
    HAL_Delay(1);
    f_close(&file);
    f_mount(NULL, vol, 1);

    // 10. 结果反馈
		if (write_ret == 0)
		{
				printf("文件%s写入芯片成功！总写入字节：%lu\r\n", filename, (unsigned long)written_bytes);
				return 0;
		}
		else if (write_ret == 6)
		{
				printf("文件%s写入芯片失败！验证发现 %lu 个Word数据不一致\r\n", filename, (unsigned long)error_count);
				return write_ret;
		}
		else
		{
				printf("文件%s写入芯片失败！\r\n", filename);
				return write_ret;
		}
}

/**
 * @brief  纳秒延迟初始化函数（仅需调用一次，建议在main开头执行）
 * @note   预计算Tick步长、单指令周期等固定参数，避免每次delayns重复计算
 * @retval None
 */
void delayns_init(void)
{
    /* 1. 获取当前Tick频率，计算uwTick递增步长（单位：us） */
    HAL_TickFreqTypeDef tick_freq = HAL_GetTickFreq();
    switch(tick_freq)
    {
        case HAL_TICK_FREQ_10HZ:    
            g_tick_step_us = 100000; // 100ms/步 = 100000us
            break;
        case HAL_TICK_FREQ_100HZ:   
            g_tick_step_us = 10000;  // 10ms/步 = 10000us
            break;
        case HAL_TICK_FREQ_1KHZ:    
            g_tick_step_us = 1000;   // 1ms/步 = 1000us
            break;
        default:                    
            g_tick_step_us = 1000;   // 默认1KHz
            break;
    }

    /* 2. 计算单指令周期耗时（ns）：向上取整避免精度丢失 */
    g_ns_per_cycle = (1000000000UL + SYSTEM_CORE_CLOCK - 1) / SYSTEM_CORE_CLOCK;
}

/**
 * @brief  纳秒级延迟实际调用函数（轻量化，仅执行延迟逻辑）
 * @param  ns: 要延迟的纳秒数
 * @retval None
 */
void delayns(uint32_t ns)
{
    /* 1. 边界处理：最小延迟（避免ns过小导致循环次数为0） */
    if(ns < MIN_DELAY_NS)
    {
        ns = MIN_DELAY_NS;
    }

    /* 2. 计算需要的循环次数（直接使用初始化好的全局参数） */
    uint32_t loop_cnt = ns / (g_ns_per_cycle * CYCLE_PER_LOOP);
    
    /* 3. 空循环延迟（volatile防止编译器优化空循环） */
    volatile uint32_t i;
    for(i = 0; i < loop_cnt; i++)
    {
        __NOP(); // 空操作指令，稳定指令周期
    }
}

void delaycmd(void)
{
	volatile uint32_t i;
	for(i = 0; i < 10; i++)
    {
        __NOP(); // 空操作指令，稳定指令周期
    }
		
}

// 遍历AM29目录下所有.bin文件并返回列表（保留原有可运行的TF卡逻辑）
AM29_File_Info_t* AM29_List_Bin_Files(uint8_t *file_count)
{
    FRESULT res;                  
    DIR dir;                      
    FILINFO fno;                  
    const TCHAR* vol = _T("0:");  
    const TCHAR* path = _T("0:/AM29"); 
    FATFS fs;

    memset(g_am29_bin_files, 0, sizeof(g_am29_bin_files));
    g_am29_bin_file_count = 0;

    if(file_count == NULL) {
        printf("参数错误：file_count指针为空\r\n");
        return NULL;
    }
    *file_count = 0;

    printf("SDIO外设初始化成功，开始挂载TF卡...\r\n");
    res = f_mount(&fs, vol, 1); 
    if(res != FR_OK)
    {
        printf("TF卡挂载失败! 错误码: %d\r\n", res);
        printf("错误原因：1=SDIO配置错误 | 4=TF卡损坏/未插好\r\n");
        return NULL;
    }
    printf("TF卡挂载成功！\r\n");

    // 暂时注释目录创建（改前可能没有这步）
    // res = f_mkdir(path);
    // if(res != FR_OK && res != FR_EXIST) { ... }

    res = f_opendir(&dir, path);
    if(res != FR_OK)
    {
        printf("打开AM29目录失败! 错误码: %d\r\n", res);
        f_mount(NULL, vol, 1); 
        return NULL;
    }

    printf("===== AM29目录下的.bin文件列表 =====\r\n");
    while(1)
    {
        if(g_am29_bin_file_count >= MAX_BIN_FILE_CNT) break;

        res = f_readdir(&dir, &fno);
        if(res != FR_OK || fno.fname[0] == 0) break;

        if((fno.fattrib & AM_DIR) == 0) 
        {
            // 暂时简化后缀判断（改前可能更简单）
            char* suffix = strrchr(fno.fname, '.');
            if(suffix && (strcmp(suffix, ".bin")==0 || strcmp(suffix, ".BIN")==0))
            {
                strncpy(g_am29_bin_files[g_am29_bin_file_count].filename, 
                        fno.fname, sizeof(g_am29_bin_files[g_am29_bin_file_count].filename)-1);
                g_am29_bin_files[g_am29_bin_file_count].filesize = fno.fsize;
                printf("文件: %s | 大小: %lu 字节\r\n", 
                       g_am29_bin_files[g_am29_bin_file_count].filename,
                       (unsigned long)g_am29_bin_files[g_am29_bin_file_count].filesize);
                g_am29_bin_file_count++;
            }
        }
    }

    f_closedir(&dir);
    f_mount(NULL, vol, 1);

    *file_count = g_am29_bin_file_count;
    if(g_am29_bin_file_count == 0)
    {
        printf("AM29目录下未找到.bin文件\r\n");
    }
    else
    {
        printf("===== 共找到 %d 个.bin文件 =====\r\n", g_am29_bin_file_count);
    }

    return g_am29_bin_files;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
