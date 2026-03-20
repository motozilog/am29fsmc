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
#include <stdio.h>             // 引入printf所需头文件
#include "stm32f1xx_ll_fsmc.h" // FSMC核心头文件（F1系列）
#include "fatfs.h"             // FATFS核心头文件（工程已包含）
#include "stm32f1xx_hal.h"     // 工程必有的HAL库头文件（包含SDIO基础定义）
#include "stm32f1xx_hal_rtc.h" // RTC HAL库头文件

// SSD1306
#include "oled.h"
#include "bmp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TF卡定义
#define TF_BLOCK_SIZE 512                  // SD卡默认块大小（固定512字节）
#define TF_MAX_BLOCK_NUM (1024 * 1024 * 4) // 4GB TF卡最大块数（可根据实际卡调整）
extern SD_HandleTypeDef hsd;               // 引用SDIO的SD卡句柄

uint32_t g_tf_block_num = 0;
uint16_t g_tf_block_size = 0;

// RTC句柄定义
RTC_HandleTypeDef hrtc;

// 按键定义：KEY_CANCEL（对应PC5，外部上拉，低电平有效）
#define KEY_CANCEL_PIN CANCEL_Pin
#define KEY_CANCEL_PORT GPIOC

// LED定义：LEDG（对应PB0的LED_G_Pin）
#define LEDG_PIN LED_G_Pin
#define LEDG_PORT GPIOB

#define LEDR_PIN LED_R_Pin
#define LEDR_PORT GPIOB

// 蜂鸣器定义（PC6）
#define BUZZER_PIN GPIO_PIN_6
#define BUZZER_PORT GPIOC

#define A25_Pin GPIO_PIN_14
#define A25_GPIO_Port GPIOG

// ********** AM29LV320 FSMC 配置 **********
#define FSMC_NOR_BASE_ADDR ((uint16_t *)0x60000000) // FSMC BANK1基地址
// AM29LV320 ID存储变量
uint16_t am29_mfg_id = 0; // 制造商ID
uint16_t am29_dev_id = 0; // 设备ID
uint16_t am29_cap_id = 0; // 容量ID

// AM29 CFI容量读取相关定义
#define AM29_CMD_READ_ID 0x90      // JEDEC ID读取指令
#define AM29_RESET_CMD 0x00F0      // 复位指令
#define AM29_CFI_ENTRY_ADDR 0x0055 // 进入CFI模式
#define AM29_CFI_ENTRY_DATA 0x0098 // 进入CFI模式的写入数据
#define AM29_CFI_CAP_ADDR 0x0027   // 读取CFI容量

// 定义RY/BY#引脚（PD6）相关宏
#define RY_BY_PIN GPIO_PIN_6
#define RY_BY_PORT GPIOD

#define SYSTEM_CORE_CLOCK 72000000UL // 系统主频（需与工程配置一致）
#define CYCLE_PER_LOOP 3UL           // 单循环指令数（实测校准）
#define MIN_DELAY_NS 10              // 最小可靠延迟（ns）

/* 延迟全局变量 */
static uint32_t g_ns_per_cycle = 0; // 单指令周期耗时（ns）
static uint32_t g_tick_step_us = 0; // Tick递增步长（us，仅作为备用参数）

const uint32_t BLOCK_64MB = 64 * 1024 * 1024; // 64MB

// 文件信息结构体（存储.bin文件名称+大小）
typedef struct
{
    char filename[256]; // 文件名（兼容长文件名）
    uint32_t filesize;  // 文件大小（字节）
} AM29_File_Info_t;

// 新增：全局文件列表缓存（最多存储32个.bin文件，可根据需求调整）
#define MAX_BIN_FILE_CNT 32
static AM29_File_Info_t g_am29_bin_files[MAX_BIN_FILE_CNT] = {0};
static uint8_t g_am29_bin_file_count = 0; // 实际找到的.bin文件数量

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
HAL_SD_CardInfoTypeDef card_info; // 新增：用于存储TF卡信息

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

// 按键
uint8_t KEY_UP_Detect(void);
uint8_t KEY_DOWN_Detect(void);
uint8_t KEY_LEFT_Detect(void);
uint8_t KEY_RIGHT_Detect(void);
uint8_t KEY_OK_Detect(void);
uint8_t KEY_CANCEL_Detect(void);

static uint8_t Get_Any_Key(void);
static uint8_t Get_User_Input(void);

void delay_us(uint32_t us);

uint32_t AM29_Read_ID(void); // 读取AM29的ID

static void MX_RTC_Init(void); // RTC初始化声明

void AM29LV320_Read_Data(uint16_t addr_halfword, uint16_t len, uint16_t *buf);

// CFI容量读取函数声明
uint16_t AM29_Read_CFI_Capacity(void);                // 读取CFI容量原始值
uint32_t AM29_Parse_CFI_Capacity(uint16_t cfi_value); // 解析CFI容量值为实际字节数

uint8_t AM29_Read_To_File(void); // 读取到文件
uint8_t AM29_Chip_Erase(void);   // 整芯片擦除

static void Set_A25_A31(uint32_t byte_address); // 设置A25~A31
static void Set_A25_A31_All_Zero(void);         // A25~A31置0

uint32_t rtc_to_unix_timestamp(RTC_DateTypeDef *date, RTC_TimeTypeDef *time);

void delayns_init(void);
void delayns(uint32_t ns);
void delaycmd(void);
void buz(void);

static uint8_t Wait_Any_Key(uint32_t timeout_ms);

AM29_File_Info_t *AM29_List_Bin_Files(uint8_t *file_count);

uint8_t AM29LV320_Check_Blank(uint32_t start_addr, uint32_t check_len, uint8_t show_details);
uint8_t AM29LV320_Check_Start_Area(uint8_t ask_user);

uint8_t AM29_Write_Data_From_File(const char *filename);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 手动定义SDIO所需的核心常量和句柄（替代sdio.h）
SD_HandleTypeDef hsd; // SDIO句柄（全局，和工程其他部分兼容）

// SDIO时钟分频、总线宽度等常量（和HAL库定义一致）
#define SDIO_CLOCK_EDGE_RISING 0x00000000U
#define SDIO_CLOCK_BYPASS_DISABLE 0x00000000U
#define SDIO_CLOCK_POWER_SAVE_DISABLE 0x00000000U
#define SDIO_BUS_WIDE_1B 0x00000000U
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
    MX_I2C1_Init();
    MX_SDIO_SD_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_FATFS_Init();
    // MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 2 */
    MX_RTC_Init();

    // LED初始化
    HAL_GPIO_WritePin(LEDG_PORT, LEDG_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LEDR_PORT, LEDR_PIN, GPIO_PIN_SET);

    // 启动时强制将USB D+引脚拉低，防止主机检测到设备 START
    printf("初始化USB引脚为低电平，防止提前检测...\r\n");

    // 将PA12配置为输出推挽，并输出低电平
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12; // 只配置D+引脚
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // 拉低D+

    // PA11配置为输入浮空（D-）
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    // 启动时强制将USB D+引脚拉低，防止主机检测到设备 END

    // SSD1306 START
    OLED_Init();
    OLED_ColorTurn(0);   // 0正常显示，1 反色显示
    OLED_DisplayTurn(0); // 0正常显示 1 屏幕翻转显示
    OLED_Clear();
    // SSD1306 END

    MX_FSMC_Init();

    /* TF START */
    // 初始化后检测TF卡状态
    printf("SDIO状态：%d\r\n", HAL_SD_GetState(&hsd));

    // 读取真实卡信息
    if (HAL_SD_GetCardInfo(&hsd, &card_info) == HAL_OK)
    {
        // 优先使用逻辑块（兼容USB MSC）
        if (card_info.LogBlockNbr > 0 && card_info.LogBlockSize > 0)
        {
            g_tf_block_num = card_info.LogBlockNbr;
            g_tf_block_size = card_info.LogBlockSize;
        }
        // 备用：逻辑块为0时用物理块
        else if (card_info.BlockNbr > 0 && card_info.BlockSize > 0)
        {
            g_tf_block_num = card_info.BlockNbr;
            g_tf_block_size = card_info.BlockSize;
        }
        // 打印真实容量（验证）
        uint64_t real_bytes = (uint64_t)g_tf_block_num * g_tf_block_size;
        uint32_t real_mb = (uint32_t)(real_bytes / 1024 / 1024);
        printf("SDIO读取真实容量：%u MB\r\n", real_mb);
        if (real_mb == 0)
        {
            printf("读取TF卡信息失败，请插入TF卡（需要MBR格式，不支持GPT。只能1个FAT32分区，不支持ExFAT）\r\n");
            // 设置默认容量以防万一
            g_tf_block_num = 0;
            g_tf_block_size = 0;

            OLED_Clear();
            OLED_ShowString2(0, "Please Insert TF CARD", 1);
            OLED_ShowString2(1, "MBR ONLY, Not GPT", 1);
            OLED_ShowString2(2, "FAT32 ONLY, Not ExFAT", 1);
            OLED_Refresh();

            while (1)
            {
                // LED闪烁提示
                HAL_GPIO_TogglePin(LEDR_PORT, LEDR_PIN);
                HAL_Delay(500);
            }
        }
    }
    else
    {
        printf("读取TF卡信息失败，请插入TF卡（需要MBR格式，不支持GPT。只能1个FAT32分区，不支持ExFAT）\r\n");
        // 设置默认容量以防万一
        g_tf_block_num = 0;
        g_tf_block_size = 0;

        OLED_Clear();
        OLED_ShowString2(0, "Please Insert TF CARD", 1);
        OLED_ShowString2(1, "MBR ONLY, Not GPT", 1);
        OLED_ShowString2(2, "FAT32 ONLY, Not ExFAT", 1);
        OLED_Refresh();

        while (1)
        {
            // LED闪烁提示
            HAL_GPIO_TogglePin(LEDR_PORT, LEDR_PIN);
            HAL_Delay(500);
        }
    }
    /* TF END */

    // 启动时通过USART1发送hello（波特率115200）
    printf("AM29 Programmer By motozilog V1.0\r\n");

    Set_A25_A31_All_Zero();

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
        while (user_choice == 0)
        {
            user_choice = Get_User_Input();
            HAL_Delay(50); // 适当延时，避免CPU占用过高
        }

        // 根据选择执行对应操作
        switch (user_choice)
        {
        case 1:
            AM29_Get_Id_And_Read10();
            NVIC_SystemReset();
            break;
        case 2:
            AM29_Erase_And_Verify();
            NVIC_SystemReset();
            break;
        case 3:
            AM29_Write_And_Verify();
            NVIC_SystemReset();
            break;
        case 4:
            AM29_Read_To_File();
            Wait_Any_Key(0); // 无限等待任意按键
            NVIC_SystemReset();
            break;
        case 5:
            switch_to_udisk();
            Wait_Any_Key(0); // 无限等待任意按键
            NVIC_SystemReset();
            break;
        default:
            printf("未知选项，请重新选择\r\n");
            UART_Menu_Show();
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
    __HAL_RCC_GPIOC_CLK_ENABLE(); // SDIO数据线使用的GPIO

    // 配置SDIO引脚为高速度
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
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
        // Error_Handler();
    }
    else
    {
        printf("TF卡己插入\r\n");
    }

    // 尝试切换到4位模式（可选）
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) == HAL_OK)
    {
        printf("切换到4位模式成功\r\n");
    }
    else
    {
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
    HAL_GPIO_WritePin(GPIOB, LED_G_Pin | LED_R_Pin | A27_Pin | A28_Pin | A29_Pin | A30_Pin | A31_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    //    HAL_GPIO_WritePin(A25_GPIO_Port, A25_Pin, GPIO_PIN_RESET);
    //    HAL_GPIO_WritePin(A26_GPIO_Port, A26_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : UP_Pin DOWN_Pin LEFT_Pin RIGHT_Pin
                             OK_Pin CANCEL_Pin */
    GPIO_InitStruct.Pin = UP_Pin | DOWN_Pin | LEFT_Pin | RIGHT_Pin | OK_Pin | CANCEL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_G_Pin LED_R_Pin A27_Pin A28_Pin
                             A29_Pin A30_Pin A31_Pin */
    GPIO_InitStruct.Pin = LED_G_Pin | LED_R_Pin | A27_Pin | A28_Pin | A29_Pin | A30_Pin | A31_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : SDIO_CD_Pin */
    GPIO_InitStruct.Pin = SDIO_CD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SDIO_CD_GPIO_Port, &GPIO_InitStruct);

    /* ===== 配置A25 (PG14) ===== */
    GPIO_InitStruct.Pin = A25_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(A25_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : A26_Pin */
    GPIO_InitStruct.Pin = A26_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(A26_GPIO_Port, &GPIO_InitStruct);

    // ===== 添加蜂鸣器引脚配置 =====
    // 配置PC6为输出推挽
    GPIO_InitStruct.Pin = BUZZER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);
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
        // Error_Handler( );
        printf("没有检测到AM29系列芯片\r\n");

        // 显示
        OLED_Clear();
        printfOled(0, 1, "NO AM29 found");
        printfOled(1, 1, "Power off");
        printfOled(2, 1, "And check");
        OLED_Refresh();

        while (1)
        {
        }
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
        Error_Handler();
    }

    /** Disconnect NADV
     */

    __HAL_AFIO_FSMCNADV_DISCONNECTED();

    /* USER CODE BEGIN FSMC_Init 2 */

    /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  根据字节地址设置A25~A31引脚状态
 * @param  byte_address: 字节地址
 * @retval None
 */
static void Set_A25_A31(uint32_t byte_address)
{
    // 从A25开始取7位（A25-A31）
    uint32_t new_a25_a31 = (byte_address >> 26) & 0x7F; // 7位：0x7F = 01111111

    // A25 对应原来的 A26? 需要根据实际硬件连接调整
    HAL_GPIO_WritePin(A25_GPIO_Port, A25_Pin, (new_a25_a31 & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A26_GPIO_Port, A26_Pin, (new_a25_a31 & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A27_GPIO_Port, A27_Pin, (new_a25_a31 & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A28_GPIO_Port, A28_Pin, (new_a25_a31 & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A29_GPIO_Port, A29_Pin, (new_a25_a31 & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A30_GPIO_Port, A30_Pin, (new_a25_a31 & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A31_GPIO_Port, A31_Pin, (new_a25_a31 & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  将A25~A31全部置0
 * @retval None
 */
static void Set_A25_A31_All_Zero(void)
{
    HAL_GPIO_WritePin(A25_GPIO_Port, A25_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A26_GPIO_Port, A26_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A27_GPIO_Port, A27_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A28_GPIO_Port, A28_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A29_GPIO_Port, A29_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A30_GPIO_Port, A30_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A31_GPIO_Port, A31_Pin, GPIO_PIN_RESET);
}

uint8_t KEY_UP_Detect(void)
{
    uint8_t key_flag = 0;
    if (HAL_GPIO_ReadPin(GPIOC, UP_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(GPIOC, UP_Pin) == GPIO_PIN_RESET)
        {
            key_flag = 1;
            while (HAL_GPIO_ReadPin(GPIOC, UP_Pin) == GPIO_PIN_RESET)
                ;
        }
    }
    return key_flag;
}

uint8_t KEY_DOWN_Detect(void)
{
    uint8_t key_flag = 0;
    if (HAL_GPIO_ReadPin(GPIOC, DOWN_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(GPIOC, DOWN_Pin) == GPIO_PIN_RESET)
        {
            key_flag = 1;
            while (HAL_GPIO_ReadPin(GPIOC, DOWN_Pin) == GPIO_PIN_RESET)
                ;
        }
    }
    return key_flag;
}

/**
 * @brief  LEFT按键检测函数（需要先在main.h中定义LEFT_Pin和LEFT_PORT）
 * @retval uint8_t: 1-按下，0-未按下
 */
uint8_t KEY_LEFT_Detect(void)
{
    uint8_t key_flag = 0;
    if (HAL_GPIO_ReadPin(GPIOC, LEFT_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(GPIOC, LEFT_Pin) == GPIO_PIN_RESET)
        {
            key_flag = 1;
            while (HAL_GPIO_ReadPin(GPIOC, LEFT_Pin) == GPIO_PIN_RESET)
                ;
        }
    }
    return key_flag;
}

/**
 * @brief  RIGHT按键检测函数（需要先在main.h中定义RIGHT_Pin和RIGHT_PORT）
 * @retval uint8_t: 1-按下，0-未按下
 */
uint8_t KEY_RIGHT_Detect(void)
{
    uint8_t key_flag = 0;
    if (HAL_GPIO_ReadPin(GPIOC, RIGHT_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(GPIOC, RIGHT_Pin) == GPIO_PIN_RESET)
        {
            key_flag = 1;
            while (HAL_GPIO_ReadPin(GPIOC, RIGHT_Pin) == GPIO_PIN_RESET)
                ;
        }
    }
    return key_flag;
}

uint8_t KEY_OK_Detect(void)
{
    uint8_t key_flag = 0;
    if (HAL_GPIO_ReadPin(GPIOC, OK_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(GPIOC, OK_Pin) == GPIO_PIN_RESET)
        {
            key_flag = 1;
            while (HAL_GPIO_ReadPin(GPIOC, OK_Pin) == GPIO_PIN_RESET)
                ;
        }
    }
    return key_flag;
}

// 获取用户输入（同时支持串口和按键）
static uint8_t Get_User_Input(void)
{
    uint8_t ch = 0;
    static uint8_t current_menu_line = 3; // 当前选中的菜单行，默认第一项（菜单从第3行开始）
    static uint8_t last_menu_line = 3;
    static uint8_t menu_initialized = 0;

    // 首次进入时显示菜单
    if (!menu_initialized)
    {
        OLED_ShowMenu(current_menu_line);
        menu_initialized = 1;
    }

    // 检测按键输入
    if (KEY_UP_Detect())
    {
        if (current_menu_line > 3) // 不能超过第一项（行3）
        {
            // 清除上一行的反色
            OLED_MenuItemReverse(current_menu_line, 1); // 第二次反色恢复原状

            current_menu_line--;

            // 新行反色
            OLED_MenuItemReverse(current_menu_line, 1);

            printf("\r\n当前选择：行 %d\r\n", current_menu_line);
        }
    }

    if (KEY_DOWN_Detect())
    {
        if (current_menu_line < 7) // 不能超过最后一项（行7）
        {
            // 清除上一行的反色
            OLED_MenuItemReverse(current_menu_line, 1); // 第二次反色恢复原状

            current_menu_line++;

            // 新行反色
            OLED_MenuItemReverse(current_menu_line, 1);

            printf("\r\n当前选择：行 %d\r\n", current_menu_line);
        }
    }

    if (KEY_OK_Detect())
    {
        // 根据当前行返回对应的选项
        switch (current_menu_line)
        {
        case 3:
            return 1; // Get AM29 ID
        case 4:
            return 2; // Erase
        case 5:
            return 3; // Write
        case 6:
            return 4; // Read to TF
        case 7:
            return 5; // Switch to UDisk
        default:
            return 0;
        }
    }

    // 检测串口输入（非阻塞）
    if (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
    {
        if (ch >= '1' && ch <= '5')
        {
            // 根据串口输入更新菜单显示
            uint8_t new_line = 3 + (ch - '1');
            if (new_line != current_menu_line)
            {
                OLED_MenuItemReverse(current_menu_line, 1);
                current_menu_line = new_line;
                OLED_MenuItemReverse(current_menu_line, 1);
            }
            return ch - '0';
        }
    }

    return 0;
}

/**
 * @brief  获取任意按键输入（同时检测串口和物理按键）
 * @retval uint8_t: 按键编码（具体定义见函数内部注释），0表示无输入
 */
static uint8_t Get_Any_Key(void)
{
    uint8_t ch = 0;

    // 1. 检测物理按键（上下左右确认取消）
    if (KEY_UP_Detect())
    {
        printf("检测到：上键\r\n");
        return 1; // 上键返回1
    }

    if (KEY_DOWN_Detect())
    {
        printf("检测到：下键\r\n");
        return 2; // 下键返回2
    }

    if (KEY_LEFT_Detect()) // 需要实现LEFT按键检测
    {
        printf("检测到：左键\r\n");
        return 3; // 左键返回3
    }

    if (KEY_RIGHT_Detect()) // 需要实现RIGHT按键检测
    {
        printf("检测到：右键\r\n");
        return 4; // 右键返回4
    }

    if (KEY_OK_Detect())
    {
        printf("检测到：确认键\r\n");
        return 5; // 确认键返回5
    }

    if (KEY_CANCEL_Detect())
    {
        printf("检测到：取消键\r\n");
        return 6; // 取消键返回6
    }

    // 2. 检测串口输入（非阻塞）
    if (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
    {
        // 回显字符（除了控制字符）
        if (ch >= 0x20 && ch <= 0x7E) // 可打印字符
        {
            HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);

            // 特殊处理：回车换行
            if (ch == '\r')
            {
                printf("\n");
            }
        }

        // 返回串口字符（保留原始值，包括控制字符）
        return ch;
    }

    return 0; // 无输入
}

/**
 * @brief  等待任意按键按下（阻塞版）
 * @param  timeout_ms: 超时时间（毫秒），0表示无限等待
 * @retval uint8_t: 按键编码（同Get_Any_Key），超时返回0
 */
static uint8_t Wait_Any_Key(uint32_t timeout_ms)
{
    uint32_t tickstart = HAL_GetTick();
    uint8_t key = 0;

    printf("请按任意键继续...\r\n");

    while (1)
    {
        key = Get_Any_Key();
        if (key != 0)
        {
            printf("\r\n"); // 换行
            return key;
        }

        // 超时检查
        if (timeout_ms > 0 && (HAL_GetTick() - tickstart) >= timeout_ms)
        {
            return 0; // 超时返回0
        }

        HAL_Delay(10); // 适当延时，避免CPU占用过高
    }
}

// 串口主菜单
static void UART_Menu_Show(void)
{
    OLED_ShowMenu(3); // 参数3表示默认选中第一项菜单（从第3行开始）

    printf("\r\n===== AM29编程器FSMC by motozilog =====\r\n");
    printf("1 - 获取AM29 ID并读取前10字\r\n");
    printf("2 - 整芯片擦除并查空验证\r\n");
    printf("3 - 选择BIN文件写入并校验\r\n");
    printf("4 - 读取AM29芯片内容保存到TF卡\r\n");
    printf("5 - 切换到U盘模式（作为U盘连接电脑）\r\n");

    printf("=== If Chinese display error, set terminal to GBK(DO NOT USE UTF8). ===\r\n");
    printf("请输入操作编号，按回车确认：");
}

// 切换到U盘
static void switch_to_udisk(void)
{
    uint8_t dummy;
    uint32_t tickstart;
    USBD_StatusTypeDef status;

    printf("\r\n===== 切换到U盘模式 =====\r\n");

    // 检查USB时钟状态
    printf("检查USB时钟状态...\r\n");
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY))
    {
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
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
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

    while ((HAL_GetTick() - tickstart) < 20000)
    { // 等待10秒
        HAL_Delay(1000);
        printf("等待中... (状态: %d)\r\n", hUsbDeviceFS.dev_state);

        // 显示
        OLED_Clear();
        printfOled(0, 1, "Switch To Udisk");
        printfOled(1, 1, "Wait for PC");
        printfOled(2, 1, "State: %d", hUsbDeviceFS.dev_state);
        OLED_Refresh();

        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        {
            printf("USB设备已配置！\r\n");
            break;
        }
    }

    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
    {
        printf("USB设备已初始化，现在可以作为U盘连接到电脑\r\n");
    }
    else
    {
        printf("USB设备状态异常: %d\r\n", hUsbDeviceFS.dev_state);

        // 显示
        OLED_Clear();
        printfOled(0, 1, "To Udisk Fail");
        printfOled(1, 1, "Code: %d", hUsbDeviceFS.dev_state);
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
        return;
    }

    printf("请在电脑上操作U盘，弹出U盘时将重启...\r\n");

    // 显示
    OLED_Clear();
    printfOled(0, 1, "UDisk Success");
    printfOled(1, 1, "Operate in PC");
    OLED_Refresh();

    // ========== 弹出检测逻辑 ==========
    uint32_t start_time = HAL_GetTick();
    uint32_t last_usb_state = hUsbDeviceFS.dev_state;
    uint32_t suspend_detected_time = 0;
    uint8_t eject_detected = 0;

    // USB状态定义 (来自 usbd_def.h)
    // #define USBD_STATE_DEFAULT     1
    // #define USBD_STATE_ADDRESSED   2
    // #define USBD_STATE_CONFIGURED  3
    // #define USBD_STATE_SUSPENDED   4

    while (!eject_detected)
    {
        // 检查是否有按键输入
        //        if (HAL_UART_Receive(&huart1, &dummy, 1, 100) == HAL_OK) {
        //            printf("检测到按键输入，返回主菜单\r\n");
        //            break;
        //        }

        // 检查USB状态变化
        uint32_t current_state = hUsbDeviceFS.dev_state;

        if (current_state != last_usb_state)
        {
            printf("USB状态变化: %u -> %u\r\n", last_usb_state, current_state);

            // 检测到挂起状态 (3 -> 4)
            if (last_usb_state == USBD_STATE_CONFIGURED &&
                current_state == USBD_STATE_SUSPENDED)
            {
                printf("检测到USB挂起，可能是Windows弹出操作\r\n");
                suspend_detected_time = HAL_GetTick();
            }
            else if (last_usb_state == USBD_STATE_ADDRESSED &&
                     current_state == USBD_STATE_SUSPENDED)
            {
                printf("检测到USB挂起2，可能是Windows弹出操作\r\n");
                suspend_detected_time = HAL_GetTick();
            }

            // 检测到断开 (从任何状态变为0或1)
            if (last_usb_state >= USBD_STATE_CONFIGURED &&
                current_state < USBD_STATE_ADDRESSED)
            {
                printf("检测到USB断开连接！\r\n");
                eject_detected = 1;
                break;
            }

            last_usb_state = current_state;
        }

        // 如果在挂起状态持续超过2秒，认为是弹出操作
        if (suspend_detected_time > 0)
        {
            uint32_t suspend_duration = HAL_GetTick() - suspend_detected_time;

            // 检查是否从挂起恢复
            if (current_state == USBD_STATE_CONFIGURED)
            {
                printf("USB从挂起恢复，继续等待\r\n");
                suspend_detected_time = 0;
            }
            // 挂起持续2秒后，触发重启
            else if (suspend_duration > 2000)
            {
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
    if (eject_detected)
    {
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

    while (1)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY) == HAL_OK)
        {
            // 处理退格键（BackSpace，ASCII码：0x08 或 0x7F）
            if ((ch == 0x08) || (ch == 0x7F))
            {
                if (input_len > 0) // 有字符可删时才处理
                {
                    input_len--;              // 输入长度减1
                    input_buf[input_len] = 0; // 清空最后一个字符

                    // 串口回显退格+空格+退格（实现“删除”视觉效果）
                    uint8_t backspace_seq[3] = {0x08, 0x20, 0x08};
                    HAL_UART_Transmit(&huart1, backspace_seq, 3, HAL_MAX_DELAY);
                }
                continue; // 跳过后续逻辑，继续读取字符
            }
            // 处理回车确认（换行/回车都识别）
            else if (ch == '\r' || ch == '\n')
            {
                printf("\r\n");
                break;
            }
            // 处理有效字符（数字1-9、0、b/B）
            else if (input_len < sizeof(input_buf) - 1)
            {
                // 允许输入：数字1-9、0、b、B
                if ((ch >= '1' && ch <= '9') || ch == '0' || ch == 'b' || ch == 'B')
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
            // 输入缓冲区已满，忽略后续字符
            else
            {
                continue;
            }
        }
    }

    // 解析输入
    if (input_len == 1)
    {
        // 数字1-9直接返回数字值
        if (input_buf[0] >= '1' && input_buf[0] <= '9')
        {
            return input_buf[0] - '0';
        }
        // 数字0返回0
        else if (input_buf[0] == '0')
        {
            return 'a';
        }
        // 字母b/B返回'b'的ASCII码（用于后续判断）
        else if (input_buf[0] == 'b' || input_buf[0] == 'B')
        {
            return 'b'; // 统一返回小写b，便于判断
        }
    }

    // 无效输入提示
    printf("输入无效！请重新输入（1-9选择文件，9下一页，0上一页，b返回）：");
    return 0x0; // 返回特殊值表示无效输入
}

// 擦除并查空验证（独立函数封装）
static void AM29_Get_Id_And_Read10(void)
{
    uint32_t am29lv320_id_32bit = 0;

    // 读取0x0000开始的多个数据（示例读取前5个半字）
    uint16_t data_buf[6] = {0};
    AM29LV320_Read_Data(0, 6, data_buf);

    // 读取AM29LV320 ID
    am29lv320_id_32bit = AM29_Read_ID();
    printf("AM29 ID: 0x%08X\r\n", am29lv320_id_32bit);

    // 读取并打印CFI容量信息
    uint16_t cfi_cap_val = AM29_Read_CFI_Capacity();
    uint32_t actual_cap = AM29_Parse_CFI_Capacity(cfi_cap_val);
    printf("AM29 CFI容量原始值: 0x%04X\r\n", cfi_cap_val);
    if (actual_cap > 0)
    {
        printf("AM29容量: %u Bytes (%u KByte)\r\n",
               actual_cap, (actual_cap / 1024));
    }
    else
    {
        printf("获取AM29容量错误\r\n");
    }

    // 显示前12byte
    printf("0x00~0x0B Data: ");
    for (int i = 0; i < 6; i++)
    {
        printf("0x%04X ", data_buf[i]);
    }
    printf("\r\n");

    // 显示
    OLED_Clear();
    printfOled(0, 1, "AM29 ID: 0x%08X", am29lv320_id_32bit);
    printfOled(1, 1, "CFI: 0x%04X", cfi_cap_val);
    printfOled(2, 1, "%u Bytes", actual_cap);
    printfOled(3, 1, "%u KBytes", actual_cap / 1024);
    printfOled(4, 1, "0x00-0x0B:");
    printfOled(5, 1, "0x%04X %04X %04X", data_buf[0], data_buf[1], data_buf[2]);
    printfOled(6, 1, "0x%04X %04X %04X", data_buf[3], data_buf[4], data_buf[5]);
    printfOled(7, 1, "Press any key");
    OLED_Refresh();

    // 等待任意按键后返回主菜单
    printf("\r\n操作完成，");
    Wait_Any_Key(0); // 无限等待任意按键
}

/**
 * @brief  读取AM29芯片全部内容并保存到TF卡
 * @retval uint8_t: 0 - 成功 | 其他 - 错误码
 */
uint8_t AM29_Read_To_File(void)
{
    FRESULT res;
    FIL file;
    FATFS fs;
    DIR dir;
    FILINFO fno;
    const TCHAR *vol = _T("0:");
    const TCHAR *path = _T("0:/AM29R");
    char file_path[260] = {0};
    uint32_t file_size = 0;
    uint32_t read_bytes = 0;
    uint32_t AM29_CAPACITY = 0;
    uint32_t next_file_num = 1;
    uint8_t buffer[512]; // 512字节缓冲区（对应256个半字）
    UINT bw = 0;

    // 定义变量：存储开始/结束的Unix时间戳
    uint32_t read_start_unix = 0;
    uint32_t read_end_unix = 0;
    uint32_t read_total_seconds = 0;
    RTC_TimeTypeDef read_start_time = {0};
    RTC_DateTypeDef read_start_date = {0};
    RTC_TimeTypeDef read_end_time = {0};
    RTC_DateTypeDef read_end_date = {0};

    // ===== 新增：64MB分块相关变量 =====
    uint32_t current_block = 0;           // 当前块号（0~3）
    uint32_t block_start_addr = 0;        // 当前块的起始地址
    uint32_t block_end_addr = BLOCK_64MB; // 当前块的结束地址
    uint32_t bytes_in_block = 0;          // 当前块已读取字节数

    printf("\r\n===== 开始读取AM29芯片内容到TF卡 =====\r\n");

    // 1. 获取AM29容量
    uint16_t cap = AM29_Read_CFI_Capacity();
    if (cap == 0)
    {
        printf("获取AM29容量失败！\r\n");
        OLED_Clear();
        printfOled(0, 1, "Read Failed!");
        printfOled(1, 1, "Get CFI Cap Fail");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
        return 99;
    }
    else if (cap > 0x1C)
    {
        printf("最大只支持S70GL02 256M\r\n");
        OLED_Clear();
        printfOled(0, 1, "Read Failed!");
        printfOled(1, 1, "Max support:256M");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
        return 98;
    }

    AM29_CAPACITY = AM29_Parse_CFI_Capacity(cap);
    printf("AM29容量: %lu Bytes (%lu MB)\r\n",
           (unsigned long)AM29_CAPACITY,
           (unsigned long)(AM29_CAPACITY / 1024 / 1024));

    // 2. 挂载TF卡
    printf("挂载TF卡...\r\n");
    res = f_mount(&fs, vol, 1);
    if (res != FR_OK)
    {
        printf("TF卡挂载失败! 错误码: %d\r\n", res);
        OLED_Clear();
        printfOled(0, 1, "Read Failed!");
        printfOled(1, 1, "TF NOT Found");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
        return 2;
    }

    // 3. 检查/创建AM29R目录
    res = f_opendir(&dir, path);
    if (res != FR_OK)
    {
        printf("AM29R目录不存在，尝试创建...\r\n");
        res = f_mkdir(path);
        if (res == FR_OK)
        {
            printf("AM29R目录创建成功\r\n");
        }
        else
        {
            printf("创建AM29R目录失败! 错误码: %d\r\n", res);
            OLED_Clear();
            printfOled(0, 1, "Read Failed!");
            printfOled(1, 1, "Create AM29R Dir");
            printfOled(2, 1, "Failed!");
            printfOled(7, 1, "Press any key");
            OLED_Refresh();
            f_mount(NULL, vol, 1);
            return 3;
        }
    }
    else
    {
        f_closedir(&dir);
        printf("AM29R目录已存在\r\n");
    }

    // 4. 查找下一个可用的文件名编号
    printf("扫描AM29R目录中的文件...\r\n");
    res = f_opendir(&dir, path);
    if (res == FR_OK)
    {
        uint32_t max_num = 0;
        while (1)
        {
            res = f_readdir(&dir, &fno);
            if (res != FR_OK || fno.fname[0] == 0)
                break;

            if ((fno.fattrib & AM_DIR) == 0)
            {
                uint8_t name_len = strlen(fno.fname);
                if (name_len == 12)
                {
                    char *suffix = strrchr(fno.fname, '.');
                    if (suffix && (strcmp(suffix, ".bin") == 0 || strcmp(suffix, ".BIN") == 0))
                    {
                        uint8_t all_digit = 1;
                        for (int i = 0; i < 8; i++)
                        {
                            if (fno.fname[i] < '0' || fno.fname[i] > '9')
                            {
                                all_digit = 0;
                                break;
                            }
                        }

                        if (all_digit)
                        {
                            char num_str[9] = {0};
                            strncpy(num_str, fno.fname, 8);
                            uint32_t file_num = atoi(num_str);
                            if (file_num > max_num)
                            {
                                max_num = file_num;
                            }
                        }
                    }
                }
            }
        }
        f_closedir(&dir);
        next_file_num = max_num + 1;
        printf("最大文件编号: %08lu, 下一个编号: %08lu\r\n",
               (unsigned long)max_num, (unsigned long)next_file_num);
    }

    // 5. 生成文件名
    sprintf(file_path, "%s/%08lu.bin", path, (unsigned long)next_file_num);
    printf("准备创建文件: %s\r\n", file_path);

    // 6. 创建文件
    res = f_open(&file, file_path, FA_CREATE_NEW | FA_WRITE);
    if (res != FR_OK)
    {
        printf("创建文件失败! 错误码: %d\r\n", res);
        OLED_Clear();
        printfOled(0, 1, "Read Failed!");
        printfOled(1, 1, "Create File Failed");
        printfOled(2, 1, "%s", file_path);
        printfOled(3, 1, "Code:%d", res);
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
        f_mount(NULL, vol, 1);
        return 4;
    }

    // 7. 获取开始时间
    if (HAL_RTC_GetTime(&hrtc, &read_start_time, RTC_FORMAT_BIN) == HAL_OK)
    {
        HAL_RTC_GetDate(&hrtc, &read_start_date, RTC_FORMAT_BIN);
        read_start_unix = rtc_to_unix_timestamp(&read_start_date, &read_start_time);
        printf("读取开始时间：%04d-%02d-%02d %02d:%02d:%02d\r\n",
               2000 + read_start_date.Year, read_start_date.Month, read_start_date.Date,
               read_start_time.Hours, read_start_time.Minutes, read_start_time.Seconds);
    }

    // 8. 分块读取芯片内容
    printf("开始读取芯片内容（分64MB块处理）...\r\n");

    // 循环处理每个64MB块
    uint32_t total_blocks = AM29_CAPACITY / BLOCK_64MB; // 总块数
    if (total_blocks == 0)
    {
        total_blocks = 1;
    }

    for (current_block = 0; current_block < total_blocks; current_block++)
    {
        block_start_addr = current_block * BLOCK_64MB;
        block_end_addr = block_start_addr + BLOCK_64MB;
        bytes_in_block = 0;

        printf("\r\n===== 处理第 %lu 块  =====\r\n",
               (unsigned long)current_block + 1);

        // 8.1 设置A26~A31对应当前块
        Set_A25_A31(block_start_addr);

        // 8.4 读取当前块数据
        while (bytes_in_block < BLOCK_64MB && read_bytes < AM29_CAPACITY)
        {
            uint32_t block_size = 512; // 每次读取512字节
            if (read_bytes + block_size > AM29_CAPACITY)
            {
                block_size = AM29_CAPACITY - read_bytes;
            }

            // 在当前块内的偏移地址（0~64MB-1）
            uint32_t offset_in_block = bytes_in_block;
            uint32_t halfword_count = block_size / 2;
            uint32_t start_halfword = offset_in_block / 2; // 在当前块内的半字偏移

            // 读取数据（使用块内偏移）
            for (uint32_t i = 0; i < halfword_count; i++)
            {
                uint16_t data = *(FSMC_NOR_BASE_ADDR + start_halfword + i);
                buffer[i * 2] = (data >> 8) & 0xFF; // 高字节
                buffer[i * 2 + 1] = data & 0xFF;    // 低字节
            }

            // 写入文件
            res = f_write(&file, buffer, block_size, &bw);
            if (res != FR_OK || bw != block_size)
            {
                printf("文件写入失败! 错误码: %d\r\n", res);
                OLED_Clear();
                printfOled(0, 1, "Read Failed!");
                printfOled(1, 1, "File write fail");
                printfOled(2, 1, "%s", file_path);
                printfOled(3, 1, "Code:%d", res);
                printfOled(7, 1, "Press any key");
                OLED_Refresh();
                f_close(&file);
                f_mount(NULL, vol, 1);
                return 5;
            }

            read_bytes += block_size;
            bytes_in_block += block_size;

            // 每读取128KB显示进度
            if (read_bytes % (128 * 1024) == 0 || read_bytes == AM29_CAPACITY)
            {
                float percent = (float)read_bytes / AM29_CAPACITY * 100;
                printf("读取进度：%lu/%lu 字节 (%.1f%%) | 当前块: %u/%u\r\n",
                       (unsigned long)read_bytes,
                       (unsigned long)AM29_CAPACITY,
                       percent,
                       current_block + 1,
                       total_blocks);

                // OLED显示进度
                OLED_Clear();
                printfOled(0, 1, "Read: %.1f%%", percent);
                printfOled(1, 1, "Block: %u/%u", current_block + 1, total_blocks);
                OLED_Refresh();

                // LED闪烁提示
                HAL_GPIO_TogglePin(LEDG_PORT, LEDG_PIN);
            }
        }
    }

    // 9. 获取结束时间并计算耗时
    if (HAL_RTC_GetTime(&hrtc, &read_end_time, RTC_FORMAT_BIN) == HAL_OK)
    {
        HAL_RTC_GetDate(&hrtc, &read_end_date, RTC_FORMAT_BIN);
        read_end_unix = rtc_to_unix_timestamp(&read_end_date, &read_end_time);
        read_total_seconds = read_end_unix - read_start_unix;

        printf("读取结束时间：%04d-%02d-%02d %02d:%02d:%02d\r\n",
               2000 + read_end_date.Year, read_end_date.Month, read_end_date.Date,
               read_end_time.Hours, read_end_time.Minutes, read_end_time.Seconds);
        printf("总耗时：%u 秒\r\n", read_total_seconds);
    }

    // 10. 关闭文件并卸载TF卡
    f_close(&file);
    f_mount(NULL, vol, 1);

    // 11. 复位所有高位地址线和芯片
    Set_A25_A31_All_Zero();
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
    HAL_Delay(1);

    printf("\r\n===== 读取完成！文件已保存为：%s =====\r\n", file_path);
    printf("文件大小：%u 字节\r\n", read_bytes);

    // OLED显示
    OLED_Clear();
    printfOled(0, 1, "Read Success!");
    printfOled(1, 1, "%u S", read_total_seconds);
    printfOled(2, 1, "%s", file_path);
    printfOled(3, 1, "%uKBytes", read_bytes / 1024);
    printfOled(7, 1, "Press any key");
    OLED_Refresh();

    if (read_total_seconds > 60)
    {
        buz();
    }

    return 0;
}

// 擦除并查空验证（独立函数封装）
static void AM29_Erase_And_Verify(void)
{
    uint8_t confirm = 0;
    uint8_t key = 0;

    // 显示警告信息
    printf("\r\n========================================\r\n");
    printf("    警告：即将执行整芯片擦除操作！\r\n");
    printf("    擦除后所有数据将丢失！\r\n");
    printf("========================================\r\n");

    // 在OLED上显示警告
    OLED_Clear();
    printfOled(0, 1, "WARNING!");
    printfOled(1, 1, "Chip Erase");
    printfOled(2, 1, "All data will");
    printfOled(3, 1, "be lost!");
    printfOled(4, 1, "Press OK");
    printfOled(5, 1, "to continue");
    printfOled(6, 1, "Press others");
    printfOled(7, 1, "to cancel");
    OLED_Refresh();

    printf("请输入 Y 确认擦除，或按确认键继续，其他键取消：\r\n");

    // 等待用户确认（超时30秒）
    key = Wait_Any_Key(30000);

    // 检查确认条件：Y/y 或者 确认键
    if (key == 'Y' || key == 'y' || key == 5) // 4=右键, 5=确认键
    {
        printf("\r\n用户已确认，开始擦除芯片...\r\n");

        // OLED显示擦除中
        OLED_Clear();
        printfOled(0, 1, "Erasing...");
        printfOled(1, 1, "Please wait");
        OLED_Refresh();

        uint8_t erase_ret = AM29_Chip_Erase();
    }
    else if (key != 0)
    {
        printf("\r\n操作已取消\r\n");

        OLED_Clear();
        printfOled(0, 1, "Operation");
        printfOled(1, 1, "Cancelled");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
    }
    else
    {
        printf("\r\n等待超时，操作已取消\r\n");

        OLED_Clear();
        printfOled(0, 1, "Timeout");
        printfOled(1, 1, "Cancelled");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
    }

    // 等待任意按键返回主菜单
    Wait_Any_Key(0);
}

// 写入BIN文件并校验
static void AM29_Write_And_Verify(void)
{
    uint8_t bin_file_cnt = 0;
    AM29_File_Info_t *bin_file_list = NULL;
    uint8_t current_page = 0;
    uint8_t total_pages = 0;
    uint8_t user_choice = 0;
    uint8_t selected_file_index = 0;
    uint8_t file_selected = 0;
    char selected_filename[256] = {0};
    uint8_t current_selection = 0; // 当前选中的文件在当前页的索引(0-7)
    uint8_t last_selection = 0;    // 上一次选中的行，用于恢复反色

    // 获取AM29目录下的所有.bin文件
    printf("\r\n===== 正在扫描AM29目录下的.bin文件 =====\r\n");
    bin_file_list = AM29_List_Bin_Files(&bin_file_cnt);

    if (bin_file_list == NULL || bin_file_cnt == 0)
    {
        printf("AM29目录下没有找到.bin文件，请先放入文件再操作\r\n");

        // OLED显示无文件
        OLED_Clear();
        printfOled(0, 1, "No BIN files");
        printfOled(1, 1, "found in AM29");
        printfOled(2, 1, "directory");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
        Wait_Any_Key(0);
        return;
    }

    // 计算总页数（每页8个文件）
    total_pages = (bin_file_cnt + 7) / 8; // 向上取整

    // 初始化上一次选中行
    last_selection = 0;

    // 循环让用户选择文件
    while (!file_selected)
    {
        // 计算当前页的文件范围
        uint8_t start_idx = current_page * 8;
        uint8_t end_idx = start_idx + 8;
        if (end_idx > bin_file_cnt)
            end_idx = bin_file_cnt;
        uint8_t files_on_page = end_idx - start_idx;

        // 初始化当前选择（如果当前选择超出范围，重置为0）
        if (current_selection >= files_on_page)
        {
            current_selection = 0;
        }

        // 显示OLED菜单（8行全部显示文件）
        OLED_Clear();

        // 显示所有文件（不带反色）
        for (uint8_t i = 0; i < 8; i++)
        {
            if (i < files_on_page)
            {
                uint8_t file_idx = start_idx + i;

                // 显示文件名（截断到合适长度）
                char display_name[17] = {0}; // 16字符 + 终止符
                strncpy(display_name, bin_file_list[file_idx].filename, 16);

                // 所有行先正常显示（不带箭头）
                printfOled(i, 1, " %s", display_name); // mode=1 正常显示
            }
            else
            {
                // 空白行
                printfOled(i, 1, "                "); // 16个空格，mode=1正常显示
            }
        }

        // 单独处理当前选中行：加箭头并设置反色
        if (current_selection < files_on_page)
        {
            uint8_t file_idx = start_idx + current_selection;
            char display_name[17] = {0};
            strncpy(display_name, bin_file_list[file_idx].filename, 16);

            // 重新显示当前行带箭头（用正常模式显示，然后取反）
            printfOled(current_selection, 1, ">%s", display_name); // mode=1正常显示

            // 对整行取反实现反色效果
            OLED_MenuItemReverse(current_selection, 1); // 1=反色
        }

        OLED_Refresh();

        // 串口显示（保持不变）
        printf("\r\n========================================\r\n");
        printf("===== 请选择要写入的BIN文件 (共%d个) =====\r\n", bin_file_cnt);
        printf("========================================\r\n");

        for (uint8_t i = start_idx; i < end_idx; i++)
        {
            uint8_t display_num = (i - start_idx) + 1; // 当前页内的编号 1-8
            printf("%d - %s (%lu 字节)\r\n",
                   display_num,
                   bin_file_list[i].filename,
                   (unsigned long)bin_file_list[i].filesize);
        }

        printf("\r\n");

        if (total_pages > 1)
        {
            printf("当前第 %d/%d 页\r\n", current_page + 1, total_pages);
            printf("9 - 下一页\r\n");
            printf("0 - 上一页\r\n");
        }

        printf("b - 返回主菜单\r\n");
        printf("请选择 (1-%d, 9下一页, 0上一页, b返回, 或使用上下键选择按OK确认): ", files_on_page);

        // 获取用户输入（同时检测按键和串口）
        uint8_t input_detected = 0;
        uint8_t key = 0;

        while (!input_detected)
        {
            // 检测按键
            if (KEY_UP_Detect())
            {
                if (current_selection > 0)
                {
                    current_selection--;
                }
                else
                {
                    // 如果在第一项，按上键跳到上一页的最后一项
                    if (current_page > 0)
                    {
                        current_page--;
                        // 重新计算上一页的文件数量
                        uint8_t prev_start = current_page * 8;
                        uint8_t prev_end = prev_start + 8;
                        if (prev_end > bin_file_cnt)
                            prev_end = bin_file_cnt;
                        current_selection = (prev_end - prev_start) - 1;
                    }
                }
                input_detected = 1;
                break;
            }

            if (KEY_DOWN_Detect())
            {
                uint8_t files_on_current = end_idx - start_idx;
                if (current_selection < files_on_current - 1)
                {
                    current_selection++;
                }
                else
                {
                    // 如果在最后一项，按下键跳到下一页的第一项
                    if (current_page < total_pages - 1)
                    {
                        current_page++;
                        current_selection = 0;
                    }
                }
                input_detected = 1;
                break;
            }

            if (KEY_LEFT_Detect()) // 左键上一页
            {
                if (current_page > 0)
                {
                    current_page--;
                    current_selection = 0;
                    input_detected = 1;
                }
                break;
            }

            if (KEY_RIGHT_Detect()) // 右键下一页
            {
                if (current_page < total_pages - 1)
                {
                    current_page++;
                    current_selection = 0;
                    input_detected = 1;
                }
                break;
            }

            if (KEY_OK_Detect())
            {
                // 确认选择当前文件
                selected_file_index = start_idx + current_selection;
                if (selected_file_index < bin_file_cnt)
                {
                    strncpy(selected_filename, bin_file_list[selected_file_index].filename, sizeof(selected_filename) - 1);
                    file_selected = 1;
                    input_detected = 1;
                }
                break;
            }

            if (KEY_CANCEL_Detect())
            {
                printf("已返回主菜单\r\n");

                OLED_Clear();
                printfOled(0, 1, "Return to");
                printfOled(1, 1, "Main Menu");
                printfOled(7, 1, "Press any key");
                OLED_Refresh();
                Wait_Any_Key(0);
                return;
            }

            // 检测串口输入（非阻塞）
            if (HAL_UART_Receive(&huart1, &key, 1, 0) == HAL_OK)
            {
                // 回显字符
                if (key >= 0x20 && key <= 0x7E)
                {
                    HAL_UART_Transmit(&huart1, &key, 1, HAL_MAX_DELAY);
                }

                // 处理数字输入
                if (key >= '1' && key <= '8')
                {
                    uint8_t choice_num = key - '0';
                    if (choice_num <= files_on_page)
                    {
                        selected_file_index = start_idx + (choice_num - 1);
                        if (selected_file_index < bin_file_cnt)
                        {
                            strncpy(selected_filename, bin_file_list[selected_file_index].filename, sizeof(selected_filename) - 1);
                            file_selected = 1;
                            input_detected = 1;
                            printf("\r\n");
                        }
                    }
                }
                else if (key == '9') // 下一页
                {
                    if (current_page < total_pages - 1)
                    {
                        current_page++;
                        current_selection = 0;
                        input_detected = 1;
                        printf("\r\n");
                    }
                    else
                    {
                        printf("\r\n已经是最后一页了！\r\n");
                    }
                }
                else if (key == '0') // 上一页
                {
                    if (current_page > 0)
                    {
                        current_page--;
                        current_selection = 0;
                        input_detected = 1;
                        printf("\r\n");
                    }
                    else
                    {
                        printf("\r\n已经是第一页了！\r\n");
                    }
                }
                else if (key == 'b' || key == 'B') // 返回主菜单
                {
                    printf("\r\n已返回主菜单\r\n");

                    OLED_Clear();
                    printfOled(0, 1, "Return to");
                    printfOled(1, 1, "Main Menu");
                    printfOled(7, 1, "Press any key");
                    OLED_Refresh();
                    Wait_Any_Key(0);
                    return;
                }
                break;
            }

            HAL_Delay(20); // 适当延时，避免CPU占用过高
        }
    }

    // 确认选择
    printf("\r\n您选择了文件：%s\r\n", selected_filename);

    // OLED显示选择的文件
    OLED_Clear();
    printfOled(0, 1, "Selected:");

    // 显示文件名（可能很长，需要分行）
    char *filename = selected_filename;
    uint8_t name_len = strlen(filename);
    if (name_len <= 16)
    {
        printfOled(1, 1, "%s", filename);
    }
    else
    {
        // 分行显示
        char line1[17] = {0};
        char line2[17] = {0};
        strncpy(line1, filename, 16);
        strncpy(line2, filename + 16, 16);
        printfOled(1, 1, "%s", line1);
        printfOled(2, 1, "%s", line2);
    }

    printfOled(4, 1, "Press OK to");
    printfOled(5, 1, "start write");
    printfOled(6, 1, "Press Cancel");
    printfOled(7, 1, "to abort");
    OLED_Refresh();

    printf("UART上按y键开始写入，按任意键取消...\r\n");

    // 等待用户确认
    uint8_t confirm = 0;
    uint8_t ch = 0;
    uint32_t confirm_timeout = HAL_GetTick();

    while ((HAL_GetTick() - confirm_timeout) < 10000) // 10秒超时
    {
        if (KEY_OK_Detect())
        {
            confirm = 1;
            break;
        }

        if (KEY_CANCEL_Detect())
        {
            confirm = 0;
            break;
        }

        // 检测串口回车
        if (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
        {
            if (ch == 'Y' || ch == 'y')
            {
                confirm = 1;
                break;
            }
            else
            {
                confirm = 0;
                break;
            }
        }

        HAL_Delay(50);
    }

    if (confirm)
    {
        printf("\r\n===== 开始写入文件: %s =====\r\n", selected_filename);

        // OLED显示开始写入
        OLED_Clear();
        printfOled(0, 1, "Writing...");
        printfOled(1, 1, "%s", selected_filename);
        OLED_Refresh();

        // 执行写入操作
        uint8_t file_write_ret = AM29_Write_Data_From_File(selected_filename);

        //        if (file_write_ret == 0)
        //        {
        //            printf("\r\n===== %s 写入成功 =====\r\n", selected_filename);
        //
        //            OLED_Clear();
        //            printfOled(0,1,"Write Success!");
        //            printfOled(1,1,"%s", selected_filename);
        //            printfOled(7,1,"Press any key");
        //            OLED_Refresh();
        //        }
        //        else
        //        {
        //            printf("\r\n===== %s 写入失败，错误码：%d =====\r\n", selected_filename, file_write_ret);
        //
        //            OLED_Clear();
        //            printfOled(0,1,"Write Failed!");
        //            printfOled(1,1,"Error: %d", file_write_ret);
        //            printfOled(7,1,"Press any key");
        //            OLED_Refresh();
        //        }

        // 等待任意按键返回主菜单
        Wait_Any_Key(0);
    }
    else
    {
        printf("操作已取消\r\n");

        OLED_Clear();
        printfOled(0, 1, "Operation");
        printfOled(1, 1, "Cancelled");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
        Wait_Any_Key(0);
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
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND; // 自动分频，1秒计数
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
static const uint8_t month_days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

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
        total_seconds += month_days[m - 1] * 86400; // 每个月的天数×86400秒/天
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
void HAL_RTC_MspInit(RTC_HandleTypeDef *rtcHandle)
{
    if (rtcHandle->Instance == RTC)
    {
        __HAL_RCC_RTC_ENABLE();
    }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef *rtcHandle)
{
    if (rtcHandle->Instance == RTC)
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
    if (HAL_GPIO_ReadPin(KEY_CANCEL_PORT, KEY_CANCEL_PIN) == GPIO_PIN_RESET)
    {
        // 2. 10ms消抖（过滤物理抖动）
        HAL_Delay(10);
        if (HAL_GPIO_ReadPin(KEY_CANCEL_PORT, KEY_CANCEL_PIN) == GPIO_PIN_RESET)
        {
            key_flag = 1;
            // 3. 等待按键释放（避免长按重复触发）
            while (HAL_GPIO_ReadPin(KEY_CANCEL_PORT, KEY_CANCEL_PIN) == GPIO_PIN_RESET)
                ;
        }
    }
    return key_flag;
}

void delay_us(uint32_t us)
{
    // 1us = 1000ns
    delayns(us * 1000);
}

// 读取AM29的JEDEC ID，返回32位数据：[31:16]=制造商ID，[15:0]=设备ID
uint32_t AM29_Read_ID(void)
{
    // 步骤1：强制复位芯片，退出所有特殊模式（确保初始状态）
    *(FSMC_NOR_BASE_ADDR + 0x00) = AM29_RESET_CMD;
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
    am29_mfg_id = *(FSMC_NOR_BASE_ADDR + 0x00); // 字节0x0000 → 制造商ID (0x0122)
    am29_dev_id = *(FSMC_NOR_BASE_ADDR + 0x01); // 字节0x0001 → 设备ID (0x00A4)
    am29_cap_id = *(FSMC_NOR_BASE_ADDR + 0x02); // 字节0x0002 → 容量ID (0x0080)

    // 步骤4：发送复位指令，退出ID模式，回到正常数据读取模式
    *(FSMC_NOR_BASE_ADDR + 0x00) = AM29_RESET_CMD;
    HAL_Delay(1);

    // 组合32位ID返回
    return ((uint32_t)am29_mfg_id << 16) | am29_dev_id;
}

// 新增：读取AM29LV320指定地址的内容（默认读取0x0000地址）
// 参数：addr_halfword - 半字偏移地址（0x0000字节地址对应0，0x0002对应1，以此类推）
//       len          - 读取的半字数量（1个半字=2字节）
//       buf          - 存储读取数据的缓冲区（uint16_t类型）
void AM29LV320_Read_Data(uint16_t addr_halfword, uint16_t len, uint16_t *buf)
{
    if (buf == NULL || len == 0)
        return; // 入参校验

    // 步骤1：发送复位指令，确保芯片处于正常读模式（避免ID模式干扰）
    *(FSMC_NOR_BASE_ADDR + 0x00) = AM29_RESET_CMD;
    HAL_Delay(1);

    // 步骤2：读取指定地址的数据（16位总线直接访问）
    for (uint16_t i = 0; i < len; i++)
    {
        // 半字偏移累加，直接读取FSMC映射地址
        buf[i] = *(FSMC_NOR_BASE_ADDR + addr_halfword + i);
        // 可选：添加短延时，适配低速芯片（如AM29LV320读时序要求）
        // HAL_Delay(1);
    }
}

// 从CFI中读取AM29LV320的容量原始值
uint16_t AM29_Read_CFI_Capacity(void)
{
    uint16_t cfi_capacity = 0;

    // 步骤1：复位芯片，退出所有特殊模式（确保初始状态）
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
    HAL_Delay(1); // 确保复位指令生效

    // 步骤2：发送进入CFI模式的命令（ADDR:0x0055 DATA:0x0098）
    *(FSMC_NOR_BASE_ADDR + AM29_CFI_ENTRY_ADDR) = AM29_CFI_ENTRY_DATA;
    delaycmd();

    // 步骤3：读取0x0027地址的容量值
    cfi_capacity = *(FSMC_NOR_BASE_ADDR + AM29_CFI_CAP_ADDR);
    delaycmd();

    // 步骤4：复位芯片，退出CFI模式，恢复正常读模式
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
    HAL_Delay(1);

    if (cfi_capacity == 0xff)
    {
        printf("异常容量");
        return 0;
    }
    else if (cfi_capacity == 0x0fff)
    {
        printf("异常容量");
        return 0;
    }
    else if (cfi_capacity == 0xffff)
    {
        printf("异常容量");
        return 0;
    }

    return cfi_capacity;
}

// 新增：解析CFI容量值为实际字节容量
uint32_t AM29_Parse_CFI_Capacity(uint16_t cfi_value)
{
    if (cfi_value == 0)
        return 0;                    // 异常值保护
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
uint8_t AM29_Chip_Erase(void)
{
    Set_A25_A31_All_Zero();
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

    uint16_t cap = AM29_Read_CFI_Capacity();
    if (cap == 0)
    {
        // 获取容量失败
        printf("获取容量失败\r\n");

        // OLED显示失败
        OLED_Clear();
        printfOled(0, 1, "Erase Failed!");
        printfOled(1, 1, "Get CFI Cap Fail");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        return 99;
    }
    else if (cap <= 0x14)
    {
        // 4M
        max_timeout = 300;
    }
    else if (cap >= 0x15 && cap <= 0x1C)
    {
        max_timeout = 300 * (cap - 0x14);
    }
    else
    {
        printf("最大支持256MByte(S70GL02)\r\n");

        // OLED显示失败
        OLED_Clear();
        printfOled(0, 1, "Erase Failed!");
        printfOled(1, 1, "Max support:256M");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        return 98;
    }

    // ===== 计算芯片总容量和分块参数 =====
    uint32_t AM29_CAPACITY = AM29_Parse_CFI_Capacity(cap);
    uint32_t total_blocks = AM29_CAPACITY / BLOCK_64MB; // 总块数
    if (total_blocks == 0)
    {
        total_blocks = 1;
    }
    uint32_t chip_total_halfwords = AM29_CAPACITY >> 1; // 总半字数

    printf("芯片容量: %u MB, 总块数: %u, 总半字数: %u\r\n",
           (AM29_CAPACITY / 1024 / 1024),
           total_blocks,
           chip_total_halfwords);

    // 全芯片擦除验证：逐地址读取并检查是否为0xFFFF
    uint32_t error_count = 0;             // 统计非0xFFFF的地址数量
    uint32_t total_checked_halfwords = 0; // 已检查的地址总数
    uint32_t current_block = 0;
    uint32_t block_start_halfword = 0;
    uint32_t block_end_halfword = BLOCK_64MB / 2; // 每块64MB对应的半字数
    uint32_t halfwords_per_block = BLOCK_64MB / 2;
    uint32_t halfwords_checked_in_block = 0;

    /************************** 1. 初始化RY/BY#引脚 **************************/
    // 使能GPIOD时钟
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = RY_BY_PIN;             // PD6引脚
    gpio_init.Mode = GPIO_MODE_INPUT;      // 输入模式
    gpio_init.Pull = GPIO_PULLUP;          // 启用内部上拉(外部无上拉时关键)
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW; // 低速模式
    HAL_GPIO_Init(RY_BY_PORT, &gpio_init);

    for (current_block = 0; current_block < total_blocks; current_block = current_block + 2)
    {
        // 复位芯片, 确保退出之前的异常状态
        *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
        HAL_Delay(100);

        block_start_halfword = current_block * halfwords_per_block;

        printf("\r\n第 %u 块\r\n", current_block + 1);

        // 设置A26~A31对应当前块
        Set_A25_A31(current_block * BLOCK_64MB);

        /************************** 2. 发送擦除命令序列 **************************/
        // 严格按照AM29LV320手册的擦除命令时序
        *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00AA; // 解锁序列1
        delaycmd();
        *(FSMC_NOR_BASE_ADDR + 0x2AA) = 0x0055; // 解锁序列2
        delaycmd();
        *(FSMC_NOR_BASE_ADDR + 0x555) = 0x0080; // 擦除解锁1
        delaycmd();
        *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00AA; // 擦除解锁2
        delaycmd();
        *(FSMC_NOR_BASE_ADDR + 0x2AA) = 0x0055; // 擦除解锁3
        delaycmd();
        *(FSMC_NOR_BASE_ADDR + 0x555) = 0x0010; // 触发整芯片擦除
        HAL_Delay(20);                          // 等待芯片进入擦除状态

        // 读取并检查擦除状态
        ry_by_state = HAL_GPIO_ReadPin(RY_BY_PORT, RY_BY_PIN);
        printf("正在检查是否进入擦除\r\n");

        // 验证是否成功进入擦除状态
        if (ry_by_state == GPIO_PIN_RESET)
        {
            printf("RY/BY# is 0, 成功进入擦除状态\r\n");
        }
        else
        {
            printf("RY/BY# is 1, 擦除状态进入失败\r\n");

            // OLED显示失败
            OLED_Clear();
            printfOled(0, 1, "Erase Failed!");
            printfOled(1, 1, "RY/BY# not 1");
            printfOled(2, 1, "Block: %u", current_block + 1);
            printfOled(7, 1, "Press any key");
            OLED_Refresh();

            return 2; // 进入擦除状态失败
        }

        /************************** 3. 循环检测擦除状态 **************************/
        uint8_t ry_by_1_count = 0;
        while (1)
        {
            // 超时保护
            if (timeout_count > max_timeout)
            {
                printf("擦除超时! (Count: %u)\r\n", timeout_count);

                // OLED显示失败
                OLED_Clear();
                printfOled(0, 1, "Erase Failed!");
                printfOled(1, 1, "TimeOut:%u", timeout_count);
                printfOled(7, 1, "Press any key");
                OLED_Refresh();

                // 复位芯片退出擦除状态
                *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
                HAL_Delay(10);
                return 1; // 超时错误
            }

            // 读取RY/BY#引脚状态并打印
            ry_by_state = HAL_GPIO_ReadPin(RY_BY_PORT, RY_BY_PIN);
            printf("计时:%ds, RY/BY#:%d\r\n", timeout_count,
                   (ry_by_state == GPIO_PIN_SET) ? 1 : 0);

            // OLED显示过程
            OLED_Clear();
            printfOled(0, 1, "Erasing:%ds", timeout_count);
            printfOled(1, 1, "RY/BY#:%d", (ry_by_state == GPIO_PIN_SET) ? 1 : 0);
            printfOled(2, 1, "Block: %u/%u", current_block + 1, total_blocks);
            OLED_Refresh();

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
                printf("擦除状态: Q7=%d, Q6=%d, Q5=%d, RY/BY#=%d\r\n",
                       q7, q6, q5, (ry_by_state == GPIO_PIN_SET) ? 1 : 0);

                // 擦除完成判断条件
                if (ry_by_state == 1 && q7 == 1 && q6 == 1 && q5 == 1)
                {
                    if (ry_by_1_count >= 3)
                    {
                        erase_ok = 1;
                        break; // 擦除完成, 退出循环
                    }
                    else
                    {
                        ry_by_1_count++;
                    }
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
               2000 + erase_end_date.Year, erase_end_date.Month, erase_end_date.Date,
               erase_end_time.Hours, erase_end_time.Minutes, erase_end_time.Seconds);

        // 3. 计算擦除耗时（直接相减，无需处理跨时段）
        erase_total_seconds = erase_end_unix - erase_start_unix;
        printf("AM29芯片擦除耗时：%u 秒\r\n", erase_total_seconds);
    }

    // 打印最终验证结果
    printf("全芯片验证完成：共校验%lu个半字 | 非0xFFFF地址数：%lu\r\n",
           (unsigned long)total_checked_halfwords,
           (unsigned long)error_count);

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

    /************************** 4. 复位芯片并验证擦除结果 **************************/
    // 复位芯片, 退出状态查询模式
    *(FSMC_NOR_BASE_ADDR + 0x0000) = 0x00F0;
    HAL_Delay(10); // 确保复位生效

    printf("开始全芯片擦除验证，总校验半字数：%lu\r\n", (unsigned long)chip_total_halfwords);

    // 逐块验证（每64MB一块）
    for (current_block = 0; current_block < total_blocks; current_block++)
    {
        block_start_halfword = current_block * halfwords_per_block;

        printf("\r\n===== 验证第 %lu 块 (半字地址范围: 0x%08lX - 0x%08lX) =====\r\n",
               (unsigned long)current_block + 1,
               (unsigned long)block_start_halfword,
               (unsigned long)(block_start_halfword + halfwords_per_block - 1));

        // 设置A26~A31对应当前块
        Set_A25_A31(current_block * BLOCK_64MB);

        // 验证当前块内的所有半字（块内偏移从0开始）
        for (halfwords_checked_in_block = 0;
             halfwords_checked_in_block < halfwords_per_block &&
             total_checked_halfwords < chip_total_halfwords;
             halfwords_checked_in_block++)
        {
            // 在当前块内使用偏移地址读取
            uint16_t current_data = *(FSMC_NOR_BASE_ADDR + halfwords_checked_in_block);

            // 检查是否为擦除后的预期值0xFFFF
            if (current_data != 0xFFFF)
            {
                error_count++;
                // 计算全局半字地址用于显示
                uint32_t global_halfword = block_start_halfword + halfwords_checked_in_block;
                // 每0x20000个错误打印一次（避免串口刷屏）
                if (error_count % 0x20000 == 0)
                {
                    printf("验证异常：全局地址0x%08X (块内偏移0x%08X) 数据0x%04X | 累计错误数：%lu\r\n",
                           (uint32_t)(FSMC_NOR_BASE_ADDR + global_halfword),
                           (uint32_t)halfwords_checked_in_block,
                           current_data, (unsigned long)error_count);
                }
            }

            total_checked_halfwords++;

            // 每校验0x40000个地址打印进度（提升交互性）
            if (total_checked_halfwords % 0x40000 == 0)
            {
                HAL_GPIO_TogglePin(LEDG_PORT, LEDG_PIN);
                printf("验证进度：%u/%u 半字 | 当前错误数：%u | 当前块: %u/%u\r\n",
                       total_checked_halfwords,
                       chip_total_halfwords,
                       error_count,
                       current_block + 1,
                       total_blocks);

                // OLED显示过程
                OLED_Clear();
                printfOled(0, 1, "Blank Check");
                uint64_t progress = total_checked_halfwords;
                uint64_t total = chip_total_halfwords;
                uint64_t percent_x1000 = (progress * 1000) / total;

                uint32_t percent_int = 0;
                uint32_t percent_frac = 0;
                percent_int = (uint32_t)(percent_x1000 / 10);
                percent_frac = (uint32_t)(percent_x1000 % 10);
                printfOled(1, 1, "%u.%u %%", percent_int, percent_frac);

                printfOled(2, 1, "Block: %u/%u", current_block + 1, total_blocks);
                printfOled(3, 1, "%u", total_checked_halfwords);
                printfOled(4, 1, "%u", chip_total_halfwords);
                OLED_Refresh();
            }
        }
    }

    /************************** 5. 复位所有高位地址线和芯片 **************************/
    Set_A25_A31_All_Zero();
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
    HAL_Delay(1);

    /************************** 6. 返回擦除结果 **************************/
    if (erase_ok)
    {
        // OLED显示
        OLED_Clear();
        printfOled(0, 1, "Erase Success");
        printfOled(1, 1, "Time:%us", erase_total_seconds);
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        if (erase_total_seconds > 60)
        {
            buz();
        }

        return 0; // 擦除成功
    }
    else
    {
        // OLED显示
        OLED_Clear();
        printfOled(0, 1, "Erase Failed");
        printfOled(1, 1, "Not 0xFFFF Count:");
        printfOled(2, 1, "%lu", (unsigned long)error_count);
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        if (erase_total_seconds > 60)
        {
            buz();
        }

        return 2; // 状态错误
    }
}

/**
 * @brief  检查AM29LV320芯片指定区域是否为空白（0xFFFF）
 * @param  start_addr: 起始字节地址（如0x00000000，必须偶对齐）
 * @param  check_len:  检查的字节长度（如0x100 = 256字节）
 * @param  show_details: 是否显示详细错误信息（1显示，0不显示）
 * @retval uint8_t: 0 - 区域完全空白（所有数据为0xFFFF）
 *                  1 - 区域非空白（存在非0xFFFF数据）
 *                  2 - 参数错误
 */
uint8_t AM29LV320_Check_Blank(uint32_t start_addr, uint32_t check_len, uint8_t show_details)
{
    // 参数校验
    if (check_len == 0)
    {
        printf("检查失败：检查长度不能为0\r\n");
        return 2;
    }

    // 地址必须偶对齐（字节地址必须是2的倍数）
    if (start_addr % 2 != 0)
    {
        printf("检查失败：起始地址0x%08X不是偶对齐\r\n", start_addr);
        return 2;
    }

    // 计算半字起始偏移和检查的半字数量
    uint32_t start_halfword = start_addr >> 1;      // 字节地址转半字偏移
    uint32_t halfword_count = (check_len + 1) >> 1; // 字节长度转半字数量（向上取整）

    // 如果是奇数长度，最后一个半字只检查有效字节
    uint8_t is_odd_len = (check_len % 2 != 0);
    uint8_t last_byte_valid = is_odd_len ? (start_addr + check_len - 1) % 2 : 0; // 最后一个有效字节在低8位还是高8位

    uint32_t error_count = 0;
    uint32_t first_error_addr = 0;
    uint16_t first_error_data = 0;
    uint32_t first_error_halfword = 0;

    // 打印检查范围信息
    if (show_details)
    {
        printf("正在检查芯片区域: 0x%08lX ~ 0x%08lX (共%lu字节, %lu半字)\r\n",
               (unsigned long)start_addr,
               (unsigned long)(start_addr + check_len - 1),
               (unsigned long)check_len,
               (unsigned long)halfword_count);
    }

    // 逐半字检查（地址偏移直接+1即可访问下一个半字）
    for (uint32_t i = 0; i < halfword_count; i++)
    {
        uint32_t current_halfword = start_halfword + i; // 半字偏移 +1 即可
        uint16_t data = *(FSMC_NOR_BASE_ADDR + current_halfword);

        // 处理最后一个半字（如果是奇数长度）
        if (is_odd_len && i == halfword_count - 1)
        {
            // 根据最后有效字节的位置检查对应的8位
            if (last_byte_valid == 0) // 最后有效字节在低8位（起始地址为偶，奇数长度）
            {
                if ((data & 0x00FF) != 0xFF) // 只检查低8位
                {
                    error_count++;
                    if (error_count == 1)
                    {
                        first_error_halfword = current_halfword;
                        first_error_addr = (uint32_t)(FSMC_NOR_BASE_ADDR + current_halfword);
                        first_error_data = data;
                    }
                    if (show_details && error_count <= 10)
                    {
                        printf("  错误 #%lu: 地址0x%08lX 数据0x%04X (低字节0x%02X非0xFF)\r\n",
                               (unsigned long)error_count,
                               (unsigned long)(FSMC_NOR_BASE_ADDR + current_halfword),
                               data, data & 0xFF);
                    }
                }
            }
            else // 最后有效字节在高8位（起始地址为奇，但我们的起始地址必须偶对齐，所以这种情况不会发生）
            {
                if ((data >> 8) != 0xFF)
                {
                    error_count++;
                    if (error_count == 1)
                    {
                        first_error_halfword = current_halfword;
                        first_error_addr = (uint32_t)(FSMC_NOR_BASE_ADDR + current_halfword);
                        first_error_data = data;
                    }
                    if (show_details && error_count <= 10)
                    {
                        printf("  错误 #%lu: 地址0x%08lX 数据0x%04X (高字节0x%02X非0xFF)\r\n",
                               (unsigned long)error_count,
                               (unsigned long)(FSMC_NOR_BASE_ADDR + current_halfword),
                               data, data >> 8);
                    }
                }
            }
        }
        else
        {
            // 正常半字，应该为0xFFFF
            if (data != 0xFFFF)
            {
                error_count++;
                if (error_count == 1)
                {
                    first_error_halfword = current_halfword;
                    first_error_addr = (uint32_t)(FSMC_NOR_BASE_ADDR + current_halfword);
                    first_error_data = data;
                }
                if (show_details && error_count <= 10)
                {
                    printf("  错误 #%lu: 地址0x%08lX 数据0x%04X (应为0xFFFF)\r\n",
                           (unsigned long)error_count,
                           (unsigned long)(FSMC_NOR_BASE_ADDR + current_halfword),
                           data);
                }
            }
        }
    }

    // 汇总报告
    if (show_details)
    {
        if (error_count == 0)
        {
            printf("? 检查通过：指定区域全部为0xFFFF\r\n");
        }
        else
        {
            printf("\r\n??  检查发现 %lu 个非空白地址", (unsigned long)error_count);

            if (error_count > 0)
            {
                printf(" (首个错误：地址0x%08lX 数据0x%04X)\r\n",
                       (unsigned long)first_error_addr,
                       first_error_data);

                if (error_count > 10)
                {
                    printf("   (只显示了前10个错误，共%lu个错误)\r\n", (unsigned long)error_count);
                }

                // 显示可能的错误原因
                printf("\r\n可能的原因：\r\n");
                printf("  1. 芯片尚未擦除 - 建议先执行整片擦除（选项2）\r\n");
                printf("  2. 芯片已有有效数据 - 直接写入可能导致数据损坏\r\n");
                printf("  3. 硬件连接问题 - 检查FSMC连接和芯片供电\r\n");
            }
            else
            {
                printf("\r\n");
            }
        }
    }

    return (error_count == 0) ? 0 : 1;
}

/**
 * @brief  快速检查芯片起始区域（0x00000000~0x000000FF）是否空白
 * @param  ask_user: 是否在发现错误时询问用户
 * @retval uint8_t: 0 - 区域空白 | 1 - 区域非空白 | 2 - 用户取消操作
 */
/**
 * @brief  快速检查芯片起始区域（0x00000000~0x000000FF）是否空白
 * @param  ask_user: 是否在发现错误时询问用户
 * @retval uint8_t: 0 - 区域空白 | 1 - 区域非空白 | 2 - 用户取消操作
 */
uint8_t AM29LV320_Check_Start_Area(uint8_t ask_user)
{
    uint8_t check_result = AM29LV320_Check_Blank(0x00000000, 0x100, 1); // 检查256字节

    if (check_result != 0 && ask_user)
    {
        printf("\r\n警告：芯片起始区域不是空白的！\r\n");
        printf("是否强制继续写入？(按 Y 键 或 OK键 继续，其他键取消): ");

        // 在OLED上显示警告
        OLED_Clear();
        printfOled(0, 1, "WARNING!");
        printfOled(1, 1, "Start area");
        printfOled(2, 1, "NOT blank!");
        printfOled(3, 1, "Force continue?");
        printfOled(4, 1, "Press Y or OK");
        printfOled(5, 1, "to continue");
        printfOled(6, 1, "Press others");
        printfOled(7, 1, "to cancel");
        OLED_Refresh();

        // 等待用户确认（10秒超时）
        uint32_t tickstart = HAL_GetTick();
        uint8_t key = 0;

        while ((HAL_GetTick() - tickstart) < 10000) // 10秒超时
        {
            // 检测OK键
            if (KEY_OK_Detect())
            {
                printf("\r\n用户按OK键确认，强制继续写入...\r\n");

                // OLED显示确认
                OLED_Clear();
                printfOled(0, 1, "Confirmed by");
                printfOled(1, 1, "OK key");
                printfOled(2, 1, "Continue...");
                OLED_Refresh();
                HAL_Delay(500);

                return 0; // 返回0表示用户同意继续
            }

            // 检测取消键
            if (KEY_CANCEL_Detect())
            {
                printf("\r\n用户按取消键，写入操作已取消\r\n");

                // OLED显示取消
                OLED_Clear();
                printfOled(0, 1, "Cancelled by");
                printfOled(1, 1, "CANCEL key");
                printfOled(7, 1, "Press any key");
                OLED_Refresh();

                return 2; // 用户取消
            }

            // 检测串口输入（非阻塞）
            uint8_t ch = 0;
            if (HAL_UART_Receive(&huart1, &ch, 1, 0) == HAL_OK)
            {
                // 回显字符
                if (ch >= 0x20 && ch <= 0x7E)
                {
                    HAL_UART_Transmit(&huart1, &ch, 1, HAL_MAX_DELAY);
                }

                if (ch == 'Y' || ch == 'y')
                {
                    printf("\r\n用户输入Y确认，强制继续写入...\r\n");

                    // OLED显示确认
                    OLED_Clear();
                    printfOled(0, 1, "Confirmed by");
                    printfOled(1, 1, "'Y' key");
                    printfOled(2, 1, "Continue...");
                    OLED_Refresh();
                    HAL_Delay(500);

                    return 0; // 返回0表示用户同意继续
                }
                else
                {
                    printf("\r\n用户输入其他字符，写入操作已取消\r\n");

                    // OLED显示取消
                    OLED_Clear();
                    printfOled(0, 1, "Cancelled by");
                    printfOled(1, 1, "other key");
                    printfOled(7, 1, "Press any key");
                    OLED_Refresh();

                    return 2; // 用户取消
                }
            }

            HAL_Delay(20);
        }

        // 超时处理
        printf("\r\n等待超时，写入操作已取消\r\n");

        // OLED显示超时
        OLED_Clear();
        printfOled(0, 1, "Timeout");
        printfOled(1, 1, "Operation");
        printfOled(2, 1, "Cancelled");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        return 2; // 用户取消
    }

    return check_result;
}

uint8_t AM29_Write_Data_From_File(const char *filename)
{
    FRESULT res;
    FIL file;
    FATFS fs;
    const TCHAR *vol = _T("0:");
    char file_path[260] = {0};
    uint8_t write_ret = 0;
    uint32_t file_size = 0;
    uint32_t written_bytes = 0;
    uint32_t AM29_MAX_CAPACITY = 0x000000; // 最大容量(字节)

    uint32_t error_count = 0; // 统计不一致的Word数量

    // 定义变量：存储开始/结束的Unix时间戳，以及日期时间结构体
    uint32_t erase_start_unix = 0;
    uint32_t erase_end_unix = 0;
    uint32_t erase_total_seconds = 0;

    RTC_TimeTypeDef erase_start_time = {0};
    RTC_DateTypeDef erase_start_date = {0};
    RTC_TimeTypeDef erase_end_time = {0};
    RTC_DateTypeDef erase_end_date = {0};

    // 64MB分块相关变量
    const uint32_t BLOCK_64MB = 64 * 1024 * 1024; // 64MB
    uint32_t current_block = 0;                   // 当前块号
    uint32_t total_blocks = 1;                    // 总块数
    uint32_t block_start_addr = 0;                // 当前块的起始地址
    uint32_t bytes_in_block = 0;                  // 当前块已写入字节数
    uint32_t block_written_bytes = 0;             // 当前块内的写入偏移

    // 获取AM29的容量
    uint16_t cap = AM29_Read_CFI_Capacity();
    if (cap == 0)
    {
        printf("获取容量失败\r\n");
        // 显示
        OLED_Clear();
        printfOled(0, 1, "Get size failed.");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        return 99;
    }
    else if (cap > 0x1C)
    {
        // 显示
        OLED_Clear();
        printfOled(0, 1, "Max Support:256M");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        printf("最大只支持S70GL02\r\n");
        return 98;
    }

    AM29_MAX_CAPACITY = AM29_Parse_CFI_Capacity(cap);

    // 计算总块数
    total_blocks = AM29_MAX_CAPACITY / BLOCK_64MB;
    if (total_blocks == 0)
    {
        total_blocks = 1;
    }

    printf("AM29容量: %lu Bytes (%lu MB), 总块数: %u\r\n",
           (unsigned long)AM29_MAX_CAPACITY,
           (unsigned long)(AM29_MAX_CAPACITY / 1024 / 1024),
           total_blocks);

    // 检查芯片起始区域是否为空（0xFFFF）
    uint8_t check_result = AM29LV320_Check_Start_Area(1); // 检查并询问用户
    if (check_result == 2)                                // 用户取消
    {
        f_mount(NULL, vol, 1);
        return 10; // 返回特定错误码表示用户取消
    }
    // 如果 check_result == 1，用户选择了强制继续，继续执行
    // 如果 check_result == 0，区域空白，继续执行

    // 1. 入参校验
    if (filename == NULL || strlen(filename) == 0)
    {
        printf("写入失败：文件名参数为空\r\n");
        // 显示
        OLED_Clear();
        printfOled(0, 1, "Max Support:256M");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        return 1;
    }
    sprintf(file_path, "0:/AM29/%s", filename); // 拼接完整文件路径

    // 2. 挂载TF卡
    res = f_mount(&fs, vol, 1);
    if (res != FR_OK)
    {
        printf("写入失败：TF卡挂载失败，错误码：%d\r\n", res);
        // 显示
        OLED_Clear();
        printfOled(0, 1, "TF NOT present");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
        return 2;
    }

    // 3. 打开BIN文件（只读模式）
    res = f_open(&file, (const TCHAR *)file_path, FA_READ);
    if (res != FR_OK)
    {
        printf("写入失败：打开文件%s失败，错误码：%d\r\n", filename, res);

        // 显示
        OLED_Clear();
        printfOled(0, 1, "Fail to open");
        printfOled(1, 1, "%s", filename);
        printfOled(2, 1, "code: %d", res);
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        f_mount(NULL, vol, 1);
        return 3;
    }

    // 4. 获取文件大小并校验
    file_size = f_size(&file);
    printf("打开文件成功：%s | 大小：%u 字节\r\n", filename, file_size);
    if (file_size > AM29_MAX_CAPACITY)
    {
        printf("写入失败：文件大小(%u字节)超过芯片最大容量(%u字节)\r\n",
               file_size, AM29_MAX_CAPACITY);

        // 显示
        OLED_Clear();
        printfOled(0, 1, "File over size");
        printfOled(1, 1, "Chip size:");
        printfOled(2, 1, "%d KB", AM29_MAX_CAPACITY / 1024);
        printfOled(3, 1, "File size:");
        printfOled(4, 1, "%d KB", file_size / 1024);
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        f_close(&file);
        f_mount(NULL, vol, 1);
        return 4;
    }

    if (file_size == 0)
    {
        printf("写入失败：文件为空\r\n");

        // 显示
        OLED_Clear();
        printfOled(0, 1, "File Size 0Byte");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        f_close(&file);
        f_mount(NULL, vol, 1);
        return 1;
    }

    // 6. 芯片初始化（仅执行一次复位）
    printf("芯片初始化（仅复位一次）...\r\n");
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
    HAL_Delay(100);

    // 7. 获取开始时间
    if (HAL_RTC_GetTime(&hrtc, &erase_start_time, RTC_FORMAT_BIN) != HAL_OK)
    {
        printf("获取写入开始时间失败！\r\n");
        // 显示
        OLED_Clear();
        printfOled(0, 1, "Get RTC Failed");
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
        return 5;
    }
    HAL_RTC_GetDate(&hrtc, &erase_start_date, RTC_FORMAT_BIN);
    erase_start_unix = rtc_to_unix_timestamp(&erase_start_date, &erase_start_time);
    printf("写入开始时间（Unix时间戳）：%lu 秒\r\n", (unsigned long)erase_start_unix);
    printf("写入开始时间（本地）：%04d-%02d-%02d %02d:%02d:%02d\r\n",
           2000 + erase_start_date.Year, erase_start_date.Month, erase_start_date.Date,
           erase_start_time.Hours, erase_start_time.Minutes, erase_start_time.Seconds);

    // 8. 分块读取文件并写入芯片
    printf("开始写入文件到芯片（分64MB块处理）...\r\n");

    uint8_t read_buf[512] = {0}; // 缓冲区，最大512字节用于Buffer Programming
    UINT br = 0;                 // 实际读取字节数

    // 判断是否使用Write Buffer Programming (仅对S70GL02，cap == 0x1C)
    uint8_t use_buffer_program = (cap == 0x1C);
    if (use_buffer_program)
    {
        printf("检测到S70GL02 (256M)芯片，启用Write Buffer Programming模式（每512字节为一个编程单元）\r\n");
    }

    // 循环处理每个64MB块
    for (current_block = 0; current_block < total_blocks; current_block++)
    {
        block_start_addr = current_block * BLOCK_64MB;
        bytes_in_block = 0;
        block_written_bytes = 0;

        printf("\r\n===== 处理第 %lu 块 (地址范围: 0x%08lX - 0x%08lX) =====\r\n",
               (unsigned long)current_block + 1,
               (unsigned long)block_start_addr,
               (unsigned long)(block_start_addr + BLOCK_64MB - 1));

        // 8.1 设置A26~A31对应当前块
        Set_A25_A31(block_start_addr);

        // 8.2 写入当前块的数据
        while (bytes_in_block < BLOCK_64MB && written_bytes < file_size)
        {
            if (use_buffer_program)
            {
                // ===== Write Buffer Programming 模式 (针对S70GL02) =====
                // 计算当前写入地址
                uint32_t block_halfword_offset = bytes_in_block / 2;
                uint32_t current_addr = block_start_addr + bytes_in_block;
                uint32_t addr_in_block = bytes_in_block;

                // 检查是否是512字节对齐边界
                uint32_t alignment_offset = addr_in_block % 512;

                if (alignment_offset != 0)
                {
                    // 这是文件尾部的情况
                    uint32_t bytes_remaining_in_file = file_size - written_bytes;
                    uint32_t bytes_to_next_boundary = 512 - alignment_offset;

                    printf("文件尾部：剩余%lu字节，当前块内偏移%lu字节\r\n",
                           (unsigned long)bytes_remaining_in_file,
                           (unsigned long)alignment_offset);

                    // 计算本次Buffer Programming的总大小（必须是2的倍数）
                    uint32_t buffer_total_size = alignment_offset + bytes_remaining_in_file;
                    // 确保是2的倍数（半字对齐）
                    if (buffer_total_size % 2 != 0)
                    {
                        buffer_total_size++; // 向上取整到偶数
                    }

                    printf("本次Buffer Programming总大小：%lu字节 (%lu个半字)\r\n",
                           (unsigned long)buffer_total_size,
                           (unsigned long)(buffer_total_size / 2));

                    // 创建缓冲区，全部填充0xFF
                    uint8_t temp_buf[512] = {0};
                    memset(temp_buf, 0xFF, buffer_total_size);

                    // 回退文件指针到当前块的起始位置
                    uint32_t block_start_pos_in_file = written_bytes - bytes_in_block;
                    f_lseek(&file, block_start_pos_in_file);

                    // 读取整个块的数据（包括已写部分和剩余部分）
                    UINT br_tail = 0;
                    res = f_read(&file, temp_buf, buffer_total_size, &br_tail);
                    if (res != FR_OK || br_tail != buffer_total_size)
                    {
                        printf("读取块数据失败，错误码：%d\r\n", res);
                        write_ret = 5;
                        break;
                    }

                    // ===== 执行Buffer Programming =====
                    // 步骤2: 解锁序列1 (地址0x555)
                    *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00AA;
                    delaycmd();

                    // 步骤3: 解锁序列2 (地址0x2AA)
                    *(FSMC_NOR_BASE_ADDR + 0x2AA) = 0x0055;
                    delaycmd();

                    // 步骤4: 写Buffer Program命令 (0x0025) 到目标地址的起始半字地址
                    // 这里需要切换到正确的块地址来写入命令
                    *(FSMC_NOR_BASE_ADDR + block_halfword_offset) = 0x0025;
                    delaycmd();

                    // 步骤5: 写入Word Count (需要保持在目标块)
                    uint16_t word_count = (buffer_total_size / 2) - 1; // 半字数-1
                    printf("Word Count: %u (0x%04X)，共%u个半字\r\n",
                           word_count, word_count, word_count + 1);
                    *(FSMC_NOR_BASE_ADDR + block_halfword_offset) = word_count;
                    delaycmd();

                    // 步骤6: 写入Buffer中的数据 (保持在目标块)
                    for (uint32_t i = 0; i < buffer_total_size; i += 2)
                    {
                        uint16_t write_data = ((uint16_t)temp_buf[i] << 8) | temp_buf[i + 1];
                        *(FSMC_NOR_BASE_ADDR + block_halfword_offset + (i / 2)) = write_data;
                        delaycmd();
                    }

                    // 步骤7: 写入Program Buffer to Flash确认命令 (保持在目标块)
                    *(FSMC_NOR_BASE_ADDR + block_halfword_offset) = 0x0029;
                    delaycmd();

                    // 等待编程完成 (保持在目标块)
                    uint32_t timeout = 10000000;
                    while (HAL_GPIO_ReadPin(RY_BY_PORT, RY_BY_PIN) == GPIO_PIN_RESET)
                    {
                        delaycmd();
                        if (--timeout == 0)
                        {
                            printf("Buffer Programming超时：地址0x%08lX\r\n", (unsigned long)current_addr);
                            *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
                            write_ret = 5;
                            break;
                        }
                    }

                    if (write_ret != 0)
                        break;

                    // 更新已写入字节数
                    written_bytes += bytes_remaining_in_file; // 只增加实际数据字节数
                    bytes_in_block += buffer_total_size;      // 整个buffer都已编程
                    block_written_bytes += buffer_total_size;

                    printf("文件尾部处理完成：实际编程%lu字节（含%lu字节填充数据）\r\n",
                           (unsigned long)buffer_total_size,
                           (unsigned long)(buffer_total_size - bytes_remaining_in_file));

                    // 文件已经读完，可以退出循环
                    break;
                }

                // 确定本次Buffer Programming的大小（不超过512字节，不超过文件剩余，不超过当前块剩余）
                uint32_t buffer_size = 512;
                uint32_t bytes_remaining_in_block = BLOCK_64MB - bytes_in_block;
                uint32_t bytes_remaining_in_file = file_size - written_bytes;

                if (bytes_remaining_in_block < buffer_size)
                {
                    buffer_size = bytes_remaining_in_block;
                }
                if (bytes_remaining_in_file < buffer_size)
                {
                    buffer_size = bytes_remaining_in_file;
                }

                // 如果剩余字节数小于512，填充0xFFFF到512字节
                uint32_t bytes_to_program = buffer_size;
                uint32_t bytes_to_pad = 0;

                if (buffer_size < 512)
                {
                    bytes_to_pad = 512 - buffer_size;
                    printf("剩余%lu字节不足512，填充%lu字节0xFFFF\r\n",
                           (unsigned long)buffer_size, (unsigned long)bytes_to_pad);
                    buffer_size = 512; // 总编程大小为512字节
                }

                // 读取实际数据到缓冲区
                memset(read_buf, 0xFF, sizeof(read_buf)); // 先全部填充0xFF

                if (bytes_to_program > 0)
                {
                    // 读取实际数据
                    res = f_read(&file, read_buf, bytes_to_program, &br);
                    if (res != FR_OK || br != bytes_to_program)
                    {
                        printf("读取文件数据失败，错误码：%d\r\n", res);
                        write_ret = 5;
                        break;
                    }
                }

                // 执行Write Buffer Programming命令序列
                *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00AA;
                delaycmd();

                // 步骤2: 解锁序列2
                *(FSMC_NOR_BASE_ADDR + 0x2AA) = 0x0055;
                delaycmd();

                // 步骤3: 写Buffer Program命令 (0x0025) 到目标地址的起始半字地址
                *(FSMC_NOR_BASE_ADDR + block_halfword_offset) = 0x0025;
                delaycmd();

                // 步骤4: 写入Word Count (实际要编程的半字数 - 1)
                uint16_t word_count = (512 / 2) - 1; // 总是256个半字 - 1 = 255 (0x00FF)
                *(FSMC_NOR_BASE_ADDR + block_halfword_offset) = word_count;
                delaycmd();

                // 步骤5: 写入Buffer中的数据 (按半字顺序)
                for (uint32_t i = 0; i < 512; i += 2)
                {
                    uint16_t write_data = ((uint16_t)read_buf[i] << 8) | read_buf[i + 1];
                    *(FSMC_NOR_BASE_ADDR + block_halfword_offset + (i / 2)) = write_data;
                    delaycmd();
                }

                // 步骤6: 写入Program Buffer to Flash确认命令 (0x0029)
                *(FSMC_NOR_BASE_ADDR + block_halfword_offset) = 0x0029;
                delaycmd();

                // 等待编程完成 (通过检查RY/BY#引脚)
                uint32_t timeout = 10000000; // 超时
                while (HAL_GPIO_ReadPin(RY_BY_PORT, RY_BY_PIN) == GPIO_PIN_RESET)
                {
                    delaycmd();
                    if (--timeout == 0)
                    {
                        printf("Buffer Programming超时：地址0x%08lX\r\n", (unsigned long)current_addr);
                        *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
                        write_ret = 5;
                        break;
                    }
                }

                // 读取状态寄存器检查是否成功
                //                uint16_t status = *(FSMC_NOR_BASE_ADDR + block_halfword_offset);
                //                if (status & 0x20) // DQ5 = 1 表示超时错误
                //                {
                //                    printf("Buffer Programming错误：DQ5=1 (超时错误)，状态:0x%04X\r\n", status);
                //                    // 复位芯片
                //                    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
                //                    write_ret = 7;
                //                    break;
                //                }

                // 更新已写入字节数 (只增加实际的有效数据字节数)
                written_bytes += bytes_to_program;
                bytes_in_block += bytes_to_program;
                block_written_bytes += bytes_to_program;

                // 如果填充了0xFFFF，需要跳过这些字节的位置（它们已经被写入，但不算在written_bytes中）
                if (bytes_to_pad > 0)
                {
                    bytes_in_block += bytes_to_pad;
                    block_written_bytes += bytes_to_pad;
                }

                // 进度打印
                if (written_bytes % (0x40000) == 0 || written_bytes == file_size)
                {
                    RTC_TimeTypeDef sTime = {0};
                    RTC_DateTypeDef sDate = {0};
                    uint32_t current_unix = 0;
                    uint32_t elapsed_sec = 0;
                    float total_est_sec = 0.0f;

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
                    elapsed_sec = current_unix - erase_start_unix;

                    // 计算预计总耗时
                    if (written_bytes > 0 && elapsed_sec > 0)
                    {
                        float progress = (float)written_bytes / file_size;
                        total_est_sec = (float)elapsed_sec / progress;
                    }

                    printf("写入进度：%lu/%lu 字节 (%.1f%%) | 块: %u/%u | 已用时：%u 秒 | 预计总耗时：%.1f 秒\r\n",
                           (unsigned long)written_bytes, (unsigned long)file_size,
                           (float)written_bytes / file_size * 100,
                           current_block + 1, total_blocks,
                           elapsed_sec, total_est_sec);

                    // 显示
                    OLED_Clear();
                    printfOled(0, 1, "Writing: %.1f%%", (float)written_bytes / file_size * 100);
                    printfOled(1, 1, "Use: %uS", elapsed_sec);
                    printfOled(2, 1, "Est: %.1fS", total_est_sec);
                    printfOled(3, 1, "Block: %u/%u", current_block + 1, total_blocks);
                    printfOled(4, 1, "Write: %uK", written_bytes / 1024);
                    printfOled(5, 1, "Total: %uK", file_size / 1024);
                    OLED_Refresh();

                    // LED闪耀提示
                    HAL_GPIO_TogglePin(LEDG_PORT, LEDG_PIN);
                }
            }
            else
            {
                // ===== 原有单字节编程模式 (用于非S70GL02芯片) =====
                // 读取2字节大端数据
                uint8_t small_buf[2] = {0};
                br = 0;
                memset(small_buf, 0, sizeof(small_buf));
                res = f_read(&file, small_buf, 2, &br);
                if (res != FR_OK)
                {
                    printf("写入失败：读取文件数据失败，错误码：%d\r\n", res);
                    write_ret = 5;
                    break;
                }
                if (br == 0)
                    break; // 无数据可读，退出循环

                // 大端字节序转16位数据
                uint16_t write_data = ((uint16_t)small_buf[0] << 8) | small_buf[1];

                // 计算当前写入地址（在当前块内的半字偏移）
                uint32_t block_halfword_offset = bytes_in_block / 2;
                uint32_t fsmc_addr = (uint32_t)FSMC_NOR_BASE_ADDR + block_halfword_offset * 2;

                // 执行写入命令序列
                *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00AA;
                delaycmd();
                *(FSMC_NOR_BASE_ADDR + 0x2AA) = 0x0055;
                delaycmd();
                *(FSMC_NOR_BASE_ADDR + 0x555) = 0x00A0;
                delaycmd();

                // 写入目标地址+数据（使用块内半字偏移）
                *(FSMC_NOR_BASE_ADDR + block_halfword_offset) = write_data;
                delaycmd();

                // 等待写入完成
                uint32_t timeout = 1000;
                while (HAL_GPIO_ReadPin(RY_BY_PORT, RY_BY_PIN) == GPIO_PIN_RESET)
                {
                    delaycmd();
                    if (--timeout == 0)
                    {
                        printf("写入超时：地址0x%08X (块内半字偏移0x%06X)\r\n",
                               fsmc_addr, block_halfword_offset);
                        *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
                        write_ret = 5;
                        break;
                    }
                }
                if (write_ret != 0)
                    break;

                // 更新已写入字节数
                written_bytes += br;
                bytes_in_block += br;
                block_written_bytes += br;

                // 进度打印
                if (written_bytes % (0x20000) == 0 || written_bytes == file_size)
                {
                    RTC_TimeTypeDef sTime = {0};
                    RTC_DateTypeDef sDate = {0};
                    uint32_t current_unix = 0;
                    uint32_t elapsed_sec = 0;
                    float total_est_sec = 0.0f;

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
                    elapsed_sec = current_unix - erase_start_unix;

                    // 计算预计总耗时
                    if (written_bytes > 0 && elapsed_sec > 0)
                    {
                        float progress = (float)written_bytes / file_size;
                        total_est_sec = (float)elapsed_sec / progress;
                    }

                    printf("写入进度：%lu/%lu 字节 (%.1f%%) | 块: %u/%u | 已用时：%u 秒 | 预计总耗时：%.1f 秒\r\n",
                           (unsigned long)written_bytes, (unsigned long)file_size,
                           (float)written_bytes / file_size * 100,
                           current_block + 1, total_blocks,
                           elapsed_sec, total_est_sec);

                    // 显示
                    OLED_Clear();
                    printfOled(0, 1, "Writing: %.1f%%", (float)written_bytes / file_size * 100);
                    printfOled(1, 1, "Use: %uS", elapsed_sec);
                    printfOled(2, 1, "Est: %.1fS", total_est_sec);
                    printfOled(3, 1, "Block: %u/%u", current_block + 1, total_blocks);
                    printfOled(4, 1, "Write: %uK", written_bytes / 1024);
                    printfOled(5, 1, "Total: %uK", file_size / 1024);
                    OLED_Refresh();

                    // LED闪耀提示
                    HAL_GPIO_TogglePin(LEDG_PORT, LEDG_PIN);
                }
            }
        }

        if (write_ret != 0)
            break;

        printf("第 %lu 块写入完成，共写入 %lu 字节\r\n",
               (unsigned long)current_block + 1, (unsigned long)block_written_bytes);
    }

    // 9. 复位所有高位地址线和芯片
    Set_A25_A31_All_Zero();
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
    HAL_Delay(1);

    // 10. 全量逐Word对比验证（分64MB块处理，与写入保持一致）
    if (write_ret == 0)
    {
        printf("开始全量逐Word验证（文件VS芯片，分64MB块处理）...\r\n");

        // 重置文件指针到起始位置
        f_lseek(&file, 0);

        uint32_t verify_addr = 0;
        uint32_t total_verify_words = 0;
        uint8_t file_buf[8192] = {0};
        UINT br = 0; // bytes read
#define VERIFY_BUF_SIZE 8192

        // 进度显示相关的变量
        uint32_t last_progress_percent = 0;
        uint32_t last_progress_bytes = 0;
        uint32_t verify_start_time = HAL_GetTick();
        uint32_t last_progress_time = verify_start_time;

        printf("芯片容量: %luMB, FSMC块大小: 64MB, 总块数: %u\r\n",
               (unsigned long)(AM29_MAX_CAPACITY / 1024 / 1024), total_blocks);

        // 按64MB块循环验证
        for (current_block = 0; current_block < total_blocks; current_block++)
        {
            block_start_addr = current_block * BLOCK_64MB;
            uint32_t block_verify_offset = 0; // 当前块内的验证偏移（字节）

            // 计算当前块需要验证的最大字节数（不超过文件剩余大小）
            uint32_t block_verify_limit = BLOCK_64MB;
            if (current_block == total_blocks - 1)
            {
                uint32_t remaining_bytes = file_size - (current_block * BLOCK_64MB);
                if (remaining_bytes < BLOCK_64MB)
                {
                    block_verify_limit = remaining_bytes;
                }
            }

            printf("\r\n===== 验证第 %lu 块 (64MB块, 地址范围: 0x%08lX - 0x%08lX) =====\r\n",
                   (unsigned long)current_block + 1,
                   (unsigned long)block_start_addr,
                   (unsigned long)(block_start_addr + block_verify_limit - 1));

            // 设置A26~A31对应当前块
            Set_A25_A31(block_start_addr);

            // 验证当前块内的数据
            while (block_verify_offset < block_verify_limit && verify_addr < file_size)
            {
                uint32_t read_len = (file_size - verify_addr) > VERIFY_BUF_SIZE ? VERIFY_BUF_SIZE : (file_size - verify_addr);

                // 确保不超出当前块的范围
                if (read_len > (block_verify_limit - block_verify_offset))
                {
                    read_len = block_verify_limit - block_verify_offset;
                }

                FRESULT res = f_read(&file, file_buf, read_len, &br);
                if (res != FR_OK || br != read_len)
                {
                    printf("文件分块读取异常！起始地址0x%08X, 期望读取%lu字节, 实际读取%lu字节\r\n",
                           verify_addr, (unsigned long)read_len, (unsigned long)br);
                    write_ret = 6;
                    break;
                }

                // 逐Word验证当前块内的数据
                for (uint32_t buf_idx = 0; buf_idx < br; buf_idx += 2)
                {
                    uint8_t byte0 = file_buf[buf_idx];
                    uint8_t byte1 = (buf_idx + 1 < br) ? file_buf[buf_idx + 1] : 0x00;
                    uint16_t data_file = ((uint16_t)byte0 << 8) | byte1;

                    // 使用块内的半字偏移读取芯片数据
                    uint32_t block_halfword_offset = (block_verify_offset + buf_idx) / 2;
                    uint16_t data_chip = *(FSMC_NOR_BASE_ADDR + block_halfword_offset);

                    if (data_chip != data_file)
                    {
                        error_count++;
                        if (error_count <= 10 || error_count % 1000 == 0) // 前10个错误详细显示，之后每1000个显示一次
                        {
                            // 计算全局地址用于显示
                            uint32_t global_addr = block_start_addr + block_verify_offset + buf_idx;
                            printf("验证异常：全局地址0x%08X (块内偏移0x%08X) | 文件0x%04X ≠ 芯片0x%04X | 错误计数：%lu\r\n",
                                   global_addr,
                                   block_verify_offset + buf_idx,
                                   data_file, data_chip, (unsigned long)error_count);
                        }
                    }

                    total_verify_words++;
                }

                verify_addr += br;
                block_verify_offset += br;

                // 进度显示
                uint32_t current_time = HAL_GetTick();

                if ((verify_addr % 0x40000 == 0 || verify_addr == file_size))
                {
                    float percent = (float)verify_addr / file_size * 100;
                    uint32_t elapsed_sec = (current_time - verify_start_time) / 1000;

                    // 计算验证速度
                    float speed = 0;
                    if (elapsed_sec > 0)
                    {
                        speed = (float)verify_addr / elapsed_sec / 1024; // KB/s
                    }

                    printf("验证进度：%.1f%% (%lu/%lu 字节) | 64MB块: %u/%u | 错误数：%lu | 速度: %.1f KB/s | 已用时: %u 秒\r\n",
                           percent,
                           (unsigned long)verify_addr,
                           (unsigned long)file_size,
                           current_block + 1, total_blocks,
                           (unsigned long)error_count,
                           speed,
                           elapsed_sec);

                    // OLED显示
                    OLED_Clear();
                    printfOled(0, 1, "Verifing: %.1f%%", percent);
                    printfOled(1, 1, "Block: %u/%u", current_block + 1, total_blocks);

                    // 显示当前块内的进度
                    uint32_t block_percent = (block_verify_offset * 100) / block_verify_limit;
                    printfOled(2, 1, "Block: %u%%", block_percent);

                    // 根据错误数显示不同颜色提示
                    if (error_count > 0)
                    {
                        printfOled(3, 1, "Errors: %lu", (unsigned long)error_count);
                        HAL_GPIO_WritePin(LEDR_PORT, LEDR_PIN, GPIO_PIN_RESET); // 亮红灯表示有错误
                    }
                    else
                    {
                        printfOled(3, 1, "No Errors");
                        HAL_GPIO_WritePin(LEDG_PORT, LEDG_PIN, !HAL_GPIO_ReadPin(LEDG_PORT, LEDG_PIN)); // 绿灯闪烁
                    }

                    printfOled(4, 1, "Time: %us", elapsed_sec);
                    OLED_Refresh();

                    // 更新最后显示的位置和时间
                    last_progress_bytes = verify_addr;
                    last_progress_time = current_time;
                }
            }

            if (write_ret != 0)
                break;

            printf("第 %lu 块 (64MB) 验证完成，本块验证 %lu 字节\r\n",
                   (unsigned long)current_block + 1,
                   (unsigned long)block_verify_offset);
        }

        uint32_t total_elapsed_sec = (HAL_GetTick() - verify_start_time) / 1000;
        printf("\r\n===== 全量验证完成 =====\r\n");
        printf("总验证字节数：%lu | 不一致Word数：%lu | 总用时：%u 秒\r\n",
               (unsigned long)verify_addr, (unsigned long)error_count, total_elapsed_sec);

        if (error_count > 0)
        {
            write_ret = 6;
            printf("验证失败！发现 %lu 个Word数据不一致\r\n", (unsigned long)error_count);

            // OLED显示失败
            OLED_Clear();
            printfOled(0, 1, "Verify Failed!");
            printfOled(1, 1, "Errors: %lu", (unsigned long)error_count);
            printfOled(2, 1, "Time: %us", total_elapsed_sec);
            printfOled(7, 1, "Press any key");
            OLED_Refresh();

            // 亮红灯指示错误
            HAL_GPIO_WritePin(LEDR_PORT, LEDR_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LEDG_PORT, LEDG_PIN, GPIO_PIN_SET);
        }
        else
        {
            printf("全量逐Word校验通过！所有数据完全一致\r\n");

            // OLED显示成功
            OLED_Clear();
            printfOled(0, 1, "Verify Success!");
            printfOled(1, 1, "Time: %us", total_elapsed_sec);
            printfOled(2, 1, "Size: %uKB", file_size / 1024);
            printfOled(7, 1, "Press any key");
            OLED_Refresh();

            // 亮绿灯指示成功
            HAL_GPIO_WritePin(LEDG_PORT, LEDG_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LEDR_PORT, LEDR_PIN, GPIO_PIN_SET);
        }
    }

    // 11. 资源释放 & 复位所有高位地址线和芯片
    f_close(&file);
    f_mount(NULL, vol, 1);

    Set_A25_A31_All_Zero();
    *(FSMC_NOR_BASE_ADDR + 0x0000) = AM29_RESET_CMD;
    HAL_Delay(1);

    // 12. 结果反馈
    if (write_ret == 0)
    {
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
                   2000 + erase_end_date.Year, erase_end_date.Month, erase_end_date.Date,
                   erase_end_time.Hours, erase_end_time.Minutes, erase_end_time.Seconds);

            erase_total_seconds = erase_end_unix - erase_start_unix;
            printf("AM29芯片写入耗时：%u 秒\r\n", erase_total_seconds);
        }

        printf("文件%s写入芯片成功！总写入字节：%u\r\n", filename, written_bytes);

        // 显示
        OLED_Clear();
        printfOled(0, 1, "Write Success");
        printfOled(1, 1, "Use: %uS", erase_total_seconds);
        printfOled(2, 1, "Size: %uKB", written_bytes / 1024);
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        if (erase_total_seconds > 60)
        {
            buz();
        }

        return 0;
    }
    else if (write_ret == 6)
    {
        printf("文件%s写入芯片失败！验证发现 %u 个Word数据不一致\r\n", filename, error_count);

        // 显示
        OLED_Clear();
        printfOled(0, 1, "Verify Failed");
        printfOled(1, 1, "File: %s", filename);
        printfOled(2, 1, "ErrorCount:");
        printfOled(3, 1, "%u", error_count);
        printfOled(7, 1, "Press any key");
        OLED_Refresh();
        if (erase_total_seconds > 60)
        {
            buz();
        }

        return write_ret;
    }
    else
    {
        printf("文件%s写入芯片失败！\r\n", filename);
        // 显示
        OLED_Clear();
        printfOled(0, 1, "Write Failed");
        printfOled(1, 1, "File: %s", filename);
        printfOled(2, 1, "ErrorCode: %u", write_ret);
        printfOled(7, 1, "Press any key");
        OLED_Refresh();

        if (erase_total_seconds > 60)
        {
            buz();
        }

        return write_ret;
    }
}

void buz(void)
{
    if (1 == 1)
    {
        printf("蜂鸣器响3秒跳过\r\n");
        return;
    }

    printf("蜂鸣器响3秒...START\r\n");
    // 4000Hz 蜂鸣器控制 START
    // 周期 = 250us，半周期 = 125us
    const uint32_t half_period_us = 125;    // 125微秒
    const uint32_t beep_duration_ms = 3000; // 3秒
    uint32_t start_time = HAL_GetTick();

    while ((HAL_GetTick() - start_time) < beep_duration_ms)
    {
        // 输出高电平（假设高电平驱动蜂鸣器）
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
        delay_us(half_period_us); // 125us延时

        // 输出低电平
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
        delay_us(half_period_us); // 125us延时
    }
    // 确保蜂鸣器关闭
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);

    printf("蜂鸣器响3秒...END\r\n");
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
    switch (tick_freq)
    {
    case HAL_TICK_FREQ_10HZ:
        g_tick_step_us = 100000; // 100ms/步 = 100000us
        break;
    case HAL_TICK_FREQ_100HZ:
        g_tick_step_us = 10000; // 10ms/步 = 10000us
        break;
    case HAL_TICK_FREQ_1KHZ:
        g_tick_step_us = 1000; // 1ms/步 = 1000us
        break;
    default:
        g_tick_step_us = 1000; // 默认1KHz
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
    if (ns < MIN_DELAY_NS)
    {
        ns = MIN_DELAY_NS;
    }

    /* 2. 计算需要的循环次数（直接使用初始化好的全局参数） */
    uint32_t loop_cnt = ns / (g_ns_per_cycle * CYCLE_PER_LOOP);

    /* 3. 空循环延迟（volatile防止编译器优化空循环） */
    volatile uint32_t i;
    for (i = 0; i < loop_cnt; i++)
    {
        __NOP(); // 空操作指令，稳定指令周期
    }
}

void delaycmd(void)
{
    volatile uint32_t i;
    for (i = 0; i < 10; i++)
    {
        __NOP(); // 空操作指令，稳定指令周期
    }
}

// 遍历AM29目录下所有.bin文件并返回列表（排除包含中文字符的文件名）
AM29_File_Info_t *AM29_List_Bin_Files(uint8_t *file_count)
{
    FRESULT res;
    DIR dir;
    FILINFO fno;
    const TCHAR *vol = _T("0:");
    const TCHAR *path = _T("0:/AM29");
    FATFS fs;

    memset(g_am29_bin_files, 0, sizeof(g_am29_bin_files));
    g_am29_bin_file_count = 0;

    if (file_count == NULL)
    {
        printf("参数错误：file_count指针为空\r\n");
        return NULL;
    }
    *file_count = 0;

    printf("SDIO外设初始化成功，开始挂载TF卡...\r\n");
    res = f_mount(&fs, vol, 1);
    if (res != FR_OK)
    {
        printf("TF卡挂载失败! 错误码: %d\r\n", res);
        printf("错误原因：1=SDIO配置错误 | 4=TF卡损坏/未插好\r\n");
        return NULL;
    }
    printf("TF卡挂载成功！\r\n");

    res = f_opendir(&dir, path);
    if (res != FR_OK)
    {
        printf("打开AM29目录失败! 错误码: %d，尝试创建目录...\r\n", res);

        // 尝试创建AM29目录
        res = f_mkdir(path);
        if (res == FR_OK)
        {
            printf("AM29目录创建成功，重新打开目录...\r\n");

            // 重新打开目录
            res = f_opendir(&dir, path);
            if (res != FR_OK)
            {
                printf("重新打开AM29目录失败! 错误码: %d\r\n", res);
                f_mount(NULL, vol, 1);
                return NULL;
            }
        }
        else
        {
            printf("创建AM29目录失败! 错误码: %d\r\n", res);
            f_mount(NULL, vol, 1);
            return NULL;
        }
    }

    printf("===== AM29目录下的.bin文件列表（排除中文文件名） =====\r\n");
    while (1)
    {
        if (g_am29_bin_file_count >= MAX_BIN_FILE_CNT)
            break;

        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0)
            break;

        if ((fno.fattrib & AM_DIR) == 0)
        {
            // 检查是否为.bin文件
            char *suffix = strrchr(fno.fname, '.');
            if (suffix && (strcmp(suffix, ".bin") == 0 || strcmp(suffix, ".BIN") == 0))
            {
                // 检查文件名是否包含中文字符（ASCII > 0x7F）
                uint8_t has_chinese = 0;
                uint8_t name_len = strlen(fno.fname);

                for (uint8_t i = 0; i < name_len; i++)
                {
                    // 如果字符的ASCII码大于0x7F，说明可能是中文字符的一部分
                    if ((uint8_t)fno.fname[i] > 0x7F)
                    {
                        has_chinese = 1;
                        break;
                    }
                }

                // 只处理不包含中文的文件名
                if (!has_chinese)
                {
                    strncpy(g_am29_bin_files[g_am29_bin_file_count].filename,
                            fno.fname, sizeof(g_am29_bin_files[g_am29_bin_file_count].filename) - 1);
                    g_am29_bin_files[g_am29_bin_file_count].filesize = fno.fsize;
                    printf("文件: %s | 大小: %lu 字节\r\n",
                           g_am29_bin_files[g_am29_bin_file_count].filename,
                           (unsigned long)g_am29_bin_files[g_am29_bin_file_count].filesize);
                    g_am29_bin_file_count++;
                }
                else
                {
                    // 可选：打印被排除的中文文件名（用于调试）
                    printf("排除中文文件名: %s\r\n", fno.fname);
                }
            }
        }
    }

    f_closedir(&dir);
    f_mount(NULL, vol, 1);

    *file_count = g_am29_bin_file_count;
    if (g_am29_bin_file_count == 0)
    {
        printf("AM29目录下未找到有效的.bin文件（无中文文件名）\r\n");
    }
    else
    {
        printf("===== 共找到 %d 个有效的.bin文件（已排除中文文件名） =====\r\n", g_am29_bin_file_count);
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

#ifdef USE_FULL_ASSERT
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
