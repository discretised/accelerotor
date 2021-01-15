/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "MPU6000.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch3_up;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t SBUSBuffer[25];
uint16_t SBUSChannelValues[18];
//channel 0: roll
//channel 1: pitch
//channel 2: throttle
//channel 3: yaw

uint32_t DSHOT_buffer[4][47] = {0};
//motor 1: B0 (TIM2 channel 1)
//motor 2:
//motor 3:
//motor 4:

int16_t gyrox, gyroy, gyroz;
float ex, ex1, ex2, ux, delta_ux;
float ey, ey1, ey2, uy, delta_uy;
float ez, ez1, ez2, uz, delta_uz;

float kpx = 1;
float kix = 0;
float kdx = 0;
float k1x, k2x, k3x;

float kpy = 1;
float kiy = 0;
float kdy = 0;
float k1y, k2y, k3y;

float kpz = 1;
float kiz = 0.1;
float kdz = 0.5;
float k1z, k2z, k3z;

int throttle_headroom;
float throttle_headroom_percentage = 0.5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint16_t generate_DSHOT_bitstream(uint16_t val);
void set_DSHOT_PWM(uint8_t, uint16_t DSHOT_bitstream);
void arm_motors(void);
void spin_motor(uint8_t motor, uint16_t throttle);
void PID_loop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_IT(&huart2, SBUSBuffer, sizeof(SBUSBuffer));

    SBUSChannelValues[0]  = (uint16_t) ((SBUSBuffer[1]    |SBUSBuffer[2] <<8)                      & 0x07FF);
    SBUSChannelValues[1]  = (uint16_t) ((SBUSBuffer[2]>>3 |SBUSBuffer[3] <<5)                      & 0x07FF);
    SBUSChannelValues[2]  = (uint16_t) ((SBUSBuffer[3]>>6 |SBUSBuffer[4] <<2 |SBUSBuffer[5]<<10)   & 0x07FF);
    SBUSChannelValues[3]  = (uint16_t) ((SBUSBuffer[5]>>1 |SBUSBuffer[6] <<7)                      & 0x07FF);
    SBUSChannelValues[4]  = (uint16_t) ((SBUSBuffer[6]>>4 |SBUSBuffer[7] <<4)                      & 0x07FF);
    SBUSChannelValues[5]  = (uint16_t) ((SBUSBuffer[7]>>7 |SBUSBuffer[8] <<1 |SBUSBuffer[9]<<9)    & 0x07FF);
    SBUSChannelValues[6]  = (uint16_t) ((SBUSBuffer[9]>>2 |SBUSBuffer[10] <<6)                     & 0x07FF);
    SBUSChannelValues[7]  = (uint16_t) ((SBUSBuffer[10]>>5|SBUSBuffer[11] <<3)                     & 0x07FF);
    SBUSChannelValues[8]  = (uint16_t) ((SBUSBuffer[12]   |SBUSBuffer[13] <<8)                     & 0x07FF);
    SBUSChannelValues[9]  = (uint16_t) ((SBUSBuffer[13]>>3|SBUSBuffer[14] <<5)                     & 0x07FF);
    SBUSChannelValues[10] = (uint16_t) ((SBUSBuffer[14]>>6|SBUSBuffer[15] <<2 |SBUSBuffer[16]<<10) & 0x07FF);
    SBUSChannelValues[11] = (uint16_t) ((SBUSBuffer[16]>>1|SBUSBuffer[17] <<7)                     & 0x07FF);
    SBUSChannelValues[12] = (uint16_t) ((SBUSBuffer[17]>>4|SBUSBuffer[18] <<4)                     & 0x07FF);
    SBUSChannelValues[13] = (uint16_t) ((SBUSBuffer[18]>>7|SBUSBuffer[19] <<1 |SBUSBuffer[20]<<9)  & 0x07FF);
    SBUSChannelValues[14] = (uint16_t) ((SBUSBuffer[20]>>2|SBUSBuffer[21] <<6)                     & 0x07FF);
    SBUSChannelValues[15] = (uint16_t) ((SBUSBuffer[21]>>5|SBUSBuffer[22] <<3)                     & 0x07FF);
    for(int i = 0; i<25; i++)
    {
        SBUSBuffer[i] = 0;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    MPU6000_deselect();
    if(MPU6000_gyro_read_flag)
    {
        for(int i = 0; i<3; i++)
        {
        }
    }
    MPU6000_busy = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    k1x = kpx + kix + kdx;
    k2x = -kpx -2*kdx;
    k3x = kdx;

    k1y = kpy + kiy + kdy;
    k2y = -kpy -2*kdy;
    k3y = kdy;

    k1z = kpz + kiz + kdz;
    k2z = -kpz -2*kdz;
    k3z = kdz;
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
    MX_DMA_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_USART2_UART_Init();
    MX_USB_DEVICE_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */
    arm_motors();
    HAL_UART_Receive_IT(&huart2, SBUSBuffer, sizeof(SBUSBuffer));

    //initialise the MPU6000 IMU
    if(MPU6000_init(0) != 0)
        Error_Handler();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while(1)
    {
        MPU6000_read_gyro();
        PID_loop();
        HAL_Delay(1);
        while(SBUSChannelValues[1] < 1600){
            spin_motor(1, 0);
            spin_motor(2, 0);
            spin_motor(3, 0);
            spin_motor(4, 0);
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

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
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 72;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 3;
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
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 120;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 1;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 120;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);

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
    huart2.Init.BaudRate = 100000;
    huart2.Init.WordLength = UART_WORDLENGTH_9B;
    huart2.Init.StopBits = UART_STOPBITS_2;
    huart2.Init.Parity = UART_PARITY_EVEN;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    /* DMA1_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    /* DMA1_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    /* DMA1_Stream5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    /* DMA2_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /*Configure GPIO pin : PA4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint16_t generate_DSHOT_bitstream(uint16_t val)
{
    uint16_t DSHOT_bitstream = 0;

    if(val > 2047)
        val = 0;

    DSHOT_bitstream = val << 1;
    DSHOT_bitstream &= ~(1<<0);
    int csum = 0;
    int csum_data = DSHOT_bitstream;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    csum &= 0xf;
    // append checksum
    DSHOT_bitstream = (DSHOT_bitstream << 4) | csum;

    return DSHOT_bitstream;
}

void set_DSHOT_PWM(uint8_t motor, uint16_t DSHOT_bitstream)
{
    //HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);

    for(int i = 0; i<16; i++)
    {
        if((DSHOT_bitstream) & (1<<(15-i)))
            DSHOT_buffer[motor-1][i] = 90;
        else
            DSHOT_buffer[motor-1][i] = 45;
    }

    switch(motor)
    {
        case 1:
            if (HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1, DSHOT_buffer[0], 47) != HAL_OK)
                Error_Handler();
            break;
        case 2:
            if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, DSHOT_buffer[1], 47) != HAL_OK)
                Error_Handler();
            break;
        case 3:
            if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, DSHOT_buffer[2], 47) != HAL_OK)
                Error_Handler();
            break;
        case 4:
            if (HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, DSHOT_buffer[3], 47) != HAL_OK)
                Error_Handler();
            break;
    }

}

void spin_motor(uint8_t motor, uint16_t throttle)
{
    uint16_t true_throttle = throttle + 48;
    uint16_t DSHOT_bitstream = generate_DSHOT_bitstream(true_throttle);
    set_DSHOT_PWM(motor, DSHOT_bitstream);
}

void arm_motors(void)
{
    for(int i = 1; i<= 4; i++)
    {
        set_DSHOT_PWM(i, 0);
        HAL_Delay(1);
    }
    HAL_Delay(1000);
    for(int i = 1; i<= 4; i++)
    {
        set_DSHOT_PWM(i, 0b0000011000000110);
        HAL_Delay(1);
    }
    HAL_Delay(2000);
}

void MPU6000_select(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void MPU6000_deselect(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void MPU6000_start_transfer(uint8_t TxData[], uint8_t length)
{
    //transfers the first [length] bytes of TxData
    //responses are stored in MPU6000_rx_buffer

    //wait until existing SPI communications are complete, then set the busy flag and pull CS low
    while(MPU6000_busy);
    MPU6000_busy = 1;
    MPU6000_select();

    //begin transfer
    if(HAL_SPI_TransmitReceive_DMA(&hspi1, TxData, MPU6000_rx_buffer, length) != HAL_OK)
        Error_Handler();

    //clear MPU6000_tx_buffer in preperation for the next transfer
    for(int i = 0; i<10; i++)
    {
        MPU6000_tx_buffer[i] = 0x00;
    }
}

uint8_t MPU6000_init(uint8_t sample_rate_div)
{
    //Disable i2c interface
    MPU6000_tx_buffer[0] = MPUREG_USER_CTRL;
    MPU6000_tx_buffer[1] = BIT_I2C_IF_DIS;
    MPU6000_start_transfer(MPU6000_tx_buffer, 2);
    while(MPU6000_busy);

    //check the device is responding before proceeding
    MPU6000_tx_buffer[0] = MPUREG_WHOAMI|READ_FLAG;
    MPU6000_tx_buffer[1] = 0x00;
    MPU6000_start_transfer(MPU6000_tx_buffer, 2);
    while(MPU6000_busy);
    if(MPU6000_rx_buffer[1] != 104)
        return(0xEE);

    //reset the device
    MPU6000_tx_buffer[0] = MPUREG_PWR_MGMT_1;
    MPU6000_tx_buffer[1] = BIT_H_RESET;
    MPU6000_start_transfer(MPU6000_tx_buffer, 2);
    while(MPU6000_busy);
    HAL_Delay(100);

    //reset signal path as described in register descriptions, page 41
    MPU6000_tx_buffer[0] = MPUREG_SIGNAL_PATH_RESET;
    MPU6000_tx_buffer[1] = BITS_SIGNAL_PATH_RESET;
    MPU6000_start_transfer(MPU6000_tx_buffer, 2);
    while(MPU6000_busy);
    HAL_Delay(100);

    //disable i2c again after reset
    MPU6000_tx_buffer[0] = MPUREG_USER_CTRL;
    MPU6000_tx_buffer[1] = BIT_I2C_IF_DIS;
    MPU6000_start_transfer(MPU6000_tx_buffer, 2);
    while(MPU6000_busy);

    //use GYRO Z as the clock instead of internal RC oscillator for improved stability
    //important to do this before setting any other registers, otherwise they appear to reset. No mention of this in the
    //datasheet or register descriptions though...
    MPU6000_tx_buffer[0] = MPUREG_PWR_MGMT_1;
    MPU6000_tx_buffer[1] = MPU_CLK_SEL_PLLGYROZ;
    MPU6000_start_transfer(MPU6000_tx_buffer, 2);
    while(MPU6000_busy);

    //set gyro sample rate divider. Formula is Sample Rate = Gyroscope Output Rate / (1 + sample_rate_div)
    //gyro output rate defaults to 8kHz, or 1kHz when enabling the digital low pass filter.
    if(sample_rate_div)
    {
        MPU6000_tx_buffer[0] = MPUREG_SMPLRT_DIV;
        MPU6000_tx_buffer[1] = sample_rate_div;
        MPU6000_start_transfer(MPU6000_tx_buffer, 2);
        while (MPU6000_busy);
    }
}

uint8_t MPU6000_read_gyro(void)
{
    //for a 250deg/sec range, 1 LSB corresponds to 0.43737261522 rad/s

    MPU6000_gyro_read_flag = 1;

    MPU6000_tx_buffer[0] = MPUREG_GYRO_XOUT_H | READ_FLAG;
    MPU6000_tx_buffer[1] = 0x00;
    MPU6000_tx_buffer[2] = 0x00;
    MPU6000_tx_buffer[3] = MPUREG_GYRO_YOUT_H | READ_FLAG;
    MPU6000_tx_buffer[4] = 0x00;
    MPU6000_tx_buffer[5] = MPUREG_GYRO_ZOUT_H | READ_FLAG;
    MPU6000_tx_buffer[6] = 0x00;

    MPU6000_start_transfer(MPU6000_tx_buffer, 6);
}

uint8_t MPU6000_gyro_selftest(uint8_t channel)
{
    //read gyro without self test enabled

    //enable self test

    //read gyro with self test enable

    //STR = with self test - without self test

    //read XG_TEST, YG_TEST, ZG_TEST

    //change = (STR-FT)/FT * 100
}

void PID_loop(void)
{
    int throttle_base = SBUSChannelValues[2];
    if(throttle_base < 200)
        throttle_base = 200;
    else if(throttle_base > 1800)
        throttle_base = 1800;

    float throttle_percentage = (throttle_base - 200)/1600;

    throttle_headroom = throttle_headroom_percentage * throttle_base;

    //read gyro
    gyrox = MPU6000_rx_buffer[1] << 8 | MPU6000_rx_buffer[2];
    gyrox = gyrox;
    gyroy = MPU6000_rx_buffer[3] << 8 | MPU6000_rx_buffer[4];
    gyroy = -gyroy;
    gyroz = MPU6000_rx_buffer[5] << 8 | MPU6000_rx_buffer[6];

    //x axis
    ex2 = ex1;
    ex1 = ex;
    ex = 0 - gyrox;
    delta_ux = k1x*ex + k2x*ex1 + k3x*ex2;
    ux = ux + delta_ux;
    if(ux < -32768)
        ux = -32768;
    else if(ux > 32767)
        ux = 32767;

    //y axis
    ey2 = ey1;
    ey1 = ey;
    ey = 0 - gyroy;
    delta_uy = k1y*ey + k2y*ey1 + k3y*ey2;
    uy = uy + delta_uy;

    if(uy < -32768)
        uy = -32768;
    else if(uy > 32767)
        uy= 32767;

    spin_motor(1, throttle_base - ((ux/32768) * throttle_headroom) - ((uy/32768) * throttle_headroom)) ;
    spin_motor(2, throttle_base + ((ux/32768) * throttle_headroom) - ((uy/32768) * throttle_headroom));
    spin_motor(3, throttle_base - ((ux/32768) * throttle_headroom) + ((uy/32768) * throttle_headroom));
    spin_motor(4, throttle_base + ((ux/32768) * throttle_headroom) + ((uy/32768) * throttle_headroom));
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
