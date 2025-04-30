/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Mpu_If.h"
#include "WS2812b_If.h"
#include <math.h>
#include "Filter.h"
#include "Mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  EN_MPU6050_XG_TEST = 0U,
  EN_MPU6050_YG_TEST,
  EN_MPU6050_ZG_TEST,
  EN_MPU6050_XA_TEST,
  EN_MPU6050_YA_TEST,
  EN_MPU6050_ZA_TEST,
} t_Mpu6050_SelfTestType;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define U1L_MPU6050_DEVICE_I2C_ADDR  ((uint8_t)0xD0U)
#define F4L_MPU6050_SELF_TEST_RESPONSE_MIN  ((float)-14.0F)
#define F4L_MPU6050_SELF_TEST_RESPONSE_MAX  ((float)14.0F)
#define F4L_M_PI ((float)3.14159265358979323846F)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t f_Mpu6050_SelfTest( void );
static void f_Mpu6050_Init( void );
static void f_Mpu6050_ReadOneByteReg( const uint8_t u1a_AddrReg, uint8_t* const p2u1a_RegDat );
static void f_Mpu6050_ReadTwoBytesReg( uint8_t u1a_StartRegAddr, uint16_t* const p2u2a_Dat );
static void f_Mpu6050_WriteReq( const uint8_t u1a_AddrReg, const uint8_t u1a_RegDat );
static void f_Mpu6050_Power( float f4a_Base, uint8_t u1a_Exponent, float* const p2f4a_Result );
static void f_Mpu6050_CalFt( t_Mpu6050_SelfTestType const ena_TestType, const uint8_t u1a_TestVal, float* const p2f4a_FtVal );
static void f_Mpu6050_JudgeSelfTestRsp( const uint16_t u2a_EnaSelfTestVal, \
                                        const uint16_t u2a_DisSelfTestVal, \
                                        const float* const p2f4a_FtVAl, \
                                        uint8_t* const p2u1a_Result );
static void f_Mpu6050_MeasurePitchRoll( uint16_t* const p2u2a_Pitch, uint16_t* const p2u2a_Roll );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t u1a_DataBuffer[10U];
  static volatile uint16_t u2a_Pitch, u2a_Roll;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //f_Mpu_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // f_WS2812b_Init();
  // f_Mpu6050_Init();
  // f_Mpu6050_SelfTest();
  while (MPU6050_Init(&hi2c1) == 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    // f_Mpu6050_MeasurePitchRoll( &u2a_Pitch, &u2a_Roll );
    MPU6050_Read_All(&hi2c1, &MPU6050);
    HAL_Delay( 500UL );
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LedOnBoard_GPIO_Port, LedOnBoard_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DI_WS2314_GPIO_Port, DI_WS2314_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LedOnBoard_Pin */
  GPIO_InitStruct.Pin = LedOnBoard_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LedOnBoard_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DI_WS2314_Pin */
  GPIO_InitStruct.Pin = DI_WS2314_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DI_WS2314_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void f_Mpu6050_ReadOneByteReg( const uint8_t u1a_AddrReg, uint8_t* const p2u1a_RegDat )
{
  /* Write address register */
  HAL_I2C_Master_Transmit( &hi2c1, U1L_MPU6050_DEVICE_I2C_ADDR, (uint8_t*)&u1a_AddrReg, 1U, 100U );
  HAL_I2C_Master_Receive( &hi2c1, U1L_MPU6050_DEVICE_I2C_ADDR, (uint8_t*)&p2u1a_RegDat[0], 1U, 1000U );

  return;
}

void f_Mpu6050_WriteReq( const uint8_t u1a_AddrReg, const uint8_t u1a_RegDat )
{
  uint8_t u1a_TransferDat[2U];
  uint8_t u1a_RegReadDat;

  /* Prepare data before transfering */
  u1a_TransferDat[0U] = u1a_AddrReg;
  u1a_TransferDat[1U] = u1a_RegDat;

  /* Transfer data via 2ic bus */
  HAL_I2C_Master_Transmit( &hi2c1, U1L_MPU6050_DEVICE_I2C_ADDR, &u1a_TransferDat[0U], 2U, 100U );

  /* Write verify */
  f_Mpu6050_ReadOneByteReg( u1a_AddrReg, &u1a_RegReadDat );
  if ( u1a_RegDat != u1a_RegReadDat )
  {
    while(1){}
  }

  return;
}

void f_Mpu6050_Power( float f4a_Base, uint8_t u1a_Exponent, float* const p2f4a_Result )
{
  *p2f4a_Result = 1.0F;

  for ( uint8_t u1a_Cnt = 0U; u1a_Cnt < u1a_Exponent; u1a_Cnt++)
  {
      *p2f4a_Result *= f4a_Base;  // Multiple base
  }

  return;
}

static void f_Mpu6050_MeasurePitchRoll( uint16_t* const p2u2a_Pitch, uint16_t* const p2u2a_Roll )
{
    uint16_t u2a_AccX;
    uint16_t u2a_AccY;
    uint16_t u2a_AccZ;
    uint8_t u1a_DataStatus;
    float f4a_Pitch;
    float f4a_PitchK;
    float f4a_Roll;

    /* Read Interupt status */
    f_Mpu6050_ReadOneByteReg( 58U, &u1a_DataStatus );
    u1a_DataStatus &= 1U;
    if ( 1U == u1a_DataStatus )
    {
      /* Read Accelermeter */
      /*
      AFS_SEL Full Scale Range LSB Sensitivity
          0       ±2g           16384 LSB/g
      */
      f_Mpu6050_ReadTwoBytesReg( 59U, &u2a_AccX );
      f_Mpu6050_ReadTwoBytesReg( 61U, &u2a_AccY );
      f_Mpu6050_ReadTwoBytesReg( 63U, &u2a_AccZ );

      u2a_AccX /= 16384U;
      u2a_AccY /= 16384U;
      u2a_AccZ /= 14418U;

      /* Calculate pitch and roll */
      // f4a_Roll = atan(u2a_AccY / sqrt(pow(u2a_AccX, 2) + pow(u2a_AccZ, 2))) * 180 / F4L_M_PI;
      f4a_Pitch = atan(-u2a_AccX / sqrt(pow(u2a_AccY, 2) + pow(u2a_AccZ, 2))) * 180 / F4L_M_PI;
      f4a_PitchK= kalman_single(f4a_Pitch, 100, 10);
      *p2u2a_Pitch = (uint16_t)f4a_PitchK;
      // *p2u2a_Roll = (uint16_t)f4a_Roll;
    }

    return;
}

static void f_Mpu6050_CalFt( t_Mpu6050_SelfTestType const ena_TestType, uint8_t u1a_TestVal, float* const p2f4a_FtVal )
{
  if ( 0U == u1a_TestVal )
  {
    *p2f4a_FtVal = 0.0F;
  }
  else
  {
    switch ( ena_TestType )
    {
    case EN_MPU6050_XG_TEST:
    case EN_MPU6050_ZG_TEST:
      u1a_TestVal -= 1U;
      f_Mpu6050_Power( 1.046F, u1a_TestVal, p2f4a_FtVal );
      *p2f4a_FtVal *= 25.0F;
      *p2f4a_FtVal *= 131.0F;
        break;
    default:
      break;
    }

  }

  return;
}

static void f_Mpu6050_ReadTwoBytesReg( uint8_t u1a_StartRegAddr, uint16_t* const p2u2a_Dat )
{
  uint8_t* p2u1a_Dat;

  p2u1a_Dat = (uint8_t*)p2u2a_Dat;

  f_Mpu6050_ReadOneByteReg( u1a_StartRegAddr, &p2u1a_Dat[0U] );
  u1a_StartRegAddr++;
  f_Mpu6050_ReadOneByteReg( u1a_StartRegAddr, &p2u1a_Dat[1U] );

  return;
}

static void f_Mpu6050_JudgeSelfTestRsp( const uint16_t u2a_EnaSelfTestVal, \
                                        const uint16_t u2a_DisSelfTestVal, \
                                        const float* const p2f4a_FtVAl, \
                                        uint8_t* const p2u1a_Result )
{
  float f4a_TestResponse;

  f4a_TestResponse = (float)( u2a_EnaSelfTestVal - u2a_DisSelfTestVal );
  f4a_TestResponse -= *p2f4a_FtVAl;
  f4a_TestResponse /= *p2f4a_FtVAl;

  if ( ( F4L_MPU6050_SELF_TEST_RESPONSE_MIN <= f4a_TestResponse )
    || ( F4L_MPU6050_SELF_TEST_RESPONSE_MAX >= f4a_TestResponse ) )
  {
    *p2u1a_Result = HAL_OK;
  }
  else
  {
    *p2u1a_Result = HAL_ERROR;
  }

  return;
}

static uint8_t f_Mpu6050_SelfTest( void )
{
  uint8_t u1a_Result;
  uint8_t u1a_RegDat[4U];
  uint8_t u1a_TestVal;
  uint8_t u1a_YgTest;
  uint8_t u1a_ZgTest;
  uint16_t u2a_ActValEnSelfTest;
  uint16_t u2a_ActValDisSelfTest;
  uint8_t u1a_Exponent;
  float f4a_FTVal;
  static volatile float f4a_SelfTestRsp;

  /*
  Power manager 1
    CLKSEL[2:0] = 0 Internal 8MHz oscillator
  */
  f_Mpu6050_WriteReq( 107U, 0x00U );

  /*
  Register 27 – Gyroscope Configuration
  When performing self test for the gyroscope, the full-scale range should be set to ±250dps.
  */
 /* Enable self test XG, set full-scale ±250dps */
  f_Mpu6050_WriteReq( 27U, 0x80U );

  /* Read Gyroscope self test */
  f_Mpu6050_ReadOneByteReg( 13U, &u1a_RegDat[0U] );
  f_Mpu6050_ReadOneByteReg( 14U, &u1a_RegDat[1U] );
  f_Mpu6050_ReadOneByteReg( 15U, &u1a_RegDat[2U] );
  f_Mpu6050_ReadOneByteReg( 16U, &u1a_RegDat[3U] );

  /* Extract Gyroscope data */
  u1a_TestVal = (u1a_RegDat[0U] & 0x1FU);
  u1a_YgTest = (u1a_RegDat[1U] & 0x1FU);
  u1a_ZgTest = (u1a_RegDat[2U] & 0x1FU);

  /* Calculate FT */
  f4a_FTVal = 0.0F;
  f_Mpu6050_CalFt( EN_MPU6050_XG_TEST, u1a_TestVal, &f4a_FTVal );
  /* FT value can not zero */
  if ( 0.0F == f4a_FTVal )
  {
    u1a_Result = HAL_ERROR;
  }

  /* Get actual value when enable self test */
  /* X gyroscope */
  f_Mpu6050_ReadTwoBytesReg( 67U, &u2a_ActValEnSelfTest );

  /* Get actual value when disable self test */
  /*
  Register 27 – Gyroscope Configuration
  When performing self test for the gyroscope, the full-scale range should be set to ±250dps.
  */
 /* Disable self test XG, set full-scale ±250dps */
  f_Mpu6050_WriteReq( 27U, 0x00U );

  /* Get actual value when disable self test */
  /* X gyroscope */
  f_Mpu6050_ReadTwoBytesReg( 67U, &u2a_ActValDisSelfTest );

  /* Calculate Selt test reponse value */
  /* X gyroscope */
  f_Mpu6050_JudgeSelfTestRsp( u2a_ActValEnSelfTest, u2a_ActValDisSelfTest, &f4a_FTVal, &u1a_Result );

  return u1a_Result;
}

void f_Mpu6050_Init( void )
{
    /*
    Power manager 1
      CLKSEL[2:0] = 0 Internal 8MHz oscillator
    */
    f_Mpu6050_WriteReq( 107U, 0U );

    /*
    Register 26 – Configuration
    DLPF_CFG[2:0] = 5
    */
    /*
    +------------------+-------------------+-------------------+-------------------+
    |  DLPF_CFG Value  | Accelerometer     | Gyroscope         | Gyro FS           |
    |                  | Bandwidth (Hz)    | Bandwidth (Hz)    | (kHz)             |
    |                  | Delay (ms)        | Delay (ms)        |                   |
    +------------------+-------------------+-------------------+-------------------+
    |       0          | 260 Hz / 0 ms     | 256 Hz / 0.98 ms  | 8 kHz             |
    |       1          | 184 Hz / 2 ms     | 188 Hz / 1.9 ms   | 1 kHz             |
    |       2          | 94 Hz / 3 ms      | 98 Hz / 2.8 ms    | 1 kHz             |
    |       3          | 44 Hz / 4.9 ms    | 42 Hz / 4.8 ms    | 1 kHz             |
    |       4          | 21 Hz / 8.5 ms    | 20 Hz / 8.3 ms    | 1 kHz             |
    |       5          | 10 Hz / 13.8 ms   | 10 Hz / 13.4 ms   | 1 kHz             |
    |       6          | 5 Hz / 19 ms      | 5 Hz / 18.6 ms    | 1 kHz             |
    |       7          | RESERVED          | RESERVED          | 8 kHz             |
    +------------------+-------------------+-------------------+-------------------+
    */
    f_Mpu6050_WriteReq( 26U, 5U );

    /*
    DLPF_CFG = 5 -> Gyroscope Output Rate = 1kHz
    Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) = 1kHz / ( 1 + 0 ) = 1kHz -> 1ms
    */
    f_Mpu6050_WriteReq( 25U, 0U );

    /*
    Register 27 – Gyroscope Configuration
    FS_SEL Full Scale Range
      0    ± 250 °/s
    */
    f_Mpu6050_WriteReq( 27U, 0U );

    /*
    Register 28 – Accelerometer Configuration
      AFS_SEL Full Scale Range
          0     ± 2g
    */
    f_Mpu6050_WriteReq( 28U, 0U );


    /*
    Register 56 – Interrupt Enable
      DATA_RDY_EN = 1
    */
    f_Mpu6050_WriteReq( 56U, 1U );

    return;
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
