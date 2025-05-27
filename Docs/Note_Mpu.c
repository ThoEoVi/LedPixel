void f_Mpu6050_ReadOneByteReg( const uint8_t u1a_AddrReg, uint8_t* const p2u1a_RegDat )
{
  /* Write address register */
  // HAL_I2C_Master_Transmit( &hi2c1, U1L_MPU6050_DEVICE_I2C_ADDR, (uint8_t*)&u1a_AddrReg, 1U, 100U );
  // HAL_I2C_Master_Receive( &hi2c1, U1L_MPU6050_DEVICE_I2C_ADDR, (uint8_t*)&p2u1a_RegDat[0], 1U, 1000U );

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
  // HAL_I2C_Master_Transmit( &hi2c1, U1L_MPU6050_DEVICE_I2C_ADDR, &u1a_TransferDat[0U], 2U, 100U );

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
      // f4a_PitchK= kalman_single(f4a_Pitch, 100, 10);
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

#define U1L_MPU6050_DEVICE_I2C_ADDR  ((uint8_t)0xD0U)
#define F4L_MPU6050_SELF_TEST_RESPONSE_MIN  ((float)-14.0F)
#define F4L_MPU6050_SELF_TEST_RESPONSE_MAX  ((float)14.0F)
#define F4L_M_PI ((float)3.14159265358979323846F)

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