#ifndef WS2812B_H
#define WS2812B_H

#include "WS2812b_If.h"
#include "main.h"
#include <math.h>


// Số LED trong ma trận
#define U1L_LED_ROWS                  ((uint8_t)4U)
#define U1L_LED_COLS                  ((uint8_t)4U)
#define U1L_LED_COLOR_NUM             ((uint8_t)3U)
#define LED_BRIGHTNESS                ((uint8_t)33U)
#define LED_BRIGHTNESS_EFFECT1        ((uint8_t)20U)
#define LED_LED_NUM_EFFECT1           ((uint8_t)6U) /* Bright 6 leds */
#define LED_BRIGHTNESS_EFFECT2        ((uint8_t)200U)

static uint8_t u1l_WS2812B_ColorMpu[4U][4U][3u];

static void f_WS2812B_Delay( const uint32_t u4a_CntThreshold );
static void f_WS2812B_SendBitOne( void );
static void f_WS2812B_SendBitZero( void );
static void f_WS2812B_SendBitZeroLatest( void );
static void f_WS2812B_SendByte( const uint8_t u1a_Byte );
static void f_WS2812B_SendColor( const uint8_t u1a_Red, const uint8_t u1a_Green, const uint8_t u1a_Blue );
static void f_WS2812B_SendByteArray( uint8_t* const pu1a_ByteArray, const uint32_t u4a_ByteCount );
static void f_WS2812B_SwapCol( const uint8_t u1a_ColId );
static uint8_t f_WS2812b_MapLedIndex( const float f4a_Angle, const float f4a_MaxAngle, \
                                      const float f4a_MinAngle, const uint8_t u1a_LedNum );
static void f_WS2812b_ClearLedValue( void );


void f_WS2812b_Init( void )
{
  /* Set low level more than 50us */
  HAL_GPIO_WritePin( DI_WS2314_GPIO_Port, DI_WS2314_Pin, GPIO_PIN_RESET );
  f_WS2812B_Delay( 2000U );

  f_WS2812b_ClearLedValue();

  return;
}

void f_WS2812b_LedPixelControlMode( float f4a_X, float f4a_Y, uint8_t* const p2u1a_Mode )
{
  static uint8_t u1a_RowPrev = 0xFFU;
  static uint8_t u1a_ColPrev = 0xFFU;
  static uint32_t u4a_Count = 0U;
  static uint8_t u1a_Row;
  static uint8_t u1a_Col;
  uint8_t u1a_TempClolorPixel[3];

  /* Clear all led value */
  f_WS2812b_ClearLedValue();

  /* Convert angle range to led index */
  u1a_Row = f_WS2812b_MapLedIndex( f4a_X, 30.0F, 0.0F, 4U );
  u1a_Col = f_WS2812b_MapLedIndex( f4a_Y, 175.0F, 100.0F, 4U );

  u1l_WS2812B_ColorMpu[u1a_Row][u1a_Col][0] = 0x01U; /* Green */
  u1l_WS2812B_ColorMpu[u1a_Row][u1a_Col][1] = 0x00U; /* Red */
  u1l_WS2812B_ColorMpu[u1a_Row][u1a_Col][2] = 0x01U; /* Blue */

  /* Replace Col 1 and Col 3 */
  f_WS2812B_SwapCol( 1U );
  f_WS2812B_SwapCol( 3U );

  f_WS2812B_SendByteArray( (uint8_t* const)&u1l_WS2812B_ColorMpu[0U], (uint32_t)(4U * 4U * 3U) );

 /* Update new column and new row */
 if ( ( u1a_RowPrev != u1a_Row )
 && ( u1a_ColPrev != u1a_Col ) )
 {
   u1a_RowPrev = u1a_Row;
   u1a_ColPrev = u1a_Col;
   u4a_Count = 0U;
 }
/* Column and Row is kept */
else
{
 u4a_Count++;
 /* Confirm mode */
 if ( u4a_Count >= 1000U )
 {
   /* Clear count */
   u4a_Count = 0U;

   /* Select mode */
   if ( ( 0U == u1a_Row )
     && ( 3U == u1a_Col ) )
   {
     *p2u1a_Mode = 1U;
   }
   else if ( ( 1U == u1a_Row )
          && ( 3U == u1a_Col ) )
   {
     *p2u1a_Mode = 2U;
   }
   else if ( ( 2U == u1a_Row )
          && ( 3U == u1a_Col ) )
   {
     *p2u1a_Mode = 3U;
   }
   else
   {}
 }
}

  return;
}

void f_WS2812b_LedEffect1( void )
{
    uint8_t u1a_Row;
    uint8_t u1a_Col;
    uint8_t u1a_LedCount;

    f_WS2812b_ClearLedValue();

    for ( u1a_LedCount = 0U; u1a_LedCount < LED_LED_NUM_EFFECT1; u1a_LedCount++ )
    {
      /* Cal row, col index */
      u1a_Row = rand() % U1L_LED_ROWS;
      u1a_Col = rand() % U1L_LED_COLS;

      u1l_WS2812B_ColorMpu[u1a_Col][u1a_Row][0U] = rand() % LED_BRIGHTNESS_EFFECT1;
      u1l_WS2812B_ColorMpu[u1a_Col][u1a_Row][1U] = rand() % LED_BRIGHTNESS_EFFECT1;
      u1l_WS2812B_ColorMpu[u1a_Col][u1a_Row][2U] = rand() % LED_BRIGHTNESS_EFFECT1;
    }

    f_WS2812B_SendByteArray( (uint8_t* const)&u1l_WS2812B_ColorMpu[0U], (uint32_t)(4U * 4U * 3U) );
}

static void f_WS2812b_IncLedCnt3( uint8_t* const p2u1a_FstLed, uint8_t* const p2u1a_SndLed, uint8_t* const p2u1a_ThirdLed )
{
  if( *p2u1a_FstLed < LED_BRIGHTNESS )
  {
    (*p2u1a_FstLed)++;
  }
  else
  {
    if( *p2u1a_SndLed < LED_BRIGHTNESS )
    {
      (*p2u1a_SndLed)++;
    }
    else
    {
      if( *p2u1a_ThirdLed < LED_BRIGHTNESS )
      {
        (*p2u1a_ThirdLed)++;
      }
    }
  }
  return;
}

static void f_WS2812b_DecLedCnt3( uint8_t* const p2u1a_FstLed, uint8_t* const p2u1a_SndLed, uint8_t* const p2u1a_ThirdLed )
{
  if( *p2u1a_FstLed > 0U )
  {
    (*p2u1a_FstLed)--;
  }
  else
  {
    if( *p2u1a_SndLed > 0U )
    {
      (*p2u1a_SndLed)--;
    }
    else
    {
      if( *p2u1a_ThirdLed > 0U )
      {
        (*p2u1a_ThirdLed)--;
      }
    }
  }
  return;
}

void f_WS2812b_LedEffect3( void )
{
  static uint8_t u1a_LedR = 0U;
  static uint8_t u1a_LedG = 0U;
  static uint8_t u1a_LedB = 0U;
  static uint8_t u1a_CountMode = 0U;
  uint8_t u1a_LedRow;
  uint8_t u1a_LedCol;

  /* Switch mode */
  switch ( u1a_CountMode )
  {
    case 0U:
    f_WS2812b_IncLedCnt3( &u1a_LedR, &u1a_LedG, &u1a_LedB );
    if ( LED_BRIGHTNESS == u1a_LedB )
    {
      u1a_CountMode++;
    }
    break;

    case 1U:
    f_WS2812b_DecLedCnt3( &u1a_LedR, &u1a_LedG, &u1a_LedB );
    if ( 0U == u1a_LedB )
    {
      u1a_CountMode++;
    }
    break;

    case 2U:
    f_WS2812b_IncLedCnt3( &u1a_LedG, &u1a_LedR, &u1a_LedB );
    if ( LED_BRIGHTNESS == u1a_LedB )
    {
      u1a_CountMode++;
    }
    break;

    case 3U:
    f_WS2812b_DecLedCnt3( &u1a_LedG,  &u1a_LedR, &u1a_LedB );
    if ( 0U == u1a_LedB )
    {
      u1a_CountMode++;
    }
    break;

    case 4U:
    f_WS2812b_IncLedCnt3( &u1a_LedR, &u1a_LedB, &u1a_LedG );
    if ( LED_BRIGHTNESS == u1a_LedG )
    {
      u1a_CountMode++;
    }
    break;

    case 5U:
    f_WS2812b_DecLedCnt3( &u1a_LedR, &u1a_LedB, &u1a_LedG );
    if ( 0U == u1a_LedG )
    {
      u1a_CountMode++;
    }
    break;

    case 6U:
    f_WS2812b_IncLedCnt3( &u1a_LedB, &u1a_LedR, &u1a_LedG );
    if ( LED_BRIGHTNESS == u1a_LedG )
    {
      u1a_CountMode++;
    }
    break;

    case 7U:
    f_WS2812b_DecLedCnt3( &u1a_LedB, &u1a_LedR, &u1a_LedG );
    if ( 0U == u1a_LedG )
    {
      u1a_CountMode++;
    }
    break;


    case 8U:
    f_WS2812b_IncLedCnt3( &u1a_LedB, &u1a_LedG, &u1a_LedR );
    if ( LED_BRIGHTNESS == u1a_LedR )
    {
      u1a_CountMode++;
    }
    break;

    case 9U:
    f_WS2812b_DecLedCnt3( &u1a_LedB, &u1a_LedG, &u1a_LedR );
    if ( 0U == u1a_LedR )
    {
      u1a_CountMode++;
    }
    break;

    case 10U:
    f_WS2812b_IncLedCnt3( &u1a_LedG, &u1a_LedB, &u1a_LedR );
    if ( LED_BRIGHTNESS == u1a_LedR )
    {
      u1a_CountMode++;
    }
    break;

    case 11U:
    f_WS2812b_DecLedCnt3( &u1a_LedG, &u1a_LedB, &u1a_LedR );
    if ( 0U == u1a_LedR )
    {
      u1a_CountMode = 0U;
    }
    break;

  default:
    break;
  }


  for ( u1a_LedRow = 0U; u1a_LedRow < 4U; u1a_LedRow++ )
    for ( u1a_LedCol = 0U; u1a_LedCol < 4U; u1a_LedCol++ )
    {
      u1l_WS2812B_ColorMpu[u1a_LedRow][u1a_LedCol][0] = u1a_LedG;
      u1l_WS2812B_ColorMpu[u1a_LedRow][u1a_LedCol][1] = u1a_LedR;
      u1l_WS2812B_ColorMpu[u1a_LedRow][u1a_LedCol][2] = u1a_LedB;
    }

  f_WS2812B_SendByteArray( (uint8_t* const)&u1l_WS2812B_ColorMpu[0U], (uint32_t)(4U * 4U * 3U) );

  return;
}

void f_WS2812b_LedEffect2( void )
{
  uint8_t* p2u1a_Buff;
  p2u1a_Buff = (uint8_t*)u1l_WS2812B_ColorMpu;
  for ( uint8_t u1a_Cnt = 0; u1a_Cnt < 48U; u1a_Cnt++ )
  {
    p2u1a_Buff[u1a_Cnt] = LED_BRIGHTNESS_EFFECT2;
  }

  f_WS2812B_SendByteArray( (uint8_t* const)&u1l_WS2812B_ColorMpu[0U], (uint32_t)(4U * 4U * 3U) );

  return;
}

static void f_WS2812B_Delay( const uint32_t u4a_CntThreshold )
{
  uint32_t u4a_Cnt;
  for( u4a_Cnt = 0UL; u4a_Cnt < u4a_CntThreshold; u4a_Cnt++ )
  {
    __asm volatile( "nop" );
  }
}

static void f_WS2812B_SendBitOne( void )
{
  /* Set High level within 800ns */
  HAL_GPIO_WritePin( DI_WS2314_GPIO_Port, DI_WS2314_Pin, GPIO_PIN_SET );
  f_WS2812B_Delay( 8UL );

  /* Set High level within 450ns */
  HAL_GPIO_WritePin( DI_WS2314_GPIO_Port, DI_WS2314_Pin, GPIO_PIN_RESET );
  __asm volatile( "nop" );

  return;
}

static void f_WS2812B_SendBitZero( void )
{
  /* Set High level within 400ns */
  HAL_GPIO_WritePin( DI_WS2314_GPIO_Port, DI_WS2314_Pin, GPIO_PIN_SET );
  f_WS2812B_Delay( 2UL );

  /* Set High level within 850ns */
  HAL_GPIO_WritePin( DI_WS2314_GPIO_Port, DI_WS2314_Pin, GPIO_PIN_RESET );
  f_WS2812B_Delay( 4UL );

  return;
}

static void f_WS2812B_SendBitZeroLatest( void )
{
  /* Set High level within 400ns */
  HAL_GPIO_WritePin( DI_WS2314_GPIO_Port, DI_WS2314_Pin, GPIO_PIN_SET );
  f_WS2812B_Delay( 2UL );

  /* Set High level within 850ns */
  HAL_GPIO_WritePin( DI_WS2314_GPIO_Port, DI_WS2314_Pin, GPIO_PIN_RESET );
  f_WS2812B_Delay( 2UL );

  return;
}

static void f_WS2812B_SendByte( const uint8_t u1a_Byte )
{
  uint8_t u1a_BitIndex;

  for( u1a_BitIndex = 0U; u1a_BitIndex < 7U; u1a_BitIndex++ )
  {
    if( ( u1a_Byte & ( 0x80U >> u1a_BitIndex ) ) != 0U )
    {
      f_WS2812B_SendBitOne();
    }
    else
    {
      f_WS2812B_SendBitZero();
    }
  }

  /* Send last byte */
  if( ( u1a_Byte & 1U ) != 0U )
  {
    f_WS2812B_SendBitOne();
  }
  else
  {
    f_WS2812B_SendBitZeroLatest();
  }

  return;
}

static void f_WS2812B_SendByteArray( uint8_t* const pu1a_ByteArray, const uint32_t u4a_ByteCount )
{
    /* Critial section */
    __set_PRIMASK(1UL);
  uint32_t u4a_Index;

  for( u4a_Index = 0UL; u4a_Index < u4a_ByteCount; u4a_Index++ )
  {
    f_WS2812B_SendByte( pu1a_ByteArray[ u4a_Index ] );
  }
  /* Critial section */
  __set_PRIMASK(0UL);
  return;
}

static void f_WS2812B_SendColor( const uint8_t u1a_Red, const uint8_t u1a_Green, const uint8_t u1a_Blue )
{
  /* Send color data in GRB format */
  f_WS2812B_SendByte( u1a_Green );
  f_WS2812B_SendByte( u1a_Red );
  f_WS2812B_SendByte( u1a_Blue );

  return;
}

static void f_WS2812B_SwapCol( const uint8_t u1a_ColId )
{
  uint8_t u1a_TempClolorPixel[U1L_LED_COLOR_NUM];
  const uint8_t u1a_RowLoop = U1L_LED_ROWS / 2U;
  const uint8_t u1a_RowIdexMax = U1L_LED_ROWS - 1U;

  /* Swap column */
  for ( uint8_t u1a_Row = 0; u1a_Row < u1a_RowLoop; u1a_Row++ )
  {
    /* Copy B -> A */
    for ( uint8_t u1a_LedColor = 0U; u1a_LedColor < U1L_LED_COLOR_NUM; u1a_LedColor++ )
    {
      u1a_TempClolorPixel[u1a_LedColor] = u1l_WS2812B_ColorMpu[u1a_ColId][u1a_Row][u1a_LedColor];
    }

    /* Copy C -> B */
    for ( uint8_t u1a_LedColor = 0U; u1a_LedColor < U1L_LED_COLOR_NUM; u1a_LedColor++ )
    {
      u1l_WS2812B_ColorMpu[u1a_ColId][u1a_Row][u1a_LedColor] = u1l_WS2812B_ColorMpu[u1a_ColId][u1a_RowIdexMax-u1a_Row][u1a_LedColor];
    }

    /* Copy A -> C */
    for ( uint8_t u1a_LedColor = 0U; u1a_LedColor < U1L_LED_COLOR_NUM; u1a_LedColor++ )
    {
      u1l_WS2812B_ColorMpu[u1a_ColId][u1a_RowIdexMax-u1a_Row][u1a_LedColor] = u1a_TempClolorPixel[u1a_LedColor];
    }
  }

  return;
}

static uint8_t f_WS2812b_MapLedIndex( const float f4a_Angle, const float f4a_MaxAngle, \
                                      const float f4a_MinAngle, const uint8_t u1a_LedNum )
{
  uint8_t u1a_LedIdex;

  if ( f4a_MinAngle > f4a_Angle )
  {
    u1a_LedIdex = 0U;
  }
  else if ( f4a_MaxAngle < f4a_Angle )
  {
    u1a_LedIdex = u1a_LedNum - 1U;
  }
  else
  {
    u1a_LedIdex = (uint8_t)( ( (float)u1a_LedNum * ( f4a_Angle - f4a_MinAngle ) ) / ( f4a_MaxAngle - f4a_MinAngle ) );
    if ( u1a_LedNum <= u1a_LedIdex )
    {
      u1a_LedIdex = u1a_LedNum - 1U;
    }
  }

  return u1a_LedIdex;
}

static void f_WS2812b_ClearLedValue( void )
{
  uint8_t* p2u1a_Buff;

  p2u1a_Buff = (uint8_t*)u1l_WS2812B_ColorMpu;
  for ( uint8_t u1a_Cnt = 0; u1a_Cnt < 48U; u1a_Cnt++ )
  {
    p2u1a_Buff[u1a_Cnt] = 0U;
  }

  return;
}

#endif // WS2812B_H
