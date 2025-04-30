#ifndef WS2812B_H
#define WS2812B_H

#include "WS2812b_If.h"
#include "main.h"

// Define the number of LEDs in your WS2812b strip
#define WS2812B_LED_COUNT 16

static uint8_t u1l_WS2812B_Color[16U][3u];
static uint32_t u4l_WS2812B_ColorIndex;



static void f_WS2812B_Delay( const uint32_t u4a_CntThreshold );
static void f_WS2812B_SendBitOne( void );
static void f_WS2812B_SendBitZero( void );
static void f_WS2812B_SendBitZeroLatest( void );
static void f_WS2812B_SendByte( const uint8_t u1a_Byte );
static void f_WS2812B_SendColor( const uint8_t u1a_Red, const uint8_t u1a_Green, const uint8_t u1a_Blue );
static void f_WS2812B_SendByteArray( uint8_t* const pu1a_ByteArray, const uint32_t u4a_ByteCount );


void f_WS2812b_Init( void )
{
  HAL_Delay( 1000UL );
  /* Set High level within 850ns */
  HAL_GPIO_WritePin( DI_WS2314_GPIO_Port, DI_WS2314_Pin, GPIO_PIN_RESET );
  f_WS2812B_Delay( 2000UL );

  return;
}

void f_WS2812b_LedEffectOne( void )
{
    u4l_WS2812B_ColorIndex++;

    for ( uint8_t u1a_LedIndex = 0U; u1a_LedIndex < 16U; u1a_LedIndex++ )
    {
      u1l_WS2812B_Color[u1a_LedIndex][0U] = 0x00U;
      u1l_WS2812B_Color[u1a_LedIndex][1U] = 0x00U;
      u1l_WS2812B_Color[u1a_LedIndex][2U] = 0x00U;
    }

      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex%16U)][0U] = 0x11U;
      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex%16U)][1U] = 0x06U;
      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex%16U)][2U] = 0x11U;

      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*35U/7U)%16U][0U] = ((u4l_WS2812B_ColorIndex*25U/8U)%16U)%0x2U;
      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*35U/7U)%16U][1U] = ((u4l_WS2812B_ColorIndex*25U/8U)%16U)%0x2U;
      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*35U/7U)%16U][2U] = ((u4l_WS2812B_ColorIndex*25U/8U)%16U)%0x1U;

      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*25U/8U)%16U][0U] = ((u4l_WS2812B_ColorIndex*2U/8U)%16U)%0x2U;
      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*25U/8U)%16U][1U] = ((u4l_WS2812B_ColorIndex*25U/8U)%16U)%0x2U;
      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*25U/8U)%16U][2U] = ((u4l_WS2812B_ColorIndex*2U/8U)%16U)%0x1U;

      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*3U/8U)%16U][0U] = ((u4l_WS2812B_ColorIndex*25U/8U)%16U)%0x10U;
      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*3U/8U)%16U][1U] = ((u4l_WS2812B_ColorIndex*25U/8U)%16U)%0x2U;
      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*3U/8U)%16U][2U] = ((u4l_WS2812B_ColorIndex*25U/8U)%16U)%0x15U;

      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*5U/8U)%16U][0U] = ((u4l_WS2812B_ColorIndex*5U/3U)%16U)%0x8U;
      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*5U/8U)%16U][1U] = ((u4l_WS2812B_ColorIndex*5U/3U)%16U)%0x7U;
      u1l_WS2812B_Color[(u4l_WS2812B_ColorIndex*5U/8U)%16U][2U] = ((u4l_WS2812B_ColorIndex*5U/3U)%16U)%0x6U;

    f_WS2812B_SendByteArray( (uint8_t* const)&u1l_WS2812B_Color[0U], (uint32_t)(16U * 3U) );
    u4l_WS2812B_ColorIndex++;
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
  uint32_t u4a_Index;

  for( u4a_Index = 0UL; u4a_Index < u4a_ByteCount; u4a_Index++ )
  {
    f_WS2812B_SendByte( pu1a_ByteArray[ u4a_Index ] );
  }

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


#endif // WS2812B_H
