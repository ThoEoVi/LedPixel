#ifndef WS2812B_IF_H
#define WS2812B_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

// Function prototypes
extern void f_WS2812b_Init( void );
extern void f_WS2812b_LedEffect1( void );
extern void f_WS2812b_LedPixelControlMode( float f4a_X, float f4a_Y, uint8_t* const p2u1a_Mode );
extern void f_WS2812b_LedEffect2( void );
extern void f_WS2812b_LedEffect3( void );

#ifdef __cplusplus
}
#endif

#endif // WS2812B_IF_H
