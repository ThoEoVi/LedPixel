#ifndef WS2812B_IF_H
#define WS2812B_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

// Function prototypes
extern void f_WS2812b_Init( void );
extern void f_WS2812b_LedEffectOne( void );

#ifdef __cplusplus
}
#endif

#endif // WS2812B_IF_H
