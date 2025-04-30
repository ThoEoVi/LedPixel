/**
 * @file Mpu.c
 * @file           : Mpu.c
 * @brief          : Source file for MPU driver
 ******************************************************************************
 * @attention
 *
 * This software component is licensed by [Your Company] under [License Name],
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        [License URL]
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "Mpu_If.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  EN_MPU_FLASH_MEMORY = 0U,
  EN_MPU_INTERNAL_SRAM, /* 1 */
  EN_MPU_GPIO_PORTC, /* 2 */
  EN_MPU_RCC, /* 3 */
  EN_MPU_GPIO_PORTB, /* 4 */
  EN_MPU_REGION_NUM, /* 5 */
  EN_MPU_REGION_MAX = 8U
} t_MPU_MemoryRegion;

typedef struct
{
  uint32_t u4_BaseAddr;
  uint32_t u4_EndAddr;
  uint32_t u4_Option;
} t_MPU_MemoryInfo;

/* Private define ------------------------------------------------------------*/
#define MPU_DEFS_RASR_SIZE_256KB (0x11UL << MPU_RASR_SIZE_Pos) /* (Region size in bytes) = 2^(SIZE+1) => 256kB = 2^(0x11 + 1) */
#define MPU_DEFS_RASR_SIZE_64KB (0x0FUL << MPU_RASR_SIZE_Pos)
#define MPU_DEFS_RASR_SIZE_1KB (0x09UL << MPU_RASR_SIZE_Pos)

#define MPU_DEFS_RASE_AP_FULL_ACCESS (0x3 << MPU_RASR_AP_Pos)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
Table 42. Memory region attributes for STM32

Memory region      TEX   C   B   S   Memory type and attributes
Flash memory       b000  1   0   0   Normal memory, Non-shareable, write-through
Internal SRAM      b000  1   0   1   Normal memory, Shareable, write-through
External SRAM      b000  1   1   1   Normal memory, Shareable, write-back, write-allocate
Peripherals        b000  0   1   1   Device memory, Shareable
*/

static const t_MPU_MemoryInfo xnl_MPU_MemoryInfo[ EN_MPU_REGION_NUM ] =
{
  {
    // EN_MPU_FLASH_MEMORY
    0x08000000UL,
    0x0803FFFFUL, /* 256KB */
    // MPU_DEFS_RASE_AP_FULL_ACCESS | U4G_MPU_FLASH_MEMORY_ATTRIBUTE | MPU_DEFS_RASR_SIZE_256KB | MPU_RASR_ENABLE_Msk
    0x3020023UL
  },
  {
    // EN_MPU_INTERNAL_SRAM
    0x20000000UL,
    0x2000FFFFUL, /* 64KB */
    // MPU_DEFS_RASE_AP_FULL_ACCESS | U4G_MPU_INTERNAL_SRAM_ATTRIBUTE | MPU_DEFS_RASR_SIZE_64KB | MPU_RASR_ENABLE_Msk
    0x306001FUL
  },
  { /* EN_MPU_GPIO_PORTC */
    0x40020800UL,
    0x40020BFFUL, /* 1KB */
    0x3050013UL
  },
  { /* EN_MPU_RCC */
    0x40023800UL,
    0x40023BFFUL, /* 1KB */
    0x3050013UL
  },
  { /* EN_MPU_GPIO_PORTB */
    0x40020400UL,
    0x400207FFUL, /* 1KB */
    0x3050013UL
  }
};

/* Private function prototypes -----------------------------------------------*/
static void f_MPU_DisableRegion( uint32_t u4a_RegionNum );
/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void f_Mpu_Init( void )
{
  /* Enable Mem fault */
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk; // Set bit 16

  /* Whether MPU is supported */
  if (MPU->TYPE==0) {return;} // Return 1 to indicate error

  /* Disable MPU */
  __DMB(); // Make sure outstanding transfers are done
  MPU->CTRL = 0UL; // Disable the MPU

  /* Configure region */
  for ( uint8_t u1a_RegionId = EN_MPU_FLASH_MEMORY; u1a_RegionId < EN_MPU_REGION_NUM; u1a_RegionId++ )
  {
    /* Select region */
    MPU->RNR = (t_MPU_MemoryRegion)u1a_RegionId;
    /* Select addr */
    MPU->RBAR = xnl_MPU_MemoryInfo[(t_MPU_MemoryRegion)u1a_RegionId].u4_BaseAddr;
    /* MPU region attribute and size register */
    MPU->RASR = xnl_MPU_MemoryInfo[(t_MPU_MemoryRegion)u1a_RegionId].u4_Option;
  }

  /* Disable remain regions */
  for ( uint8_t u1a_RegionId = EN_MPU_REGION_NUM; u1a_RegionId < EN_MPU_REGION_MAX; u1a_RegionId++ )
  {
    f_MPU_DisableRegion( u1a_RegionId );
  }

  /* Enalbe MPU */
  MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_HFNMIENA_Msk; // Disable the MPU
  __DSB(); // Ensure MPU settings take effects
  __ISB(); // Sequence instruction fetches using update settings

  return;
}

static void f_MPU_DisableRegion( uint32_t u4a_RegionNum )
{
  MPU->RNR = u4a_RegionNum;
  MPU->RBAR = 0UL;
  MPU->RASR = 0UL;
  return;
}

/* EOF */
