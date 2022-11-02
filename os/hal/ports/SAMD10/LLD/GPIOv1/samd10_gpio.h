/*
    ChibiOS - Copyright (C) 2006..2022 Theodore Ateba

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    GPIOv1/samd10_gpio.h
 * @brief   SAMD10 GPIO units common header.
 * @note    This file requires definitions from the ATMEL SAMD10 header file.
 *
 * @addtogroup SAMD10_GPIOv1
 * @{
 */

#ifndef SAMD10_GPIO_H
#define SAMD10_GPIO_H

#include <stdint.h>
#include "samd10d14am.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

//#define samd10_gpio_t PORT_TypeDef

/**
 * @name    GPIO ports definitions
 * @{
 */
#define GPIOA                           ((samd10_gpio_t *)PORT_BASE)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   SAMD10 GPIO registers block.
 */
typedef struct {
  volatile uint32_t   DIR;
  volatile uint32_t   DIRCLR;
  volatile uint32_t   DIRSET;
  volatile uint32_t   DIRTGL;
  volatile uint32_t   OUT;
  volatile uint32_t   OUTCLR;
  volatile uint32_t   OUTSET;
  volatile uint32_t   OUTTGL;
  volatile uint32_t   IN;
  volatile uint32_t   CTRL;
  
  volatile union {
    uint32_t reg;
    struct {
      uint32_t PINMASK:16;
      uint32_t PMUXEN:1;
      uint32_t INEN:1;
      uint32_t PULLEN:1;
      uint32_t :3;
      uint32_t DRVSTR:1;
      uint32_t :1;
      uint32_t PMUX:4;
      uint32_t WRPMUX:1;
      uint32_t :1;
      uint32_t WRPINCFG:1;
      uint32_t HWSEL:1;
    } bit;
  } WRCONFIG;

  volatile uint8_t    Reserved1[4];

  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX0;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX1;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX2;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX3;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX4;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX5;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX6;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX7;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX8;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX9;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX10;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX11;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX12;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX13;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX14;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXE:4;
      uint8_t PMUXO:4;
    } bit;
  } PMUX15;
  
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG0;

  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG1;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG2;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG3;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG4;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG5;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG6;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG7;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG8;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG9;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG10;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG11;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG12;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG13;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG14;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG15;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG16;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG17;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG18;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG19;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG20;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG21;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG22;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG23;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG24;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG25;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG26;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG27;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG28;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG29;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG30;
  volatile union {
    uint8_t reg;
    struct {
      uint8_t PMUXEN:1;
      uint8_t INEN:1;
      uint8_t PULLEN:1;
      uint8_t :3;
      uint8_t DRVSTR:1;
      uint8_t :1;
    } bit;
  } PINCFG31;

  volatile uint8_t    Reserved2[32];
} samd10_gpio_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* STM32_GPIO_H */

/** @} */
