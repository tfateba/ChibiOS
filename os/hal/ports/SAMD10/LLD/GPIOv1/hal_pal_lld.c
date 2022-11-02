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
 * @file    GPIOv1/hal_pal_lld.c
 * @brief   SAMD10 PAL low level driver code.
 *
 * @addtogroup PAL
 * @{
 */

#include "hal.h"
#include "samd10_gpio.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

ioportmask_t pal_lld_readport(samd10_gpio_t *port) {
  return (ioportmask_t)port->IN;
}

ioportmask_t pal_lld_readlatch(samd10_gpio_t *port) {
  return port->OUT;
}

void pal_lld_writeport(samd10_gpio_t *port, ioportmask_t bits) {
  port->OUT |= (1 << bits);
}

void pal_lld_setport(samd10_gpio_t *port, ioportmask_t bits) {
  port->OUTSET |= (1 << bits);
}

void pal_lld_clearport(samd10_gpio_t *port, ioportmask_t bits) {
  port->OUTCLR |= (1 << bits);
}

void pal_lld_toggleport(samd10_gpio_t *port, ioportmask_t bits)  {
  port->OUTTGL |= (1 << bits);
}

/**
 * @brief   PAL driver initialization.
 *
 * @notapi
 */
void __pal_lld_init(void) {

  //hal_lld_peripheral_unreset(RESETS_ALLREG_IO_BANK0);
  //hal_lld_peripheral_unreset(RESETS_ALLREG_PADS_BANK0);
}

#endif /* HAL_USE_PAL */

/** @} */
