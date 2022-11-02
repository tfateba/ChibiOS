/**
 * @file    samd10d14am.h
 * @brief   CMSIS device file for the SAMD10D14AM.
 *
 * @defgroup CMSIS_SAMD10D14AM SAMD10D14AM Device File
 * @ingroup CMSIS_DEVICE
 * @{
 */
    
#ifndef SAMD10D14AM_H
#define SAMD10D14AM_H

#include <stdint.h>

/*
 * @brief Configuration of the Cortex-M0+ Processor and Core Peripherals
 */
#define __CM0PLUS_REV          1  /**< Core revision r0p1.                            */
#define __MPU_PRESENT          0  /**< MPU present or not.                            */
#define __VTOR_PRESENT         1  /**< VTOR present or not.                           */
#define __NVIC_PRIO_BITS       2  /**< Number of bits used for Priority Levels.       */
#define __Vendor_SysTickConfig 0  /**< Set to 1 if different SysTick Config is used.  */

/**
 * @brief   Interrupt vector numbers.
 */
typedef enum {
  /**
   * @brief   Cortex-M0+ Processor Exceptions Numbers
   */
  NonMaskableInt_IRQn = -14,      /**<  2 Non Maskable Interrupt.         */
  HardFault_IRQn      = -13,      /**<  3 Hard Fault Interrupt.           */
  SVCall_IRQn         = -5,       /**< 11 SV Call Interrupt.              */
  PendSV_IRQn         = -2,       /**< 14 Pend SV Interrupt.              */
  SysTick_IRQn        = -1,       /**< 15 System Tick Interrupt.          */

  /**
   * @brief   SAMD10D14AM-specific Interrupt Numbers
   */
  PM_IRQn             =  0,       /**< Power Manager.                     */
  SYSCTRL_IRQn        =  1,       /**< System Control.                    */
  WDT_IRQn            =  2,       /**< Watchdog Timer.                    */
  RTC_IRQn            =  3,       /**< Real-Time Counter.                 */
  EIC_IRQn            =  4,       /**< External Interrupt Controller.     */
  NVMCTRL_IRQn        =  5,       /**< Non-Volatile Memory Controller.    */
  DMAC_IRQn           =  6,       /**< Direct Memory Access Controller.   */
  EVSYS_IRQn          =  8,       /**< Event System Interface.            */
  SERCOM0_IRQn        =  9,       /**< Serial Communication Interface 0.  */
  SERCOM1_IRQn        = 10,       /**< Serial Communication Interface 1.  */
  SERCOM2_IRQn        = 11,       /**< Serial Communication Interface 2.  */
  TCC0_IRQn           = 12,       /**< Timer Counter Control 0.           */
  TC1_IRQn            = 13,       /**< Basic Timer Counter 1.             */
  TC2_IRQn            = 14,       /**< Basic Timer Counter 2.             */
  ADC_IRQn            = 15,       /**< Analog Digital Converter.          */
  AC_IRQn             = 16,       /**< Analog Comparators.                */
  DAC_IRQn            = 17,       /**< Digital Analog Converter.          */
  PTC_IRQn            = 18,       /**< Peripheral Touch Controller.       */
  PERIPH_COUNT_IRQn   = 19        /**< Number of peripheral IDs.          */
} IRQn_Type;

#include "core_cm0plus.h"   /* Cortex-M0plus processor and periphrals   */
#include "system_samd10.h"  /* SAMD10 system header                     */
#include <stdint.h>

typedef struct {
  __IO uint8_t  CTRLA;
  __O  uint8_t  CTRLB;
  __IO uint16_t EVCTRL;
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
       uint8_t  Reserved1[1];
  __I  uint8_t  STATUSA;
  __I  uint8_t  STATUSB;
  __I  uint8_t  STATUSC;
       uint8_t  Reserved2[1];
  __IO uint8_t  WINCTRL;
       uint8_t  Reserved3[3];
  __IO uint32_t COMPCTRL0;
  __IO uint32_t COMPCTRL1;
       uint8_t  Reserved4[8];
  __IO uint8_t  SCALER0;
  __IO uint8_t  SCALER1;
} AC_Typedef;

typedef struct {
  __IO uint8_t  CTRLA;
  __IO uint8_t  REFCTRL;
  __IO uint8_t  AVGCTRL;
  __IO uint8_t  SAMPCTRL;
  __IO uint16_t CTRLB;
       uint8_t  Reserved1[2];
  __IO uint8_t  WINCTRL;
       uint8_t  Reserved2[3];
  __IO uint8_t  SWTRIG;
       uint8_t  Reserved3[3];
  __IO uint32_t INPUTCTRL;
  __IO uint8_t  EVCTRL;
       uint8_t  Reserved4[1];
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
  __I  uint8_t  STATUS;
  __I  uint16_t RESULT;
  __IO uint16_t WINLT;
       uint8_t  Reserved5[2];
  __IO uint16_t WINUT;
       uint8_t  Reserved6[2];
  __IO uint16_t GAINCORR;
  __IO uint16_t OFFSETCORR;
  __IO uint16_t CALIB;
  __IO uint8_t  DBGCTRL;
} ADC_Typedef;

typedef struct {
  __IO uint8_t  CTRLA;
  __IO uint8_t  CTRLB;
  __IO uint8_t  EVCTRL;
       uint8_t  Reserved1[1];
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
  __I  uint8_t  STATUS;
  __IO uint16_t DATA;
       uint8_t  Reserved2[2];
  __IO uint16_t DATABUF;
} DAC_Typedef;

typedef struct {
  __IO uint16_t CTRL;
  __IO uint16_t CRCCTRL;
  __IO uint32_t CRCDATAIN;
  __IO uint32_t CRCCHKSUM;
  __IO uint8_t  CRCSTATUS;
  __IO uint8_t  DBGCTRL;
  __IO uint8_t  QOSCTRL;
       uint8_t  Reserved1[1];
  __IO uint32_t SWTRIGCTRL;
  __IO uint32_t PRICTRL0;
       uint8_t  Reserved2[8];
  __IO uint16_t INTPEND;
       uint8_t  Reserved3[2];
  __I  uint32_t INTSTATUS;
  __I  uint32_t BUSYCH;
  __I  uint32_t PENDCH;
  __I  uint32_t ACTIVE;
  __IO uint32_t BASEADDR;
  __IO uint32_t WRBADDR;
       uint8_t  Reserved4[3];
  __IO uint8_t  CHID;
  __IO uint8_t  CHCTRLA;
       uint8_t  Reserved5[3];
  __IO uint32_t CHCTRLB;
       uint8_t  Reserved6[4];
  __IO uint8_t  CHINTENCLR;
  __IO uint8_t  CHINTENSET;
  __IO uint8_t  CHINTFLAG;
  __I  uint8_t  CHSTATUS;
} DMAC_Typedef;

// DMA Controler descriptor.
typedef struct {
  __IO uint16_t BTCTRL;
  __IO uint16_t BTCNT;
  __IO uint32_t SRCADDR;
  __IO uint32_t DSTADDR;
  __IO uint32_t DESCADDR;
} DMACD_Typedef;

typedef struct {
  __O  uint8_t  CTRL;
  __IO uint8_t  STATUSA;
  __I  uint8_t  STATUSB;
       uint8_t   Reserved1[1];
  __IO uint32_t ADDR;
  __IO uint32_t LENGTH;
  __IO uint32_t DATA;
  __IO uint32_t DCC0;
  __IO uint32_t DCC1;
  __I  uint32_t DID;
       uint8_t  Reserved2[212];
  __IO uint32_t DCFG0;
  __IO uint32_t DCFG1;
       uint8_t  Reserved3[3848];
  __I  uint32_t ENTRY0;
  __I  uint32_t ENTRY1;
  __I  uint32_t END;
       uint8_t  Reserved4[4032];
  __I  uint32_t MEMTYPE;
  __I  uint32_t PID4;
  __I  uint32_t PID5;
  __I  uint32_t PID6;
  __I  uint32_t PID7;
  __I  uint32_t PID0;
  __I  uint32_t PID1;
  __I  uint32_t PID2;
  __I  uint32_t PID3;
  __I  uint32_t CID0;
  __I  uint32_t CID1;
  __I  uint32_t CID2;
  __I  uint32_t CID3;
} DSU_Typedef;

typedef struct {
  __IO uint8_t  CTRL;
  __I  uint8_t  STATUS;
  __IO uint8_t  NMICTRL;
  __IO uint8_t  NMIFLAG;
  __IO uint32_t EVCTRL;
  __IO uint32_t INTENCLR;
  __IO uint32_t INTENSET;
  __IO uint32_t INTFLAG;
  __IO uint32_t WAKEUP;
  __IO uint32_t CONFIG0;
} EIC_Typedef;

typedef struct {
  __O  uint8_t  CTRL;
       uint8_t  Reserved1[3];
  __IO uint32_t CHANNEL;
  __IO uint16_t USER;
       uint8_t  Reserved2[2];
  __I  uint32_t CHSTATUS;
  __IO uint32_t INTENCLR;
  __IO uint32_t INTENSET;
  __IO uint32_t INTFLAG;
} EVSYS_Typedef;

typedef struct {
  __IO uint8_t  CTRL;
  __I  uint8_t  STATUS;
  __IO uint16_t CLKCTRL;
  __IO uint32_t GENCTRL;
  __IO uint32_t GENDIV;
} GCLK_Typedef;

/*
typedef struct {

} MTB_Typedef;*/

typedef struct {
  __IO uint16_t CTRLA;
       uint8_t  Reserved1[2];
  __IO uint32_t CTRLB;
  __IO uint32_t PARAM;
  __IO uint8_t  INTENCLR;
       uint8_t  Reserved2[3];
  __IO uint8_t  INTENSET;
       uint8_t  Reserved3[3];
  __IO uint8_t  INTFLAG;
       uint8_t  Reserved4[3];
  __IO uint16_t STATUS;
       uint8_t  Reserved5[2];
  __IO uint32_t ADDR;
  __IO uint16_t LOCK;
} NVMCTRL_Typedef;

/*
typedef struct {

} PAC0_Typedef;

typedef struct {

} PAC1_Typedef;

typedef struct {

} PAC2_Typedef;
*/

typedef struct {
  __IO uint8_t  CTRL;
  __IO uint8_t  SLEEP;
  __IO uint8_t  EXTCTRL;
       uint8_t  Reserved1[5];
  __IO uint8_t  CPUSEL;
  __IO uint8_t  APBASEL;
  __IO uint8_t  APBBSEL;
  __IO uint8_t  APBCSEL;
       uint8_t  Reserved2[8];
  __IO uint32_t AHBMASK;
  __IO uint32_t APBAMASK;
  __IO uint32_t APBBMASK;
  __IO uint32_t APBCMASK;
       uint8_t  Reserved3[16];
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
       uint8_t  Reserved4[1];
  __I  uint8_t  RCAUSE;
} PM_Typedef;

typedef struct {
  __IO uint32_t   DIR;
  __IO uint32_t   DIRCLR;
  __IO uint32_t   DIRSET;
  __IO uint32_t   DIRTGL;
  __IO uint32_t   OUT;
  __IO uint32_t   OUTCLR;
  __IO uint32_t   OUTSET;
  __IO uint32_t   OUTTGL;
  __I  uint32_t   IN;
  __IO uint32_t   CTRL;
  __O  uint32_t   WRCONFIG;
       uint8_t    Reserved1[4];
  __IO uint8_t    PMUX[16];
  __IO uint8_t    PINCFG[32];
       uint8_t    Reserved2[32];
} PORT_TypeDef;

/*
typedef struct {

} PTC_Typedef;
*/
typedef struct {
  __IO uint16_t CTRL;
  __IO uint16_t READREQ;
  __IO uint16_t EVCTRL;
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
       uint8_t  Reserved1[1];
  __IO uint8_t  STATUS;
  __IO uint8_t  DBGCTRL;
  __IO uint8_t  FREQCORR;
       uint8_t  Reserved2[3];
  __IO uint32_t COUNT;
       uint8_t  Reserved3[4];
  __IO uint32_t COMP0;
} RTCMODE0_Typedef;

typedef struct {
  __IO uint16_t CTRL;
  __IO uint16_t READREQ;
  __IO uint16_t EVCTRL;
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
       uint8_t  Reserved1[1];
  __IO uint8_t  STATUS;
  __IO uint8_t  DBGCTRL;
  __IO uint8_t  FREQCORR;
       uint8_t  Reserved2[3];
  __IO uint16_t COUNT;
       uint8_t  Reserved3[2];
  __IO uint16_t PER;
       uint8_t  Reserved4[2];
  __IO uint16_t COMP0;
  __IO uint16_t COMP1;
} RTCMODE1_Typedef;

typedef struct {
  __IO uint16_t CTRL;
  __IO uint16_t READREQ;
  __IO uint16_t EVCTRL;
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
       uint8_t  Reserved1[1];
  __IO uint8_t  STATUS;
  __IO uint8_t  DBGCTRL;
  __IO uint8_t  FREQCORR;
       uint8_t  Reserved2[3];
  __IO uint32_t CLOCK;
       uint8_t  Reserved3[4];
       uint16_t ALARM;
       uint8_t  MASK;
} RTCMODE2_Typedef;

typedef struct {
  RTCMODE0_Typedef  MODE0;
  RTCMODE1_Typedef  MODE1;
  RTCMODE2_Typedef  MODE2;
} RTC_Typedef;

typedef struct {
  __IO uint32_t CTRLA;
  __IO uint32_t CTRLB;
       uint8_t  Reserved1[4];
  __IO uint16_t BAUD;
  __IO uint8_t  RXPL;
       uint8_t  Reserved2[5];
  __IO uint8_t  INTENCLR;
       uint8_t  Reserved3[1];
  __IO uint8_t  INTENSET;
       uint8_t  Reserved4[1];
  __IO uint8_t  INTFLAG;
       uint8_t  Reserved5[1];
  __IO uint16_t STATUS;
  __I  uint32_t SYNCBUSY;
       uint8_t  Reserved6[8];
  __IO uint16_t DATA;
       uint8_t  Reserved7[6];
  __IO uint8_t  DBGCTRL; 
} SERCOMUSART_Typedef;

typedef struct {
  __IO uint32_t CTRLA;
  __IO uint32_t CTRLB;
       uint8_t  Reserved1[4];
  __IO uint8_t  BAUD;
       uint8_t  Reserved2[7];
  __IO uint8_t  INTENCLR;
       uint8_t  Reserved3[1];
  __IO uint8_t  INTENSET;
       uint8_t  Reserved4[1];
  __IO uint8_t  INTFLAG;
       uint8_t  Reserved5[1];
  __IO uint16_t STATUS;
  __I  uint32_t SYNCBUSY;
       uint8_t  Reserved6[4];
  __IO uint32_t ADDR;
  __IO uint16_t DATA;
       uint8_t  Reserved7[4];
  __IO uint8_t  DBGCTRL;
} SERCOMSPI_Typedef;

typedef struct {
  __IO uint32_t CTRLA;
  __IO uint32_t CTRLB;
       uint8_t  Reserved1[12];
  __IO uint8_t  INTENCLR;
       uint8_t  Reserved2[1];
  __IO uint8_t  INTENSET;
       uint8_t  Reserved3[1];
  __IO uint8_t  INTFLAG;
       uint8_t  Reserved4[1];
  __IO uint16_t STATUS;
  __I  uint32_t SYNCBUSY;
       uint8_t  Reserved5[4];
  __IO uint32_t ADDR;
  __IO uint16_t DATA;
} SERCOMI2CSLAVEMODE_Typedef;

typedef struct {
  __IO uint32_t CTRLA;
  __IO uint32_t CTRLB;
       uint8_t  Reserved1[4];
  __IO uint32_t BAUD;
       uint8_t  Reserved2[4];
  __IO uint8_t  INTENCLR;
       uint8_t  Reserved3[1];
  __IO uint8_t  INTENSET;
       uint8_t  Reserved4[1];
  __IO uint8_t  INTFLAG;
       uint8_t  Reserved5[1];
  __IO uint16_t STATUS;
  __I  uint32_t SYNCBUSY;
       uint8_t  Reserved6[4];
  __IO uint32_t ADDR;
  __IO uint16_t DATA;
       uint8_t  Reserved7[7];
  __IO uint8_t  DBGCTRL;
} SERCOMI2CMASTERMODE_Typedef;

typedef struct {
  SERCOMI2CSLAVEMODE_Typedef  SLAVEMODE;
  SERCOMI2CMASTERMODE_Typedef MASTERMODE;
} SERCOMI2C_Typedef;

typedef struct {
  SERCOMUSART_Typedef USART;
  SERCOMSPI_Typedef   SPI;
  SERCOMI2C_Typedef   I2C;
} SERCOM_Typedef;

typedef struct {
  __IO uint32_t INTENCLR;
  __IO uint32_t INTENSET;
  __IO uint32_t INTFLAG;
  __I  uint32_t PCLKSR;
  __IO uint16_t XOSC;
       uint8_t  Reserved1[2];
  __IO uint16_t XOSC32K;
       uint8_t  Reserved2[2];
  __IO uint32_t OSC32K;
  __IO uint8_t  OSCULP32K;
       uint8_t  Reserved3[3];
  __IO uint32_t OSC8M;
  __IO uint16_t DFLLCTRL;
       uint8_t  Reserved4[2];
  __IO uint32_t DFLLVAL;
  __IO uint32_t DFLLMUL;
  __IO uint8_t  DFLLSYNC;
       uint8_t  Reserved5[3];
  __IO uint32_t BOD33;
       uint8_t  Reserved6[8];
  __IO uint32_t VREF;
  __IO uint8_t  DPLLCTRLA;
       uint8_t  Reserved7[3];
  __IO uint32_t DPLLRATIO;
  __IO uint32_t DPLLCTRLB;
  __I  uint8_t  DPLLSTATUS;
} SYSCTRL_Typedef;

typedef struct {
  __IO uint16_t CTRLA;
  __IO uint16_t READREQ;
  __IO uint8_t  CTRLBCLR;
  __IO uint8_t  CTRLBSET;
  __IO uint8_t  CTRLC;
       uint8_t  Reserved1[1];
  __IO uint8_t  DBGCTRL;
       uint8_t  Reserved2[1];
  __IO uint16_t EVCTRL;
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
  __I  uint8_t  STATUS;
  __IO uint8_t  COUNT;
       uint8_t  Reserved3[3];
  __IO uint8_t  PER;
       uint8_t  Reserved4[3];
  __IO uint8_t  CC0;
  __IO uint8_t  CC1;
} TC8Bits_Typedef;

typedef struct {
  __IO uint16_t CTRLA;
  __IO uint16_t READREQ;
  __IO uint8_t  CTRLBCLR;
  __IO uint8_t  CTRLBSET;
  __IO uint8_t  CTRLC;
       uint8_t  Reserved1[1];
  __IO uint8_t  DBGCTRL;
       uint8_t  Reserved2[1];
  __IO uint16_t EVCTRL;
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
  __I  uint8_t  STATUS;
  __IO uint16_t COUNT;
       uint16_t Reserved3[6];
  __IO uint16_t CC0;
  __IO uint16_t CC1;
} TC16Bits_Typedef;

typedef struct {
  __IO uint16_t CTRLA;
  __IO uint16_t READREQ;
  __IO uint8_t  CTRLBCLR;
  __IO uint8_t  CTRLBSET;
  __IO uint8_t  CTRLC;
       uint8_t  Reserved1[1];
  __IO uint8_t  DBGCTRL;
       uint8_t  Reserved2[1];
  __IO uint16_t EVCTRL;
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
  __I  uint8_t  STATUS;
  __IO uint32_t COUNT;
       uint8_t  Reserved3[4];
  __IO uint32_t CC0;
  __IO uint32_t CC1;
} TC32Bits_Typedef;

typedef struct {
  TC8Bits_Typedef   MODE8BIT;
  TC16Bits_Typedef  MODE16BIT;
  TC32Bits_Typedef  MODE32BIT;
} TC_Typedef;

typedef struct {
  __IO uint32_t CTRLA;
  __IO uint8_t  CTRLBCLR;
  __IO uint8_t  CTRLBSET;
       uint8_t  Reserved1[2];
  __I  uint32_t SYNCBUSY;
  __IO uint32_t FCTRLA;
  __IO uint32_t FCTRLB;
  __IO uint32_t WEXCTRL;
  __IO uint32_t DRVCTRL;
       uint8_t  Reserved2[2];
  __IO uint8_t  DBGCTRL;
       uint8_t  Reserved3[1];
  __IO uint32_t EVCTRL;
  __IO uint32_t INTENCLR;
  __IO uint32_t INTENSET;
  __IO uint32_t INTFLAG;
  __IO uint32_t STATUS;
  __IO uint32_t COUNT;
  __IO uint16_t PATT;
       uint8_t  Reserved4[2];
  __IO uint32_t WAVE;
  __IO uint32_t PER;
  __IO uint32_t CC0;
  __IO uint32_t CC1;
  __IO uint32_t CC2;
  __IO uint32_t CC3;
       uint8_t  Reserved5[16];
  __IO uint16_t PATTB;
       uint8_t  Reserved6[2];
  __IO uint32_t WAVEB;
  __IO uint32_t PERB;
  __IO uint32_t CCB0;
  __IO uint32_t CCB1;
  __IO uint32_t CCB2;
  __IO uint32_t CCB3;
} TCC_Typedef;

typedef struct {
  __IO uint8_t  CTRL;
  __IO uint8_t  CONFIG;
  __IO uint8_t  EWCTRL;
       uint8_t  Reserved1[1];
  __IO uint8_t  INTENCLR;
  __IO uint8_t  INTENSET;
  __IO uint8_t  INTFLAG;
  __I  uint8_t  STATUS;
  __O  uint8_t  CLEAR;
} WDT_Typedef;

#define FLASH_BASE  ((uint32_t)0x00000000U)  /*!< Embedded FLASH base address.          */
#define SRAM_BASE   ((uint32_t)0x20000000U)  /*!< Embedded SRAM base address.           */
#define PERIPH_BASE ((uint32_t)0x40000000U)  /*!< Peripheral base address.              */

/*!< Peripheral memory map  */
#define AHBAPB_BRIDGEA (PERIPH_BASE + 0x00000000)
#define AHBAPB_BRIDGEB (PERIPH_BASE + 0x01000000)
#define AHBAPB_BRIDGEC (PERIPH_BASE + 0x02000000)

/* AHB-APB Bridge A peripherals.  */
#define PAC0_BASE     (AHBAPB_BRIDGEA + 0x00000000)
#define PM_BASE       (AHBAPB_BRIDGEA + 0x00000400)
#define SYSCTRL_BASE  (AHBAPB_BRIDGEA + 0x00000800)
#define GCLK_BASE     (AHBAPB_BRIDGEA + 0x00000C00)
#define WDT_BASE      (AHBAPB_BRIDGEA + 0x00001000)
#define RTC_BASE      (AHBAPB_BRIDGEA + 0x00001400)
#define EIC_BASE      (AHBAPB_BRIDGEA + 0x00001800)

/* AHB-APB Bridge B peripherals.  */
#define PAC1_BASE     (AHBAPB_BRIDGEB + 0x00000000)
#define DSU_BASE      (AHBAPB_BRIDGEB + 0x00002000)
#define NVMCTRL_BASE  (AHBAPB_BRIDGEB + 0x00004000)
#define PORT_BASE     (AHBAPB_BRIDGEB + 0x00004400)
#define DMAC_BASE     (AHBAPB_BRIDGEB + 0x00004800)
#define MTB_BASE      (AHBAPB_BRIDGEB + 0x00006000)

/* AHB-APB Bridge C peripherals.  */
#define PAC2_BASE     (AHBAPB_BRIDGEC + 0x00000000)
#define EVSYS_BASE    (AHBAPB_BRIDGEC + 0x00000400)
#define SERCOM0_BASE  (AHBAPB_BRIDGEC + 0x00000800)
#define SERCOM1_BASE  (AHBAPB_BRIDGEC + 0x00000C00)
#define SERCOM2_BASE  (AHBAPB_BRIDGEC + 0x00001000)
#define TCC0_BASE     (AHBAPB_BRIDGEC + 0x00001400)
#define TC1_BASE      (AHBAPB_BRIDGEC + 0x00001800)
#define TC2_BASE      (AHBAPB_BRIDGEC + 0x00001C00)
#define ADC_BASE      (AHBAPB_BRIDGEC + 0x00002000)
#define AC_BASE       (AHBAPB_BRIDGEC + 0x00002400)
#define DAC_BASE      (AHBAPB_BRIDGEC + 0x00002800)
#define PTC_BASE      (AHBAPB_BRIDGEC + 0x00002C00)













//#define PAC0  ((PAC_Typedef *)  PAC0_BASE         )
#define PM      ((PM_Typedef      *)  PM_BASE     )
#define SYSCTRL ((SYSCRTL_Typedef *)  SYSCTRL_BASE)
#define GCLK    ((GCLK_Typedef    *)  GCLK_BASE   )
#define WDT     ((WDT_Typedef     *)  WDT_BASE    )
#define RTC     ((RTC_Typedef     *)  RTC_BASE    )
#define EIC     ((EIC_Typedef     *)  EIC_BASE    )

/* AHB-APB Bridge B peripherals.  */
//#define PAC1    ((PAC1_Typedef    *)  PAC1_BASE   )
#define DSU     ((DSU_Typedef     *) DSU_BASE     )
#define NVMCTRL ((NVMCTRL_Typedef *) NVMCTRL_BASE )
#define PORT    ((PORT_TypeDef    *) PORT_BASE    )
#define DMAC    ((DMAC_Typedef    *) DMAC_BASE    )
//#define MTB     ((MTB_TypeDef     *) MTB_BASE     )

/* AHB-APB Bridge C peripherals.  */
//#define PAC2    ((PCA2_Typedef    *) PAC2_BASE    )
#define EVSYS   ((EVSYS_Typedef   *) EVSYS_BASE   )
#define SERCOM0 ((SERCOM_Typedef  *) SERCOM0_BASE )
#define SERCOM1 ((SERCOM_Typedef  *) SERCOM1_BASE )
#define SERCOM2 ((SERCOM_Typedef  *) SERCOM2_BASE )
#define TCC0    ((TCC_Typedef     *) TCC0_BASE    )
#define TC1     ((TC_Typedef      *) TC1_BASE     )
#define TC2     ((TC_Typedef      *) TC2_BASE     )
#define ADC     ((ADC_Typedef     *) ADC_BASE     )
#define AC      ((AC_Typedef      *) AC_BASE      )
#define DAC     ((DAC_Typedef     *) DAC_BASE     )
#define PTC     ((PTC_Typedef     *) PTC_BASE     )

#define PORT_DIR_DIR_Pos           (0U)
#define PORT_DIR_DIR_Msk           (0xFFFFFFFF << PORT_DIR_DIR_Pos)
#define PORT_DIR_DIR               PORT_DIR_DIR_Msk

#define PORT_DIRCLR_DIRCLR_Pos     (0U)
#define PORT_DIRCLR_DIRCLR_Msk     (0xFFFFFFFF << PORT_DIRCLR_DIRCLR_Pos)
#define PORT_DIRCLR_DIRCLR         PORT_DIRCLR_DIRCLR_Msk

#define PORT_DIRSET_DIRSET_Pos     (0U)
#define PORT_DIRSET_DIRSET_Msk     (0xFFFFFFFF << PORT_DIRSET_DIRSET_Pos)
#define PORT_DIRSET_DIRSET         PORT_DIRSET_DIRSET_Msk

#define PORT_DIRTGL_DIRTGL_Pos     (0U)
#define PORT_DIRTGL_DIRTGL_Msk     (0xFFFFFFFF << PORT_DIRTGL_DIRTGL_Pos)
#define PORT_DIRTGL_DIRTGL         PORT_DIRTGL_DIRTGL_Msk

#define PORT_OUT_OUT_Pos           (0U)
#define PORT_OUT_OUT_Msk           (0xFFFFFFFF << PORT_OUT_OUT_Pos)
#define PORT_OUT_OUT               PORT_OUT_OUT_Msk

#define PORT_OUTCLR_OUTCLR_Pos     (0U)
#define PORT_OUTCLR_OUTCLR_Msk     (0xFFFFFFFF << PORT_OUTCLR_OUTCLR_Pos)
#define PORT_OUTCLR_OUTCLR         PORT_OUTCLR_OUTCLR_Msk

#define PORT_OUTSET_OUTSET_Pos     (0U)
#define PORT_OUTSET_OUTSET_Msk     (0xFFFFFFFF << PORT_OUTSET_OUTSET_Pos)
#define PORT_OUTSET_OUTSET         PORT_OUTSET_OUTSET_Msk

#define PORT_OUTTGL_OUTTGL_Pos     (0U)
#define PORT_OUTTGL_OUTTGL_Msk     (0xFFFFFFFF << PORT_OUTTGL_OUTTGL_Pos)
#define PORT_OUTTGL_OUTTGL         PORT_OUTTGL_OUTTGL_Msk

#define PORT_IN_IN_Pos             (0U)
#define PORT_IN_IN_Msk             (0xFFFFFFFF << PORT_IN_IN_Pos)
#define PORT_IN_IN                 PORT_IN_IN_Msk

#define PORT_CTRL_CTRL_Pos         (0U)
#define PORT_CTRL_CTRL_Msk         (0xFFFFFFFF << PORT_CTRL_CTRL_Pos)
#define PORT_CTRL_CTRL             PORT_CTRL_CTRL_Msk


#define PORT_WRCONFIG_HWSEL_Pos    (31U)
#define PORT_WRCONFIG_HWSEL_Msk    (0x1 << PORT_WRCONFIG_HWSEL_Pos)
#define PORT_WRCONFIG_HWSEL        PORT_WRCONFIG_HWSEL_Msk

#define PORT_WRCONFIG_WRPINCFG_Pos (30U)
#define PORT_WRCONFIG_WRPINCFG_Msk (0x1 << PORT_WRCONFIG_WRPINCFG_Pos)
#define PORT_WRCONFIG_WRPINCFG     PORT_WRCONFIG_WRPINCFG_Msk

#define PORT_WRCONFIG_WRPMUX_Pos   (28U)
#define PORT_WRCONFIG_WRPMUX_Msk   (0x1 << PORT_WRCONFIG_WRPMUX_Pos)
#define PORT_WRCONFIG_WRPMUX       PORT_WRCONFIG_WRPMUX_Msk

#define PORT_WRCONFIG_PMUX_Pos     (24U)
#define PORT_WRCONFIG_PMUX_Msk     (0xF << PORT_WRCONFIG_PMUX_Pos)
#define PORT_WRCONFIG_PMUX         PORT_WRCONFIG_PMUX_Msk

#define PORT_WRCONFIG_DRVSTR_Pos   (22U)
#define PORT_WRCONFIG_DRVSTR_Msk   (0x1 << PORT_WRCONFIG_DRVSTR_Pos)
#define PORT_WRCONFIG_DRVSTR       PORT_WRCONFIG_DRVSTR_Msk

#define PORT_WRCONFIG_PULLEN_Pos   (18U)
#define PORT_WRCONFIG_PULLEN_Msk   (0x1 << PORT_WRCONFIG_PULLEN_Pos)
#define PORT_WRCONFIG_PULLEN       PORT_WRCONFIG_PULLEN_Msk

#define PORT_WRCONFIG_INEN_Pos     (17U)
#define PORT_WRCONFIG_INEN_Msk     (0x1 << PORT_WRCONFIG_INEN_Pos)
#define PORT_WRCONFIG_INEN         PORT_WRCONFIG_INEN_Msk

#define PORT_WRCONFIG_PMUXEN_Pos   (16U)
#define PORT_WRCONFIG_PMUXEN_Msk   (0x1 << PORT_WRCONFIG_PMUXEN_Pos)
#define PORT_WRCONFIG_PMUXEN       PORT_WRCONFIG_PMUXEN_Msk

#define PORT_WRCONFIG_PINMAK_Pos   (0U)
#define PORT_WRCONFIG_PINMAK_Msk   (0xFFFF << PORT_WRCONFIG_PINMAK_Pos)
#define PORT_WRCONFIG_PINMAK       PORT_WRCONFIG_PINMAK_Msk


/*

#define PORT_PMUX0
#define PORT_PMUX1
#define PORT_PMUX2
#define PORT_PMUX3
#define PORT_PMUX4
#define PORT_PMUX5
#define PORT_PMUX6
#define PORT_PMUX7
#define PORT_PMUX8
#define PORT_PMUX9
#define PORT_PMUX10
#define PORT_PMUX11
#define PORT_PMUX12
#define PORT_PMUX13
#define PORT_PMUX14
#define PORT_PMUX15

#define PORT_PINCFG0
#define PORT_PINCFG1
#define PORT_PINCFG2
#define PORT_PINCFG3
#define PORT_PINCFG4
#define PORT_PINCFG5
#define PORT_PINCFG6
#define PORT_PINCFG7
#define PORT_PINCFG8
#define PORT_PINCFG9
#define PORT_PINCFG10
#define PORT_PINCFG11
#define PORT_PINCFG12
#define PORT_PINCFG13
#define PORT_PINCFG14
#define PORT_PINCFG15
#define PORT_PINCFG16
#define PORT_PINCFG17
#define PORT_PINCFG18
#define PORT_PINCFG19
#define PORT_PINCFG20
#define PORT_PINCFG21
#define PORT_PINCFG22
#define PORT_PINCFG23
#define PORT_PINCFG24
#define PORT_PINCFG25
#define PORT_PINCFG26
#define PORT_PINCFG27
#define PORT_PINCFG28
#define PORT_PINCFG29
#define PORT_PINCFG30
#define PORT_PINCFG31
*/

// Enable
#define WDT_CTRL_ENABLE_Pos        (1U)
#define WDT_CTRL_ENABLE_Msk        (0x01 << WDT_CTRL_ENABLE_Pos)
#define WDT_CTRL_ENABLE            WDT_CTRL_ENABLE_Msk

// Windows Enable
#define WDT_CTRL_WEN_Pos           (2U)
#define WDT_CTRL_WEN_Msk           (0x01 << WDT_CTRL_WEN_Pos)
#define WDT_CTRL_WEN               WDT_CTRL_WEN_Msk

// Always Enable
#define WDT_CTRL_Pos               (7U)
#define WDT_CTRL_Msk               (0x01 << WDT_CTRL_Pos)
#define WDT_CTRL                   WDT_CTRL_Msk


// Windows mode
#define WDT_CONFIG_WINDOW_Pos      (4U)
#define WDT_CONFIG_WINDOW_Msk      (0xF << WDT_CONFIG_WINDOW_Pos)
#define WDT_CONFIG_WINDOW          WDT_CONFIG_WINDOW_Msk

// Normal mode
#define WDT_CONFIG_PER_Pos         (0U)
#define WDT_CONFIG_PER_Msk         (0xF << WDT_CONFIG_PER_Pos)
#define WDT_CONFIG_PER             WDT_CONFIG_PER_Msk

#define WDT_EWCTRL_EWOFFSET_Pos    (0U)
#define WDT_EWCTRL_EWOFFSET_Msk    (0xF << WDT_EWCTRL_EWOFFSET_Pos)
#define WDT_EWCTRL_EWOFFSET        WDT_EWCTRL_EWOFFSET_Msk

#define WDT_INTENCRL_EW_Pos        (0U)
#define WDT_INTENCRL_EW_Msk        (0x1 << WDT_INTENCRL_EW_Pos)
#define WDT_INTENCRL_EW            WDT_INTENCRL_EW_Msk

#define WDT_INTENSET_EW_Pos        (0U)
#define WDT_INTENSET_EW_Msk        (0x1 << WDT_INTENSET_EW_Pos)
#define WDT_INTENSET_EW            WDT_INTENSET_EW_Msk

#define WDT_INTFLAG_EW_Pos         (0U)
#define WDT_INTFLAG_EW_Msk         (0x1 << WDT_INTFLAG_EW_Pos)
#define WDT_INTFLAG_EW             WDT_INTFLAG_EW_Msk

#define WDT_STATUS_SYNCBUSY_Pos    (7U)
#define WDT_STATUS_SYNCBUSY_Msk    (0x1 << WDT_STATUS_SYNCBUSY_Pos)
#define WDT_STATUS_SYNCBUSY        WDT_STATUS_SYNCBUSY_Msk

#define WDT_CLEAR_CLEAR_Pos        (7U)
#define WDT_CLEAR_CLEAR_Msk        (0x1 << WDT_CLEAR_CLEAR_Pos)
#define WDT_CLEAR_CLEAR            WDT_CLEAR_CLEAR_Msk


#define WDT_CLEAR_KEY                                  0xA5

#define WDT_WINDOW_MODE_TIMEOUT_8                      0x0
#define WDT_WINDOW_MODE_TIMEOUT_16                     0x1
#define WDT_WINDOW_MODE_TIMEOUT_32                     0x2
#define WDT_WINDOW_MODE_TIMEOUT_62                     0x3
#define WDT_WINDOW_MODE_TIMEOUT_128                    0x4
#define WDT_WINDOW_MODE_TIMEOUT_256                    0x5
#define WDT_WINDOW_MODE_TIMEOUT_512                    0x6
#define WDT_WINDOW_MODE_TIMEOUT_1K                     0x7
#define WDT_WINDOW_MODE_TIMEOUT_2K                     0x8
#define WDT_WINDOW_MODE_TIMEOUT_4K                     0x9
#define WDT_WINDOW_MODE_TIMEOUT_8K                     0xA
#define WDT_WINDOW_MODE_TIMEOUT_16K                    0xB

#define WDT_NORMAL_MODE_TIMEOUT_8                      0x0
#define WDT_NORMAL_MODE_TIMEOUT_16                     0x1
#define WDT_NORMAL_MODE_TIMEOUT_32                     0x2
#define WDT_NORMAL_MODE_TIMEOUT_64                     0x3
#define WDT_NORMAL_MODE_TIMEOUT_128                    0x4
#define WDT_NORMAL_MODE_TIMEOUT_256                    0x5
#define WDT_NORMAL_MODE_TIMEOUT_512                    0x6
#define WDT_NORMAL_MODE_TIMEOUT_1K                     0x7
#define WDT_NORMAL_MODE_TIMEOUT_2K                     0x8
#define WDT_NORMAL_MODE_TIMEOUT_4K                     0x9
#define WDT_NORMAL_MODE_TIMEOUT_8K                     0xA
#define WDT_NORMAL_MODE_TIMEOUT_16K                    0xB

#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_8      0x0
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_16     0x1
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_32     0x2
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_64     0x3
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_128    0x4
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_256    0x5
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_512    0x6
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_1K     0x7
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_2K     0x8
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_4K     0x9
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_8K     0xA
#define WDT_EARLY_WARNING_INTERRUPT_TIME_OFFSET_16K    0xB

#endif  /* SAMD10D14AM */