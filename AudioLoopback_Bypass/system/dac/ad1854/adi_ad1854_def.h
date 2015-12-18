/*********************************************************************************

Copyright(c) 2011 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/

/*!
 * @file:    adi_ad1854_def.h
 * @brief:   Private header file for AD1854 driver implementation.
 * @version: $Revision: 11128 $
 * @date:    $Date: 2012-08-29 04:24:56 -0400 (Wed, 29 Aug 2012) $
 *
 * @details
 *           This is a private header file for AD1854 Stereo Audio DAC driver
 *           that contains definitions used in AD1854 driver implementation.
 */

#ifndef __ADI_AD1854_DEF_H__
#define __ADI_AD1854_DEF_H__

/*=============  I N C L U D E S   =============*/

/* ADI AD1854 driver includes */
#include <drivers/dac/ad1854/adi_ad1854.h>
/* NULL definition includes */
#include <string.h>
/* Memory size check */
#include <assert.h>

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_5_1:"Allow identifiers to be more than 31 characters")
#pragma diag(suppress:misra_rule_5_3:"Structure name had to defined earlier for function prototype")
#pragma diag(suppress:misra_rule_5_4:"Structure name had to defined earlier for function prototype")
#endif

/*==============  D E F I N E S  ===============*/

/*
 * SPORT Control register 1 value for AD1854 as Master
 * External Clock & FS, Zero Fill, Receive MSB first, Receive FS required,
 * Active Low FS, Early FS, Sample data and FS with raising edge of RSCLK
 */
#define ADI_AD1854_SPORT_SLAVE_TX_CTRL1    (0x5400u)

/*
 * SPORT Control register 2 value for AD1854 as Master
 * 24-bit wordlength, Secondary disabled, FS becomes L/R Clock, Left channel first
 */
#define ADI_AD1854_SPORT_SLAVE_TX_CTRL2    (0x0217u)

/*
 * SPORT Control register 1 value for AD1854 as Slave
 * Internal Clock & FS, Zero Fill, Receive MSB first, Receive FS required,
 * Active Low FS, Early FS, Drive FS on raising edge of RSCLK
 */
#define ADI_AD1854_SPORT_MASTER_TX_CTRL1     (0x1602u)

/*
 * SPORT Control register 2 value for AD1854 as Master
 * 24-bit wordlength, Secondary disabled, FS becomes L/R Clock, Left channel first
 */
#define ADI_AD1854_SPORT_MASTER_TX_CTRL2     (0x0217u)

#pragma pack (4)
/* Structure to handle AD1854 device instance */
typedef struct ADI_AD1854_DEV
{
    uint8_t                     SportDevMem[ADI_SPORT_DMA_MEMORY_SIZE]; /* Memory to handle a DMA driven SPORT Device */
    bool                        bIsEnabled;                             /* TRUE if AD1854 dataflow is enabled */
    bool                        bIsMaster;                              /* TRUE if AD1854 is the master */
    uint32_t                    DevNum;                              	/* Device instance number */
    ADI_SPORT_HANDLE            hSportDev;                              /* SPORT Device Handle */
    ADI_CALLBACK                pfCallback;                             /* Address of application callback function */
    void                        *pCBParam;                              /* Parameter passed back to application callback */
    struct ADI_AD1854_DEV       *pPrevious;                             /* Pointer to previous device instance in chain */
    struct ADI_AD1854_DEV       *pNext;                                 /* Pointer to next device instance in chain */
} ADI_AD1854_DEV;
#pragma pack ()

/*=============  C A L L B A C K    F U N C T I O N    P R O T O T Y P E S  =============*/

/* Callback from SPORT */
static void SportCallback(
    void                    *AppHandle,
    uint32_t                Event,
    void                    *pArg);

/*=============  D E B U G    F U N C T I O N    P R O T O T Y P E S  =============*/
/*
 * Debug Functions
 */
/* IF (Debug information enabled) */
#if defined (ADI_DEBUG)

/* Validates AD1854 device handle */
static ADI_AD1854_RESULT ValidateHandle (ADI_AD1854_HANDLE hDevice);
/* Checks if the supplied AD1854 device number is already open */
static ADI_AD1854_RESULT ValidateDevNumber (uint32_t DeviceNum);

/* End of debug functions */
#endif /* ADI_DEBUG */

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */

#endif  /* __ADI_AD1854_DEF_H__ */

/*****/

