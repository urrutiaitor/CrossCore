/*********************************************************************************

Copyright(c) 2011 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/
/*!
 * @file      adi_ad1871.c
 * @brief     AD1871 Stereo Audio ADC driver implementation.
 * @version:  $Revision: 11128 $
 * @date:     $Date: 2012-08-29 04:24:56 -0400 (Wed, 29 Aug 2012) $
 *
 * @details
 *            This is the primary source file for AD1871 Stereo Audio ADC driver.
 *
 */

#ifdef _MISRA_RULES
#pragma diag(push)
#pragma diag(suppress:misra_rule_2_4:"Allows doxygen addtogroup block")
#pragma diag(suppress:misra_rule_5_1:"Allow identifiers to be more than 31 characters")
#pragma diag(suppress:misra_rule_13_7:"Allow 'assert' function to validate memory size")
#pragma diag(suppress:misra_rule_14_6:"Allows multiple break statements")
#pragma diag(suppress:misra_rule_14_7:"Allows multiple exit points")
#endif

/** @addtogroup ADI_AD1871_LL_Driver
 *  @{

    AD1871 Stereo Audio ADC Low Level driver

The AD1871 driver assumes that the device operating mode is already
established by external hardware control. External control is enabled
by tying the XCTRL Pin (Pin 8) high. This configuration does not support
SPI access to AD1871 hardware registers.

The driver supports the following operating modes for AD1871 ADC.

- AD1871 as Master
- 256 x Fs MCLK (48 kHz sampling rate)
- 24 bit Word length
- I2S mode for audio data transfer

 */

/*=============  I N C L U D E S   =============*/

/* Include header file with definitions specific to AD1871 Stereo Audio ADC driver implementation */
#include "adi_ad1871_def.h"

/*==============  D E F I N E S  ===============*/

/* Transmit FS frequency in Hz (sampling rate = 48kHz) */
#define AD1871_RFS_FREQ         (48000u)
/* Transmit Clock freqency in Hz (FS * 32 clock per channel * 2 channels) */
#define AD1871_RCLK_FREQ        (3072000u)
/* 2 x Transmit Clock freqency in Hz */
#define AD1871_RCLK_FREQX2      (6144000u)

/*=============  D A T A  =============*/

/* Pointer to AD1871 device instance open */
static ADI_AD1871_DEV      *pAD1871LastDev = NULL;

/*=============  C O D E  =============*/

/*=============  L O C A L    F U N C T I O N S  =============*/

/*=============  C A L L B A C K   F U N C T I O N S =============*/

/*
 * Callback from SPORT
 *
 * Parameters
 *  - AppHandle   AD1871 instance to which the DMA belongs.
 *  - Event       DMA Event ID
 *  - pArg        Event specific argument
 *
 * Return
 *  None
 *
 */
static void SportCallback(
    void        *AppHandle,
    uint32_t    Event,
    void        *pArg)
{
    /* Pointer to AD1871 device instance to work on */
    ADI_AD1871_DEV *pDevice = (ADI_AD1871_DEV *)AppHandle;

    /* IF (we've a valid callback function) */
    if (pDevice->pfCallback)
    {
        /* IF (SPORT Buffer processed) */
        if (Event == (uint32_t) ADI_SPORT_EVENT_RX_BUFFER_PROCESSED)
        {
            /* Post callback to application reporting that a buffer has been processed
               Post the processed buffer address as event specific argument */
            (pDevice->pfCallback) (pDevice->pCBParam, (uint32_t) ADI_AD1871_EVENT_BUFFER_PROCESSED, pArg);
        }
        /* ELSE (DMA error) */
        else
        {
            /* Post callback to application reporting DMA error */
            (pDevice->pfCallback) (pDevice->pCBParam, (uint32_t) ADI_AD1871_EVENT_DMA_ERROR, NULL);
        }
    }
}

/*=============  D E B U G   F U N C T I O N S =============*/

/* IF (Debug mode enabled) */
#if defined (ADI_DEBUG)

/*
 * Validates AD1871 device handle.
 *
 * Parameters
 *  - [in]  hDevice    AD1871 handle to validate.
 *
 * Returns:  Status
 *  - ADI_AD1871_SUCCESS: Successfully validated AD1871 device handle.
 *  - ADI_AD1871_BAD_HANDLE: Invalid Device Handle.
 *
 */
static ADI_AD1871_RESULT ValidateHandle (ADI_AD1871_HANDLE hDevice)
{
    /* Pointer to the device instance to validate */
    ADI_AD1871_DEV     *pDeviceToValidate = (ADI_AD1871_DEV *) hDevice;
    /* Last open AD1871 instance in chain */
    ADI_AD1871_DEV     *pDeviceInChain;

    /* FOR (AD1871 instances in chain) */
    for (pDeviceInChain = pAD1871LastDev; pDeviceInChain != NULL; pDeviceInChain = pDeviceInChain->pPrevious)
    {
        /* IF (Supplied handle is valid) */
        if (pDeviceToValidate == pDeviceInChain)
        {
            /* This is a valid device */
            return (ADI_AD1871_SUCCESS);
        }
    }

    /* This device handle must be invalid */
    return (ADI_AD1871_BAD_HANDLE);
}

/*
 * Checks if the supplied AD1871 device number is already open.
 *
 * Parameters
 *  - [in]  DeviceNum    AD1871 device number to validate.
 *
 * Returns:  Status
 *  - ADI_AD1871_SUCCESS: Successfully validated AD1871 device number.
 *  - ADI_AD1871_IN_USE: Device number is already in use.
 *
 */
static ADI_AD1871_RESULT ValidateDevNumber (uint32_t    DeviceNum)
{
    /* AD1871 instance in chain */
    ADI_AD1871_DEV     *pDeviceInChain;

    /* FOR (All Open AD1871 instances in chain) */
    for (pDeviceInChain = pAD1871LastDev; pDeviceInChain != NULL; pDeviceInChain = pDeviceInChain->pPrevious)
    {
        /* IF (Supplied device number is already in use) */
        if (pDeviceInChain->DevNum == DeviceNum)
        {
            /* This device number is already in use*/
            return (ADI_AD1871_IN_USE);
        }
    }

    /* This device number is not in use */
    return (ADI_AD1871_SUCCESS);
}

/* End of debug functions */
#endif /* ADI_DEBUG */

/*=============  P U B L I C   F U N C T I O N S =============*/

/**
 * @brief Opens AD1871 device instance for use.
 *
 * @details     Opens the given AD1871 device instance and returns
 *              the handle to the opened device. The returned handle should
 *              be used in all the other APIs.
 *
 * @param [in]  DeviceNum       AD1871 instance number to open.
 * @param [in]  pDeviceMemory   Pointer to ADI_AD1871_REQ_MEMORY sized memory
 *                              location to handle the device instance.
 * @param [in]  MemorySize      Size of the buffer to which "pDeviceMemory" points.
 * @param [out] phDevice        Address where the AD1871 Device handle will be stored.
 *
 * @return  Status
 *          - #ADI_AD1871_SUCCESS: Successfully opened AD1871 instance.
 *          - #ADI_AD1871_IN_USE: Device number is already in use.
 *
 * @sa  adi_ad1871_Close()
 *
 */
ADI_AD1871_RESULT adi_ad1871_Open(
    uint32_t                            DeviceNum,
    void                        *const  pDeviceMemory,
    uint32_t                            MemorySize,
    ADI_AD1871_HANDLE           *const  phDevice)
{
    /* Pointer to the device instance memory to work on */
    ADI_AD1871_DEV     *pDevice = pDeviceMemory;

/* IF (Debug information enabled) */
#if defined (ADI_DEBUG)

    /* IF (This Device Number is already open) */
    if (ValidateDevNumber (DeviceNum) != ADI_AD1871_SUCCESS)
    {
        /* Report failure as device in use */
        return (ADI_AD1871_IN_USE);
    }

    /* Check if the given pointer parameters are valid */
    if((pDeviceMemory == NULL) || (phDevice == NULL))
    {
        return (ADI_AD1871_NULL_POINTER);
    }

    /* Check if the given memory is insufficient */
    if(MemorySize < ADI_AD1871_MEMORY_SIZE)
    {
        return (ADI_AD1871_INSUFFICIENT_MEMORY);
    }

    /* Asserts to validate AD1871 device memory size */
    assert(ADI_AD1871_MEMORY_SIZE >= sizeof(ADI_AD1871_DEV));

#endif /* ADI_DEBUG */

    /* Clear the given memory */
    memset(pDeviceMemory, 0, MemorySize);

    /* Initialize the device instance memory */
    pDevice->DevNum = DeviceNum;

    /* Protect this section of code - entering a critical region */
    adi_osal_EnterCriticalRegion();

    /* Add this instance to AD1871 device chain */
    pDevice->pPrevious = pAD1871LastDev;
    /* IF (This is not the first instance to be opened) */
    if (pAD1871LastDev != NULL)
    {
        /* Chain this device to the last instance in chain */
        pAD1871LastDev->pNext = pDevice;
    }
    /* Make this device as last instance in chain */
    pAD1871LastDev = pDevice;

    /* Exit the critical region */
    adi_osal_ExitCriticalRegion();

    /* Pass the physical device handle to client */
    *phDevice = (ADI_AD1871_HANDLE *)pDevice;

    /* Return */
    return(ADI_AD1871_SUCCESS);
}

/**
 * @brief Closes AD1871 instance.
 *
 * @details     Closes AD1871 driver instance and disables the data flow.
 *              Once the drive is closed, it is not allowed access any of the other
 *              APIs as the device handle will not be valid anymore.
 *
 * @param [in]  hDevice     AD1871 instance to close.
 *
 * @return  Status
 *          - #ADI_AD1871_SUCCESS: Successfully closed AD1871 instance.
 *          - #ADI_AD1871_BAD_HANDLE: Supplied Device handle is invalid.
 *          - #ADI_AD1871_SPORT_ERROR: SPORT Driver Error.
 *
 * @sa  adi_ad1871_Open()
 *
 */
ADI_AD1871_RESULT adi_ad1871_Close(
    ADI_AD1871_HANDLE           const   hDevice)
{
    /* Pointer to AD1871 device instance to work on */
    ADI_AD1871_DEV     *pDevice = (ADI_AD1871_DEV *) hDevice;
    /* Return code */
    ADI_AD1871_RESULT  eResult;

/* IF (Debug information enabled) */
#if defined (ADI_DEBUG)

    /* IF (AD1871 Handle is invalid) */
    if (ValidateHandle (hDevice) != ADI_AD1871_SUCCESS)
    {
        return (ADI_AD1871_BAD_HANDLE);
    }

#endif

    /* IF (AD1871 dataflow is enabled) */
    if(pDevice->bIsEnabled)
    {
        /* Disable AD1871 dataflow */
        eResult = adi_ad1871_Enable (hDevice, false);

        /* IF (Failed to disable AD1871 dataflow) */
        if (eResult != ADI_AD1871_SUCCESS)
        {
            return (eResult);
        }
    }

    /* IF (SPORT device is open) */
    if (pDevice->hSportDev)
    {
        /* Close SPORT Device */
        if (adi_sport_Close (pDevice->hSportDev) != ADI_SPORT_SUCCESS)
        {
            /* Report error */
            return (ADI_AD1871_SPORT_ERROR);
        }
    }

    /*-- Remove this device from AD1871 device in use chain --*/

    /* Protect this section of code - entering a critical region    */
    adi_osal_EnterCriticalRegion();

    /* IF (This is the last device instance in chain) */
    if (pAD1871LastDev == pDevice)
    {
        /* Make the previous instance linked to this device as last in chain */
        pAD1871LastDev = pDevice->pPrevious;
    }

    /* IF (This is not the first device in chain) */
    if (pDevice->pPrevious != NULL)
    {
        /* Remove this instance from the chain */
        pDevice->pPrevious->pNext = pDevice->pNext;
    }
    /* ELSE (This is the first device in chain) */
    else
    {
        /* IF (we've more than one device in chain) */
        if (pDevice->pNext != NULL)
        {
            /* Remove this instance from the chain */
            pDevice->pNext->pPrevious = NULL;
        }
    }

    /* Exit the critical region */
    adi_osal_ExitCriticalRegion();

    /* Return */
    return(ADI_AD1871_SUCCESS);
}

/**
 * @brief Sets SPORT Device number to communicate with AD1871 instance.
 *
 * @param [in]  hDevice             AD1871 instance to work on.
 * @param [in]  SportDevNum         SPORT Device number to use.
 * @param [in]  bAd1871Master      	TRUE to operate AD1871 as Master (source LRCLK and BCLK from AD1871, SPORT as slave)
 *                                  FALSE to operate AD1871 as Slave (source LRCLK and BCLK from SPORT, AD1871 as slave)
 *
 * @return  Status
 *          - #ADI_AD1871_SUCCESS: Successfully updated AD1871 settings.
 *          - #ADI_AD1871_BAD_HANDLE: Supplied Device handle is invalid.
 *          - #ADI_AD1871_ALREADY_RUNNING: Cannot perform this operation when dataflow is enabled.
 *          - #ADI_AD1871_SPORT_ERROR: SPORT Driver Error.
 *
 * @note    Slave mode should be used with caution. Processor SCLK should be configured such that the SCLK frequency
 * 			is a multiple of BCLK/LRCLK. This is required so that SPORT clock and FS divide register can be
 * 			configured to generate the exact BCLK and LRCLK (48kHz) from SCLK.
 *
 */
ADI_AD1871_RESULT  adi_ad1871_SetSportDevice(
    ADI_AD1871_HANDLE           const   hDevice,
    uint32_t                            SportDevNum,
    bool                                bAd1871Master)
{
    /* Control Register value */
    uint16_t            CtrlReg1, CtrlReg2;
    /* Clock and FS div */
    uint16_t            RClkDiv, RFsDiv;
    /* Location to hold SCLK freq */
    uint32_t            fsclk;
    /* Pointer to AD1871 device instance to work on */
    ADI_AD1871_DEV     *pDevice = (ADI_AD1871_DEV *) hDevice;

/* IF (Debug information enabled) */
#if defined (ADI_DEBUG)

    /* IF (AD1871 Handle is invalid) */
    if (ValidateHandle (hDevice) != ADI_AD1871_SUCCESS)
    {
        return (ADI_AD1871_BAD_HANDLE);
    }

#endif

    /* IF (AD1871 dataflow enabled) */
    if (pDevice->bIsEnabled)
    {
        /* Report failure as AD1871 is already running */
        return (ADI_AD1871_ALREADY_RUNNING);
    }

    do
    {
        /* IF (SPORT Device is already open) */
        if (pDevice->hSportDev)
        {
            /* Close the SPORT Device */
            if (adi_sport_Close(pDevice->hSportDev) != ADI_SPORT_SUCCESS)
            {
                break;
            }

            /* Clear the SPORT Device handle */
            pDevice->hSportDev = NULL;
        }

        /* Open the SPORT Device */
        if (adi_sport_Open (SportDevNum,
                            ADI_SPORT_DIR_RX,
                            ADI_SPORT_I2S_MODE,
                            &(pDevice->SportDevMem[0]),
                            ADI_SPORT_DMA_MEMORY_SIZE,
                            &(pDevice->hSportDev)) != ADI_SPORT_SUCCESS)
        {
            break;
        }

        /*-- Configure SPORT driver --*/

        /* IF (AD1871 as Master, SPORT as Slave)) */
        if (bAd1871Master)
        {
            /* SPORT Rx Control register 1 */
            CtrlReg1 = ADI_AD1871_SPORT_SLAVE_RX_CTRL1;
            /* SPORT Rx Control register 2 */
            CtrlReg2 = ADI_AD1871_SPORT_SLAVE_RX_CTRL2;
        }
        /* ELSE (SPORT is Master, AD1871 as Slave) */
        else
        {
        	/* SPORT Rx Control register 1 */
        	CtrlReg1 = ADI_AD1871_SPORT_MASTER_RX_CTRL1;
        	/* SPORT Rx Control register 2 */
        	CtrlReg2 = ADI_AD1871_SPORT_MASTER_RX_CTRL2;

            /* Get current SCLK */
            if ((uint32_t)adi_pwr_GetSystemFreq(&fsclk) != 0u)
            {
                /* Return error as failed to configure SPORT */
                break;
            }

            /* Calculate RCLKDIV */
            /* RSCLKx frequency = (SCLK frequency)/(2 x (SPORTx_RCLKDIV + 1)) */
            /* SPORTx_RCLKDIV = ((SCLK frequency) / (2 x RSCLKx frequency)) - 1 */
            RClkDiv = (uint16_t) ((fsclk / AD1871_RCLK_FREQX2) - 1u);

            /* Calculate RFSDIV */
            /* SPORTxRFS frequency = (RSCLKx frequency)/(SPORTx_RFSDIV + 1) */
            /* SPORTx_RFSDIV = ((RSCLKx frequency/SPORTxRFS) - 1) */
            RFsDiv = (uint16_t) ((AD1871_RCLK_FREQ / AD1871_RFS_FREQ) - 1u);

            /* Configure SPORTx_RCLKDIV register - Internal */
            if (adi_sport_ConfigClock (pDevice->hSportDev, RClkDiv, true, true))
            {
                break;
            }

            /* Configure SPORTx_RFSDIV register - Internal */
            if (adi_sport_ConfigFrameSync (pDevice->hSportDev, RFsDiv, true, true, true, false, false))
            {
                break;
            }
        }

        /* Configure SPORT Rx Control register 1 */
        if (adi_sport_SetControlReg (pDevice->hSportDev, ADI_SPORT_CONTROL_RX_REGISTER, CtrlReg1) != ADI_SPORT_SUCCESS)
        {
            break;
        }

        /* Configure SPORT Rx Control register 2 */
        if (adi_sport_SetControlReg (pDevice->hSportDev, ADI_SPORT_CONTROL_RX_REGISTER2, CtrlReg2) != ADI_SPORT_SUCCESS)
        {
            break;
        }

        /* IF (we've a valid callback function) */
        if (pDevice->pfCallback != NULL)
        {
            /* Pass a valid callback function to SPORT driver */
            if (adi_sport_RegisterCallback (pDevice->hSportDev, SportCallback, pDevice) != ADI_SPORT_SUCCESS)
            {
                break;
            }
        }

        /* Return Success */
        return (ADI_AD1871_SUCCESS);

    } while (0);

    /* Return Failure */
    return (ADI_AD1871_SPORT_ERROR);
}

/**
 * @brief Submits a buffer to AD1871 instance to fill with received audio data.
 *
 * @param [in]  hDevice         AD1871 instance to work on.
 * @param [in]  pBuffer         Pointer to audio buffer to fill.
 * @param [in]  BufferSize      Audio buffer size in bytes.
 *
 * @return  Status
 *          - #ADI_AD1871_SUCCESS: Successfully submitted audio buffer.
 *          - #ADI_AD1871_BAD_HANDLE: Supplied Device handle is invalid.
 *          - #ADI_AD1871_SPORT_DEVICE_NUM_NOT_SET: SPORT Device number is not yet set.
 *          - #ADI_AD1871_BUF_INVALID: Supplied audio buffer address and/or size invalid.
 *          - #ADI_AD1871_BUF_SUBMIT_FAILED: Failed to submit audio buffer.
 *
 */
/* Submits a buffer to AD1871 instance to fill with received audio data */
ADI_AD1871_RESULT  adi_ad1871_SubmitRxBuffer(
    ADI_AD1871_HANDLE           const   hDevice,
    void                                *pBuffer,
    uint32_t                            BufferSize)
{
    /* Pointer to AD1871 device instance to work on */
    ADI_AD1871_DEV     *pDevice = (ADI_AD1871_DEV *) hDevice;

/* IF (Debug information enabled) */
#if defined (ADI_DEBUG)

    /* IF (AD1871 Handle is invalid) */
    if (ValidateHandle (hDevice) != ADI_AD1871_SUCCESS)
    {
        return (ADI_AD1871_BAD_HANDLE);
    }

    /* IF (SPORT handle is invalid) */
    if (pDevice->hSportDev == NULL)
    {
        /* Report error as AD1871 SPORT device number is not configured */
        return (ADI_AD1871_SPORT_DEVICE_NUM_NOT_SET);
    }

    /* IF (Buffer address is invalid) */
    if (pBuffer == NULL)
    {
        return (ADI_AD1871_BUF_ADDR_INVALID);
    }

#endif

    /* Submit buffer to SPORT */
    if (adi_sport_SubmitBuffer (pDevice->hSportDev, pBuffer, BufferSize) != ADI_SPORT_SUCCESS)
    {
        /* Report error */
        return (ADI_AD1871_BUF_SUBMIT_FAILED);
    }
    else
    {
        /* Return success */
        return (ADI_AD1871_SUCCESS);
    }
}

/**
 * @brief Peek function for non-OS application to check if a processed audio buffer is available or not.
 *
 * @param [in]  hDevice             AD1871 instance to query.
 * @param [out] pbIsBufAvailable    Pointer to a location to store if a processed buffer is available (TRUE) or not (FALSE).
 *
 * @return  Status
 *          - #ADI_AD1871_SUCCESS: Successfully queried processed audio buffer availability.
 *          - #ADI_AD1871_BAD_HANDLE: Supplied Device handle is invalid.
 *          - #ADI_AD1871_SPORT_ERROR: SPORT Driver Error.
 *
 */
ADI_AD1871_RESULT  adi_ad1871_IsRxBufAvailable(
    ADI_AD1871_HANDLE           const   hDevice,
    bool                                *pbIsBufAvailable)
{
    /* Pointer to AD1871 device instance to work on */
    ADI_AD1871_DEV     *pDevice = (ADI_AD1871_DEV *) hDevice;

/* IF (Debug information enabled) */
#if defined (ADI_DEBUG)

    /* IF (AD1871 Handle is invalid) */
    if (ValidateHandle (hDevice) != ADI_AD1871_SUCCESS)
    {
        return (ADI_AD1871_BAD_HANDLE);
    }

#endif

    /* IF (Successfully queried SPORT driver for buffer available status) */
    if (adi_sport_IsBufferAvailable (pDevice->hSportDev, pbIsBufAvailable) == ADI_SPORT_SUCCESS)
    {
        /* Return success */
        return (ADI_AD1871_SUCCESS);
    }
    /* ELSE (Failed to query SPORT driver for buffer available status) */
    else
    {
        /* Return failure */
        return (ADI_AD1871_SPORT_ERROR);
    }
}

/**
 * @brief Gets address of a processed audio buffer.
 *
 * @param [in]  hDevice    AD1871 instance to query.
 * @param [out] ppBuffer   Location to store address of a processed audio buffer.
 *
 * @return  Status
 *          - #ADI_AD1871_SUCCESS: Successfully acquired a processed audio buffer address.
 *          - #ADI_AD1871_BAD_HANDLE: Supplied Device handle is invalid.
 *          - #ADI_AD1871_SPORT_ERROR: SPORT Driver Error.
 *
 */
ADI_AD1871_RESULT  adi_ad1871_GetRxBuffer(
    ADI_AD1871_HANDLE           const   hDevice,
    void                                **ppBuffer)
{
    /* Pointer to AD1871 device instance to work on */
    ADI_AD1871_DEV     *pDevice = (ADI_AD1871_DEV *) hDevice;

/* IF (Debug information enabled) */
#if defined (ADI_DEBUG)

    /* IF (AD1871 Handle is invalid) */
    if (ValidateHandle (hDevice) != ADI_AD1871_SUCCESS)
    {
        return (ADI_AD1871_BAD_HANDLE);
    }

#endif

    /* IF (Successfully queried SPORT driver for a buffer address) */
    if (adi_sport_GetBuffer (pDevice->hSportDev, ppBuffer) == ADI_SPORT_SUCCESS)
    {
        /* Return success */
        return (ADI_AD1871_SUCCESS);
    }
    /* ELSE (Failed to query SPORT driver for a buffer address) */
    else
    {
        /* Return failure */
        return (ADI_AD1871_SPORT_ERROR);
    }
}

/**
 * @brief Set callback function to report AD1871 specific events.
 *
 * @param [in]  hDevice     AD1871 instance work on.
 * @param [in]  pfCallback  Address of application callback function.
 * @param [in]  pCBParam    Parameter passed back to application callback.
 *
 * @return  Status
 *          - #ADI_AD1871_SUCCESS: Successfully set AD1871 callback function.
 *          - #ADI_AD1871_BAD_HANDLE: Supplied Device handle is invalid.
 *          - #ADI_AD1871_SPORT_ERROR: SPORT Driver Error.
 *          - #ADI_AD1871_ALREADY_RUNNING: Cannot perform this operation when dataflow is enabled.
 *
 */
ADI_AD1871_RESULT  adi_ad1871_SetCallback(
    ADI_AD1871_HANDLE           const   hDevice,
    ADI_CALLBACK                        pfCallback,
    void                                *pCBParam)
{
    /* Pointer to AD1871 device instance to work on */
    ADI_AD1871_DEV     *pDevice = (ADI_AD1871_DEV *) hDevice;

/* IF (Debug information enabled) */
#if defined (ADI_DEBUG)

    /* IF (AD1871 Handle is invalid) */
    if (ValidateHandle (hDevice) != ADI_AD1871_SUCCESS)
    {
        return (ADI_AD1871_BAD_HANDLE);
    }

#endif

    /* IF (AD1871 dataflow enabled) */
    if (pDevice->bIsEnabled)
    {
        /* Report failure as AD1871 is already running */
        return (ADI_AD1871_ALREADY_RUNNING);
    }

    /* Save the supplied callback information */
    pDevice->pfCallback = pfCallback;
    pDevice->pCBParam   = pCBParam;

    /* IF (SPORT device is open) */
    if (pDevice->hSportDev != NULL)
    {
        /* IF (we've a valid callback function) */
        if (pDevice->pfCallback != NULL)
        {
            /* Pass a valid callback function to SPORT driver */
            if (adi_sport_RegisterCallback (pDevice->hSportDev, SportCallback, pDevice) != ADI_SPORT_SUCCESS)
            {
                /* Return failure */
                return (ADI_AD1871_SPORT_ERROR);
            }
        }
        /* ELSE (Un-register SPORT Callback) */
        else
        {
            /* Un-register SPORT Callback */
            if (adi_sport_RegisterCallback (pDevice->hSportDev, NULL, NULL) != ADI_SPORT_SUCCESS)
            {
                /* Return failure */
                return (ADI_AD1871_SPORT_ERROR);
            }
        }
    }

    /* Return */
    return (ADI_AD1871_SUCCESS);
}

/**
 * @brief Hard-resets AD1871.
 *
 * @details Resets the AD1871 device instance via hardware by toggling the
 *          GPIO pin connected to AD1871 device.
 *
 * @param [in]  hDevice    AD1871 instance to work on.
 * @param [in]  bEnable    TRUE to enable AD1871 dataflow, FALSE to disable.
 *
 * @return  Status
 *          - #ADI_AD1871_SUCCESS: Successfully updated AD1871 dataflow.
 *          - #ADI_AD1871_BAD_HANDLE: Supplied Device handle is invalid.
 *          - #ADI_AD1871_GPIO_ERROR: GPIO configuration error.
 *
 */
ADI_AD1871_RESULT  adi_ad1871_HwReset(
    ADI_AD1871_HANDLE           const   hDevice,
    ADI_GPIO_PORT               const   ePort,
    uint32_t                    const   Pin)
{
    /* Pointer to AD1871 device instance to work on */
    ADI_AD1871_DEV      *pDevice = (ADI_AD1871_DEV *) hDevice;
    /* Loop variable */
    volatile uint32_t   i;

/* IF (Debug information enabled) */
#if defined (ADI_DEBUG)

    /* IF (AD1871 Handle is invalid) */
    if (ValidateHandle (hDevice) != ADI_AD1871_SUCCESS)
    {
        return (ADI_AD1871_BAD_HANDLE);
    }

#endif

    /* IF (AD1871 dataflow enabled) */
    if (pDevice->bIsEnabled)
    {
        /* Report failure as AD1871 is already running */
        return (ADI_AD1871_ALREADY_RUNNING);
    }

    /* Set the GPIO direction */
    if (adi_gpio_SetDirection (ePort, Pin, ADI_GPIO_DIRECTION_OUTPUT))
    {
        /* Failed to configure GPIO, return failure */
        return (ADI_AD1871_GPIO_ERROR);
    }

    /* De-assert reset signal */
    if (adi_gpio_Clear (ePort, Pin))
    {
        /* Failed to configure GPIO, return failure */
        return (ADI_AD1871_GPIO_ERROR);
    }

    /* Provide some delay to reset the device */
    for (i = 0xFFFFFu; i > 0u; i--)
    {
    	;
    }

    /* Assert reset signal */
    if (adi_gpio_Set (ePort, Pin))
    {
        /* Failed to configure GPIO, return failure */
        return (ADI_AD1871_GPIO_ERROR);
    }

    /* Return success */
    return (ADI_AD1871_SUCCESS);
}

/**
 * @brief Enable/Disable AD1871 dataflow.
 *
 * @param [in]  hDevice    AD1871 instance to work on.
 * @param [in]  bEnable    TRUE to enable AD1871 dataflow, FALSE to disable.
 *
 * @return  Status
 *          - #ADI_AD1871_SUCCESS: Successfully updated AD1871 dataflow.
 *          - #ADI_AD1871_BAD_HANDLE: Supplied Device handle is invalid.
 *          - #ADI_AD1871_SPORT_DEVICE_NUM_NOT_SET: SPORT Device number is not yet set.
 *
 */
ADI_AD1871_RESULT  adi_ad1871_Enable(
    ADI_AD1871_HANDLE           const   hDevice,
    bool                                bEnable)
{
    /* Pointer to AD1871 device instance to work on */
    ADI_AD1871_DEV     *pDevice = (ADI_AD1871_DEV *) hDevice;

/* IF (Debug information enabled) */
#if defined (ADI_DEBUG)

    /* IF (AD1871 Handle is invalid) */
    if (ValidateHandle (hDevice) != ADI_AD1871_SUCCESS)
    {
        return (ADI_AD1871_BAD_HANDLE);
    }

#endif

    /* IF (No change in dataflow status) */
    if (pDevice->bIsEnabled == bEnable)
    {
        /* Report success as AD1871 is already running */
        return (ADI_AD1871_SUCCESS);
    }

    /* IF (we've a valid SPORT device handle) */
    if (pDevice->hSportDev)
    {
        /* Update SPORT dataflow status */
        if (adi_sport_Enable (pDevice->hSportDev, bEnable) != ADI_SPORT_SUCCESS)
        {
            /* Return failure */
            return (ADI_AD1871_SPORT_ERROR);
        }

        /* Update dataflow status */
        pDevice->bIsEnabled = bEnable;
    }
    /* ELSE (SPORT device not open yet) */
    else
    {
        /* IF (Enable dataflow) */
        if (bEnable)
        {
            /* Return error as we can't start dataflow */
            return (ADI_AD1871_SPORT_DEVICE_NUM_NOT_SET);
        }
    }

    /* Return success */
    return (ADI_AD1871_SUCCESS);
}

/*****/

/*@}*/

#ifdef _MISRA_RULES
#pragma diag(pop)
#endif /* _MISRA_RULES */


