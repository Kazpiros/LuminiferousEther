/*
  (c) 2025 Microchip Technology Inc. and its subsidiaries

  Subject to your compliance with these terms, you may use this Microchip 
  software and any derivatives of this software. You must retain the above
  copyright notice with any redistribution of this software and the following
  disclaimers. It is your responsibility to comply with third party license
  terms applicable to your use of third party software (including open source
  software) that may accompany this Microchip software. THIS SOFTWARE IS
  SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR
  STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF 
  NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
  INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
  WHATSOEVER RELATED TO THIS SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
  BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
  FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS
  IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF
  ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*/

#ifndef MCP4XCXFX_SPI_H
#define MCP4XCXFX_SPI_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/*
 *   Section: Included Files
 */
#include <stdint.h>
#include <stdbool.h>
#include "driver/spi/drv_spi.h"
// In this example, the included header file "plib_sercom1_spi_master.h" provides the SPI support 
//#include "config/default/peripheral/sercom/spi_master/plib_sercom1_spi_master.h"
// or define the following data types and constants

/* Placeholder definition, if no SPI driver included */
//typedef uintptr_t DRV_SPI_HANDLE;
typedef DRV_HANDLE DRV_SPI_HANDLE;

/* Placeholder definition, if no SPI driver included */
//typedef uintptr_t DRV_SPI_TRANSFER_HANDLE;

/* Placeholder definition, if no SPI driver included */
//typedef struct
//{
//    /* Baud Rate or clock frequency */
//    uint32_t    baudRateInHz;
//    /* Clock Phase */
//    int32_t     clockPhase;
//    /* Clock Polarity */
//    int32_t     clockPolarity;
//    /* Number of bits per transfer */
//    int32_t     dataBits;
//    /* CS pin ID */
//    int32_t     chipSelect;
//    /* CS pin polarity */
//    int32_t     csPolarity;
//} DRV_SPI_TRANSFER_SETUP;

/* Placeholder definition, if no SPI driver included */
//#define DRV_SPI_HANDLE_INVALID  ((DRV_SPI_HANDLE)(-1))
#define DRV_SPI_HANDLE_INVALID          DRV_HANDLE_INVALID

/* Placeholder definition, if no SPI driver included */
//#define DRV_SPI_TRANSFER_HANDLE_INVALID ((DRV_SPI_TRANSFER_HANDLE)(-1))

/* Placeholder definition, if no SPI driver included */
//typedef enum
//{
//    /* Transfer request is pending */
//    DRV_SPI_TRANSFER_EVENT_PENDING = 0, 
//    /* All data were transfered successfully. */
//    DRV_SPI_TRANSFER_EVENT_COMPLETE = 1,
//    /* Transfer Handle given is expired. Transfer is completed but error ststus not known. */
//    DRV_SPI_TRANSFER_EVENT_HANDLE_EXPIRED = 2,
//    /* There was an error while processing transfer request. */
//    DRV_SPI_TRANSFER_EVENT_ERROR = -1,
//    /* Transfer Handle given is invalid */
//    DRV_SPI_TRANSFER_EVENT_HANDLE_INVALID = -2
//} DRV_SPI_TRANSFER_EVENT;

/** MCP4XCXFX_SPI_TRANSFER_EVENT enum type defines the SPI event identifiers returned 
 * by the SPI transfer functions or reported to the SPI transfer callback function 
 * (if supported by the driver).
 */
typedef enum
{
    /** Transfer request is pending */
    MCP4XCXFX_SPI_TRANSFER_EVENT_PENDING    = DRV_SPI_TRANSFER_EVENT_PENDING,
    /** All data from or to the buffer was transferred successfully. */
    MCP4XCXFX_SPI_TRANSFER_EVENT_COMPLETE   = DRV_SPI_TRANSFER_EVENT_COMPLETE,
    MCP4XCXFX_SPI_TRANSFER_EVENT_HANDLE_EXPIRED = DRV_SPI_TRANSFER_EVENT_HANDLE_EXPIRED,
    /** There was an error while processing the buffer transfer request. */
    MCP4XCXFX_SPI_TRANSFER_EVENT_ERROR      = DRV_SPI_TRANSFER_EVENT_ERROR,
    MCP4XCXFX_SPI_TRANSFER_EVENT_HANDLE_INVALID = DRV_SPI_TRANSFER_EVENT_HANDLE_INVALID,
    /* additional event codes */
    MCP4XCXFX_SPI_TRANSFER_NO_EVENT         = 100
} MCP4XCXFX_SPI_TRANSFER_EVENT, *MCP4XCXFX_SPI_TRANSFER_EVENT_P;

/* Placeholder definition, if no SPI driver included */
//typedef enum
//{
//    DRV_IO_INTENT_READ               = 1 << 0, /* Read */
//    DRV_IO_INTENT_WRITE              = 1 << 1, /* Write */
//    DRV_IO_INTENT_READWRITE          = DRV_IO_INTENT_READ|DRV_IO_INTENT_WRITE,   /* Read and Write*/
//    /* The driver will block and will return when the operation is complete */
//    DRV_IO_INTENT_BLOCKING           = 0 << 2,
//    /* The driver will return immediately */
//    DRV_IO_INTENT_NONBLOCKING        = 1 << 2,
//    /* The driver will support only one client at a time */
//    DRV_IO_INTENT_EXCLUSIVE          = 1 << 3,
//    /* The driver will support multiple clients at a time */
//    DRV_IO_INTENT_SHARED             = 0 << 3
//} DRV_SPI_IO_INTENT;


typedef DRV_SPI_HANDLE (*DRV_SPI_Open_P)(const SYS_MODULE_INDEX, const DRV_IO_INTENT);


typedef void (*DRV_SPI_WriteReadTransferAdd_P)(
                const DRV_SPI_HANDLE,               //driver handle
                void*,                              //transmit data
                size_t,                             //tx size
                void*,                              //receive data
                size_t,                             //rx size
                DRV_SPI_TRANSFER_HANDLE * const);   //transfer handle


typedef void (*DRV_SPI_WriteTransferAdd_P)(
                const   DRV_SPI_HANDLE,             //driver handle
                void*,                              //transmit data
                size_t,                             //tx size
                DRV_SPI_TRANSFER_HANDLE * const);   //transfer handle


typedef void (*DRV_SPI_ReadTransferAdd_P)(
                const   DRV_SPI_HANDLE,             //driver handle
                void*,                              //receive data
                size_t,                             //rx size
                DRV_SPI_TRANSFER_HANDLE * const);   //transfer handle

/** 
 * @brief Defines a pointer to a function that sets up an SPI transfer.
 *
 * This typedef defines a function pointer type for a function that sets up an SPI transfer.
 *
 * @param handle A handle to the SPI driver instance.
 * @param setup A pointer to a DRV_SPI_TRANSFER_SETUP structure that contains the transfer setup parameters.
 * @return Returns true if the setup is successful, false otherwise.
 */
typedef bool (*DRV_SPI_TransferSetup_P) ( const DRV_SPI_HANDLE handle, DRV_SPI_TRANSFER_SETUP* setup);

/** 
 * @brief Pointer type to the SPI transfer status query function.
 * 
 * The function implementation is platform specific. 
 * For example, it may return the last SPI event type.
 * 
 * @attention The typedef must be updated according to the platform SPI API
 */
typedef DRV_SPI_TRANSFER_EVENT (*DRV_SPI_TransferStatusGet_P)(const DRV_SPI_TRANSFER_HANDLE );

/** 
 * @brief SPI event callback function type.
 * 
 * The SPI event handler function is implemented by the MCP4XCXFX device library and is  
 * called by the SPI communication adaptation layer if the platform SPI driver 
 * supports the registration of callback event handlers.
 * The library event handler function is registered to the SPI adaptation layer by the 
 * call of the MCP4XCXFX_SPI_TransferEventHandlerSet() function.
 * Note: If the platform SPI driver has no callback support, the library checks the 
 * status of the SPI communication using MCP4XCXFX_SPI_TransferStatusGet() then calls the event handler.
 * @param event [in] - MCP4XCXFX_SPI_TRANSFER_EVENT type
 * @param context [in] - uintptr_t type pointer to the object registered by the library, 
 *                       returned back to the library by the event handler call.
 */
typedef void ( *MCP4XCXFX_SPI_TRANSFER_EVENT_HANDLER_P )( MCP4XCXFX_SPI_TRANSFER_EVENT event, uintptr_t context );

/* Placeholder definition, if no SPI driver included */
//typedef void (*DRV_SPI_TRANSFER_EVENT_HANDLER )( 
//                DRV_SPI_TRANSFER_EVENT event, 
//                DRV_SPI_TRANSFER_HANDLE transferHandle, 
//                uintptr_t context );

/** 
 * @brief Pointer type to the SPI transfer event callback registration function.
 * 
 * The function implementation is platform specific. 
 * For example, it may require as parameter the "SPI handle" returned by the "open" function.
 * 
 * @remark Some platforms may not implement support for SPI event callback, hence
 * they will have no callback registration function. This implies that the SPI communication 
 * is synchronous (the communication functions return when the communication 
 * is completed or in case of error). The MCP4XCXFX library can handle this scenario also.
 * 
 * @attention The typedef must be updated according to the platform SPI API
 */
typedef void (*DRV_SPI_TransferEventHandlerSet_P)(
                const DRV_SPI_HANDLE,
                const DRV_SPI_TRANSFER_EVENT_HANDLER,
                const uintptr_t);

typedef void (*SPI_CS_PinControl_P)(void);

/**
 * @struct _MCP4XCXFX_SPI_INIT
 * @brief _MCP4XCXFX_SPI_INIT structure type parameter for the MCP4XCXFX_SPI_Initialize() function.
 * 
 * The structure must contain the initialization values for various SPI attributes and members 
 * of the _MCP4XCXFX_SPI_CONTEXT structure.
 * - drvIndex:
 *      + SPI bus index. On some platforms it indicates which SPI bus/controller
 *        must be initialized and "open" for communication 
 *        (MCP4XCXFX_SPI_Initialize() calls the driver "open" function). 
 *        On other platforms it may not exist.
 *      + *This property is platform specific.*
 * - setup:
 *      + SPI communication parameters.
 *      + *This property is platform specific.*
 * - spiOpen:
 *      + Pointer to the spi driver "open" function. 
 *        The "Open" function is called by the MCP4XCXFX_SPI_Initialize().
 *      + It must be set to NULL if the platform spi driver does not implement the "Open" function. 
 *      + *This property is platform specific.*
 * - spiWriteRead:
 *      + Pointer to the spi driver "WriteRead" function. 
 *      + The platform spi "WriteRead" function must be supported.
 *      + *This property is platform specific.*
 * - spiWrite:
 *      + Pointer to the spi driver "Write" function. 
 *      + The platform spi "Write" function must be supported.
 *        It typically implements the spi "write" operations.
 *      + *This property is platform specific.*
 * - spiRead:
 *      + Pointer to the spi driver "Read" function. 
 *      + The platform spi "Read" function must be supported.
 *        It typically implements the spi "read" operations.
 *      + *This property is platform specific.*
 * - spiTransferSetup:
 *      + Pointer to the spi driver "TransferSetup" function. 
 *      + The platform spi "spiTransferSetup" function may not be supported.
 *        It must be set to NULL if the platform spi driver does not implement 
 *        the "TransferSetup" function.
 *      + *This property is platform specific.*
 * - spiTransferStatusGet:
 *      + Pointer to the spi transfer "StatusGet" function. 
 *      + If "StatusGet" function is not supported by the spi driver, 
 *        then it must support the event handler callback and the callback registration function. 
 *      + *This property is platform specific.*
 * - spiTransferCallbackSet:
 *      + Pointer to the spi transfer event "callback registration" function. 
 *      + If the "callback registration" function is not supported by the spi driver, 
 *        then it must support the spi "transfer status query" function. 
 *      + *This property is platform specific.*
 * - spiEnableCS:
 *      + Pointer to the spi chip select "Enable" function.
 *      + *This property is platform specific.*
 * - spiDisableCS:
 *      + Pointer to the spi chip select "Disable" function.
 *      + *This property is platform specific.*
*/
typedef struct _MCP4XCXFX_SPI_INIT{
    SYS_MODULE_INDEX                    drvIndex;               /**< SPI bus index. 
                                                                    @attention This property is platform specific. */
    DRV_SPI_TRANSFER_SETUP              setup;                  /**< SPI communication parameters. 
                                                                    @attention This property is platform specific. */
    //pointers to i2c driver functions
    DRV_SPI_Open_P                      spiOpen;                /**< Pointer to the SPI "Open" function, or NULL 
                                                                    @attention This property is platform specific. */
    DRV_SPI_WriteReadTransferAdd_P      spiWriteRead;           /**< Pointer to the SPI "WriteRead" function 
                                                                    @attention This property is platform specific. */
    DRV_SPI_WriteTransferAdd_P          spiWrite;               /**< Pointer to the SPI "Write" function 
                                                                    @attention This property is platform specific. */
    DRV_SPI_ReadTransferAdd_P           spiRead;                /**< Pointer to the SPI "Write" function 
                                                                    @attention This property is platform specific. */
    DRV_SPI_TransferSetup_P             spiTransferSetup;       /**< Pointer to the SPI "TransferSetup" function, or NULL 
                                                                    @attention This property is platform specific. */
    DRV_SPI_TransferStatusGet_P         spiTransferStatusGet;   /**< Pointer to the SPI "StatusGet" function, or NULL 
                                                                    @attention This property is platform specific. */
    DRV_SPI_TransferEventHandlerSet_P   spiTransferCallbackSet; /**< Pointer to the SPI transfer event callback "registration" function, or NULL 
                                                                    @attention This property is platform specific. */
    
    SPI_CS_PinControl_P                 spiEnableCS;            /**< Pointer to the SPI chip select "Enable" function 
                                                                    @attention This property is platform specific. */
    SPI_CS_PinControl_P                 spiDisableCS;           /**< Pointer to the SPI chip select "Disable" function 
                                                                    @attention This property is platform specific. */
}MCP4XCXFX_SPI_INIT, *MCP4XCXFX_SPI_INIT_P;

/** @struct _MCP4XCXFX_SPI_CONTEXT
 * @brief _MCP4XCXFX_SPI_CONTEXT structure type holds the SPI related attributes for one MCP4XCXFX device.
 * 
 * The structure is initialized by the MCP4XCXFX_SPI_Initialize() function using the configuration
 * values provisioned with MCP4XCXFX_SPI_INIT parameter.
 * - pspiEventHandler:
 *      + the pointer to the MCP4XCXFX library callback function for the spi transfer events,
 *        called by MCP4XCXFX_SPIDRV_EventHandler().
 *      + If the SPI driver does not support the events callback, 
 *        the events are polled and handled by the library directly. 
 * - pspiEventHandlerContext:
 *      + the pointer to the object registered by the MCP4XCXFX_SPI_TransferEventHandlerSet() call, 
 *        which is returned back to the library by the event handler callback.
 * - spiEventCallbackRegistered:
 *      + true if a callback function was registered (pspiEventHandler not NULL) for the spi transfer events,
 *        false otherwise.
 * - spiHandle:
 *      + The handle returned by the "Open" function, if implemented by the spi driver.
 *      + *This property is platform specific.*
 * - spiTransferHandle:
 *      + The handle returned by the calls to the spi driver transfer functions"WriteRead" and "Write". 
 *      + *This property is platform specific.*
 * - spiWriteRead: 
 *      + Pointer to the spi driver "WriteRead" function.
 *      + The platform spi "WriteRead" function must be supported.
 *      + *This property is platform dependent.*
 * - spiWrite:
 *      + Pointer to the spi driver "Write" function. 
 *      + The platform spi "Write" function must be supported. It typically implements the spi "write" operations.
 *      + *This property is platform dependent.* 
 * - spiRead:
 *      + Pointer to the spi driver "Read" function. 
 *      + The platform spi "Read" function must be supported. It typically implements the spi "read" operations.
 *      + *This property is platform dependent.* 
 * - spiTransferSetup:
 *      + Pointer to the spi driver "TransferSetup" function. 
 *      + The platform spi "spiTransferSetup" function may not be supported. 
 *        It must be set to NULL if the platform spi driver does not implement the "TransferSetup" function.
 *      + *This property is platform dependent.*
 * - spiTransferStatusGet:
 *      + Pointer to the spi transfer "StatusGet" function. 
 *      + If "StatusGet" function is not supported by the spi driver, 
 *        then it must support the event handler callback and the callback registration function. 
 *      + *This property is platform dependent.*
 * - spiTransferCallbackSet:
 *      + Pointer to the spi transfer event "callback registration" function. 
 *      + If the "callback registration" function is not supported by the spi driver, 
 *        then it must support the spi "transfer status query" function. 
 *      + *This property is platform dependent.*
 */
typedef struct _MCP4XCXFX_SPI_CONTEXT{
    MCP4XCXFX_SPI_TRANSFER_EVENT_HANDLER_P  pspiEventHandler;           /**< Callback function for the SPI transfer events */
    uintptr_t                               pspiEventHandlerContext;    /**< Context object returned to the callback function */
    bool                                    spiEventCallbackRegistered; /**< Indication that callback function is registered */
    //spi driver dependent handles
    DRV_SPI_HANDLE                          spiHandle;                  /**< The handle returned by the "Open" function 
                                                                            @attention This property is platform specific. */
    DRV_SPI_TRANSFER_HANDLE                 spiTransferHandle;          /**< SPI transfer handle returned by "WriteRead" and "Write" functions
                                                                            @attention This property is platform specific. */
    //pointers to i2c driver functions
    DRV_SPI_WriteReadTransferAdd_P          spiWriteRead;               /**< Pointer to the spi "WriteRead" function 
                                                                            @attention This property is platform specific. */
    DRV_SPI_WriteTransferAdd_P              spiWrite;                   /**< Pointer to the spi "Write" function 
                                                                            @attention This property is platform specific. */
    DRV_SPI_ReadTransferAdd_P               spiRead;                    /**< Pointer to the spi "Read" function 
                                                                            @attention This property is platform specific. */
    DRV_SPI_TransferSetup_P                 spiTransferSetup;           /**< Pointer to the spi "TransferSetup" function, or NULL 
                                                                            @attention This property is platform specific. */
    DRV_SPI_TransferStatusGet_P             spiTransferStatusGet;       /**< Pointer to the spi "StatusGet" function, or NULL 
                                                                            @attention This property is platform specific. */
    DRV_SPI_TransferEventHandlerSet_P       spiTransferCallbackSet;     /**< Pointer to the spi transfer event callback "registration" function, or NULL 
                                                                            @attention This property is platform specific. */
    
    SPI_CS_PinControl_P                     spiEnableCS;
    SPI_CS_PinControl_P                     spiDisableCS;  
}MCP4XCXFX_SPI_CONTEXT, *MCP4XCXFX_SPI_CONTEXT_P;

/**
 * @brief Sets up the SPI transfer for the MCP4XCXFX.
 *
 * This function configures the SPI transfer settings for the MCP4XCXFX device.
 * It checks if the transfer setup function pointer in the SPI context is not NULL,
 * and if so, it calls this function with the provided setup and clock parameters.
 * If the transfer setup function pointer is NULL, it returns false.
 *
 * @param[in] spi_context The SPI context containing the transfer setup function pointer.
 * @param[in] setup Pointer to the SPI transfer setup structure.
 *
 * @return true if the transfer setup function is called successfully, false otherwise.
 */
static bool MCP4XCXFX_spi_transfer_setup (const MCP4XCXFX_SPI_CONTEXT spi_context, DRV_SPI_TRANSFER_SETUP *setup){
    if(spi_context.spiTransferSetup != NULL){
        return spi_context.spiTransferSetup(spi_context.spiHandle ,setup);       
    }else{
        return false;
    }
}

/**
 * @brief Initializes the SPI context for the MCP4XCXFX device.
 *
 * This function sets up the SPI context for the MCP4XCXFX device using the 
 * provided initialization parameters. It configures the transfer setup, chip 
 * select (CS) control, serial data output (SDO) control, serial data input (SDI)
 * pull-up control, and SPI buffer read/write functions.
 *
 * @param[in] pspi_context Pointer to the MCP4XCXFX SPI context structure to be initialized.
 * @param[in] spi_init Structure containing the initialization parameters for the SPI context.
 *
 * @return true if the SPI transfer setup is successful, false otherwise.
 */
static inline bool MCP4XCXFX_SPI_Initialize( MCP4XCXFX_SPI_CONTEXT_P pspi_context, MCP4XCXFX_SPI_INIT spi_init){
    if( (pspi_context == NULL) ||
        (spi_init.spiWriteRead == NULL) ||
        (spi_init.spiWrite == NULL) ||
        (spi_init.spiRead == NULL) ) return false;
        
    // the spi driver must support at least one of the spi transfer event status query or event callback functions
    if( (spi_init.spiTransferStatusGet == NULL) && (spi_init.spiTransferCallbackSet == NULL) ) return false;
    
    // we must use CS pin control functions if the SPI interface does not control it
    if (( spi_init.spiEnableCS == NULL ) || (spi_init.spiDisableCS == NULL)) return false;
    
    // create a new spiHandle
    if(spi_init.spiOpen != NULL){
        pspi_context->spiHandle = spi_init.spiOpen(spi_init.drvIndex, 
                                                   DRV_IO_INTENT_READWRITE /*| DRV_IO_INTENT_NONBLOCKING | DRV_IO_INTENT_EXCLUSIVE */);
    }else{
        pspi_context->spiHandle = 0;
    }
    if(pspi_context->spiHandle == DRV_SPI_TRANSFER_HANDLE_INVALID){
        return false;
    }
    // or get it from the initialization parameters
    // pspi_context->spiHandle = spi_init.spiHandle;

    // set the pointers to i2c driver functions
    pspi_context->spiWriteRead           = spi_init.spiWriteRead;
    pspi_context->spiWrite               = spi_init.spiWrite;
    pspi_context->spiRead                = spi_init.spiRead;
    pspi_context->spiTransferSetup       = spi_init.spiTransferSetup;
    pspi_context->spiTransferStatusGet   = spi_init.spiTransferStatusGet;
    pspi_context->spiTransferCallbackSet = spi_init.spiTransferCallbackSet;
    
    //set the pointers to the CS pin control functions
    pspi_context->spiEnableCS            = spi_init.spiEnableCS;
    pspi_context->spiDisableCS           = spi_init.spiDisableCS;
    
    pspi_context->spiEventCallbackRegistered = false;
    
    if( false == MCP4XCXFX_spi_transfer_setup(*pspi_context, &spi_init.setup) ){
        return false;
    }
    return true;
}

/**
 * @brief Gets the current SPI transfer status for the specified SPI context.
 *
 * This function queries the platform-specific SPI driver for the current transfer status
 * using the function pointer in the SPI context. If the transfer is not pending, the chip select
 * (CS) is deactivated. If the function pointer is NULL, a "no event" status is returned.
 *
 * @param spi_context [in] - SPI context structure holding the SPI attributes for one SPI device.
 *
 * @return SPI transfer event status as defined by the platform SPI driver.
 */
inline MCP4XCXFX_SPI_TRANSFER_EVENT MCP4XCXFX_SPI_TransferStatusGet(MCP4XCXFX_SPI_CONTEXT spi_context){
    if (spi_context.spiTransferStatusGet != NULL){
        DRV_SPI_TRANSFER_EVENT spi_event;
        spi_event = spi_context.spiTransferStatusGet(spi_context.spiHandle);

        if (spi_event != DRV_SPI_TRANSFER_EVENT_PENDING) spi_context.spiDisableCS();
        
        return spi_event;
    }else{
        return MCP4XCXFX_SPI_TRANSFER_NO_EVENT;
    }
}

/**
 * @brief Performs a combined SPI write and read operation.
 *
 * This function initiates a write/read transfer using the platform-specific SPI driver.
 * The chip select (CS) is activated before the transfer and deactivated if the transfer handle is invalid.
 * The transfer handle is updated in the SPI context.
 *
 * @param pspi_context [in,out] - Pointer to the SPI context structure for the SPI device.
 * @param writeBuffer  [in]     - Pointer to the buffer containing data to write.
 * @param writeSize    [in]     - Number of bytes to write.
 * @param readBuffer   [out]    - Pointer to the buffer to store read data.
 * @param readSize     [in]     - Number of bytes to read.
 *
 * @retval true  Transfer was successfully initiated.
 * @retval false Transfer handle is invalid or transfer could not be started.
 */
static inline bool MCP4XCXFX_SPI_WriteRead (
    MCP4XCXFX_SPI_CONTEXT_P pspi_context,
    void* const writeBuffer,
    const size_t writeSize,
    void* const readBuffer,
    const size_t readSize)
{
    bool retcode = true;
    pspi_context->spiTransferHandle = DRV_SPI_TRANSFER_HANDLE_INVALID;

    pspi_context->spiEnableCS();
    
    (pspi_context->spiWriteRead)( pspi_context->spiHandle, 
                                  writeBuffer, writeSize, 
                                  readBuffer, readSize, 
                                  &(pspi_context->spiTransferHandle) );    
    if(pspi_context->spiTransferHandle == DRV_SPI_TRANSFER_HANDLE_INVALID ){
        retcode = false;
        
        pspi_context->spiDisableCS();
    }
    return retcode;
};

/**
 * @brief Performs an SPI write operation.
 *
 * This function initiates a write transfer using the platform-specific SPI driver.
 * The chip select (CS) is activated before the transfer and deactivated if the transfer handle is invalid.
 * The transfer handle is updated in the SPI context.
 *
 * @param pspi_context [in,out] - Pointer to the SPI context structure for the SPI device.
 * @param writeBuffer  [in]     - Pointer to the buffer containing data to write.
 * @param writeSize    [in]     - Number of bytes to write.
 *
 * @retval true  Transfer was successfully initiated.
 * @retval false Transfer handle is invalid or transfer could not be started.
 */
static inline bool MCP4XCXFX_SPI_Write(
    MCP4XCXFX_SPI_CONTEXT_P pspi_context,
    void* const writeBuffer,
    const size_t writeSize)
{
    bool retcode = true;
    pspi_context->spiTransferHandle = DRV_SPI_TRANSFER_HANDLE_INVALID;

    pspi_context->spiEnableCS();
    
    pspi_context->spiWrite( pspi_context->spiHandle, 
                            writeBuffer, 
                            writeSize, 
                            &(pspi_context->spiTransferHandle));
    if(pspi_context->spiTransferHandle == DRV_SPI_TRANSFER_HANDLE_INVALID){
        retcode = false;
        
        pspi_context->spiDisableCS();
    }
    return retcode;
}

/**
 * @brief Performs an SPI read operation.
 *
 * This function initiates a read transfer using the platform-specific SPI driver.
 * The chip select (CS) is activated before the transfer and deactivated if the transfer handle is invalid.
 * The transfer handle is updated in the SPI context.
 *
 * @param pspi_context [in,out] - Pointer to the SPI context structure for the SPI device.
 * @param readBuffer   [out]    - Pointer to the buffer to store read data.
 * @param readSize     [in]     - Number of bytes to read.
 *
 * @retval true  Transfer was successfully initiated.
 * @retval false Transfer handle is invalid or transfer could not be started.
 */
static inline bool MCP4XCXFX_SPI_Read(
    MCP4XCXFX_SPI_CONTEXT_P pspi_context,
    void* const readBuffer,
    const size_t readSize)
{
    bool retcode = true;
    pspi_context->spiTransferHandle = DRV_SPI_TRANSFER_HANDLE_INVALID;
    
    pspi_context->spiEnableCS();
    
    pspi_context->spiRead(pspi_context->spiHandle,
                          readBuffer, 
                          readSize,
                          &(pspi_context->spiTransferHandle));
    if(pspi_context->spiTransferHandle == DRV_SPI_TRANSFER_HANDLE_INVALID){
        retcode = false;
        
        pspi_context->spiDisableCS();
    }
    return retcode;
}

/**
 * @brief SPI platform driver specific, transfer event handler function
 * 
 * This function implements the platform specific SPI driver transfer event 
 * callback prototype (if supported). So, this is the callback function which gets 
 * registered by MCP4XCXFX_SPI_TransferEventHandlerSet() to the SPI driver,
 * as SPI event callback. 
 * This function process the received parameters and then calls the event callback
 * function implemented by the MCP4XCXFX library, which has a more generic API.
 * @param spi_event [in] - SPI event ID as defined by the platform SPI driver
 * @param spi_transferHandle [in] - spi transfer handle, as defined by the platform SPI driver
 * @param pspi_context [in] - transparent pointer to the MCP4XCXFX_SPI_CONTEXT structure 
 *                            holding the SPI attributes related to one SPI device.
 */
static inline void MCP4XCXFX_SPIDRV_EventHandler( DRV_SPI_TRANSFER_EVENT spi_event, 
                                                DRV_SPI_TRANSFER_HANDLE spi_transferHandle,         //pspi_context->transferHandle
                                                uintptr_t pspi_context ){                           //pspi_context

    if (spi_event != DRV_SPI_TRANSFER_EVENT_PENDING) ((MCP4XCXFX_SPI_CONTEXT_P)pspi_context)->spiDisableCS();
    
    (((MCP4XCXFX_SPI_CONTEXT_P)pspi_context)->pspiEventHandler)(spi_event, ((MCP4XCXFX_SPI_CONTEXT_P)pspi_context)->pspiEventHandlerContext);
}

/**
 * @brief Register the SPI transfer event callback function
 * 
 * This function uses the platform SPI driver specific API to register the 
 * MCP4XCXFX_SPIDRV_EventHandler() function as SPI event callback and uses the 
 * device SPI context structure to save the callback function implemented by 
 * the MCP4XCXFX library along with the callback function context parameter.
 * @param pspi_context [in,out] - pointer to the MCP4XCXFX_SPI_CONTEXT structure 
 *                            holding the SPI attributes related to one SPI device.
 * @param eventHandler [in] - pointer to the callback function implemented by 
 *                            the MCP4XCXFX library (MCP4XCXFX_SPI_TRANSFER_EVENT_HANDLER_P)
 * @param context [i] - transparent pointer to the context object which is returned 
 *                      back as parameter to the MCP4XCXFX event handler.
 * @return 
 *  - *true* - if the callback is successfully registered with the SPI driver
 *  - *false* - if the pspi_context->i2cHandle is invalid (MCP4XCXFX_SPI_HANDLE_INVALID)
 *  - *false* - if the SPI driver does not support event handler registration
 *    (pi2c_context->spiTransferCallbackSet is NULL)
 */
static inline bool MCP4XCXFX_SPI_TransferEventHandlerSet(
    const MCP4XCXFX_SPI_CONTEXT_P pspi_context,                     //&(pdevice->spi_context)
    const MCP4XCXFX_SPI_TRANSFER_EVENT_HANDLER_P eventHandler,      //MCP4XCXFX_SPIEventHandler()
    const uintptr_t context                                         //pdevice
){
    pspi_context->pspiEventHandler = eventHandler;
    pspi_context->pspiEventHandlerContext = context;
    if( (pspi_context->spiHandle != DRV_SPI_HANDLE_INVALID) &&
        (pspi_context->spiTransferCallbackSet != NULL)){
        pspi_context->spiTransferCallbackSet( pspi_context->spiHandle, 
                                                (DRV_SPI_TRANSFER_EVENT_HANDLER)MCP4XCXFX_SPIDRV_EventHandler, 
                                                (uintptr_t)pspi_context);
        return true;    //spi event callback registered with success
    }else{
        return false;   //spi event callback NOT registered
    }
}

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* MCP4XCXFX_SPI_H */
