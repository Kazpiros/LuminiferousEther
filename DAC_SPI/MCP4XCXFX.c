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

#include "MCP4XCXFX.h"

void MCP4XCXFX_I2CEventHandler(MCP4XCXFX_I2C_TRANSFER_EVENT event,  uintptr_t context);

void MCP4XCXFX_SPIEventHandler(MCP4XCXFX_SPI_TRANSFER_EVENT event,  uintptr_t context);

static int16_t MCP4XCXFX_Get_Register(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, 
                                    void* pregister_val, uint8_t registerAddr, size_t regSize, 
                                    MCP4XCXFX_procState processingState);

static int16_t MCP4XCXFX_Set_Register(MCP4XCXFX_DEVICE_CONTEXT_P pdevice,
                                        uint8_t *pregisterBytes, uint8_t registerAddr,
                                        size_t regSize, MCP4XCXFX_procState processingState);


static void MCP4XCXFX_GetVrefCtrl_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);
static void MCP4XCXFX_GetPwrDownCtrl_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);
static void MCP4XCXFX_GetGainCtrl_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);
static void MCP4XCXFX_GetGainCtrl_NV_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);
static void MCP4XCXFX_SetGainCtrl_NV_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);
static void MCP4XCXFX_GetWiperlock_NV_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);

static void MCP4XCXFX_Get_Reg16bitProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);

static void MCP4XCXFX_GetI2Caddr_NV_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);
static void MCP4XCXFX_GetPorStatus_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);
static void MCP4XCXFX_GetNVaccessStatus_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);


///////////////////////////////////////////////////////////////////////////////

/*
 * Library functions, target platform independent 
 */

int16_t MCP4XCXFX_AbortRequest(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    if (pdevice != NULL){
        pdevice->ABORT_REQUESTED_FLAG = true;
        return MCP4XCXFX_SUCCESS;
    }else{
        return MCP4XCXFX_INVALID_PARAMETER;
    }
}

static void inline callUserCallback(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    // call the user call-back if there is one registered
    if(pdevice->userCallback != NULL){
        MCP4XCXFX_EVENT_HANDLER userCallback;
        MCP4XCXFX_EVENT event;
        uintptr_t userContext; 
        
        userCallback = pdevice->userCallback;
        event = pdevice->deviceEventStatus;
        userContext = pdevice->userContext;
        userCallback(event, userContext);
    }
}

//pdevice_context is the device context structure, MCP4XCXFX_PDEVICE_CONTEXT
void MCP4XCXFX_I2CEventHandler(MCP4XCXFX_I2C_TRANSFER_EVENT event, uintptr_t pdevice_context)
{
    if(pdevice_context == 0){
        return;
    }
    switch(event){
        case MCP4XCXFX_I2C_TRANSFER_EVENT_PENDING:
            ((MCP4XCXFX_DEVICE_CONTEXT_P)pdevice_context)->if_status = MCP4XCXFX_INTERFACE_EVENT_PENDING;
            break;
        case MCP4XCXFX_I2C_TRANSFER_EVENT_COMPLETE:
            ((MCP4XCXFX_DEVICE_CONTEXT_P)pdevice_context)->if_status = MCP4XCXFX_INTERFACE_EVENT_COMPLETE;            
            break;
        case MCP4XCXFX_I2C_TRANSFER_NO_EVENT:
            ((MCP4XCXFX_DEVICE_CONTEXT_P)pdevice_context)->if_status = MCP4XCXFX_INTERFACE_NO_EVENT;            
            break;
        case MCP4XCXFX_I2C_TRANSFER_EVENT_HANDLE_EXPIRED:
        case MCP4XCXFX_I2C_TRANSFER_EVENT_ERROR:
        case MCP4XCXFX_I2C_TRANSFER_EVENT_HANDLE_INVALID:
        default:
            ((MCP4XCXFX_DEVICE_CONTEXT_P)pdevice_context)->if_status = MCP4XCXFX_INTERFACE_EVENT_ERROR;
    }
}

 void MCP4XCXFX_SPIEventHandler(MCP4XCXFX_SPI_TRANSFER_EVENT event, uintptr_t pdevice_context)
{
    if(pdevice_context == 0){
        return;
    }
    switch(event){
        case MCP4XCXFX_SPI_TRANSFER_EVENT_PENDING:
            ((MCP4XCXFX_DEVICE_CONTEXT_P)pdevice_context)->if_status = MCP4XCXFX_INTERFACE_EVENT_PENDING;
            break;
        case MCP4XCXFX_SPI_TRANSFER_EVENT_COMPLETE:
            ((MCP4XCXFX_DEVICE_CONTEXT_P)pdevice_context)->if_status = MCP4XCXFX_INTERFACE_EVENT_COMPLETE;            
            break;
        case MCP4XCXFX_SPI_TRANSFER_NO_EVENT:
            ((MCP4XCXFX_DEVICE_CONTEXT_P)pdevice_context)->if_status = MCP4XCXFX_INTERFACE_NO_EVENT;            
            break;
        case MCP4XCXFX_SPI_TRANSFER_EVENT_HANDLE_EXPIRED:
        case MCP4XCXFX_SPI_TRANSFER_EVENT_ERROR:
        case MCP4XCXFX_SPI_TRANSFER_EVENT_HANDLE_INVALID:
        default:
            ((MCP4XCXFX_DEVICE_CONTEXT_P)pdevice_context)->if_status = MCP4XCXFX_INTERFACE_EVENT_ERROR;
    }
}

static void RequestCompletion(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_EVENT deviceEvent, int16_t retcode){
    pdevice->deviceEventStatus = deviceEvent;
    pdevice->processError = retcode;
    //call the user callback
    callUserCallback(pdevice);
    if ( pdevice->syncMode == true){
        pdevice->processingState = Sync;
    }else{
        pdevice->processingState = Idle;
    } 
}

int16_t MCP4XCXFX_LibTask(MCP4XCXFX_DEVICE_CONTEXT_P pdevice)
{
    if (pdevice == NULL) return MCP4XCXFX_LIBTASK_FAIL;
    
    switch(pdevice->processingState){
        case Sync:
            if ( pdevice->syncMode != true) pdevice->processingState = Idle;
        case Idle:
        case Uninitialized:
            pdevice->ABORT_REQUESTED_FLAG = false;
            return MCP4XCXFX_LIBTASK_DONE;
        default:
            break;
    }
    
    //if requested, abort the current processing
    if(pdevice->ABORT_REQUESTED_FLAG == true){
        pdevice->ABORT_REQUESTED_FLAG = false;
        RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_ABORT, MCP4XCXFX_REQUEST_ABORT);
        return MCP4XCXFX_LIBTASK_DONE;
    }
    
    if(pdevice->if_type == MCP4XCXFX_I2C){
       if(pdevice->if_ctxt.i2cCtxt.i2cEventCallbackRegistered == false){
        // if the communication events are not already reported via call-back function then 
        // we check for the communication status here.
            MCP4XCXFX_I2C_TRANSFER_EVENT i2cEvent;
            i2cEvent = MCP4XCXFX_I2C_TransferStatusGet(pdevice->if_ctxt.i2cCtxt);
            MCP4XCXFX_I2CEventHandler(i2cEvent, (uintptr_t)pdevice);
        }    
    }
    else{        
        if(pdevice->if_ctxt.spiCtxt.spiEventCallbackRegistered == false){
        // if the communication events are not already reported via call-back function then 
        // we check for the communication status here.
            MCP4XCXFX_SPI_TRANSFER_EVENT spiEvent;
            spiEvent = MCP4XCXFX_SPI_TransferStatusGet(pdevice->if_ctxt.spiCtxt);
            MCP4XCXFX_SPIEventHandler(spiEvent, (uintptr_t)pdevice);
        }
    }
    // if we are servicing an API request and we got a communication error
    // we terminate the request and signal the error
    if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_ERROR){
       RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_FAIL, MCP4XCXFX_COMMUNICATION_FAIL);
       return MCP4XCXFX_LIBTASK_DONE;    
    }    

    // if we are servicing an API request, continue the processing
    switch(pdevice->processingState)
    {
         case GetRegister16bitReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
                MCP4XCXFX_Get_Reg16bitProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }
            return MCP4XCXFX_LIBTASK_DONE;
            
        case GetVrefCtrlReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
               MCP4XCXFX_GetVrefCtrl_regProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }             
            return MCP4XCXFX_LIBTASK_DONE;
            
        case GetPwrDownCtrlReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
               MCP4XCXFX_GetPwrDownCtrl_regProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }             
            return MCP4XCXFX_LIBTASK_DONE;
            
        case GetGainCtrlReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
               MCP4XCXFX_GetGainCtrl_regProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }             
            return MCP4XCXFX_LIBTASK_DONE;
        
        case GetGainCtrl_NVReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
               MCP4XCXFX_GetGainCtrl_NV_regProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }             
            return MCP4XCXFX_LIBTASK_DONE;
            
        case SetGainCtrl_NVReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
                MCP4XCXFX_SetGainCtrl_NV_regProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }
            return MCP4XCXFX_LIBTASK_DONE;
            
        case GetWiperlock_NVReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
               MCP4XCXFX_GetWiperlock_NV_regProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }             
            return MCP4XCXFX_LIBTASK_DONE;

        case SetRegisterReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }
            return MCP4XCXFX_LIBTASK_DONE;
                      
        case ConfigBitUpdateReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }
            return MCP4XCXFX_LIBTASK_DONE;
            
        case GetI2Caddr_NVReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
               MCP4XCXFX_GetI2Caddr_NV_regProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }             
            return MCP4XCXFX_LIBTASK_DONE;
        
        case GetI2CaddrLock_NVReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
               MCP4XCXFX_GetI2Caddr_NV_regProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }             
            return MCP4XCXFX_LIBTASK_DONE;
        
        case GetPorStatusReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
               MCP4XCXFX_GetPorStatus_regProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }             
            return MCP4XCXFX_LIBTASK_DONE;
         
        case GetNVaccessStatusReq:
            if(pdevice->if_status == MCP4XCXFX_INTERFACE_EVENT_COMPLETE){
               MCP4XCXFX_GetNVaccessStatus_regProcess(pdevice);
                //complete the request and go to Sync state
                RequestCompletion(pdevice, MCP4XCXFX_EVENT_REQUEST_SUCCESS, MCP4XCXFX_SUCCESS);
            }             
            return MCP4XCXFX_LIBTASK_DONE;
            
        default:
            return MCP4XCXFX_LIBTASK_DONE;
    }
}


bool MCP4XCXFX_Device_IsInitialized(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    if (pdevice == NULL){ 
        return false;   // return "false" if pdevice is NULL
    }else{
        return (pdevice->processingState != Uninitialized);
    }
}


bool MCP4XCXFX_Device_IsBusy(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    bool deviceIsBusy = false;
    if (pdevice != NULL){
        if( pdevice->processingState != Idle )  deviceIsBusy = true; 
    }
    return deviceIsBusy;
}

int16_t MCP4XCXFX_InitDevice(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_DEVICE_INIT deviceInit) {
    int16_t retcode = MCP4XCXFX_SUCCESS;
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    //step 1 - initialize the device context
    memset(pdevice, 0, sizeof(MCP4XCXFX_DEVICE_CONTEXT));

    // Create the processingState mutex
    if( MCP4XCXFX_MUTEX_Create(&(pdevice->mutexProcState)) == false )
    {
        return MCP4XCXFX_MUTEX_FAIL;
    }
    
    // lock the processingState mutex
    if( MCP4XCXFX_MUTEX_Lock(&(pdevice->mutexProcState)) == false )
    {
        return MCP4XCXFX_BUSY;
    }
    
    pdevice->processingState = Uninitialized;
    pdevice->deviceEventStatus = MCP4XCXFX_EVENT_NONE;
    pdevice->ABORT_REQUESTED_FLAG = false;
    
    //step 2 - device selection
    pdevice->deviceID = deviceInit.deviceID;
    
    switch (pdevice->deviceID) {
        // 8-bit resolution devices
        case MCP47FEB01: case MCP47FEB02: case MCP47FEB04: case MCP47FEB08:
        case MCP47FVB01: case MCP47FVB02: case MCP47FVB04: case MCP47FVB08:
        case MCP47CMB01: case MCP47CMB02: case MCP47CMB04: case MCP47CMB08:
        case MCP47CVB01: case MCP47CVB02: case MCP47CVB04: case MCP47CVB08:
        case MCP47CMD01: case MCP47CMD02: case MCP47CVD01: case MCP47CVD02:
        case MCP48FEB01: case MCP48FEB02: case MCP48FEB04: case MCP48FEB08:
        case MCP48FVB01: case MCP48FVB02: case MCP48FVB04: case MCP48FVB08:
        case MCP48CMB01: case MCP48CMB02: case MCP48CMB04: case MCP48CMB08:
        case MCP48CVB01: case MCP48CVB02: case MCP48CVB04: case MCP48CVB08:
        case MCP48CMD01: case MCP48CMD02: case MCP48CVD01: case MCP48CVD02:
            {
                pdevice->resolution =  MCP4XCXFX_8bit;
                break;
            }

        // 10-bit resolution devices
        case MCP47FEB11: case MCP47FEB12: case MCP47FEB14: case MCP47FEB18:
        case MCP47FVB11: case MCP47FVB12: case MCP47FVB14: case MCP47FVB18:
        case MCP47CMB11: case MCP47CMB12: case MCP47CMB14: case MCP47CMB18:
        case MCP47CVB11: case MCP47CVB12: case MCP47CVB14: case MCP47CVB18:
        case MCP47CMD11: case MCP47CMD12: case MCP47CVD11: case MCP47CVD12:
        case MCP48FEB11: case MCP48FEB12: case MCP48FEB14: case MCP48FEB18:
        case MCP48FVB11: case MCP48FVB12: case MCP48FVB14: case MCP48FVB18:
        case MCP48CMB11: case MCP48CMB12: case MCP48CMB14: case MCP48CMB18:
        case MCP48CVB11: case MCP48CVB12: case MCP48CVB14: case MCP48CVB18:
        case MCP48CMD11: case MCP48CMD12: case MCP48CVD11: case MCP48CVD12:
            {
                pdevice->resolution =  MCP4XCXFX_10bit;
                break;
            }

        // 12-bit resolution devices
        case MCP47FEB21: case MCP47FEB22: case MCP47FEB24: case MCP47FEB28:
        case MCP47FVB21: case MCP47FVB22: case MCP47FVB24: case MCP47FVB28:
        case MCP47CMB21: case MCP47CMB22: case MCP47CMB24: case MCP47CMB28:
        case MCP47CVB21: case MCP47CVB22: case MCP47CVB24: case MCP47CVB28:
        case MCP47CMD21: case MCP47CMD22: case MCP47CVD21: case MCP47CVD22:
        case MCP48FEB21: case MCP48FEB22: case MCP48FEB24: case MCP48FEB28:
        case MCP48FVB21: case MCP48FVB22: case MCP48FVB24: case MCP48FVB28:
        case MCP48CMB21: case MCP48CMB22: case MCP48CMB24: case MCP48CMB28:
        case MCP48CVB21: case MCP48CVB22: case MCP48CVB24: case MCP48CVB28:
        case MCP48CMD21: case MCP48CMD22: case MCP48CVD21: case MCP48CVD22:
            {
                pdevice->resolution =  MCP4XCXFX_12bit;
                break;
            }

        default:
            retcode = MCP4XCXFX_INVALID_DEVICE;
            goto initialize_error;
    }

    switch (pdevice->deviceID) {
        // 1-channel devices
        case MCP47FEB01: case MCP47FEB11: case MCP47FEB21:
        case MCP47FVB01: case MCP47FVB11: case MCP47FVB21:
        case MCP47CMB01: case MCP47CMB11: case MCP47CMB21:
        case MCP47CVB01: case MCP47CVB11: case MCP47CVB21:
        case MCP47CMD01: case MCP47CMD11: case MCP47CMD21:
        case MCP47CVD01: case MCP47CVD11: case MCP47CVD21:
        case MCP48FEB01: case MCP48FEB11: case MCP48FEB21:
        case MCP48FVB01: case MCP48FVB11: case MCP48FVB21:
        case MCP48CMB01: case MCP48CMB11: case MCP48CMB21:
        case MCP48CVB01: case MCP48CVB11: case MCP48CVB21:
        case MCP48CMD01: case MCP48CMD11: case MCP48CMD21:
        case MCP48CVD01: case MCP48CVD11: case MCP48CVD21:
            {
                pdevice->channels = MCP4XCXFX_1ch;
                break;
            }

        // 2-channel devices
        case MCP47FEB02: case MCP47FEB12: case MCP47FEB22:
        case MCP47FVB02: case MCP47FVB12: case MCP47FVB22:
        case MCP47CMB02: case MCP47CMB12: case MCP47CMB22:
        case MCP47CVB02: case MCP47CVB12: case MCP47CVB22:
        case MCP47CMD02: case MCP47CMD12: case MCP47CMD22:
        case MCP47CVD02: case MCP47CVD12: case MCP47CVD22:
        case MCP48FEB02: case MCP48FEB12: case MCP48FEB22:
        case MCP48FVB02: case MCP48FVB12: case MCP48FVB22:
        case MCP48CMB02: case MCP48CMB12: case MCP48CMB22:
        case MCP48CVB02: case MCP48CVB12: case MCP48CVB22:
        case MCP48CMD02: case MCP48CMD12: case MCP48CMD22:
        case MCP48CVD02: case MCP48CVD12: case MCP48CVD22:
            {
                pdevice->channels = MCP4XCXFX_2ch;
                break;
            }

        // 4-channel devices
        case MCP47FEB04: case MCP47FEB14: case MCP47FEB24:
        case MCP47FVB04: case MCP47FVB14: case MCP47FVB24:
        case MCP47CMB04: case MCP47CMB14: case MCP47CMB24:
        case MCP47CVB04: case MCP47CVB14: case MCP47CVB24:
        case MCP48FEB04: case MCP48FEB14: case MCP48FEB24:
        case MCP48FVB04: case MCP48FVB14: case MCP48FVB24:
        case MCP48CMB04: case MCP48CMB14: case MCP48CMB24:
        case MCP48CVB04: case MCP48CVB14: case MCP48CVB24:
            {
                pdevice->channels = MCP4XCXFX_4ch;
                break;
            }

        // 8-channel devices
        case MCP47FEB08: case MCP47FEB18: case MCP47FEB28:
        case MCP47FVB08: case MCP47FVB18: case MCP47FVB28:
        case MCP47CMB08: case MCP47CMB18: case MCP47CMB28:
        case MCP47CVB08: case MCP47CVB18: case MCP47CVB28:
        case MCP48FEB08: case MCP48FEB18: case MCP48FEB28:
        case MCP48FVB08: case MCP48FVB18: case MCP48FVB28:
        case MCP48CMB08: case MCP48CMB18: case MCP48CMB28:
        case MCP48CVB08: case MCP48CVB18: case MCP48CVB28:
            {
                pdevice->channels = MCP4XCXFX_8ch;
                break;
            }

        default:
            retcode = MCP4XCXFX_INVALID_DEVICE;
            goto initialize_error;
    }
    
    switch (pdevice->deviceID) {
        // MCP47xxxxx devices (I2C interface)
        case MCP47FEB01: case MCP47FEB02: case MCP47FEB04: case MCP47FEB08:
        case MCP47FEB11: case MCP47FEB12: case MCP47FEB14: case MCP47FEB18:
        case MCP47FEB21: case MCP47FEB22: case MCP47FEB24: case MCP47FEB28:
        case MCP47FVB01: case MCP47FVB02: case MCP47FVB04: case MCP47FVB08:
        case MCP47FVB11: case MCP47FVB12: case MCP47FVB14: case MCP47FVB18:
        case MCP47FVB21: case MCP47FVB22: case MCP47FVB24: case MCP47FVB28:
        case MCP47CMB01: case MCP47CMB02: case MCP47CMB04: case MCP47CMB08:
        case MCP47CMB11: case MCP47CMB12: case MCP47CMB14: case MCP47CMB18:
        case MCP47CMB21: case MCP47CMB22: case MCP47CMB24: case MCP47CMB28:
        case MCP47CVB01: case MCP47CVB02: case MCP47CVB04: case MCP47CVB08:
        case MCP47CVB11: case MCP47CVB12: case MCP47CVB14: case MCP47CVB18:
        case MCP47CVB21: case MCP47CVB22: case MCP47CVB24: case MCP47CVB28:
        case MCP47CMD01: case MCP47CMD02: case MCP47CMD11: case MCP47CMD12:
        case MCP47CMD21: case MCP47CMD22: case MCP47CVD01: case MCP47CVD02:
        case MCP47CVD11: case MCP47CVD12: case MCP47CVD21: case MCP47CVD22:
            {
                pdevice->if_type = MCP4XCXFX_I2C;
                break;
            }

        // MCP48xxxxx devices (SPI interface)
        case MCP48FEB01: case MCP48FEB02: case MCP48FEB04: case MCP48FEB08:
        case MCP48FEB11: case MCP48FEB12: case MCP48FEB14: case MCP48FEB18:
        case MCP48FEB21: case MCP48FEB22: case MCP48FEB24: case MCP48FEB28:
        case MCP48FVB01: case MCP48FVB02: case MCP48FVB04: case MCP48FVB08:
        case MCP48FVB11: case MCP48FVB12: case MCP48FVB14: case MCP48FVB18:
        case MCP48FVB21: case MCP48FVB22: case MCP48FVB24: case MCP48FVB28:
        case MCP48CMB01: case MCP48CMB02: case MCP48CMB04: case MCP48CMB08:
        case MCP48CMB11: case MCP48CMB12: case MCP48CMB14: case MCP48CMB18:
        case MCP48CMB21: case MCP48CMB22: case MCP48CMB24: case MCP48CMB28:
        case MCP48CVB01: case MCP48CVB02: case MCP48CVB04: case MCP48CVB08:
        case MCP48CVB11: case MCP48CVB12: case MCP48CVB14: case MCP48CVB18:
        case MCP48CVB21: case MCP48CVB22: case MCP48CVB24: case MCP48CVB28:
        case MCP48CMD01: case MCP48CMD02: case MCP48CMD11: case MCP48CMD12:
        case MCP48CMD21: case MCP48CMD22: case MCP48CVD01: case MCP48CVD02:
        case MCP48CVD11: case MCP48CVD12: case MCP48CVD21: case MCP48CVD22:
            {
                pdevice->if_type = MCP4XCXFX_SPI;
                break;
            }

        default:
            retcode = MCP4XCXFX_INVALID_DEVICE;
            goto initialize_error;
    }
    
    switch (pdevice->deviceID) {
        // MCP4xxVxxx devices (No_NV)
        case MCP47FVB01: case MCP47FVB02: case MCP47FVB04: case MCP47FVB08:
        case MCP47FVB11: case MCP47FVB12: case MCP47FVB14: case MCP47FVB18:
        case MCP47FVB21: case MCP47FVB22: case MCP47FVB24: case MCP47FVB28:
        case MCP47CVB01: case MCP47CVB02: case MCP47CVB04: case MCP47CVB08:
        case MCP47CVB11: case MCP47CVB12: case MCP47CVB14: case MCP47CVB18:
        case MCP47CVB21: case MCP47CVB22: case MCP47CVB24: case MCP47CVB28:
        case MCP47CVD01: case MCP47CVD02: case MCP47CVD11: case MCP47CVD12:
        case MCP47CVD21: case MCP47CVD22:
        case MCP48FVB01: case MCP48FVB02: case MCP48FVB04: case MCP48FVB08:
        case MCP48FVB11: case MCP48FVB12: case MCP48FVB14: case MCP48FVB18:
        case MCP48FVB21: case MCP48FVB22: case MCP48FVB24: case MCP48FVB28:
        case MCP48CVB01: case MCP48CVB02: case MCP48CVB04: case MCP48CVB08:
        case MCP48CVB11: case MCP48CVB12: case MCP48CVB14: case MCP48CVB18:
        case MCP48CVB21: case MCP48CVB22: case MCP48CVB24: case MCP48CVB28:
        case MCP48CVD01: case MCP48CVD02: case MCP48CVD11: case MCP48CVD12:
        case MCP48CVD21: case MCP48CVD22:
            {
                pdevice->NVtype = No_NV;
                break;
            }

        // MCP4xCMxx devices (MTP)
        case MCP47CMB01: case MCP47CMB02: case MCP47CMB04: case MCP47CMB08:
        case MCP47CMB11: case MCP47CMB12: case MCP47CMB14: case MCP47CMB18:
        case MCP47CMB21: case MCP47CMB22: case MCP47CMB24: case MCP47CMB28:
        case MCP47CMD01: case MCP47CMD02: case MCP47CMD11: case MCP47CMD12:
        case MCP47CMD21: case MCP47CMD22:
        case MCP48CMB01: case MCP48CMB02: case MCP48CMB04: case MCP48CMB08:
        case MCP48CMB11: case MCP48CMB12: case MCP48CMB14: case MCP48CMB18:
        case MCP48CMB21: case MCP48CMB22: case MCP48CMB24: case MCP48CMB28:
        case MCP48CMD01: case MCP48CMD02: case MCP48CMD11: case MCP48CMD12:
        case MCP48CMD21: case MCP48CMD22:
            {
                pdevice->NVtype = MTP;
                break;
            }

        // MCP4xFExxx devices (EEPROM)
        case MCP47FEB01: case MCP47FEB02: case MCP47FEB04: case MCP47FEB08:
        case MCP47FEB11: case MCP47FEB12: case MCP47FEB14: case MCP47FEB18:
        case MCP47FEB21: case MCP47FEB22: case MCP47FEB24: case MCP47FEB28:
        case MCP48FEB01: case MCP48FEB02: case MCP48FEB04: case MCP48FEB08:
        case MCP48FEB11: case MCP48FEB12: case MCP48FEB14: case MCP48FEB18:
        case MCP48FEB21: case MCP48FEB22: case MCP48FEB24: case MCP48FEB28:
            {
                pdevice->NVtype = EEPROM;
                break;
            }

        default:
            retcode = MCP4XCXFX_INVALID_DEVICE;
            goto initialize_error;
    }
    
    
    //step 3 - configure the communication   
    switch(pdevice->if_type){
        case MCP4XCXFX_I2C: {
            if (false == MCP4XCXFX_I2C_Initialize(&(pdevice->if_ctxt.i2cCtxt), deviceInit.interfaceInit.i2cInit)){ 
                pdevice->processError = MCP4XCXFX_COMMUNICATION_FAIL;
                retcode = MCP4XCXFX_COMMUNICATION_FAIL;
                goto initialize_error;    
            }

            if(false == MCP4XCXFX_I2C_TransferEventHandlerSet(&(pdevice->if_ctxt.i2cCtxt), 
                                                            MCP4XCXFX_I2CEventHandler, 
                                                            (uintptr_t)pdevice)){        
                pdevice->if_ctxt.i2cCtxt.i2cEventCallbackRegistered = false;
            }else{
                pdevice->if_ctxt.i2cCtxt.i2cEventCallbackRegistered = true;
            }
            break;
        }
        case MCP4XCXFX_SPI:{
            if (false == MCP4XCXFX_SPI_Initialize(&(pdevice->if_ctxt.spiCtxt), deviceInit.interfaceInit.spiInit)){ 
                pdevice->processError = MCP4XCXFX_COMMUNICATION_FAIL;
                retcode = MCP4XCXFX_COMMUNICATION_FAIL;
                goto initialize_error;    
            }

            if(false == MCP4XCXFX_SPI_TransferEventHandlerSet(&(pdevice->if_ctxt.spiCtxt), 
                                                            MCP4XCXFX_SPIEventHandler, 
                                                            (uintptr_t)pdevice)){        
                pdevice->if_ctxt.spiCtxt.spiEventCallbackRegistered = false;
            }else{
                pdevice->if_ctxt.spiCtxt.spiEventCallbackRegistered = true;
             }
            break;
        }
        default:
            retcode = MCP4XCXFX_INVALID_DEVICE;
            goto initialize_error;
    }
        
    pdevice->syncMode = true;
    pdevice->processingState = Idle; // set the state machine to Idle to allow function calls
      
    //unlock the processingState mutex
    MCP4XCXFX_MUTEX_Unlock(&(pdevice->mutexProcState));
    
    //step 4 - set the initial device configuration
    
    if ((pdevice->if_type == MCP4XCXFX_I2C) && (pdevice->NVtype != No_NV) ){
        MCP4XCXFX_GCAD_REGFIELDS gainCtrl;
        retcode = MCP4XCXFX_GetGainCtrl_NV(pdevice, &gainCtrl);
    }
      
    initialize_error:
    if(retcode != MCP4XCXFX_SUCCESS){        
        pdevice->processingState = Uninitialized;
    }
    pdevice->syncMode = deviceInit.syncMode;              // set the user sync mode selection
    MCP4XCXFX_MUTEX_Unlock(&(pdevice->mutexProcState));   //no issue if the mutex was already un-locked 
    return retcode;
}

int16_t MCP4XCXFX_SetUserCallback(
    const MCP4XCXFX_DEVICE_CONTEXT_P pdevice,
    const MCP4XCXFX_EVENT_HANDLER userCallback,
    const uintptr_t userContext
){
    if( (pdevice == NULL) || (userCallback == NULL) ){
        return MCP4XCXFX_INVALID_PARAMETER;
    }

    //check if new device request is allowed
    // lock the processingState mutex
    if( MCP4XCXFX_MUTEX_Lock(&(pdevice->mutexProcState)) == false )    
    {
        return MCP4XCXFX_BUSY;
    }    
    
    if(pdevice->processingState == Idle){
        // set the callback
        pdevice->userCallback = userCallback;
        pdevice->userContext = userContext;
        MCP4XCXFX_MUTEX_Unlock(&(pdevice->mutexProcState));        
        return MCP4XCXFX_SUCCESS;
    }else
    {
        // reject new request
        MCP4XCXFX_MUTEX_Unlock(&(pdevice->mutexProcState));
        return MCP4XCXFX_BUSY;
    }  
}

int16_t MCP4XCXFX_GetEventStatus(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_EVENT_P pevent, int16_t* pProcessError){
    if ( (pdevice == NULL) || (pevent == NULL) || (pProcessError == NULL) ) return MCP4XCXFX_INVALID_PARAMETER;
    *pevent = pdevice->deviceEventStatus;
    *pProcessError = pdevice->processError;
    return MCP4XCXFX_SUCCESS;
}

static int16_t RequestReturn(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    //wait here for the request processing completion if the library is in SYNC mode
    int16_t retcode = MCP4XCXFX_REQUEST_PENDING;
    if(pdevice->syncMode == true){ 
        while (pdevice->processingState != Sync){
            retcode = MCP4XCXFX_LibTask(pdevice);
        }
        retcode = pdevice->processError;
        pdevice->processingState = Idle;     //the request is fully completed here
    }
    return retcode;
}

int16_t MCP4XCXFX_GetWiper(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint16_t* pwiperValue, MCP4XCXFX_device_channelNo channelNo){
    int16_t retcode = MCP4XCXFX_SUCCESS; 
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    if((uint8_t)channelNo >= (uint8_t)(pdevice->channels)){
        retcode = MCP4XCXFX_INVALID_PARAMETER;
        return retcode;
    }
    
    retcode =  MCP4XCXFX_Get_Register(pdevice, (void*)pwiperValue, 
                                (MCP4XCXFX_DAC0_ADDR + channelNo), MCP4XCXFX_WIPER_SZ, 
                                GetRegister16bitReq);
       
    return retcode;
}

int16_t MCP4XCXFX_SetWiper(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint16_t wiperValue, MCP4XCXFX_device_channelNo channelNo){
    int16_t retcode = MCP4XCXFX_SUCCESS;
    uint8_t pregisterBytes[MCP4XCXFX_WIPER_SZ];
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    if((uint8_t)channelNo >= (uint8_t)(pdevice->channels)){
        retcode = MCP4XCXFX_INVALID_PARAMETER;
        return retcode;
    }
    
    if (wiperValue >= (1 << (pdevice->resolution))) return MCP4XCXFX_INVALID_PARAMETER;
    
    MCP4XCXFX_Reg16bitToRawBytes(wiperValue, pregisterBytes);
    
    retcode =  MCP4XCXFX_Set_Register(pdevice, pregisterBytes, 
                                (MCP4XCXFX_DAC0_ADDR + channelNo), MCP4XCXFX_WIPER_SZ, 
                                SetRegisterReq);
    
    return retcode;
}

uint16_t MCP4XCXFX_GetVrefCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_VREF_REGFIELDS_P pVRctrl){
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    return MCP4XCXFX_Get_Register(pdevice, (void*)pVRctrl, 
                                MCP4XCXFX_VREF_ADDR, MCP4XCXFX_REG_SZ, 
                                GetVrefCtrlReq);
}

static void MCP4XCXFX_GetVrefCtrl_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    uint8_t* pRawValue;
    
    if(pdevice->if_type == MCP4XCXFX_I2C){
        pRawValue = pdevice->if_rxBuffer;
    }
    else{ //MCP4XCXFX_SPI
        pRawValue = pdevice->if_rxBuffer+1;
    }
    
    MCP4XCXFX_VrefCtrlBytesToRegfields(pRawValue, (MCP4XCXFX_VREF_REGFIELDS_P)pdevice->outData);
}

void MCP4XCXFX_VrefCtrlBytesToRegfields(uint8_t* pVrefCtrlBytes, MCP4XCXFX_VREF_REGFIELDS_P pVrefCtrlRegfields){

    pVrefCtrlRegfields->VR7  =   (pVrefCtrlBytes[0] >> MCP4XCXFX_VREF_CTRL_VR7_BITPOSMSB)      & MCP4XCXFX_VREF_CTRL_VR7_BITMASK;
    pVrefCtrlRegfields->VR6  =   (pVrefCtrlBytes[0] >> MCP4XCXFX_VREF_CTRL_VR6_BITPOSMSB)      & MCP4XCXFX_VREF_CTRL_VR6_BITMASK;
    pVrefCtrlRegfields->VR5  =   (pVrefCtrlBytes[0] >> MCP4XCXFX_VREF_CTRL_VR5_BITPOSMSB)      & MCP4XCXFX_VREF_CTRL_VR5_BITMASK;
    pVrefCtrlRegfields->VR4  =   (pVrefCtrlBytes[0] /*>> MCP4XCXFX_VREF_CTRL_VR4_BITPOSMSB*/)  & MCP4XCXFX_VREF_CTRL_VR4_BITMASK;
    pVrefCtrlRegfields->VR3  =   (pVrefCtrlBytes[1] >> MCP4XCXFX_VREF_CTRL_VR3_BITPOSLSB)      & MCP4XCXFX_VREF_CTRL_VR3_BITMASK;
    pVrefCtrlRegfields->VR2  =   (pVrefCtrlBytes[1] >> MCP4XCXFX_VREF_CTRL_VR2_BITPOSLSB)      & MCP4XCXFX_VREF_CTRL_VR2_BITMASK;
    pVrefCtrlRegfields->VR1  =   (pVrefCtrlBytes[1] >> MCP4XCXFX_VREF_CTRL_VR1_BITPOSLSB)      & MCP4XCXFX_VREF_CTRL_VR1_BITMASK;    
    pVrefCtrlRegfields->VR0  =   (pVrefCtrlBytes[1] /*>> MCP4XCXFX_VREF_CTRL_VR0_BITPOSLSB*/)  & MCP4XCXFX_VREF_CTRL_VR0_BITMASK;
}

void MCP4XCXFX_VrefCtrlRegfieldsToBytes(MCP4XCXFX_VREF_REGFIELDS VrefCtrlRegfields, uint8_t* pVrefCtrlBytes){
    //MSB
    pVrefCtrlBytes[0] = (uint8_t)(
                    (VrefCtrlRegfields.VR7 << MCP4XCXFX_VREF_CTRL_VR7_BITPOSMSB) |
                    (VrefCtrlRegfields.VR6 << MCP4XCXFX_VREF_CTRL_VR6_BITPOSMSB) |
                    (VrefCtrlRegfields.VR5 << MCP4XCXFX_VREF_CTRL_VR5_BITPOSMSB) |
                    (VrefCtrlRegfields.VR4 /* << MCP4XCXFX_VREF_CTRL_VR4_BITPOSMSB */ ));
    //LSB
    pVrefCtrlBytes[1] = (uint8_t)(
                    (VrefCtrlRegfields.VR3 << MCP4XCXFX_VREF_CTRL_VR3_BITPOSLSB)       |
                    (VrefCtrlRegfields.VR2 << MCP4XCXFX_VREF_CTRL_VR2_BITPOSLSB)                 |
                    (VrefCtrlRegfields.VR1 << MCP4XCXFX_VREF_CTRL_VR1_BITPOSLSB) |
                    (VrefCtrlRegfields.VR0 /* << MCP4XCXFX_VREF_CTRL_VR0_BITPOSLSB */ )); 
}

uint16_t MCP4XCXFX_SetVrefCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_VREF_REGFIELDS VRctrl){
    uint8_t pregisterBytes[MCP4XCXFX_REG_SZ];
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    MCP4XCXFX_VrefCtrlRegfieldsToBytes(VRctrl, pregisterBytes);
    return MCP4XCXFX_Set_Register(pdevice, pregisterBytes, 
                                MCP4XCXFX_VREF_ADDR, MCP4XCXFX_REG_SZ, 
                                SetRegisterReq);
}


uint16_t MCP4XCXFX_GetPwrDownCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_PWDN_REGFIELDS_P pPDctrl){
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    return MCP4XCXFX_Get_Register(pdevice, (void*)pPDctrl, 
                                MCP4XCXFX_PWDN_ADDR, MCP4XCXFX_REG_SZ, 
                                GetPwrDownCtrlReq);
}

static void MCP4XCXFX_GetPwrDownCtrl_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    uint8_t* pRawValue;
    
    if(pdevice->if_type == MCP4XCXFX_I2C){
        pRawValue = pdevice->if_rxBuffer;
    }
    else{ //MCP4XCXFX_SPI
        pRawValue = pdevice->if_rxBuffer+1;
    } 
    
    MCP4XCXFX_PwrDownCtrlBytesToRegfields(pRawValue, (MCP4XCXFX_PWDN_REGFIELDS_P)pdevice->outData);
}

void MCP4XCXFX_PwrDownCtrlBytesToRegfields(uint8_t* pPwrDownCtrlBytes, MCP4XCXFX_PWDN_REGFIELDS_P pPwrDownCtrlRegfields){

    pPwrDownCtrlRegfields->PD7  =   (pPwrDownCtrlBytes[0] >> MCP4XCXFX_PWDN_CTRL_PD7_BITPOSMSB)      & MCP4XCXFX_PWDN_CTRL_PD7_BITMASK;
    pPwrDownCtrlRegfields->PD6  =   (pPwrDownCtrlBytes[0] >> MCP4XCXFX_PWDN_CTRL_PD6_BITPOSMSB)      & MCP4XCXFX_PWDN_CTRL_PD6_BITMASK;
    pPwrDownCtrlRegfields->PD5  =   (pPwrDownCtrlBytes[0] >> MCP4XCXFX_PWDN_CTRL_PD5_BITPOSMSB)      & MCP4XCXFX_PWDN_CTRL_PD5_BITMASK;
    pPwrDownCtrlRegfields->PD4  =   (pPwrDownCtrlBytes[0] /*>> MCP4XCXFX_PWDN_CTRL_PD4_BITPOSMSB*/)  & MCP4XCXFX_PWDN_CTRL_PD4_BITMASK;
    pPwrDownCtrlRegfields->PD3  =   (pPwrDownCtrlBytes[1] >> MCP4XCXFX_PWDN_CTRL_PD3_BITPOSLSB)      & MCP4XCXFX_PWDN_CTRL_PD3_BITMASK;
    pPwrDownCtrlRegfields->PD2  =   (pPwrDownCtrlBytes[1] >> MCP4XCXFX_PWDN_CTRL_PD2_BITPOSLSB)      & MCP4XCXFX_PWDN_CTRL_PD2_BITMASK;
    pPwrDownCtrlRegfields->PD1  =   (pPwrDownCtrlBytes[1] >> MCP4XCXFX_PWDN_CTRL_PD1_BITPOSLSB)      & MCP4XCXFX_PWDN_CTRL_PD1_BITMASK;    
    pPwrDownCtrlRegfields->PD0  =   (pPwrDownCtrlBytes[1] /*>> MCP4XCXFX_PWDN_CTRL_PD0_BITPOSLSB*/)  & MCP4XCXFX_PWDN_CTRL_PD0_BITMASK;
}

void MCP4XCXFX_PwrDownCtrlRegfieldsToBytes(MCP4XCXFX_PWDN_REGFIELDS PwrDownCtrlRegfields, uint8_t* pPwrDownCtrlBytes){
    //MSB
    pPwrDownCtrlBytes[0] = (uint8_t)(
                    (PwrDownCtrlRegfields.PD7 << MCP4XCXFX_PWDN_CTRL_PD7_BITPOSMSB) |
                    (PwrDownCtrlRegfields.PD6 << MCP4XCXFX_PWDN_CTRL_PD6_BITPOSMSB) |
                    (PwrDownCtrlRegfields.PD5 << MCP4XCXFX_PWDN_CTRL_PD5_BITPOSMSB) |
                    (PwrDownCtrlRegfields.PD4 /* << MCP4XCXFX_PWDN_CTRL_PD4_BITPOSMSB */ ));
    //LSB
    pPwrDownCtrlBytes[1] = (uint8_t)(
                    (PwrDownCtrlRegfields.PD3 << MCP4XCXFX_PWDN_CTRL_PD3_BITPOSLSB)       |
                    (PwrDownCtrlRegfields.PD2 << MCP4XCXFX_PWDN_CTRL_PD2_BITPOSLSB)                 |
                    (PwrDownCtrlRegfields.PD1 << MCP4XCXFX_PWDN_CTRL_PD1_BITPOSLSB) |
                    (PwrDownCtrlRegfields.PD0 /* << MCP4XCXFX_PWDN_CTRL_PD0_BITPOSLSB */ )); 
}

uint16_t MCP4XCXFX_SetPwrDownCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_PWDN_REGFIELDS PDctrl){
    uint8_t pregisterBytes[MCP4XCXFX_REG_SZ];
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    MCP4XCXFX_PwrDownCtrlRegfieldsToBytes(PDctrl, pregisterBytes);
    return MCP4XCXFX_Set_Register(pdevice, pregisterBytes, 
                                MCP4XCXFX_PWDN_ADDR, MCP4XCXFX_REG_SZ, 
                                SetRegisterReq);
}

uint16_t MCP4XCXFX_GetGainCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_GCST_REGFIELDS_P pgainCtrl){
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    return MCP4XCXFX_Get_Register(pdevice, (void*)pgainCtrl, 
                                MCP4XCXFX_GCST_ADDR, MCP4XCXFX_REG_SZ, 
                                GetGainCtrlReq);
}

static void MCP4XCXFX_GetGainCtrl_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    uint8_t* pRawValue;
    
    if(pdevice->if_type == MCP4XCXFX_I2C){
        pRawValue = pdevice->if_rxBuffer;
    }
    else{ //MCP4XCXFX_SPI
        pRawValue = pdevice->if_rxBuffer+1;
    }
    
    MCP4XCXFX_GainStatusBytesToRegfields(pRawValue, (MCP4XCXFX_GCST_REGFIELDS_P)pdevice->outData);
}

void MCP4XCXFX_GainStatusBytesToRegfields(uint8_t* pGainCtrlBytes, MCP4XCXFX_GCST_REGFIELDS_P pGainCtrlRegfields){

    pGainCtrlRegfields->G7  =   (pGainCtrlBytes[0] >> MCP4XCXFX_GCST_G7_BITPOSMSB)        & MCP4XCXFX_GCST_G7_BITMASK;
    pGainCtrlRegfields->G6  =   (pGainCtrlBytes[0] >> MCP4XCXFX_GCST_G6_BITPOSMSB)        & MCP4XCXFX_GCST_G6_BITMASK;
    pGainCtrlRegfields->G5  =   (pGainCtrlBytes[0] >> MCP4XCXFX_GCST_G5_BITPOSMSB)        & MCP4XCXFX_GCST_G5_BITMASK;
    pGainCtrlRegfields->G4  =   (pGainCtrlBytes[0] >> MCP4XCXFX_GCST_G4_BITPOSMSB)        & MCP4XCXFX_GCST_G4_BITMASK;
    pGainCtrlRegfields->G3  =   (pGainCtrlBytes[0] >> MCP4XCXFX_GCST_G3_BITPOSMSB)        & MCP4XCXFX_GCST_G3_BITMASK;
    pGainCtrlRegfields->G2  =   (pGainCtrlBytes[0] >> MCP4XCXFX_GCST_G2_BITPOSMSB)        & MCP4XCXFX_GCST_G2_BITMASK;
    pGainCtrlRegfields->G1  =   (pGainCtrlBytes[0] >> MCP4XCXFX_GCST_G1_BITPOSMSB)        & MCP4XCXFX_GCST_G1_BITMASK;    
    pGainCtrlRegfields->G0  =   (pGainCtrlBytes[0] /*>> MCP4XCXFX_GCST_G0_BITPOSMSB*/)    & MCP4XCXFX_GCST_G0_BITMASK;
    pGainCtrlRegfields->POR  =  (pGainCtrlBytes[1] >> MCP4XCXFX_GCST_POR_BITPOSLSB)       & MCP4XCXFX_GCST_POR_BITMASK;    
    pGainCtrlRegfields->NV  =   (pGainCtrlBytes[1] >> MCP4XCXFX_GCST_NV_BITPOSLSB)        & MCP4XCXFX_GCST_NV_BITMASK;
}

void MCP4XCXFX_GainStatusRegfieldsToBytes(MCP4XCXFX_GCST_REGFIELDS GainStatusRegfields, uint8_t* pGainStatusBytes){
    //MSB
    pGainStatusBytes[0] = (uint8_t)(
                    (GainStatusRegfields.G7 << MCP4XCXFX_GCST_G7_BITPOSMSB) |
                    (GainStatusRegfields.G6 << MCP4XCXFX_GCST_G6_BITPOSMSB) |
                    (GainStatusRegfields.G5 << MCP4XCXFX_GCST_G5_BITPOSMSB) |
                    (GainStatusRegfields.G4 << MCP4XCXFX_GCST_G4_BITPOSMSB) |
                    (GainStatusRegfields.G3 << MCP4XCXFX_GCST_G3_BITPOSMSB) |
                    (GainStatusRegfields.G2 << MCP4XCXFX_GCST_G2_BITPOSMSB) |
                    (GainStatusRegfields.G1 << MCP4XCXFX_GCST_G1_BITPOSMSB) |
                    (GainStatusRegfields.G0 /* << MCP4XCXFX_GCST_G0_BITPOSMSB */ ));
    //LSB
    pGainStatusBytes[1] = (uint8_t)(
                    (GainStatusRegfields.POR << MCP4XCXFX_GCST_POR_BITPOSLSB) |
                    (GainStatusRegfields.NV  << MCP4XCXFX_GCST_NV_BITPOSLSB )); 
}

uint16_t MCP4XCXFX_SetGainCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_GCST_REGFIELDS gainCtrl){
    uint8_t pregisterBytes[MCP4XCXFX_REG_SZ];
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    MCP4XCXFX_GainStatusRegfieldsToBytes(gainCtrl, pregisterBytes);
    
    return MCP4XCXFX_Set_Register(pdevice, pregisterBytes, 
                                MCP4XCXFX_GCST_ADDR, MCP4XCXFX_REG_SZ, 
                                SetRegisterReq);
}

static int16_t MCP4XCXFX_Get_Register(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, void* pregister_val, uint8_t registerAddr, size_t regSize, MCP4XCXFX_procState processingState){
    int16_t retcode;
    bool bSuccess;
    
    if ((pdevice == NULL) || (pregister_val == NULL)) return MCP4XCXFX_INVALID_PARAMETER;
    
    // check if new device request is allowed
    // lock the processingState mutex
    if( MCP4XCXFX_MUTEX_Lock(&(pdevice->mutexProcState)) == false )    
    {
        return MCP4XCXFX_BUSY;
    }
    
    if(pdevice->processingState == Idle){
        // start new request 
        pdevice->processingState = processingState;
        pdevice->deviceEventStatus = MCP4XCXFX_EVENT_NONE;
        MCP4XCXFX_MUTEX_Unlock(&(pdevice->mutexProcState));        
    }else
    {
        // reject new request
        MCP4XCXFX_MUTEX_Unlock(&(pdevice->mutexProcState));        
        return MCP4XCXFX_BUSY;
    }
    
    pdevice->outData = (void*)pregister_val;
    
    //register address <<3, add "11" read command    
    registerAddr = (registerAddr << 3) | 0b110;
            
    pdevice->if_txBuffer[0] = registerAddr;
    
    pdevice->if_status = MCP4XCXFX_INTERFACE_NO_EVENT;
    
    if(pdevice->if_type == MCP4XCXFX_I2C){
        size_t i2cRxSize = regSize;
        bSuccess = MCP4XCXFX_I2C_WriteRead(&(pdevice->if_ctxt.i2cCtxt), (void*) pdevice->if_txBuffer, 1, 
                                            (void*) pdevice->if_rxBuffer, i2cRxSize);       
    }else{ /* MCP4XCXFX_SPI */
        size_t spiRxSize = regSize + 1;
        bSuccess = MCP4XCXFX_SPI_WriteRead(&(pdevice->if_ctxt.spiCtxt), (void*) pdevice->if_txBuffer, 1,
                                           (void*) pdevice->if_rxBuffer, spiRxSize); 
    }

    if(bSuccess == false){
        pdevice->processingState = Idle;
        pdevice->processError = MCP4XCXFX_COMMUNICATION_FAIL;
        return MCP4XCXFX_COMMUNICATION_FAIL;
    }
    
    // wait here for the request processing completion if the library is in SYNC mode
    retcode = RequestReturn(pdevice);
    return retcode;
}

static void MCP4XCXFX_Get_Reg16bitProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    uint8_t* pRawBytes;
    uint16_t regVal;
    
    pRawBytes = pdevice->if_rxBuffer; 
    
    if(pdevice->if_type == MCP4XCXFX_I2C){
        regVal = MCP4XCXFX_RawBytestoReg16bit(pRawBytes);
    }
    else{ //MCP4XCXFX_SPI
        regVal = MCP4XCXFX_RawBytestoReg16bit(pRawBytes+1);
    }
         
    *((uint16_t*)pdevice->outData) = regVal;
}

static int16_t MCP4XCXFX_Set_Register(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t *pregisterBytes, uint8_t registerAddr, size_t regSize, MCP4XCXFX_procState processingState){
    int16_t retcode;
    bool bSuccess;
    
    if ( (pdevice == NULL) || (pregisterBytes == NULL) ) return MCP4XCXFX_INVALID_PARAMETER;
    
    // check if new device request is allowed
    // lock the processingState mutex
    if( MCP4XCXFX_MUTEX_Lock(&(pdevice->mutexProcState)) == false )
    {
        return MCP4XCXFX_BUSY;
    }
    
    if(pdevice->processingState == Idle){
        // start new request 
        pdevice->processingState = processingState;
        pdevice->deviceEventStatus = MCP4XCXFX_EVENT_NONE;
        MCP4XCXFX_MUTEX_Unlock(&(pdevice->mutexProcState));        
    }else
    {
        // reject new request
        MCP4XCXFX_MUTEX_Unlock(&(pdevice->mutexProcState));        
        return MCP4XCXFX_BUSY;
    }
    
    //register address <<3, add "00" write command    
    registerAddr = (registerAddr << 3) | 0b000;
    
    pdevice->if_txBuffer[0] = registerAddr;
    
    for(int cnt=0; cnt < regSize; cnt++){
        pdevice->if_txBuffer[cnt+1] = pregisterBytes[cnt];
    }
    
    pdevice->if_status = MCP4XCXFX_INTERFACE_NO_EVENT;

    if(pdevice->if_type == MCP4XCXFX_I2C){
        bSuccess = MCP4XCXFX_I2C_Write(&(pdevice->if_ctxt.i2cCtxt), (void*) pdevice->if_txBuffer, (regSize + 1));
    }else{ /* MCP4XCXFX_SPI */
        bSuccess = MCP4XCXFX_SPI_Write(&(pdevice->if_ctxt.spiCtxt), (void*) pdevice->if_txBuffer, (regSize + 1));
    }
    
    if(bSuccess == false){
        pdevice->processingState = Idle;
        pdevice->processError = MCP4XCXFX_COMMUNICATION_FAIL;
        return MCP4XCXFX_COMMUNICATION_FAIL;
    }    
    // wait here for the request processing completion if the library is in SYNC mode
    retcode = RequestReturn(pdevice);
    return retcode;
}

void MCP4XCXFX_Reg16bitToRawBytes(uint16_t regVal, uint8_t* pRawBytes){
    pRawBytes[0] = (regVal >> 8) & 0xff;
    pRawBytes[1] = regVal & 0xff;
}

uint16_t MCP4XCXFX_RawBytestoReg16bit(uint8_t* pRawBytes){
    uint16_t regVal;
    
    regVal = (uint16_t)pRawBytes[0];
    regVal = (regVal << 8) | (uint16_t)pRawBytes[1];
    
    return regVal;
}

//Non-volatile memory register get/set methods

int16_t MCP4XCXFX_GetWiper_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint16_t* pwiperValue, MCP4XCXFX_device_channelNo channelNo){
    int16_t retcode = MCP4XCXFX_SUCCESS; 
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    if((uint8_t)channelNo >= (uint8_t)(pdevice->channels)){
        retcode = MCP4XCXFX_INVALID_PARAMETER;
        return retcode;
    }
    
    retcode =  MCP4XCXFX_Get_Register(pdevice, (void*)pwiperValue, 
                                (MCP4XCXFX_DAC0_ADDR_NV + channelNo), MCP4XCXFX_WIPER_SZ, 
                                GetRegister16bitReq);
    
    return retcode;
}

int16_t MCP4XCXFX_SetWiper_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint16_t wiperValue, MCP4XCXFX_device_channelNo channelNo){
    int16_t retcode = MCP4XCXFX_SUCCESS;
    uint8_t pregisterBytes[MCP4XCXFX_WIPER_SZ];
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    if((uint8_t)channelNo >= (uint8_t)(pdevice->channels)){
        retcode = MCP4XCXFX_INVALID_PARAMETER;
        return retcode;
    }

    MCP4XCXFX_Reg16bitToRawBytes(wiperValue, pregisterBytes);
    
    retcode =  MCP4XCXFX_Set_Register(pdevice, pregisterBytes, 
                                (MCP4XCXFX_DAC0_ADDR_NV + channelNo), MCP4XCXFX_WIPER_SZ, 
                                SetRegisterReq);
        
    return retcode;
}

uint16_t MCP4XCXFX_GetVrefCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_VREF_REGFIELDS_P pVRctrl){
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    return MCP4XCXFX_Get_Register(pdevice, (void*)pVRctrl, 
                                MCP4XCXFX_VREF_ADDR_NV, MCP4XCXFX_REG_SZ, 
                                GetVrefCtrlReq);
}

uint16_t MCP4XCXFX_SetVrefCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_VREF_REGFIELDS VRctrl){
    uint8_t pregisterBytes[MCP4XCXFX_REG_SZ];
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    MCP4XCXFX_VrefCtrlRegfieldsToBytes(VRctrl, pregisterBytes);
    return MCP4XCXFX_Set_Register(pdevice, pregisterBytes, 
                                MCP4XCXFX_VREF_ADDR_NV, MCP4XCXFX_REG_SZ, 
                                SetRegisterReq);
}

uint16_t MCP4XCXFX_GetPwrDownCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_PWDN_REGFIELDS_P pPDctrl){
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    return MCP4XCXFX_Get_Register(pdevice, (void*)pPDctrl, 
                                MCP4XCXFX_PWDN_ADDR_NV, MCP4XCXFX_REG_SZ, 
                                GetPwrDownCtrlReq);
}

uint16_t MCP4XCXFX_SetPwrDownCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_PWDN_REGFIELDS PDctrl){
    uint8_t pregisterBytes[MCP4XCXFX_REG_SZ];
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    MCP4XCXFX_PwrDownCtrlRegfieldsToBytes(PDctrl, pregisterBytes);
    return MCP4XCXFX_Set_Register(pdevice, pregisterBytes, 
                                MCP4XCXFX_PWDN_ADDR_NV, MCP4XCXFX_REG_SZ, 
                                SetRegisterReq);
}

uint16_t MCP4XCXFX_GetGainCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_GCAD_REGFIELDS_P pgainCtrl){
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    return MCP4XCXFX_Get_Register(pdevice, (void*)pgainCtrl, 
                                MCP4XCXFX_GCAD_ADDR_NV, MCP4XCXFX_REG_SZ, 
                                GetGainCtrl_NVReq);
}

static void MCP4XCXFX_GetGainCtrl_NV_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    uint8_t* pRawValue;
    
    if(pdevice->if_type == MCP4XCXFX_I2C){
        pRawValue = pdevice->if_rxBuffer;
    }
    else{ //MCP4XCXFX_SPI
        pRawValue = pdevice->if_rxBuffer+1;
    }
    MCP4XCXFX_GainCtrlBytesToRegfields(pRawValue, (MCP4XCXFX_GCAD_REGFIELDS_P)pdevice->outData);
    memcpy(pdevice->gainCtrlNV_cache, pRawValue, MCP4XCXFX_REG_SZ);
}

static void MCP4XCXFX_SetGainCtrl_NV_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    
    memcpy(pdevice->gainCtrlNV_cache, pdevice->stagingBytes, MCP4XCXFX_REG_SZ);
   
}

void MCP4XCXFX_GainCtrlBytesToRegfields(uint8_t* pGainCtrlBytes, MCP4XCXFX_GCAD_REGFIELDS_P pGainCtrlRegfields){

    pGainCtrlRegfields->G7  =       (pGainCtrlBytes[0] >> MCP4XCXFX_GCAD_G7_BITPOSMSB)        & MCP4XCXFX_GCAD_G7_BITMASK;
    pGainCtrlRegfields->G6  =       (pGainCtrlBytes[0] >> MCP4XCXFX_GCAD_G6_BITPOSMSB)        & MCP4XCXFX_GCAD_G6_BITMASK;
    pGainCtrlRegfields->G5  =       (pGainCtrlBytes[0] >> MCP4XCXFX_GCAD_G5_BITPOSMSB)        & MCP4XCXFX_GCAD_G5_BITMASK;
    pGainCtrlRegfields->G4  =       (pGainCtrlBytes[0] >> MCP4XCXFX_GCAD_G4_BITPOSMSB)        & MCP4XCXFX_GCAD_G4_BITMASK;
    pGainCtrlRegfields->G3  =       (pGainCtrlBytes[0] >> MCP4XCXFX_GCAD_G3_BITPOSMSB)        & MCP4XCXFX_GCAD_G3_BITMASK;
    pGainCtrlRegfields->G2  =       (pGainCtrlBytes[0] >> MCP4XCXFX_GCAD_G2_BITPOSMSB)        & MCP4XCXFX_GCAD_G2_BITMASK;
    pGainCtrlRegfields->G1  =       (pGainCtrlBytes[0] >> MCP4XCXFX_GCAD_G1_BITPOSMSB)        & MCP4XCXFX_GCAD_G1_BITMASK;    
    pGainCtrlRegfields->G0  =       (pGainCtrlBytes[0] /*>> MCP4XCXFX_GCAD_G0_BITPOSMSB*/)    & MCP4XCXFX_GCAD_G0_BITMASK;
    pGainCtrlRegfields->ADLCK  =    (pGainCtrlBytes[1] >> MCP4XCXFX_GCAD_ADLCK_BITPOSLSB)       & MCP4XCXFX_GCAD_ADLCK_BITMASK;    
    pGainCtrlRegfields->ADDR_I2C  = (pGainCtrlBytes[1] >> MCP4XCXFX_GCAD_ADDR_I2C_BITPOSLSB)        & MCP4XCXFX_GCAD_ADDR_I2C_BITMASK;
}

void MCP4XCXFX_GainCtrlRegfieldsToBytes(MCP4XCXFX_GCAD_REGFIELDS GainStatusRegfields, uint8_t* pGainStatusBytes){
    //MSB
    pGainStatusBytes[0] = (uint8_t)(
                    (GainStatusRegfields.G7 << MCP4XCXFX_GCAD_G7_BITPOSMSB) |
                    (GainStatusRegfields.G6 << MCP4XCXFX_GCAD_G6_BITPOSMSB) |
                    (GainStatusRegfields.G5 << MCP4XCXFX_GCAD_G5_BITPOSMSB) |
                    (GainStatusRegfields.G4 << MCP4XCXFX_GCAD_G4_BITPOSMSB) |
                    (GainStatusRegfields.G3 << MCP4XCXFX_GCAD_G3_BITPOSMSB) |
                    (GainStatusRegfields.G2 << MCP4XCXFX_GCAD_G2_BITPOSMSB) |
                    (GainStatusRegfields.G1 << MCP4XCXFX_GCAD_G1_BITPOSMSB) |
                    (GainStatusRegfields.G0 /* << MCP4XCXFX_GCAD_G0_BITPOSMSB */ ));
    //LSB
    pGainStatusBytes[1] = (uint8_t)(
                    (GainStatusRegfields.ADLCK << MCP4XCXFX_GCAD_ADLCK_BITPOSLSB) |
                    (GainStatusRegfields.ADDR_I2C  << MCP4XCXFX_GCAD_ADDR_I2C_BITPOSLSB )); 
}

uint16_t MCP4XCXFX_SetGainCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_GCAD_REGFIELDS gainCtrl){
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    MCP4XCXFX_GainCtrlRegfieldsToBytes(gainCtrl, pdevice->stagingBytes);    
    return MCP4XCXFX_Set_Register(pdevice, pdevice->stagingBytes, 
                                MCP4XCXFX_GCAD_ADDR_NV, MCP4XCXFX_REG_SZ, 
                                SetGainCtrl_NVReq);
}

//L3 functions

uint32_t MCP4XCXFX_OutputVoltage_uV(MCP4XCXFX_device_resolution resolution, uint16_t wiperValue, uint32_t Vref_uV, MCP4XCXFX_gainCtrlBit gain){
    uint32_t voltage;
    uint16_t resistor_no = 1;
    
    switch (resolution){   
        case MCP4XCXFX_8bit:{
            resistor_no = 256;
            break;
        }
        case MCP4XCXFX_10bit:{
            resistor_no = 1024;
            break;
        }
        case MCP4XCXFX_12bit:{
            resistor_no = 4096;
            break;
        }           
    }
    
    voltage = (Vref_uV * wiperValue * (gain + 1)) / resistor_no; //equation 4-2 from DS20006537B
    
    return voltage;
}       

uint32_t MCP4XCXFX_StepVoltage_uV(MCP4XCXFX_device_resolution resolution, uint32_t Vref_uV, MCP4XCXFX_gainCtrlBit gain){
    uint32_t voltage;
    uint16_t resistor_no = 1;
    
    switch (resolution){
        case MCP4XCXFX_8bit:{
            resistor_no = 256;
            break;
        }
        case MCP4XCXFX_10bit:{
            resistor_no = 1024;
            break;
        }
        case MCP4XCXFX_12bit:{
            resistor_no = 4096;
            break;
        }           
    }
    
    voltage = (Vref_uV * (gain + 1)) / resistor_no; //equation 4-3 from DS20006537B
    
    return voltage;
}                                      

uint16_t MCP4XCXFX_WiperValue(MCP4XCXFX_device_resolution resolution, int32_t OutputVoltage_uV, int32_t Vref_uV, MCP4XCXFX_gainCtrlBit gain){
    uint32_t wiper_val;
    uint16_t resistor_no = 1;
    
    switch (resolution){
        case MCP4XCXFX_8bit:{
            resistor_no = 256;
            break;
        }
        case MCP4XCXFX_10bit:{
            resistor_no = 1024;
            break;
        }
        case MCP4XCXFX_12bit:{
            resistor_no = 4096;
            break;
        }           
    }
    
    wiper_val = (OutputVoltage_uV * resistor_no) / (Vref_uV * (gain + 1));
    
    // Saturate to max value for resolution
    if (wiper_val > (resistor_no - 1))
        wiper_val = (resistor_no - 1);
    
    return wiper_val;
}

//I2C address

uint16_t MCP4XCXFX_SetI2Caddr_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t i2cAddr){
    int16_t retcode = MCP4XCXFX_SUCCESS;
    MCP4XCXFX_GCAD_REGFIELDS gainCtrl;
    
    if ((pdevice->if_type) != MCP4XCXFX_I2C) return MCP4XCXFX_INVALID_DEVICE;    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    MCP4XCXFX_GainCtrlBytesToRegfields(pdevice->gainCtrlNV_cache, &gainCtrl);
    
    if((i2cAddr & 0b0000011) != (gainCtrl.ADDR_I2C & 0b0000011)) return MCP4XCXFX_INVALID_PARAMETER;
    
    gainCtrl.ADDR_I2C = i2cAddr;
    
    retcode = MCP4XCXFX_SetGainCtrl_NV(pdevice, gainCtrl);
        
    return retcode;
}

uint16_t MCP4XCXFX_GetI2Caddr_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t* pi2cAddr){
    if ((pdevice->if_type) != MCP4XCXFX_I2C) return MCP4XCXFX_INVALID_DEVICE;    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    return MCP4XCXFX_Get_Register(pdevice, (void*)pi2cAddr, 
                                MCP4XCXFX_GCAD_ADDR_NV, MCP4XCXFX_REG_SZ, 
                                GetI2Caddr_NVReq);
}

static void MCP4XCXFX_GetI2Caddr_NV_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    uint8_t* pRawValue;
    MCP4XCXFX_GCAD_REGFIELDS gainCtrl;
    
    if(pdevice->if_type == MCP4XCXFX_I2C){
        pRawValue = pdevice->if_rxBuffer;
    }
    else{ //MCP4XCXFX_SPI
        pRawValue = pdevice->if_rxBuffer+1;
    }  
    
    memcpy(pdevice->gainCtrlNV_cache, pRawValue, MCP4XCXFX_REG_SZ);
    
    MCP4XCXFX_GainCtrlBytesToRegfields(pRawValue, &gainCtrl);
    
    if(pdevice->processingState == GetI2Caddr_NVReq){
        *(uint8_t*)pdevice->outData = gainCtrl.ADDR_I2C;
    }
    else{
        *(uint8_t*)pdevice->outData = gainCtrl.ADLCK;
    }    
}

uint16_t MCP4XCXFX_SetI2CaddrLock_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_LCKcfgBit i2cLock){
    int16_t retcode = MCP4XCXFX_SUCCESS;
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    if (pdevice->NVtype != EEPROM){
        return MCP4XCXFX_INVALID_DEVICE;
    }
    
    retcode = MCP4XCXFX_ConfigBitUpdate(pdevice, SALCK_BIT, i2cLock);
    
    return retcode;
}   //only on EEPROM ("F") devices. 

uint16_t MCP4XCXFX_GetI2CaddrLock_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t* pi2cLock){
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;    
    if (pdevice->NVtype != EEPROM){
        return MCP4XCXFX_INVALID_DEVICE;
    }   
    
    return MCP4XCXFX_Get_Register(pdevice, (void*)pi2cLock, 
                                MCP4XCXFX_GCAD_ADDR_NV, MCP4XCXFX_REG_SZ, 
                                GetI2CaddrLock_NVReq);
} //only on EEPROM ("F") devices

//status check functions
uint16_t MCP4XCXFX_GetPorStatus(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t* pstatus){
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    return MCP4XCXFX_Get_Register(pdevice, (void*)pstatus, 
                                MCP4XCXFX_GCST_ADDR, MCP4XCXFX_REG_SZ, 
                                GetPorStatusReq);
}

static void MCP4XCXFX_GetPorStatus_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    uint8_t* pRawValue;
    MCP4XCXFX_GCST_REGFIELDS gainStatus;
    
    if(pdevice->if_type == MCP4XCXFX_I2C){
        pRawValue = pdevice->if_rxBuffer;
    }
    else{ //MCP4XCXFX_SPI
        pRawValue = pdevice->if_rxBuffer+1;
    }    
    MCP4XCXFX_GainStatusBytesToRegfields(pRawValue, &gainStatus);
    
    *(uint8_t*)pdevice->outData = gainStatus.POR;
}

uint16_t MCP4XCXFX_GetNVaccessStatus(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t* pstatus){
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;    
    
    return MCP4XCXFX_Get_Register(pdevice, (void*)pstatus, 
                                MCP4XCXFX_GCST_ADDR, MCP4XCXFX_REG_SZ, 
                                GetNVaccessStatusReq);
}

static void MCP4XCXFX_GetNVaccessStatus_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    uint8_t* pRawValue;
    MCP4XCXFX_GCST_REGFIELDS gainStatus;
    
    if(pdevice->if_type == MCP4XCXFX_I2C){
        pRawValue = pdevice->if_rxBuffer;
    }
    else{ //MCP4XCXFX_SPI
        pRawValue = pdevice->if_rxBuffer+1;
    }
    MCP4XCXFX_GainStatusBytesToRegfields(pRawValue, &gainStatus);
    
    *(uint8_t*)pdevice->outData = gainStatus.NV;
}

//CL:DL config bit enable and disable commands
uint16_t MCP4XCXFX_ConfigBitUpdate(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_LCKcfgBitAddr cfgBitAddr, MCP4XCXFX_LCKcfgBit bit){
    int16_t retcode = MCP4XCXFX_SUCCESS;
    bool bSuccess;
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    if (pdevice->NVtype != EEPROM){
        return MCP4XCXFX_INVALID_DEVICE;
    }
    
    MCP4XCXFX_writeEnable_NV();
    MCP4XCXFX_delay_ms(100);    
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    // check if new device request is allowed
    // lock the processingState mutex
    if( MCP4XCXFX_MUTEX_Lock(&(pdevice->mutexProcState)) == false )
    {
        return MCP4XCXFX_BUSY;
    }
    
    if(pdevice->processingState == Idle){
        // start new request 
        pdevice->processingState = ConfigBitUpdateReq;
        pdevice->deviceEventStatus = MCP4XCXFX_EVENT_NONE;
        MCP4XCXFX_MUTEX_Unlock(&(pdevice->mutexProcState));        
    }else
    {
        // reject new request
        MCP4XCXFX_MUTEX_Unlock(&(pdevice->mutexProcState));        
        return MCP4XCXFX_BUSY;
    }
    
    //register address <<3, add "00" write command
    if(bit == MCP4XCXFX_LCKbit_locked){
        cfgBitAddr = (cfgBitAddr << 3) | 0b100;
    }
    else{
        cfgBitAddr = (cfgBitAddr << 3) | 0b010;
    }
    
    pdevice->if_txBuffer[0] = cfgBitAddr;

    pdevice->if_status = MCP4XCXFX_INTERFACE_NO_EVENT;

    if(pdevice->if_type == MCP4XCXFX_I2C){
        bSuccess = MCP4XCXFX_I2C_Write(&(pdevice->if_ctxt.i2cCtxt), (void*) pdevice->if_txBuffer, 1);
    }else{ /* MCP4XCXFX_SPI */        
        bSuccess = MCP4XCXFX_SPI_Write(&(pdevice->if_ctxt.spiCtxt), (void*) pdevice->if_txBuffer, 1);
    }
    
    if(bSuccess == false){
        pdevice->processingState = Idle;
        pdevice->processError = MCP4XCXFX_COMMUNICATION_FAIL;
        return MCP4XCXFX_COMMUNICATION_FAIL;
    }    
    // wait here for the request processing completion if the library is in SYNC mode
    retcode = RequestReturn(pdevice);
    
    MCP4XCXFX_writeDisable_NV();
    
    return retcode;
}//use the "enable" or "disable" commands on EEPROM only

//I2C General call command
uint16_t MCP4XCXFX_I2C_General_Call(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_Gen_call_cmd cmd){
    int16_t retcode = MCP4XCXFX_SUCCESS;
    uint8_t pregisterBytes[MCP4XCXFX_GEN_CALL_SZ];
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    if (pdevice->if_type != MCP4XCXFX_I2C){
        return MCP4XCXFX_INVALID_DEVICE;
    }
        
    switch (cmd){
        case MCP4XCXFX_I2C_reset_cmd:{
            pregisterBytes[0] = MCP4XCXFX_GEN_CALL_RESET;
            return MCP4XCXFX_Set_Register(pdevice, pregisterBytes, 
                                MCP4XCXFX_GEN_CALL_ADDR, MCP4XCXFX_GEN_CALL_SZ, 
                                SetRegisterReq);
        }
        case MCP4XCXFX_I2C_wakeup_cmd:{
            pregisterBytes[0] = MCP4XCXFX_GEN_CALL_WAKEUP;
            return MCP4XCXFX_Set_Register(pdevice, pregisterBytes, 
                                MCP4XCXFX_GEN_CALL_ADDR, MCP4XCXFX_GEN_CALL_SZ, 
                                SetRegisterReq);
        }
    }
    
    
    return retcode;
}

int16_t MCP4XCXFX_SetWiper_block(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_WIPER_BLOCK wiperValues, bool activate){
    int16_t retcode = MCP4XCXFX_SUCCESS;
        
    //this function sets the wiper values for all DAC channels but activate only when needed by using WiperLatchActivate/Deactivate functions depending on activate status.
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    if (activate == false){
        MCP4XCXFX_wiperLatchDeactivate();
    }
       
    retcode = MCP4XCXFX_SetWiper(pdevice, wiperValues.CH0, MCP4XCXFX_ch0);
    retcode = MCP4XCXFX_SetWiper(pdevice, wiperValues.CH1, MCP4XCXFX_ch1);
    retcode = MCP4XCXFX_SetWiper(pdevice, wiperValues.CH2, MCP4XCXFX_ch2);
    retcode = MCP4XCXFX_SetWiper(pdevice, wiperValues.CH3, MCP4XCXFX_ch3);
    retcode = MCP4XCXFX_SetWiper(pdevice, wiperValues.CH4, MCP4XCXFX_ch4);
    retcode = MCP4XCXFX_SetWiper(pdevice, wiperValues.CH5, MCP4XCXFX_ch5);
    retcode = MCP4XCXFX_SetWiper(pdevice, wiperValues.CH6, MCP4XCXFX_ch6);
    retcode = MCP4XCXFX_SetWiper(pdevice, wiperValues.CH7, MCP4XCXFX_ch7);
    
    
    if (activate == true){
        MCP4XCXFX_wiperLatchActivate();
    }
     
    return retcode;
}       

uint16_t MCP4XCXFX_GetWiperlock_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_WLCK_REGFIELDS_P pwiperlockCtrl){
    int16_t retcode = MCP4XCXFX_SUCCESS;
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    switch (pdevice->NVtype){
        case No_NV:{
            return MCP4XCXFX_INVALID_DEVICE;
        }
        case MTP:{
            return MCP4XCXFX_Get_Register(pdevice, (void*)pwiperlockCtrl, 
                                MCP4XCXFX_WLCR_ADDR_NV, MCP4XCXFX_REG_SZ, 
                                GetWiperlock_NVReq);            
        }
        case EEPROM:{
            return MCP4XCXFX_Get_Register(pdevice, (void*)pwiperlockCtrl, 
                                MCP4XCXFX_WLST_ADDR, MCP4XCXFX_REG_SZ, 
                                GetWiperlock_NVReq); 
        }
    }
    
    return retcode;
}    //on EEPROM devices reads 0Bh register
     //on MTP devices reads the 1Bh register

static void MCP4XCXFX_GetWiperlock_NV_regProcess(MCP4XCXFX_DEVICE_CONTEXT_P pdevice){
    uint8_t* pRawValue;
    if(pdevice->if_type == MCP4XCXFX_I2C){
        pRawValue = pdevice->if_rxBuffer;
    }
    else{ //MCP4XCXFX_SPI
        pRawValue = pdevice->if_rxBuffer+1;
    }  
    MCP4XCXFX_WiperlockBytesToRegfields(pRawValue, (MCP4XCXFX_WLCK_REGFIELDS_P)pdevice->outData);
}

void MCP4XCXFX_WiperlockBytesToRegfields(uint8_t* pWiperlockBytes, MCP4XCXFX_WLCK_REGFIELDS_P pWiperlockRegfields){

    pWiperlockRegfields->WL7  =   (pWiperlockBytes[0] >> MCP4XCXFX_WLCK_CTRL_WL7_BITPOSMSB)      & MCP4XCXFX_WLCK_CTRL_WL7_BITMASK;
    pWiperlockRegfields->WL6  =   (pWiperlockBytes[0] >> MCP4XCXFX_WLCK_CTRL_WL6_BITPOSMSB)      & MCP4XCXFX_WLCK_CTRL_WL6_BITMASK;
    pWiperlockRegfields->WL5  =   (pWiperlockBytes[0] >> MCP4XCXFX_WLCK_CTRL_WL5_BITPOSMSB)      & MCP4XCXFX_WLCK_CTRL_WL5_BITMASK;
    pWiperlockRegfields->WL4  =   (pWiperlockBytes[0] /*>> MCP4XCXFX_WLCK_CTRL_WL4_BITPOSMSB*/)  & MCP4XCXFX_WLCK_CTRL_WL4_BITMASK;
    pWiperlockRegfields->WL3  =   (pWiperlockBytes[1] >> MCP4XCXFX_WLCK_CTRL_WL3_BITPOSLSB)      & MCP4XCXFX_WLCK_CTRL_WL3_BITMASK;
    pWiperlockRegfields->WL2  =   (pWiperlockBytes[1] >> MCP4XCXFX_WLCK_CTRL_WL2_BITPOSLSB)      & MCP4XCXFX_WLCK_CTRL_WL2_BITMASK;
    pWiperlockRegfields->WL1  =   (pWiperlockBytes[1] >> MCP4XCXFX_WLCK_CTRL_WL1_BITPOSLSB)      & MCP4XCXFX_WLCK_CTRL_WL1_BITMASK;    
    pWiperlockRegfields->WL0  =   (pWiperlockBytes[1] /*>> MCP4XCXFX_WLCK_CTRL_WL0_BITPOSLSB*/)  & MCP4XCXFX_WLCK_CTRL_WL0_BITMASK;
}

void MCP4XCXFX_WiperlockRegfieldsToBytes(MCP4XCXFX_WLCK_REGFIELDS WiperlockRegfields, uint8_t* pWiperlockBytes){
    //MSB
    pWiperlockBytes[0] = (uint8_t)(
                    (WiperlockRegfields.WL7 << MCP4XCXFX_WLCK_CTRL_WL7_BITPOSMSB) |
                    (WiperlockRegfields.WL6 << MCP4XCXFX_WLCK_CTRL_WL6_BITPOSMSB) |
                    (WiperlockRegfields.WL5 << MCP4XCXFX_WLCK_CTRL_WL5_BITPOSMSB) |
                    (WiperlockRegfields.WL4 /* << MCP4XCXFX_WLCK_CTRL_WL4_BITPOSMSB */ ));
    //LSB
    pWiperlockBytes[1] = (uint8_t)(
                    (WiperlockRegfields.WL3 << MCP4XCXFX_WLCK_CTRL_WL3_BITPOSLSB)       |
                    (WiperlockRegfields.WL2 << MCP4XCXFX_WLCK_CTRL_WL2_BITPOSLSB)                 |
                    (WiperlockRegfields.WL1 << MCP4XCXFX_WLCK_CTRL_WL1_BITPOSLSB) |
                    (WiperlockRegfields.WL0 /* << MCP4XCXFX_WLCK_CTRL_WL0_BITPOSLSB */ )); 
}

uint16_t MCP4XCXFX_SetWiperlock_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_WLCK_REGFIELDS wiperlockCtrl){
    int16_t retcode = MCP4XCXFX_SUCCESS;
    uint8_t pregisterBytes[MCP4XCXFX_REG_SZ];
    bool syncMode;
    
    if (pdevice == NULL) return MCP4XCXFX_INVALID_PARAMETER;
    
    switch (pdevice->NVtype){
        case No_NV:{
            return MCP4XCXFX_INVALID_DEVICE;
        }
        case MTP:{
            MCP4XCXFX_WiperlockRegfieldsToBytes(wiperlockCtrl, pregisterBytes);
            return MCP4XCXFX_Set_Register(pdevice, pregisterBytes, 
                                MCP4XCXFX_WLCR_ADDR_NV, MCP4XCXFX_REG_SZ, 
                                SetRegisterReq);
        }
        case EEPROM:{
            syncMode = pdevice->syncMode;
            pdevice->syncMode = true;
            for (int i = 0; i <= (pdevice->channels - 1); i++) {
                uint8_t dl_lock, cl_lock;
                uint8_t wl_value;
                uint8_t dl_bit, cl_bit;

                switch (i) {
                    case 7:
                        wl_value = wiperlockCtrl.WL7;
                        dl_bit = DL7_BIT;
                        cl_bit = CL7_BIT;
                        break;
                    case 6:
                        wl_value = wiperlockCtrl.WL6;
                        dl_bit = DL6_BIT;
                        cl_bit = CL6_BIT;
                        break;
                    case 5:
                        wl_value = wiperlockCtrl.WL5;
                        dl_bit = DL5_BIT;
                        cl_bit = CL5_BIT;
                        break;
                    case 4:
                        wl_value = wiperlockCtrl.WL4;
                        dl_bit = DL4_BIT;
                        cl_bit = CL4_BIT;
                        break;
                    case 3:
                        wl_value = wiperlockCtrl.WL3;
                        dl_bit = DL3_BIT;
                        cl_bit = CL3_BIT;
                        break;
                    case 2:
                        wl_value = wiperlockCtrl.WL2;
                        dl_bit = DL2_BIT;
                        cl_bit = CL2_BIT;
                        break;
                    case 1:
                        wl_value = wiperlockCtrl.WL1;
                        dl_bit = DL1_BIT;
                        cl_bit = CL1_BIT;
                        break;
                    case 0:
                        wl_value = wiperlockCtrl.WL0;
                        dl_bit = DL0_BIT;
                        cl_bit = CL0_BIT;
                        break;
                    default:
                        continue;
                }

                switch (wl_value) {
                    case MCP4XCXFX_WLCK_CTRL_WR_LOCKED_DC_LOCKED:
                        dl_lock = MCP4XCXFX_LCKbit_locked;
                        cl_lock = MCP4XCXFX_LCKbit_locked;
                        break;
                    case MCP4XCXFX_WLCK_CTRL_WR_LOCKED_DC_UNLOCKED:
                        dl_lock = MCP4XCXFX_LCKbit_locked;
                        cl_lock = MCP4XCXFX_LCKbit_unlocked;
                        break;
                    case MCP4XCXFX_WLCK_CTRL_WR_UNLOCKED_DC_LOCKED:
                        dl_lock = MCP4XCXFX_LCKbit_unlocked;
                        cl_lock = MCP4XCXFX_LCKbit_locked;
                        break;
                    default:
                        dl_lock = MCP4XCXFX_LCKbit_unlocked;
                        cl_lock = MCP4XCXFX_LCKbit_unlocked;
                        break;
                }
                                
                retcode = MCP4XCXFX_ConfigBitUpdate(pdevice, dl_bit, dl_lock);
                retcode = MCP4XCXFX_ConfigBitUpdate(pdevice, cl_bit, cl_lock);                
            }
            pdevice->syncMode = syncMode;
            return retcode;
        }
    }
    
    return retcode;
}      //on EEPROM devices writes CL:DL bits