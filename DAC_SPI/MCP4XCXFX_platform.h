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

#ifndef MCP4XCXFX_PLATFORM_H
#define MCP4XCXFX_PLATFORM_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

/*
 *   Section: Included Files
 */
#include <stdint.h>
#include <stdbool.h>

// In this example, the included header file "osal.h" provides the MUTEX support 
// from the MCHP Harmony OSAL (Operating Abstraction Layer).
//"plib_systick.h" provides timing functions
//"plib_port.h" provides GPIO control functions    
 #include "osal/osal.h" 
 #include "config/default/peripheral/systick/plib_systick.h"
 #include "config/default/peripheral/port/plib_port.h"
// or define the following data types and constants
typedef uint8_t OSAL_MUTEX_HANDLE_TYPE;

/**
 * @brief Delays execution for a specified number of milliseconds.
 * 
 * @param delay_ms Number of milliseconds to delay.
 * @attention Platform specific.
 */
static inline void MCP4XCXFX_delay_ms(uint32_t delay_ms) {

    SYSTICK_DelayMs(delay_ms);

    return;
}

/**
 * @brief Delays execution for a specified number of microseconds.
 * 
 * @param delay_us Number of microseconds to delay.
 * @attention Platform specific.
 */
static inline void MCP4XCXFX_delay_us(uint32_t delay_us) {

    SYSTICK_DelayUs(delay_us);

    return;
}

/** 
 * @typedef MCP4XCXFX_MUTEX
 * The MCP4XCXFX library uses a MUTEX object to avoid simultaneous access to device
 * data structures from parallel processing threads. 
 * The MUTEX object type may differ on different platforms.  
 * @attention This type definition must be updated for the target platform .
 */
typedef OSAL_MUTEX_HANDLE_TYPE  MCP4XCXFX_MUTEX;

/* MUTEX support functions */

/**
 * @brief The function creates the MUTEX object
 * 
 * If the user application is single-threaded, the function may just return "true".
 * @param pmutex Pointer to the mutex object
 * @return true if mutex object was created, false otherwise.
 */
static inline bool MCP4XCXFX_MUTEX_Create(MCP4XCXFX_MUTEX *pmutex){

    if( OSAL_MUTEX_Create(pmutex) != OSAL_RESULT_TRUE )
        return false;
    else
        return true;

    return true;
}

/**
 * @brief The function locks the MUTEX.
 * 
 * If the user application is single-threaded, the function may just return "true".
 * @param pmutex Pointer to the mutex object
 * @return true if mutex was locked, false otherwise.
 */
static inline bool MCP4XCXFX_MUTEX_Lock(MCP4XCXFX_MUTEX *pmutex){

    if( OSAL_MUTEX_Lock(pmutex, OSAL_WAIT_FOREVER) == OSAL_RESULT_FALSE )
        return false;
    else
        return true;

    return true;
}

/**
 * @brief The function un-locks the MUTEX.
 * 
 * @param pmutex Pointer to the mutex object
 */
static inline void MCP4XCXFX_MUTEX_Unlock(MCP4XCXFX_MUTEX *pmutex){

    OSAL_MUTEX_Unlock(pmutex);

    return;
}

/**
 * @brief Activates the wiper latch to apply new wiper data.
 *
 * Enables high voltage source and disables HVC/LATn to apply new wiper data to the device.
 * This function manipulates the relevant GPIO pins to trigger the latch mechanism.
 * @attention Platform specific.
 */
static inline void MCP4XCXFX_wiperLatchActivate(){
    //LAT = 0 HV = 1 to apply new wiper data
    GPIO_PA04_Set();    //HV = 1
    GPIO_PB13_Clear();  //LAT = 0
}

/**
 * @brief Deactivates the wiper latch to preserve current configuration.
 *
 * Enables both high voltage source and HVC/LATn to prevent new wiper data from being applied,
 * preserving the current device configuration.
 * This function manipulates the relevant GPIO pins to deactivate the latch.
 * @attention Platform specific.
 */
static inline void MCP4XCXFX_wiperLatchDeactivate(){
    //LAT = 1 HV = 1 to keep data from applying (to preserve current configuration)    
    GPIO_PA04_Set();  //HV = 1
    GPIO_PB13_Set();  //LAT = 1 
}

/**
 * @brief Enables non-volatile memory write access (HVC handling).
 *
 * Disables high voltage source and sets the HVC/LATn pin to enable write access to non-volatile memory.
 * Required by "Set__NV" functions on MTP devices or "set__Lock" functions on EEPROM devices.
 * This function manipulates the relevant GPIO pins to enable writing.
 * @attention Platform specific.
 */
static inline void MCP4XCXFX_writeEnable_NV(){
    
    GPIO_PA04_Clear(); //HV = 0
    GPIO_PB13_Set();   //LAT = 1 
}

/**
 * @brief Disables non-volatile memory write access (HVC handling).
 *
 * Waits for the required write cycle time, then enables the high voltage source and clears the HVC/LATn to disable write access.
 * Required by "Set__NV" functions on MTP devices or "set__Lock" functions on EEPROM devices.
 * This function manipulates the relevant GPIO pins and includes a delay to meet timing requirements.
 * @attention Platform specific.
 */
static inline void MCP4XCXFX_writeDisable_NV(){
    
    MCP4XCXFX_delay_us(250); //twc = 250 us from datasheet
    GPIO_PA04_Set();    //HV = 1
    GPIO_PB13_Clear();  //LAT = 0
}

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	// _MCP4XCXFX_PLATFORM_H

/**
  End of File
*/
