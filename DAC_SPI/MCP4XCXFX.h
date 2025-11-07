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


#ifndef MCP4XCXFX_H
#define MCP4XCXFX_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

// MCP4XCXFX generic library version
#define MCP4XCXFX_LIBVER "1.0"

/*
 * Included Files
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "MCP4XCXFX_i2c.h"
#include "MCP4XCXFX_spi.h" //maybe better to be moved to platform.h
#include "MCP4XCXFX_platform.h"

/** @defgroup MCP4XCXFX_ErrorCodes Error Codes
 *  @brief Error and status codes returned by the MCP4XCXFX library API.
 *  @{
 */
//Return codes from the public API
#define MCP4XCXFX_SUCCESS                               0 /**< The API request or function call is completed with no error. */
#define MCP4XCXFX_BUSY                                  1 /**< The new request is rejected while the library is busy processing the previous request. */    
#define MCP4XCXFX_REQUEST_ABORT                         2 /**< The API return code indicates that the pending request processing has been aborted. */
#define MCP4XCXFX_REQUEST_PENDING                       3 /**< The API return code indicates that the request processing has been started. 
                                                             The processing completion will be signaled (asynchronously) by the user callback function
                                                             registered with MCP4XCXFX_SetUserCallback(), 
                                                             or determined by polling the processing status with MCP4XCXFX_GetEventStatus() */

#define MCP4XCXFX_COMMUNICATION_FAIL                    4 /**< The API request processing failed because of I2C communication failure. */
#define MCP4XCXFX_MUTEX_FAIL                            5 /**< MCP4XCXFX_Device_Initialize() failed to create the mutex. */
#define MCP4XCXFX_INVALID_PARAMETER                     6 /**< The request is rejected because of incorrect or invalid parameter. */
#define MCP4XCXFX_INVALID_DEVICE                        7 /**< MCP4XCXFX_Device_Initialize() failed to recognize the MCP4XCXFX device ID. */
#define MCP4XCXFX_LIBTASK_FAIL                          8 /**< MCP4XCXFX_Libtask()function call failed. */
#define MCP4XCXFX_LIBTASK_DONE                          0 /**< MCP4XCXFX_Libtask()function call completed with no error. */

//Non-public return codes from non-public API (used internally)
#define MCP4XCXFX_ALREADY_CACHED                     1000 /**< Internal only return code used by non-public API. 
                                                             Not returned to user application by the public API. */

/** @} */
/** @defgroup MCP4XCXFX_Registers Register Addresses and Constants
 *  @brief Register addresses and constants for MCP4XCXFX devices.
 *  @{
 */
// Volatile register addresses
#define MCP4XCXFX_DAC0_ADDR             0x00 /**< DAC0 register address. */
#define MCP4XCXFX_DAC1_ADDR             0x01 /**< DAC1 register address. */
#define MCP4XCXFX_DAC2_ADDR             0x02 /**< DAC2 register address. */
#define MCP4XCXFX_DAC3_ADDR             0x03 /**< DAC3 register address. */
#define MCP4XCXFX_DAC4_ADDR             0x04 /**< DAC4 register address. */
#define MCP4XCXFX_DAC5_ADDR             0x05 /**< DAC5 register address. */
#define MCP4XCXFX_DAC6_ADDR             0x06 /**< DAC6 register address. */
#define MCP4XCXFX_DAC7_ADDR             0x07 /**< DAC7 register address. */
#define MCP4XCXFX_VREF_ADDR             0x08 /**< VREF register address. */
#define MCP4XCXFX_PWDN_ADDR             0x09 /**< Power-down register address. */
#define MCP4XCXFX_GCST_ADDR             0x0A /**< Gain Control / Status Register address. */
#define MCP4XCXFX_WLST_ADDR             0x0B /**< Wiper Lock Status register address (EEPROM only devices). */

// Non-volatile register addresses
#define MCP4XCXFX_DAC0_ADDR_NV          0x10 /**< DAC0 non-volatile register address. */
#define MCP4XCXFX_DAC1_ADDR_NV          0x11 /**< DAC1 non-volatile register address. */
#define MCP4XCXFX_DAC2_ADDR_NV          0x12 /**< DAC2 non-volatile register address. */
#define MCP4XCXFX_DAC3_ADDR_NV          0x13 /**< DAC3 non-volatile register address. */
#define MCP4XCXFX_DAC4_ADDR_NV          0x14 /**< DAC4 non-volatile register address. */
#define MCP4XCXFX_DAC5_ADDR_NV          0x15 /**< DAC5 non-volatile register address. */
#define MCP4XCXFX_DAC6_ADDR_NV          0x16 /**< DAC6 non-volatile register address. */
#define MCP4XCXFX_DAC7_ADDR_NV          0x17 /**< DAC7 non-volatile register address. */
#define MCP4XCXFX_VREF_ADDR_NV          0x18 /**< VREF non-volatile register address. */
#define MCP4XCXFX_PWDN_ADDR_NV          0x19 /**< Power-down non-volatile register address. */
#define MCP4XCXFX_GCAD_ADDR_NV          0x1A /**< Gain Control / Client Address register (NV). */
#define MCP4XCXFX_WLCR_ADDR_NV          0x1B /**< WiperLock Control Register (MTP only devices). */

// MTP only devices
#define MCP4XCXFX_MTP0C_ADDR_NV         0x0C /**< MTP0C non-volatile register address. */
#define MCP4XCXFX_MTP0D_ADDR_NV         0x0D /**< MTP0D non-volatile register address. */
#define MCP4XCXFX_MTP0E_ADDR_NV         0x0E /**< MTP0E non-volatile register address. */
#define MCP4XCXFX_MTP0F_ADDR_NV         0x0F /**< MTP0F non-volatile register address. */
#define MCP4XCXFX_MTP1C_ADDR_NV         0x1C /**< MTP1C non-volatile register address. */
#define MCP4XCXFX_MTP1D_ADDR_NV         0x1D /**< MTP1D non-volatile register address. */
#define MCP4XCXFX_MTP1E_ADDR_NV         0x1E /**< MTP1E non-volatile register address. */
#define MCP4XCXFX_MTP1F_ADDR_NV         0x1F /**< MTP1F non-volatile register address. */

//MCP4XCXFX maximum transmission and reception data byte-count
#define MCP4XCXFX_INTERFACE_TX_MAXSIZE  3  /**< The maximum data size to write TO device in one I2C or SPI transfer. */
#define MCP4XCXFX_INTERFACE_RX_MAXSIZE  3  /**< The maximum data size to read FROM device in one I2C or SPI transfer. */

//General Call commands
#define MCP4XCXFX_GEN_CALL_RESET    0x06
#define MCP4XCXFX_GEN_CALL_WAKEUP   0x0A
#define MCP4XCXFX_GEN_CALL_ADDR     0x00
        
//Register sizes
#define MCP4XCXFX_WIPER_SZ       2
#define MCP4XCXFX_REG_SZ         2
#define MCP4XCXFX_GEN_CALL_SZ    1
    
/** @} */
    
/** @defgroup MCP4XCXFX_Types Type Definitions
 *  @brief Enumerations and structures for MCP4XCXFX devices.
 *  @{
 */

/**
 * @brief Enumeration of MCP4XCXFX DAC device IDs.
 */
typedef enum {
// Enumeration of MCP4XCXFX DAC devices with I2C communication interface(60 devices)
    MCP47FEB01  = 1000,
    MCP47FEB02,
    MCP47FEB04,
    MCP47FEB08,
    MCP47FEB11,
    MCP47FEB12,
    MCP47FEB14,
    MCP47FEB18,
    MCP47FEB21,
    MCP47FEB22,
    MCP47FEB24,
    MCP47FEB28,
    MCP47FVB01,
    MCP47FVB02,
    MCP47FVB04,
    MCP47FVB08,
    MCP47FVB11,
    MCP47FVB12,
    MCP47FVB14,
    MCP47FVB18,
    MCP47FVB21,
    MCP47FVB22,
    MCP47FVB24,
    MCP47FVB28,
    MCP47CMB01,
    MCP47CMB02,
    MCP47CMB04,
    MCP47CMB08,
    MCP47CMB11,
    MCP47CMB12,
    MCP47CMB14,
    MCP47CMB18,
    MCP47CMB21,
    MCP47CMB22,
    MCP47CMB24,
    MCP47CMB28,
    MCP47CVB01,
    MCP47CVB02,
    MCP47CVB04,
    MCP47CVB08,
    MCP47CVB11,
    MCP47CVB12,
    MCP47CVB14,
    MCP47CVB18,
    MCP47CVB21,
    MCP47CVB22,
    MCP47CVB24,
    MCP47CVB28,
    MCP47CMD01,
    MCP47CMD02,
    MCP47CMD11,
    MCP47CMD12,
    MCP47CMD21,
    MCP47CMD22,
    MCP47CVD01,
    MCP47CVD02,
    MCP47CVD11,
    MCP47CVD12,
    MCP47CVD21,
    MCP47CVD22,
    
// Enumeration of MCP4XCXFX DAC devices with SPI communication interface(60 devices)
    MCP48FEB01  = 2000,
    MCP48FEB02,
    MCP48FEB04,
    MCP48FEB08,
    MCP48FEB11,
    MCP48FEB12,
    MCP48FEB14,
    MCP48FEB18,
    MCP48FEB21,
    MCP48FEB22,
    MCP48FEB24,
    MCP48FEB28,
    MCP48FVB01,
    MCP48FVB02,
    MCP48FVB04,
    MCP48FVB08,
    MCP48FVB11,
    MCP48FVB12,
    MCP48FVB14,
    MCP48FVB18,
    MCP48FVB21,
    MCP48FVB22,
    MCP48FVB24,
    MCP48FVB28,
    MCP48CMB01,
    MCP48CMB02,
    MCP48CMB04,
    MCP48CMB08,
    MCP48CMB11,
    MCP48CMB12,
    MCP48CMB14,
    MCP48CMB18,
    MCP48CMB21,
    MCP48CMB22,
    MCP48CMB24,
    MCP48CMB28,
    MCP48CVB01,
    MCP48CVB02,
    MCP48CVB04,
    MCP48CVB08,
    MCP48CVB11,
    MCP48CVB12,
    MCP48CVB14,
    MCP48CVB18,
    MCP48CVB21,
    MCP48CVB22,
    MCP48CVB24,
    MCP48CVB28,
    MCP48CMD01,
    MCP48CMD02,
    MCP48CMD11,
    MCP48CMD12,
    MCP48CMD21,
    MCP48CMD22,
    MCP48CVD01,
    MCP48CVD02,
    MCP48CVD11,
    MCP48CVD12,
    MCP48CVD21,
    MCP48CVD22
} MCP4XCXFX_device_ID;

/**
 * @brief Communication interface type.
 */
typedef enum{
    MCP4XCXFX_I2C = 0, /**< I2C interface. */
    MCP4XCXFX_SPI = 1  /**< SPI interface. */
} MCP4XCXFX_INTERFACE_TYPE;

/**
 * @brief Enumeration of MCP4XCXFX register addresses.
 */
typedef enum{
    MCP4XCXFX_DAC0_REG = MCP4XCXFX_DAC0_ADDR,
    MCP4XCXFX_DAC1_REG = MCP4XCXFX_DAC1_ADDR,
    MCP4XCXFX_DAC2_REG = MCP4XCXFX_DAC2_ADDR,
    MCP4XCXFX_DAC3_REG = MCP4XCXFX_DAC3_ADDR,
    MCP4XCXFX_DAC4_REG = MCP4XCXFX_DAC4_ADDR,
    MCP4XCXFX_DAC5_REG = MCP4XCXFX_DAC5_ADDR,
    MCP4XCXFX_DAC6_REG = MCP4XCXFX_DAC6_ADDR,
    MCP4XCXFX_DAC7_REG = MCP4XCXFX_DAC7_ADDR,
    MCP4XCXFX_VREF_REG = MCP4XCXFX_VREF_ADDR,
    MCP4XCXFX_PWDN_REG = MCP4XCXFX_PWDN_ADDR,
    MCP4XCXFX_GCST_REG = MCP4XCXFX_GCST_ADDR,
    MCP4XCXFX_WLST_REG = MCP4XCXFX_WLST_ADDR,

    MCP4XCXFX_DAC0_NV_REG = MCP4XCXFX_DAC0_ADDR_NV,
    MCP4XCXFX_DAC1_NV_REG = MCP4XCXFX_DAC1_ADDR_NV,
    MCP4XCXFX_DAC2_NV_REG = MCP4XCXFX_DAC2_ADDR_NV,
    MCP4XCXFX_DAC3_NV_REG = MCP4XCXFX_DAC3_ADDR_NV,
    MCP4XCXFX_DAC4_NV_REG = MCP4XCXFX_DAC4_ADDR_NV,
    MCP4XCXFX_DAC5_NV_REG = MCP4XCXFX_DAC5_ADDR_NV,
    MCP4XCXFX_DAC6_NV_REG = MCP4XCXFX_DAC6_ADDR_NV,
    MCP4XCXFX_DAC7_NV_REG = MCP4XCXFX_DAC7_ADDR_NV,
    MCP4XCXFX_VREF_NV_REG = MCP4XCXFX_VREF_ADDR_NV,
    MCP4XCXFX_PWDN_NV_REG = MCP4XCXFX_PWDN_ADDR_NV,
    MCP4XCXFX_GCAD_NV_REG = MCP4XCXFX_GCAD_ADDR_NV,
    MCP4XCXFX_WLCR_NV_REG = MCP4XCXFX_WLCR_ADDR_NV,

    //MTP only devices
    MCP4XCXFX_MTP0C_NV_REG = MCP4XCXFX_MTP0C_ADDR_NV,
    MCP4XCXFX_MTP0D_NV_REG = MCP4XCXFX_MTP0D_ADDR_NV,
    MCP4XCXFX_MTP0E_NV_REG = MCP4XCXFX_MTP0E_ADDR_NV,
    MCP4XCXFX_MTP0F_NV_REG = MCP4XCXFX_MTP0F_ADDR_NV,
    MCP4XCXFX_MTP1C_NV_REG = MCP4XCXFX_MTP1C_ADDR_NV,
    MCP4XCXFX_MTP1D_NV_REG = MCP4XCXFX_MTP1D_ADDR_NV,
    MCP4XCXFX_MTP1E_NV_REG = MCP4XCXFX_MTP1E_ADDR_NV,
    MCP4XCXFX_MTP1F_NV_REG = MCP4XCXFX_MTP1F_ADDR_NV
} MCP4XCXFX_regAddr;

/**
 * @brief Enumeration of WIPERLOCK configuration bit addresses (EEPROM only).
 */
typedef enum {
    CL0_BIT = MCP4XCXFX_DAC0_ADDR,
    CL1_BIT = MCP4XCXFX_DAC1_ADDR,
    CL2_BIT = MCP4XCXFX_DAC2_ADDR,
    CL3_BIT = MCP4XCXFX_DAC3_ADDR,
    CL4_BIT = MCP4XCXFX_DAC4_ADDR,
    CL5_BIT = MCP4XCXFX_DAC5_ADDR,
    CL6_BIT = MCP4XCXFX_DAC6_ADDR,
    CL7_BIT = MCP4XCXFX_DAC7_ADDR,

    DL0_BIT = MCP4XCXFX_DAC0_ADDR_NV,
    DL1_BIT = MCP4XCXFX_DAC1_ADDR_NV,
    DL2_BIT = MCP4XCXFX_DAC2_ADDR_NV,
    DL3_BIT = MCP4XCXFX_DAC3_ADDR_NV,
    DL4_BIT = MCP4XCXFX_DAC4_ADDR_NV,
    DL5_BIT = MCP4XCXFX_DAC5_ADDR_NV,
    DL6_BIT = MCP4XCXFX_DAC6_ADDR_NV,
    DL7_BIT = MCP4XCXFX_DAC7_ADDR_NV,
    
    SALCK_BIT = MCP4XCXFX_GCAD_ADDR_NV

} MCP4XCXFX_LCKcfgBitAddr;

/**
 * @brief DAC resolution options.
 */
typedef enum {
    MCP4XCXFX_8bit  =  8, /**< 8-bit resolution. */
    MCP4XCXFX_10bit = 10, /**< 10-bit resolution. */
    MCP4XCXFX_12bit = 12  /**< 12-bit resolution. */
} MCP4XCXFX_device_resolution;

/**
 * @brief WIPERLOCK bit status options.
 */
typedef enum {
    MCP4XCXFX_LCKbit_unlocked = 0, /**< Bit unlocked. */
    MCP4XCXFX_LCKbit_locked   = 1  /**< Bit locked. */
} MCP4XCXFX_LCKcfgBit;

/**
 * @brief Number of channels options.
 */
typedef enum {
    MCP4XCXFX_1ch   = 1, /**< 1 channel. */
    MCP4XCXFX_2ch   = 2, /**< 2 channels. */
    MCP4XCXFX_4ch   = 4, /**< 4 channels. */
    MCP4XCXFX_8ch   = 8  /**< 8 channels. */
} MCP4XCXFX_device_channelCount;

/**
 * @brief Non-volatile memory type.
 */
typedef enum{
    No_NV = 0, /**< No non-volatile memory. */
    MTP = 1,   /**< MTP type. */
    EEPROM = 2 /**< EEPROM type. */
} MCP4XCXFX_device_NVtype;

/**
 * @brief Channel number selection.
 */
typedef enum {
    MCP4XCXFX_ch0   = 0, /**< Channel 0. */
    MCP4XCXFX_ch1   = 1, /**< Channel 1. */
    MCP4XCXFX_ch2   = 2, /**< Channel 2. */
    MCP4XCXFX_ch3   = 3, /**< Channel 3. */
    MCP4XCXFX_ch4   = 4, /**< Channel 4. */
    MCP4XCXFX_ch5   = 5, /**< Channel 5. */
    MCP4XCXFX_ch6   = 6, /**< Channel 6. */
    MCP4XCXFX_ch7   = 7  /**< Channel 7. */
} MCP4XCXFX_device_channelNo;

/**
 * @brief General call command enumeration.
 */
typedef enum {
    MCP4XCXFX_I2C_reset_cmd = 3,   /**< I2C General Call Reset. */
    MCP4XCXFX_I2C_wakeup_cmd = 5   /**< I2C General Call Wakeup. */
} MCP4XCXFX_Gen_call_cmd;

/**
 * @brief Gain control options.
 */
typedef enum {
     MCP4XCXFX_GC1x = 0, /**< Gain 1x. */
     MCP4XCXFX_GC2x = 1  /**< Gain 2x. */
}  MCP4XCXFX_gainCtrlBit; 

/**
 * @brief Wiperlock control register bit-field values.
 */
typedef enum {
    MCP4XCXFX_WLCK_CTRL_WR_LOCKED_DC_LOCKED = 0b11,
    MCP4XCXFX_WLCK_CTRL_WR_LOCKED_DC_UNLOCKED = 0b10,
    MCP4XCXFX_WLCK_CTRL_WR_UNLOCKED_DC_LOCKED = 0b01,
    MCP4XCXFX_WLCK_CTRL_WR_UNLOCKEDDC_UNLOCKED = 0b00
} MCP4XCXFX_WLCKValues;
/** @} */

/** @defgroup MCP4XCXFX_Structs Register and Device Structures
 *  @brief Register bitfield and device context structures.
 *  @{
 */

/**
 * @brief VREF Control Register structure.
 */
typedef struct _MCP4XCXFX_VREF_REGFIELDS {
    uint8_t VR7 :2; /**< VR7B:VR7A */
    uint8_t VR6 :2; /**< VR6B:VR6A */
    uint8_t VR5 :2; /**< VR5B:VR5A */
    uint8_t VR4 :2; /**< VR4B:VR4A */
    uint8_t VR3 :2; /**< VR3B:VR3A */
    uint8_t VR2 :2; /**< VR2B:VR2A */
    uint8_t VR1 :2; /**< VR1B:VR1A */
    uint8_t VR0 :2; /**< VR0B:VR0A */
} MCP4XCXFX_VREF_REGFIELDS, *MCP4XCXFX_VREF_REGFIELDS_P;

/* VREF CTRL register bit-field positions */
#define MCP4XCXFX_VREF_CTRL_VR7_BITPOSMSB   6 /**< VR7 field position in the VREF CTRL register MSB */
#define MCP4XCXFX_VREF_CTRL_VR6_BITPOSMSB   4 /**< VR6 field position in the VREF CTRL register MSB */
#define MCP4XCXFX_VREF_CTRL_VR5_BITPOSMSB   2 /**< VR5 field position in the VREF CTRL register MSB */
#define MCP4XCXFX_VREF_CTRL_VR4_BITPOSMSB   0 /**< VR4 field position in the VREF CTRL register MSB */
#define MCP4XCXFX_VREF_CTRL_VR3_BITPOSLSB   6 /**< VR3 field position in the VREF CTRL register LSB */
#define MCP4XCXFX_VREF_CTRL_VR2_BITPOSLSB   4 /**< VR2 field position in the VREF CTRL register LSB */
#define MCP4XCXFX_VREF_CTRL_VR1_BITPOSLSB   2 /**< VR1 field position in the VREF CTRL register LSB */
#define MCP4XCXFX_VREF_CTRL_VR0_BITPOSLSB   0 /**< VR0 field position in the VREF CTRL register LSB */
/* VREF CTRL register bit-field masks */
#define MCP4XCXFX_VREF_CTRL_VR7_BITMASK   0x03
#define MCP4XCXFX_VREF_CTRL_VR6_BITMASK   0x03
#define MCP4XCXFX_VREF_CTRL_VR5_BITMASK   0x03
#define MCP4XCXFX_VREF_CTRL_VR4_BITMASK   0x03
#define MCP4XCXFX_VREF_CTRL_VR3_BITMASK   0x03
#define MCP4XCXFX_VREF_CTRL_VR2_BITMASK   0x03
#define MCP4XCXFX_VREF_CTRL_VR1_BITMASK   0x03
#define MCP4XCXFX_VREF_CTRL_VR0_BITMASK   0x03
/* VREF CTRL register bit-field values */
#define MCP4XCXFX_VREF_CTRL_PIN_BUFF_ENABLED     0b11
#define MCP4XCXFX_VREF_CTRL_PIN_BUFF_DISABLED    0b10
#define MCP4XCXFX_VREF_CTRL_INT_BUFF_ENABLED     0b11
#define MCP4XCXFX_VREF_CTRL_VDD_BUFF_DISABLED    0b11

/**
 * @brief PWDN Control Register structure.
 */ 
typedef struct _MCP4XCXFX_PWDN_REGFIELDS {
    /* register MSB */
    uint8_t PD7 :2; /**< PD7B:PD7A */
    uint8_t PD6 :2; /**< PD6B:PD6A */
    uint8_t PD5 :2; /**< PD5B:PD5A */
    uint8_t PD4 :2; /**< PD4B:PD4A */
    /* register LSB */
    uint8_t PD3 :2; /**< PD3B:PD3A */
    uint8_t PD2 :2; /**< PD2B:PD2A */
    uint8_t PD1 :2; /**< PD1B:PD1A */
    uint8_t PD0 :2; /**< PD0B:PD0A */
} MCP4XCXFX_PWDN_REGFIELDS, *MCP4XCXFX_PWDN_REGFIELDS_P;

/* PWDN CTRL register bit-field positions */
#define MCP4XCXFX_PWDN_CTRL_PD7_BITPOSMSB   6 /**< PD7 field position in the PWDN register MSB */
#define MCP4XCXFX_PWDN_CTRL_PD6_BITPOSMSB   4 /**< PD6 field position in the PWDN register MSB */
#define MCP4XCXFX_PWDN_CTRL_PD5_BITPOSMSB   2 /**< PD5 field position in the PWDN register MSB */
#define MCP4XCXFX_PWDN_CTRL_PD4_BITPOSMSB   0 /**< PD4 field position in the PWDN register MSB */
#define MCP4XCXFX_PWDN_CTRL_PD3_BITPOSLSB   6 /**< PD3 field position in the PWDN register LSB */
#define MCP4XCXFX_PWDN_CTRL_PD2_BITPOSLSB   4 /**< PD2 field position in the PWDN register LSB */
#define MCP4XCXFX_PWDN_CTRL_PD1_BITPOSLSB   2 /**< PD1 field position in the PWDN register LSB */
#define MCP4XCXFX_PWDN_CTRL_PD0_BITPOSLSB   0 /**< PD0 field position in the PWDN register LSB */
/* PWDN CTRL register bit-field masks */
#define MCP4XCXFX_PWDN_CTRL_PD7_BITMASK   0x03
#define MCP4XCXFX_PWDN_CTRL_PD6_BITMASK   0x03
#define MCP4XCXFX_PWDN_CTRL_PD5_BITMASK   0x03
#define MCP4XCXFX_PWDN_CTRL_PD4_BITMASK   0x03
#define MCP4XCXFX_PWDN_CTRL_PD3_BITMASK   0x03
#define MCP4XCXFX_PWDN_CTRL_PD2_BITMASK   0x03
#define MCP4XCXFX_PWDN_CTRL_PD1_BITMASK   0x03
#define MCP4XCXFX_PWDN_CTRL_PD0_BITMASK   0x03
/* PWDN CTRL register bit-field values */
#define MCP4XCXFX_PWDN_CTRL_PD_VOUT_OC   0b11
#define MCP4XCXFX_PWDN_CTRL_PD_VOUT_100K 0b10
#define MCP4XCXFX_PWDN_CTRL_PD_VOUT_1K   0b11
#define MCP4XCXFX_PWDN_CTRL_NON_PD       0b11

/**
 * @brief Gain Control and System Status Register structure.
 */
typedef struct _MCP4XCXFX_GCST_REGFIELDS {
    /* register MSB */
    uint8_t G7  :1; /**< Gain bit 7 */
    uint8_t G6  :1; /**< Gain bit 6 */
    uint8_t G5  :1; /**< Gain bit 5 */
    uint8_t G4  :1; /**< Gain bit 4 */
    uint8_t G3  :1; /**< Gain bit 3 */
    uint8_t G2  :1; /**< Gain bit 2 */
    uint8_t G1  :1; /**< Gain bit 1 */
    uint8_t G0  :1; /**< Gain bit 0 */
    /* register LSB */
    uint8_t POR :1; /**< Power-on reset status */
    uint8_t NV  :1; /**< Non-volatile status - EEWA on EEPROM (write cycle) devices, MTPMA (read/write cycle) on MTP devices*/
    uint8_t     :6; /**< Reserved */
} MCP4XCXFX_GCST_REGFIELDS, *MCP4XCXFX_GCST_REGFIELDS_P;

/* GCST register bit-field positions */
#define MCP4XCXFX_GCST_G7_BITPOSMSB    7 /**< G7 field position in the GCST register MSB */
#define MCP4XCXFX_GCST_G6_BITPOSMSB    6 /**< G6 field position in the GCST register MSB */
#define MCP4XCXFX_GCST_G5_BITPOSMSB    5 /**< G5 field position in the GCST register MSB */
#define MCP4XCXFX_GCST_G4_BITPOSMSB    4 /**< G4 field position in the GCST register MSB */
#define MCP4XCXFX_GCST_G3_BITPOSMSB    3 /**< G3 field position in the GCST register MSB */
#define MCP4XCXFX_GCST_G2_BITPOSMSB    2 /**< G2 field position in the GCST register MSB */
#define MCP4XCXFX_GCST_G1_BITPOSMSB    1 /**< G1 field position in the GCST register MSB */
#define MCP4XCXFX_GCST_G0_BITPOSMSB    0 /**< G0 field position in the GCST register MSB */
#define MCP4XCXFX_GCST_POR_BITPOSLSB   7 /**< POR field position in the GCST register LSB */
#define MCP4XCXFX_GCST_NV_BITPOSLSB    6 /**< NV  field position in the GCST register LSB */

/* GCST register bit-field masks */
#define MCP4XCXFX_GCST_G7_BITMASK   0x01
#define MCP4XCXFX_GCST_G6_BITMASK   0x01
#define MCP4XCXFX_GCST_G5_BITMASK   0x01
#define MCP4XCXFX_GCST_G4_BITMASK   0x01
#define MCP4XCXFX_GCST_G3_BITMASK   0x01
#define MCP4XCXFX_GCST_G2_BITMASK   0x01
#define MCP4XCXFX_GCST_G1_BITMASK   0x01
#define MCP4XCXFX_GCST_G0_BITMASK   0x01
#define MCP4XCXFX_GCST_POR_BITMASK  0x01
#define MCP4XCXFX_GCST_NV_BITMASK   0x01

/**
 * @brief Gain Control and Client Address Register structure.
 */
typedef struct _MCP4XCXFX_GCAD_REGFIELDS {
    /* register MSB */
    uint8_t G7  :1; /**< Gain bit 7 */
    uint8_t G6  :1; /**< Gain bit 6 */
    uint8_t G5  :1; /**< Gain bit 5 */
    uint8_t G4  :1; /**< Gain bit 4 */
    uint8_t G3  :1; /**< Gain bit 3 */
    uint8_t G2  :1; /**< Gain bit 2 */
    uint8_t G1  :1; /**< Gain bit 1 */
    uint8_t G0  :1; /**< Gain bit 0 */
    /* register LSB */
    uint8_t ADLCK       :1; /**< Address lock bit */
    uint8_t ADDR_I2C    :7; /**< 7-bit I2C address */
} MCP4XCXFX_GCAD_REGFIELDS, *MCP4XCXFX_GCAD_REGFIELDS_P;

/* GCAD register bit-field positions */
#define MCP4XCXFX_GCAD_G7_BITPOSMSB    7 /**< G7 field position in the GCAD register MSB */
#define MCP4XCXFX_GCAD_G6_BITPOSMSB    6 /**< G6 field position in the GCAD register MSB */
#define MCP4XCXFX_GCAD_G5_BITPOSMSB    5 /**< G5 field position in the GCAD register MSB */
#define MCP4XCXFX_GCAD_G4_BITPOSMSB    4 /**< G4 field position in the GCAD register MSB */
#define MCP4XCXFX_GCAD_G3_BITPOSMSB    3 /**< G3 field position in the GCAD register MSB */
#define MCP4XCXFX_GCAD_G2_BITPOSMSB    2 /**< G2 field position in the GCAD register MSB */
#define MCP4XCXFX_GCAD_G1_BITPOSMSB    1 /**< G1 field position in the GCAD register MSB */
#define MCP4XCXFX_GCAD_G0_BITPOSMSB    0 /**< G0 field position in the GCAD register MSB */
#define MCP4XCXFX_GCAD_ADLCK_BITPOSLSB   7 /**< ADLCK field position in the GCAD register LSB */
#define MCP4XCXFX_GCAD_ADDR_I2C_BITPOSLSB    2 /**< ADDR_I2C  field position in the GCAD register LSB */

/* GCAD register bit-field masks */
#define MCP4XCXFX_GCAD_G7_BITMASK   0x01
#define MCP4XCXFX_GCAD_G6_BITMASK   0x01
#define MCP4XCXFX_GCAD_G5_BITMASK   0x01
#define MCP4XCXFX_GCAD_G4_BITMASK   0x01
#define MCP4XCXFX_GCAD_G3_BITMASK   0x01
#define MCP4XCXFX_GCAD_G2_BITMASK   0x01
#define MCP4XCXFX_GCAD_G1_BITMASK   0x01
#define MCP4XCXFX_GCAD_G0_BITMASK   0x01
#define MCP4XCXFX_GCAD_ADLCK_BITMASK  0x01
#define MCP4XCXFX_GCAD_ADDR_I2C_BITMASK   0x7F

/**
 * @brief DAC Wiperlock Technology Status Register structure.
 */
typedef struct _MCP4XCXFX_WLCK_REGFIELDS {
    /* register MSB */
    MCP4XCXFX_WLCKValues WL7 :2; /**< WL7B:WL7A */
    MCP4XCXFX_WLCKValues WL6 :2; /**< WL6B:WL6A */
    MCP4XCXFX_WLCKValues WL5 :2; /**< WL5B:WL5A */
    MCP4XCXFX_WLCKValues WL4 :2; /**< WL4B:WL4A */
    /* register LSB */
    MCP4XCXFX_WLCKValues WL3 :2; /**< WL3B:WL3A */
    MCP4XCXFX_WLCKValues WL2 :2; /**< WL2B:WL2A */
    MCP4XCXFX_WLCKValues WL1 :2; /**< WL1B:WL1A */
    MCP4XCXFX_WLCKValues WL0 :2; /**< WL0B:WL0A */
} MCP4XCXFX_WLCK_REGFIELDS, *MCP4XCXFX_WLCK_REGFIELDS_P;

/* WLCK CTRL register bit-field positions */
#define MCP4XCXFX_WLCK_CTRL_WL7_BITPOSMSB   6 /**< WL7 field position in the WLCK register MSB */
#define MCP4XCXFX_WLCK_CTRL_WL6_BITPOSMSB   4 /**< WL6 field position in the WLCK register MSB */
#define MCP4XCXFX_WLCK_CTRL_WL5_BITPOSMSB   2 /**< WL5 field position in the WLCK register MSB */
#define MCP4XCXFX_WLCK_CTRL_WL4_BITPOSMSB   0 /**< WL4 field position in the WLCK register MSB */
#define MCP4XCXFX_WLCK_CTRL_WL3_BITPOSLSB   6 /**< WL3 field position in the WLCK register LSB */
#define MCP4XCXFX_WLCK_CTRL_WL2_BITPOSLSB   4 /**< WL2 field position in the WLCK register LSB */
#define MCP4XCXFX_WLCK_CTRL_WL1_BITPOSLSB   2 /**< WL1 field position in the WLCK register LSB */
#define MCP4XCXFX_WLCK_CTRL_WL0_BITPOSLSB   0 /**< WL0 field position in the WLCK register LSB */
/* WLCK CTRL register bit-field masks */
#define MCP4XCXFX_WLCK_CTRL_WL7_BITMASK   0x03
#define MCP4XCXFX_WLCK_CTRL_WL6_BITMASK   0x03
#define MCP4XCXFX_WLCK_CTRL_WL5_BITMASK   0x03
#define MCP4XCXFX_WLCK_CTRL_WL4_BITMASK   0x03
#define MCP4XCXFX_WLCK_CTRL_WL3_BITMASK   0x03
#define MCP4XCXFX_WLCK_CTRL_WL2_BITMASK   0x03
#define MCP4XCXFX_WLCK_CTRL_WL1_BITMASK   0x03
#define MCP4XCXFX_WLCK_CTRL_WL0_BITMASK   0x03

/**
 * @brief Wiper block structure for all channels.
 */
typedef struct _MCP4XCXFX_WIPER_BLOCK {
    uint16_t CH7; /**< Channel 7 wiper value */
    uint16_t CH6; /**< Channel 6 wiper value */
    uint16_t CH5; /**< Channel 5 wiper value */
    uint16_t CH4; /**< Channel 4 wiper value */
    uint16_t CH3; /**< Channel 3 wiper value */
    uint16_t CH2; /**< Channel 2 wiper value */
    uint16_t CH1; /**< Channel 1 wiper value */
    uint16_t CH0; /**< Channel 0 wiper value */
} MCP4XCXFX_WIPER_BLOCK, *MCP4XCXFX_WIPER_BLOCK_P;

/**
 * @brief Interface initialization.
 */
typedef union _MCP4XCXFX_INTERFACE_INIT{
    MCP4XCXFX_I2C_INIT i2cInit;
    MCP4XCXFX_SPI_INIT spiInit;    
}MCP4XCXFX_INTERFACE_INIT;

/**
 * @brief Interface context.
 */
typedef union _MCP4XCXFX_INTERFACE_CONTEXT{
    MCP4XCXFX_I2C_CONTEXT i2cCtxt;
    MCP4XCXFX_SPI_CONTEXT spiCtxt;    
}MCP4XCXFX_INTERFACE_CONTEXT;

/**
 * @brief Device initialization structure.
 */
typedef struct _MCP4XCXFX_DEVICE_INIT{
    MCP4XCXFX_device_ID    deviceID; /**< Device ID */ 
    
   bool    syncMode;                 /**< if *true* ("synchronous mode") the library API returns after the request processing is completed, 
                                                 otherwise the function calls return immediately and the request processing 
                                                 is signaled by user callback or by status polling */
    uint16_t vRef_ext_mV;            /**< External reference voltage in mV */
    MCP4XCXFX_INTERFACE_INIT    interfaceInit;  /**< Interface initialization union */
} MCP4XCXFX_DEVICE_INIT; 

/** @brief Event enumeration.
 * MCP4XCXFX_EVENT enum defines the MCP4XCXFX library event identifiers returned 
 * by the MCP4XCXFX_GetEventStatus() function or by the event handler function 
 * registered with MCP4XCXFX_SetUserCallback().
 */
typedef enum {
    MCP4XCXFX_EVENT_NONE             = -1,     /**< No library event occurred (yet) for the latest API request. */
    MCP4XCXFX_EVENT_REQUEST_SUCCESS  =  0,     /**< The latest API request was completed successfully. */
    MCP4XCXFX_EVENT_REQUEST_FAIL     =  1,     /**< The latest API request failed. 
                                                 The reason is indicated by the _MCP4XCXFX_DEVICE_CONTEXT::processError. */
    MCP4XCXFX_EVENT_REQUEST_ABORT    =  2,     /**< The latest asynchronous API processing was aborted 
                                                 as a result of the MCP4XCXFX_AbortRequest() API call. */
}MCP4XCXFX_EVENT, *MCP4XCXFX_EVENT_P;

/** @brief Interface event enumeration.
 * MCP4XCXFX_INTERFACE_EVENT enum type defines the communication interface event identifiers 
 * for the i2c or spi transfer events reported by the transfer functions or to the 
 * transfer callback functions (if supported by the i2c or spi drivers).
 */
typedef enum
{
    /** Transfer request is pending */
    MCP4XCXFX_INTERFACE_EVENT_PENDING   = 0,
    /** All data from or to the buffer was transferred successfully. */
    MCP4XCXFX_INTERFACE_EVENT_COMPLETE  = 1,
    /** There was an error while processing the buffer transfer request. */
    MCP4XCXFX_INTERFACE_EVENT_ERROR     = 3,
    /* additional event codes */
    MCP4XCXFX_INTERFACE_NO_EVENT        = 100
} MCP4XCXFX_INTERFACE_EVENT, *MCP4XCXFX_INTERFACE_EVENT_P;


/**
 * @brief Library event handler function prototype.
 * 
 * Function prototype for the library events handler that a user application can
 * register as "callback function", using the MCP4XCXFX_SetUserCallback() API.
 * The library executes the "callback function" when the processing of asynchronous 
 * library API requests is finished, passing down to the user call-back
 * function the processing completion event type and the user "context" data pointer parameters.
 * @param event [in] - @ref MCP4XCXFX_EVENT event type indicating how the processing request was completed.
 * @param context [in] - "Transparent" pointer to a user variable of any datatype, 
 *                       as registered by the user application and returned back to the user application.
 */
typedef void ( *MCP4XCXFX_EVENT_HANDLER )( MCP4XCXFX_EVENT event, uintptr_t context );

/** @cond */
/**
 * MCP4XCXFX_procState enum defines the IDs associated with the various library processing stages 
 * that occur during processing the library requests. 
 * @note They are not part of the public API, being used only by the library internal logic.
 */
typedef enum _MCP4XCXFX_procState{
    Uninitialized               = 0,
    Idle                        = 1,
    Sync                        = 2,    
    SetRegisterReq              = 10,            
    GetRegister16bitReq         = 20,        
    GetWiperlock_NVReq          = 30,
    GetVrefCtrlReq              = 40,
    GetPwrDownCtrlReq           = 50,
    GetGainCtrlReq              = 60,
    GetGainCtrl_NVReq           = 70,
    SetGainCtrl_NVReq           = 80,
    ConfigBitUpdateReq          = 90,
    GetI2Caddr_NVReq            = 100,
    GetI2CaddrLock_NVReq        = 110,
    GetPorStatusReq             = 120,
    GetNVaccessStatusReq        = 130,
    
} MCP4XCXFX_procState;
/** @endcond */
/**
 * @brief Device context structure for MCP4XCXFX.
 */
typedef struct _MCP4XCXFX_DEVICE_CONTEXT
{
    bool    syncMode;                             /**< Synchronous/asynchronous mode. */
    MCP4XCXFX_MUTEX mutexProcState;               /**< Mutex which helps avoiding to process 
                                                     concurrent API requests. */
    volatile MCP4XCXFX_procState processingState; /**< Processing state machine */
    MCP4XCXFX_EVENT deviceEventStatus;            /**< Keeps the processing events status.
                                                     It is reported by MCP4XCXFX_GetEventStatus().
                                                     It is reset every time a new request is initiated. */
    int16_t processError;                         /**< Contains the error code at the end of the request processing, 
                                                     except for the API requests rejected upfront due to:
                                                     - MCP4XCXFX_BUSY
                                                     - MCP4XCXFX_MUTEX_FAIL
                                                     - MCP4XCXFX_INVALID_PARAMETER
                                                     It is reported by MCP4XCXFX_GetEventStatus(). */
    
    MCP4XCXFX_device_ID    deviceID;            /**< Device ID. */
    MCP4XCXFX_device_resolution    resolution;  /**< Device resolution. */
    MCP4XCXFX_device_channelCount    channels;  /**< Number of channels. */
    MCP4XCXFX_device_NVtype     NVtype;         /**< Non-volatile memory type. */
    
    uint16_t vRef_int_mV;       /**< Internal reference voltage (mV). */
    uint16_t vRef_ext_mV;       /**< External reference voltage (mV). */
        
    int16_t    lastErrorCode;   /**< Last error code. */
    
    MCP4XCXFX_INTERFACE_CONTEXT     if_ctxt;    /** <Interface context - I2C or SPI */
    MCP4XCXFX_INTERFACE_TYPE        if_type;    /**< Interface type. */
    uint8_t                         if_txBuffer[MCP4XCXFX_INTERFACE_TX_MAXSIZE];    /**< TX buffer. */
    uint8_t                         if_rxBuffer[MCP4XCXFX_INTERFACE_RX_MAXSIZE];    /**< RX buffer. */
   
    MCP4XCXFX_INTERFACE_EVENT    if_status;         /**< Communication interface status tracking flag. 
                                                     It is reset before each new I2C or SPI transfer. */

    MCP4XCXFX_EVENT_HANDLER     userCallback;       /**< Pointer to the registered user call-back function
                                                     for the library events. */
    uintptr_t    userContext;                       /**< Pointer to the registered user variable of any datatype, 
                                                     returned back to the user application by the call-back 
                                                     function for the library events. */ 
    /* cached register values */
    uint8_t gainCtrlNV_cache[MCP4XCXFX_REG_SZ];   /**< Cached gain control non-volatile register. */
        
    uint8_t stagingBytes[MCP4XCXFX_REG_SZ];     /**< Staged parameters. */
    
    void *outData;                              /**< Pointer to the user variable to which is copied 
                                                     the result of the API processing. */   

    bool ABORT_REQUESTED_FLAG;                  /**< True while a processing abort request is pending. */

    
} MCP4XCXFX_DEVICE_CONTEXT, *MCP4XCXFX_DEVICE_CONTEXT_P;
/** @} */

/** @defgroup MCP4XCXFX_API Public API Functions
 *  @brief Public API functions for MCP4XCXFX library.
 *  @{
 */

/**
 * @brief Initializes the MCP4XCXFX device context.
 *
 * Initializes the device context structure and configures the device according to the
 * specified initialization parameters. This function must be called before any other
 * library function.
 *
 * @param[out]    pdevice   Pointer to the device context structure to initialize.
 * @param[in]     devInit   Device initialization parameters structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
int16_t MCP4XCXFX_InitDevice(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_DEVICE_INIT devInit);

//L3/L4 functions
/**
 * @brief Main library task handler.
 *
 * Handles asynchronous processing and event management for the MCP4XCXFX library.
 * Should be called periodically in the main loop if asynchronous mode is used.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * 
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
int16_t MCP4XCXFX_LibTask(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);

/**
 * @brief Sets the wiper value for a channel.
 *
 * Writes the specified wiper value to the selected DAC channel.
 *
 * @param[in,out] pdevice     Pointer to the device context structure.
 * @param[in]     wiperValue  Wiper value to set.
 * @param[in]     channelNo   Channel number to update.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
int16_t MCP4XCXFX_SetWiper(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint16_t wiperValue, MCP4XCXFX_device_channelNo channelNo);

/**
 * @brief Gets the wiper value for a channel.
 *
 * Reads the current wiper value from the selected DAC channel.
 *
 * @param[in,out] pdevice      Pointer to the device context structure.
 * @param[out]    pwiperValue  Pointer to store the read wiper value.
 * @param[in]     channelNo    Channel number to read.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
int16_t MCP4XCXFX_GetWiper(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint16_t* pwiperValue, MCP4XCXFX_device_channelNo channelNo);

/**
 * @brief Sets the non-volatile wiper value for a channel.
 *
 * Writes the specified wiper value to the selected DAC channel's non-volatile memory.
 *
 * @param[in,out] pdevice     Pointer to the device context structure.
 * @param[in]     wiperValue  Wiper value to set.
 * @param[in]     channelNo   Channel number to update.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
int16_t MCP4XCXFX_SetWiper_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint16_t wiperValue, MCP4XCXFX_device_channelNo channelNo);     //non-Volatile

/**
 * @brief Gets the non-volatile wiper value for a channel.
 *
 * Reads the wiper value from the selected DAC channel's non-volatile memory.
 *
 * @param[in,out] pdevice      Pointer to the device context structure.
 * @param[out]    pwiperValue  Pointer to store the read wiper value.
 * @param[in]     channelNo    Channel number to read.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
int16_t MCP4XCXFX_GetWiper_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint16_t* pwiperValue, MCP4XCXFX_device_channelNo channelNo);   //non-Volatile

/**
 * @brief Sets a block of wiper values.
 *
 * Writes a block of wiper values to all DAC channels. Optionally activates the new values.
 *
 * @param[in,out] pdevice      Pointer to the device context structure.
 * @param[in]     wiperValues  Structure containing wiper values for all channels.
 * @param[in]     activate     If true, activates the new wiper values by using WiperLatchActivate function.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
int16_t MCP4XCXFX_SetWiper_block(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_WIPER_BLOCK wiperValues, bool activate);

/**
 * @brief Sets the wiperlock configuration in non-volatile memory.
 *
 * Writes the wiperlock control register structure to the device's non-volatile memory.
 * On EEPROM devices writes CL:DL bits. On MTP devices writes 1Bh register.
 *
 * @param[in,out] pdevice         Pointer to the device context structure.
 * @param[in]     wiperlockCtrl   Wiperlock control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_SetWiperlock_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_WLCK_REGFIELDS wiperlockCtrl);      

/**
 * @brief Gets the wiperlock configuration from non-volatile memory.
 *
 * Reads the wiperlock control register structure from the device's non-volatile memory.
 * On EEPROM devices reads 0Bh register. On MTP devices reads the 1Bh register.
 *
 * @param[in,out] pdevice           Pointer to the device context structure.
 * @param[out]    pwiperlockCtrl    Pointer to the wiperlock control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetWiperlock_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_WLCK_REGFIELDS_P pwiperlockCtrl);    

//Vref ctrl
/**
 * @brief Sets the VREF control register.
 *
 * Writes the VREF control register structure to the device.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[in]     VRctrl    VREF control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_SetVrefCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_VREF_REGFIELDS VRctrl);

/**
 * @brief Gets the VREF control register.
 *
 * Reads the VREF control register structure from the device.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[out]    pVRctrl   Pointer to the VREF control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetVrefCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_VREF_REGFIELDS_P pVRctrl);

/**
 * @brief Sets the VREF control register in non-volatile memory.
 *
 * Writes the VREF control register structure to the device's non-volatile memory.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[in]     VRctrl    VREF control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_SetVrefCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_VREF_REGFIELDS VRctrl);

/**
 * @brief Gets the VREF control register from non-volatile memory.
 *
 * Reads the VREF control register structure from the device's non-volatile memory.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[out]    pVRctrl   Pointer to the VREF control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetVrefCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_VREF_REGFIELDS_P pVRctrl);

//PD ctrl
/**
 * @brief Sets the power-down control register.
 *
 * Writes the power-down control register structure to the device.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[in]     PDctrl    Power-down control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_SetPwrDownCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_PWDN_REGFIELDS PDctrl);

/**
 * @brief Gets the power-down control register.
 *
 * Reads the power-down control register structure from the device.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[out]    pPDctrl   Pointer to the power-down control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetPwrDownCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_PWDN_REGFIELDS_P pPDctrl);

/**
 * @brief Sets the power-down control register in non-volatile memory.
 *
 * Writes the power-down control register structure to the device's non-volatile memory.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[in]     PDctrl    Power-down control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_SetPwrDownCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_PWDN_REGFIELDS PDctrl);

/**
 * @brief Gets the power-down control register from non-volatile memory.
 *
 * Reads the power-down control register structure from the device's non-volatile memory.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[out]    pPDctrl   Pointer to the power-down control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetPwrDownCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_PWDN_REGFIELDS_P pPDctrl);

//gain ctrl
/**
 * @brief Sets the gain control register.
 *
 * Writes the gain control register structure to the device.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[in]     gainCtrl  Gain control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_SetGainCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_GCST_REGFIELDS gainCtrl);

/**
 * @brief Gets the gain control register.
 *
 * Reads the gain control register structure from the device.
 *
 * @param[in,out] pdevice    Pointer to the device context structure.
 * @param[out]    pgainCtrl  Pointer to the gain control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetGainCtrl(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_GCST_REGFIELDS_P pgainCtrl);

/**
 * @brief Sets the gain control register in non-volatile memory.
 *
 * Writes the gain control register structure to the device's non-volatile memory.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[in]     gainCtrl  Gain control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_SetGainCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_GCAD_REGFIELDS gainCtrl);

/**
 * @brief Gets the gain control register from non-volatile memory.
 *
 * Reads the gain control register structure from the device's non-volatile memory.
 *
 * @param[in,out] pdevice    Pointer to the device context structure.
 * @param[out]    pgainCtrl  Pointer to the gain control register structure.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetGainCtrl_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_GCAD_REGFIELDS_P pgainCtrl);

//I2C addr
/**
 * @brief Sets the I2C address in non-volatile memory.
 *
 * Writes the specified I2C address to the device's non-volatile memory.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[in]     i2cAddr   The new I2C address to be set.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_SetI2Caddr_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t i2cAddr);

/**
 * @brief Gets the I2C address from non-volatile memory.
 *
 * Reads the I2C address from the device's non-volatile memory.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[out]    pi2cAddr   Pointer to the current I2C address.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetI2Caddr_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t* pi2cAddr);

/**
 * @brief Sets the I2C address lock bit in non-volatile memory.
 *
 * Locks or unlocks the I2C address on EEPROM ("F") devices.
 * Uses MCP4XCXFX_ConfigBitUpdate() for this operation.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[in]     i2cLock   Lock configuration bit value.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_SetI2CaddrLock_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_LCKcfgBit i2cLock);

/**
 * @brief Gets the I2C address lock bit from non-volatile memory.
 *
 * Reads the I2C address lock status from EEPROM ("F") devices.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[out]    pi2cLock   Pointer to the lock configuration bit value.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetI2CaddrLock_NV(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t* pi2cLock);

//status check functions
/**
 * @brief Gets the Power-on Reset (POR) status.
 *
 * Reads the POR status from the device.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[out]    pstatus   Pointer to the variable to store the POR status.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetPorStatus(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t* pstatus);

/**
 * @brief Gets the non-volatile memory access status.
 *
 * Reads the status of non-volatile memory access from the device.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[out]    pstatus   Pointer to the variable to store the NV access status.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_GetNVaccessStatus(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, uint8_t* pstatus);

/**
 * @brief Updates the configuration bits in EEPROM non-volatile memory.
 *
 * Enables or disables a configuration bit on devices equipped with EEPROM non-volatile memory.
 *
 * @param[in,out] pdevice     Pointer to the device context structure.
 * @param[in]     cfgBitAddr  Address of the configuration bit.
 * @param[in]     bit         Value to set (enable/disable).
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_ConfigBitUpdate(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_LCKcfgBitAddr cfgBitAddr, MCP4XCXFX_LCKcfgBit bit);

//L3 functions
/**
 * @brief Calculates the output voltage in microvolts.
 *
 * Computes the output voltage based on device resolution, wiper value, reference voltage, and gain.
 * Implements equation 4-2 from DS20006537B.
 *
 * @param[in] resolution   Device resolution.
 * @param[in] wiperValue   Current wiper value.
 * @param[in] Vref_uV      Reference voltage in microvolts.
 * @param[in] gain         Gain control bit.
 *
 * @return Output voltage in microvolts.
 */
uint32_t MCP4XCXFX_OutputVoltage_uV(MCP4XCXFX_device_resolution resolution, uint16_t wiperValue, uint32_t Vref_uV, MCP4XCXFX_gainCtrlBit gain);       //equation 4-2 from DS20006537B

/**
 * @brief Calculates the step voltage in microvolts.
 *
 * Computes the voltage step size based on device resolution, reference voltage, and gain.
 * Implements equation 4-3 from DS20006537B.
 *
 * @param[in] resolution   Device resolution.
 * @param[in] Vref_uV      Reference voltage in microvolts.
 * @param[in] gain         Gain control bit.
 *
 * @return Step voltage in microvolts.
 */
uint32_t MCP4XCXFX_StepVoltage_uV(MCP4XCXFX_device_resolution resolution, uint32_t Vref_uV, MCP4XCXFX_gainCtrlBit gain);                             //equation 4-3 from DS20006537B

/**
 * @brief Calculates the wiper value for a given output voltage, gain, and reference voltage.
 *
 * Computes the wiper value required to achieve the specified output voltage.
 *
 * @param[in] resolution         Device resolution.
 * @param[in] OutputVoltage_uV   Desired output voltage in microvolts.
 * @param[in] Vref_uV            Reference voltage in microvolts.
 * @param[in] gain               Gain control bit.
 *
 * @return Wiper value.
 */
uint16_t MCP4XCXFX_WiperValue(MCP4XCXFX_device_resolution resolution, int32_t OutputVoltage_uV, int32_t Vref_uV, MCP4XCXFX_gainCtrlBit gain);

/**
 * @brief Issues an I2C General Call command.
 *
 * Sends a general call command to the device over I2C.
 *
 * @param[in,out] pdevice   Pointer to the device context structure.
 * @param[in]     cmd       General call command to be sent.
 *
 * @return Error code (see @ref MCP4XCXFX_ErrorCodes).
 */
uint16_t MCP4XCXFX_I2C_General_Call(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_Gen_call_cmd cmd);

/**
 * @brief Copies a 16-bit unsigned value to a memory buffer as a 2-byte BIG-ENDIAN byte-stream.
 *
 * Copies the given 16-bit value into the provided buffer in BIG-ENDIAN order.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  regVal     16-bit register value to copy.
 * @param[out] pRawBytes  Pointer to the buffer to receive the bytes.
 */
void MCP4XCXFX_Reg16bitToRawBytes(uint16_t regVal, uint8_t* pRawBytes);

/**
 * @brief Converts a 2-byte BIG-ENDIAN byte-stream to a 16-bit unsigned value.
 *
 * Reads a 16-bit value from the provided BIG-ENDIAN byte buffer.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in] pRawBytes  Pointer to the buffer containing the bytes.
 *
 * @return 16-bit unsigned value.
 */
uint16_t MCP4XCXFX_RawBytestoReg16bit(uint8_t* pRawBytes);

/**
 * @brief Copies VREF control register bytes to a VREF register fields structure.
 *
 * Parses the raw VREF control register bytes into the corresponding structure.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  pVrefCtrlBytes      Pointer to the buffer containing the register bytes.
 * @param[out] pVrefCtrlRegfields  Pointer to the structure to receive the field values.
 */
void MCP4XCXFX_VrefCtrlBytesToRegfields(uint8_t* pVrefCtrlBytes, MCP4XCXFX_VREF_REGFIELDS_P pVrefCtrlRegfields);

/**
 * @brief Copies a VREF register fields structure to VREF control register bytes.
 *
 * Serializes the VREF register fields structure into a raw byte buffer.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  VrefCtrlRegfields  Structure containing the field values.
 * @param[out] pVrefCtrlBytes     Pointer to the buffer to receive the register bytes.
 */
void MCP4XCXFX_VrefCtrlRegfieldsToBytes(MCP4XCXFX_VREF_REGFIELDS VrefCtrlRegfields, uint8_t* pVrefCtrlBytes);

/**
 * @brief Copies power-down control register bytes to a power-down register fields structure.
 *
 * Parses the raw power-down control register bytes into the corresponding structure.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  pPwrDownCtrlBytes      Pointer to the buffer containing the register bytes.
 * @param[out] pPwrDownCtrlRegfields  Pointer to the structure to receive the field values.
 */
void MCP4XCXFX_PwrDownCtrlBytesToRegfields(uint8_t* pPwrDownCtrlBytes, MCP4XCXFX_PWDN_REGFIELDS_P pPwrDownCtrlRegfields);

/**
 * @brief Copies a power-down register fields structure to power-down control register bytes.
 *
 * Serializes the power-down register fields structure into a raw byte buffer.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  PwrDownCtrlRegfields  Structure containing the field values.
 * @param[out] pPwrDownCtrlBytes     Pointer to the buffer to receive the register bytes.
 */
void MCP4XCXFX_PwrDownCtrlRegfieldsToBytes(MCP4XCXFX_PWDN_REGFIELDS PwrDownCtrlRegfields, uint8_t* pPwrDownCtrlBytes);

/**
 * @brief Copies gain status register bytes to a gain status register fields structure.
 *
 * Parses the raw gain status register bytes into the corresponding structure.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  pGainStatusBytes      Pointer to the buffer containing the register bytes.
 * @param[out] pGainStatusRegfields  Pointer to the structure to receive the field values.
 */
void MCP4XCXFX_GainStatusBytesToRegfields(uint8_t* pGainStatusBytes, MCP4XCXFX_GCST_REGFIELDS_P pGainStatusRegfields);

/**
 * @brief Copies a gain status register fields structure to gain status register bytes.
 *
 * Serializes the gain status register fields structure into a raw byte buffer.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  GainStatusRegfields  Structure containing the field values.
 * @param[out] pGainStatusBytes     Pointer to the buffer to receive the register bytes.
 */
void MCP4XCXFX_GainStatusRegfieldsToBytes(MCP4XCXFX_GCST_REGFIELDS GainStatusRegfields, uint8_t* pGainStatusBytes);

/**
 * @brief Copies gain control register bytes to a gain control register fields structure.
 *
 * Parses the raw gain control register bytes into the corresponding structure.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  pGainCtrlBytes      Pointer to the buffer containing the register bytes.
 * @param[out] pGainCtrlRegfields  Pointer to the structure to receive the field values.
 */
void MCP4XCXFX_GainCtrlBytesToRegfields(uint8_t* pGainCtrlBytes, MCP4XCXFX_GCAD_REGFIELDS_P pGainCtrlRegfields);

/**
 * @brief Copies a gain control register fields structure to gain control register bytes.
 *
 * Serializes the gain control register fields structure into a raw byte buffer.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  GainCtrlRegfields  Structure containing the field values.
 * @param[out] pGainCtrlBytes     Pointer to the buffer to receive the register bytes.
 */
void MCP4XCXFX_GainCtrlRegfieldsToBytes(MCP4XCXFX_GCAD_REGFIELDS GainCtrlRegfields, uint8_t* pGainCtrlBytes);

/**
 * @brief Copies wiper lock register bytes to a wiper lock register fields structure.
 *
 * Parses the raw wiper lock register bytes into the corresponding structure.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  pWiperlockBytes      Pointer to the buffer containing the register bytes.
 * @param[out] pWiperlockRegfields  Pointer to the structure to receive the field values.
 */
void MCP4XCXFX_WiperlockBytesToRegfields(uint8_t* pWiperlockBytes, MCP4XCXFX_WLCK_REGFIELDS_P pWiperlockRegfields);

/**
 * @brief Copies a wiper lock register fields structure to wiper lock register bytes.
 *
 * Serializes the wiper lock register fields structure into a raw byte buffer.
 * 
 * @warning
 * The function makes no NULL-pointer check.
 *
 * @param[in]  WiperlockRegfields  Structure containing the field values.
 * @param[out] pWiperlockBytes     Pointer to the buffer to receive the register bytes.
 */
void MCP4XCXFX_WiperlockRegfieldsToBytes(MCP4XCXFX_WLCK_REGFIELDS WiperlockRegfields, uint8_t* pWiperlockBytes);

/**
 * @brief Checks if the device is initialized.
 *
 * Returns the initialization status of the device context.
 *
 * @param[in] pdevice   Pointer to the device context structure.
 *
 * @retval true   Device is initialized.
 * @retval false  Device is not initialized.
 */
bool MCP4XCXFX_Device_IsInitialized(MCP4XCXFX_DEVICE_CONTEXT_P pdevice);


/**
 * @brief Gets the event status and process error.
 *
 * Reads the event status and optionally returns a process error code.
 *
 * @param[in,out] pdevice        Pointer to the device context structure.
 * @param[out]    pevent         Pointer to the event structure.
 * @param[out]    pProcessError  Pointer to the process error code.
 *
 * @return Event status code.
 */
int16_t MCP4XCXFX_GetEventStatus(MCP4XCXFX_DEVICE_CONTEXT_P pdevice, MCP4XCXFX_EVENT_P pevent, int16_t* pProcessError);

/** @} */
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	// MCP4XCXFX_H

/**
  End of File
*/