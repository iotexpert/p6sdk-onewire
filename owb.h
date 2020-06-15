/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 * Copyright (c) 2017 Chris Morgan <chmorgan@gmail.com>
 * Copyright (c) 2020 Alan Hawse <engineer@iotexpert.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file
 * @brief Interface definitions for the 1-Wire bus component.
 *
 * This component provides structures and functions that are useful for communicating
 * with devices connected to a Maxim Integrated 1-Wire® bus via a single GPIO.
 *
 * Currently only externally powered devices are supported. Parasitic power is not supported.
 * 
 * This library was materially updated for PSoC 6
 * 
 */

#ifndef OWB_H
#define OWB_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include "cyhal.h"
#include "FreeRTOS.h"
#include "semphr.h"

/**
 * @brief Represents a set of the legal one wire bus commands
 */
typedef enum {
    OWB_ROM_SEARCH        =0xF0,  ///< Perform Search ROM cycle to identify devices on the bus
    OWB_ROM_READ          =0x33,  ///< Read device ROM (single device on bus only)
    OWB_ROM_MATCH         =0x55,  ///< Address a specific device on the bus by ROM
    OWB_ROM_SKIP          =0xCC,  ///< Address all devices on the bus simultaneously
    OWB_ROM_SEARCH_ALARM  =0xEC,  ///< Address all devices on the bus with a set alarm flag
} owb_commands_t;


/**
 * @brief Structure containing 1-Wire bus information relevant to a single instance.
 */
typedef struct  
{
    bool is_init;
    cyhal_gpio_t  pin;
    cyhal_gpio_t strong_pullup_gpio;
    bool use_parasitic_power;
    bool use_crc;
    // Internal use only
    bool detect;
    SemaphoreHandle_t signalSemaphore;
    cyhal_timer_t bitTimer;
    SemaphoreHandle_t owb_num_active;
    uint8_t scratchBitValue;
} OneWireBus;        


/**
 * @brief Represents a 1-Wire ROM Code. This is a sequence of eight bytes, where
 *        the first byte is the family number, then the following 6 bytes form the
 *        serial number. The final byte is the CRC8 check byte.
 */
typedef union
{
    /// Provides access via field names
    struct fields
    {
        uint8_t family[1];         ///< family identifier (1 byte, LSB - read/write first)
        uint8_t serial_number[6];  ///< serial number (6 bytes)
        uint8_t crc[1];            ///< CRC check byte (1 byte, MSB - read/write last)
    } __PACKED fields;             ///< Provides access via field names

    uint8_t bytes[8];              ///< Provides raw byte access

} OneWireBus_ROMCode;


/**
 * @brief Represents the state of a device search on the 1-Wire bus.
 *
 *        Pass a pointer to this structure to owb_search_first() and
 *        owb_search_next() to iterate through detected devices on the bus.
 */
typedef struct
{
    OneWireBus_ROMCode rom_code;   ///< Device ROM code
    int last_discrepancy;          ///< Bit index that identifies from which bit the next search discrepancy check should start
    int last_family_discrepancy;   ///< Bit index that identifies the last discrepancy within the first 8-bit family code of the ROM code
    int last_device_flag;          ///< Flag to indicate previous search was the last device detected
} OneWireBus_SearchState;

/**
 * @brief Represents the result of OWB API functions.
 */
typedef enum {
    OWB_STATUS_NOT_SET = -1,           ///< A status value has not been set
    OWB_STATUS_OK = 0,                 ///< Operation succeeded
    OWB_STATUS_NOT_INITIALIZED,        ///< Function was passed an uninitialised variable
    OWB_STATUS_PARAMETER_NULL,         ///< Function was passed a null pointer
    OWB_STATUS_DEVICE_NOT_RESPONDING,  ///< No response received from the addressed device or devices
    OWB_STATUS_CRC_FAILED,             ///< CRC failed on data received from a device or devices
    OWB_STATUS_TOO_MANY_BITS,          ///< Attempt to write an incorrect number of bits to the One Wire Bus
    OWB_STATUS_HW_ERROR                ///< A hardware error occurred
} owb_ret_t ;


/**
 * @brief Initialize the one-wire bus structure and hardware.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[out] none
 * @return status
 */    
owb_ret_t owb_init(OneWireBus *bus);

/**
 * @brief Reset the 1-Wire bus.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[out] is_present set to true if at least one device is present on the bus
 * @return status
 */    
owb_ret_t owb_reset(OneWireBus *bus, bool *result);

/**
 * @brief Write a bit to the 1-Wire bus.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] bit Value to write (lsb only).
 * @return status
 */
owb_ret_t owb_write_bit( OneWireBus *bus, uint8_t val);


/**
 * @brief Write a single byte to the 1-Wire bus.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] data Byte value to write to bus.
 * @return status
 */
owb_ret_t owb_write_byte( OneWireBus *bus, uint8_t val);

/**
 * @brief Write a number of bytes to the 1-Wire bus.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] buffer Pointer to buffer to write data from.
 * @param[in] len Number of bytes to write.
 * @return status
 */
owb_ret_t owb_write_bytes( OneWireBus *bus, uint8_t *buffer, uint32_t length);

/**
 * @brief Read a single bit from the 1-Wire bus.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[out] out The bit value read from the bus.
 * @return status
 */
owb_ret_t owb_read_bit( OneWireBus *bus, uint8_t *bit);

/**
 * @brief Read a single byte from the 1-Wire bus.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[out] out The byte value read from the bus (lsb only).
 * @return status
 */
owb_ret_t owb_read_byte( OneWireBus *bus, uint8_t *byte);

/**
 * @brief Read a number of bytes from the 1-Wire bus.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in, out] buffer Pointer to buffer to receive read data.
 * @param[in] len Number of bytes to read, must not exceed length of receive buffer.
 * @return status.
 */
owb_ret_t owb_read_bytes( OneWireBus *bus, uint8_t *buffer, uint32_t length);


/**
 * @brief Enable or disable use of CRC checks on device communications.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] use_crc True to enable CRC checks, false to disable.
 * @return status
 */
owb_ret_t owb_use_crc(OneWireBus * bus, bool use_crc);

/**
 * @brief Enable or disable use of parasitic power on the One Wire Bus.
 *        This affects how devices signal on the bus, as devices cannot
 *        signal by pulling the bus low when it is pulled high.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] use_parasitic_power True to enable parasitic power, false to disable.
 * @return status
 */
owb_ret_t owb_use_parasitic_power(OneWireBus * bus, bool use_parasitic_power);

/**
 * @brief Enable or disable use of extra GPIO to activate strong pull-up circuit.
 *        This only has effect if parasitic power mode is enabled.
 *        signal by pulling the bus low when it is pulled high.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] gpio Set to GPIO number to use, or GPIO_NUM_NC to disable.
 * @return status
 */
owb_ret_t owb_use_strong_pullup_gpio(OneWireBus * bus, cyhal_gpio_t gpio);

/**
 * @brief Enable or disable the strong-pullup GPIO, if configured.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] enable If true, enable the external strong pull-up by setting the GPIO high.
 *                   If false, disable the external strong pull-up by setting the GPIO low.
 * @return status
 */
owb_ret_t owb_set_strong_pullup(OneWireBus * bus, bool enable);


/**
 * @brief Read ROM code from device - only works when there is a single device on the bus.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[out] rom_code the value read from the device's rom
 * @return status
 */
owb_ret_t owb_read_rom(OneWireBus * bus, OneWireBus_ROMCode * rom_code);

/**
 * @brief Verify the device specified by ROM code is present.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] rom_code ROM code to verify.
 * @param[out] is_present Set to true if a device is present, false if not
 * @return status
 */
owb_ret_t owb_verify_rom(OneWireBus * bus, OneWireBus_ROMCode *rom_code, bool * is_present);


/**
 * @brief Write a ROM code to the 1-Wire bus ensuring LSB is sent first.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] rom_code ROM code to write to bus.
 * @return status
 */
owb_ret_t owb_write_rom_code(OneWireBus * bus, OneWireBus_ROMCode *rom_code);

/**
 * @brief Create a string representation of a ROM code, most significant byte (CRC8) first.
 * @param[in] rom_code The ROM code to convert to string representation.
 * @param[out] buffer The destination for the string representation. It will be null terminated.
 * @param[in] len The length of the buffer in bytes. 64-bit ROM codes require 16 characters
 *                to represent as a string, plus a null terminator, for 17 bytes.
 *                See OWB_ROM_CODE_STRING_LENGTH.
 * @return pointer to the byte beyond the last byte written
 */
char * owb_string_from_rom_code(OneWireBus_ROMCode *rom_code, char * buffer, size_t len);


/**
 * @brief 1-Wire 8-bit CRC lookup.
 * @param[in] crc Starting CRC value. Pass in prior CRC to accumulate.
 * @param[in] data Byte to feed into CRC.
 * @return Resultant CRC value.
 *         Should be zero if last byte was the CRC byte and the CRC matches.
 */
uint8_t owb_crc8_byte(uint8_t crc, uint8_t data);

/**
 * @brief 1-Wire 8-bit CRC lookup with accumulation over a block of bytes.
 * @param[in] crc Starting CRC value. Pass in prior CRC to accumulate.
 * @param[in] data Array of bytes to feed into CRC.
 * @param[in] len Length of data array in bytes.
 * @return Resultant CRC value.
 *         Should be zero if last byte was the CRC byte and the CRC matches.
 */
uint8_t owb_crc8_bytes(uint8_t crc, const uint8_t * data, size_t len);

/**
 * @brief Locates the first device on the 1-Wire bus, if present.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in,out] state Pointer to an existing search state structure.
 * @param[out] found_device True if a device is found, false if no devices are found.
 *         If a device is found, the ROM Code can be obtained from the state.
 * @return status
 */
owb_ret_t owb_search_first(OneWireBus * bus, OneWireBus_SearchState * state, bool *found_device);

/**
 * @brief Locates the next device on the 1-Wire bus, if present, starting from
 *        the provided state. Further calls will yield additional devices, if present.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in,out] state Pointer to an existing search state structure.
 * @param[out] found_device True if a device is found, false if no devices are found.
 *         If a device is found, the ROM Code can be obtained from the state.
 * @return status
 */
owb_ret_t owb_search_next(OneWireBus * bus, OneWireBus_SearchState * state, bool *found_device);



#endif

#ifdef __cplusplus
}
#endif
