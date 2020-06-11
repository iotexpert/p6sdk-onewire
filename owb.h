
#ifndef OWB_H
#define OWB_H

#ifdef __cplusplus
extern "C" {
#endif

#define OWB_ROM_MATCH 0
#define OWB_ROM_SKIP 0

#include <stdint.h>
#include "cyhal.h"
#include "FreeRTOS.h"
#include "semphr.h"

typedef struct  
{
    cyhal_gpio_t  pin;
    bool use_parasitic_power;
    SemaphoreHandle_t signalSemaphore;
    cyhal_timer_t bitTimer;
    bool detect;
    volatile uint32_t scratchBitValue;
    cyhal_clock_t bitTimerClock;
    SemaphoreHandle_t owb_num_active;

} OneWireBus;        

typedef struct {
    uint8_t romAddr[8];

} OneWireBus_ROMCode;

typedef enum {
    OWB_STATUS_OK,
    OWB_STATUS_ERROR,
} owb_ret_t ;

owb_ret_t owb_init(OneWireBus *bus);
    
owb_ret_t owb_reset(OneWireBus *bus, bool *result);

owb_ret_t owb_write_bit( OneWireBus *bus, uint8_t val);
owb_ret_t owb_write_byte( OneWireBus *bus, uint8_t val);
owb_ret_t owb_write_bytes( OneWireBus *bus, uint8_t *buffer, uint32_t length);

owb_ret_t owb_write_rom_code(OneWireBus *bus, OneWireBus_ROMCode romcode);

owb_ret_t owb_read_bit( OneWireBus *bus, uint8_t *bit);
owb_ret_t owb_read_byte( OneWireBus *bus, uint8_t *byte);
owb_ret_t owb_read_bytes( OneWireBus *bus, uint8_t *buffer, uint32_t length);

owb_ret_t owb_crc8_bytes(uint32_t val, uint8_t *buffer, uint32_t length);

void   owb_set_strong_pullup( OneWireBus *bus, bool val);

void testpwm();
void testprint();

#endif

#ifdef __cplusplus
}
#endif
