#include "owb.h"
#include <stdint.h>
#include "cyhal.h"
#include "cybsp.h"
#include "task.h"
#include <stdbool.h>

owb_ret_t owb_init(OneWireBus *bus)
{
    cy_rslt_t rslt = cyhal_timer_init(&bus->bitTimer,NC,0);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
    rslt = cyhal_gpio_init(bus->pin,CYHAL_GPIO_DIR_BIDIRECTIONAL,CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW,true);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
    return OWB_STATUS_OK;    
}

void reset_gpio_event_callback(void *callback_arg, cyhal_gpio_event_t event)
{
    OneWireBus *bus = (OneWireBus *)callback_arg;
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(bus->signalSemaphore,&xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

void reset_timer_event_callback(void *callback_arg, cyhal_timer_event_t event)
{
    OneWireBus *bus = (OneWireBus *)callback_arg;
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(bus->signalSemaphore,&xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
 
}

owb_ret_t owb_reset(OneWireBus *bus, bool *result)
{
    owb_ret_t rval;
    bus->signalSemaphore = xSemaphoreCreateBinary();


    cyhal_timer_cfg_t cfgBit = {
        .is_continuous = false,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = false,
        .period = 480,
        .value = 0
    };
    cyhal_timer_configure(&bus->bitTimer,&cfgBit);

    bus->signalSemaphore = xSemaphoreCreateBinary();

    cyhal_timer_register_callback(&bus->bitTimer,reset_timer_event_callback,bus);
    cyhal_timer_enable_event(&bus->bitTimer,CYHAL_TIMER_IRQ_TERMINAL_COUNT,5,true);

    cyhal_gpio_write(bus->pin,0);
    cyhal_timer_reset(&bus->bitTimer);
    cyhal_timer_start(&bus->bitTimer);

    BaseType_t rsem = xSemaphoreTake(bus->signalSemaphore,2); // worst case is really 480uS
    cyhal_timer_enable_event(&bus->bitTimer,CYHAL_TIMER_IRQ_TERMINAL_COUNT,5,false); // Disable timer

    if(rsem != pdTRUE)
    {
        rval = OWB_STATUS_ERROR;
        return rval;
    }

    cyhal_gpio_register_callback(bus->pin, reset_gpio_event_callback,(void *)bus);
    cyhal_gpio_enable_event(bus->pin,CYHAL_GPIO_IRQ_FALL,5,true);

    cyhal_gpio_write(bus->pin,1);

    rsem = xSemaphoreTake(bus->signalSemaphore,1); // worst case is really 480uS
    cyhal_gpio_enable_event(bus->pin,CYHAL_GPIO_IRQ_FALL,4,false); // Disable the interrupt on this pin

    if(rsem == pdTRUE)
    {
        rval = OWB_STATUS_OK;
    }
    else
    {
        rval = OWB_STATUS_ERROR;
    }

    vSemaphoreDelete(bus->signalSemaphore);

    return rval;
}


///////////////////////////////////////////////////////////////////////////////
// Write Functions
///////////////////////////////////////////////////////////////////////////////


void write_bit_event_callback(void *callback_arg, cyhal_timer_event_t event)
{
    OneWireBus *bus = (OneWireBus *)callback_arg;

    if(event & CYHAL_TIMER_IRQ_CAPTURE_COMPARE)
    {
        cyhal_gpio_write(bus->pin,1);
    }
    if(event & CYHAL_TIMER_IRQ_TERMINAL_COUNT)
    {
        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(bus->signalSemaphore,&xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

}

owb_ret_t owb_write_bit(OneWireBus *bus,uint8_t val)
{
    owb_ret_t rval = OWB_STATUS_OK;
    //printf("Val=%d\n",val);

    cyhal_gpio_write(bus->pin,0);
    if(val == 0)
    {
        CyDelayUs(60);
        cyhal_gpio_write(bus->pin,1);
        CyDelayUs(2);

    }
    else
    {
        CyDelayUs(1);
        cyhal_gpio_write(bus->pin,1);
        CyDelayUs(60);
    }
    
    return rval;


}

#if 0
owb_ret_t owb_write_bit(OneWireBus *bus,uint8_t val)
{
    owb_ret_t rval;

    cyhal_timer_cfg_t cfgBit = {
        .is_continuous = false,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = true,
        .period = 61,
        .value = 0
    };

    bus->signalSemaphore = xSemaphoreCreateBinary();

    if(val == 0)
        cfgBit.compare_value = 60;
    else
        cfgBit.compare_value = 1;

    cyhal_timer_configure(&bus->bitTimer,&cfgBit);

    cyhal_timer_register_callback(&bus->bitTimer,write_bit_event_callback,bus);
    cyhal_timer_enable_event(&bus->bitTimer,CYHAL_TIMER_IRQ_TERMINAL_COUNT|CYHAL_TIMER_IRQ_CAPTURE_COMPARE,5,true);

    cyhal_gpio_write(bus->pin,0);

    cyhal_timer_reset(&bus->bitTimer);
    cyhal_timer_start(&bus->bitTimer);
    
    BaseType_t rsem = xSemaphoreTake(bus->signalSemaphore,1); // Then entire write cycle is 61uS so 1ms would be a major failure

    cyhal_timer_enable_event(&bus->bitTimer,CYHAL_TIMER_IRQ_TERMINAL_COUNT|CYHAL_TIMER_IRQ_CAPTURE_COMPARE,5,false);

    if(rsem == pdTRUE)
    {
        rval = OWB_STATUS_OK;
    }
    else
    {
        rval = OWB_STATUS_ERROR;
    }

    vSemaphoreDelete(bus->signalSemaphore);

    return rval;
}
#endif

owb_ret_t owb_write_byte( OneWireBus *bus, uint8_t val)
{
    owb_ret_t ret = OWB_STATUS_OK;
    for(int i=0;i<8;i++)
    {
        ret = owb_write_bit(bus,(val>>i) & 0x01);
        if(ret != OWB_STATUS_OK)
            return ret;        
    }
    return OWB_STATUS_OK;
}

owb_ret_t owb_write_bytes( OneWireBus *bus, uint8_t *buffer, uint32_t length)
{
    owb_ret_t ret = OWB_STATUS_OK;
    for(int i=0;i<length;i++)
    {
        ret = owb_write_byte(bus,buffer[i]);
        if(ret != OWB_STATUS_OK)
            return ret;
    }
    return OWB_STATUS_OK;
}

// TODO ARH
owb_ret_t owb_write_rom_code( OneWireBus *bus, OneWireBus_ROMCode romcode)
{
    return OWB_STATUS_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Read Functions
///////////////////////////////////////////////////////////////////////////////

void read_bit_event_callback(void *callback_arg, cyhal_timer_event_t event)
{
    OneWireBus *bus = (OneWireBus *)callback_arg;

    if(event & CYHAL_TIMER_IRQ_CAPTURE_COMPARE)
    {
        bus->scratchBitValue = cyhal_gpio_read(bus->pin);
    }
    if(event & CYHAL_TIMER_IRQ_TERMINAL_COUNT)
    {
        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(bus->signalSemaphore,&xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

}

owb_ret_t owb_read_bit( OneWireBus *bus, uint8_t *bit)
{
    owb_ret_t rval = OWB_STATUS_OK;

    cyhal_gpio_write(bus->pin,0);
    CyDelayUs(1);
    cyhal_gpio_write(bus->pin,1);
    CyDelayUs(5);

    *bit = cyhal_gpio_read(bus->pin);
    CyDelayUs(60);
    return rval;
}

#if 0

owb_ret_t owb_read_bit( OneWireBus *bus, uint8_t *bit)
{
    owb_ret_t rval;

    bus->signalSemaphore = xSemaphoreCreateBinary();

    cyhal_timer_cfg_t cfgBit = {
        .is_continuous = false,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = true,
        .compare_value = 5,
        .period = 61,
        .value = 0
    };

    cyhal_timer_configure(&bus->bitTimer,&cfgBit);

    cyhal_timer_register_callback(&bus->bitTimer,read_bit_event_callback,bus);
    cyhal_timer_enable_event(&bus->bitTimer,CYHAL_TIMER_IRQ_TERMINAL_COUNT|CYHAL_TIMER_IRQ_CAPTURE_COMPARE,5,true);


    // Pull a 0
    cyhal_gpio_write(bus->pin,0);
    CyDelayUs(1);
    cyhal_gpio_write(bus->pin,1);

    cyhal_timer_reset(&bus->bitTimer);
    cyhal_timer_start(&bus->bitTimer);

    BaseType_t rsem = xSemaphoreTake(bus->signalSemaphore,1); // Then entire write cycle is 61uS so 1ms would be a major failure

    cyhal_timer_enable_event(&bus->bitTimer,CYHAL_TIMER_IRQ_TERMINAL_COUNT|CYHAL_TIMER_IRQ_CAPTURE_COMPARE,5,false); // Disable the callback

    if(rsem == pdTRUE)
    {
        rval = OWB_STATUS_OK;
    }
    else
    {
        rval = OWB_STATUS_ERROR;
    }
    *bit = bus->scratchBitValue;

    vSemaphoreDelete(bus->signalSemaphore);

    return rval;
}
#endif

owb_ret_t owb_read_byte( OneWireBus *bus, uint8_t *byte)
{
    uint8_t bit;
    owb_ret_t ret = OWB_STATUS_OK;

    *byte = 0;
    for(int i=0;i<8;i++)
    {
        ret = owb_read_bit(bus,&bit);
        if(ret != OWB_STATUS_OK)
            return ret;
        *byte = *byte | (bit<<i);
    }
    return OWB_STATUS_OK;
}

owb_ret_t owb_read_bytes( OneWireBus *bus, uint8_t *buffer, uint32_t count)
{
    owb_ret_t ret = OWB_STATUS_OK;
    for(int i=0;i<count;i++)
    {
        ret = owb_read_byte(bus,&buffer[i]);
        if(ret != OWB_STATUS_OK)
            return ret;       
    }
    return ret;
}

// TODO ARH
owb_ret_t owb_crc8_bytes(uint32_t val, uint8_t *buffer, uint32_t length)
{
    return OWB_STATUS_OK;
}

void owb_set_strong_pullup( OneWireBus *bus, bool val)
{
    if(val)
        cyhal_gpio_configure(bus->pin,CYHAL_GPIO_DIR_BIDIRECTIONAL,CYHAL_GPIO_DRIVE_PULLUP);
    else
        cyhal_gpio_configure(bus->pin,CYHAL_GPIO_DIR_BIDIRECTIONAL,CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW);

}
