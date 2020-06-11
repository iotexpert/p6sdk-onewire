#include "owb.h"
#include <stdint.h>
#include "cyhal.h"
#include "cybsp.h"
#include "task.h"
#include <stdbool.h>
#include <stdio.h>

static bool owb_power_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void* callback_arg);


owb_ret_t owb_init(OneWireBus *bus)
{
    cy_rslt_t rslt;

    bus->signalSemaphore = xSemaphoreCreateBinary();

    bus->owb_num_active = xSemaphoreCreateCounting(0xFFFF,0);

    cyhal_syspm_callback_data_t powerCallBackData =
    {
        .callback = owb_power_callback,
        .states = CYHAL_SYSPM_CB_CPU_DEEPSLEEP|CYHAL_SYSPM_CB_SYSTEM_HIBERNATE,
//        .ignore_modes = CYHAL_SYSPM_CHECK_FAIL|CYHAL_SYSPM_BEFORE_TRANSITION|CYHAL_SYSPM_AFTER_TRANSITION,
        .ignore_modes = 0,
        .args = (void*)bus,

    };

    cyhal_syspm_register_callback(&powerCallBackData);

    rslt = cyhal_timer_init(&bus->bitTimer,NC,0);
    rslt = cyhal_timer_set_frequency(&bus->bitTimer,1000000);


    cyhal_gpio_init(CYBSP_D5,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG,false);

    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
    rslt = cyhal_gpio_init(bus->pin,CYHAL_GPIO_DIR_BIDIRECTIONAL,CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW,true);
    CY_ASSERT(rslt == CY_RSLT_SUCCESS);
    return OWB_STATUS_OK;    
}


static bool owb_power_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void* callback_arg)
{
    OneWireBus *bus = (OneWireBus *)callback_arg;
    bool allow_transition = true;
    switch (mode)
    {
        case CYHAL_SYSPM_CHECK_READY:
            allow_transition = uxSemaphoreGetCount( bus->owb_num_active )>0?false:true;
            break;
        case CYHAL_SYSPM_CHECK_FAIL:
            /* Undo anything done as part of CHECK_READY since it is not actually chaning. */
            break;
        case CYHAL_SYSPM_BEFORE_TRANSITION:
            /* Do anything necessary to shut things down. */
            break;
        case CYHAL_SYSPM_AFTER_TRANSITION:
            /* Undo anything done as part of CHECK_READY or BEFORE_TRANSITION since it is coming back up. */
            break;
    }
    return allow_transition;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reset
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void reset_gpio_event_callback(void *callback_arg, cyhal_gpio_event_t event)
{
    OneWireBus *bus = (OneWireBus *)callback_arg;

    if(event & CYHAL_GPIO_IRQ_FALL)
    {
        bus->detect = true;
    }
}

void reset_timer_event_callback(void *callback_arg, cyhal_timer_event_t event)
{
    OneWireBus *bus = (OneWireBus *)callback_arg;

    if(event & CYHAL_TIMER_IRQ_CAPTURE_COMPARE)
    {
        cyhal_gpio_write(bus->pin,1);
        cyhal_gpio_register_callback(bus->pin, reset_gpio_event_callback,(void *)bus);
        cyhal_gpio_enable_event(bus->pin,CYHAL_GPIO_IRQ_FALL,5,true);
    }
    if(event & CYHAL_TIMER_IRQ_TERMINAL_COUNT)
    {

        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(bus->signalSemaphore,&xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
 
}

owb_ret_t owb_reset(OneWireBus *bus, bool *result)
{
    owb_ret_t rval = OWB_STATUS_OK;

    bus->detect = false;

    cyhal_timer_cfg_t cfgBit = {
        .is_continuous = false,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = true,
        .compare_value = 480,
        .period = 960,
        .value = 0
    };
    cyhal_timer_configure(&bus->bitTimer,&cfgBit);
    cyhal_timer_reset(&bus->bitTimer);

    cyhal_gpio_write(bus->pin,0);
    
    cyhal_timer_register_callback(&bus->bitTimer,reset_timer_event_callback,bus);
    cyhal_timer_enable_event(&bus->bitTimer,CYHAL_TIMER_IRQ_ALL,5,true);

    cyhal_timer_start(&bus->bitTimer);

    xSemaphoreTake(bus->signalSemaphore,2); // worst case is really 960uS

    return rval;
}


///////////////////////////////////////////////////////////////////////////////
// Write Functions
///////////////////////////////////////////////////////////////////////////////


#if 0
owb_ret_t owb_write_bit(OneWireBus *bus,uint8_t val)
{
    owb_ret_t rval = OWB_STATUS_OK;
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
#endif


#if 1

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
    owb_ret_t rval;

    cyhal_timer_cfg_t cfgBit = {
        .is_continuous = false,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = true,
        .compare_value = 1,
        .period = 62,
        .value = 0,
    };

    if(val == 0)
        cfgBit.compare_value = 60;
    else
        cfgBit.compare_value = 1;
    
    cyhal_timer_configure(&bus->bitTimer,&cfgBit);
    cyhal_timer_reset(&bus->bitTimer);

    cyhal_timer_register_callback(&bus->bitTimer,write_bit_event_callback,bus);
    cyhal_timer_enable_event(&bus->bitTimer,CYHAL_TIMER_IRQ_TERMINAL_COUNT|CYHAL_TIMER_IRQ_CAPTURE_COMPARE,5,true);

    taskENTER_CRITICAL();
    cyhal_gpio_write(bus->pin,0);

    cyhal_timer_start(&bus->bitTimer);
    taskEXIT_CRITICAL();
        
    BaseType_t rsem = xSemaphoreTake(bus->signalSemaphore,2); 

    if(rsem == pdTRUE)
        rval = OWB_STATUS_OK;
    else
        rval = OWB_STATUS_ERROR;

    return rval;
}

#endif

#if 0
volatile uint8_t bitCount=0;
volatile uint8_t byteOut=0;

static void setupBit(OneWireBus * bus)
{
    uint32_t newCompare;
    if(byteOut>>bitCount & 0x01)
        newCompare= 1;
    else
        newCompare = 65;

    cyhal_timer_setcompare(&bus->bitTimer,newCompare);

    bitCount += 1;
    cyhal_gpio_write(bus->pin,0);
    
    cyhal_timer_start(&bus->bitTimer);

}

void write_bit_event_callback(void *callback_arg, cyhal_timer_event_t event)
{

    OneWireBus *bus = (OneWireBus *)callback_arg;

    if(event & CYHAL_TIMER_IRQ_CAPTURE_COMPARE)
    {
        cyhal_gpio_write(bus->pin,1);
    }

    if(event & CYHAL_TIMER_IRQ_TERMINAL_COUNT)
    {
        if(bitCount < 8)
        {
            setupBit(bus);
        }
        else
        {
            BaseType_t xHigherPriorityTaskWoken;
            xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(bus->signalSemaphore,&xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        }        
    }
}

owb_ret_t owb_write_byte( OneWireBus *bus, uint8_t val)
{
    owb_ret_t rval = OWB_STATUS_OK;
    BaseType_t rsem;
    byteOut = val;
    bitCount = 0;
    bus->signalSemaphore = xSemaphoreCreateBinary();

    static cyhal_timer_cfg_t cfgBit = {
        .is_continuous = false,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = true,
        .compare_value = 1,
        .period = 70,
        .value = 0,
    };

    if(byteOut>>bitCount & 0x01)
    {
        cfgBit.compare_value = 1;
    }
    else
    {
        cfgBit.compare_value = 65;
    }

    bitCount += 1;

    cyhal_timer_configure(&bus->bitTimer,&cfgBit);
    cyhal_timer_register_callback(&bus->bitTimer,write_bit_event_callback,bus);
    cyhal_timer_enable_event(&bus->bitTimer,CYHAL_TIMER_IRQ_TERMINAL_COUNT|CYHAL_TIMER_IRQ_CAPTURE_COMPARE,6,true);

    cyhal_gpio_write(bus->pin,0);
    cyhal_timer_start(&bus->bitTimer);

    rsem = xSemaphoreTake(bus->signalSemaphore,2); // Then entire write cycle is 61uS so 1ms would be a major failure
    vSemaphoreDelete(bus->signalSemaphore);

    if(rsem == pdTRUE)
        rval = OWB_STATUS_OK;
    else
        rval = OWB_STATUS_ERROR;

    return rval;

}
#endif

#if 1

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
#endif

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


#if 0
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
#endif

#if 1

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
    owb_ret_t rval;

    cyhal_timer_cfg_t cfgBit = {
        .is_continuous = false,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = true,
        .compare_value = 10,
        .period = 61,
        .value = 0
    };

    cyhal_timer_configure(&bus->bitTimer,&cfgBit);

    cyhal_timer_register_callback(&bus->bitTimer,read_bit_event_callback,bus);
    cyhal_timer_enable_event(&bus->bitTimer,CYHAL_TIMER_IRQ_TERMINAL_COUNT|CYHAL_TIMER_IRQ_CAPTURE_COMPARE,5,true);

    void taskENTER_CRITICAL();

    cyhal_gpio_write(bus->pin,0);     // Pull a 0
    CyDelayUs(1);
    cyhal_gpio_write(bus->pin,1);

    cyhal_timer_start(&bus->bitTimer);
    void taskEXIT_CRITICAL( );


    BaseType_t rsem = xSemaphoreTake(bus->signalSemaphore,2); // Then entire write cycle is 61uS so 2ms would be a major failure
    *bit = bus->scratchBitValue;

    if(rsem == pdTRUE)
        rval = OWB_STATUS_OK;
    else
        rval = OWB_STATUS_ERROR;

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
        cyhal_gpio_configure(bus->pin,CYHAL_GPIO_DIR_BIDIRECTIONAL,CYHAL_GPIO_DRIVE_STRONG);
    else
        cyhal_gpio_configure(bus->pin,CYHAL_GPIO_DIR_BIDIRECTIONAL,CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW);

}
