#include "owb.h"
#include <stdint.h>
#include "cyhal.h"
#include "cybsp.h"
#include "task.h"
#include <stdbool.h>
#include <stdio.h>
#include "log.h"

static bool owb_power_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void* callback_arg);
static  char * TAG = "ds18b20";

owb_ret_t owb_init(OneWireBus *bus)
{
    cy_rslt_t rslt;

    bus->signalSemaphore = xSemaphoreCreateBinary();

    bus->owb_num_active = xSemaphoreCreateBinary();

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
    bus->is_init = true;

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

    *result=bus->detect;

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
        rval = OWB_STATUS_HW_ERROR;

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
        rval = OWB_STATUS_HW_ERROR;

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


owb_ret_t owb_set_strong_pullup(OneWireBus * bus, bool enable)
{

    owb_ret_t status = OWB_STATUS_NOT_SET;

    if (!bus)
    {
        status = OWB_STATUS_PARAMETER_NULL;
    }
    else if (!bus->is_init)
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }
    else
    {
        if (bus->use_parasitic_power && bus->strong_pullup_gpio != NC)
        {
            //gpio_set_level(bus->strong_pullup_gpio, enable ? 1 : 0);
            cyhal_gpio_write(bus->strong_pullup_gpio,enable);
            ESP_LOGD(TAG, "strong pullup GPIO %d", enable);
        }  // else ignore

        status = OWB_STATUS_OK;
    }

    return status;
}

/**
 * @brief Enable or disable use of CRC checks on device communications.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] use_crc True to enable CRC checks, false to disable.
 * @return status
 */
owb_ret_t owb_use_crc(OneWireBus * bus, bool use_crc)
{
    CY_ASSERT(bus);
    owb_ret_t ret = OWB_STATUS_OK;
    bus->use_crc = use_crc;

    return ret;
}

/**
 * @brief Enable or disable use of parasitic power on the One Wire Bus.
 *        This affects how devices signal on the bus, as devices cannot
 *        signal by pulling the bus low when it is pulled high.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] use_parasitic_power True to enable parasitic power, false to disable.
 * @return status
 */
owb_ret_t owb_use_parasitic_power(OneWireBus * bus, bool use_parasitic_power)
{
    CY_ASSERT(bus);
    owb_ret_t ret = OWB_STATUS_OK;
    bus->use_parasitic_power = use_parasitic_power;
    return ret;
}

/**
 * @brief Enable or disable use of extra GPIO to activate strong pull-up circuit.
 *        This only has effect if parasitic power mode is enabled.
 *        signal by pulling the bus low when it is pulled high.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] gpio Set to GPIO number to use, or GPIO_NUM_NC to disable.
 * @return status
 */
owb_ret_t owb_use_strong_pullup_gpio(OneWireBus * bus, cyhal_gpio_t gpio)
{
    owb_ret_t status = OWB_STATUS_NOT_SET;
 
    if (!bus)
    {
        status = OWB_STATUS_PARAMETER_NULL;
    }
    else if (!bus->is_init)
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }
    else
    {
        if (gpio != NC) {
            // The strong GPIO pull-up is only activated if parasitic-power mode is enabled
            if (!bus->use_parasitic_power) {
                ESP_LOGW(TAG, "Strong pull-up GPIO set with parasitic-power disabled");
            }

            //gpio_pad_select_gpio(gpio);
            //gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
            cyhal_gpio_init(bus->strong_pullup_gpio,CYHAL_GPIO_DIR_OUTPUT,CYHAL_GPIO_DRIVE_STRONG,0);
        }
        else
        {
            //gpio_reset_pin(bus->strong_pullup_gpio);
            cyhal_gpio_free(bus->strong_pullup_gpio);
        }

        bus->strong_pullup_gpio = gpio;
        ESP_LOGD(TAG, "use_strong_pullup_gpio %d", bus->strong_pullup_gpio);

        status = OWB_STATUS_OK;
    }

    return status;
}


/**
 * @brief Read ROM code from device - only works when there is a single device on the bus.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[out] rom_code the value read from the device's rom
 * @return status
 */
owb_ret_t owb_read_rom(OneWireBus * bus, OneWireBus_ROMCode * rom_code)
{
    owb_ret_t status = OWB_STATUS_NOT_SET;

    memset(rom_code, 0, sizeof(OneWireBus_ROMCode));

    if (!bus || !rom_code)
    {
        status = OWB_STATUS_PARAMETER_NULL;
    }
    else if (!bus->is_init)
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }
    else
    {
        bool is_present;
        owb_reset(bus,&is_present);

        if (is_present)
        {
            uint8_t value = OWB_ROM_READ;
            owb_write_byte(bus,value);
            owb_read_bytes(bus, rom_code->bytes, sizeof(OneWireBus_ROMCode));

            if (bus->use_crc)
            {
                if (owb_crc8_bytes(0, rom_code->bytes, sizeof(OneWireBus_ROMCode)) != 0)
                {
                  //  ESP_LOGE(TAG, "CRC failed");
                    memset(rom_code->bytes, 0, sizeof(OneWireBus_ROMCode));
                    status = OWB_STATUS_CRC_FAILED;
                }
                else
                {
                    status = OWB_STATUS_OK;
                }
            }
            else
            {
                status = OWB_STATUS_OK;
            }
            char rom_code_s[sizeof(OneWireBus_ROMCode)];
            owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
       //     ESP_LOGD(TAG, "rom_code %s", rom_code_s);
        }
        else
        {
            status = OWB_STATUS_DEVICE_NOT_RESPONDING;
       //     ESP_LOGE(TAG, "ds18b20 device not responding");
        }
    }

    return status;
}


// TODO ARH
/**
 * @param[out] is_found true if a device was found, false if not
 * @return status
 */
static owb_ret_t _search( OneWireBus * bus, OneWireBus_SearchState * state, bool * is_found)
{
    // Based on https://www.maximintegrated.com/en/app-notes/index.mvp/id/187

    // initialize for search
    int id_bit_number = 1;
    int last_zero = 0;
    int rom_byte_number = 0;
    uint8_t id_bit = 0;
    uint8_t cmp_id_bit = 0;
    uint8_t rom_byte_mask = 1;
    uint8_t search_direction = 0;
    bool search_result = false;
    uint8_t crc8 = 0;
    owb_ret_t status = OWB_STATUS_NOT_SET;

    // if the last call was not the last one
    if (!state->last_device_flag)
    {
        // 1-Wire reset
        bool is_present;
        owb_reset(bus,&is_present);
        if (!is_present)
        {
            // reset the search
            state->last_discrepancy = 0;
            state->last_device_flag = false;
            state->last_family_discrepancy = 0;
            *is_found = false;
            return OWB_STATUS_OK;
        }

        // issue the search command

        owb_write_byte(bus,OWB_ROM_SEARCH);

        // loop to do the search
        do
        {
            id_bit = cmp_id_bit = 0;

            // read a bit and its complement
            //bus->driver->read_bits(bus, &id_bit, 1);
            //bus->driver->read_bits(bus, &cmp_id_bit, 1);

            owb_read_bit(bus,&id_bit);
            owb_read_bit(bus,&cmp_id_bit);
            

            // check for no devices on 1-wire (signal level is high in both bit reads)
            if (id_bit && cmp_id_bit)
            {
                break;
            }
            else
            {
                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit)
                {
                    search_direction = (id_bit) ? 1 : 0;  // bit write value for search
                }
                else
                {
                    // if this discrepancy if before the Last Discrepancy
                    // on a previous next then pick the same as last time
                    if (id_bit_number < state->last_discrepancy)
                    {
                        search_direction = ((state->rom_code.bytes[rom_byte_number] & rom_byte_mask) > 0);
                    }
                    else
                    {
                        // if equal to last pick 1, if not then pick 0
                        search_direction = (id_bit_number == state->last_discrepancy);
                    }

                    // if 0 was picked then record its position in LastZero
                    if (search_direction == 0)
                    {
                        last_zero = id_bit_number;

                        // check for Last discrepancy in family
                        if (last_zero < 9)
                        {
                            state->last_family_discrepancy = last_zero;
                        }
                    }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction == 1)
                {
                    state->rom_code.bytes[rom_byte_number] |= rom_byte_mask;
                }
                else
                {
                    state->rom_code.bytes[rom_byte_number] &= ~rom_byte_mask;
                }

                // serial number search direction write bit
//                bus->driver->write_bits(bus, search_direction, 1);
                owb_write_bit(bus,search_direction);

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0)
                {
                    crc8 = owb_crc8_byte(crc8, state->rom_code.bytes[rom_byte_number]);  // accumulate the CRC
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        }
        while (rom_byte_number < 8);  // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!((id_bit_number < 65) || (crc8 != 0)))
        {
            // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
            state->last_discrepancy = last_zero;

            // check for last device
            if (state->last_discrepancy == 0)
            {
                state->last_device_flag = true;
            }

            search_result = true;
        }
    }

    // if no device found then reset counters so next 'search' will be like a first
    if (!search_result || !state->rom_code.bytes[0])
    {
        state->last_discrepancy = 0;
        state->last_device_flag = false;
        state->last_family_discrepancy = 0;
        search_result = false;
    }

    status = OWB_STATUS_OK;

    *is_found = search_result;

    return status;
}


/**
 * @brief Verify the device specified by ROM code is present.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] rom_code ROM code to verify.
 * @param[out] is_present Set to true if a device is present, false if not
 * @return status
 */
owb_ret_t owb_verify_rom(OneWireBus * bus, OneWireBus_ROMCode *rom_code, bool * is_present)
{
    owb_ret_t status = OWB_STATUS_NOT_SET;
    bool result = false;

    if (!bus || !bus->detect)
    {
        status = OWB_STATUS_PARAMETER_NULL;
    }
    else if (!bus->is_init)
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }
    else
    {
        OneWireBus_SearchState state = {
            //.rom_code = rom_code,
            .last_discrepancy = 64,
            .last_device_flag = false,
        };

        memcpy(&state.rom_code,rom_code,sizeof(OneWireBus_ROMCode));

        bool is_found = false;
        _search(bus, &state, &is_found);
        if (is_found)
        {
            result = true;
            for (int i = 0; i < sizeof(state.rom_code.bytes) && result; ++i)
            {
                result = rom_code->bytes[i] == state.rom_code.bytes[i];
                ESP_LOGD(TAG, "%02x %02x", rom_code->bytes[i], state.rom_code.bytes[i]);
            }
            is_found = result;
        }
        ESP_LOGD(TAG, "state.last_discrepancy %d, state.last_device_flag %d, is_found %d",
                 state.last_discrepancy, state.last_device_flag, is_found);

        ESP_LOGD(TAG, "rom code %sfound", result ? "" : "not ");
        *is_present = result;
        status = OWB_STATUS_OK;
    }

    return status;
}



/**
 * @brief Write a ROM code to the 1-Wire bus ensuring LSB is sent first.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in] rom_code ROM code to write to bus.
 * @return status
 */
owb_ret_t owb_write_rom_code( OneWireBus *bus, OneWireBus_ROMCode *romcode)
{
    owb_ret_t rval = owb_write_bytes(bus,romcode->bytes,sizeof(OneWireBus_ROMCode));
    return rval;
}


/**
 * @brief 1-Wire 8-bit CRC lookup.
 * @param[in] crc Starting CRC value. Pass in prior CRC to accumulate.
 * @param[in] data Byte to feed into CRC.
 * @return Resultant CRC value.
 */
static uint8_t _calc_crc(uint8_t crc, uint8_t data)
{
    // https://www.maximintegrated.com/en/app-notes/index.mvp/id/27
    static const uint8_t table[256] = {
            0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
            157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
            35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
            190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
            70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
            219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
            101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
            248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
            140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
            17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
            175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
            50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
            202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
            87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
            233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
            116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
    };

    return table[crc ^ data];
}

static uint8_t _calc_crc_block(uint8_t crc, const uint8_t * buffer, size_t len)
{
    do
    {
        crc = _calc_crc(crc, *buffer++);
//        ESP_LOGD(TAG, "buffer 0x%02x, crc 0x%02x, len %d", (uint8_t)*(buffer - 1), (int)crc, (int)len);
    }
    while (--len > 0);
    return crc;
}

/**
 * @brief 1-Wire 8-bit CRC lookup.
 * @param[in] crc Starting CRC value. Pass in prior CRC to accumulate.
 * @param[in] data Byte to feed into CRC.
 * @return Resultant CRC value.
 *         Should be zero if last byte was the CRC byte and the CRC matches.
 */

uint8_t owb_crc8_byte(uint8_t crc, uint8_t data)
{
    return _calc_crc(crc, data);
}



/**
 * @brief 1-Wire 8-bit CRC lookup with accumulation over a block of bytes.
 * @param[in] crc Starting CRC value. Pass in prior CRC to accumulate.
 * @param[in] data Array of bytes to feed into CRC.
 * @param[in] len Length of data array in bytes.
 * @return Resultant CRC value.
 *         Should be zero if last byte was the CRC byte and the CRC matches.
 */

uint8_t owb_crc8_bytes(uint8_t crc, const uint8_t * data, size_t len)
{
    return _calc_crc_block(crc, data, len);
}

/**
 * @brief Locates the first device on the 1-Wire bus, if present.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in,out] state Pointer to an existing search state structure.
 * @param[out] found_device True if a device is found, false if no devices are found.
 *         If a device is found, the ROM Code can be obtained from the state.
 * @return status
 */
owb_ret_t owb_search_first(OneWireBus * bus, OneWireBus_SearchState * state, bool *found_device)
{
    bool result;
    owb_ret_t status = OWB_STATUS_NOT_SET;

    if (!bus || !state || !found_device)
    {
        status = OWB_STATUS_PARAMETER_NULL;
    }
    else if (!bus->is_init)
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }
    else
    {
        memset(&state->rom_code, 0, sizeof(state->rom_code));
        state->last_discrepancy = 0;
        state->last_family_discrepancy = 0;
        state->last_device_flag = false;
        _search(bus, state, &result);
        status = OWB_STATUS_OK;

        *found_device = result;
    }

    return status;
}

// TODO ARH

/**
 * @brief Locates the next device on the 1-Wire bus, if present, starting from
 *        the provided state. Further calls will yield additional devices, if present.
 * @param[in] bus Pointer to initialised bus instance.
 * @param[in,out] state Pointer to an existing search state structure.
 * @param[out] found_device True if a device is found, false if no devices are found.
 *         If a device is found, the ROM Code can be obtained from the state.
 * @return status
 */
owb_ret_t owb_search_next(OneWireBus * bus, OneWireBus_SearchState * state, bool *found_device)
{
    owb_ret_t status = OWB_STATUS_NOT_SET;
    bool result = false;

    if (!bus || !state || !found_device)
    {
        status = OWB_STATUS_PARAMETER_NULL;
    }
    else if (!bus->is_init)
    {
        status = OWB_STATUS_NOT_INITIALIZED;
    }
    else
    {
        _search(bus, state, &result);
        status = OWB_STATUS_OK;

        *found_device = result;
    }

    return status;
}

/**
 * @brief Create a string representation of a ROM code, most significant byte (CRC8) first.
 * @param[in] rom_code The ROM code to convert to string representation.
 * @param[out] buffer The destination for the string representation. It will be null terminated.
 * @param[in] len The length of the buffer in bytes. 64-bit ROM codes require 16 characters
 *                to represent as a string, plus a null terminator, for 17 bytes.
 *                See OWB_ROM_CODE_STRING_LENGTH.
 * @return pointer to the byte beyond the last byte written
 */
char * owb_string_from_rom_code(OneWireBus_ROMCode *rom_code, char * buffer, size_t len)
{
    CY_ASSERT(sizeof(rom_code->bytes)*2+1<len);

    for (int i = sizeof(rom_code->bytes) - 1; i >= 0; i--)
    {
        sprintf(buffer, "%02x", rom_code->bytes[i]);
        buffer += 2;
    }
    return buffer;
}