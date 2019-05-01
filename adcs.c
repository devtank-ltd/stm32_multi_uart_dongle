#include <inttypes.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>

#include "log.h"
#include "adcs.h"
#include "pinmap.h"


static uint8_t adc_channel_array[] = ADC_CHANNELS;

typedef struct
{
    unsigned max_value;
    unsigned min_value;
    uint64_t total_value;
    unsigned count;
} adc_channel_info_t;


static volatile adc_channel_info_t adc_channel_info[ARRAY_SIZE(adc_channel_array)] = {{0}};

static volatile adc_channel_info_t adc_channel_info_cur[ARRAY_SIZE(adc_channel_array)] = {{0}};

static volatile uint16_t last_value[ARRAY_SIZE(adc_channel_array)] = {0};

static unsigned call_count = 0;
static unsigned adc_index = ARRAY_SIZE(adc_channel_array) - 1;


void adcs_init()
{
    const port_n_pins_t port_n_pins[] = ADCS_PORT_N_PINS;

    for(unsigned n = 0; n < ARRAY_SIZE(port_n_pins); n++)
    {
        rcc_periph_clock_enable(PORT_TO_RCC(port_n_pins[n].port));
        gpio_mode_setup(port_n_pins[n].port,
                        GPIO_MODE_ANALOG,
                        GPIO_PUPD_NONE,
                        port_n_pins[n].pins);
    }

    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADCEN);

    adc_power_off(ADC1);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    adc_calibrate(ADC1);
    adc_set_operation_mode(ADC1, ADC_MODE_SEQUENTIAL);
    adc_set_continuous_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_regular_sequence(ADC1, ARRAY_SIZE(adc_channel_array), adc_channel_array);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    adc_power_on(ADC1);
}


void adcs_do_samples()
{
    call_count++;

    unsigned state = call_count % 2;

    switch(state)
    {
        case 0: adc_start_conversion_regular(ADC1); break;
        case 1:
        {
            if (!adc_eoc(ADC1))
                log_error("ADC sampling not complete!");

            uint32_t adc = adc_read_regular(ADC1);

            volatile adc_channel_info_t * channel_info = &adc_channel_info[adc_index];

            if (adc > channel_info->max_value)
                channel_info->max_value = adc;

            if (adc < channel_info->min_value)
                channel_info->min_value = adc;

            channel_info->total_value += adc;
            channel_info->count ++;
            last_value[adc_index]   = adc;

            adc_index = (adc_index + 1) % ARRAY_SIZE(adc_channel_array);
        }
    }
}


void adcs_second_boardary()
{
    unsigned sample_count = call_count / ARRAY_SIZE(adc_channel_array) / 2;

    log_debug("ADCS SPS %u", sample_count);

    for(unsigned n = 0; n < ARRAY_SIZE(adc_channel_array); n++)
    {
        volatile adc_channel_info_t * channel_info = &adc_channel_info[n];

        adc_channel_info_cur[n] = *channel_info;

        channel_info->max_value = 0;
        channel_info->min_value = 0xFFFFFFFF;
        channel_info->total_value  = 0;
        channel_info->count = 0;
    }

    call_count = 0;
}


void adcs_get(unsigned   adc,
              unsigned * min_value,
              unsigned * max_value,
              double *   av_value)
{
    if (adc >= ARRAY_SIZE(adc_channel_array))
    {
        if (max_value)
            *max_value = 0;
        if (min_value)
            *min_value = 0;
        if (av_value)
            *av_value = 0;
        return;
    }

    volatile adc_channel_info_t * channel_info = &adc_channel_info_cur[adc];

    if (max_value)
        *max_value = channel_info->max_value;
    if (min_value)
        *min_value = channel_info->min_value;
    if (av_value)
        *av_value = ((double)channel_info->total_value) / channel_info->count;
}


unsigned adcs_get_count()
{
    return ARRAY_SIZE(adc_channel_array);
}


unsigned  adcs_get_last(unsigned adc)
{
    if (adc >= ARRAY_SIZE(adc_channel_array))
        return 0;

    return last_value[adc];
}


unsigned  adcs_get_tick(unsigned adc)
{
    if (adc >= ARRAY_SIZE(adc_channel_array))
        return 0;

    return adc_channel_info[adc].count;
}


void adcs_adc_log(unsigned adc)
{
    if (adc >= ARRAY_SIZE(adc_channel_array))
        return;

    volatile adc_channel_info_t * channel_info = &adc_channel_info_cur[adc];

    log_out("ADC : %u (Channel : %u)", adc, adc_channel_array[adc]);
    log_out("Min : %u", channel_info->min_value);
    log_out("Max : %u", channel_info->max_value);
    log_out("Avg : %f", ((double)channel_info->total_value) / channel_info->count );
}


void adcs_log()
{
    for(unsigned n = 0; n < ARRAY_SIZE(adc_channel_info_cur); n++)
        adcs_adc_log(n);
}
