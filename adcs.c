#include <inttypes.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>

#include "log.h"
#include "adcs.h"
#include "pinmap.h"


static uint8_t adc_channel_array[] = {4,6,7,8,9,10,11,12,13,14,15};

static volatile uint16_t adc_values[ARRAY_SIZE(adc_channel_array)];

typedef struct
{
    uint16_t max_value;
    uint16_t min_value;
    double   av_value;
}  adc_channel_info_t;

static adc_channel_info_t          adc_channel_info[ARRAY_SIZE(adc_channel_array)] = {{0}};

static volatile adc_channel_info_t adc_channel_info_cur[ARRAY_SIZE(adc_channel_array)] = {{0}};

static unsigned call_count = 0;

static volatile bool do_adcs = false;

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

//    rcc_periph_clock_enable(RCC_ADCS_ADC);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADCEN);
    rcc_periph_clock_enable(RCC_ADCS_DMA);

    dma_disable_channel(ADCS_DMA, ADCS_DMA_CHANNEL);

    dma_enable_circular_mode(ADCS_DMA, ADCS_DMA_CHANNEL);
    dma_enable_memory_increment_mode(ADCS_DMA, ADCS_DMA_CHANNEL);

    dma_set_peripheral_size(ADCS_DMA, ADCS_DMA_CHANNEL, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(ADCS_DMA, ADCS_DMA_CHANNEL, DMA_CCR_MSIZE_16BIT);

    dma_set_read_from_peripheral(ADCS_DMA, ADCS_DMA_CHANNEL);
    dma_set_peripheral_address(ADCS_DMA, ADCS_DMA_CHANNEL, (uint32_t) &ADC_DR(ADCS_ADC));

    dma_set_memory_address(ADCS_DMA, ADCS_DMA_CHANNEL, (uint32_t) &adc_values);
    dma_set_number_of_data(ADCS_DMA, ADCS_DMA_CHANNEL, ARRAY_SIZE(adc_values));

    dma_enable_transfer_complete_interrupt(ADCS_DMA, ADCS_DMA_CHANNEL);
    dma_enable_channel(ADCS_DMA, ADCS_DMA_CHANNEL);

    adc_power_off(ADCS_ADC);
    adc_set_clk_source(ADCS_ADC, ADC_CLKSOURCE_ADC);
    adc_calibrate(ADCS_ADC);
    adc_set_operation_mode(ADCS_ADC, ADC_MODE_SEQUENTIAL);
    adc_set_continuous_conversion_mode(ADCS_ADC);
    adc_set_right_aligned(ADCS_ADC);
    adc_set_regular_sequence(ADCS_ADC, ARRAY_SIZE(adc_channel_array), adc_channel_array);
    adc_set_sample_time_on_all_channels(ADCS_ADC, ADC_SMPTIME_071DOT5);
    adc_set_resolution(ADCS_ADC, ADC_RESOLUTION_12BIT);
    adc_power_on(ADCS_ADC);

    adc_enable_dma(ADCS_ADC);
    adc_enable_dma_circular_mode(ADCS_ADC);
    adc_start_conversion_regular(ADC1);
}


void adcs_do_samples()
{
    if (!do_adcs)
        return;
    call_count++;
    for(unsigned n = 0; n < ARRAY_SIZE(adc_channel_array); n++)
    {
        uint16_t adc = adc_values[n];

        adc_channel_info_t * channel_info = &adc_channel_info[n];

        if (adc > channel_info->max_value)
            channel_info->max_value = adc;

        if (adc < channel_info->min_value)
            channel_info->min_value = adc;

        channel_info->av_value += adc;
    }
}


void ADCS_DMA_CHANNEL_ISR(void)
{
    dma_clear_interrupt_flags(ADCS_DMA, ADCS_DMA_CHANNEL, DMA_IFCR_CGIF1);
}


void adcs_second_boardary()
{
    if (!do_adcs)
        return;
    for(unsigned n = 0; n < ARRAY_SIZE(adc_channel_array); n++)
    {
        adc_channel_info_t * channel_info = &adc_channel_info[n];
        unsigned count = call_count;

        adc_channel_info_cur[n] = *channel_info;

        channel_info->max_value = 0;
        channel_info->min_value = 0xFFFF;
        channel_info->av_value  = 0;

        adc_channel_info_cur[n].av_value /= count;
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
        *av_value = channel_info->av_value;
}


uint16_t adcs_get_now(unsigned adc)
{
    if (adc >= ARRAY_SIZE(adc_channel_array))
        return 0;

    return adc_values[adc];
}


unsigned adcs_get_count()
{
    return ARRAY_SIZE(adc_channel_array);
}


void     adcs_enable_sampling(bool enabled)
{
    do_adcs = enabled;
}


bool     adcs_is_enabled()
{
    return do_adcs;
}


void adcs_adc_log(unsigned adc)
{
    if (adc >= ARRAY_SIZE(adc_channel_array))
        return;

    volatile adc_channel_info_t * channel_info = &adc_channel_info_cur[adc];

    log_out("ADC : %u (Channel : %u)", adc, adc_channel_array[adc]);
    log_out("Min : %u", channel_info->min_value);
    log_out("Max : %u", channel_info->max_value);
    log_out("Avg : %f", channel_info->av_value);
}


void adcs_log()
{
    for(unsigned n = 0; n < ARRAY_SIZE(adc_channel_info_cur); n++)
        adcs_adc_log(n);
}
