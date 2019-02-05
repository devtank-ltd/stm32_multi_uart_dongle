#include <inttypes.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>


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

    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_DMA1);

    adc_power_off(ADC1);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    adc_calibrate(ADC1);
    adc_set_operation_mode(ADC1, ADC_MODE_SEQUENTIAL);
    adc_set_continuous_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_regular_sequence(ADC1, ARRAY_SIZE(adc_channel_array), adc_channel_array);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    adc_enable_dma(ADC1);

    dma_disable_channel(DMA1, DMA_CHANNEL1);

    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t) &ADC_DR(ADC1));
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);

    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t) &adc_values);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, ARRAY_SIZE(adc_values));
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);

    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);

    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

    adc_power_on(ADC1);
    // wait for ADC
    for (int i = 0; i < 800000; i++)
        asm("nop");

    dma_enable_channel(DMA1, DMA_CHANNEL1);
    adc_start_conversion_regular(ADC1);
}


void dma1_channel1_isr(void)
{
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_IFCR_CGIF1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);
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
