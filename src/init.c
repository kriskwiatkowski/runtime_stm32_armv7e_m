/*
 * Copyright (C) Kris Kwiatkowski, Among Bytes LTD
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.  See <http://www.fsf.org/copyleft/gpl.txt>.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rng.h>
#include <libopencm3/stm32/usart.h>
#include <platform/stm32f4.h>
#include <stddef.h>

/// ############################
/// Internal implementation
/// ############################

volatile unsigned long long stm32_sys_tick_overflowcnt = 0;

/* 24 MHz */
static const struct rcc_clock_scale clock_24MHZ = {
    .pllm  = 8,    //VCOin = HSE / PLLM = 1 MHz
    .plln  = 192,  //VCOout = VCOin * PLLN = 192 MHz
    .pllp  = 8,    //PLLCLK = VCOout / PLLP = 24 MHz (low to have 0WS)
    .pllq  = 4,    //PLL48CLK = VCOout / PLLQ = 48 MHz (required for USB, RNG)
    .pllr  = 0,
    .hpre  = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_2,
    .ppre2 = RCC_CFGR_PPRE_DIV_NONE,
    .pll_source     = RCC_CFGR_PLLSRC_HSE_CLK,
    .voltage_scale  = PWR_SCALE1,
    .flash_config   = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_0WS,
    .ahb_frequency  = 24000000,
    .apb1_frequency = 12000000,
    .apb2_frequency = 24000000,
};

static void gpio_setup(void) {
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
}

static void usart_setup(int baud) {
    usart_set_baudrate(USART2, baud);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable(USART2);
}

static void systick_setup(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(16777215);
    systick_interrupt_enable();

    systick_counter_enable();
}

static void reset_clock(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_RNG);

    flash_prefetch_enable();
    systick_setup();

    // wait for the first systick overflow
    // improves reliability of the benchmarking scripts since it makes it much
    // less likely that the host will miss the start of the output
    unsigned long long old = stm32_sys_tick_overflowcnt;
    while (old == stm32_sys_tick_overflowcnt) {};
}

static void set_clock(platform_attr_stm32_t a) {
    if (a == PLATFORM_CLOCK_MAX) {
        rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    } else if (a == PLATFORM_CLOCK_24MHZ) {
        rcc_clock_setup_pll(&clock_24MHZ);
    } else {
        // Do nothing
        return;
    }
    reset_clock();
}

// Implements printf. Send a char to the terminal.
void _putchar(char character) {
    usart_send_blocking(USART2, (unsigned char)(character));
}

/// ############################
/// External API
/// ############################

int platform_init(void) {
    set_clock(PLATFORM_CLOCK_24MHZ);
    gpio_setup();
    usart_setup(115200);
    systick_setup();
    rng_enable();
    return 0;
}

void platform_set_attr(const struct platform_attr_t *a) {
    size_t i;
    for (i = 0; i < a->n; i++) {
        switch (a->attr[i]) {
            case PLATFORM_CLOCK_24MHZ:
                set_clock(PLATFORM_CLOCK_24MHZ);
                break;
            case PLATFORM_CLOCK_MAX:
                set_clock(PLATFORM_CLOCK_MAX);
            default:
                break;
        }
    }
}

uint64_t platform_cpu_cyclecount(void) {
    while (true) {
        unsigned long long before = stm32_sys_tick_overflowcnt;
        unsigned long long result =
            (before + 1) * 16777216llu - systick_get_value();
        if (stm32_sys_tick_overflowcnt == before) {
            return result;
        }
    }
}
