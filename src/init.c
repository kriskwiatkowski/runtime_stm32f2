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

#define SERIAL_GPIO GPIOD
#define SERIAL_USART USART3
#define SERIAL_PINS (GPIO8 | GPIO9)

#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rng.h>
#include <libopencm3/stm32/usart.h>
#include <platform/platform.h>
#include <stddef.h>

#include "printf.h"

static uint32_t _clock_freq;

#ifdef STM32F2
extern uint32_t rcc_apb1_frequency;
extern uint32_t rcc_apb2_frequency;
#endif

/// ############################
/// Internal implementation
/// ############################

volatile unsigned long long stm32_sys_tick_overflowcnt = 0;

static void usart_setup(int baud) {
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_USART3);
    gpio_set_output_options(SERIAL_GPIO, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
                            SERIAL_PINS);
    gpio_mode_setup(SERIAL_GPIO, GPIO_MODE_AF, GPIO_PUPD_PULLUP, SERIAL_PINS);
    gpio_set_af(SERIAL_GPIO, GPIO_AF7, SERIAL_PINS);
    usart_set_baudrate(SERIAL_USART, baud);
    usart_set_databits(SERIAL_USART, 8);
    usart_set_stopbits(SERIAL_USART, USART_STOPBITS_1);
    usart_set_mode(SERIAL_USART, USART_MODE_TX_RX);
    usart_set_parity(SERIAL_USART, USART_PARITY_NONE);
    usart_set_flow_control(SERIAL_USART, USART_FLOWCONTROL_NONE);
    usart_disable_rx_interrupt(SERIAL_USART);
    usart_disable_tx_interrupt(SERIAL_USART);
    usart_enable(SERIAL_USART);
}

static void systick_setup(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(0xFFFFFFu);
    systick_interrupt_enable();
    systick_counter_enable();
}

static void set_clock(platform_op_mode_t a) {
    size_t flash_wait_state;

    /* Some STM32 Platform */
    rcc_periph_clock_enable(RCC_RNG);
    rcc_periph_clock_enable(RCC_GPIOH);
    /* All of them use an external oscillator with bypass. */
    rcc_osc_off(RCC_HSE);
    rcc_osc_bypass_enable(RCC_HSE);
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);

    if (a == PLATFORM_CLOCK_MAX) {
        rcc_ahb_frequency  = 120000000;
        rcc_apb1_frequency = 30000000;
        rcc_apb2_frequency = 60000000;
        _clock_freq        = 120000000;
        flash_wait_state   = FLASH_ACR_LATENCY_3WS;

        rcc_set_hpre(RCC_CFGR_HPRE_DIV_NONE);
        rcc_set_ppre1(RCC_CFGR_PPRE_DIV_4);
        rcc_set_ppre2(RCC_CFGR_PPRE_DIV_2);
    } else if (a == PLATFORM_CLOCK_USERSPACE) {
        rcc_ahb_frequency  = 30000000;
        rcc_apb1_frequency = 30000000;
        rcc_apb2_frequency = 30000000;
        _clock_freq        = 30000000;
        flash_wait_state   = FLASH_ACR_LATENCY_0WS;

        rcc_set_hpre(RCC_CFGR_HPRE_DIV_4);
        rcc_set_ppre1(RCC_CFGR_PPRE_DIV_NONE);
        rcc_set_ppre2(RCC_CFGR_PPRE_DIV_NONE);
    } else {
        // Do nothing
        return;
    }
    rcc_osc_off(RCC_PLL);

    /* Configure the PLL oscillator (use CUBEMX tool). */
    rcc_set_main_pll_hse(8, 240, 2, 5);

    /* Enable PLL oscillator and wait for it to stabilize. */
    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);
    flash_dcache_enable();
    flash_icache_enable();
    flash_set_ws(FLASH_ACR_LATENCY_3WS);
    flash_prefetch_enable();

    rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
    rcc_wait_for_sysclk_status(RCC_PLL);
}

static void setup_rng(void) { rng_enable(); }

// Implements printf. Send a char to the terminal.
void _putchar(char character) {
    usart_send_blocking(SERIAL_USART, (unsigned char)(character));
}

static volatile unsigned long long overflowcnt = 0;

/// ############################
/// External API
/// ############################

int platform_init(platform_op_mode_t a) {
    set_clock(a);
    setup_rng();
    usart_setup(115200);
    systick_setup();
    // wait for the first systick overflow
    // improves reliability of the benchmarking scripts since it makes it much
    // less likely that the host will miss the start of the output
    return 0;
}

void platform_set_attr(const struct platform_attr_t *a) {
    size_t i;
    for (i = 0; i < a->n; i++) {
        switch (a->attr[i]) {
            case PLATFORM_CLOCK_USERSPACE:
                set_clock(PLATFORM_CLOCK_USERSPACE);
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
