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
 *
 */
#define STM32

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <platform/platform.h>
#include <platform/printf.h>
#include <stddef.h>

extern volatile unsigned long long stm32_sys_tick_overflowcnt;

void hard_fault_handler(void) {
    __asm volatile(
        "TST lr, #4 \n"              // Test EXC_RETURN bit 2
        "ITE EQ \n"                  // If bit is 0, go to msp
        "MRSEQ r0, MSP \n"           // Use MSP
        "MRSNE r0, PSP \n"           // Use PSP
        "B hard_fault_handler_c \n"  // Branch to C handler
    );
}

void hard_fault_handler_c(uint32_t *stacked_args) {
    volatile uint32_t stacked_r0  = stacked_args[0];
    volatile uint32_t stacked_r1  = stacked_args[1];
    volatile uint32_t stacked_r2  = stacked_args[2];
    volatile uint32_t stacked_r3  = stacked_args[3];
    volatile uint32_t stacked_r12 = stacked_args[4];
    volatile uint32_t stacked_lr  = stacked_args[5];
    volatile uint32_t stacked_pc  = stacked_args[6];
    volatile uint32_t stacked_psr = stacked_args[7];

    // You can log these registers using a debugging interface
    // Example:
    // debug_print("HardFault: R0 = 0x%08X\n", stacked_r0);
    // debug_print("R1 = 0x%08X\n", stacked_r1);
    // debug_print("R2 = 0x%08X\n", stacked_r2);
    // debug_print("R3 = 0x%08X\n", stacked_r3);
    // debug_print("R12 = 0x%08X\n", stacked_r12);
    // debug_print("LR = 0x%08X\n", stacked_lr);
    // debug_print("PC = 0x%08X\n", stacked_pc);
    // debug_print("PSR = 0x%08X\n", stacked_psr);

    // Optional: Inspect the HardFault Status Register
    volatile uint32_t hfsr = SCB_HFSR;
    // debug_print("HFSR = 0x%08X\n", hfsr);

    // Optional: Inspect Configurable Fault Status Register (CFSR)
    volatile uint32_t cfsr = SCB_CFSR;
    // debug_print("CFSR = 0x%08X\n", cfsr);

    // Optional: BusFault Address Register (BFAR)
    volatile uint32_t bfar = SCB_BFAR;
    // debug_print("BFAR = 0x%08X\n", bfar);

    // Optional: Memory Management Fault Address Register (MMFAR)
    volatile uint32_t mmfar = SCB_MMFAR;
    // debug_print("MMFAR = 0x%08X\n", mmfar);

    // Infinite loop to halt the program for debugging
    while (1)
        ;
}

void platform_sync(void) {
    // wait for the first systick overflow
    // improves reliability of the benchmarking scripts since it makes it much
    // less likely that the host will miss the start of the output
    unsigned long long old = stm32_sys_tick_overflowcnt;
    while (old == stm32_sys_tick_overflowcnt)
        ;
}

void sys_tick_handler(void) { ++stm32_sys_tick_overflowcnt; }
