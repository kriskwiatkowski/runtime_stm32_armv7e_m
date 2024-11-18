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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <platform/printf.h>
#include <platform/stm32f4.h>
#include <stddef.h>

extern volatile unsigned long long stm32_sys_tick_overflowcnt;

void hard_fault_handler(void) {
    // Read the fault status registers using libopencm3 macros
    uint32_t cfsr  = SCB_CFSR;   // Configurable Fault Status Register
    uint32_t hfsr  = SCB_HFSR;   // HardFault Status Register
    uint32_t mmfar = SCB_MMFAR;  // MemManage Fault Address Register
    uint32_t bfar  = SCB_BFAR;   // BusFault Address Register

    // Output fault information (ensure UART/SWO is configured to handle printf)
    printf("\n-----------------------------------------\n");
    printf("\n Fault handler:\n");
    printf("\n-----------------------------------------\n");
    printf("CFSR: 0x%08lx\n", cfsr);
    printf("HFSR: 0x%08lx\n", hfsr);

    // Check if a MemManage fault occurred (CFSR bits [7:0])
    if (cfsr & (1 << 0)) {
        printf("Instruction access violation\n");
    }
    if (cfsr & (1 << 1)) {
        printf("Data access violation\n");
    }
    if (cfsr & (1 << 7)) {
        printf("MemManage Fault at address: 0x%08lx\n", mmfar);
    }

    // Check if a BusFault occurred (CFSR bits [15:8])
    if (cfsr & (1 << 8)) {
        printf("Instruction bus error\n");
    }
    if (cfsr & (1 << 9)) {
        printf("Precise data bus error\n");
    }
    if (cfsr & (1 << 15)) {
        printf("Bus Fault at address: 0x%08lx\n", bfar);
    }

    // Infinite loop to indicate HardFault occurred
    while (1)
        ;
}

void sys_tick_handler(void) { ++stm32_sys_tick_overflowcnt; }
