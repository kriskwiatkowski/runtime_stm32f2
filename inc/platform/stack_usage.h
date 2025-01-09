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

/**
 * \file stack_usage.h
 * \brief Utility for stack usage measurement.
 *
 * This is a utility that attempt to measure stack usage. It relies on
 * undefined behavior and works by assuming that the stack grows downward
 * in memory.
 *
 * The usage is:
 *   utils_stack_poison();
 *   FUNCTION_UNDER_TEST();
 *   unsigned stack_usage = utils_stack_measure();
 *
 * One may also want to use MEASURE_STACK_SIZE() macro.
 *
 * NOTE: Those macros work better in debug mode. In release mode compiler
 *       may optimize utils_stack_poison function away.
 */

#include <platform/utils.h>

#ifndef STACK_USAGE_MAX_STACK
// Assume 100KB of stack
#define STACK_USAGE_MAX_STACK 100 * 1024u
#endif

#ifndef STACK_USAGE_CANARY
#define STACK_USAGE_CANARY 'x'
#endif

/**
 * Fill in the array with the known pattern.
 *
 * It must be unlikely that function unders test generates
 * the value of known pattern.
 *
 * The all optimisations must be disabled for this function,
 * otherwise compiler will optimize out the operation.
 */
SWITCH_OPTIMS_OFF static void utils_stack_poison(void) {
    size_t n = STACK_USAGE_MAX_STACK;
    unsigned char array[STACK_USAGE_MAX_STACK];
    volatile unsigned char *v = &array[0];

    while (n--) { *v++ = STACK_USAGE_CANARY; }
}

/**
 * Scan the stack for where the known pattern.
 *
 * We assume pattern on the stack will be overwritten by the
 * stack of the function under test.

 * The all optimisations must be disabled for this function,
 * otherwise compiler will optimize out the operation.
 */
SWITCH_OPTIMS_OFF static unsigned utils_stack_measure(void) {
    /* Define the same array (in the same place on the stack) and */
    /* scan for where the known pattern was overwritten */
    unsigned char array[STACK_USAGE_MAX_STACK];
    unsigned i;
    for (i = 0; i < STACK_USAGE_MAX_STACK; i++) {
        if (array[i] != STACK_USAGE_CANARY)
            break;
    }
    return STACK_USAGE_MAX_STACK - i;
}

// Handy macro that implements the measurement gatthering. If function
// returns 0, then stack size couldn't be calculated correctly.
#define MEASURE_STACK_SIZE(size, r, func_call)             \
    do {                                                   \
        utils_stack_poison();                              \
        r    = func_call;                                  \
        size = utils_stack_measure();                      \
        size = (size == STACK_USAGE_MAX_STACK) ? 0 : size; \
    } while (0)
