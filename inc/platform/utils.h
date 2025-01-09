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
 * \file utils.h
 * \brief Miscellaneous utilities used by library tests.
 */

/* Switches off compiler optimizations for selected function
    Usage:
        SWITCH_OPTIMS_OFF
        func() {...}
        SWITCH_OPTIMS_ON
*/
#if defined(__ARMCC__)
#define SWITCH_OPTIMS_OFF _Pragma("push") _Pragma("O0")
#define SWITCH_OPTIMS_ON _Pragma("pop")
#elif defined(__clang__)
#define SWITCH_OPTIMS_OFF __attribute__((optnone))
#define SWITCH_OPTIMS_ON
#elif defined(__GNUC__)
#define SWITCH_OPTIMS_OFF __attribute__((optimize("O0")))
#define SWITCH_OPTIMS_ON
#else
#error "SWITCH_OPTIMS_OFF not supported by compiler"
#endif
