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

#include <libopencm3/stm32/rng.h>
#include <stdint.h>

int platform_get_random(void *out, unsigned len) {
    union {
        unsigned char aschar[4];
        uint32_t asint;
    } random;

    uint8_t *obuf = (uint8_t *)out;

    while (len > 4) {
        random.asint = rng_get_random_blocking();
        *obuf++      = random.aschar[0];
        *obuf++      = random.aschar[1];
        *obuf++      = random.aschar[2];
        *obuf++      = random.aschar[3];
        len          -= 4;
    }
    if (len > 0) {
        for (random.asint = rng_get_random_blocking(); len > 0; --len) {
            *obuf++ = random.aschar[len - 1];
        }
    }

    return 0;
}
