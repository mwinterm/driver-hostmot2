/*
 * How the shim publishes a pin, checked without a board: a HAL_IO pin and a
 * HAL_IN pin the driver writes anyway -- the encoder's `probe-enable` -- go
 * across the channel both ways; any other HAL_IN pin goes one way, from the
 * control.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdio.h>
#include <string.h>

#include <rtapi.h>
#include <hal.h>

#include "hm2_shim.h"

/* How many times `name` is published toward the control, and from it. */
static void count(const char *name, int *to_core, int *from_core) {
    *to_core = 0;
    *from_core = 0;
    for (size_t i = 0; i < hm2_shim_signal_count(); i++) {
        const hm2_shim_signal *signal = hm2_shim_signal_at(i);
        if (strcmp(signal->name, name) != 0) {
            continue;
        }
        if (signal->dir == HM2_SHIM_TO_CORE) {
            (*to_core)++;
        } else {
            (*from_core)++;
        }
    }
}

static int expect(const char *name, int to_core, int from_core) {
    int to = 0;
    int from = 0;
    count(name, &to, &from);
    if (to != to_core || from != from_core) {
        fprintf(stderr, "%s: published %d toward the control and %d from it; expected %d and %d\n",
                name, to, from, to_core, from_core);
        return 1;
    }
    printf("%s: %s\n", name, to && from ? "both ways" : from ? "from the control" : "to the control");
    return 0;
}

int main(void) {
    if (hm2_shim_init(1 << 16, 32, 4) != 0) {
        return 1;
    }
    int comp = hal_init("t");
    hal_bool_t probe_enable;
    hal_bool_t index_enable;
    hal_bool_t probe_invert;
    hal_bool_t input;
    if (comp < 0 ||
        hal_pin_new_bool(comp, HAL_IN, &probe_enable, 0, "t.encoder.%02d.probe-enable", 0) ||
        hal_pin_new_bool(comp, HAL_IO, &index_enable, 0, "t.encoder.%02d.index-enable", 0) ||
        hal_pin_new_bool(comp, HAL_IN, &probe_invert, 0, "t.encoder.%02d.probe-invert", 0) ||
        hal_pin_new_bool(comp, HAL_OUT, &input, 0, "t.gpio.%03d.in", 0)) {
        fprintf(stderr, "the pins were not created\n");
        return 1;
    }
    int failures = 0;
    failures += expect("t.encoder.00.probe-enable", 1, 1);
    failures += expect("t.encoder.00.index-enable", 1, 1);
    failures += expect("t.encoder.00.probe-invert", 0, 1);
    failures += expect("t.gpio.000.in", 1, 0);
    hm2_shim_fini();
    return failures ? 1 : 0;
}
