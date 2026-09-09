/*
 * HAL parameter values from the host's configuration file.
 *
 * A HAL parameter is configuration -- an encoder's scale, a stepgen's step
 * timing, whether a GPIO pin is an output. In LinuxCNC a `setp` line in the
 * HAL file sets it after the driver loads. Here the host's configuration file
 * sets it *before* the driver reads it, which is both simpler and stricter:
 * there is no moment at which the driver is running on a default nobody chose.
 *
 * The table is small -- tens of entries -- and consulted once per declaration,
 * at start-up, so a linear search is the right shape and a hash would be a
 * data structure to maintain for no reason.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 the driver-hostmot2 contributors
 */
#include <stdlib.h>
#include <string.h>

#include <rtapi.h>

#include "hm2_shim.h"
#include "hm2_shim_internal.h"

/* What the configuration asked for. */
static const char **param_names;
static const double *param_values;
static size_t param_count;
/* Which of them a declaration actually claimed. */
static unsigned char *param_claimed;

/* What the driver declared, so the host can report the whole surface. */
typedef struct {
    const char *name;
    hm2_shim_type type;
    void *cell;
} declared_param;

static declared_param *declared;
static size_t declared_count;
static size_t declared_capacity;

void hm2_shim_set_params(const char **names, const double *values, size_t count) {
    free(param_claimed);
    param_names = names;
    param_values = values;
    param_count = count;
    param_claimed = count ? calloc(count, 1) : NULL;
}

int hm2_shim_param_lookup(const char *name, double *out) {
    if (!name || !param_names) {
        return 0;
    }
    for (size_t i = 0; i < param_count; i++) {
        if (strcmp(param_names[i], name) == 0) {
            if (param_claimed) {
                param_claimed[i] = 1;
            }
            if (out) {
                *out = param_values[i];
            }
            return 1;
        }
    }
    return 0;
}

void hm2_shim_note_param(const char *name, hm2_shim_type type, void *cell) {
    if (declared_count == declared_capacity) {
        size_t grown = declared_capacity ? declared_capacity * 2 : 256;
        declared_param *bigger = realloc(declared, grown * sizeof(*bigger));
        if (!bigger) {
            return;
        }
        declared = bigger;
        declared_capacity = grown;
    }
    declared[declared_count].name = name;
    declared[declared_count].type = type;
    declared[declared_count].cell = cell;
    declared_count++;
}

size_t hm2_shim_declared_param_count(void) { return declared_count; }

const char *hm2_shim_declared_param_at(size_t index, double *value) {
    if (index >= declared_count) {
        return NULL;
    }
    if (value) {
        hm2_shim_signal view = {
            .name = declared[index].name,
            .type = declared[index].type,
            .dir = HM2_SHIM_TO_CORE,
            .cell = declared[index].cell,
        };
        *value = hm2_shim_cell_get(&view);
    }
    return declared[index].name;
}

/*
 * Parameters the configuration set that no declaration claimed.
 *
 * Almost always a typo, and the reason to report it is that the alternative is
 * silence: a misspelled `hm2_7i92.0.encoder.00.scal` leaves the encoder on its
 * default scale, the machine moves by the wrong distance, and nothing anywhere
 * says why. The same argument the kinematics plugins make for declaring their
 * parameters (ADR 0013).
 */
size_t hm2_shim_unclaimed_params(const char **into, size_t capacity) {
    size_t found = 0;
    for (size_t i = 0; i < param_count; i++) {
        if (param_claimed && param_claimed[i]) {
            continue;
        }
        if (into && found < capacity) {
            into[found] = param_names[i];
        }
        found++;
    }
    return found;
}
