/*
 * The HAL half of the shim (ADR 0022 §4).
 *
 * Pins, parameters, the arena and the exported functions. What HAL does with
 * shared memory, a name server and a linking step, this does with an array:
 * a pin is a cell plus a name, and what a name *means* is settled on the other
 * side of the channel by the machine description (ADR 0022 §5).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 the driver-hostmot2 contributors
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include <rtapi.h>
#include <hal.h>

#include "hm2_shim.h"
#include "hm2_shim_internal.h"

/*
 * The cell every accessor in hal.h casts a handle to. Restated here rather
 * than included because hal.h only defines it inside the accessor functions,
 * under a macro it then undefines. It is eight bytes and the members overlap;
 * `hal_set_bool` writes the whole width, so a bool cell read as a uint is 0
 * or 1 rather than 1 with rubbish above it.
 */
typedef union {
    volatile rtapi_bool b;
    volatile rtapi_s32 ss;
    volatile rtapi_u32 su;
    volatile rtapi_sint s;
    volatile rtapi_uint u;
    volatile rtapi_real r;
} hm2_cell;

/* ---------------------------------------------------------------------------
 * The arena
 * ------------------------------------------------------------------------- */

static char *arena;
static size_t arena_size;
static size_t arena_used;
/* Set once the arena has been exhausted, so the message is said once. */
static int arena_overflowed;

static hm2_shim_signal *signals;
static size_t signal_capacity;
static size_t signal_count;

static hm2_shim_funct *functs;
static size_t funct_capacity;
static size_t funct_count;

/* The component the driver created. One is all hostmot2 asks for. */
static char component_name[HAL_NAME_LEN + 1];
static int component_id;
static int component_ready;

int hm2_shim_init(size_t arena_bytes, size_t max_signals, size_t max_functs) {
    arena = calloc(1, arena_bytes);
    signals = calloc(max_signals, sizeof(*signals));
    functs = calloc(max_functs, sizeof(*functs));
    if (!arena || !signals || !functs) {
        hm2_shim_log(RTAPI_MSG_ERR, "shim: could not allocate the arena (%zu bytes) "
                                    "or its tables", arena_bytes);
        hm2_shim_fini();
        return -1;
    }
    arena_size = arena_bytes;
    arena_used = 0;
    arena_overflowed = 0;
    signal_capacity = max_signals;
    signal_count = 0;
    funct_capacity = max_functs;
    funct_count = 0;

    /*
     * Locked here, with everything else the cyclic path touches. `calloc` has
     * already faulted every page in, so this only keeps them; a failure is a
     * warning rather than a refusal, for the reason the core makes the same
     * judgement about itself -- a developer's machine without CAP_IPC_LOCK
     * should still be able to run the thing.
     */
    if (mlock(arena, arena_size) != 0) {
        hm2_shim_log(RTAPI_MSG_WARN,
                     "shim: could not lock the arena into memory; a page fault in the "
                     "cyclic path will show up as a late cycle. Grant CAP_IPC_LOCK and "
                     "raise RLIMIT_MEMLOCK on a real-time host");
    }
    return 0;
}

void hm2_shim_fini(void) {
    if (arena) {
        munlock(arena, arena_size);
        free(arena);
    }
    arena = NULL;
    arena_size = arena_used = 0;
    free(signals);
    signals = NULL;
    signal_capacity = signal_count = 0;
    free(functs);
    functs = NULL;
    funct_capacity = funct_count = 0;
    component_ready = 0;
    component_id = 0;
}

/*
 * HAL's allocator, and the driver's only one after start-up.
 *
 * Bump-allocated and never freed, which is what the real-time contract asks
 * for and also what the driver expects: HAL never frees either, and the whole
 * arena goes away when the component does.
 */
void *hal_malloc(long int size) {
    if (size < 0) {
        return NULL;
    }
    /* Every cell the accessors touch is eight bytes and must be aligned. */
    size_t want = ((size_t)size + 7u) & ~(size_t)7u;
    if (!arena || want > arena_size - arena_used) {
        if (!arena_overflowed) {
            arena_overflowed = 1;
            hm2_shim_log(RTAPI_MSG_ERR,
                         "shim: the arena is full: %zu of %zu bytes used and %ld more "
                         "asked for. Raise arena_bytes in the host configuration -- it is "
                         "sized once, at start-up, because allocating later would allocate "
                         "in the cyclic path",
                         arena_used, arena_size, size);
        }
        return NULL;
    }
    void *block = arena + arena_used;
    arena_used += want;
    return block;
}

size_t hm2_shim_arena_used(void) { return arena_used; }
size_t hm2_shim_arena_size(void) { return arena_size; }

/* ---------------------------------------------------------------------------
 * The component
 * ------------------------------------------------------------------------- */

int hal_init(const char *name) {
    if (!name) {
        return -EINVAL;
    }
    if (component_id != 0) {
        hm2_shim_log(RTAPI_MSG_ERR,
                     "shim: the driver asked for a second component ('%s' after '%s'); "
                     "this host runs one",
                     name, component_name);
        return -EINVAL;
    }
    snprintf(component_name, sizeof(component_name), "%s", name);
    /* Any positive number. HAL's is an index into its component table; here it
       is only something to hand back to the calls that take it. */
    component_id = 1;
    component_ready = 0;
    hm2_shim_log(RTAPI_MSG_INFO, "shim: component '%s' created", component_name);
    return component_id;
}

int hal_exit(int comp_id) {
    (void)comp_id;
    component_id = 0;
    component_ready = 0;
    return 0;
}

/*
 * In HAL this makes the component's pins visible to everything else. Here the
 * host has been able to see them all along -- there is nothing to publish to
 * but the channel, and the host builds that after `rtapi_app_main` returns.
 * So this records that the driver considers itself finished declaring, which
 * is worth knowing when a later declaration turns up.
 */
int hal_ready(int comp_id) {
    (void)comp_id;
    component_ready = 1;
    hm2_shim_log(RTAPI_MSG_INFO, "shim: component '%s' ready: %zu pin(s), %zu function(s), "
                                 "%zu of %zu arena bytes",
                 component_name, signal_count, funct_count, arena_used, arena_size);
    return 0;
}

int hm2_shim_component_ready(void) { return component_ready; }
const char *hm2_shim_component_name(void) { return component_name; }

/* ---------------------------------------------------------------------------
 * Pins and parameters
 * ------------------------------------------------------------------------- */

/* Copies a formatted name into the arena, so it outlives the caller's stack. */
static const char *arena_name(const char *fmt, va_list args) {
    char buffer[HAL_NAME_LEN + 1];
    int written = vsnprintf(buffer, sizeof(buffer), fmt, args);
    if (written < 0) {
        return NULL;
    }
    if ((size_t)written >= sizeof(buffer)) {
        /*
         * Refused rather than kept: two names cut to the same bytes would be
         * two pins the machine description cannot tell apart, and the first
         * symptom would be one axis moving when another was commanded.
         */
        hm2_shim_log(RTAPI_MSG_ERR, "shim: the pin name '%s' is longer than HAL's %d "
                                    "characters and is refused rather than truncated",
                     buffer, HAL_NAME_LEN);
        return NULL;
    }
    size_t bytes = (size_t)written + 1;
    char *copy = hal_malloc((long)bytes);
    if (!copy) {
        return NULL;
    }
    memcpy(copy, buffer, bytes);
    return copy;
}

/*
 * The direction a pin crosses the channel.
 *
 * HAL's directions are the component's: HAL_IN is a pin the driver reads, so
 * across the channel it is the core writing. The words invert; see the note on
 * `hm2_shim_dir`. HAL_IO is both, and is handled by the caller.
 */
static hm2_shim_dir channel_dir(hal_pdir_t dir) {
    return (dir & HAL_IN) ? HM2_SHIM_FROM_CORE : HM2_SHIM_TO_CORE;
}

/* Records one published signal. Returns its cell, or NULL. */
static void *publish(const char *name, hm2_shim_type type, hm2_shim_dir dir, void *cell) {
    if (signal_count >= signal_capacity) {
        hm2_shim_log(RTAPI_MSG_ERR,
                     "shim: more than %zu pins; raise max_signals in the host "
                     "configuration",
                     signal_capacity);
        return NULL;
    }
    signals[signal_count].name = name;
    signals[signal_count].type = type;
    signals[signal_count].dir = dir;
    signals[signal_count].cell = cell;
    signal_count++;
    return cell;
}

/*
 * The common half of every `hal_pin_new_*`: a cell, a name, and one or two
 * entries in the table.
 *
 * A HAL_IO pin gets two, with the same name and opposite directions, sharing
 * one cell. That is what makes `index-enable` work across a process boundary:
 * the core writes its request into the cell before the driver's read function
 * and reads the driver's answer out of it after the write function, so a
 * conversation HAL holds in one memory location becomes two signals and one
 * cycle of latency (see hm2_shim.h).
 */
static int new_pin(hal_pdir_t dir, void **ref, hm2_shim_type type, hm2_cell initial,
                   const char *fmt, va_list args) {
    if (!ref) {
        return -EINVAL;
    }
    const char *name = arena_name(fmt, args);
    if (!name) {
        return -ENOMEM;
    }
    hm2_cell *cell = hal_malloc((long)sizeof(*cell));
    if (!cell) {
        return -ENOMEM;
    }
    *cell = initial;

    if ((dir & HAL_IO) == HAL_IO) {
        if (!publish(name, type, HM2_SHIM_FROM_CORE, cell) ||
            !publish(name, type, HM2_SHIM_TO_CORE, cell)) {
            return -ENOMEM;
        }
    } else if (!publish(name, type, channel_dir(dir), cell)) {
        return -ENOMEM;
    }
    *ref = cell;
    return 0;
}

#define DEFINE_PIN(suffix, ctype, halref, member, shimtype)                              \
    int hal_pin_new_##suffix(int compid, hal_pdir_t dir, halref *ref, ctype def,         \
                             const char *fmt, ...) {                                     \
        (void)compid;                                                                    \
        hm2_cell initial = {0};                                                           \
        initial.member = def;                                                             \
        va_list args;                                                                     \
        va_start(args, fmt);                                                              \
        int r = new_pin(dir, (void **)ref, shimtype, initial, fmt, args);                 \
        va_end(args);                                                                     \
        return r;                                                                         \
    }

DEFINE_PIN(bool, rtapi_bool, hal_bool_t, u, HM2_SHIM_BOOL)
DEFINE_PIN(si32, rtapi_s32, hal_sint_t, s, HM2_SHIM_SINT)
DEFINE_PIN(ui32, rtapi_u32, hal_uint_t, u, HM2_SHIM_UINT)
DEFINE_PIN(sint, rtapi_sint, hal_sint_t, s, HM2_SHIM_SINT)
DEFINE_PIN(uint, rtapi_uint, hal_uint_t, u, HM2_SHIM_UINT)
DEFINE_PIN(real, rtapi_real, hal_real_t, r, HM2_SHIM_REAL)

/*
 * Parameters are configuration, not cyclic data.
 *
 * In HAL a parameter is set with `setp` at load time and occasionally poked at
 * afterwards. Here the host's configuration file sets it before
 * `rtapi_app_main` runs, and after that it is read-only -- which is what ADR
 * 0022 §4 asks for. They are deliberately *not* published on the channel: the
 * core's configuration is the machine description, and a second place to
 * configure the same machine is a second place for the two to disagree.
 *
 * The value is looked up by name at declaration, so `hm2_7i92.0.encoder.00.
 * scale = 8000` in the host's file reaches the cell before the driver reads
 * it for the first time.
 */
static int new_param(hal_pdir_t dir, void **ref, hm2_shim_type type, hm2_cell initial,
                     const char *fmt, va_list args) {
    (void)dir;
    if (!ref) {
        return -EINVAL;
    }
    const char *name = arena_name(fmt, args);
    if (!name) {
        return -ENOMEM;
    }
    hm2_cell *cell = hal_malloc((long)sizeof(*cell));
    if (!cell) {
        return -ENOMEM;
    }
    *cell = initial;

    double configured;
    if (hm2_shim_param_lookup(name, &configured)) {
        switch (type) {
        case HM2_SHIM_BOOL: cell->u = configured != 0.0; break;
        case HM2_SHIM_SINT: cell->s = (rtapi_sint)configured; break;
        case HM2_SHIM_UINT: cell->u = (rtapi_uint)configured; break;
        case HM2_SHIM_REAL: cell->r = configured; break;
        }
        hm2_shim_log(RTAPI_MSG_INFO, "shim: parameter %s = %g, from the host configuration",
                     name, configured);
    }
    hm2_shim_note_param(name, type, cell);
    *ref = cell;
    return 0;
}

#define DEFINE_PARAM(suffix, ctype, halref, member, shimtype)                            \
    int hal_param_new_##suffix(int compid, hal_pdir_t dir, halref *ref, ctype def,       \
                               const char *fmt, ...) {                                   \
        (void)compid;                                                                    \
        hm2_cell initial = {0};                                                           \
        initial.member = def;                                                             \
        va_list args;                                                                     \
        va_start(args, fmt);                                                              \
        int r = new_param(dir, (void **)ref, shimtype, initial, fmt, args);                \
        va_end(args);                                                                     \
        return r;                                                                         \
    }

DEFINE_PARAM(bool, rtapi_bool, hal_bool_t, u, HM2_SHIM_BOOL)
DEFINE_PARAM(si32, rtapi_s32, hal_sint_t, s, HM2_SHIM_SINT)
DEFINE_PARAM(ui32, rtapi_u32, hal_uint_t, u, HM2_SHIM_UINT)
DEFINE_PARAM(sint, rtapi_sint, hal_sint_t, s, HM2_SHIM_SINT)
DEFINE_PARAM(uint, rtapi_uint, hal_uint_t, u, HM2_SHIM_UINT)
DEFINE_PARAM(real, rtapi_real, hal_real_t, r, HM2_SHIM_REAL)

/*
 * A parameter with storage and no name.
 *
 * The driver uses this where a field exists in its instance structure but the
 * hardware has nothing behind it -- an `invert_output` on a pin that is not an
 * output, say. It still has to be readable, because the code that reads it
 * runs either way; it just never means anything. So: a cell, no entry.
 */
int hal_param_new_fake(int compid, hal_refs_u *refs) {
    (void)compid;
    if (!refs) {
        return -EINVAL;
    }
    hm2_cell *cell = hal_malloc((long)sizeof(*cell));
    if (!cell) {
        return -ENOMEM;
    }
    memset(cell, 0, sizeof(*cell));
    refs->b = (hal_bool_t)cell;
    return 0;
}

/*
 * HAL renames a parameter so a configuration can refer to it by another name.
 * Nothing on this side of the channel refers to a parameter by name at all --
 * the host's file does, at declaration, and that has already happened -- so
 * this is accepted and does nothing. Saying so beats failing a start-up over
 * an alias that would not have been used.
 */
int hal_param_alias(const char *pin_name, const char *alias) {
    hm2_shim_log(RTAPI_MSG_DBG, "shim: parameter alias %s -> %s is accepted and unused",
                 pin_name ? pin_name : "?", alias ? alias : "?");
    return 0;
}

/* ---------------------------------------------------------------------------
 * Exported functions
 * ------------------------------------------------------------------------- */

int hal_export_functf(void (*funct)(void *, long), void *arg, int uses_fp, int reentrant,
                      int comp_id, const char *fmt, ...) {
    (void)reentrant;
    (void)comp_id;
    if (!funct) {
        return -EINVAL;
    }
    if (funct_count >= funct_capacity) {
        hm2_shim_log(RTAPI_MSG_ERR, "shim: more than %zu exported functions", funct_capacity);
        return -ENOMEM;
    }
    va_list args;
    va_start(args, fmt);
    const char *name = arena_name(fmt, args);
    va_end(args);
    if (!name) {
        return -ENOMEM;
    }
    functs[funct_count].name = name;
    functs[funct_count].funct = funct;
    functs[funct_count].arg = arg;
    functs[funct_count].uses_fp = uses_fp;
    funct_count++;
    hm2_shim_log(RTAPI_MSG_INFO, "shim: exported function '%s'", name);
    return 0;
}

/* ---------------------------------------------------------------------------
 * What the host reads back
 * ------------------------------------------------------------------------- */

size_t hm2_shim_signal_count(void) { return signal_count; }

const hm2_shim_signal *hm2_shim_signal_at(size_t index) {
    return index < signal_count ? &signals[index] : NULL;
}

size_t hm2_shim_funct_count(void) { return funct_count; }

const hm2_shim_funct *hm2_shim_funct_at(size_t index) {
    return index < funct_count ? &functs[index] : NULL;
}

const hm2_shim_funct *hm2_shim_funct_ending(const char *suffix) {
    if (!suffix) {
        return NULL;
    }
    size_t want = strlen(suffix);
    for (size_t i = 0; i < funct_count; i++) {
        size_t have = strlen(functs[i].name);
        if (have >= want && strcmp(functs[i].name + have - want, suffix) == 0) {
            return &functs[i];
        }
    }
    return NULL;
}

double hm2_shim_cell_get(const hm2_shim_signal *signal) {
    if (!signal || !signal->cell) {
        return 0.0;
    }
    const hm2_cell *cell = signal->cell;
    switch (signal->type) {
    case HM2_SHIM_BOOL: return cell->u ? 1.0 : 0.0;
    case HM2_SHIM_SINT: return (double)cell->s;
    case HM2_SHIM_UINT: return (double)cell->u;
    case HM2_SHIM_REAL: return cell->r;
    }
    return 0.0;
}

void hm2_shim_cell_set(const hm2_shim_signal *signal, double value) {
    if (!signal || !signal->cell) {
        return;
    }
    hm2_cell *cell = signal->cell;
    switch (signal->type) {
    /* The whole width, as `hal_set_bool` writes it, so a bool read back as a
       uint is 0 or 1 rather than 1 with the old value above it. */
    case HM2_SHIM_BOOL: cell->u = value != 0.0; break;
    case HM2_SHIM_SINT: cell->s = (rtapi_sint)value; break;
    case HM2_SHIM_UINT: cell->u = value < 0.0 ? 0u : (rtapi_uint)value; break;
    case HM2_SHIM_REAL: cell->r = value; break;
    }
}
