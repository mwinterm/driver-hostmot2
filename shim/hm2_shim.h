/*
 * The HAL/RTAPI shim: what the host sees of the driver it hosts.
 *
 * The driver in third_party/linuxcnc/ is LinuxCNC's, unmodified, and it talks
 * to LinuxCNC's HAL. This shim is the other end of that conversation: a
 * reimplementation of the small part of HAL and RTAPI the driver actually
 * uses, backed by the outboard channel's signal table instead of by HAL's
 * shared memory (ADR 0022 §4).
 *
 * The surface is 25 functions. It is that small because the driver was
 * already written against HAL's accessor API -- a pin is an opaque handle and
 * every read goes through `hal_get_bool` and friends, which are static inline
 * in hal.h and operate on whatever the handle points at. So the shim owns the
 * storage completely and the driver never dereferences a pointer of ours.
 *
 * WHAT THE SHIM IS NOT
 *   It is not HAL. There are no signals, no links, no `halcmd`, no threads and
 *   no shared memory between processes. A pin here is a cell in an arena with
 *   a name, and the host copies it to and from the channel each cycle. What
 *   HAL calls linking, the machine description does on the other side of the
 *   channel (ADR 0022 §5).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 the driver-hostmot2 contributors
 */
#ifndef HM2_SHIM_H
#define HM2_SHIM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Signals
 * ------------------------------------------------------------------------- */

/*
 * What a cell holds. These are HAL's four accessor types; the union the
 * accessors cast a handle to is eight bytes whichever it is, so the type says
 * how to read the cell rather than how big it is.
 */
typedef enum {
    HM2_SHIM_BOOL = 0,
    HM2_SHIM_SINT = 1,
    HM2_SHIM_UINT = 2,
    HM2_SHIM_REAL = 3
} hm2_shim_type;

/*
 * Which way a signal crosses the channel.
 *
 * This is the one thing in the shim that is easy to get backwards, so it is
 * worth stating in both vocabularies at once. HAL's directions are named from
 * the *component's* point of view:
 *
 *   HAL_IN  -- the driver reads it, somebody else writes it. A command.
 *              Across the channel that is the core writing: an OUTPUT.
 *   HAL_OUT -- the driver writes it. A feedback, a status, an input pin's
 *              state. Across the channel that is the core reading: an INPUT.
 *
 * So the words invert, and they invert because the two ends of the sentence
 * are different. A pin the driver calls an input is a value the core produces.
 */
typedef enum {
    /* Driver -> core. A HAL_OUT pin. */
    HM2_SHIM_TO_CORE = 0,
    /* Core -> driver. A HAL_IN pin. */
    HM2_SHIM_FROM_CORE = 1
} hm2_shim_dir;

/*
 * One published pin.
 *
 * `cell` is the eight bytes the driver's accessors read and write, and the
 * host copies between it and the channel's value block each cycle. It is in
 * the arena, so it does not move and is never freed while the driver runs.
 */
typedef struct {
    /* The HAL name: "hm2_7i92.0.encoder.00.position". Arena-owned. */
    const char *name;
    hm2_shim_type type;
    hm2_shim_dir dir;
    void *cell;
} hm2_shim_signal;

/*
 * A HAL_IO pin -- one the driver both reads and writes -- is published as
 * *two* signals with the same name, one in each direction. `index-enable` is
 * the case that matters: the caller sets it to ask for an index reset and the
 * driver clears it when the index arrives, which is a conversation rather than
 * a value, and a channel signal goes one way.
 *
 * The channel allows one name in each direction for exactly this. What it
 * costs is a cycle of latency on the round trip, because the driver's answer
 * to this cycle's request is published in the next one. Say so wherever an
 * IO pin's timing matters.
 */

/* ---------------------------------------------------------------------------
 * Exported functions
 * ------------------------------------------------------------------------- */

/*
 * A function the driver exported with `hal_export_functf`. The host calls the
 * ones it needs each cycle, by name: hostmot2 exports "<board>.read",
 * "<board>.write", "<board>.read_gpio", "<board>.write_gpio" and
 * "<board>.pet_watchdog".
 */
typedef struct {
    const char *name;
    void (*funct)(void *arg, long period);
    void *arg;
    int uses_fp;
} hm2_shim_funct;

/* ---------------------------------------------------------------------------
 * The shim's own interface to the host
 * ------------------------------------------------------------------------- */

/*
 * Prepares the shim. Call once, before `rtapi_app_main`.
 *
 * `arena_bytes` sizes the pool `hal_malloc` hands out of. It is allocated and
 * touched here, once, and never freed or grown -- §5.3 forbids allocating in a
 * cyclic path, and the driver allocates all of its per-instance state during
 * `rtapi_app_main`. A driver that asks for more than this fails to start,
 * loudly, rather than allocating in the wrong place later.
 *
 * Returns 0, or -1 if the arena could not be allocated or locked.
 */
int hm2_shim_init(size_t arena_bytes, size_t max_signals, size_t max_functs);

/* Releases everything the shim owns. After `rtapi_app_exit`. */
void hm2_shim_fini(void);

/* The published pins, in the order they were declared. */
size_t hm2_shim_signal_count(void);
const hm2_shim_signal *hm2_shim_signal_at(size_t index);

/* The exported functions. */
size_t hm2_shim_funct_count(void);
const hm2_shim_funct *hm2_shim_funct_at(size_t index);
/* The first function whose name ends in `suffix`, or NULL. */
const hm2_shim_funct *hm2_shim_funct_ending(const char *suffix);

/* Reads a cell as a double, whatever its type. For the host's copy out. */
double hm2_shim_cell_get(const hm2_shim_signal *signal);
/* Writes a cell from a double, converting to its type. For the copy in. */
void hm2_shim_cell_set(const hm2_shim_signal *signal, double value);

/*
 * Sets a module parameter by name, as `loadrt hm2_eth board_ip=10.10.10.10`
 * does -- the host's configuration file is where these come from now.
 *
 * The driver's parameters are `static`, so there is no symbol to assign to.
 * What there is, because `RTAPI_MP_*` puts it there, is a non-static
 * `rtapi_info_address_<name>` holding the parameter's address and a
 * `rtapi_info_type_<name>` holding its type letter. That is the mechanism
 * LinuxCNC's own loader uses, which is why the sources need no edit.
 *
 * `module` is the `dlopen` handle to look in. The driver modules are opened
 * `RTLD_LOCAL` -- two LinuxCNC modules, each with its own `rtapi_app_main` --
 * so there is no flat namespace to search, and the caller tries each module
 * it loaded.
 *
 * `index` is the element for an array parameter, and 0 for a scalar.
 * Returns 0 if it was set, 1 if this module does not have it, -1 on an error
 * that was logged.
 */
int hm2_shim_set_module_param(void *module, const char *name, size_t index, const char *value);

/*
 * Where the shim's diagnostics go. The host sets this so that a driver message
 * and a host message end up in the same place, in order.
 *
 * `level` is RTAPI's: 0 none, 1 error, 2 warning, 3 info, 4 debug, 5 all.
 */
void hm2_shim_set_log(void (*sink)(int level, const char *line));

/* The message level the driver's `rtapi_print_msg` is filtered against. */
void hm2_shim_set_msg_level(int level);

/* Flushes a driver message left without a trailing newline. At shutdown. */
void hm2_shim_flush_log(void);

/* ---------------------------------------------------------------------------
 * HAL parameters
 * ------------------------------------------------------------------------- */

/*
 * Hands the shim the parameter values the host's configuration file set, by
 * HAL name. Called before `rtapi_app_main`, because a parameter is read at
 * declaration and there is deliberately no moment at which the driver runs on
 * a default nobody chose.
 *
 * The arrays are borrowed, not copied: the host owns them and must keep them
 * alive until `hm2_shim_fini`.
 */
void hm2_shim_set_params(const char **names, const double *values, size_t count);

/* Whether the driver called `hal_ready`, and what it called itself. */
int hm2_shim_component_ready(void);
const char *hm2_shim_component_name(void);

/* How much of the arena the driver took, for the start-up report. */
size_t hm2_shim_arena_used(void);
size_t hm2_shim_arena_size(void);

/*
 * Parameters the configuration set that no declaration ever claimed, written
 * into `into` up to `capacity`. Returns how many there were.
 *
 * Almost always a typo, and worth refusing a start-up over: a misspelled
 * encoder scale leaves the encoder on its default, the machine moves by the
 * wrong distance, and nothing anywhere says why.
 */
size_t hm2_shim_unclaimed_params(const char **into, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* HM2_SHIM_H */
