/*
 * `hm2-host`: the driver process (ADR 0022 §3).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 the driver-hostmot2 contributors
 */
#ifndef HM2_HOST_H
#define HM2_HOST_H

#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "cnc_outboard.h"
#include "hm2_shim.h"

/* ---------------------------------------------------------------------------
 * Diagnostics
 * ------------------------------------------------------------------------- */

enum {
    HM2_LOG_ERROR = 1,
    HM2_LOG_WARN = 2,
    HM2_LOG_INFO = 3,
    HM2_LOG_DEBUG = 4
};

void hm2_log(int level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void hm2_log_set_level(int level);

/* ---------------------------------------------------------------------------
 * Smart Serial faults (sserial_watch.c)
 * ------------------------------------------------------------------------- */

/* The port's number in a pin named `....sserial.port-N.fault-count`, or -1. */
int sserial_port_of(const char *name);
/* Watches `signal` if it is a port's fault count. Returns whether it is. */
int sserial_watch_add(const hm2_shim_signal *signal);
size_t sserial_watched(void);
/*
 * After a write: how many watched ports' fault counts rose since the last
 * call, each logged with the cycle, how long before this period's read the
 * previous write had ended, whether that write waited for a core that did not
 * answer in time, and `so_far` the faults before these.
 */
uint64_t sserial_faults_seen(uint64_t cycle, double gap_s, int gap_late, uint64_t so_far);

/* ---------------------------------------------------------------------------
 * The region
 * ------------------------------------------------------------------------- */

enum { HM2_COLLECT_FRESH = 0, HM2_COLLECT_STALE = 1, HM2_COLLECT_NEVER = 2 };

typedef struct {
    void *base;
    size_t bytes;
    cnc_outboard_shm *shm;
    cnc_outboard_signal_block block;
    char name[256];
    int owner;
    /* Cycles this process could not publish because the core was behind. */
    uint64_t overruns;
} hm2_region;

typedef struct {
    uint32_t axis_count;
    uint32_t cycle_ns;
    uint32_t core_watchdog_cycles;
    uint32_t response_watchdog_cycles;
    uint32_t drive_watchdog_us;
    uint32_t spin_iterations;
    uint64_t worst_case_cycle_ns;
    /* CNC_OUTBOARD_CONFIG_* (channel 2.3.0). */
    uint32_t flags;
} hm2_region_config;

size_t hm2_region_layout(size_t inputs, size_t outputs, cnc_outboard_signal_block *out);
int hm2_region_create(hm2_region *region, const char *name, size_t inputs, size_t outputs);
void hm2_region_describe(hm2_region *region, const hm2_region_config *config);
void hm2_region_declare(hm2_region *region, size_t index, const char *name, uint32_t direction,
                        uint32_t type, uint32_t role, uint32_t unit, uint32_t value_index,
                        uint32_t flags);
void hm2_region_set_state(hm2_region *region, uint32_t state);
double *hm2_region_inputs(hm2_region *region, uint64_t cycle);
const double *hm2_region_outputs(hm2_region *region, uint64_t cycle);
void hm2_region_seed_output(hm2_region *region, size_t index, double value);
uint64_t hm2_region_begin(hm2_region *region);
/* Whether the board answers, for cycle `cycle` (channel 2.2.0, ADR 0045 §3). */
void hm2_region_set_bus(hm2_region *region, uint64_t cycle, uint32_t state, uint32_t fault);
void hm2_region_publish(hm2_region *region, uint64_t cycle);
int hm2_region_collect(hm2_region *region, uint64_t *answered);
/* The newest cycle the core has answered; 0 before its first answer. */
uint64_t hm2_region_answered(hm2_region *region);
/*
 * Waits for the core to answer `cycle`, until `deadline` on CLOCK_MONOTONIC
 * (channel 2.3.0, ADR 0046 §1). Returns 1 if it did, 0 if the deadline came.
 */
int hm2_region_wait_answer(hm2_region *region, uint64_t cycle, const struct timespec *deadline);
uint32_t hm2_region_state(hm2_region *region);
void hm2_region_destroy(hm2_region *region);

/* ---------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

/*
 * What the host was told to be.
 *
 * A flat `key = value` file, because the alternative was a dependency on a
 * parser and this has fifteen keys. Three of them are namespaced:
 *
 *   module.<name>[n] = <value>   a driver module parameter, exactly what a
 *                                LinuxCNC `loadrt` line sets: board_ip,
 *                                config, debug.
 *   param.<hal name>  = <number> a HAL parameter, exactly what a `setp` line
 *                                sets: an encoder scale, a stepgen's timing.
 *   pin.<hal name>    = <value>  a pin's value, fixed: what a `setp` line on a
 *                                pin nothing drives sets -- a stepgen's
 *                                control-type, a DPLL timer, an encoder's
 *                                quad-error-enable. `true`, `false` or a
 *                                number. The pin is then published for
 *                                reading only (ADR 0045 §2).
 */
typedef struct {
    char region[256];
    /*
     * Which transport module to load beside the generic driver: `hm2_eth` for
     * an Ethernet board, `hm2_test` for the fake one upstream ships. The
     * transports are separate LinuxCNC modules and always have been, so this
     * is a name rather than a mode.
     */
    char transport[64];
    uint32_t axis_count;
    uint32_t cycle_us;
    uint32_t core_watchdog_cycles;
    uint32_t response_watchdog_cycles;
    uint32_t drive_watchdog_us;
    uint32_t spin_iterations;
    uint64_t worst_case_cycle_ns;
    /* How far into the period the outputs go on the wire, 0..1. */
    double send_deadline;
    /*
     * Whether the core's answer is written in the period it was published in
     * (ADR 0046): read, publish, wait for the answer until the send deadline,
     * write. Off, the period writes the answer to the previous publish.
     */
    int same_cycle;
    size_t arena_bytes;
    size_t max_signals;
    size_t max_functs;
    int log_level;
    int rt_priority;
    int cpu_affinity;

    /* module.<name>[index] = value */
    struct hm2_module_param {
        char name[128];
        size_t index;
        char value[256];
    } *module_params;
    size_t module_param_count;

    /* param.<hal name> = value */
    const char **param_names;
    double *param_values;
    size_t param_count;

    /* pin.<hal name> = value, and the line each came from */
    const char **pin_names;
    double *pin_values;
    int *pin_lines;
    size_t pin_count;
} hm2_config;

int hm2_config_load(hm2_config *config, const char *path);
void hm2_config_free(hm2_config *config);

#endif /* HM2_HOST_H */
