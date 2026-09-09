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

#include "cnc_outboard.h"

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
} hm2_region_config;

size_t hm2_region_layout(size_t inputs, size_t outputs, cnc_outboard_signal_block *out);
int hm2_region_create(hm2_region *region, const char *name, size_t inputs, size_t outputs);
void hm2_region_describe(hm2_region *region, const hm2_region_config *config);
void hm2_region_declare(hm2_region *region, size_t index, const char *name, uint32_t direction,
                        uint32_t type, uint32_t role, uint32_t unit, uint32_t value_index);
void hm2_region_set_state(hm2_region *region, uint32_t state);
double *hm2_region_inputs(hm2_region *region, uint64_t cycle);
const double *hm2_region_outputs(hm2_region *region, uint64_t cycle);
uint64_t hm2_region_begin(hm2_region *region);
void hm2_region_publish(hm2_region *region, uint64_t cycle);
int hm2_region_collect(hm2_region *region, uint64_t *answered);
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
} hm2_config;

int hm2_config_load(hm2_config *config, const char *path);
void hm2_config_free(hm2_config *config);

#endif /* HM2_HOST_H */
