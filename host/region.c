/*
 * The core's end of the channel, from this side (ADR 0011, ADR 0022 §3).
 *
 * The layout is `cnc_fieldbus.h`, vendored under host/include/. That file is
 * the protocol -- there is no library to link, and there deliberately is not:
 * the core's Rust implementation and this C one are two descriptions of one
 * set of bytes, and the whole reason the header is the source of truth is that
 * neither side has to trust the other's code.
 *
 * WHAT THE HANDSHAKE IS
 *   Two counters and two pairs of buffers, selected by the parity of the cycle
 *   number. This process writes input[n & 1] and stores cycle = n with release
 *   ordering; the core loads cycle with acquire, computes, writes
 *   output[n & 1] and stores response = n with release. At the send deadline
 *   this process loads response with acquire and reads output[response & 1] --
 *   this cycle's work if the core kept up, and the last cycle's if it did not.
 *
 *   **This process may publish cycle n only while response >= n - 1.** That
 *   guard is the whole basis for touching the buffers without a lock, and the
 *   argument for it is written out in full in the core's `channel.rs`. It is
 *   not restated here, because a second copy of an argument is a second thing
 *   to keep true; what is restated is the guard itself, below, where it is
 *   enforced.
 *
 *   From channel 1.1.0 the named-signal value blocks ride the same counters
 *   and the same parity. They are covered by that same argument, which is why
 *   they were an append rather than a design.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 the driver-hostmot2 contributors
 */
#include <errno.h>
#include <fcntl.h>
#include <linux/futex.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include "cnc_fieldbus.h"
#include "hm2_host.h"

/* Rounds up to a whole number of cache lines, as the core's `Layout` does. */
static size_t to_line(size_t bytes) {
    size_t line = CNC_FIELDBUS_CACHE_LINE;
    return ((bytes + line - 1) / line) * line;
}

/*
 * Lays the region out for `signals` signals, filling `out` with the offsets
 * the core will read back and check.
 *
 * The core validates every one of these rather than recomputing them: the
 * fixed part is a compile-time layout both sides reach independently, and this
 * part cannot be, so the process writes it down. Getting it wrong here is
 * refused at the core's attach with the reason named, which is the outcome to
 * want -- the alternative is two processes disagreeing about where a value is.
 */
size_t hm2_region_layout(size_t inputs, size_t outputs, cnc_fieldbus_signal_block *out) {
    size_t fixed = sizeof(cnc_fieldbus_shm);
    size_t count = inputs + outputs;
    memset(out, 0, sizeof(*out));
    if (count == 0) {
        /* A process with only axes writes a region of exactly the 1.0.0 size,
           which is what lets a core built before 1.1.0 drive it unchanged. */
        out->region_size = fixed;
        return fixed;
    }

    size_t table_offset = to_line(fixed + sizeof(cnc_fieldbus_signal_block));
    size_t table_size = count * sizeof(cnc_fieldbus_signal_desc);
    size_t input_offset = to_line(table_offset + table_size);
    size_t input_stride = to_line(inputs * sizeof(double));
    size_t output_offset = input_offset + 2 * input_stride;
    size_t output_stride = to_line(outputs * sizeof(double));
    size_t region_size = output_offset + 2 * output_stride;

    out->count = (uint32_t)count;
    out->input_count = (uint32_t)inputs;
    out->output_count = (uint32_t)outputs;
    out->desc_size = (uint32_t)sizeof(cnc_fieldbus_signal_desc);
    out->table_offset = table_offset;
    out->input_offset = input_offset;
    out->output_offset = output_offset;
    out->input_stride = input_stride;
    out->output_stride = output_stride;
    out->region_size = region_size;
    return region_size;
}

/*
 * Creates the shared object and maps it.
 *
 * Unlinked first, always. A core still attached to a previous run's object
 * keeps its own mapping alive and never sees this one, which is exactly what
 * ADR 0011 §6 wants: a restarted driver process must not silently resume a
 * live core. The core finds out because the cycles stop and because the
 * generation changed, both of which it already treats as faults.
 */
int hm2_region_create(hm2_region *region, const char *name, size_t inputs, size_t outputs) {
    memset(region, 0, sizeof(*region));

    cnc_fieldbus_signal_block block;
    size_t bytes = hm2_region_layout(inputs, outputs, &block);

    shm_unlink(name);
    int fd = shm_open(name, O_CREAT | O_EXCL | O_RDWR, S_IRUSR | S_IWUSR);
    if (fd < 0) {
        hm2_log(HM2_LOG_ERROR, "could not create the shared region %s: %s", name,
                strerror(errno));
        return -1;
    }
    if (ftruncate(fd, (off_t)bytes) != 0) {
        hm2_log(HM2_LOG_ERROR, "could not size %s to %zu bytes: %s", name, bytes,
                strerror(errno));
        close(fd);
        shm_unlink(name);
        return -1;
    }
    void *base = mmap(NULL, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (base == MAP_FAILED) {
        hm2_log(HM2_LOG_ERROR, "could not map %s: %s", name, strerror(errno));
        shm_unlink(name);
        return -1;
    }
    /* The other side of the same judgement the shim makes about its arena: a
       page fault in a cyclic path is a late cycle, and a warning is what a
       developer's machine gets instead of a refusal. */
    if (mlock(base, bytes) != 0) {
        hm2_log(HM2_LOG_WARN,
                "could not lock the shared region into memory: %s. Grant CAP_IPC_LOCK and "
                "raise RLIMIT_MEMLOCK on a real-time host",
                strerror(errno));
    }

    region->base = base;
    region->bytes = bytes;
    region->shm = base;
    region->block = block;
    snprintf(region->name, sizeof(region->name), "%s", name);
    region->owner = 1;
    return 0;
}

/* Fills the read-only header. Once, before either side starts cycling. */
void hm2_region_describe(hm2_region *region, const hm2_region_config *config) {
    cnc_fieldbus_shm *shm = region->shm;
    memset(&shm->config, 0, sizeof(shm->config));
    shm->config.version_major = CNC_FIELDBUS_ABI_VERSION_MAJOR;
    shm->config.version_minor = CNC_FIELDBUS_ABI_VERSION_MINOR;
    shm->config.version_patch = CNC_FIELDBUS_ABI_VERSION_PATCH;
    shm->config.axis_count = config->axis_count;
    /*
     * A different value on every start, and the only thing that lets the core
     * tell a restarted process from the one it attached to. The monotonic
     * clock rather than the wall clock: a machine whose clock is set backwards
     * between two runs would otherwise repeat a generation.
     */
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    shm->config.generation = (uint64_t)now.tv_sec * 1000000000ull + (uint64_t)now.tv_nsec;
    /* The fixed part only. The tail says its own size; see cnc_fieldbus.h. */
    shm->config.layout_size = sizeof(cnc_fieldbus_shm);
    shm->config.cycle_time_ns = config->cycle_ns;
    shm->config.core_watchdog_cycles = config->core_watchdog_cycles;
    shm->config.response_watchdog_cycles = config->response_watchdog_cycles;
    shm->config.sync_manager_watchdog_us = config->drive_watchdog_us;
    shm->config.spin_iterations = config->spin_iterations;
    shm->config.worst_case_cycle_ns = config->worst_case_cycle_ns;

    if (region->block.count > 0) {
        cnc_fieldbus_signal_block *block =
            (void *)((char *)region->base + sizeof(cnc_fieldbus_shm));
        *block = region->block;
    }
    atomic_store_explicit((_Atomic uint32_t *)&shm->state, CNC_FIELDBUS_STATE_STARTING,
                          memory_order_release);
}

/* One descriptor. Written before cycling starts and read-only after. */
void hm2_region_declare(hm2_region *region, size_t index, const char *name, uint32_t direction,
                        uint32_t type, uint32_t role, uint32_t unit, uint32_t value_index) {
    cnc_fieldbus_signal_desc *table =
        (void *)((char *)region->base + region->block.table_offset);
    cnc_fieldbus_signal_desc *desc = &table[index];
    memset(desc, 0, sizeof(*desc));
    snprintf(desc->name, sizeof(desc->name), "%s", name);
    desc->direction = direction;
    desc->type = type;
    desc->role = role;
    desc->unit = unit;
    desc->value_index = value_index;
    desc->update_divisor = 1;
}

void hm2_region_set_state(hm2_region *region, uint32_t state) {
    cnc_fieldbus_shm *shm = region->shm;
    atomic_store_explicit((_Atomic uint32_t *)&shm->state, state, memory_order_release);
}

/* This parity's half of a value block. */
static double *value_half(hm2_region *region, uint64_t offset, uint64_t stride, uint64_t cycle) {
    if (region->block.count == 0) {
        return NULL;
    }
    return (double *)((char *)region->base + offset + (cycle & 1) * stride);
}

double *hm2_region_inputs(hm2_region *region, uint64_t cycle) {
    return value_half(region, region->block.input_offset, region->block.input_stride, cycle);
}

const double *hm2_region_outputs(hm2_region *region, uint64_t cycle) {
    return value_half(region, region->block.output_offset, region->block.output_stride, cycle);
}

/*
 * Begins a cycle: the guard, and where to write.
 *
 * Returns the cycle number, or 0 if the core has not finished the previous one.
 * Nothing is written in that case and the caller repeats its last complete
 * output, which ADR 0011 §5 says to do -- right immediately, wrong
 * indefinitely, which is what the response watchdog is for.
 */
uint64_t hm2_region_begin(hm2_region *region) {
    cnc_fieldbus_shm *shm = region->shm;
    uint64_t next =
        atomic_load_explicit((_Atomic uint64_t *)&shm->cycle, memory_order_relaxed) + 1;

    /*
     * The guard. From cycle 2 onwards, publishing while the core is behind
     * would let this process write a buffer the core is reading, and let the
     * core write one this process is about to read.
     */
    if (next >= 2) {
        uint64_t response =
            atomic_load_explicit((_Atomic uint64_t *)&shm->response, memory_order_acquire);
        if (response + 1 < next) {
            region->overruns++;
            return 0;
        }
    }
    return next;
}

/*
 * Ends a cycle: publishes everything written since `begin`, and wakes the core.
 *
 * The release store is what orders every write above it against the core's
 * acquire load -- the axis buffer, and from 1.1.0 the input value block
 * outside it, which is the whole of why that append needed no new mechanism.
 */
void hm2_region_publish(hm2_region *region, uint64_t cycle) {
    cnc_fieldbus_shm *shm = region->shm;
    atomic_store_explicit((_Atomic uint64_t *)&shm->cycle, cycle, memory_order_release);

    /*
     * The wake-up is a hint and never the truth: a spurious one finds `cycle`
     * unchanged and goes back to sleep, and a missed one is caught by the
     * next. That is what lets the notification be replaced without revisiting
     * any of the ordering above.
     *
     * Not FUTEX_PRIVATE_FLAG: the two sides are separate processes sharing a
     * mapping, so this is a shared futex. The private variant is faster and
     * would simply not wake anybody.
     */
    atomic_fetch_add_explicit((_Atomic uint32_t *)&shm->futex_word, 1, memory_order_release);
    syscall(SYS_futex, &shm->futex_word, FUTEX_WAKE, INT32_MAX, NULL, NULL, 0);
}

/*
 * Reads the newest complete outputs the core has produced.
 *
 * Call at the send deadline, not straight after publishing: a real master puts
 * its outputs on the wire at the end of the period, which is what gives the
 * core the cycle to answer in.
 *
 * `*answered` is the cycle those outputs came from, which is not always the
 * current one -- and the caller must index the value block with *that* number
 * rather than the current cycle, or it pairs one cycle's outputs with
 * another's signals.
 */
int hm2_region_collect(hm2_region *region, uint64_t *answered) {
    cnc_fieldbus_shm *shm = region->shm;
    uint64_t response =
        atomic_load_explicit((_Atomic uint64_t *)&shm->response, memory_order_acquire);
    *answered = response;
    if (response == 0) {
        return HM2_COLLECT_NEVER;
    }
    uint64_t cycle = atomic_load_explicit((_Atomic uint64_t *)&shm->cycle, memory_order_relaxed);
    return response == cycle ? HM2_COLLECT_FRESH : HM2_COLLECT_STALE;
}

uint32_t hm2_region_state(hm2_region *region) {
    cnc_fieldbus_shm *shm = region->shm;
    return atomic_load_explicit((_Atomic uint32_t *)&shm->state, memory_order_acquire);
}

void hm2_region_destroy(hm2_region *region) {
    if (!region->base) {
        return;
    }
    /* Say so before going away, so the core reports a shutdown rather than a
       channel that simply stopped. It may not be looking, which is why it also
       has a watchdog. */
    hm2_region_set_state(region, CNC_FIELDBUS_STATE_SHUTDOWN);
    munlock(region->base, region->bytes);
    munmap(region->base, region->bytes);
    if (region->owner) {
        shm_unlink(region->name);
    }
    region->base = NULL;
}
