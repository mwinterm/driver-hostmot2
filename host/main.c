/*
 * `hm2-host`: LinuxCNC's HostMot2 driver as a process of its own (ADR 0022 §3).
 *
 * The GPL code never enters `cnc-core`. It runs here, and reaches the core
 * through the mechanism ADR 0011 and ADR 0012 already built for the IgH
 * EtherCAT master and nothing bespoke: a mapped region whose layout is
 * `cnc_outboard.h`, an ordinary driver plugin at the core's end, and a stub
 * for CI without hardware.
 *
 * WHAT ONE CYCLE IS
 *   This process owns the cycle (ADR 0022 §7) and paces itself on its own
 *   monotonic clock, because the FPGA has no clock to discipline it to. At
 *   each tick:
 *
 *     1. take the core's answered outputs -- or the last complete set, if the
 *        core has not finished -- into the driver's pins;
 *     2. call the driver's write function: setpoints out, watchdog petted;
 *     3. call its read function: one LBP16 round trip over UDP;
 *     4. publish the driver's pins as the next cycle, and wake the core.
 *
 *   The round trip is what makes this a 1 ms-class interface, and the process
 *   declares that in the worst-case cycle time it puts in the channel header,
 *   where the core's budget check reads it (§10.2).
 *
 * WHAT IT DOES NOT DO
 *   It has no notion of axes, kinematics, the NC/PLC contract or the NC. It
 *   publishes every HAL pin under its HAL name and nothing else. What a signal
 *   *means* is said in the machine description, on the other side of the
 *   channel (ADR 0022 §5).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 the driver-hostmot2 contributors
 */
#include <dlfcn.h>
#include <errno.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "hm2_host.h"
#include "hm2_shim.h"

/* ---------------------------------------------------------------------------
 * Diagnostics
 * ------------------------------------------------------------------------- */

static int log_level = HM2_LOG_INFO;
static const char *const level_name[] = {"", "ERROR", "WARN", "INFO", "DEBUG", "TRACE"};

void hm2_log_set_level(int level) { log_level = level; }

void hm2_log(int level, const char *fmt, ...) {
    if (level > log_level) {
        return;
    }
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    struct tm parts;
    gmtime_r(&now.tv_sec, &parts);
    char when[32];
    strftime(when, sizeof(when), "%Y-%m-%dT%H:%M:%S", &parts);

    /* stderr, line-buffered, so this interleaves correctly with whatever is
       collecting the core's own log beside it. */
    fprintf(stderr, "%s.%06ldZ %-5s hm2-host: ", when, now.tv_nsec / 1000,
            level_name[level < 6 ? level : 5]);
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fputc('\n', stderr);
}

/* The shim's messages, and the driver's, through the same place. */
static void shim_log_sink(int rtapi_level, const char *line) {
    /* RTAPI's levels and this host's happen to agree: 1 error, 2 warning,
       3 info, 4 debug. Mapped explicitly anyway, because "happen to agree" is
       not a thing to rely on across two projects' headers. */
    int level = rtapi_level <= 1   ? HM2_LOG_ERROR
                : rtapi_level == 2 ? HM2_LOG_WARN
                : rtapi_level == 3 ? HM2_LOG_INFO
                                   : HM2_LOG_DEBUG;
    hm2_log(level, "%s", line);
}

/* ---------------------------------------------------------------------------
 * Shutdown
 * ------------------------------------------------------------------------- */

static volatile sig_atomic_t stopping;

static void on_signal(int signum) {
    (void)signum;
    stopping = 1;
}

/* ---------------------------------------------------------------------------
 * Real-time
 * ------------------------------------------------------------------------- */

/*
 * Asks for the priority and the CPU this process needs to be a timebase.
 *
 * A warning rather than a refusal if it is not granted, for the reason the
 * core makes the same judgement about itself: the real-time guarantee comes
 * from bringing up an RT host properly, not from refusing to run on a laptop.
 * What that owes in exchange is a line nobody can miss, which is this one.
 */
static void ask_for_realtime(const hm2_config *config) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        hm2_log(HM2_LOG_WARN, "could not lock memory: %s", strerror(errno));
    }
    struct sched_param param = {.sched_priority = config->rt_priority};
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        hm2_log(HM2_LOG_WARN,
                "no SCHED_FIFO at priority %d (%s): this process is the timebase, and being "
                "preempted by the thread it releases makes the cycle late by definition. "
                "Cycles will be late and the core will see it",
                config->rt_priority, strerror(errno));
    } else {
        hm2_log(HM2_LOG_INFO, "SCHED_FIFO granted at priority %d", config->rt_priority);
    }
    if (config->cpu_affinity >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET((size_t)config->cpu_affinity, &set);
        if (sched_setaffinity(0, sizeof(set), &set) != 0) {
            hm2_log(HM2_LOG_WARN, "could not pin to CPU %d: %s", config->cpu_affinity,
                    strerror(errno));
        } else {
            hm2_log(HM2_LOG_INFO, "pinned to CPU %d", config->cpu_affinity);
        }
    }
}

/* ---------------------------------------------------------------------------
 * The driver modules
 * ------------------------------------------------------------------------- */

/*
 * Two shared objects, in the order `loadrt` would load them: the generic
 * driver first, then the transport that registers itself with it.
 *
 * `RTLD_LOCAL` per handle rather than one flat namespace, because both export
 * `rtapi_app_main` -- they are two LinuxCNC modules, and merging them would
 * mean editing sources this repository keeps byte-identical to upstream.
 * `RTLD_GLOBAL` on the generic one, because the transport links against it.
 */
typedef struct {
    void *handle;
    int (*app_main)(void);
    void (*app_exit)(void);
    const char *name;
} hm2_module;

static int load_module(hm2_module *module, const char *path, int global) {
    module->name = path;
    module->handle = dlopen(path, RTLD_NOW | (global ? RTLD_GLOBAL : RTLD_LOCAL));
    if (!module->handle) {
        hm2_log(HM2_LOG_ERROR, "could not load %s: %s", path, dlerror());
        return -1;
    }
    /*
     * Through a memcpy, because ISO C does not allow assigning a `void *` to
     * a function pointer and POSIX requires `dlsym` to work anyway. This is
     * the spelling that satisfies both, and it is what the POSIX rationale
     * itself suggests.
     */
    void *symbol = dlsym(module->handle, "rtapi_app_main");
    memcpy(&module->app_main, &symbol, sizeof(symbol));
    symbol = dlsym(module->handle, "rtapi_app_exit");
    memcpy(&module->app_exit, &symbol, sizeof(symbol));
    if (!module->app_main) {
        hm2_log(HM2_LOG_ERROR, "%s exports no rtapi_app_main, so it is not a driver module",
                path);
        return -1;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * The signal table
 * ------------------------------------------------------------------------- */

/*
 * Turns the shim's pins into the channel's signals.
 *
 * Every pin, under its HAL name, with the value index dense within its
 * direction -- which is the channel's rule and the one thing a reader of the
 * table must not assume from its order (ADR 0022 §3). What a name means is
 * decided on the other side; this is a table of names and nothing more.
 */
typedef struct {
    const hm2_shim_signal *signal;
    size_t value_index;
} bound_signal;

static bound_signal *bound_inputs;
static size_t bound_input_count;
static bound_signal *bound_outputs;
static size_t bound_output_count;

static int bind_signals(void) {
    size_t count = hm2_shim_signal_count();
    bound_inputs = calloc(count ? count : 1, sizeof(*bound_inputs));
    bound_outputs = calloc(count ? count : 1, sizeof(*bound_outputs));
    if (!bound_inputs || !bound_outputs) {
        return -1;
    }
    for (size_t i = 0; i < count; i++) {
        const hm2_shim_signal *signal = hm2_shim_signal_at(i);
        if (signal->dir == HM2_SHIM_TO_CORE) {
            bound_inputs[bound_input_count].signal = signal;
            bound_inputs[bound_input_count].value_index = bound_input_count;
            bound_input_count++;
        } else {
            bound_outputs[bound_output_count].signal = signal;
            bound_outputs[bound_output_count].value_index = bound_output_count;
            bound_output_count++;
        }
    }
    return 0;
}

/* The channel's type for one of the shim's. Both sides' enumerators agree. */
static uint32_t channel_type(hm2_shim_type type) {
    switch (type) {
    case HM2_SHIM_BOOL: return CNC_OUTBOARD_SIGNAL_BOOL;
    case HM2_SHIM_SINT:
    case HM2_SHIM_UINT: return CNC_OUTBOARD_SIGNAL_I32;
    case HM2_SHIM_REAL: return CNC_OUTBOARD_SIGNAL_F64;
    }
    return CNC_OUTBOARD_SIGNAL_F64;
}

/*
 * A role for a pin, guessed from its name.
 *
 * The channel carries a role so the core can refuse to route a fast output to
 * a signal that is not one (ADR 0017 §8), and everything else it does with a
 * role is diagnostic. HostMot2 has no notion of one, so this is the shape of
 * the HAL name and nothing more -- deliberately shallow, and deliberately
 * defaulting to DIGITAL or ANALOG by type rather than claiming something
 * specific. The machine description says what a pin is *for*; this only says
 * what it looks like.
 */
static uint32_t channel_role(const char *name, hm2_shim_type type) {
    if (strstr(name, ".position") || strstr(name, ".counts")) {
        return CNC_OUTBOARD_ROLE_POSITION;
    }
    if (strstr(name, ".velocity")) {
        return CNC_OUTBOARD_ROLE_VELOCITY;
    }
    if (strstr(name, ".index-enable") || strstr(name, ".latch")) {
        return CNC_OUTBOARD_ROLE_PROBE_LATCH;
    }
    return type == HM2_SHIM_BOOL ? CNC_OUTBOARD_ROLE_DIGITAL : CNC_OUTBOARD_ROLE_ANALOG;
}

/* The channel's own spellings, for the table below. */
static const char *type_name(uint32_t type) {
    switch (type) {
    case CNC_OUTBOARD_SIGNAL_BOOL: return "bool";
    case CNC_OUTBOARD_SIGNAL_I32: return "i32";
    case CNC_OUTBOARD_SIGNAL_F64: return "f64";
    }
    return "?";
}

static const char *role_name(uint32_t role) {
    switch (role) {
    case CNC_OUTBOARD_ROLE_DIGITAL: return "digital";
    case CNC_OUTBOARD_ROLE_ANALOG: return "analog";
    case CNC_OUTBOARD_ROLE_POSITION: return "position";
    case CNC_OUTBOARD_ROLE_VELOCITY: return "velocity";
    case CNC_OUTBOARD_ROLE_TORQUE: return "torque";
    case CNC_OUTBOARD_ROLE_STATUS_WORD: return "status-word";
    case CNC_OUTBOARD_ROLE_CONTROL_WORD: return "control-word";
    case CNC_OUTBOARD_ROLE_ENCODER_COUNT: return "encoder-count";
    case CNC_OUTBOARD_ROLE_PROBE_LATCH: return "probe-latch";
    case CNC_OUTBOARD_ROLE_FAST_DIGITAL: return "fast-digital";
    }
    return "?";
}

/*
 * One line per signal, under the name the other side has to say.
 *
 * A count is not enough, and that is not a matter of taste. The machine
 * description on the far side names these pins as text -- `axis.0.position_fb:
 * hm2_7i76e.0.stepgen.00.position-fb` -- and a name that does not match is
 * refused at start-up with nothing to compare it against, because the count
 * this process used to print says only how many there were. The names depend
 * on the board, on which sserial devices answered, and on how many characters
 * of the board name the transport copied, so they cannot be derived from the
 * configuration either: they have to be read from the process that made them.
 *
 * At INFO rather than DEBUG because this runs once, before the cycle starts,
 * and because the run that needs it is the one nobody planned to debug. It is
 * a few hundred lines on a fully populated card, once per start, and it is the
 * record of what the card actually presented that day.
 */
static void log_declared(size_t index, const char *name, const char *direction, uint32_t type,
                         uint32_t role, size_t value_index) {
    hm2_log(HM2_LOG_INFO, "  signal %3zu %-3s %-7s %-13s value %3zu  %s", index, direction,
            type_name(type), role_name(role), value_index, name);
}

static void declare_signals(hm2_region *region) {
    size_t index = 0;
    for (size_t i = 0; i < bound_input_count; i++) {
        const hm2_shim_signal *signal = bound_inputs[i].signal;
        uint32_t type = channel_type(signal->type);
        uint32_t role = channel_role(signal->name, signal->type);
        size_t slot = index++;
        hm2_region_declare(region, slot, signal->name, CNC_OUTBOARD_SIGNAL_INPUT, type, role,
                           CNC_OUTBOARD_UNIT_NONE, (uint32_t)bound_inputs[i].value_index);
        log_declared(slot, signal->name, "in", type, role, bound_inputs[i].value_index);
    }
    for (size_t i = 0; i < bound_output_count; i++) {
        const hm2_shim_signal *signal = bound_outputs[i].signal;
        uint32_t type = channel_type(signal->type);
        uint32_t role = channel_role(signal->name, signal->type);
        size_t slot = index++;
        hm2_region_declare(region, slot, signal->name, CNC_OUTBOARD_SIGNAL_OUTPUT, type, role,
                           CNC_OUTBOARD_UNIT_NONE, (uint32_t)bound_outputs[i].value_index);
        log_declared(slot, signal->name, "out", type, role, bound_outputs[i].value_index);
    }
}

/* ---------------------------------------------------------------------------
 * The cycle
 * ------------------------------------------------------------------------- */

static void sleep_until(const struct timespec *deadline) {
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, deadline, NULL) == EINTR) {
        if (stopping) {
            return;
        }
    }
}

static void add_ns(struct timespec *when, long long ns) {
    when->tv_nsec += ns % 1000000000LL;
    when->tv_sec += ns / 1000000000LL;
    if (when->tv_nsec >= 1000000000L) {
        when->tv_nsec -= 1000000000L;
        when->tv_sec += 1;
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr,
                "usage: hm2-host <config file> [module directory]\n"
                "\n"
                "Runs LinuxCNC's HostMot2 driver as a process of its own and publishes\n"
                "every HAL pin it declares into a shared region the control attaches to\n"
                "(ADR 0022). What a pin means is said in the control's machine\n"
                "description, not here.\n");
        return 2;
    }
    const char *config_path = argv[1];
    const char *module_dir = argc > 2 ? argv[2] : ".";

    hm2_config config;
    if (hm2_config_load(&config, config_path) != 0) {
        return 1;
    }
    hm2_log_set_level(config.log_level);
    hm2_shim_set_log(shim_log_sink);
    hm2_shim_set_msg_level(config.log_level);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* The arena and the tables, before the driver allocates anything. */
    if (hm2_shim_init(config.arena_bytes, config.max_signals, config.max_functs) != 0) {
        hm2_config_free(&config);
        return 1;
    }
    hm2_shim_set_params(config.param_names, config.param_values, config.param_count);

    char path[1024];
    hm2_module generic = {0};
    hm2_module transport = {0};
    snprintf(path, sizeof(path), "%s/libhostmot2.so", module_dir);
    if (load_module(&generic, path, 1) != 0) {
        goto fail;
    }
    snprintf(path, sizeof(path), "%s/lib%s.so", module_dir, config.transport);
    if (load_module(&transport, path, 0) != 0) {
        goto fail;
    }

    /*
     * The module parameters, before either module's `rtapi_app_main`: this is
     * the `loadrt hm2_eth board_ip=10.10.10.10 config="num_encoders=3"` line,
     * moved into a file. They are set after the modules are loaded because
     * that is when their symbols exist, and before main is called because that
     * is when they are read.
     */
    for (size_t i = 0; i < config.module_param_count; i++) {
        const struct hm2_module_param *param = &config.module_params[i];
        /* The transport first, because that is where a board's parameters
           are, and the generic driver second. Whichever has it, wins. */
        int placed = hm2_shim_set_module_param(transport.handle, param->name, param->index,
                                               param->value);
        if (placed > 0) {
            placed = hm2_shim_set_module_param(generic.handle, param->name, param->index,
                                               param->value);
        }
        if (placed != 0) {
            hm2_log(HM2_LOG_ERROR,
                    "module.%s in %s is not a parameter either driver module declares. "
                    "These are the names a LinuxCNC `loadrt` line uses -- board_ip, "
                    "config, debug",
                    param->name, config_path);
            goto fail;
        }
    }

    /*
     * The generic driver first, then the transport, which is the order
     * `loadrt` uses: the transport registers its boards with the generic
     * driver, and the generic driver is what turns an IDROM into pins.
     */
    hm2_log(HM2_LOG_INFO, "starting the driver");
    int status = generic.app_main();
    if (status != 0) {
        hm2_log(HM2_LOG_ERROR, "libhostmot2 failed to start: %d", status);
        goto fail;
    }
    status = transport.app_main();
    if (status != 0) {
        hm2_log(HM2_LOG_ERROR,
                "the %s transport failed to start: %d. The board's address in %s is the "
                "first thing to check, then whether anything else has the board open",
                config.transport, status, config_path);
        goto fail;
    }
    if (!hm2_shim_component_ready()) {
        hm2_log(HM2_LOG_WARN, "the driver never called hal_ready; carrying on with the pins "
                              "it declared");
    }

    /*
     * What nobody claimed. A parameter the file set that no declaration
     * matched is a typo far more often than anything else, and the failure it
     * produces otherwise is a machine that moves the wrong distance in
     * silence.
     */
    const char *unclaimed[16];
    size_t stray = hm2_shim_unclaimed_params(unclaimed, 16);
    for (size_t i = 0; i < stray && i < 16; i++) {
        hm2_log(HM2_LOG_ERROR,
                "param.%s in %s matched no parameter the driver declared. Check the "
                "spelling: nothing will use it and the driver is on its default",
                unclaimed[i], config_path);
    }
    if (stray > 0) {
        goto fail;
    }

    const hm2_shim_funct *read = hm2_shim_funct_ending(".read");
    const hm2_shim_funct *write = hm2_shim_funct_ending(".write");
    if (!read || !write) {
        hm2_log(HM2_LOG_ERROR,
                "the driver exported no read or write function; there is nothing to cycle");
        goto fail;
    }

    if (bind_signals() != 0) {
        goto fail;
    }

    hm2_region region;
    if (hm2_region_create(&region, config.region, bound_input_count, bound_output_count) != 0) {
        goto fail;
    }
    hm2_region_config header = {
        .axis_count = config.axis_count,
        .cycle_ns = config.cycle_us * 1000u,
        .core_watchdog_cycles = config.core_watchdog_cycles,
        .response_watchdog_cycles = config.response_watchdog_cycles,
        .drive_watchdog_us = config.drive_watchdog_us,
        .spin_iterations = config.spin_iterations,
        .worst_case_cycle_ns = config.worst_case_cycle_ns,
    };
    hm2_region_describe(&region, &header);
    declare_signals(&region);
    hm2_region_set_state(&region, CNC_OUTBOARD_STATE_RUNNING);

    hm2_log(HM2_LOG_INFO,
            "region %s: %zu signal(s), %zu in and %zu out, %u us cycle, generation %llu",
            config.region, bound_input_count + bound_output_count, bound_input_count,
            bound_output_count, config.cycle_us,
            (unsigned long long)region.shm->config.generation);
    hm2_log(HM2_LOG_INFO, "cyclic functions: '%s' and '%s'", read->name, write->name);

    ask_for_realtime(&config);

    const long long period_ns = (long long)config.cycle_us * 1000LL;
    const long long send_ns = (long long)(period_ns * config.send_deadline);
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    uint64_t published = 0;
    uint64_t unanswered = 0;
    uint64_t stale = 0;
    int drives_dropped = 0;

    while (!stopping) {
        add_ns(&next, period_ns);
        sleep_until(&next);
        if (stopping) {
            break;
        }

        /*
         * 1. The core's answer, into the driver's pins.
         *
         * Indexed by the cycle the outputs actually came from rather than the
         * current one: those differ exactly when the core is late, which is
         * the case this has to be right for.
         */
        uint64_t answered = 0;
        int freshness = hm2_region_collect(&region, &answered);
        if (freshness == HM2_COLLECT_FRESH || freshness == HM2_COLLECT_STALE) {
            const double *values = hm2_region_outputs(&region, answered);
            if (values) {
                for (size_t i = 0; i < bound_output_count; i++) {
                    hm2_shim_cell_set(bound_outputs[i].signal, values[bound_outputs[i].value_index]);
                }
            }
            if (freshness == HM2_COLLECT_STALE) {
                stale++;
            }
        }

        /*
         * 2. and 3. The driver's own cycle: write, then read.
         *
         * In that order, and it is the order LinuxCNC's HAL file uses. The
         * write puts this cycle's setpoints into the outgoing packet and pets
         * the watchdog; the read is the round trip that brings the answer
         * back. Reading first would publish feedback one cycle older than the
         * setpoints beside it.
         */
        write->funct(write->arg, (long)period_ns);
        read->funct(read->arg, (long)period_ns);

        /* 4. Publish, and wake the core. */
        uint64_t cycle = hm2_region_begin(&region);
        if (cycle == 0) {
            /*
             * The core has not finished the previous cycle. ADR 0011 says to
             * repeat the last complete output, which is what happens by doing
             * nothing -- the driver keeps the setpoints it has.
             */
            unanswered++;
            if (!drives_dropped && config.response_watchdog_cycles > 0 &&
                unanswered >= config.response_watchdog_cycles) {
                /*
                 * Right immediately, wrong indefinitely. The core has stopped
                 * answering, so nothing is steering the machine any more, and
                 * the honest end is to stop petting the FPGA's watchdog and
                 * let it put the outputs in their safe state and drop the
                 * drive enables (ADR 0011 §5, ADR 0022 §7). It is the FPGA
                 * that stops the machine, not this process, which is the
                 * point: this process may itself be what has gone wrong.
                 */
                drives_dropped = 1;
                hm2_log(HM2_LOG_ERROR,
                        "the core has not answered for %llu cycles; letting the FPGA "
                        "watchdog bite. The outputs go to their safe state and the drive "
                        "enables drop",
                        (unsigned long long)unanswered);
                hm2_region_set_state(&region, CNC_OUTBOARD_STATE_FAULTED);
                break;
            }
        } else {
            unanswered = 0;
            double *values = hm2_region_inputs(&region, cycle);
            if (values) {
                for (size_t i = 0; i < bound_input_count; i++) {
                    values[bound_inputs[i].value_index] = hm2_shim_cell_get(bound_inputs[i].signal);
                }
            }
            hm2_region_publish(&region, cycle);
            published = cycle;
        }

        /* The core said it is going away. Nothing to wait for. */
        uint32_t state = hm2_region_state(&region);
        if (state == CNC_OUTBOARD_STATE_SHUTDOWN || state == CNC_OUTBOARD_STATE_FAULTED) {
            hm2_log(HM2_LOG_INFO, "the core has %s",
                    state == CNC_OUTBOARD_STATE_SHUTDOWN ? "shut down" : "faulted");
            break;
        }

        /* The send deadline, for the next collect: a real master puts its
           outputs on the wire near the end of the period, which is what gives
           the core the cycle to answer in. */
        struct timespec send = next;
        add_ns(&send, send_ns);
        sleep_until(&send);
    }

    hm2_log(HM2_LOG_INFO, "stopping: %llu cycle(s), %llu overrun(s), %llu stale collect(s)",
            (unsigned long long)published, (unsigned long long)region.overruns,
            (unsigned long long)stale);

    if (transport.app_exit) {
        transport.app_exit();
    }
    if (generic.app_exit) {
        generic.app_exit();
    }
    hm2_shim_flush_log();
    hm2_region_destroy(&region);
    free(bound_inputs);
    free(bound_outputs);
    hm2_shim_fini();
    hm2_config_free(&config);
    return 0;

fail:
    if (transport.app_exit && transport.handle) {
        transport.app_exit();
    }
    if (generic.app_exit && generic.handle) {
        generic.app_exit();
    }
    hm2_shim_flush_log();
    free(bound_inputs);
    free(bound_outputs);
    hm2_shim_fini();
    hm2_config_free(&config);
    return 1;
}
