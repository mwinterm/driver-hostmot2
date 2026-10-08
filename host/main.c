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
 *     2. call the driver's read function: one LBP16 round trip over UDP;
 *     3. call its write function: setpoints out, watchdog petted, the next
 *        Smart Serial transaction started -- last, so it has the period;
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
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
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

/*
 * The log, once the cycle runs: a ring the cycle's thread writes lines into and
 * a writer thread of ordinary priority drains to stderr, so the cycle never
 * waits for a disk. stderr is a file on the machine's SD card, and on the
 * control's side a log line written from the servo thread during a burst of
 * disk writes blocked it for 1.7 s (the control's ADR 0046). This process is
 * the control's clock, and the driver logs from inside the cycle -- a late
 * reply is a line -- so the same write here would stop both.
 *
 * One producer, the thread that runs the cycle (this process has no other that
 * logs), and one consumer, so two counters and no lock. A line the ring has no
 * room for is dropped and counted, never waited for; the writer says how many
 * when it next writes. Before the writer starts and after it stops, a line goes
 * straight to stderr: start-up's few hundred lines -- the signal table among
 * them, the record of what the card presented -- are worth waiting for.
 */
#define LOG_SLOTS 256
#define LOG_LINE 512
static char log_ring[LOG_SLOTS][LOG_LINE];
static atomic_size_t log_head;
static atomic_size_t log_tail;
static atomic_ullong log_dropped;
static atomic_int log_ringed;
static atomic_int log_stop;
static pthread_t log_thread;

static void *log_writer(void *unused) {
    (void)unused;
    unsigned long long reported = 0;
    for (;;) {
        size_t tail = atomic_load_explicit(&log_tail, memory_order_relaxed);
        size_t head = atomic_load_explicit(&log_head, memory_order_acquire);
        for (; tail != head; tail++) {
            fputs(log_ring[tail % LOG_SLOTS], stderr);
            atomic_store_explicit(&log_tail, tail + 1, memory_order_release);
        }
        unsigned long long dropped = atomic_load(&log_dropped);
        if (dropped != reported) {
            fprintf(stderr, "%llu log line(s) dropped: the writer could not keep up\n",
                    dropped - reported);
            reported = dropped;
        }
        fflush(stderr);
        if (atomic_load(&log_stop) &&
            atomic_load_explicit(&log_head, memory_order_acquire) == tail) {
            return NULL;
        }
        struct timespec pause = {.tv_sec = 0, .tv_nsec = 10 * 1000 * 1000};
        nanosleep(&pause, NULL);
    }
}

/*
 * Starts the writer. Before the process takes its real-time priority and its
 * CPU, so the writer inherits neither: it runs where ordinary work runs.
 */
static void log_to_ring(void) {
    if (pthread_create(&log_thread, NULL, log_writer, NULL) == 0) {
        atomic_store(&log_ringed, 1);
    }
}

/* Drains the ring and stops the writer; lines go straight out again. */
static void log_from_ring(void) {
    if (!atomic_load(&log_ringed)) {
        return;
    }
    atomic_store(&log_stop, 1);
    pthread_join(log_thread, NULL);
    atomic_store(&log_ringed, 0);
}

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

    char line[LOG_LINE];
    int used = snprintf(line, sizeof(line), "%s.%06ldZ %-5s hm2-host: ", when,
                        now.tv_nsec / 1000, level_name[level < 6 ? level : 5]);
    if (used < 0 || (size_t)used >= sizeof(line) - 1) {
        used = 0;
    }
    va_list args;
    va_start(args, fmt);
    int message = vsnprintf(line + used, sizeof(line) - (size_t)used - 1, fmt, args);
    va_end(args);
    size_t length = (size_t)used + (message < 0 ? 0 : (size_t)message);
    if (length > sizeof(line) - 2) {
        length = sizeof(line) - 2; /* cut, and say so with the newline kept */
    }
    line[length] = '\n';
    line[length + 1] = '\0';

    if (!atomic_load(&log_ringed)) {
        /* stderr, so this interleaves with whatever collects the core's log. */
        fputs(line, stderr);
        return;
    }
    size_t head = atomic_load_explicit(&log_head, memory_order_relaxed);
    size_t tail = atomic_load_explicit(&log_tail, memory_order_acquire);
    if (head - tail >= LOG_SLOTS) {
        atomic_fetch_add(&log_dropped, 1);
        return;
    }
    memcpy(log_ring[head % LOG_SLOTS], line, length + 2);
    atomic_store_explicit(&log_head, head + 1, memory_order_release);
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
    /* CNC_OUTBOARD_SIGNAL_* (channel 2.2.0, ADR 0045). */
    uint32_t flags;
    /* On an ON_CHANGE output, the value last applied to the pin. */
    double last;
} bound_signal;

static bound_signal *bound_inputs;
static size_t bound_input_count;
static bound_signal *bound_outputs;
static size_t bound_output_count;

/*
 * What this process measures of itself (ADR 0046 §4), published as inputs
 * after the driver's own, under names no HAL pin has. In SI, as everything on
 * the channel is: seconds, and a count.
 */
enum { HOST_READ_TIME, HOST_ANSWER_TIME, HOST_ANSWERS_LATE, HOST_SSERIAL_FAULTS, HOST_SIGNALS };
static const struct {
    const char *name;
    uint32_t type;
    uint32_t unit;
} host_signals[HOST_SIGNALS] = {
    {"hm2-host.read-time", CNC_OUTBOARD_SIGNAL_F64, CNC_OUTBOARD_UNIT_SECOND},
    {"hm2-host.answer-time", CNC_OUTBOARD_SIGNAL_F64, CNC_OUTBOARD_UNIT_SECOND},
    {"hm2-host.answers-late", CNC_OUTBOARD_SIGNAL_I32, CNC_OUTBOARD_UNIT_COUNT},
    {"hm2-host.sserial-faults", CNC_OUTBOARD_SIGNAL_I32, CNC_OUTBOARD_UNIT_COUNT},
};

/* The pins `pin.` lines fixed, so binding publishes them for reading only. */
static const hm2_shim_signal **fixed_pins;
static size_t fixed_pin_count;

static int is_fixed(const hm2_shim_signal *signal) {
    for (size_t i = 0; i < fixed_pin_count; i++) {
        if (fixed_pins[i] == signal) {
            return 1;
        }
    }
    return 0;
}

/*
 * Whether the driver writes this pin as well as reads it: a HAL_IO pin, which
 * the shim publishes once in each direction over one cell (hm2_shim.h).
 */
static int written_by_the_driver(const hm2_shim_signal *signal) {
    size_t count = hm2_shim_signal_count();
    for (size_t i = 0; i < count; i++) {
        const hm2_shim_signal *other = hm2_shim_signal_at(i);
        if (other != signal && other->cell == signal->cell && other->dir == HM2_SHIM_TO_CORE) {
            return 1;
        }
    }
    return 0;
}

/*
 * The `pin.` lines (ADR 0045 §2): each names a pin the driver reads and only
 * reads, whose value is then set here, before the board's first cycle, and
 * published for reading only. Everything else is refused by name: a pin
 * nobody declared, one the driver writes (it would overwrite the value at its
 * next read), a both-ways pin, a value that does not fit the pin's type.
 */
static int fix_pins(const hm2_config *config, const char *config_path) {
    fixed_pins = calloc(config->pin_count ? config->pin_count : 1, sizeof(*fixed_pins));
    if (!fixed_pins) {
        return -1;
    }
    int failures = 0;
    size_t count = hm2_shim_signal_count();
    for (size_t p = 0; p < config->pin_count; p++) {
        const char *name = config->pin_names[p];
        double value = config->pin_values[p];
        int line = config->pin_lines[p];
        const hm2_shim_signal *read_by_driver = NULL;
        const hm2_shim_signal *written = NULL;
        for (size_t i = 0; i < count; i++) {
            const hm2_shim_signal *signal = hm2_shim_signal_at(i);
            if (strcmp(signal->name, name) != 0) {
                continue;
            }
            if (signal->dir == HM2_SHIM_FROM_CORE) {
                read_by_driver = signal;
            } else {
                written = signal;
            }
        }
        if (!read_by_driver && !written) {
            hm2_log(HM2_LOG_ERROR,
                    "%s:%d: pin.%s is not a pin the driver declared. The names are the ones "
                    "this process logs at start, one line per signal",
                    config_path, line, name);
            failures++;
            continue;
        }
        if (written) {
            hm2_log(HM2_LOG_ERROR,
                    "%s:%d: pin.%s is a pin the driver writes%s, so a value fixed here would "
                    "be overwritten at its next read. Only a pin the driver reads and nothing "
                    "drives can be fixed (ADR 0045 §2)",
                    config_path, line, name, read_by_driver ? " as well as reads" : "");
            failures++;
            continue;
        }
        int fits = 1;
        switch (read_by_driver->type) {
        case HM2_SHIM_BOOL: fits = value == 0.0 || value == 1.0; break;
        case HM2_SHIM_SINT:
            fits = value == (double)(long long)value && value >= -2147483648.0 &&
                   value <= 2147483647.0;
            break;
        case HM2_SHIM_UINT:
            fits = value == (double)(long long)value && value >= 0.0 && value <= 4294967295.0;
            break;
        case HM2_SHIM_REAL: break;
        }
        if (!fits) {
            hm2_log(HM2_LOG_ERROR,
                    "%s:%d: pin.%s = %g does not fit the pin, which is %s",
                    config_path, line, name, value,
                    read_by_driver->type == HM2_SHIM_BOOL ? "a bit: true or false"
                    : read_by_driver->type == HM2_SHIM_REAL ? "a number"
                                                            : "an integer in its range");
            failures++;
            continue;
        }
        hm2_shim_cell_set(read_by_driver, value);
        fixed_pins[fixed_pin_count++] = read_by_driver;
        hm2_log(HM2_LOG_INFO, "pin %s fixed at %g; published for reading only", name, value);
    }
    return failures ? -1 : 0;
}

/*
 * The driver's parameters as the channel carries them (the control's ADR
 * 0053): one view per direction a parameter crosses in, so a bound signal
 * points at a view as it points at a pin. Allocated once, at binding.
 */
static hm2_shim_signal *param_views;
static size_t param_view_count;
static size_t params_published;
static size_t params_writable;

/* Whether a pin already crosses the channel under `name` in `dir`: the
   channel allows one name per direction, and the pin keeps it. */
static int pin_named(const char *name, hm2_shim_dir dir) {
    size_t count = hm2_shim_signal_count();
    for (size_t i = 0; i < count; i++) {
        const hm2_shim_signal *pin = hm2_shim_signal_at(i);
        if (pin->dir == dir && strcmp(pin->name, name) == 0) {
            return 1;
        }
    }
    return 0;
}

/*
 * Every parameter, after the pins (the control's ADR 0053): read back as an
 * input, so the control sees what the driver runs on; and a parameter the
 * driver declared HAL_RW written as an output as well, applied only when the
 * control's value changes -- `halcmd setp` across the channel. Its starting
 * value is the one the configuration set, so nothing changes until the
 * control asks. Whether a change is allowed at that moment is the control's
 * to decide; this process applies what it is sent.
 */
static void bind_params(void) {
    size_t count = hm2_shim_param_count();
    for (size_t k = 0; k < count; k++) {
        hm2_shim_signal view;
        int writable = 0;
        if (!hm2_shim_param_view(k, &view, &writable)) {
            continue;
        }
        if (pin_named(view.name, HM2_SHIM_TO_CORE)) {
            hm2_log(HM2_LOG_WARN, "parameter %s has a pin's name; the pin is published "
                                  "under it and the parameter is not", view.name);
            continue;
        }
        hm2_shim_signal *in = &param_views[param_view_count++];
        *in = view;
        in->dir = HM2_SHIM_TO_CORE;
        bound_inputs[bound_input_count].signal = in;
        bound_inputs[bound_input_count].value_index = bound_input_count;
        bound_inputs[bound_input_count].flags = 0u;
        bound_input_count++;
        params_published++;
        if (writable && !pin_named(view.name, HM2_SHIM_FROM_CORE)) {
            hm2_shim_signal *out = &param_views[param_view_count++];
            *out = view;
            out->dir = HM2_SHIM_FROM_CORE;
            bound_outputs[bound_output_count].signal = out;
            bound_outputs[bound_output_count].value_index = bound_output_count;
            bound_outputs[bound_output_count].flags = CNC_OUTBOARD_SIGNAL_ON_CHANGE;
            bound_output_count++;
            params_writable++;
        }
    }
}

static int bind_signals(void) {
    size_t count = hm2_shim_signal_count();
    size_t params = hm2_shim_param_count();
    bound_inputs = calloc(count + params + 1, sizeof(*bound_inputs));
    bound_outputs = calloc(count + params + 1, sizeof(*bound_outputs));
    param_views = calloc(2 * params + 1, sizeof(*param_views));
    if (!bound_inputs || !bound_outputs || !param_views) {
        return -1;
    }
    for (size_t i = 0; i < count; i++) {
        const hm2_shim_signal *signal = hm2_shim_signal_at(i);
        if (signal->dir == HM2_SHIM_TO_CORE || is_fixed(signal)) {
            /* A pin a `pin.` line fixed is read, never written: the core sees
               its value and nothing on that side can change it. */
            bound_inputs[bound_input_count].signal = signal;
            bound_inputs[bound_input_count].value_index = bound_input_count;
            bound_inputs[bound_input_count].flags =
                signal->dir == HM2_SHIM_TO_CORE ? 0u : CNC_OUTBOARD_SIGNAL_FIXED;
            bound_input_count++;
        } else {
            /* A pin the driver writes too is applied only when the core's
               value changes, and left to the driver between (ADR 0045 §1). */
            bound_outputs[bound_output_count].signal = signal;
            bound_outputs[bound_output_count].value_index = bound_output_count;
            bound_outputs[bound_output_count].flags =
                written_by_the_driver(signal) ? CNC_OUTBOARD_SIGNAL_ON_CHANGE : 0u;
            bound_output_count++;
        }
    }
    bind_params();
    hm2_log(HM2_LOG_INFO, "%zu parameter(s) published, %zu of them writable at run time "
                          "(ADR 0053)",
            params_published, params_writable);
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
    if (strstr(name, ".index-enable") || strstr(name, ".probe-enable") || strstr(name, ".latch")) {
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
                         uint32_t role, size_t value_index, uint32_t flags) {
    const char *how = (flags & CNC_OUTBOARD_SIGNAL_ON_CHANGE) ? "on-change"
                      : (flags & CNC_OUTBOARD_SIGNAL_FIXED)   ? "fixed"
                                                              : "";
    hm2_log(HM2_LOG_INFO, "  signal %3zu %-3s %-7s %-13s value %3zu %-9s %s", index, direction,
            type_name(type), role_name(role), value_index, how, name);
}

static void declare_signals(hm2_region *region) {
    size_t index = 0;
    for (size_t i = 0; i < bound_input_count; i++) {
        const hm2_shim_signal *signal = bound_inputs[i].signal;
        uint32_t type = channel_type(signal->type);
        uint32_t role = channel_role(signal->name, signal->type);
        size_t slot = index++;
        hm2_region_declare(region, slot, signal->name, CNC_OUTBOARD_SIGNAL_INPUT, type, role,
                           CNC_OUTBOARD_UNIT_NONE, (uint32_t)bound_inputs[i].value_index,
                           bound_inputs[i].flags);
        log_declared(slot, signal->name, "in", type, role, bound_inputs[i].value_index,
                     bound_inputs[i].flags);
    }
    for (size_t i = 0; i < HOST_SIGNALS; i++) {
        size_t slot = index++;
        size_t value_index = bound_input_count + i;
        hm2_region_declare(region, slot, host_signals[i].name, CNC_OUTBOARD_SIGNAL_INPUT,
                           host_signals[i].type, CNC_OUTBOARD_ROLE_ANALOG, host_signals[i].unit,
                           (uint32_t)value_index, 0u);
        log_declared(slot, host_signals[i].name, "in", host_signals[i].type,
                     CNC_OUTBOARD_ROLE_ANALOG, value_index, 0u);
    }
    for (size_t i = 0; i < bound_output_count; i++) {
        const hm2_shim_signal *signal = bound_outputs[i].signal;
        uint32_t type = channel_type(signal->type);
        uint32_t role = channel_role(signal->name, signal->type);
        size_t slot = index++;
        hm2_region_declare(region, slot, signal->name, CNC_OUTBOARD_SIGNAL_OUTPUT, type, role,
                           CNC_OUTBOARD_UNIT_NONE, (uint32_t)bound_outputs[i].value_index,
                           bound_outputs[i].flags);
        log_declared(slot, signal->name, "out", type, role, bound_outputs[i].value_index,
                     bound_outputs[i].flags);
    }
}

/*
 * The newest complete answer from the core, into the driver's pins.
 *
 * Indexed by the cycle the outputs actually came from rather than the current
 * one: those differ exactly when the core is late, which is the case this has
 * to be right for.
 */
static void take_answer(hm2_region *region, int *attached, uint64_t *stale,
                        uint32_t response_watchdog_cycles) {
    uint64_t answered = 0;
    int freshness = hm2_region_collect(region, &answered);
    if (freshness != HM2_COLLECT_FRESH && freshness != HM2_COLLECT_STALE) {
        return;
    }
    const double *values = hm2_region_outputs(region, answered);
    if (values) {
        for (size_t i = 0; i < bound_output_count; i++) {
            double value = values[bound_outputs[i].value_index];
            /*
             * A pin the driver writes too is the core's only when the core
             * changes it (ADR 0045 §1): written every cycle, the core's held
             * zero undid a watchdog bite the cycle after it happened. A stale
             * answer repeats the last one, which is no change, so a write is
             * applied once.
             */
            if (bound_outputs[i].flags & CNC_OUTBOARD_SIGNAL_ON_CHANGE) {
                if (value == bound_outputs[i].last) {
                    continue;
                }
                bound_outputs[i].last = value;
            }
            hm2_shim_cell_set(bound_outputs[i].signal, value);
        }
    }
    if (freshness == HM2_COLLECT_STALE) {
        (*stale)++;
    }
    if (!*attached) {
        *attached = 1;
        hm2_log(HM2_LOG_INFO, "the core has attached; from now on %u unanswered cycle(s) "
                "let the FPGA watchdog bite", response_watchdog_cycles);
    }
}

static double seconds_between(const struct timespec *from, const struct timespec *to) {
    return (double)(to->tv_sec - from->tv_sec) + (double)(to->tv_nsec - from->tv_nsec) * 1e-9;
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

/*
 * The worst cycle this process declares in the channel header, where the
 * core's start-up check reads it (§10.2): measured rather than guessed.
 *
 * Before the region exists, so no core can attach and read the figure while
 * it is still being made, the card is read and written for `measure_cycles`
 * periods at the cycle's own period and priority -- what happens before a
 * core attaches anyway: the driver's own values out, the watchdog petted,
 * read then write as the cycle without one does, which leaves a Smart Serial
 * transfer the rest of the period. The first tenth is not counted: the first
 * packets and transactions and a cold cache are the start, not a cycle.
 * What is declared is the worst read plus write of the rest, times
 * `worst_case_margin`. A figure in the file is declared as it is, and the
 * measurement is logged beside it.
 */
static uint64_t declare_worst_cycle(const hm2_config *config, const hm2_shim_funct *read,
                                    const hm2_shim_funct *write) {
    const long long period_ns = (long long)config->cycle_us * 1000LL;
    const uint32_t skipped = config->measure_cycles / 10u;
    double worst_s = 0.0;
    double read_worst_s = 0.0;
    double write_worst_s = 0.0;
    uint32_t counted = 0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    for (uint32_t i = 0; i < config->measure_cycles && !stopping; i++) {
        add_ns(&next, period_ns);
        sleep_until(&next);
        struct timespec start;
        struct timespec between;
        struct timespec end;
        clock_gettime(CLOCK_MONOTONIC, &start);
        read->funct(read->arg, (long)period_ns);
        clock_gettime(CLOCK_MONOTONIC, &between);
        write->funct(write->arg, (long)period_ns);
        clock_gettime(CLOCK_MONOTONIC, &end);
        if (i < skipped) {
            continue;
        }
        double read_s = seconds_between(&start, &between);
        double write_s = seconds_between(&between, &end);
        counted++;
        if (read_s > read_worst_s) {
            read_worst_s = read_s;
        }
        if (write_s > write_worst_s) {
            write_worst_s = write_s;
        }
        if (read_s + write_s > worst_s) {
            worst_s = read_s + write_s;
        }
    }
    if (config->worst_case_cycle_ns > 0) {
        if (counted > 0) {
            hm2_log(HM2_LOG_INFO,
                    "worst cycle: %.0f us declared, as the file says; measured %.0f us at worst "
                    "over %u period(s) (read %.0f us, write %.0f us)",
                    (double)config->worst_case_cycle_ns / 1e3, worst_s * 1e6, counted,
                    read_worst_s * 1e6, write_worst_s * 1e6);
        }
        return config->worst_case_cycle_ns;
    }
    uint64_t declared = (uint64_t)(worst_s * config->worst_case_margin * 1e9) + 1u;
    hm2_log(HM2_LOG_INFO,
            "worst cycle: %.0f us declared, measured %.0f us at worst over %u period(s) (read "
            "%.0f us, write %.0f us) times %.2f",
            (double)declared / 1e3, worst_s * 1e6, counted, read_worst_s * 1e6,
            write_worst_s * 1e6, config->worst_case_margin);
    if (declared > (uint64_t)period_ns) {
        hm2_log(HM2_LOG_WARN,
                "the declared worst cycle, %.0f us, does not fit the %u us cycle: the core will "
                "refuse this machine with both figures named",
                (double)declared / 1e3, config->cycle_us);
    }
    return declared;
}

/* ---------------------------------------------------------------------------
 * --setsserial: a Smart Serial remote's stored settings and firmware
 * ------------------------------------------------------------------------- */

/*
 * LinuxCNC's `setsserial` without LinuxCNC. The driver is loaded from the same
 * file as for a run, every Smart Serial remote's parameters are listed -- its
 * stored settings, the `nv...` ones, and its revisions among them -- and a
 * command, if there is one, is run by upstream's own setsserial module,
 * unmodified: `set <parameter> <value>` writes a stored setting, `flash
 * <remote> <file>.BIN` the remote's firmware. Then the process exits: no
 * region, no cycle, and the card's watchdog stops its outputs meanwhile, as
 * under LinuxCNC's halrun.
 */
static int setsserial_command(const char *word) {
    return strcmp(word, "set") == 0 || strcmp(word, "flash") == 0;
}

/*
 * A control attached to the card, or one that died and left its region: the
 * card answers whoever talks to it, and two masters writing it is how a
 * remote is left half-flashed. So the region the file names must not exist.
 */
static int region_in_use(const hm2_config *config) {
    int fd = shm_open(config->region, O_RDONLY, 0);
    if (fd < 0) {
        return 0;
    }
    close(fd);
    hm2_log(HM2_LOG_ERROR,
            "%s exists: a control is running on this card, or one stopped without removing "
            "it. Stop the control first; if no hm2-host is running, remove /dev/shm%s",
            config->region, config->region);
    return 1;
}

/*
 * Every parameter of a Smart Serial remote, on stdout: hostmot2's own
 * `hm2_get_sserial` says which, as it does for setsserial -- a parameter is a
 * remote's when the remote's name is in it.
 */
static size_t list_sserial(void *generic) {
    void *symbol = dlsym(generic, "hm2_get_sserial");
    void *(*find)(void **, const char *) = NULL;
    memcpy(&find, &symbol, sizeof(symbol));
    if (!find) {
        hm2_log(HM2_LOG_ERROR, "libhostmot2 exports no hm2_get_sserial");
        return 0;
    }
    size_t listed = 0;
    for (size_t i = 0; i < hm2_shim_param_count(); i++) {
        hm2_shim_signal view;
        int writable = 0;
        if (!hm2_shim_param_view(i, &view, &writable)) {
            continue;
        }
        void *board = NULL;
        if (!find(&board, view.name)) {
            continue;
        }
        const char *leaf = strrchr(view.name, '.');
        int stored = leaf && strncmp(leaf + 1, "nv", 2) == 0;
        printf("%-56s %14.10g%s\n", view.name, hm2_shim_cell_get(&view),
               stored ? "  stored" : "");
        listed++;
    }
    fflush(stdout);
    return listed;
}

/* Upstream's setsserial module, given `command` as its `cmd` parameter. */
static int run_setsserial(const char *module_dir, const char *command) {
    char path[1024];
    snprintf(path, sizeof(path), "%s/libsetsserial.so", module_dir);
    hm2_module module = {0};
    if (load_module(&module, path, 0) != 0) {
        return -1;
    }
    if (hm2_shim_set_module_param(module.handle, "cmd", 0, command) != 0) {
        hm2_log(HM2_LOG_ERROR, "%s declares no cmd parameter", path);
        return -1;
    }
    hm2_log(HM2_LOG_INFO, "setsserial: %s", command);
    int status = module.app_main();
    if (module.app_exit) {
        module.app_exit();
    }
    return status;
}

static int setsserial_mode(void *generic, const char *module_dir, char **words, int count) {
    size_t listed = list_sserial(generic);
    if (listed == 0) {
        hm2_log(HM2_LOG_WARN, "no Smart Serial remote answered: nothing to list or set");
    }
    if (count == 0) {
        return listed > 0 ? 0 : 1;
    }
    char command[1024] = "";
    for (int i = 0; i < count; i++) {
        if (strlen(command) + strlen(words[i]) + 2 > sizeof(command)) {
            hm2_log(HM2_LOG_ERROR, "the setsserial command is too long");
            return 1;
        }
        if (i > 0) {
            strcat(command, " ");
        }
        strcat(command, words[i]);
    }
    int status = run_setsserial(module_dir, command);
    if (status != 0) {
        hm2_log(HM2_LOG_ERROR, "setsserial failed (%d): %s", status, command);
        return 1;
    }
    hm2_log(HM2_LOG_INFO,
            "setsserial: done. A remote reads its stored settings when it starts: the new "
            "value is in force, and listed here, after the remote has been powered off and on");
    return 0;
}

int main(int argc, char **argv) {
    int setsserial = argc > 1 && strcmp(argv[1], "--setsserial") == 0;
    int first = setsserial ? 2 : 1;
    if (argc < first + 1) {
        fprintf(stderr,
                "usage: hm2-host <config file> [module directory]\n"
                "       hm2-host --setsserial <config file> [module directory] [command]\n"
                "\n"
                "Runs LinuxCNC's HostMot2 driver as a process of its own and publishes\n"
                "every HAL pin it declares into a shared region the control attaches to\n"
                "(ADR 0022). What a pin means is said in the control's machine\n"
                "description, not here.\n"
                "\n"
                "--setsserial, with the control stopped: lists every Smart Serial remote's\n"
                "parameters, its stored settings marked, and runs LinuxCNC's setsserial\n"
                "on a command:\n"
                "  set <remote's parameter> <value>   a stored setting, one the list marks\n"
                "  flash <remote> <path>.BIN          the remote's firmware\n");
        return 2;
    }
    const char *config_path = argv[first];
    const char *module_dir = ".";
    int command_at = first + 1;
    if (argc > command_at && !setsserial_command(argv[command_at])) {
        module_dir = argv[command_at];
        command_at++;
    }
    int command_words = argc > command_at ? argc - command_at : 0;
    if (command_words > 0 && (!setsserial || command_words != 3 ||
                              !setsserial_command(argv[command_at]))) {
        fprintf(stderr, "hm2-host: a command is `set <parameter> <value>` or `flash <remote> "
                        "<path>.BIN`, after --setsserial\n");
        return 2;
    }

    hm2_config config;
    if (hm2_config_load(&config, config_path) != 0) {
        return 1;
    }
    if (setsserial && region_in_use(&config)) {
        hm2_config_free(&config);
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

    if (setsserial) {
        int failed = setsserial_mode(generic.handle, module_dir, argv + command_at, command_words);
        if (transport.app_exit) {
            transport.app_exit();
        }
        if (generic.app_exit) {
            generic.app_exit();
        }
        hm2_shim_flush_log();
        hm2_shim_fini();
        hm2_config_free(&config);
        return failed;
    }

    const hm2_shim_funct *read = hm2_shim_funct_ending(".read");
    const hm2_shim_funct *write = hm2_shim_funct_ending(".write");
    if (!read || !write) {
        hm2_log(HM2_LOG_ERROR,
                "the driver exported no read or write function; there is nothing to cycle");
        goto fail;
    }

    if (fix_pins(&config, config_path) != 0) {
        goto fail;
    }
    if (bind_signals() != 0) {
        goto fail;
    }

    /* At the cycle's priority from here on, the measurement included. */
    ask_for_realtime(&config);
    uint64_t worst_case_cycle_ns = declare_worst_cycle(&config, read, write);
    if (stopping) {
        goto fail;
    }

    hm2_region region;
    if (hm2_region_create(&region, config.region, bound_input_count + HOST_SIGNALS,
                          bound_output_count) != 0) {
        goto fail;
    }
    hm2_region_config header = {
        .axis_count = config.axis_count,
        .cycle_ns = config.cycle_us * 1000u,
        .core_watchdog_cycles = config.core_watchdog_cycles,
        .response_watchdog_cycles = config.response_watchdog_cycles,
        .drive_watchdog_us = config.drive_watchdog_us,
        .spin_iterations = config.spin_iterations,
        .worst_case_cycle_ns = worst_case_cycle_ns,
        .flags = config.same_cycle ? CNC_OUTBOARD_CONFIG_SAME_CYCLE : 0u,
    };
    hm2_region_describe(&region, &header);
    declare_signals(&region);
    /*
     * Every output's own value, before a core can attach (channel 2.1.0): what
     * the driver set each pin to, and what it holds with nobody driving it. The
     * core starts every output its machine description does not drive from
     * here. Without it, it wrote zero onto them all -- and a Smart Serial
     * port's `run`, which the driver defaults to true, stopped the port and
     * every field input and output of a 7I76 with it.
     */
    for (size_t i = 0; i < bound_output_count; i++) {
        /* An on-change output's starting value is the level the core starts
           from, so its first answer is a write only if it differs. */
        bound_outputs[i].last = hm2_shim_cell_get(bound_outputs[i].signal);
        hm2_region_seed_output(&region, bound_outputs[i].value_index, bound_outputs[i].last);
    }
    hm2_region_set_state(&region, CNC_OUTBOARD_STATE_RUNNING);

    hm2_log(HM2_LOG_INFO,
            "region %s: %zu signal(s), %zu in and %zu out, %u us cycle, generation %llu",
            config.region, bound_input_count + HOST_SIGNALS + bound_output_count,
            bound_input_count + HOST_SIGNALS, bound_output_count, config.cycle_us,
            (unsigned long long)region.shm->config.generation);
    hm2_log(HM2_LOG_INFO, "cyclic functions: '%s' and '%s'", read->name, write->name);

    log_to_ring();
    hm2_shim_enter_cycle();

    const long long period_ns = (long long)config.cycle_us * 1000LL;
    const long long send_ns = (long long)(period_ns * config.send_deadline);
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    uint64_t published = 0;
    uint64_t unanswered = 0;
    uint64_t stale = 0;
    int drives_dropped = 0;
    /*
     * Whether the core has ever answered. Until it has, nothing has been
     * commanded: every output is still the value this process seeded it with
     * (region.c), so there is nothing a silent core could leave on the
     * machine, and its silence is not counted. From its first answer on it is:
     * a core that then stops answering leaves its last setpoints on the
     * drives -- on an analog servo machine, a velocity -- and
     * `response_watchdog_cycles` is how long that may last. Counting from the
     * first cycle instead forced a machine's start script to give the core a
     * minute to attach, which is a minute of a dead core's last velocity.
     */
    int attached = 0;

    /*
     * Whether the board answers (channel 2.2.0, ADR 0045 §3), from what the
     * driver says about itself. The transport sets its `io_error` parameter
     * when packet errors pass its limit and then reads and writes nothing:
     * the link is dead, and it stays dead for this run of the process,
     * because whether the board kept its configuration and its counts across
     * the outage cannot be known from here. A watchdog bite raises `has_bit`,
     * and the board's outputs are in their safe state until it is lowered.
     * Either way this process goes on cycling: it is the core's clock, and a
     * core that keeps its clock faults every axis and says why.
     */
    hm2_shim_signal io_error = {0};
    int has_io_error = hm2_shim_param_ending(".io_error", &io_error);
    const hm2_shim_signal *has_bit = NULL;
    for (size_t i = 0; i < bound_input_count; i++) {
        const char *name = bound_inputs[i].signal->name;
        size_t length = strlen(name);
        if (length >= 17 && strcmp(name + length - 17, ".watchdog.has_bit") == 0) {
            has_bit = bound_inputs[i].signal;
            break;
        }
    }
    if (!has_io_error) {
        hm2_log(HM2_LOG_WARN, "the driver declared no io_error parameter, so a dead link to "
                              "the board cannot be told to the core");
    }
    for (size_t i = 0; i < bound_input_count; i++) {
        sserial_watch_add(bound_inputs[i].signal);
    }
    if (sserial_watched() > 0) {
        hm2_log(HM2_LOG_INFO, "watching %zu Smart Serial port(s) for faults: each is counted in "
                              "hm2-host.sserial-faults and logged",
                sserial_watched());
    }
    uint64_t sserial_faults = 0;
    /* When the last write ended, and whether its answer came in time: what
       the next read's Smart Serial transfer had to work with. */
    struct timespec written_at = {0};
    int written_late = 0;
    double gap_s = 0.0;
    int link_dead = 0;
    int bitten = 0;

    /*
     * What this process measures of itself (ADR 0046 §4), published beside the
     * driver's pins and logged at the end: how long the read took, how long the
     * core took to answer the last publish, and how many answers missed the
     * send deadline. In seconds, as everything on the channel is.
     */
    double read_s = 0.0;
    double answer_s = 0.0;
    uint64_t answers_late = 0;
    uint64_t answers_on_time = 0;
    double read_max_s = 0.0;
    double answer_max_s = 0.0;
    double answer_sum_s = 0.0;
    struct timespec published_at = {0};
    hm2_log(HM2_LOG_INFO,
            config.same_cycle
                ? "same cycle: each period reads, publishes, waits for the core's answer until "
                  "%.0f%% of the period and writes it (ADR 0046)"
                : "not same cycle: each period writes the core's answer to the previous publish "
                  "(send deadline %.0f%%)",
            config.send_deadline * 100.0);

    while (!stopping) {
        add_ns(&next, period_ns);
        sleep_until(&next);
        if (stopping) {
            break;
        }

        /*
         * Not same cycle, the shape before ADR 0046: the core's answer to the
         * *previous* publish goes into the pins first, and read and write
         * follow back to back -- so what is written reaches the card a period
         * after the sample it was computed from.
         */
        if (!config.same_cycle) {
            take_answer(&region, &attached, &stale, config.response_watchdog_cycles);
        }

        /*
         * The read: the round trip that brings the card's state back, first
         * in the period as in a LinuxCNC servo thread. Timed, because it is
         * most of this process's worst cycle and the header's figure for that
         * is a guess until it is measured.
         */
        struct timespec read_start;
        struct timespec read_end;
        clock_gettime(CLOCK_MONOTONIC, &read_start);
        int gap_late = written_late;
        gap_s = written_at.tv_sec ? seconds_between(&written_at, &read_start) : 0.0;
        read->funct(read->arg, (long)period_ns);
        clock_gettime(CLOCK_MONOTONIC, &read_end);
        read_s = seconds_between(&read_start, &read_end);
        if (read_s > read_max_s) {
            read_max_s = read_s;
        }
        if (!config.same_cycle) {
            write->funct(write->arg, (long)period_ns);
            clock_gettime(CLOCK_MONOTONIC, &written_at);
            written_late = 0;
            sserial_faults += sserial_faults_seen(published + 1, gap_s, gap_late, sserial_faults);
        }

        if (!link_dead && has_io_error && hm2_shim_cell_get(&io_error) != 0.0) {
            link_dead = 1;
            hm2_log(HM2_LOG_ERROR,
                    "the link to the board is dead: the driver set io_error and reads and "
                    "writes nothing more, so every input from here on is stale and the "
                    "board's own watchdog has stopped its outputs. The core faults every axis "
                    "behind this process; once the cause is fixed the control may clear "
                    "io_error, or be restarted (ADR 0045 §3, ADR 0053)");
        } else if (link_dead && has_io_error && hm2_shim_cell_get(&io_error) == 0.0) {
            /* The control cleared io_error (ADR 0053): the transport tries the
               board again, and sets it again if the board still does not
               answer. */
            link_dead = 0;
            hm2_log(HM2_LOG_WARN, "io_error was cleared: talking to the board again");
        }
        int bit_now = has_bit && hm2_shim_cell_get(has_bit) != 0.0;
        if (bit_now != bitten) {
            bitten = bit_now;
            if (bitten) {
                hm2_log(HM2_LOG_ERROR, "the board's watchdog has bitten: its outputs are in "
                                       "their safe state until has_bit is lowered, and the "
                                       "core faults every axis behind this process");
            } else {
                hm2_log(HM2_LOG_INFO, "has_bit lowered: the driver re-sends every setting and "
                                      "the board resumes");
            }
        }
        uint32_t bus_state =
            link_dead || bitten ? CNC_OUTBOARD_BUS_FAULT : CNC_OUTBOARD_BUS_OPERATIONAL;
        uint32_t bus_fault = link_dead ? CNC_OUTBOARD_BUS_FAULT_LINK
                             : bitten  ? CNC_OUTBOARD_BUS_FAULT_WATCHDOG
                                       : CNC_OUTBOARD_BUS_FAULT_NONE;

        /* 4. Publish, and wake the core. */
        uint64_t cycle = hm2_region_begin(&region);
        if (cycle == 0) {
            /*
             * The core has not finished the previous cycle. ADR 0011 says to
             * repeat the last complete output, which is what happens by doing
             * nothing -- the driver keeps the setpoints it has.
             */
            if (attached) {
                unanswered++;
            }
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
            if (values) {
                values[bound_input_count + HOST_READ_TIME] = read_s;
                values[bound_input_count + HOST_ANSWER_TIME] = answer_s;
                values[bound_input_count + HOST_ANSWERS_LATE] = (double)answers_late;
                values[bound_input_count + HOST_SSERIAL_FAULTS] = (double)sserial_faults;
            }
            hm2_region_set_bus(&region, cycle, bus_state, bus_fault);
            clock_gettime(CLOCK_MONOTONIC, &published_at);
            hm2_region_publish(&region, cycle);
            published = cycle;
        }

        /*
         * Same cycle (ADR 0046 §1): the core's answer to what was just
         * published, written in this period. Waited for until the send
         * deadline -- the core wakes the futex word after answering -- and
         * only once the core has answered at all: before that there is nobody
         * to wait for. A late answer is not an error: what is written then is
         * the last complete one, as every period wrote before, and it is
         * counted.
         *
         * The write puts the setpoints into the outgoing packet, pets the
         * watchdog and starts the next Smart Serial transaction by setting
         * its DoIt bit, which has the rest of the period to finish before the
         * next read looks for it. Write-then-read back to back gave it tens of
         * microseconds once (v0.3.1): on a 7I76EU every read found DoIt still
         * set, and the port stopped after 200 errors with every field input
         * and output behind it.
         */
        if (config.same_cycle) {
            written_late = 0;
            if (cycle != 0 && hm2_region_answered(&region) > 0) {
                struct timespec send = next;
                add_ns(&send, send_ns);
                if (hm2_region_wait_answer(&region, cycle, &send)) {
                    struct timespec now;
                    clock_gettime(CLOCK_MONOTONIC, &now);
                    answer_s = seconds_between(&published_at, &now);
                    answers_on_time++;
                    answer_sum_s += answer_s;
                    if (answer_s > answer_max_s) {
                        answer_max_s = answer_s;
                    }
                } else {
                    answers_late++;
                    written_late = 1;
                }
            }
            take_answer(&region, &attached, &stale, config.response_watchdog_cycles);
            write->funct(write->arg, (long)period_ns);
            clock_gettime(CLOCK_MONOTONIC, &written_at);
            sserial_faults += sserial_faults_seen(cycle ? cycle : published, gap_s, gap_late,
                                                  sserial_faults);
        }

        /* The core said it is going away. Nothing to wait for. */
        uint32_t state = hm2_region_state(&region);
        if (state == CNC_OUTBOARD_STATE_SHUTDOWN || state == CNC_OUTBOARD_STATE_FAULTED) {
            hm2_log(HM2_LOG_INFO, "the core has %s",
                    state == CNC_OUTBOARD_STATE_SHUTDOWN ? "shut down" : "faulted");
            break;
        }

        /* Not same cycle: the send deadline, for the next collect. A real
           master puts its outputs on the wire near the end of the period,
           which is what gives the core the cycle to answer in. */
        if (!config.same_cycle) {
            struct timespec send = next;
            add_ns(&send, send_ns);
            sleep_until(&send);
        }
    }

    hm2_log(HM2_LOG_INFO, "stopping: %llu cycle(s), %llu overrun(s), %llu stale collect(s)",
            (unsigned long long)published, (unsigned long long)region.overruns,
            (unsigned long long)stale);
    hm2_log(HM2_LOG_INFO, "the read took %.0f us at worst", read_max_s * 1e6);
    if (sserial_watched() > 0) {
        hm2_log(sserial_faults ? HM2_LOG_WARN : HM2_LOG_INFO, "Smart Serial: %llu fault(s)",
                (unsigned long long)sserial_faults);
    }
    if (config.same_cycle) {
        hm2_log(HM2_LOG_INFO,
                "same cycle: %llu answer(s) within the period, %llu late; the core answered "
                "%.0f us after the publish on average, %.0f us at worst",
                (unsigned long long)answers_on_time, (unsigned long long)answers_late,
                answers_on_time ? answer_sum_s / (double)answers_on_time * 1e6 : 0.0,
                answer_max_s * 1e6);
    }

    if (transport.app_exit) {
        transport.app_exit();
    }
    if (generic.app_exit) {
        generic.app_exit();
    }
    hm2_shim_flush_log();
    log_from_ring();
    hm2_region_destroy(&region);
    free(bound_inputs);
    free(bound_outputs);
    free(param_views);
    free(fixed_pins);
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
    log_from_ring();
    free(bound_inputs);
    free(bound_outputs);
    free(fixed_pins);
    hm2_shim_fini();
    hm2_config_free(&config);
    return 1;
}
