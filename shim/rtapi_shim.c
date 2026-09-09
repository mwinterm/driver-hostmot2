/*
 * The RTAPI half of the shim (ADR 0022 §4).
 *
 * RTAPI is LinuxCNC's portability layer over "kernel module or userspace
 * process". This host is a userspace process, so most of it is a thin layer
 * over POSIX: `rtapi_get_time` is `clock_gettime`, `rtapi_delay` is a bounded
 * spin, `rtapi_print_msg` is a line in the host's log.
 *
 * The part that is not thin is the module parameters. `loadrt hm2_eth
 * board_ip=10.10.10.10` is how a LinuxCNC configuration reaches this driver,
 * and the parameters are `static`, so there is no symbol to assign. What there
 * is -- because `RTAPI_MP_*` expands to it -- is a non-static
 * `rtapi_info_address_<name>` holding the address and `rtapi_info_type_<name>`
 * holding the type letter. That is the mechanism LinuxCNC's own loader uses,
 * and using it too is why third_party/linuxcnc/ needs no edit at all.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 the driver-hostmot2 contributors
 */
#include <dlfcn.h>
#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <rtapi.h>
#include <rtapi_firmware.h>
#include <rtapi_string.h>

#include "hm2_shim.h"
#include "hm2_shim_internal.h"

/* ---------------------------------------------------------------------------
 * Diagnostics
 * ------------------------------------------------------------------------- */

static void (*log_sink)(int level, const char *line);
static int msg_level = RTAPI_MSG_INFO;

void hm2_shim_set_log(void (*sink)(int level, const char *line)) { log_sink = sink; }

void hm2_shim_set_msg_level(int level) { msg_level = level; }

static void emit(int level, const char *line) {
    if (log_sink) {
        log_sink(level, line);
    } else {
        fprintf(stderr, "%s\n", line);
    }
}

void hm2_shim_log(int level, const char *fmt, ...) {
    if (level > msg_level) {
        return;
    }
    char line[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    emit(level, line);
}

/*
 * The driver's own diagnostics.
 *
 * It prints with trailing newlines and sometimes in fragments, because in
 * LinuxCNC this reaches a terminal. The host's log is line-oriented, so the
 * fragments are gathered here and flushed on a newline -- otherwise a
 * multi-part message about a board that failed to come up arrives as six lines
 * that each look like a different problem.
 */
static char pending[1024];
static size_t pending_len;
static int pending_level = RTAPI_MSG_ERR;

static void gather(int level, const char *fmt, va_list args) {
    char chunk[1024];
    int written = vsnprintf(chunk, sizeof(chunk), fmt, args);
    if (written < 0) {
        return;
    }
    if (pending_len == 0) {
        pending_level = level;
    }
    for (const char *p = chunk; *p; p++) {
        if (*p == '\n') {
            pending[pending_len] = '\0';
            if (pending_len > 0 && pending_level <= msg_level) {
                emit(pending_level, pending);
            }
            pending_len = 0;
            pending_level = level;
        } else if (pending_len + 1 < sizeof(pending)) {
            pending[pending_len++] = *p;
        }
    }
}

void rtapi_print_msg(msg_level_t level, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    gather(level, fmt, args);
    va_end(args);
}

void rtapi_print(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    gather(RTAPI_MSG_INFO, fmt, args);
    va_end(args);
}

/* Flushes anything the driver left without a newline. Called at shutdown. */
void hm2_shim_flush_log(void) {
    if (pending_len > 0) {
        pending[pending_len] = '\0';
        emit(pending_level, pending);
        pending_len = 0;
    }
}

int rtapi_snprintf(char *buf, unsigned long int size, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(buf, size, fmt, args);
    va_end(args);
    return written;
}

int rtapi_set_msg_level(int level) {
    msg_level = level;
    return 0;
}

int rtapi_get_msg_level(void) { return msg_level; }

/* ---------------------------------------------------------------------------
 * Time
 * ------------------------------------------------------------------------- */

/*
 * Nanoseconds on a clock that does not step.
 *
 * CLOCK_MONOTONIC rather than CLOCK_REALTIME: the driver measures intervals
 * with this -- packet round trips, the time since the last cycle -- and a
 * realtime clock that an NTP step moves backwards would make one of those
 * intervals negative. `clock_gettime` on this is a vDSO call and does not
 * enter the kernel, so it is safe in the cyclic path.
 */
long long int rtapi_get_time(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (long long)now.tv_sec * 1000000000LL + now.tv_nsec;
}

/*
 * Longest delay `rtapi_delay` will actually take.
 *
 * The driver reads this to decide how to break a long wait into short ones.
 * It is a spin, so the figure is a promise about how long this will hold a
 * real-time thread rather than a property of any hardware.
 */
long int rtapi_delay_max(void) { return 100000; }

/*
 * A short busy wait.
 *
 * A spin rather than `nanosleep`, deliberately: the driver calls this inside
 * its cyclic functions, between two register writes that need a settling time
 * between them, and a sleep there would hand the CPU to something else and
 * come back late. Bounded by `rtapi_delay_max` so a caller cannot spin
 * unboundedly by accident.
 */
void rtapi_delay(long int nsec) {
    if (nsec <= 0) {
        return;
    }
    if (nsec > rtapi_delay_max()) {
        nsec = rtapi_delay_max();
    }
    long long until = rtapi_get_time() + nsec;
    while (rtapi_get_time() < until) {
        /* Politely, on a hyperthreaded core. */
#if defined(__x86_64__) || defined(__i386__)
        __builtin_ia32_pause();
#elif defined(__aarch64__)
        __asm__ __volatile__("yield");
#endif
    }
}

/*
 * Which task is running. There is one, and it is the cycle thread.
 *
 * The driver uses it only to tell "in a cyclic function" from "not", which
 * this host knows for certain by other means, so a constant is honest here in
 * a way it would not be under a real RTAPI.
 */
int rtapi_task_self(void) { return 0; }

/* ---------------------------------------------------------------------------
 * Argument splitting
 * ------------------------------------------------------------------------- */

/*
 * Splits a config string the way the kernel's `argv_split` does. The driver
 * uses it on the `config=` module parameter -- "num_encoders=3 num_pwmgens=3"
 * -- so this must split on whitespace and nothing cleverer.
 */
char **rtapi_argv_split(rtapi_gfp_t gfp_mask, const char *str, int *argcp) {
    (void)gfp_mask;
    if (!str) {
        return NULL;
    }
    size_t length = strlen(str);
    /* At most one argument per two characters, plus the NULL terminator. */
    char **argv = calloc(length / 2 + 2, sizeof(*argv));
    char *copy = strdup(str);
    if (!argv || !copy) {
        free(argv);
        free(copy);
        return NULL;
    }
    int argc = 0;
    for (char *token = strtok(copy, " \t\n\r"); token; token = strtok(NULL, " \t\n\r")) {
        argv[argc++] = token;
    }
    argv[argc] = NULL;
    if (argcp) {
        *argcp = argc;
    }
    /*
     * `argv[0]` points into `copy`, so freeing it frees the strings too --
     * which is what `rtapi_argv_free` does, and why it must not free them
     * individually. An empty string leaves argv[0] NULL and leaks nothing,
     * because `copy` is freed here.
     */
    if (argc == 0) {
        free(copy);
    }
    return argv;
}

void rtapi_argv_free(char **argv) {
    if (!argv) {
        return;
    }
    /* One allocation held the lot; see `rtapi_argv_split`. */
    free(argv[0]);
    free(argv);
}

/* ---------------------------------------------------------------------------
 * Firmware
 * ------------------------------------------------------------------------- */

/*
 * Reads a bitfile off disk so the driver can program an FPGA from it.
 *
 * Only the boards that are configured over their own bus need this -- a 7i92
 * arrives with its firmware already in flash. It is here so that a board that
 * does need it fails on a missing file rather than on a missing symbol.
 */
int rtapi_request_firmware(const struct rtapi_firmware **fw, const char *name,
                           struct rtapi_device *device) {
    (void)device;
    if (!fw || !name) {
        return -EINVAL;
    }
    *fw = NULL;

    const char *root = getenv("HM2_FIRMWARE_PATH");
    char path[PATH_MAX];
    snprintf(path, sizeof(path), "%s/%s", root ? root : "/lib/firmware", name);

    FILE *file = fopen(path, "rb");
    if (!file) {
        hm2_shim_log(RTAPI_MSG_ERR, "shim: no firmware at %s: %s. Set HM2_FIRMWARE_PATH "
                                    "if it lives somewhere else",
                     path, strerror(errno));
        return -ENOENT;
    }
    if (fseek(file, 0, SEEK_END) != 0) {
        fclose(file);
        return -EIO;
    }
    long size = ftell(file);
    rewind(file);
    if (size < 0) {
        fclose(file);
        return -EIO;
    }

    struct rtapi_firmware *loaded = calloc(1, sizeof(*loaded));
    unsigned char *data = malloc((size_t)size ? (size_t)size : 1);
    if (!loaded || !data || fread(data, 1, (size_t)size, file) != (size_t)size) {
        free(loaded);
        free(data);
        fclose(file);
        return -EIO;
    }
    fclose(file);
    loaded->size = (size_t)size;
    loaded->data = data;
    *fw = loaded;
    hm2_shim_log(RTAPI_MSG_INFO, "shim: loaded %ld bytes of firmware from %s", size, path);
    return 0;
}

void rtapi_release_firmware(const struct rtapi_firmware *fw) {
    if (!fw) {
        return;
    }
    free((void *)fw->data);
    free((void *)fw);
}

/* ---------------------------------------------------------------------------
 * Module parameters
 * ------------------------------------------------------------------------- */

/*
 * Sets one, by the name a LinuxCNC configuration would use.
 *
 * `rtapi_info_address_<name>` and `rtapi_info_type_<name>` are emitted beside
 * every `RTAPI_MP_*` declaration and are not static, so they can be looked up
 * even though the parameter itself is. `dlsym(RTLD_DEFAULT, ...)` finds them
 * in this executable, which is why the host is linked with `-rdynamic`: it is
 * looking up its own symbols.
 *
 * The type letter is the kernel's: "i" an int, "l" a long, "s" a string.
 */
int hm2_shim_set_module_param(const char *name, size_t index, const char *value) {
    if (!name || !value) {
        return -1;
    }
    char symbol[256];

    snprintf(symbol, sizeof(symbol), "rtapi_info_address_%s", name);
    void **address = dlsym(RTLD_DEFAULT, symbol);
    snprintf(symbol, sizeof(symbol), "rtapi_info_type_%s", name);
    const char **type = dlsym(RTLD_DEFAULT, symbol);

    if (!address || !type || !*address || !*type) {
        hm2_shim_log(RTAPI_MSG_ERR,
                     "shim: the driver has no module parameter '%s'. These are the names a "
                     "LinuxCNC `loadrt` line would use -- board_ip, config, debug",
                     name);
        return -1;
    }

    /* An array parameter's size, where the declaration recorded one. */
    snprintf(symbol, sizeof(symbol), "rtapi_info_size_%s", name);
    const int *size = dlsym(RTLD_DEFAULT, symbol);
    if (size && index >= (size_t)*size) {
        hm2_shim_log(RTAPI_MSG_ERR,
                     "shim: module parameter '%s' has %d element(s) and %zu was asked for",
                     name, *size, index);
        return -1;
    }
    if (!size && index != 0) {
        hm2_shim_log(RTAPI_MSG_ERR, "shim: module parameter '%s' is not an array", name);
        return -1;
    }

    switch (**type) {
    case 'i': {
        int *slot = (int *)*address;
        slot[index] = (int)strtol(value, NULL, 0);
        break;
    }
    case 'l': {
        long *slot = (long *)*address;
        slot[index] = strtol(value, NULL, 0);
        break;
    }
    case 's': {
        char **slot = (char **)*address;
        /* Never freed, and never has to be: it lives as long as the driver. */
        slot[index] = strdup(value);
        if (!slot[index]) {
            return -1;
        }
        break;
    }
    default:
        hm2_shim_log(RTAPI_MSG_ERR, "shim: module parameter '%s' has type '%s', which this "
                                    "shim does not set",
                     name, *type);
        return -1;
    }
    hm2_shim_log(RTAPI_MSG_INFO, "shim: module parameter %s[%zu] = %s", name, index, value);
    return 0;
}

/*
 * The kernel's name for `strtol`.
 *
 * `rtapi.h` declares it in userspace because the driver was written against
 * the kernel's string helpers and RTAPI does not paper over this one. In the
 * kernel it differs from libc's in its error handling -- it has none -- which
 * does not matter here: the driver calls it on strings it has already checked.
 */
long int simple_strtol(const char *nptr, char **endptr, int base) {
    return strtol(nptr, endptr, base);
}
