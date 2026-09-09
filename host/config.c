/*
 * The host's configuration file (ADR 0022 §4, §5).
 *
 * Flat `key = value` lines, `#` to end of line for a comment. A parser rather
 * than a dependency: there are fifteen keys and two namespaces, and a YAML
 * library in a real-time process is a library to audit for allocations.
 *
 * WHAT THIS FILE IS FOR, AND WHAT IT IS NOT
 *   It says what the *hardware* is: which board, at which address, with which
 *   modules configured, and what an encoder counts in. It says nothing about
 *   what any of it means to a machine -- which pin is an axis's feedback, what
 *   an axis's limits are, which way is X. That is the machine description, on
 *   the other side of the channel, and it stays there (ADR 0022 §5). Two files
 *   that both described the machine would be two files to disagree.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 the driver-hostmot2 contributors
 */
#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hm2_host.h"

static void trim(char *text) {
    char *start = text;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }
    if (start != text) {
        memmove(text, start, strlen(start) + 1);
    }
    size_t length = strlen(text);
    while (length > 0 && isspace((unsigned char)text[length - 1])) {
        text[--length] = '\0';
    }
}

/*
 * Splits `name[3]` into a name and an index.
 *
 * Array module parameters are how a LinuxCNC configuration names several
 * boards -- `board_ip=10.10.10.10 board_ip=10.10.10.11` on one `loadrt` line
 * becomes two elements. A key without brackets is element zero, which is the
 * scalar case and the common one.
 */
static size_t split_index(char *name) {
    char *bracket = strchr(name, '[');
    if (!bracket) {
        return 0;
    }
    *bracket = '\0';
    return (size_t)strtoul(bracket + 1, NULL, 10);
}

static int add_module_param(hm2_config *config, char *name, const char *value) {
    size_t index = split_index(name);
    /*
     * Refused rather than truncated, for the reason a pin name is: two keys
     * cut to the same bytes would be two settings that silently became one,
     * and the second would win with nothing saying so.
     */
    if (strlen(name) >= sizeof(config->module_params->name) ||
        strlen(value) >= sizeof(config->module_params->value)) {
        hm2_log(HM2_LOG_ERROR, "module.%s: the name or the value is too long", name);
        return -1;
    }
    struct hm2_module_param *grown =
        realloc(config->module_params, (config->module_param_count + 1) * sizeof(*grown));
    if (!grown) {
        return -1;
    }
    config->module_params = grown;
    struct hm2_module_param *slot = &grown[config->module_param_count++];
    snprintf(slot->name, sizeof(slot->name), "%s", name);
    slot->index = index;
    snprintf(slot->value, sizeof(slot->value), "%s", value);
    return 0;
}

static int add_hal_param(hm2_config *config, const char *name, double value) {
    const char **names =
        realloc((void *)config->param_names, (config->param_count + 1) * sizeof(*names));
    if (!names) {
        return -1;
    }
    config->param_names = names;
    double *values = realloc(config->param_values, (config->param_count + 1) * sizeof(*values));
    if (!values) {
        return -1;
    }
    config->param_values = values;
    char *copy = strdup(name);
    if (!copy) {
        return -1;
    }
    config->param_names[config->param_count] = copy;
    config->param_values[config->param_count] = value;
    config->param_count++;
    return 0;
}

int hm2_config_load(hm2_config *config, const char *path) {
    memset(config, 0, sizeof(*config));

    /*
     * Defaults, and each is a decision rather than a round number.
     *
     * `cycle_us` at 1000 is what ADR 0022 §7 calls a 1 ms-class interface: one
     * LBP16 round trip over UDP per cycle. `worst_case_cycle_ns` is what this
     * process tells the core it needs, and the core refuses a machine whose
     * cycle it does not fit -- so it is deliberately not the same number as
     * the cycle, and a bench measures it rather than this file guessing.
     *
     * The watchdogs are ADR 0011's: the core gives up on a channel after
     * `core_watchdog_cycles`, this process takes the drives down after
     * `response_watchdog_cycles` unanswered, and `drive_watchdog_us` is the
     * stop of last resort the FPGA's own watchdog provides -- ten times the
     * cycle, which is where ADR 0011's follow-up said to start from.
     */
    snprintf(config->region, sizeof(config->region), "%s", "/cnc-hm2-0");
    config->axis_count = 3;
    config->cycle_us = 1000;
    config->core_watchdog_cycles = 100;
    config->response_watchdog_cycles = 100;
    config->drive_watchdog_us = 10000;
    config->spin_iterations = 0;
    config->worst_case_cycle_ns = 800000;
    config->send_deadline = 0.8;
    config->arena_bytes = 8u * 1024u * 1024u;
    config->max_signals = 4096;
    config->max_functs = 64;
    config->log_level = HM2_LOG_INFO;
    /* Above the core's servo thread, because this process is the timebase and
       being preempted by the thread it releases makes the cycle late by
       definition (ADR 0011). The core's servo thread runs at 90. */
    config->rt_priority = 92;
    config->cpu_affinity = -1;

    FILE *file = fopen(path, "r");
    if (!file) {
        hm2_log(HM2_LOG_ERROR, "could not read the configuration %s: %s", path,
                strerror(errno));
        return -1;
    }

    char line[1024];
    int number = 0;
    int failures = 0;
    while (fgets(line, sizeof(line), file)) {
        number++;
        char *comment = strchr(line, '#');
        if (comment) {
            *comment = '\0';
        }
        trim(line);
        if (line[0] == '\0') {
            continue;
        }
        char *equals = strchr(line, '=');
        if (!equals) {
            hm2_log(HM2_LOG_ERROR, "%s:%d: '%s' is not a key = value line", path, number, line);
            failures++;
            continue;
        }
        *equals = '\0';
        char *key = line;
        char *value = equals + 1;
        trim(key);
        trim(value);

        if (strncmp(key, "module.", 7) == 0) {
            if (add_module_param(config, key + 7, value) != 0) {
                failures++;
            }
        } else if (strncmp(key, "param.", 6) == 0) {
            if (add_hal_param(config, key + 6, strtod(value, NULL)) != 0) {
                failures++;
            }
        } else if (strcmp(key, "region") == 0) {
            snprintf(config->region, sizeof(config->region), "%s", value);
        } else if (strcmp(key, "axes") == 0) {
            config->axis_count = (uint32_t)strtoul(value, NULL, 0);
        } else if (strcmp(key, "cycle_us") == 0) {
            config->cycle_us = (uint32_t)strtoul(value, NULL, 0);
        } else if (strcmp(key, "worst_case_cycle_ns") == 0) {
            config->worst_case_cycle_ns = strtoull(value, NULL, 0);
        } else if (strcmp(key, "core_watchdog_cycles") == 0) {
            config->core_watchdog_cycles = (uint32_t)strtoul(value, NULL, 0);
        } else if (strcmp(key, "response_watchdog_cycles") == 0) {
            config->response_watchdog_cycles = (uint32_t)strtoul(value, NULL, 0);
        } else if (strcmp(key, "drive_watchdog_us") == 0) {
            config->drive_watchdog_us = (uint32_t)strtoul(value, NULL, 0);
        } else if (strcmp(key, "spin_iterations") == 0) {
            config->spin_iterations = (uint32_t)strtoul(value, NULL, 0);
        } else if (strcmp(key, "send_deadline") == 0) {
            config->send_deadline = strtod(value, NULL);
        } else if (strcmp(key, "arena_bytes") == 0) {
            config->arena_bytes = (size_t)strtoull(value, NULL, 0);
        } else if (strcmp(key, "max_signals") == 0) {
            config->max_signals = (size_t)strtoull(value, NULL, 0);
        } else if (strcmp(key, "max_functs") == 0) {
            config->max_functs = (size_t)strtoull(value, NULL, 0);
        } else if (strcmp(key, "log_level") == 0) {
            config->log_level = (int)strtol(value, NULL, 0);
        } else if (strcmp(key, "rt_priority") == 0) {
            config->rt_priority = (int)strtol(value, NULL, 0);
        } else if (strcmp(key, "cpu_affinity") == 0) {
            config->cpu_affinity = (int)strtol(value, NULL, 0);
        } else {
            /*
             * Refused rather than ignored. An unknown key is a misspelling
             * far more often than it is a future feature, and the failure it
             * would otherwise produce is a machine running on a default
             * nobody chose, with nothing anywhere saying so.
             */
            hm2_log(HM2_LOG_ERROR, "%s:%d: '%s' is not a key this host knows", path, number,
                    key);
            failures++;
        }
    }
    fclose(file);

    if (failures > 0) {
        hm2_config_free(config);
        return -1;
    }
    if (config->cycle_us == 0) {
        hm2_log(HM2_LOG_ERROR, "%s: cycle_us is zero", path);
        hm2_config_free(config);
        return -1;
    }
    if (config->send_deadline <= 0.0 || config->send_deadline >= 1.0) {
        hm2_log(HM2_LOG_ERROR,
                "%s: send_deadline is %g; it is a fraction of the period, strictly between "
                "0 and 1",
                path, config->send_deadline);
        hm2_config_free(config);
        return -1;
    }
    return 0;
}

void hm2_config_free(hm2_config *config) {
    free(config->module_params);
    for (size_t i = 0; i < config->param_count; i++) {
        free((void *)config->param_names[i]);
    }
    free((void *)config->param_names);
    free(config->param_values);
    config->module_params = NULL;
    config->param_names = NULL;
    config->param_values = NULL;
    config->module_param_count = 0;
    config->param_count = 0;
}
