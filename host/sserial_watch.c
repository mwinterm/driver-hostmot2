/*
 * Each Smart Serial port's fault count, watched for faults (the control's
 * ADR 0045 follow-up).
 *
 * The driver adds `fault-inc` to a port's `fault-count` when the port's
 * transfer had not finished by the next read -- "DoIt not cleared from
 * previous servo thread" -- or failed, takes `fault-dec` off it every good
 * cycle, and stops the port past `fault-lim`. It says so in words once, at
 * the fourth fault, and never again, and the count decays to nothing within
 * a few cycles: how often a machine's ports fault cannot be read from it.
 * So the count is read after every write, where the driver moves it, and
 * each rise is one fault -- counted, published as `hm2-host.sserial-faults`
 * and logged with what the period looked like.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 the driver-hostmot2 contributors
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hm2_host.h"
#include "hm2_shim.h"

#define MAX_SSERIAL_PORTS 8

/*
 * Logged one by one up to this many, then every this-many-th, so a port
 * that fails every cycle does not fill the log.
 */
#define SSERIAL_FAULTS_LOGGED 100

static const hm2_shim_signal *pins[MAX_SSERIAL_PORTS];
static int ports[MAX_SSERIAL_PORTS];
static double last[MAX_SSERIAL_PORTS];
static size_t watched;

/* The port's number in a pin named `....sserial.port-N.fault-count`. */
int sserial_port_of(const char *name) {
    static const char suffix[] = ".fault-count";
    static const char port[] = ".sserial.port-";
    size_t length = strlen(name);
    size_t tail = sizeof(suffix) - 1;
    if (length <= tail || strcmp(name + length - tail, suffix) != 0) {
        return -1;
    }
    const char *at = strstr(name, port);
    if (!at) {
        return -1;
    }
    return atoi(at + sizeof(port) - 1);
}

int sserial_watch_add(const hm2_shim_signal *signal) {
    int port = sserial_port_of(signal->name);
    if (port < 0 || watched >= MAX_SSERIAL_PORTS) {
        return 0;
    }
    pins[watched] = signal;
    ports[watched] = port;
    last[watched] = hm2_shim_cell_get(signal);
    watched++;
    return 1;
}

size_t sserial_watched(void) { return watched; }

double sserial_transfer_wait(double since_write_s, double transfer_s) {
    return since_write_s < transfer_s ? transfer_s - since_write_s : 0.0;
}

uint64_t sserial_faults_seen(uint64_t cycle, double gap_s, const hm2_period *before,
                             double wake_late_s, uint64_t so_far) {
    uint64_t seen = 0;
    for (size_t i = 0; i < watched; i++) {
        double count = hm2_shim_cell_get(pins[i]);
        if (count > last[i]) {
            seen++;
            uint64_t number = so_far + seen;
            if (number <= SSERIAL_FAULTS_LOGGED || number % SSERIAL_FAULTS_LOGGED == 0) {
                char answered[64];
                if (before->answer_s > 0.0) {
                    snprintf(answered, sizeof(answered), "answered %.0f us after the publish",
                             before->answer_s * 1e6);
                } else if (before->answer_s < 0.0) {
                    snprintf(answered, sizeof(answered),
                             "had not answered by the send deadline");
                } else {
                    snprintf(answered, sizeof(answered), "was not waited for");
                }
                hm2_log(HM2_LOG_WARN,
                        "Smart Serial port %d: fault %llu, cycle %llu: its transfer had not "
                        "finished at this period's read, or failed. The previous write ended "
                        "%.0f us before the read. The period before woke %.0f us late, its "
                        "read took %.0f us, the core %s and its write took %.0f us; this one "
                        "woke %.0f us late. Fault count %.0f%s",
                        ports[i], (unsigned long long)number, (unsigned long long)cycle,
                        gap_s * 1e6, before->wake_late_s * 1e6, before->read_s * 1e6, answered,
                        before->write_s * 1e6, wake_late_s * 1e6, count,
                        number == SSERIAL_FAULTS_LOGGED ? "; from here every 100th is logged"
                                                        : "");
            }
        }
        last[i] = count;
    }
    return seen;
}
