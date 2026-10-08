/*
 * The Smart Serial fault watch, checked without a board: which pins it takes
 * for a port's fault count, and that each rise of a count is one fault and
 * a fall -- the driver's decay -- is none.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <rtapi.h>
#include <hal.h>

#include "hm2_host.h"

static int logged;
static char last_line[1024];

/* hm2-host's log, which the watch writes its faults to. */
void hm2_log(int level, const char *fmt, ...) {
    (void)level;
    va_list args;
    va_start(args, fmt);
    vsnprintf(last_line, sizeof(last_line), fmt, args);
    va_end(args);
    printf("%s\n", last_line);
    logged++;
}

static int failures;

static void expect(int ok, const char *what) {
    if (!ok) {
        fprintf(stderr, "failed: %s\n", what);
        failures++;
    }
}

int main(void) {
    expect(sserial_port_of("hm2_7i92.0.sserial.port-1.fault-count") == 1, "port 1 by name");
    expect(sserial_port_of("hm2_7i92.0.sserial.port-0.fault-count") == 0, "port 0 by name");
    expect(sserial_port_of("hm2_7i92.0.sserial.port-0.port_state") == -1, "not a fault count");
    expect(sserial_port_of("hm2_7i92.0.encoder.00.fault-count") == -1, "not a port");

    if (hm2_shim_init(1 << 16, 16, 4) != 0) {
        return 1;
    }
    int comp = hal_init("t");
    hal_uint_t port0;
    hal_uint_t port1;
    hal_uint_t other;
    if (comp < 0 ||
        hal_pin_new_ui32(comp, HAL_OUT, &port0, 0, "t.sserial.port-0.fault-count") ||
        hal_pin_new_ui32(comp, HAL_OUT, &port1, 0, "t.sserial.port-1.fault-count") ||
        hal_pin_new_ui32(comp, HAL_OUT, &other, 0, "t.sserial.port-1.port_state")) {
        fprintf(stderr, "the pins were not created\n");
        return 1;
    }
    for (size_t i = 0; i < hm2_shim_signal_count(); i++) {
        sserial_watch_add(hm2_shim_signal_at(i));
    }
    expect(sserial_watched() == 2, "both ports' fault counts watched, nothing else");

    const hm2_shim_signal *count0 = hm2_shim_signal_at(0);
    const hm2_shim_signal *count1 = hm2_shim_signal_at(1);
    uint64_t faults = 0;
    /* A period that answered in time, and one whose core did not. */
    const hm2_period quiet = {.wake_late_s = 0.000002, .read_s = 0.0003, .answer_s = 0.00004};
    const hm2_period late = {.wake_late_s = 0.000002, .read_s = 0.0003, .answer_s = -1.0};
    faults += sserial_faults_seen(1, 0.0009, &quiet, 0.0, faults);
    expect(faults == 0, "no fault while nothing rose");

    /* Both ports fault in one cycle, as on the WF41C at 00:01:00. */
    hm2_shim_cell_set(count0, 10);
    hm2_shim_cell_set(count1, 10);
    faults += sserial_faults_seen(2, 0.00015, &late, 0.0, faults);
    expect(faults == 2, "one fault per port that rose");
    expect(strstr(last_line, "ended 150 us before the read") != NULL, "the gap is said");
    expect(strstr(last_line, "read took 300 us and the core had not answered by the send "
                             "deadline") != NULL,
           "the period before is said, its late answer among it");

    /* The driver's decay is no fault. */
    for (int n = 9; n >= 0; n--) {
        hm2_shim_cell_set(count0, n);
        hm2_shim_cell_set(count1, n);
        faults += sserial_faults_seen(3, 0.0009, &quiet, 0.0, faults);
    }
    expect(faults == 2, "a falling count is no fault");

    /* A rise on a count that had not decayed to zero is still one. */
    hm2_shim_cell_set(count1, 10);
    faults += sserial_faults_seen(4, 0.0009, &quiet, 0.0, faults);
    hm2_shim_cell_set(count1, 19);
    faults += sserial_faults_seen(5, 0.0009, &quiet, 0.0, faults);
    expect(faults == 4, "every rise is one fault");
    expect(logged == 4, "every fault logged");

    hm2_shim_fini();
    if (failures) {
        return 1;
    }
    printf("the fault watch counts each rise of a port's fault count, and nothing else\n");
    return 0;
}
