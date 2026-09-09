/*
 * Between the shim's own translation units. Not part of the host's interface;
 * that is hm2_shim.h.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (c) 2026 the driver-hostmot2 contributors
 */
#ifndef HM2_SHIM_INTERNAL_H
#define HM2_SHIM_INTERNAL_H

#include <stddef.h>

#include "hm2_shim.h"

/* One diagnostic line, at an RTAPI message level. printf-style. */
void hm2_shim_log(int level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/*
 * A parameter value from the host's configuration file, by HAL name.
 * Returns 1 and fills `out` if the file set it, 0 if it did not.
 */
int hm2_shim_param_lookup(const char *name, double *out);

/* Records a declared parameter, so the host can report the ones nobody set. */
void hm2_shim_note_param(const char *name, hm2_shim_type type, void *cell);

/* Every parameter the driver declared, for the start-up report. */
size_t hm2_shim_declared_param_count(void);
const char *hm2_shim_declared_param_at(size_t index, double *value);


#endif /* HM2_SHIM_INTERNAL_H */
