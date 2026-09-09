/* Stand-in for LinuxCNC's autoconf-generated config.h.
 *
 * The extracted driver includes it for one symbol: USPACE_XENOMAI_EVL, which
 * selects the EVL (Xenomai 4) socket path in hm2_eth.c. This host is plain
 * POSIX -- PREEMPT_RT, not Xenomai -- so the symbol stays undefined and the
 * ordinary path is taken.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef CNC_HM2_CONFIG_H
#define CNC_HM2_CONFIG_H
#endif
