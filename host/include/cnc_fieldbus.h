/*
 * Open CNC Control — the cyclic channel between the core and the fieldbus
 * process (ADR 0011, specification §5.2, §5.3, §9.9, §10.3).
 *
 * THIS HEADER IS THE SOURCE OF TRUTH. The Rust bindings in this crate are
 * generated from it by bindgen at build time; they are never edited by hand.
 *
 * ADR 0004 put the IgH EtherCAT master in its own process so that its GPL
 * never reaches the core, which makes the layout below a contract between two
 * separately built and separately licensed programs rather than an internal
 * detail. ADR 0009 made the fieldbus process the timebase: it wakes on the
 * DC-disciplined bus cycle and releases the core, rather than being called
 * by it.
 *
 * REAL-TIME CONTRACT (§5.3)
 *   Both sides map and lock this region before either enters its cyclic path.
 *   Nothing here is allocated, grown or remapped while running. The exchange
 *   never blocks and never retries in either direction: the core reads the
 *   newest published cycle, and the fieldbus sends whatever the core has
 *   finished, saying whether that was this cycle's work.
 *
 * WHAT CROSSES THIS BOUNDARY
 *   SI units throughout, per ground rule 3: meters, radians, seconds. Encoder
 *   scaling, the CiA 402 control and status words and the drive state machine
 *   all live inside the fieldbus process, which is the only side that knows
 *   what is on the wire. The core sees axes, not slaves.
 *
 * LICENSE (ADR 0022 §3)
 *   This file is Apache-2.0 OR MIT, and it is the only file in this repository
 *   that is. The reason is the process on the other side: ADR 0004 puts the
 *   IgH EtherCAT master there and ADR 0022 puts the HostMot2 driver there, and
 *   both are GPL-2.0. Apache-2.0 is not GPL-2.0-compatible in the FSF's
 *   reading, so a GPL-2.0 program could not compile against this header as it
 *   stood -- which is to say the boundary those ADRs designed could not be
 *   crossed by the programs it was designed for. MIT can be.
 *
 *   It also stands alone: it includes nothing of this project's, so vendoring
 *   it into the other repository is copying one file. CNC_FIELDBUS_MAX_AXES
 *   below is CNC_MAX_JOINTS from cnc_abi.h, copied for that reason and checked
 *   against the original in this crate's tests.
 *
 *   Nothing else changes license. `cnc-fieldbus` (the Rust channel) and
 *   `cnc-driver-fieldbus` (the plugin) stay Apache-2.0, because the process on
 *   the other side never links them: it lays out the same bytes from this
 *   file, which is the whole point of a header being the source of truth.
 *
 * Changing this file requires a version bump below plus an ADR or changelog
 * entry (see CLAUDE.md).
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */
#ifndef CNC_FIELDBUS_H
#define CNC_FIELDBUS_H

#include <stdalign.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Version
 * ------------------------------------------------------------------------- */

/*
 * Channel version, semantic. Each side refuses a region whose major differs
 * from its own: the layout is mapped, not parsed, so a mismatch is not
 * something either end can work around.
 *
 * Appending a field to the end of an axis structure is a major bump here, not
 * a minor one, because the arrays are indexed by offset on both sides. Only
 * changes that leave every existing offset alone are minor.
 */
#define CNC_FIELDBUS_ABI_VERSION_MAJOR 1
#define CNC_FIELDBUS_ABI_VERSION_MINOR 1
#define CNC_FIELDBUS_ABI_VERSION_PATCH 0

/*
 * 1.1.0 (ADR 0022 §3) added named signals: a descriptor table and two value
 * blocks, appended after the fixed layout below and published by the same two
 * counters. Every offset a 1.0.0 process computes is where it was, which is
 * what makes this a minor bump under the rule above. A 1.0.0 process declares
 * no signals and is driven unchanged; a 1.1.0 process with nothing but axes to
 * report writes a region of exactly the 1.0.0 size.
 *
 * The reason it exists: what the channel carried was axes and nothing else, so
 * a driver process with digital I/O, an analog output or an index latch to
 * report had nowhere to put them. A HostMot2 card has hundreds of such pins.
 */

/*
 * Axes the region carries. Fixed rather than sized from the machine
 * description: a compile-time layout is one both sides compute identically
 * without agreeing on an arithmetic, which is worth far more than the few
 * kilobytes it costs. `axis_count` below says how many are in use.
 *
 * Matches CNC_MAX_JOINTS from cnc_abi.h so an axis that fits the kinematics
 * fits the wire. Written out rather than included so this file stands alone
 * for the process on the other side (see LICENSE above); the two are asserted
 * equal in cnc-fieldbus-abi's tests, which is where a divergence is caught.
 */
#define CNC_FIELDBUS_MAX_AXES 32

/* Cache line both sides align to. See the padding note on cnc_fieldbus_shm. */
#define CNC_FIELDBUS_CACHE_LINE 64

/* -------------------------------------------------------------------------
 * Per-axis payloads
 * ------------------------------------------------------------------------- */

/*
 * What a drive reports. Linear axes are meters and meters per second; rotary
 * axes are radians and radians per second (§7.4).
 */
typedef struct cnc_fieldbus_axis_input {
  double position;
  double velocity;
  /* Newton-meters, or newtons on a linear axis. Zero where not reported. */
  double torque;
  /* cnc_fieldbus_axis_state. */
  uint32_t state;
  /* Drive-specific fault code, passed through for diagnostics. 0 = none. */
  uint32_t fault_code;
} cnc_fieldbus_axis_input;

/*
 * What the core commands. Position is the setpoint for cyclic synchronous
 * position mode; velocity is feed-forward, which a drive may ignore.
 */
typedef struct cnc_fieldbus_axis_output {
  double position_cmd;
  double velocity_ff;
  /* cnc_fieldbus_axis_request. */
  uint32_t request;
  uint32_t reserved;
} cnc_fieldbus_axis_output;

/*
 * Where a drive is in its state machine, as the core needs to see it. The CiA
 * 402 states themselves stay inside the fieldbus process; these are the
 * distinctions the core acts on.
 */
typedef enum cnc_fieldbus_axis_state {
  /* Powered down. The axis will not move. */
  CNC_FIELDBUS_AXIS_DISABLED = 0,
  /* Enabled and following setpoints. */
  CNC_FIELDBUS_AXIS_ENABLED = 1,
  /* Faulted. It stopped on its own and needs a reset before it will enable. */
  CNC_FIELDBUS_AXIS_FAULT = 2,
  /* Between states: a transition the drive has not finished. */
  CNC_FIELDBUS_AXIS_TRANSITION = 3
} cnc_fieldbus_axis_state;

/*
 * What the core is asking of a drive. The fieldbus process owns CiA 402 and
 * turns these into the transitions that reach it; the core says what it wants,
 * not how to get there.
 */
typedef enum cnc_fieldbus_axis_request {
  CNC_FIELDBUS_REQUEST_DISABLE = 0,
  CNC_FIELDBUS_REQUEST_ENABLE = 1,
  /* Clear a fault. Rising-edge: the drive enables only on a later ENABLE. */
  CNC_FIELDBUS_REQUEST_RESET_FAULT = 2,
  /* Decelerate on the drive's own quick-stop ramp and hold. */
  CNC_FIELDBUS_REQUEST_QUICK_STOP = 3
} cnc_fieldbus_axis_request;

/* -------------------------------------------------------------------------
 * Whole-cycle payloads
 * ------------------------------------------------------------------------- */

/* How the bus itself is doing, published with every cycle. */
typedef enum cnc_fieldbus_bus_state {
  /* Not yet cycling: slaves are being brought up. */
  CNC_FIELDBUS_BUS_INIT = 0,
  /* Cycling, but the distributed clocks have not settled. */
  CNC_FIELDBUS_BUS_SYNCING = 1,
  /* Cycling with the clocks locked. The only state motion is allowed in. */
  CNC_FIELDBUS_BUS_OPERATIONAL = 2,
  /* A slave dropped out or the bus failed. */
  CNC_FIELDBUS_BUS_FAULT = 3
} cnc_fieldbus_bus_state;

typedef struct cnc_fieldbus_input {
  /* Distributed-clock time this cycle was sampled at [ns]. */
  uint64_t timestamp_ns;
  /*
   * Worst sync-monitor difference seen since the last cycle [ns].
   *
   * ADR 0009's follow-up asks for DC lock to be visible from the first cycle
   * rather than assumed; this is where it arrives, so it reaches the core's
   * diagnostics without a second mechanism.
   */
  int64_t dc_difference_ns;
  /* cnc_fieldbus_bus_state. */
  uint32_t bus_state;
  /* Slaves in OP, against the number configured. Both zero before INIT ends. */
  uint32_t slaves_operational;
  uint32_t slaves_configured;
  uint32_t reserved;
  cnc_fieldbus_axis_input axes[CNC_FIELDBUS_MAX_AXES];
} cnc_fieldbus_input;

typedef struct cnc_fieldbus_output {
  cnc_fieldbus_axis_output axes[CNC_FIELDBUS_MAX_AXES];
} cnc_fieldbus_output;

/* -------------------------------------------------------------------------
 * Named signals (1.1.0, ADR 0022 §3)
 * -------------------------------------------------------------------------
 *
 * The fixed part above carries axes and nothing else, which is all an
 * EtherCAT master with CiA 402 drives has to say. A driver process on a
 * general-purpose I/O card has more: digital inputs and outputs, analog
 * outputs, index latches, a spindle encoder, hundreds of pins on sserial
 * remotes. Those arrive here, as a flat table of named values.
 *
 * The shape is the driver ABI's (`cnc_signal_desc` in cnc_driver.h) with the
 * same enumerator values, so `cnc-driver-fieldbus` passes a signal through
 * rather than translating it -- the same relationship the axis state and
 * request enums above have with that ABI. It is restated here rather than
 * included because this file stands alone (see LICENSE at the top).
 *
 * WHAT A NAME MEANS
 *   Nothing, to this channel. The process publishes each signal under its own
 *   name -- for HostMot2 that is the HAL pin name, `hm2_7i92.0.7i77.0.1.
 *   analogout0` -- and what it *means* is said in the machine description, as
 *   it is for every driver (§9.9). The channel carries the name so that the
 *   two ends can agree on which value is which without agreeing on an order.
 *
 * WHY EVERY VALUE IS A DOUBLE
 *   One stride, one alignment, one layout for both sides to compute, and no
 *   packing rule to disagree about. A bool is 0.0 or 1.0 and an int32 is
 *   exact in a double, so `type` says how to present the value, not how to
 *   find it. The PLC image on the bus is a double per signal for the same
 *   reason (ADR 0019 §6).
 */

/*
 * Longest signal name, NUL included. A HostMot2 sserial pin name -- board,
 * instance, port, channel, pin -- is the long case, and 64 holds it with room.
 * A name that does not fit is a signal the process must not declare: silently
 * truncating two names to the same bytes would map one pin onto another.
 */
#define CNC_FIELDBUS_SIGNAL_NAME_MAX 64

typedef enum cnc_fieldbus_signal_direction {
  /* The driver process publishes it: feedback, inputs, status. */
  CNC_FIELDBUS_SIGNAL_INPUT = 0,
  /* The core writes it: setpoints, outputs, control. */
  CNC_FIELDBUS_SIGNAL_OUTPUT = 1
} cnc_fieldbus_signal_direction;

/* How to present the value. Every value is a double on the wire regardless. */
typedef enum cnc_fieldbus_signal_type {
  CNC_FIELDBUS_SIGNAL_BOOL = 0,
  CNC_FIELDBUS_SIGNAL_I32 = 1,
  CNC_FIELDBUS_SIGNAL_F64 = 2
} cnc_fieldbus_signal_type;

/* What the signal is for. Values are cnc_signal_role's, from cnc_driver.h. */
typedef enum cnc_fieldbus_signal_role {
  CNC_FIELDBUS_ROLE_DIGITAL = 0,
  CNC_FIELDBUS_ROLE_ANALOG = 1,
  CNC_FIELDBUS_ROLE_POSITION = 2,
  CNC_FIELDBUS_ROLE_VELOCITY = 3,
  CNC_FIELDBUS_ROLE_TORQUE = 4,
  CNC_FIELDBUS_ROLE_STATUS_WORD = 5,
  CNC_FIELDBUS_ROLE_CONTROL_WORD = 6,
  CNC_FIELDBUS_ROLE_ENCODER_COUNT = 7,
  CNC_FIELDBUS_ROLE_PROBE_LATCH = 8,
  /*
   * A digital output the process promises to put on the wire in the cycle it
   * is written, never buffered to the next (ADR 0017 §8). Across this channel
   * that promise costs a cycle of latency the process must account for in the
   * worst-case cycle time it declares; a process that cannot keep it must not
   * claim this role.
   */
  CNC_FIELDBUS_ROLE_FAST_DIGITAL = 9
} cnc_fieldbus_signal_role;

/* SI units, per ground rule 3. Values are cnc_unit's, from cnc_abi.h. */
typedef enum cnc_fieldbus_unit {
  CNC_FIELDBUS_UNIT_NONE = 0,
  CNC_FIELDBUS_UNIT_METRE = 1,
  CNC_FIELDBUS_UNIT_RADIAN = 2,
  CNC_FIELDBUS_UNIT_METRE_PER_SECOND = 3,
  CNC_FIELDBUS_UNIT_RADIAN_PER_SECOND = 4,
  CNC_FIELDBUS_UNIT_NEWTON = 5,
  CNC_FIELDBUS_UNIT_NEWTON_METRE = 6,
  CNC_FIELDBUS_UNIT_VOLT = 7,
  CNC_FIELDBUS_UNIT_AMPERE = 8,
  CNC_FIELDBUS_UNIT_SECOND = 9,
  CNC_FIELDBUS_UNIT_COUNT = 10
} cnc_fieldbus_unit;

/*
 * One signal. Written once by the driver process before it starts cycling and
 * read-only thereafter, exactly like cnc_fieldbus_config: the table describes
 * the region, so it cannot be something either side may change while the other
 * is indexing through it.
 *
 * The name is an inline array rather than a pointer because a pointer means
 * nothing across a process boundary -- the two sides do not share an address
 * space, which is the whole reason this channel exists.
 */
typedef struct cnc_fieldbus_signal_desc {
  /* NUL-terminated. Unique within the region, in either direction. */
  char name[CNC_FIELDBUS_SIGNAL_NAME_MAX];
  /* cnc_fieldbus_signal_direction. */
  uint32_t direction;
  /* cnc_fieldbus_signal_type. */
  uint32_t type;
  /* cnc_fieldbus_signal_role. */
  uint32_t role;
  /* cnc_fieldbus_unit. */
  uint32_t unit;
  /*
   * Index of this signal's value within its direction's block. Assigned by the
   * process, dense from zero within each direction, and never reordered while
   * the region lives.
   */
  uint32_t value_index;
  /*
   * Update rate as a divisor of the cycle: 1 = every cycle, 10 = every tenth.
   * For slow signals -- a drive temperature, an sserial remote's diagnostics.
   * A hint to the core; the value is read every cycle regardless.
   */
  uint32_t update_divisor;
  uint32_t reserved[2];
} cnc_fieldbus_signal_desc;

/*
 * Where the table and the two value blocks are.
 *
 * This structure sits immediately after cnc_fieldbus_shm, at
 * sizeof(cnc_fieldbus_shm) from the start of the region -- which is a multiple
 * of CNC_FIELDBUS_CACHE_LINE, because that structure's alignment is. It exists
 * only in a region whose version_minor is at least 1.
 *
 * Every offset below is from the start of the region and is written by the
 * process rather than computed by the reader. That is deliberate: the fixed
 * part above is safe to compute independently because it is a compile-time
 * layout, and this part is not. The core validates each offset -- in bounds,
 * aligned, not overlapping -- and refuses the region rather than adapting to
 * it, as it does with everything else it checks at attach.
 */
typedef struct cnc_fieldbus_signal_block {
  /* Descriptors in the table. Zero is legal: a process with only axes. */
  uint32_t count;
  /* Of those, how many are INPUT and how many OUTPUT. They sum to `count`. */
  uint32_t input_count;
  uint32_t output_count;
  /* sizeof(cnc_fieldbus_signal_desc), so a size skew is caught, not suffered. */
  uint32_t desc_size;
  /* The descriptor table: `count` entries of `desc_size` bytes. */
  uint64_t table_offset;
  /*
   * The value blocks. Each is two halves of `*_count` doubles, selected by the
   * parity of the cycle number exactly as the fixed buffers above are, and
   * published by the same two counters. `input` is written by the process and
   * read by the core; `output` the other way about.
   */
  uint64_t input_offset;
  uint64_t output_offset;
  /*
   * Bytes from one parity half to the other -- `*_count * sizeof(double)`
   * rounded up to a cache line, so the two halves never share one. Written
   * out rather than derived for the same reason as the offsets.
   */
  uint64_t input_stride;
  uint64_t output_stride;
  /* Total bytes of the region, this block and everything after it included. */
  uint64_t region_size;
  uint64_t reserved[2];
} cnc_fieldbus_signal_block;

/* -------------------------------------------------------------------------
 * Read-only header
 * ------------------------------------------------------------------------- */

/*
 * Written once by the fieldbus process at creation and never again. The core
 * checks every field of it at attach and refuses the region rather than
 * adapting to it: a mapping is not something to negotiate over.
 */
typedef struct cnc_fieldbus_config {
  uint32_t version_major;
  uint32_t version_minor;
  uint32_t version_patch;
  /* Axes actually in use. The arrays above are always CNC_FIELDBUS_MAX_AXES. */
  uint32_t axis_count;
  /*
   * Identifies this run of the fieldbus process.
   *
   * The process unlinks and recreates the region on every start, so a stale
   * mapping is never adopted. The core refuses to continue across a generation
   * it did not attach to: a fieldbus restart is a fault that reaches the
   * operator, not a hiccup (ADR 0011 §6). By then the drives have faulted on
   * their own watchdog and an axis is somewhere the core no longer knows.
   */
  uint64_t generation;
  /*
   * sizeof(struct cnc_fieldbus_shm), checked at attach against the mapping.
   *
   * The fixed part only. From 1.1.0 the object is larger than this whenever
   * the process declares signals; `cnc_fieldbus_signal_block.region_size` says
   * how much larger. Keeping this field's meaning is what lets a 1.0.0 core
   * attach to a 1.1.0 region and drive its axes.
   */
  uint64_t layout_size;
  /* The bus cycle [ns]. The core's servo period, since the bus sets it. */
  uint32_t cycle_time_ns;
  /*
   * Missed cycles before the core declares the channel dead.
   *
   * All the core can do then is stop and report: it has no path to the drives
   * except the channel that just died, so it cannot decelerate an axis. The
   * machine is stopped by the drives' own watchdog below.
   */
  uint32_t core_watchdog_cycles;
  /*
   * Unanswered cycles before the fieldbus process takes the drives down
   * through quick-stop. Until then it repeats the last complete output, which
   * in CSP holds the axis — right immediately, wrong indefinitely.
   */
  uint32_t response_watchdog_cycles;
  /*
   * The drives' own sync-manager watchdog [us]. The stop of last resort.
   *
   * A safety parameter of this design rather than a fieldbus detail: it is the
   * only mechanism that stops the machine when the master dies, so it is
   * configured rather than left at whatever the drive defaults to. At 300 mm/s
   * a 100 ms timeout is 30 mm of travel after the bus goes quiet.
   */
  uint32_t sync_manager_watchdog_us;
  /*
   * Iterations the core spins on `cycle` before falling back to the futex.
   *
   * Zero by default: pay for the syscall until measurement says otherwise.
   */
  uint32_t spin_iterations;
  uint32_t reserved;
  /*
   * Worst-case duration of the process's own cyclic work [ns]. Added in 1.1.0;
   * zero means a process that does not declare one.
   *
   * §10.2 has the host refuse a driver whose worst case does not fit the cycle
   * budget, and until now that check stopped at the process boundary: a driver
   * plugin on this side of it reported its own copying time, which is
   * microseconds, while the process doing the actual work -- a UDP round trip
   * to an FPGA card, for HostMot2 -- reported nothing at all. It says so here
   * and `cnc-driver-fieldbus` reports it as its own, so the budget check
   * covers a driver on the far side of the boundary exactly as it covers one
   * in-process (ADR 0022 §3).
   *
   * This fits without moving anything: the fields above end at 56 bytes and
   * this fills the header out to exactly one cache line, which is where
   * `cycle` already began.
   */
  uint64_t worst_case_cycle_ns;
} cnc_fieldbus_config;

/* -------------------------------------------------------------------------
 * The region
 * ------------------------------------------------------------------------- */

/*
 * Each direction has two buffers selected by the parity of the cycle number,
 * and one counter that publishes them.
 *
 * The fieldbus writes input[n & 1] and then stores cycle = n with release
 * ordering. The core loads cycle with acquire ordering, reads input[n & 1],
 * computes, writes output[n & 1] and stores response = n with release
 * ordering. At its send deadline the fieldbus loads response with acquire
 * ordering and reads output[response & 1] — this cycle's work if the core
 * finished, and the previous cycle's if it did not.
 *
 * The fieldbus may publish cycle n only while response >= n - 1. That guard is
 * the whole basis for touching these buffers without a lock, and the argument
 * is written out in full in the Rust module that implements it.
 *
 * Each counter and each buffer pair sits on its own cache line. Two processes
 * writing adjacent lines every cycle is false sharing on the hottest path in
 * the system, and on ARM it costs more than on x86.
 *
 * THE WHOLE REGION (1.1.0)
 *   This structure is the fixed part and it is first, so every offset in it is
 *   a compile-time constant both sides reach independently. From 1.1.0 the
 *   named signals follow it:
 *
 *     0                          cnc_fieldbus_shm          (this structure)
 *     sizeof(cnc_fieldbus_shm)   cnc_fieldbus_signal_block (where the rest is)
 *     block.table_offset         cnc_fieldbus_signal_desc[block.count]
 *     block.input_offset         double[2][block.input_count]
 *     block.output_offset        double[2][block.output_count]
 *     block.region_size          end
 *
 *   `config.layout_size` is the size of this structure alone -- the layout the
 *   two sides must agree on byte for byte -- and `block.region_size` is the
 *   size of the object. A 1.0.0 region has no block and the two are equal.
 */
typedef struct cnc_fieldbus_shm {
  cnc_fieldbus_config config;

  /* Fieldbus -> core. Accessed atomically; see the ordering note above. */
  alignas(CNC_FIELDBUS_CACHE_LINE) uint64_t cycle;
  alignas(CNC_FIELDBUS_CACHE_LINE) cnc_fieldbus_input input[2];

  /* Core -> fieldbus. */
  alignas(CNC_FIELDBUS_CACHE_LINE) uint64_t response;
  alignas(CNC_FIELDBUS_CACHE_LINE) cnc_fieldbus_output output[2];

  /*
   * FUTEX_WAIT/FUTEX_WAKE word. A hint, never the truth: a spurious wake finds
   * `cycle` unchanged and goes back to sleep, and a missed wake is caught by
   * the next one. That is what lets the notification be replaced without
   * revisiting any of the ordering above.
   */
  alignas(CNC_FIELDBUS_CACHE_LINE) uint32_t futex_word;
  /* cnc_fieldbus_channel_state. */
  alignas(CNC_FIELDBUS_CACHE_LINE) uint32_t state;
} cnc_fieldbus_shm;

/*
 * What the channel as a whole is doing. Either side may move it to SHUTDOWN or
 * FAULTED; neither moves it back.
 */
typedef enum cnc_fieldbus_channel_state {
  /* The region exists but the fieldbus has not started cycling. */
  CNC_FIELDBUS_STATE_STARTING = 0,
  /* Cycling. */
  CNC_FIELDBUS_STATE_RUNNING = 1,
  /* One side is going away by intent. */
  CNC_FIELDBUS_STATE_SHUTDOWN = 2,
  /* One side gave up. Not resumable: the region is recreated on restart. */
  CNC_FIELDBUS_STATE_FAULTED = 3
} cnc_fieldbus_channel_state;

#ifdef __cplusplus
}
#endif

#endif /* CNC_FIELDBUS_H */
