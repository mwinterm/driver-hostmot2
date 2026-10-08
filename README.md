# driver-hostmot2

LinuxCNC's HostMot2 driver for Mesa Electronics FPGA cards, running as a
process of its own and talking to [LibreCNC](https://github.com/mwinterm/LibreCNC)
over a shared-memory channel.

**GPL-2.0-or-later**, and that is the reason this repository exists. LibreCNC
is Apache-2.0 and a plugin it loads runs in its process, so a GPL driver cannot
be one. This is the other shape: the GPL code runs here, and the control's end
is an ordinary Apache-2.0 plugin that maps a region and knows nothing about
HostMot2 (LibreCNC ADR 0022).

## What is inherited and what is new

| | |
|---|---|
| `third_party/linuxcnc/` | LinuxCNC's `src/hal/drivers/mesa-hostmot2`, extracted with `git subtree split` and **byte-identical to upstream**. Full history and authorship preserved. See `NOTICE`. |
| `shim/` | A reimplementation of the HAL/RTAPI surface the driver uses — 25 functions — backed by the channel's signal table instead of HAL's shared memory. Plus the upstream headers it compiles against. |
| `host/` | `hm2-host`: the process. Configuration, the shared region, and the cycle. |

Nothing in `third_party/linuxcnc/` is edited. Not as a preference — as the
thing that keeps `git subtree pull` from upstream a merge rather than a
rewrite. The two places that would have needed an edit are handled instead:
module parameters are `static`, so they are set through the non-static
`rtapi_info_address_*` symbols `RTAPI_MP_*` emits beside them, which is what
LinuxCNC's own loader does.

## Building

```sh
make                       # build/hm2-host, and the three shared objects
./build/hm2-host examples/7i92-7i77.conf build
```

No dependencies beyond a C compiler and libc.

## Configuring

One flat file, and it says what the **hardware** is:

```
region = /cnc-mesa-0
cycle_us = 1000
module.board_ip = 10.10.10.10
module.config = num_encoders=6 sserial_port_0=00xxxx
param.hm2_7i92.0.encoder.00.scale = 2000
pin.hm2_7i92.0.stepgen.00.control-type = 1
```

`module.*` is what a LinuxCNC `loadrt` line sets; `param.*` is what a `setp`
line sets. A key it does not know, and a `param.` nothing claimed, are both
refused — a misspelled encoder scale otherwise leaves the machine moving the
wrong distance in silence.

`pin.*` is what a `setp` on a pin nothing drives sets: a stepgen's
`control-type`, a DPLL's `timer-us`, an encoder's `timer-number` or
`quad-error-enable` (the control's ADR 0045 §2). The value -- `true`,
`false` or a number -- is set before the board's first cycle, and the pin is
then published **for reading only**, flagged `FIXED`, so nothing in the
control can write it and a machine description that tries is told why. A pin
nobody declared, one the driver writes, a both-ways pin and a value that does
not fit the pin's type are each refused by name, with the line.

**Every parameter is published too** (the control's ADR 0053), after the
pins: read back as an input, so the control sees what the driver runs on --
`halcmd show param` -- and, where the driver declared it writable, written as
an output applied only when the control's value changes -- `halcmd setp`.
Its starting value is the `param.` line's, or the driver's default, so
nothing changes until the control asks; when it may ask is the control's to
decide. That includes `io_error`: cleared by the control after the link
died, it lets the transport try the board again, and this process stops
reporting the bus as faulted unless the board still does not answer.

Two more things the process says without being asked (channel 2.2.0, ADR
0045):

- A pin the driver writes as well as reads -- a `HAL_IO` pin such as an
  encoder's `index-enable` or the watchdog's `has_bit` -- is published once
  in each direction, and its output is applied **only in a cycle the
  control's value changes**, flagged `ON_CHANGE`. Between changes the pin is
  the driver's: the card lowers `index-enable` at the index and it stays
  lowered, a bite raises `has_bit` and it stays raised until the control
  lowers it. Written every cycle, the control's held zero used to undo a
  watchdog bite the cycle after it happened. The encoder's `probe-enable`
  goes the same way although upstream declares it `HAL_IN`: the driver
  clears it when the probe latches, as it clears `index-enable` at an index
  (the control's ADR 0052 §2). `make check` shows it, with no board.
- Whether the board answers, in the channel's `bus_state` and `bus_fault`
  every cycle. When the transport sets `io_error` -- too many late or lost
  replies, after which it reads and writes nothing -- the bus is `FAULT`
  with reason `LINK` for the rest of this run, and the control faults every
  axis behind the process and says a restart brings it back. While `has_bit`
  reads true it is `FAULT` with reason `WATCHDOG`. The process goes on
  cycling either way: it is the control's clock.

**One period, read to write** (channel 2.3.0, the control's ADR 0046). Each
period reads the card, publishes what it read, waits for the control's answer
to it -- the control wakes the process -- until `send_deadline` (a fraction
of the period, 0.8 by default) and writes it, the way a LinuxCNC servo thread
reads, computes and writes. A sample reaches the card's outputs the period it
was taken, where it used to take two. An answer that misses the deadline is
written as the last complete one and counted. `same_cycle = false` keeps the
old order -- the answer to the previous publish written first -- to compare
the two on a bench.

The process says how that goes, as inputs beside the driver's pins, in SI:
`hm2-host.read-time` [s] (the last read's round trip), `hm2-host.answer-time`
[s] (from the last publish to the control's answer) and
`hm2-host.answers-late` (a count). A recorder on the control's side
(LeafSCOPE) records them like any pin; the worst of each is logged at stop.

**Every Smart Serial fault is counted**, in `hm2-host.sserial-faults`, and
logged with its cycle, how long before that period's read the previous write
had ended -- the time the port's transfer had -- and whether that write
waited for an answer that did not come in time. The driver itself says
"DoIt not cleared from previous servo thread" once, at a port's fourth
fault, and its `fault-count` pin decays to zero within cycles, so neither
says how often a port faults. A fault costs the port that period's update
(its outputs, an analog command among them, keep last period's values); a
port faulting about twenty times in quick succession is stopped for good.
A port that has just been started may fault a few times while it comes up --
the driver lets four pass before it says anything -- and those are counted
too.

What a pin *means* — which is an axis's feedback, what its limits are, which
way is X — is **not here**. That is the control's machine description, on the
other side of the channel. Two files that both described the machine would be
two files to disagree.

## Running it without a Mesa card

Upstream ships a fake board — `hm2_test`, which shows the generic driver a test
pattern instead of an IDROM read off a card — and this repository builds it:

```sh
./build/hm2-host examples/test-board.conf build
```

That exercises the whole path: both modules load, the generic driver parses the
IDROM and enumerates the board's pins, the shim collects them, and the host
publishes them into a shared region and cycles. Test pattern 12 presents a
24-pin GPIO board, which comes out as 72 HAL pins and 72 channel signals.

Attach a control to the same region and the two halves meet:

```yaml
drivers:
  - name: hm2
    library: libcnc_driver_outboard.so
    config:
      axes: "3"
      region: "/cnc-hm2-test"
plc:
  io:
    lamp: hm2.hm2_test.0.gpio.000.out
```

What this does **not** exercise is the only thing that matters in the end: a
real card answering a real packet in a real millisecond. Everything in the
matrix below is still `untested`.

## Support matrix

A function class is *supported* when it has a mapping on the control's side and
has run on a bench. A board is supported when every function on it is. CI
cannot test hardware it does not have, so this table is the claim — not the
list of part numbers upstream drives.

| | Status |
|---|---|
| 7i92 (Ethernet, LBP16/UDP) | **untested** — builds, and the first bench target |
| 7i77 (analog servo, ±10 V) | **untested** — the first bench target |
| 7i74 + sserial remotes (7i70, 7i71) | **untested** — the first bench target |
| Every other Ethernet board (7i93…7i98) | expected: same transport, same IDROM |
| PCI, SPI and EPP boards | in the tree, not in the build (`Makefile`) |
| Encoder, stepgen, PWM, sserial, DPLL, watchdog, … | compiled; none exercised against hardware |

Nothing here has been run against a Mesa card yet. The `untested` rows are the
ones with a bench waiting for them; everything else is upstream's breadth,
which arrives with the fork and is not a claim this repository has earned.

## Code flows one way: to nowhere

No function, snippet or algorithm from this repository or from LinuxCNC is
copied into LibreCNC. Its CI fails on a GPL license header anywhere in its
tree. The only thing that crosses is `cnc_outboard.h`, in this direction, and
it is Apache-2.0 OR MIT precisely so that it may.
