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
```

`module.*` is what a LinuxCNC `loadrt` line sets; `param.*` is what a `setp`
line sets. A key it does not know, and a `param.` nothing claimed, are both
refused — a misspelled encoder scale otherwise leaves the machine moving the
wrong distance in silence.

What a pin *means* — which is an axis's feedback, what its limits are, which
way is X — is **not here**. That is the control's machine description, on the
other side of the channel. Two files that both described the machine would be
two files to disagree.

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
tree. The only thing that crosses is `cnc_fieldbus.h`, in this direction, and
it is Apache-2.0 OR MIT precisely so that it may.
