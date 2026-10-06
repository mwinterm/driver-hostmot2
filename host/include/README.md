# Vendored from LibreCNC

`cnc_outboard.h` is the outboard channel's layout, copied from the LibreCNC
repository (ADR 0022 §8: this repository vendors it and says which version).

    Upstream repo : https://github.com/mwinterm/LibreCNC
    Path          : crates/cnc-outboard-abi/include/cnc_outboard.h
    Channel version: 2.2.0
    Copied from commit: 032fd5d4218d8266b800bd89a2a75e0b0aea1f77 (LeafCNC branch feature/mikron-wf41c, ADR 0045)
    Copied on     : 2026-10-07

It is **Apache-2.0 OR MIT**, which is why this GPL-2.0 repository may compile
against it, and it is the whole reason that dual license exists (ADR 0022 §3).
It includes no other header of that project's, so this is one file and not a
tree.

Nothing flows the other way. No function, snippet or algorithm from this
repository or from LinuxCNC is copied into LibreCNC, and its CI fails on a GPL
license header anywhere in its tree.

To take a newer channel version: copy the file again, update the commit above,
and check the version constants at the top of it against what the host asserts.

Renamed at 2.0.0
----------------

It was `cnc_fieldbus.h`, with `CNC_FIELDBUS_*` symbols, until the channel went
to 2.0.0. Nothing about the layout changed -- byte for byte a 2.0.0 region is
a 1.1.0 region -- but the word *fieldbus* described only the first thing that
was ever on the far side of it. A Mesa card reached by UDP is not a fieldbus.
What every case has in common is a driver mounted **outboard**, outside the
control's process, which is what this repository is.
