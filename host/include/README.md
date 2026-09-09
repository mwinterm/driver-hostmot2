# Vendored from LibreCNC

`cnc_fieldbus.h` is the fieldbus channel's layout, copied from the LibreCNC
repository (ADR 0022 §8: this repository vendors it and says which version).

    Upstream repo : https://github.com/mwinterm/LibreCNC
    Path          : crates/cnc-fieldbus-abi/include/cnc_fieldbus.h
    Channel version: 1.1.0
    Copied from commit: 64aef9cd42a8cd609a91b3c9672e8911ecc0f810
    Copied on     : 2026-09-09

It is **Apache-2.0 OR MIT**, which is why this GPL-2.0 repository may compile
against it, and it is the whole reason that dual license exists (ADR 0022 §3).
It includes no other header of that project's, so this is one file and not a
tree.

Nothing flows the other way. No function, snippet or algorithm from this
repository or from LinuxCNC is copied into LibreCNC, and its CI fails on a GPL
license header anywhere in its tree.

To take a newer channel version: copy the file again, update the commit above,
and check the version constants at the top of it against what the host asserts.
