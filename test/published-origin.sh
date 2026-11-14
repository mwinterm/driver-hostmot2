#!/bin/sh
# Read the channel a real hm2-host publishes with the fake board.
# SPDX-License-Identifier: GPL-2.0-or-later
set -eu
build=$(realpath "$1")
tmp=$(mktemp -d)
region="/hm2-origin-$$"
pid=
cleanup() {
    if [ -n "$pid" ]; then
        kill "$pid" 2>/dev/null || true
        wait "$pid" || true
    fi
    rm -rf "$tmp"
}
trap cleanup EXIT
check_host() {
    transport=$1
    modules=$2
    same_cycle=$3
    origin=$4
    sed -e "s|^region = .*|region = $region|" -e "s|^transport = .*|transport = $transport|" \
        examples/test-board.conf > "$tmp/host.conf"
    printf 'same_cycle = %s\nmeasure_cycles = 10\n' "$same_cycle" >> "$tmp/host.conf"
    "$build/hm2-host" "$tmp/host.conf" "$modules" > "$tmp/host.log" 2>&1 &
    pid=$!
    n=0
    while [ ! -e "/dev/shm$region" ]; do
        if ! kill -0 "$pid" 2>/dev/null || [ "$n" -ge 100 ]; then
            cat "$tmp/host.log" >&2
            exit 1
        fi
        sleep 0.05
        n=$((n + 1))
    done
    "$build/region-origin" "/dev/shm$region" "$same_cycle" "$origin"
    kill "$pid"
    wait "$pid"
    pid=
}

check_host hm2_test "$build" true modeled
check_host hm2_test "$build" false modeled

# An Ethernet filename in a module directory cannot turn a test board into
# physical probe evidence. Run only the fake board under that name.
ln -s "$build/libhostmot2.so" "$tmp/libhostmot2.so"
ln -s "$build/libhm2_test.so" "$tmp/libhm2_eth.so"
check_host hm2_eth "$tmp" true unknown
