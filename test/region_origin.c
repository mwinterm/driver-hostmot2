/* HostMot2 observation provenance on outboard 2.4. SPDX-License-Identifier: GPL-2.0-or-later */
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <rtapi.h>
#include <hal.h>

#include "hm2_host.h"

void hm2_log(int level, const char *fmt, ...) {
    (void)level;
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
}

static int expect_origin(const char *selected, uint32_t expected) {
    uint32_t got = hm2_region_io_origin(selected);
    if (got != expected) {
        fprintf(stderr, "selected %s, ready '%s': origin 0x%x, expected 0x%x\n",
                selected, hm2_shim_transport_name(), got, expected);
        return 1;
    }
    return 0;
}

static void cycle_callback(void *arg, long period) {
    (void)arg;
    (void)period;
}

static int registration(const char *name, const char *write_name, uint32_t expected) {
    if (hm2_shim_init(1 << 16, 16, 4) != 0) {
        return 1;
    }
    hal_ready(hal_init("hostmot2"));
    int transport = hal_init(name);
    int failed = expect_origin(name, 0);
    failed |= hal_export_functf(cycle_callback, NULL, 0, 0, transport, "board.0.read");
    failed |= expect_origin(name, 0);
    failed |= hal_export_functf(cycle_callback, NULL, 0, 0, transport, "%s", write_name);
    failed |= expect_origin(name, 0);
    hal_ready(transport);
    failed |= expect_origin(name, expected);
    const char *other = strcmp(name, "hm2_eth") == 0 ? "hm2_test" : "hm2_eth";
    failed |= expect_origin(other, 0);
    if (strcmp(name, "hm2_test") == 0) {
        hal_ready(hal_init("another_transport"));
    } else {
        failed |= hal_export_functf(cycle_callback, NULL, 0, 0, transport, "board.1.read");
        failed |= expect_origin(name, 0);
        failed |= hal_export_functf(cycle_callback, NULL, 0, 0, transport, "board.1.write");
    }
    failed |= expect_origin(name, 0);
    hm2_shim_fini();
    return failed;
}

/* The public region is atomically renamed after its immutable header and
   named-signal block have been finalized. Only read those two structures. */
static int read_published(const char *path, uint32_t flags) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror(path);
        return 1;
    }
    cnc_outboard_config config;
    cnc_outboard_signal_block block;
    struct stat statbuf;
    int ok = pread(fd, &config, sizeof(config), 0) == sizeof(config) &&
             pread(fd, &block, sizeof(block), sizeof(cnc_outboard_shm)) == sizeof(block) &&
             fstat(fd, &statbuf) == 0;
    close(fd);
    if (!ok || config.version_major != 2 || config.version_minor != 4 ||
        config.version_patch != 0 || config.flags != flags ||
        config.layout_size != sizeof(cnc_outboard_shm) || block.count == 0 ||
        block.region_size != (uint64_t)statbuf.st_size) {
        fprintf(stderr, "%s: expected channel 2.4.0, flags 0x%x and named signals only\n",
                path, flags);
        return 1;
    }
    printf("%s: channel 2.4.0, flags 0x%x, %u named signals\n", path, config.flags, block.count);
    return 0;
}

int main(int argc, char **argv) {
    if (argc == 1) {
        int failed = registration("hm2_eth", "board.0.write", CNC_OUTBOARD_CONFIG_HARDWARE_IO);
        failed |= registration("hm2_test", "board.0.write", CNC_OUTBOARD_CONFIG_MODELED_IO);
        failed |= registration("unqualified", "board.0.write", 0);
        failed |= registration("hm2_eth", "board.1.write", 0);
        return failed;
    }
    if (argc != 4 || (strcmp(argv[2], "true") != 0 && strcmp(argv[2], "false") != 0) ||
        (strcmp(argv[3], "modeled") != 0 && strcmp(argv[3], "unknown") != 0)) {
        return 2;
    }
    uint32_t expected = strcmp(argv[3], "modeled") == 0 ? CNC_OUTBOARD_CONFIG_MODELED_IO : 0;
    if (strcmp(argv[2], "true") == 0) {
        expected |= CNC_OUTBOARD_CONFIG_SAME_CYCLE;
    }
    return read_published(argv[1], expected);
}
