# driver-hostmot2 -- the HostMot2 driver as a process of its own (ADR 0022).
#
# Three pieces and the shape upstream already has:
#
#   libhm2shim.so    the HAL/RTAPI surface the driver is written against,
#                    reimplemented over the fieldbus channel (shim/).
#   libhostmot2.so   LinuxCNC's generic HostMot2 driver: the IDROM, the module
#                    descriptors, and every function the FPGA implements.
#   libhm2_eth.so    the LBP16-over-UDP transport, for the Ethernet boards.
#   hm2-host         the process: configuration, the region, the cycle.
#
# The two driver libraries are separate because upstream's are: `hostmot2` and
# `hm2_eth` are two LinuxCNC modules, each with its own `rtapi_app_main`, and
# the transport registers itself with the generic driver. Merging them would
# mean editing sources this repository keeps byte-identical to upstream, and
# the host loads them in the order LinuxCNC's `loadrt` would.
#
# The shim is shared rather than linked into each, because there is one arena
# and one signal table and both driver modules allocate out of them.
#
# SPDX-License-Identifier: GPL-2.0-or-later

UPSTREAM  := third_party/linuxcnc
SHIM      := shim
HOST      := host
BUILD     ?= build

# -DRTAPI, not -DULAPI: this is a real-time component that happens to run in
# userspace, which is the distinction LinuxCNC's uspace build makes too.
# -D_GNU_SOURCE for `environ`, which hm2_eth.c uses to spawn its firewall
# helper.
CPPFLAGS  += -DRTAPI -D_GNU_SOURCE -I$(SHIM) -I$(SHIM)/include \
             -I$(SHIM)/include/linuxcnc -I$(UPSTREAM)
CFLAGS    ?= -O2 -g -fPIC
# The upstream sources are not warning-clean under this project's taste and
# are not ours to make so; they are compiled as they arrive. The shim and the
# host are held to a stricter set below.
CFLAGS    += -Wall
LDFLAGS   += -Wl,-rpath,'$$ORIGIN'
LDLIBS    += -lm -lpthread

STRICT    := -Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes

# The generic driver: hostmot2.c and every function the FPGA implements.
HM2_SRC := hostmot2.c tram.c pins.c ioport.c encoder.c abs_encoder.c resolver.c \
           pwmgen.c tp_pwmgen.c rcpwmgen.c stepgen.c ssr.c outm.c inm.c inmux.c \
           oneshot.c periodm.c dpll.c watchdog.c led.c raw.c uart.c pktuart.c \
           bspi.c xy2mod.c sserial.c bitfile.c llio_info.c

# The Ethernet transport. The other transports upstream carries -- PCI, SPI,
# the parallel-port boards -- are in the tree and are enabled when there is a
# bench with one of those boards to try them on (ADR 0022 §9).
ETH_SRC := hm2_eth.c hm2_eth_net_posix.c

SHIM_SRC := hal_shim.c rtapi_shim.c param_table.c
HOST_SRC := main.c config.c region.c

HM2_OBJ  := $(addprefix $(BUILD)/hm2/,$(HM2_SRC:.c=.o))
ETH_OBJ  := $(addprefix $(BUILD)/eth/,$(ETH_SRC:.c=.o))
SHIM_OBJ := $(addprefix $(BUILD)/shim/,$(SHIM_SRC:.c=.o))
HOST_OBJ := $(addprefix $(BUILD)/host/,$(HOST_SRC:.c=.o))

.PHONY: all clean
all: $(BUILD)/hm2-host $(BUILD)/libhostmot2.so $(BUILD)/libhm2_eth.so

$(BUILD)/libhm2shim.so: $(SHIM_OBJ)
	@mkdir -p $(@D)
	$(CC) -shared -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(BUILD)/libhostmot2.so: $(HM2_OBJ) $(BUILD)/libhm2shim.so
	@mkdir -p $(@D)
	$(CC) -shared -o $@ $(HM2_OBJ) -L$(BUILD) -lhm2shim $(LDFLAGS) $(LDLIBS)

# Against libhostmot2: the transport calls hm2_register and its siblings.
$(BUILD)/libhm2_eth.so: $(ETH_OBJ) $(BUILD)/libhostmot2.so
	@mkdir -p $(@D)
	$(CC) -shared -o $@ $(ETH_OBJ) -L$(BUILD) -lhostmot2 -lhm2shim $(LDFLAGS) $(LDLIBS)

# -rdynamic so the shim can find `rtapi_info_address_*` in whichever module
# defines it; that is how a module parameter is set without editing a source.
$(BUILD)/hm2-host: $(HOST_OBJ) $(BUILD)/libhm2shim.so
	@mkdir -p $(@D)
	$(CC) -rdynamic -o $@ $(HOST_OBJ) -L$(BUILD) -lhm2shim $(LDFLAGS) $(LDLIBS) -ldl

$(BUILD)/hm2/%.o: $(UPSTREAM)/%.c
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD)/eth/%.o: $(UPSTREAM)/%.c
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

$(BUILD)/shim/%.o: $(SHIM)/%.c
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(STRICT) -c -o $@ $<

$(BUILD)/host/%.o: $(HOST)/%.c
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(STRICT) -c -o $@ $<

clean:
	rm -rf $(BUILD)
