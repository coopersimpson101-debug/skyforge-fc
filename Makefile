# skyforge-fc Makefile (GNU make)
#
#   make test            host unit tests (system C compiler, default cc)
#   make sim             simulator shared library + CLI
#   make sim-test        pytest closed-loop tests through ctypes (needs `make sim`)
#   make firmware        STM32F405 image with arm-none-eabi-gcc
#   make flash-stlink    flash via ST-Link + OpenOCD
#   make flash-dfu       flash via USB DFU (board in bootloader mode)
#
# Windows without make: use `python tools/build.py test|sim|firmware` (zig cc).

CC        ?= cc
PYTHON    ?= python3
ARM_PREFIX ?= arm-none-eabi-
BUILD     := build

CORE_SRC  := $(wildcard src/core/*.c)
DRV_SRC   := $(wildcard src/drivers/*.c)
SIM_SRC   := sim/quad_sim.c
TEST_SRC  := $(wildcard tests/test_*.c)
TEST_BIN  := $(patsubst tests/%.c,$(BUILD)/host/%,$(TEST_SRC))

HOST_CFLAGS := -std=c11 -O2 -g -Wall -Wextra -Wshadow -Wno-unused-function -DFC_HOST
HOST_LDLIBS := -lm

UNAME_S := $(shell uname -s 2>/dev/null)
ifeq ($(UNAME_S),Darwin)
  SHLIB := dylib
else
  SHLIB := so
endif

.PHONY: all test sim sim-test firmware flash-stlink flash-dfu clean

all: test sim firmware

# ---- host ------------------------------------------------------------------
$(BUILD)/host/test_%: tests/test_%.c $(CORE_SRC) $(DRV_SRC) $(SIM_SRC) $(wildcard src/*/*.h sim/*.h tests/*.h)
	@mkdir -p $(dir $@)
	$(CC) $(HOST_CFLAGS) -o $@ $< $(CORE_SRC) $(DRV_SRC) $(SIM_SRC) $(HOST_LDLIBS)

test: $(TEST_BIN)
	@set -e; for t in $(TEST_BIN); do echo "=== $$t"; ./$$t; done; echo "ALL HOST TESTS PASSED"

sim: $(BUILD)/host/skyforge_sim.$(SHLIB) $(BUILD)/host/sim_main

$(BUILD)/host/skyforge_sim.$(SHLIB): $(SIM_SRC) $(CORE_SRC)
	@mkdir -p $(dir $@)
	$(CC) $(HOST_CFLAGS) -shared -fPIC -o $@ $(SIM_SRC) $(CORE_SRC) $(HOST_LDLIBS)

$(BUILD)/host/sim_main: sim/sim_main.c $(SIM_SRC) $(CORE_SRC)
	@mkdir -p $(dir $@)
	$(CC) $(HOST_CFLAGS) -o $@ sim/sim_main.c $(SIM_SRC) $(CORE_SRC) $(HOST_LDLIBS)

sim-test: sim
	$(PYTHON) -m pytest -q tests/test_sim_py.py

# ---- firmware (arm-none-eabi-gcc) -------------------------------------------
FW_ELF := $(BUILD)/fw/skyforge-f405.elf
FW_BIN := $(BUILD)/fw/skyforge-f405.bin
FW_SRC := $(CORE_SRC) $(DRV_SRC) $(wildcard src/platform/stm32f4/*.c) \
          $(wildcard src/boards/f405_generic/*.c) src/app/main.c
FW_CFLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
             -std=c11 -O2 -g -Wall -Wextra -Wno-unused-function \
             -ffreestanding -fno-builtin -fno-math-errno -ffunction-sections -fdata-sections \
             -fno-unwind-tables -fno-asynchronous-unwind-tables \
             -DSTM32F405xx -DBOARD_F405_GENERIC -Isrc
FW_LDFLAGS := -nostdlib -Wl,--gc-sections -Wl,-T,ld/stm32f405.ld -Wl,-Map=$(BUILD)/fw/skyforge-f405.map

firmware: $(FW_BIN)

$(FW_ELF): $(FW_SRC) ld/stm32f405.ld $(wildcard src/*/*.h src/*/*/*.h)
	@mkdir -p $(dir $@)
	$(ARM_PREFIX)gcc $(FW_CFLAGS) $(FW_LDFLAGS) -o $@ $(FW_SRC) -lgcc
	$(ARM_PREFIX)size $@

$(FW_BIN): $(FW_ELF)
	$(ARM_PREFIX)objcopy -O binary $< $@

flash-stlink: $(FW_ELF)
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	        -c "program $(FW_ELF) verify reset exit"

flash-dfu: $(FW_BIN)
	dfu-util -a 0 -s 0x08000000:leave -D $(FW_BIN)

clean:
	rm -rf $(BUILD)
