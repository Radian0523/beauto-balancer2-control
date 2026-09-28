# Beauto Balancer 2 用ファームのビルド。 `make` で build/v5.bin ができる。
# 書き込みは ../bb2 flash v5 (Mac ではドライブがマウントされないので ../tools/flash_mac.py で直接書く)。
TOOL ?= /Applications/ArmGNUToolchain/15.3.rel1/arm-none-eabi/bin/arm-none-eabi-
CC      = $(TOOL)gcc
OBJCOPY = $(TOOL)objcopy
SIZE    = $(TOOL)size

V = vendor
CFLAGS  = -mcpu=cortex-m3 -mthumb -O0 -ffunction-sections -fdata-sections \
          -fcommon -fno-builtin -D__USE_CMSIS=Balancer2_lib \
          -I. -I$(V) -I$(V)/lib -Wall -Wno-unused-variable
LDFLAGS = -mcpu=cortex-m3 -mthumb -nostartfiles --specs=nano.specs --specs=nosys.specs \
          -Wl,--gc-sections -Wl,-Map=build/bb2.map -T bb2.ld

vpath %.c . $(V)

# 既定は v5。 make だけで build/v5.bin ができる
all: v5

build/%.o: %.c Makefile | build
	$(CC) $(CFLAGS) -c $< -o $@

build:
	mkdir -p build

clean:
	rm -rf build

.PHONY: all clean FORCE
FORCE:

# 切り分け用: 純正サンプル(Default)をこの環境でビルドしたもの。
# これで立つのに自作ファームで倒れるなら、原因はビルド環境ではなく制御側。 make stock
STOCK_OBJS = build/stock_main.o build/control.o build/cr_startup_lpc13.o
build/stock.axf: $(STOCK_OBJS) bb2.ld
	$(CC) $(LDFLAGS) -o $@ $(STOCK_OBJS) -L$(V)/lib -lBalancer2_lib -lm
build/stock.bin: build/stock.axf
	$(OBJCOPY) -O binary $< $@
	python3 lpc_checksum.py $@
stock: build/stock.bin
.PHONY: stock

# v5 (main.c) の調整値。 ../bb2 flash v5 VMAX=0.4 のように上書きする。意味は main.c の先頭
VMAX ?= 0.5
VBACK ?= 0.3
ACC ?= 0.5
FFK ?= 1
TI ?= 2
ICLIP ?= 8
EMAX ?= 4
VBW ?= 20
KBR ?= 1
KWR ?= 1
BOOST ?= 0
BACC ?= 0.6
CAL_RANGE ?= 30
GTRIM ?= 5
LOGDIV ?= 20
DIAG ?= 0
DIAG_DELAY ?= 0
DEFS = -DV_MAX='($(VMAX))' -DV_BACK='($(VBACK))' -DACC_MAX='($(ACC))' -DFF_K='($(FFK))' -DTI_S='($(TI))' -DI_CLIP='($(ICLIP))' \
       -DE_MAX='($(EMAX))' -DVBW='($(VBW))' -DK_BR='($(KBR))' -DK_WR='($(KWR))' -DV_BOOST='($(BOOST))' -DACC_BOOST='($(BACC))' \
       -DCAL_RANGE=$(CAL_RANGE) -DGYRO_TRIM_T='($(GTRIM))' -DREC_DIV=$(LOGDIV) -DDIAG=$(DIAG) -DDIAG_DELAY=$(DIAG_DELAY)

# 引数を変えたときに作り直されるよう、main.o は毎回コンパイルする (FORCE)
build/main.o: main.c Makefile FORCE | build
	$(CC) $(CFLAGS) $(DEFS) -c $< -o $@
V5_OBJS = build/main.o build/control.o build/cr_startup_lpc13.o
build/v5.axf: $(V5_OBJS) bb2.ld
	$(CC) $(LDFLAGS) -o $@ $(V5_OBJS) -L$(V)/lib -lBalancer2_lib -lm
	$(SIZE) $@
build/v5.bin: build/v5.axf
	$(OBJCOPY) -O binary $< $@
	python3 lpc_checksum.py $@
v5: build/v5.bin
.PHONY: v5
