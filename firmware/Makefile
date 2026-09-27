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

# main.c の共通設定。v3 = コントローラ操縦 + 段差・坂対策。 make v3 TI=2 EMAX=4 ACC=0.5 VMAX=0.2 BOOST=0.25 BACC=0.6
TI ?= 2
EMAX ?= 4
ACC ?= 0.5
VMAX ?= 0.5
BOOST ?= 0
BACC ?= 0.6
V3_FIX ?= 1
FFK ?= 0
FF_DIR ?= 1
VOBS ?= 0
VBW ?= 20
LOGDIV ?= 20
NOTCH ?= 0
ICLIP ?= 8
DIAG ?= 0
DIAG_DELAY ?= 0
NQ ?= 2
V3_DEFS = -DDIAG=$(DIAG) -DDIAG_DELAY=$(DIAG_DELAY) -DI_CLIP='($(ICLIP))' -DNOTCH=$(NOTCH) -DNQ='($(NQ))' -DREC_DIV=$(LOGDIV) -DVOBS=$(VOBS) -DVBW='($(VBW))' -DFF='($(FFK))' -DFF_DIR='($(FF_DIR).0)' -DFIX_SPD_REF=$(V3_FIX) -DTI_S='($(TI))' -DE_MAX='($(EMAX))' -DACC_MAX='($(ACC))' -DV_MAX='($(VMAX))' -DV_BOOST='($(BOOST))' -DACC_BOOST='($(BACC))'
build/main.o: main.c Makefile FORCE | build
	$(CC) $(CFLAGS) $(V3_DEFS) -c $< -o $@
V3_OBJS = build/main.o build/control.o build/cr_startup_lpc13.o
build/v3.axf: $(V3_OBJS) bb2.ld
	$(CC) $(LDFLAGS) -o $@ $(V3_OBJS) -L$(V)/lib -lBalancer2_lib -lm
	$(SIZE) $@
build/v3.bin: build/v3.axf
	$(OBJCOPY) -O binary $< $@
	python3 lpc_checksum.py $@
v3:
	rm -f build/main.o build/v3.axf build/v3.bin
	$(MAKE) build/v3.bin
.PHONY: v3

# v4 = v3 + 目標軌道フィードフォワード (前傾 a/g)。 make v4 FF_DIR=-1 で向き反転
v4:
	rm -f build/main.o build/v3.axf build/v3.bin
	$(MAKE) build/v3.bin FFK=1 FF_DIR=$(FF_DIR)
	cp build/v3.bin build/v4.bin
.PHONY: v4

# v5 = v4 + 車輪角速度オブザーバ。 make v5 VBW=20
v5:
	rm -f build/main.o build/v3.axf build/v3.bin
	$(MAKE) build/v3.bin FFK=1 FF_DIR=$(FF_DIR) VOBS=1 VBW=$(VBW) LOGDIV=$(LOGDIV) NOTCH=$(NOTCH) NQ=$(NQ) ICLIP=$(ICLIP) DIAG=$(DIAG) DIAG_DELAY=$(DIAG_DELAY)
	cp build/v3.bin build/v5.bin
.PHONY: v5
