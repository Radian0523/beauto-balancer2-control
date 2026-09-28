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
VSLOW ?= 0.08
ACC ?= 0.5
FFK ?= 1
TI ?= 2
ICLIP ?= 8
EMAX ?= 4
VBW ?= 20
SMAX ?= 0
KCP ?= 4
KCI ?= 1
KBA ?= 1
KWA ?= 1
KBR ?= 1
KWR ?= 1
BOOST ?= 0
BACC ?= 0.6
CAL_RANGE ?= 30
GTRIM ?= 5
BZT ?= 2
BL ?= 0
BLW ?= 500
LOGDIV ?= 20
LOGSET ?= 1
DIAG ?= 0
DIAG_DELAY ?= 0
TEST ?= 0
TAMP ?= 3000
THZ ?= 2
DEFS = -DV_MAX='($(VMAX))' -DV_BACK='($(VBACK))' -DV_SLOW='($(VSLOW))' -DACC_MAX='($(ACC))' -DFF_K='($(FFK))' -DTI_S='($(TI))' -DI_CLIP='($(ICLIP))' \
       -DE_MAX='($(EMAX))' -DVBW='($(VBW))' -DS_MAX='($(SMAX))' -DK_CP='($(KCP))' -DK_CI='($(KCI))' -DK_BA='($(KBA))' -DK_WA='($(KWA))' -DK_BR='($(KBR))' -DK_WR='($(KWR))' -DV_BOOST='($(BOOST))' -DACC_BOOST='($(BACC))' \
       -DCAL_RANGE=$(CAL_RANGE) -DGYRO_TRIM_T='($(GTRIM))' -DBZ_T='($(BZT))' -DBL_I='($(BL))' -DBL_W='($(BLW))' -DREC_DIV=$(LOGDIV) -DLOG_SET=$(LOGSET) -DDIAG=$(DIAG) -DDIAG_DELAY=$(DIAG_DELAY) -DTEST=$(TEST) -DTEST_AMP=$(TAMP) -DTEST_HZ=$(THZ)

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
