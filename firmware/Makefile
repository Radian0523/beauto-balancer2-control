# Beauto Balancer 2 用ファームのビルド。 `make` で build/bb2.bin ができる。
# 書き込み: USB で繋ぐと出る CRP DISABLE ドライブの firmware.bin を消して bb2.bin をコピー。
TOOL ?= /Applications/ArmGNUToolchain/15.3.rel1/arm-none-eabi/bin/arm-none-eabi-
CC      = $(TOOL)gcc
OBJCOPY = $(TOOL)objcopy
SIZE    = $(TOOL)size

V = vendor
GYRO ?= +1
WHEEL ?= +1
CUR ?= +1
SIGNS = -DSIGN_GYRO='($(GYRO).0f)' -DSIGN_WHEEL='($(WHEEL).0f)' -DSIGN_CUR='($(CUR).0f)'

CFLAGS  = $(SIGNS) -mcpu=cortex-m3 -mthumb -Os -ffunction-sections -fdata-sections \
          -fcommon -fno-builtin -D__USE_CMSIS=Balancer2_lib \
          -I. -I$(V) -I$(V)/lib -Wall -Wno-unused-variable
LDFLAGS = -mcpu=cortex-m3 -mthumb -nostartfiles --specs=nano.specs --specs=nosys.specs \
          -Wl,--gc-sections -Wl,-Map=build/bb2.map -T bb2.ld

SRCS = main_bb2.c bb2_control.c $(V)/control.c $(V)/cr_startup_lpc13.c
OBJS = $(patsubst %.c,build/%.o,$(notdir $(SRCS)))
vpath %.c . $(V)

all: build/bb2.bin

build/%.o: %.c bb2_control.h gains.h pace.h Makefile | build
	$(CC) $(CFLAGS) -c $< -o $@

build/bb2.axf: $(OBJS) bb2.ld
	$(CC) $(LDFLAGS) -o $@ $(OBJS) -L$(V)/lib -lBalancer2_lib -lm
	$(SIZE) $@

build/bb2.bin: build/bb2.axf
	$(OBJCOPY) -O binary $< $@
	python3 lpc_checksum.py $@

build:
	mkdir -p build

clean:
	rm -rf build

.PHONY: all clean

# 切り分け用: 純正サンプル(Default)をこの環境でビルドしたもの。
# これで立つのに bb2.bin で倒れるなら、原因はビルド環境ではなく制御側(符号・ゲイン)。
STOCK_OBJS = build/stock_main.o build/control.o build/cr_startup_lpc13.o
build/stock.axf: $(STOCK_OBJS) bb2.ld
	$(CC) $(LDFLAGS) -o $@ $(STOCK_OBJS) -L$(V)/lib -lBalancer2_lib -lm
build/stock.bin: build/stock.axf
	$(OBJCOPY) -O binary $< $@
	python3 lpc_checksum.py $@
stock: build/stock.bin
.PHONY: stock
