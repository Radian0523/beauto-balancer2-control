/* LPC1343 (Beauto Balancer 2) 用リンカスクリプト。
   純正 Balancer2_Debug.ld / _mem.ld を newlib 用にまとめたもの。
   0x7000-0x7FFF は純正ライブラリがゲイン等(memmap)を保存する領域。 */
MEMORY
{
  MFlash32 (rx)  : ORIGIN = 0x0,        LENGTH = 0x7000
  FlashMap (rx)  : ORIGIN = 0x7000,     LENGTH = 0x1000
  RamLoc8  (rwx) : ORIGIN = 0x10000000, LENGTH = 0x2000
}
__top_RamLoc8 = 0x10000000 + 0x2000;

ENTRY(ResetISR)

SECTIONS
{
  .text : { KEEP(*(.isr_vector)) *(.text*) *(.rodata*) } > MFlash32
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > MFlash32
  __exidx_start = .;
  .ARM.exidx : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } > MFlash32
  __exidx_end = .;
  _etext = .;
  .cmap : { KEEP(*(.Const_Map*)) } > FlashMap
  .data : { _data = .; *(vtable) *(.data*) _edata = .; } > RamLoc8 AT> MFlash32
  .bss  : { _bss = .; *(.bss*) *(COMMON) _ebss = .; } > RamLoc8
  .cr_heap : { end = .; _pvHeapStart = .; } > RamLoc8
  _vRamTop = __top_RamLoc8;
  _vStackTop = _vRamTop - 0;
}
