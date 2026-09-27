"""LPC13xx のブートローダが要求するベクタ表チェックサムを .bin に書き込む。
先頭 8 ワードの和が 0 になるよう 8 ワード目(0x1C)を埋める。純正 IDE の checksum コマンドと同じ処理。"""
import struct, sys

path = sys.argv[1]
with open(path, "r+b") as f:
    w = list(struct.unpack("<7I", f.read(28)))
    f.seek(0x1C)
    f.write(struct.pack("<I", (-sum(w)) & 0xFFFFFFFF))
print(f"checksum written: {path}")
