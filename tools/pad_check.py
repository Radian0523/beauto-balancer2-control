"""機体が受け取っているコントローラの値をその場で表示する (./bb2 pad)。

機体の電源を入れて USB を挿し、コントローラの電源も入れて実行する。Ctrl+C で終了。
スティックから手を離したときに RX / LY が ±10 を超えていると、スタート時の値を中立として
覚える処理が無ければ、止まっていても旋回・前後に動いてしまう。
"""
import struct
import sys
import time

from read_log import open_dev, read

PAD = 114      # memmap の PAD_BTN の番地 (続いて RX, RY, LX, LY が 1 バイトずつ)


def main():
    dev = open_dev()
    print("ボタン    右X  右Y  左X  左Y   (Ctrl+C で終了)")
    try:
        while True:
            m = read(dev, PAD, 6)
            btn, = struct.unpack_from("<H", m, 0)
            rx, ry, lx, ly = struct.unpack_from("<4b", m, 2)
            print(f"\r{btn:#06x}  {rx:4d} {ry:4d} {lx:4d} {ly:4d}", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    dev.close()


if __name__ == "__main__":
    main()
