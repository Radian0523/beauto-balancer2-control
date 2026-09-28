"""機体が受け取っているコントローラの値をその場で表示する (./bb2 pad)。

機体の電源を入れて USB を挿し、コントローラの電源も入れて実行する。Ctrl+C で終了。
R1 / R2 を押したときの「押されたビット」が、make の BTNSLOW / BTNFAST (既定 0x0800 / 0x0200) と合っているか確かめる。
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
    print("ボタンに触らずに起動すること (その時の値を基準にする)")
    print("ボタン   押されたビット  右X  右Y  左X  左Y   (Ctrl+C で終了)")
    idle = None
    try:
        while True:
            m = read(dev, PAD, 6)
            btn, = struct.unpack_from("<H", m, 0)
            rx, ry, lx, ly = struct.unpack_from("<4b", m, 2)
            if idle is None:
                idle = btn
            pressed = btn ^ idle
            ua = struct.unpack("<3d", read(dev, 120, 24))    # 機体が書き出した値 (走行中だけ更新される)
            print(f"\r{btn:#06x}  {pressed:#06x}        {rx:4d} {ry:4d} {lx:4d} {ly:4d}   "
                  f"| 機体: 基準 {int(ua[0]):#06x} 押された {int(ua[1]):#06x} 目標速度 {ua[2]:+.2f} m/s   ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    dev.close()


if __name__ == "__main__":
    main()
