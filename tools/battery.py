"""繋いでいる機体の電池の状態を測る (./bb2 battery)。

memmap に電池の電圧は無いので、車輪を浮かせて空回ししたときの最高の回転の速さで測る。
空回しの速さはモータの逆起電力 = 電池の電圧で決まる (電圧が下がると遅くなる)。

手順: 機体の電源を入れて USB を挿す → 待機中 (スタート前) のまま、機体を持ち上げて車輪を浮かせる → Enter
  - 左右の車輪が前向きに約 1.5 秒、全力で回る。車輪に触らない・床に着けない
  - 測った値は logs/battery_history.csv に追記する。満充電のときの値と比べて見る

しくみ: 機体の待機中は、v5 は電流指令を書かないので、USB から memmap に直接書き込む。
  MODE の電流制御ビットを立て、T_CURRENT_L/R に一定の指令を書く。空回しは電流がほとんど要らないので
  電流制御は電圧の上限まで上げきり、そのときの速さ = 電池の電圧で決まる最高の速さになる。
  終わったら (Ctrl+C でも) 指令を 0 に戻し、MODE を元に戻す。
"""
import csv
import datetime
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_log import WHEEL_R, open_dev, read

ADDR_MODE = 3               # memmap の番地 (firmware/vendor/lib/Balancer2.h の並び)
ADDR_T_CURRENT = 8          # T_CURRENT_L, T_CURRENT_R (short × 2)
ADDR_WHEEL_ANGLE = 56       # WHEEL_ANGLE_L, WHEEL_ANGLE_R (double × 2) [rad]
MODE_CURRENT_CTL_EN = 0x01
CMD = 4000                  # 電流指令。空回しなら上限まで回りきる大きさ
SPIN_T = 1.5                # 回す時間 [s]
MEASURE_T = 0.5             # 最後のこの秒数の平均の速さを使う (回り始めは除く)
HISTORY = "logs/battery_history.csv"
FULL = 46.0                 # 満充電の目安 [rad/s] (2026-09-27〜28 の空回しで 44〜50)。新しい電池で測って直す


def write(dev, addr, data):
    out = bytearray(65)
    out[1:5] = bytes([ord("W"), addr & 0xFF, addr >> 8, len(data)])
    out[5:5 + len(data)] = data
    dev.write(out)
    time.sleep(0.01)


def set_current(dev, u):
    write(dev, ADDR_T_CURRENT, struct.pack("<hh", u, u))


def wheel_angles(dev):
    return struct.unpack("<2d", read(dev, ADDR_WHEEL_ANGLE, 16))


def main():
    dev = open_dev()
    mode0 = read(dev, ADDR_MODE, 1)[0]
    if mode0 & MODE_CURRENT_CTL_EN:
        sys.exit("機体が走行中 (電流制御が有効)。ボタンで止めて、待機中にしてから実行する。")
    print("機体を持ち上げて、車輪を浮かせる。左右の車輪が前向きに 1.5 秒、全力で回る。")
    try:
        input("準備ができたら Enter (やめるなら Ctrl+C) ")
    except KeyboardInterrupt:
        print()
        return

    samples = []
    try:
        set_current(dev, 0)
        write(dev, ADDR_MODE, bytes([mode0 | MODE_CURRENT_CTL_EN]))
        set_current(dev, CMD)
        t0 = time.time()
        while time.time() - t0 < SPIN_T:
            set_current(dev, CMD)       # 念のため書き直す (機体側に上書きされていないか)
            samples.append((time.time(), *wheel_angles(dev)))
            time.sleep(0.02)
    finally:
        set_current(dev, 0)
        write(dev, ADDR_MODE, bytes([mode0]))
        set_current(dev, 0)
    dev.close()

    use = [s for s in samples if s[0] - samples[0][0] > SPIN_T - MEASURE_T]
    if len(use) < 3:
        sys.exit("測れなかった (読み出しが少なすぎる)。もう一度実行する。")
    dt = use[-1][0] - use[0][0]
    wl = abs(use[-1][1] - use[0][1]) / dt
    wr = abs(use[-1][2] - use[0][2]) / dt
    w = (wl + wr) / 2
    if w < 5:
        sys.exit(f"車輪がほとんど回らなかった (左 {wl:.1f}、右 {wr:.1f} rad/s)。"
                 "車輪が浮いているか、機体が待機中か確かめる。")

    pct = w / FULL * 100
    print(f"空回しの速さ: 左 {wl:.1f} rad/s、右 {wr:.1f} rad/s、平均 {w:.1f} rad/s ({w * WHEEL_R:.2f} m/s)")
    print(f"満充電の目安 {FULL:.0f} rad/s に対して {pct:.0f}%")
    if pct < 88:
        print("  → 電池が弱っている。充電・交換する")
    elif pct < 94:
        print("  → そろそろ換えどき")
    else:
        print("  → 十分")

    os.makedirs("logs", exist_ok=True)
    new = not os.path.exists(HISTORY)
    with open(HISTORY, "a", newline="") as f:
        wtr = csv.writer(f)
        if new:
            wtr.writerow(["time", "left_rad_s", "right_rad_s", "mean_rad_s"])
        wtr.writerow([datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), f"{wl:.2f}", f"{wr:.2f}", f"{w:.2f}"])
    print(f"記録: {HISTORY}")


if __name__ == "__main__":
    main()
