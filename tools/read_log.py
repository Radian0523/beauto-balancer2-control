"""v5 の走行ログを吸い出してグラフと CSV にする。

手順: 走らせる → ボタンで止める(倒れて止まってもよい) → 電源は切らずに USB を挿す → これを実行
    .venv/bin/python tools/read_log.py
logs/ に CSV と PNG を保存して、PNG を開く。sudo は不要。

通信: 機体は USB HID "Balancer2" として見える。64 バイトのレポートで
  'R', 番地(下位,上位), 長さ  → 'R', 長さ, データ...   (memmap は 0〜351 番地、ログは 0x1000〜)
"""
import csv
import datetime
import os
import struct
import subprocess
import sys
import time

import hid

VID, PID = 0x1962, 0x2080
LOG_BASE = 0x1000
HDR = 8
NAMES = ["body_angle_rad", "theta_ref_rad", "body_rate_rad_s", "wheel_spd_rad_s",
         "target_spd_rad_s", "current_cmd", "wheel_err_rad", "wheel_err_int_rad_s"]
SCALE = [1e-4, 1e-4, 1e-3, 1e-2, 1e-2, 1.0, 1e-3, 1e-3]
WHEEL_R = 0.021
# 診断版 (make v5 DIAG=1) の並び
DIAG_NAMES = ["body_angle_rad", "gyro_raw_mean_lsb", "body_rate_rad_s", "wheel_spd_rad_s",
              "theta_rawint_rad", "current_cmd", "gyro_updates", "wheel_err_int_rad_s"]
DIAG_SCALE = [1e-4, 1e-2, 1e-3, 1e-2, 1e-4, 1.0, 1.0, 1e-3]
GYRO_K = 0.041849 * 0.125     # 純正 GyroProc の換算 [rad/s per LSB]
# 10 変数版 (2026-09-27 夜から) は末尾に実際の電流 (左, 右) が付く
EXTRA_NAMES = ["m_current_l", "m_current_r"]
EXTRA_SCALE = [1.0, 1.0]


def open_dev():
    for _ in range(20):
        try:
            d = hid.device()
            d.open(VID, PID)
            return d
        except OSError:
            time.sleep(0.5)
    sys.exit("機体が見つからない。電源を入れたまま USB を挿す (書き込みモードではなく通常起動で)。")


def read(dev, addr, n):
    out = bytearray(65)
    out[1:5] = bytes([ord("R"), addr & 0xFF, addr >> 8, n])
    dev.write(out)
    for _ in range(10):
        r = dev.read(64, 500)
        if r and r[0] == ord("R"):
            return bytes(r[2:2 + r[1]])
    raise RuntimeError(f"読み出しに応答がない (番地 {addr:#x})")


def read_block(dev, addr, n):
    buf = b""
    while len(buf) < n:
        k = min(60, n - len(buf))
        buf += read(dev, addr + len(buf), k)
    return buf


def main():
    dev = open_dev()
    pid = struct.unpack("<H", read(dev, 0, 2))[0]
    print(f"接続 OK (Product_ID {pid:#06x})")
    hdr = read(dev, LOG_BASE, HDR)
    if hdr[:2] not in (b"LG", b"LD"):
        sys.exit("ログが無い。v5 で走らせたあと電源を切らずに繋いだか確認する。")
    diag = hdr[:2] == b"LD"
    names, scale = (DIAG_NAMES, DIAG_SCALE) if diag else (NAMES, SCALE)
    nvar, div = hdr[2], hdr[3]
    names, scale = (names + EXTRA_NAMES)[:nvar], (scale + EXTRA_SCALE)[:nvar]
    idx, cnt = struct.unpack_from("<HH", hdr, 4)
    rate = 500.0 / div
    cap = (4096 - HDR) // (nvar * 2)
    print(f"{cnt} サンプル ({cnt / rate:.1f} 秒, {rate:.0f} Hz) を読み出し中...")
    if cnt == 0:
        sys.exit("記録が 0 件。止めたあとにもう一度スタートしてしまうと記録が消える (ボタンのチャタリングも含む)。")
    raw = read_block(dev, LOG_BASE + HDR, cap * nvar * 2)
    dev.close()

    rows = [struct.unpack_from(f"<{nvar}h", raw, i * nvar * 2) for i in range(cap)]
    start = (idx - cnt) % cap                 # リングの古い順に並べ直す
    rows = [rows[(start + i) % cap] for i in range(cnt)]
    t = [(i - cnt + 1) / rate for i in range(cnt)]     # 止めた瞬間を 0 秒
    data = [[v * k for v, k in zip(r, scale)] for r in rows]

    os.makedirs("logs", exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path_csv = f"logs/{'diag' if diag else 'run'}_{stamp}.csv"
    with open(path_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s"] + names)
        for ti, r in zip(t, data):
            w.writerow([f"{ti:.3f}"] + [f"{x:.5g}" for x in r])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]
    col = list(zip(*data))
    deg = 57.29578
    if diag:
        plot_diag(plt, t, col, rate, stamp)
        print(f"保存: {path_csv}\n保存: logs/diag_{stamp}.png")
        summarize_diag(t, col, rate)
        subprocess.run(["open", f"logs/diag_{stamp}.png"])
        return
    fig, ax = plt.subplots(5, 1, figsize=(11, 12), sharex=True)
    ax[0].plot(t, [x * deg for x in col[0]], label="本体角 (純正の推定)")
    ax[0].plot(t, [x * deg for x in col[1]], "--", label="目標角 (前傾FF)")
    ax[0].set_ylabel("deg")
    ax[1].plot(t, [x * deg for x in col[2]], label="本体角速度")
    ax[1].set_ylabel("deg/s")
    ax[2].plot(t, [x * WHEEL_R for x in col[3]], label="速度 (車輪)")
    ax[2].plot(t, [x * WHEEL_R for x in col[4]], "--", label="目標速度")
    ax[2].set_ylabel("m/s")
    ax[3].plot(t, col[5], label="電流指令 (左右平均)")
    if len(col) >= 10:
        ax[3].plot(t, col[8], lw=0.8, alpha=0.7, label="実際の電流 左 (生値)")
        ax[3].plot(t, col[9], lw=0.8, alpha=0.7, label="実際の電流 右 (生値)")
    ax[3].set_ylabel("T_CURRENT")
    ax[4].plot(t, [x * WHEEL_R * 100 for x in col[6]], label="目標とのずれ")
    ax[4].plot(t, [x * WHEEL_R * 100 for x in col[7]], label="ずれの積分 [cm*s]")
    ax[4].set_ylabel("cm")
    ax[4].set_xlabel("止めた瞬間からの時間 [s]")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc="upper left", fontsize=9)
    fig.suptitle(f"走行ログ {stamp} ({rate:.0f} Hz)")
    fig.tight_layout()
    path_png = f"logs/run_{stamp}.png"
    fig.savefig(path_png, dpi=110)
    print(f"保存: {path_csv}\n保存: {path_png}")
    subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "log_stats.py"), path_csv])
    subprocess.run(["open", path_png])


def plot_diag(plt, t, col, rate, stamp):
    deg = 57.29578
    fig, ax = plt.subplots(5, 1, figsize=(11, 12), sharex=True)
    ax[0].plot(t, [x * deg for x in col[0]], label="純正の傾き (0 に戻す処理あり)")
    ax[0].plot(t, [x * deg for x in col[4]], label="生ジャイロを単純積分した傾き")
    ax[0].set_ylabel("deg")
    ax[1].plot(t, col[1], label="生ジャイロ (0点を引いた値) の区間平均")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("LSB")
    ax[2].plot(t, col[6], label=f"区間 ({500 / rate:.0f} 周期) 内でジャイロの値が変わった回数")
    ax[2].set_ylabel("回")
    ax[3].plot(t, [x * WHEEL_R for x in col[3]], label="速度 (車輪)")
    ax[3].set_ylabel("m/s")
    ax[4].plot(t, col[5], label="電流指令")
    ax[4].plot(t, [x * 1320 / 2 for x in col[7]], label="積分項の寄与 (電流換算)")
    ax[4].set_ylabel("T_CURRENT")
    ax[4].set_xlabel("止めた瞬間からの時間 [s]")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc="upper left", fontsize=9)
    fig.suptitle(f"ジャイロ診断ログ {stamp} ({rate:.0f} Hz)")
    fig.tight_layout()
    fig.savefig(f"logs/diag_{stamp}.png", dpi=110)


def summarize_diag(t, col, rate):
    n = len(t)
    g = col[1]
    th = col[4]
    mean_g = sum(g) / n
    drift = (th[-1] - th[0]) / (t[-1] - t[0]) * 57.29578 if n > 1 else 0.0
    upd = sum(col[6]) / n
    print(f"生ジャイロの平均: {mean_g:+.2f} LSB = {mean_g * GYRO_K * 57.29578:+.2f} deg/s")
    print(f"単純積分した傾きのずれる速さ: {drift:+.2f} deg/s (全区間)")
    print(f"ジャイロの値が変わった回数: 平均 {upd:.1f} 回 / {500 / rate:.0f} 周期")


if __name__ == "__main__":
    main()
