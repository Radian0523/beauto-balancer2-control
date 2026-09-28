"""ケーブルを繋いだまま、走行ログをリアルタイムでグラフに流す (./bb2 live)。

機体は走行中、ログを RAM のリングバッファ (srcbuf, USB の 0x1000 番地〜) に書き続けている。
それを USB で読み続けて、新しく増えたサンプルだけを取り出す。ファームの変更は不要。
  - 画面: 直近 WINDOW 秒をグラフで表示 (本体角・本体角速度・速度・電流)
  - 保存: logs/live_<日時>.csv に全部保存 (./bb2 stats にそのままかけられる)
  - 走行を止めて再スタートすると、新しいファイルに切り替わる
  - Ctrl+C またはグラフの窓を閉じると終了

注意: ケーブルの重さ・引っ張りで、ケーブル無しの走行とは動きが少し変わる。
"""
import csv
import datetime
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_log import (DIAG_NAMES, DIAG_SCALE, EXTRA, HDR, LOG_BASE, NAMES, SCALE,
                      WHEEL_R, open_dev, read, read_block)

WINDOW = 10.0        # グラフに表示する秒数
DRAW_EVERY = 0.15    # グラフを描き直す間隔 [s]
DEG = 57.29578


def columns(nvar, diag):
    names, scale = (DIAG_NAMES, DIAG_SCALE) if diag else (NAMES, SCALE)
    en, es = EXTRA.get(nvar, ([], []))
    return (names + en)[:nvar], (scale + es)[:nvar]


class Session:
    """1 回の走行 (スタート〜停止) ぶんの記録"""

    def __init__(self, names, rate):
        self.names, self.rate, self.n = names, rate, 0
        self.rows = []
        self.f = None
        self.path = "(まだ記録なし)"

    def add(self, vals):
        if self.f is None:      # 最初のサンプルが来たときにファイルを作る
            os.makedirs("logs", exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.path = f"logs/live_{stamp}.csv"
            self.f = open(self.path, "w", newline="")
            self.w = csv.writer(self.f)
            self.w.writerow(["t_s"] + self.names)
            print(f"記録開始: {self.path} ({self.rate:.0f} Hz)")
        t = self.n / self.rate
        self.n += 1
        self.w.writerow([f"{t:.3f}"] + [f"{x:.5g}" for x in vals])
        self.rows.append((t, vals))
        keep = int(WINDOW * self.rate) + 10
        if len(self.rows) > 2 * keep:
            self.rows = self.rows[-keep:]

    def close(self):
        if self.f is None:
            return
        self.f.close()
        print(f"保存: {self.path} ({self.n} サンプル, {self.n / self.rate:.1f} 秒)")


def main():
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["Hiragino Sans", "sans-serif"]

    dev = open_dev()
    print("接続 OK。機体をスタートさせると記録が始まる (Ctrl+C で終了)")
    fig, ax = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    plt.ion()
    plt.show(block=False)

    sess = None
    last_idx = last_cnt = None
    nvar = div = cap = None
    names = scale = None
    last_draw = 0.0
    try:
        while plt.fignum_exists(fig.number):
            hdr = read(dev, LOG_BASE, HDR)
            if hdr[:2] not in (b"LG", b"LD"):
                time.sleep(0.2)
                continue
            if (hdr[2], hdr[3]) != (nvar, div):
                nvar, div = hdr[2], hdr[3]
                cap = (4096 - HDR) // (nvar * 2)
                names, scale = columns(nvar, hdr[:2] == b"LD")
                last_idx = last_cnt = None
            idx, cnt = struct.unpack_from("<HH", hdr, 4)

            if last_cnt is None or cnt < last_cnt:
                # 初回、または再スタートで記録が消えた (cnt が減った) → 新しい走行として記録し直す
                if sess:
                    sess.close()
                sess = Session(names, 500.0 / div)
                if last_cnt is None and cnt >= cap:
                    last_idx = idx          # 繋いだ時点でリングが一周済み: 今あるぶんは捨ててここから
                else:
                    last_idx = (idx - cnt) % cap    # 今の走行の最初から読む
                last_cnt = cnt

            new = (idx - last_idx) % cap
            if new:
                if new > cap - 20:
                    print("警告: 読むのが追いつかずサンプルが抜けた")
                # 古い順に、リングの折り返しを考えて読む
                start = last_idx
                raw = b""
                remain = new
                while remain:
                    k = min(remain, cap - start)
                    raw += read_block(dev, LOG_BASE + HDR + start * nvar * 2, k * nvar * 2)
                    start = (start + k) % cap
                    remain -= k
                for i in range(new):
                    v = struct.unpack_from(f"<{nvar}h", raw, i * nvar * 2)
                    sess.add([a * b for a, b in zip(v, scale)])
                last_idx, last_cnt = idx, cnt

            now = time.time()
            if sess and sess.rows and now - last_draw > DRAW_EVERY:
                draw(ax, sess)
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                last_draw = now
            else:
                plt.pause(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        if sess:
            sess.close()
        dev.close()


def draw(ax, sess):
    t = [r[0] for r in sess.rows]
    c = list(zip(*[r[1] for r in sess.rows]))
    n = {k: i for i, k in enumerate(sess.names)}
    for a in ax:
        a.cla()
        a.grid(alpha=0.3)
    ax[0].plot(t, [x * DEG for x in c[n["body_angle_rad"]]], label="本体角")
    if "theta_ref_rad" in n:
        ref = [x * DEG for x in c[n["theta_ref_rad"]]]
        if "balance_angle_rad" in n:
            ref = [a + b * DEG for a, b in zip(ref, c[n["balance_angle_rad"]])]
        ax[0].plot(t, ref, "--", label="目標角 (釣り合う角度 + 前傾)")
    ax[0].set_ylabel("deg")
    ax[1].plot(t, [x * DEG for x in c[n["body_rate_rad_s"]]], label="本体角速度")
    ax[1].set_ylabel("deg/s")
    ax[2].plot(t, [x * WHEEL_R for x in c[n["wheel_spd_rad_s"]]], label="速度 (車輪)")
    if "target_spd_rad_s" in n:
        ax[2].plot(t, [x * WHEEL_R for x in c[n["target_spd_rad_s"]]], "--", label="目標速度")
    ax[2].set_ylabel("m/s")
    ax[3].plot(t, c[n["current_cmd"]], label="電流指令")
    if "m_current" in n:
        ax[3].plot(t, [x * 10 for x in c[n["m_current"]]], lw=0.8, label="実際の電流 ×10")
    ax[3].set_ylabel("電流")
    ax[3].set_xlabel("スタートからの時間 [s]")
    ax[3].set_xlim(max(0.0, t[-1] - WINDOW), max(WINDOW, t[-1]))
    for a in ax:
        a.legend(loc="upper left", fontsize=8)
    ax[0].set_title(f"リアルタイム {sess.path}  ({t[-1]:.1f} s)")


if __name__ == "__main__":
    main()
