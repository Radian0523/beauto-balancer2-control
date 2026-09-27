"""走行ログ (logs/run_*.csv) を 止まっている / 加速中 / 一定速度 / 減速中 に分けて集計し、数字で比べる。

  ./bb2 stats                     最新 3 本を比べる
  ./bb2 stats logs/run_A.csv logs/run_B.csv ...   指定したログを比べる

見かた
  角速度の揺れ  … 本体角速度の標準偏差。11Hz の細かい振動の大きさ (小さいほど良い)
  速度の揺れ    … 車輪速度 - 目標速度 の標準偏差 (小さいほど良い)
  傾きのずれ    … 本体角 - 目標前傾 の平均。傾き推定の 0 点のずれ (0 に近いほど良い)
  電流の追従    … 実際の電流 ÷ 電流指令 (低速のときを 1 とした比)。速いときに下がるなら
                   モータの逆起電力で電圧が足りず、指令どおりの力が出ていない
倒れた (またはつかんで持ち上げた) 後は集計から外す。
"""
import csv
import glob
import os
import statistics as st
import sys

WHEEL_R = 0.021
GRAV = 9.81
DEG = 57.29578
FALL_CUR = 30000     # 電流指令が上限 (32767) 近くに 2 回続けて張り付いたら倒れたとみなす
FALL_CUT = 0.5       # 倒れる前のこの秒数も外す
END_CUT = 1.0        # 倒れていないときも、止める直前 (ボタン操作・持ち上げ) は外す


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return [{k: float(v) for k, v in r.items() if v != ""} for r in rows]


def cut_end(rows):
    """倒れた時刻 (なければ None) と、集計に使う行を返す"""
    fall = None
    for a, b in zip(rows, rows[1:]):
        if abs(a["current_cmd"]) >= FALL_CUR and abs(b["current_cmd"]) >= FALL_CUR:
            fall = a["t_s"]
            break
    end = (fall - FALL_CUT) if fall is not None else (rows[-1]["t_s"] - END_CUT)
    return fall, [r for r in rows if r["t_s"] < end]


def phase(r):
    v = r["target_spd_rad_s"] * WHEEL_R
    a = -r["theta_ref_rad"] * GRAV           # 目標前傾 = -a/g から目標加速度を戻す
    d = "前" if v > 0.02 else ("後" if v < -0.02 else "")
    if abs(a) < 0.05:
        return "止まっている" if not d else "一定速度" + d
    return ("加速中" if a * v > 0 else "減速中") + (d or ("前" if a > 0 else "後"))


def sd(xs):
    return st.pstdev(xs) if len(xs) > 1 else float("nan")


def current_tracking(rows):
    """実際の電流が指令にどれだけ付いてきているかを速度別に返す。実電流の列が無ければ None"""
    keys = [k for k in ("m_current_l", "m_current_r", "m_current") if k in rows[0]]
    if not keys:
        return None
    cmd = [r["current_cmd"] for r in rows]
    # 指令との相関の符号で向きをそろえてから平均する (モータの向きが逆でもよいように)
    act = []
    for key in keys:
        m = [r[key] for r in rows]
        s = 1.0 if sum(c * x for c, x in zip(cmd, m)) >= 0 else -1.0
        act.append([s * x for x in m])
    act = [sum(v) / len(v) for v in zip(*act)]
    spd = [abs(r["wheel_spd_rad_s"]) * WHEEL_R for r in rows]

    def slope(idx):
        num = sum(cmd[i] * act[i] for i in idx)
        den = sum(cmd[i] * cmd[i] for i in idx)
        return num / den if den > 0 and len(idx) >= 10 else float("nan")

    def peak(idx):
        """実電流の大きさの上位 10% の値 = 出せている電流の上限の目安"""
        a = sorted(abs(act[i]) for i in idx)
        return a[int(len(a) * 0.9)] if len(a) >= 10 else float("nan")

    low = slope([i for i in range(len(rows)) if spd[i] < 0.15])
    out = []
    for lo, hi in ((0.0, 0.15), (0.15, 0.3), (0.3, 0.45), (0.45, 9.0)):
        idx = [i for i in range(len(rows)) if lo <= spd[i] < hi]
        k = slope(idx)
        out.append((lo, hi, len(idx), k / low if low == low and low != 0 else float("nan"), peak(idx)))
    return low, out


def report(path):
    rows = load(path)
    fall, use = cut_end(rows)
    name = os.path.basename(path)
    print(f"\n== {name}   ({len(rows)} サンプル、" + (f"倒れた: {fall:+.2f} s" if fall is not None else "倒れていない") + ")")
    groups = {}
    for r in use:
        groups.setdefault(phase(r), []).append(r)
    print("  状態          件数  角速度の揺れ  速度の揺れ  傾きのずれ  電流指令の揺れ")
    order = ["止まっている", "加速中前", "一定速度前", "減速中前", "加速中後", "一定速度後", "減速中後"]
    for g in order:
        l = groups.get(g, [])
        if len(l) < 5:
            continue
        rate = sd([r["body_rate_rad_s"] * DEG for r in l])
        verr = sd([(r["wheel_spd_rad_s"] - r["target_spd_rad_s"]) * WHEEL_R for r in l])
        tilt = st.mean([(r["body_angle_rad"] - r["theta_ref_rad"]) * DEG for r in l])
        cur = sd([r["current_cmd"] for r in l])
        print(f"  {g:<{12 - len(g)}} {len(l):>4}  {rate:>7.0f}°/s  {verr:>6.2f}m/s  {tilt:>+7.1f}°  {cur:>10.0f}")
    tr = current_tracking(use)
    if tr is None:
        print("  (実際の電流は記録されていない。古いログ)")
    else:
        low, out = tr
        print(f"  電流の追従 (低速で 実電流 = {low:.3f} × 指令 を 1 とした比) と、出せている実電流の上限の目安 (上位10%):")
        for lo, hi, n, k, pk in out:
            rng = f"{lo:.2f}〜{hi:.2f}" if hi < 9 else f"{lo:.2f}〜    "
            print(f"    速度 {rng} m/s  {n:>4} 件  比 {k:>5.2f}  上限 {pk:>5.0f}" if n >= 10 else f"    速度 {rng} m/s  {n:>4} 件   (少なすぎ)")
        high = [k for lo, hi, n, k, pk in out if lo >= 0.3 and n >= 10 and k == k]
        if high and min(high) < 0.7:
            print("    → 速いときに電流が指令に届いていない。電圧 (逆起電力・電池) が限界の疑い")
        elif high:
            print("    → 速いときも電流は指令どおり出ている。速度の限界は制御側の問題")
    if "yaw_err_rad" in use[0]:
        still = [r for r in use if abs(r["turn_cmd_rad_s"]) < 1e-3]
        turning = len(use) - len(still)
        ye = [r["yaw_err_rad"] * WHEEL_R * 100 for r in use]
        print(f"  向き: 旋回の指令が出ていた時間 {turning / len(use) * 100:.0f}%、"
              f"向きのずれ 平均 {st.mean(ye):+.1f} cm 最大 {max(ye, key=abs):+.1f} cm")
        if still and turning == 0:
            print("    → スティックは中立。回ったならスティック以外 (左右の車輪の差) が原因")
    s = [r for r in use if phase(r) == "止まっている"]
    return {
        "name": name,
        "rate": sd([r["body_rate_rad_s"] * DEG for r in s]) if len(s) > 5 else float("nan"),
        "verr": sd([(r["wheel_spd_rad_s"] - r["target_spd_rad_s"]) * WHEEL_R for r in use]),
        "vmax": max(r["target_spd_rad_s"] for r in rows) * WHEEL_R,
        "vmin": min(r["target_spd_rad_s"] for r in rows) * WHEEL_R,
        "fall": fall,
    }


def main():
    paths = sys.argv[1:] or sorted(glob.glob("logs/run_*.csv"), key=os.path.getmtime)[-3:]
    if not paths:
        sys.exit("logs/run_*.csv が無い")
    res = [report(p) for p in paths]
    if len(res) > 1:
        print("\n== まとめ\n  ログ                      静止時の角速度の揺れ  速度の揺れ(全体)  目標速度 前/後      倒れた")
        for r in res:
            print(f"  {r['name']:<24} {r['rate']:>12.0f}°/s  {r['verr']:>14.2f}m/s    {r['vmax']:>+5.2f}/{r['vmin']:>+5.2f}m/s  "
                  + ("はい" if r["fall"] is not None else "いいえ"))


if __name__ == "__main__":
    main()
