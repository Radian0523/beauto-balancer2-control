"""本番コースでの比較実験 (表 + 図を out/ に出力)"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from params import Params, G
from estimators import KalmanIP, StockHPF
from controllers import ModernCtl
from run_ablation import perturb
import sim, mission as MIS

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
plt.rcParams["font.family"] = ["Hiragino Sans", "Arial Unicode MS", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

CONFIGS = {
    "① HPF + 状態FB(積分あり)": dict(est="hpf", ff=False, gov=False),
    "② KF  + 同じ制御則":       dict(est="kf", ff=False, gov=False),
    "③ KF  + 外乱前置補償":     dict(est="kf", ff=True, gov=False),
    "④ ③ + 参照ガバナ/復帰":    dict(est="kf", ff=True, gov=True),
}


def build(cfg, p_nom, drift=None):
    est = StockHPF(p_nom) if cfg["est"] == "hpf" else KalmanIP(p_nom)
    ctl = ModernCtl(p_nom, use_ff=cfg["ff"], use_governor=cfg["gov"],
                    use_recover=cfg.get("gov", False))
    return ctl, est


def mc(cfg, n=16, v_nom=0.35, drift_scale=1.0, p_nom=None):
    p_nom = p_nom or Params()
    res = []
    for k in range(n):
        rng = np.random.default_rng(2000 + k)
        p_true = perturb(rng)
        p_true = type(p_true)(**{**p_true.__dict__,
                                 "gyro_drift": p_nom.gyro_drift * drift_scale})
        ctl, est = build(cfg, p_nom)
        lg = sim.run_mission(p_true, ctl, est, seed=k, v_nom=v_nom, box=0.5)
        m = lg["t"] > 3.0
        res.append(dict(zone=lg["zone"], goal=lg["t_goal"], fail=lg["fail"],
                        dist=lg["dist"],
                        drift_A=abs(lg["s"][lg["t"] < MIS.STAND_TIME]).max()
                        if (lg["t"] < MIS.STAND_TIME).any() else 0.0,
                        eerr=np.rad2deg(np.sqrt(np.mean((lg["th"][m] - lg["th_hat"][m]) ** 2)))
                        if m.any() else np.nan))
    goals = [r["goal"] for r in res if r["goal"]]
    return dict(goal_rate=100 * len(goals) / n,
                t_goal=np.mean(goals) if goals else np.nan,
                zone=np.mean([r["zone"] for r in res]),
                dist=np.mean([r["dist"] for r in res]),
                drift_A=np.mean([r["drift_A"] for r in res]),
                eerr=np.nanmean([r["eerr"] for r in res]))


def table1(n=16):
    print("\n=== 表1: 本番コース(A〜E, 6.2m) / 機体パラメータを±20〜30% 振った {} 回 ===".format(n))
    print(f"{'構成':<26}{'完走率%':>8}{'完走タイム s':>13}{'平均到達ゾーン':>14}"
          f"{'ゾーンA中の最大ずれ m':>20}{'角度推定誤差 deg':>18}")
    print("-" * 100)
    for name, cfg in CONFIGS.items():
        r = mc(cfg, n=n)
        print(f"{name:<26}{r['goal_rate']:>8.0f}{r['t_goal']:>13.1f}{r['zone']:>14.2f}"
              f"{r['drift_A']:>20.3f}{r['eerr']:>18.2f}")


def table2(n=10):
    print("\n=== 表2: ジャイロのドリフトに対する頑健性 (完走率%) ===")
    print(f"{'ドリフト強度':<14}" + "".join(f"{k:>26}" for k in CONFIGS))
    for ds, lab in [(0.25, "小 0.2deg/s/√s"), (1.0, "中 0.8deg/s/√s"), (2.5, "大 2.0deg/s/√s")]:
        row = f"{lab:<14}"
        for name, cfg in CONFIGS.items():
            r = mc(cfg, n=n, drift_scale=ds)
            row += f"{r['goal_rate']:>26.0f}"
        print(row)


def table3(n=10):
    print("\n=== 表3: 目標速度を上げたとき (完走率% / 完走タイム s) ===")
    print(f"{'目標速度':<12}" + "".join(f"{k:>26}" for k in CONFIGS))
    for v in [0.30, 0.45, 0.60, 0.80]:
        row = f"{v:<12.2f}"
        for name, cfg in CONFIGS.items():
            r = mc(cfg, n=n, v_nom=v)
            row += f"{r['goal_rate']:>18.0f}/{r['t_goal']:>7.1f}"
        print(row)


def figures():
    p_nom = Params()
    rng = np.random.default_rng(2005)
    p_true = perturb(rng)
    runs = {}
    for lab, cfg in [("① HPF + 状態FB", CONFIGS["① HPF + 状態FB(積分あり)"]),
                     ("④ KF + 前置補償 + ガバナ", CONFIGS["④ ③ + 参照ガバナ/復帰"])]:
        ctl, est = build(cfg, p_nom)
        runs[lab] = sim.run_mission(p_true, ctl, est, seed=5, v_nom=0.35, box=0.5)

    fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    for lab, lg in runs.items():
        ls = "--" if lab.startswith("①") else "-"
        ax[0].plot(lg["t"], lg["s"], ls, label=lab)
        ax[1].plot(lg["t"], np.rad2deg(lg["th"]), ls, label=lab + " 実際の傾き")
        ax[1].plot(lg["t"], np.rad2deg(lg["th_hat"]), ":", lw=1, label=lab + " 推定")
        ax[2].plot(lg["t"], np.rad2deg(lg["th"] - lg["th_hat"]), ls, label=lab)
        ax[3].plot(lg["t"], lg["i_L"], ls, label=lab + " 左電流")
    lg4 = runs["④ KF + 前置補償 + ガバナ"]
    ax[0].plot(lg4["t"], [MIS.GOAL] * len(lg4["t"]), "k:", lw=0.8)
    for name, s0, s1 in MIS.ZONES[1:]:
        ax[0].axhline(s1, color="0.85", lw=0.7)
    ax[0].set_ylabel("進行距離 s [m]"); ax[0].legend(fontsize=8); ax[0].set_title("本番コース走行 (A:直立15秒 → B直進 → C S字 → D坂 → E急坂/シーソー)")
    ax[1].set_ylabel("本体の傾き [deg]"); ax[1].legend(fontsize=7, ncol=2)
    ax[2].set_ylabel("角度推定誤差 [deg]"); ax[2].legend(fontsize=8)
    ax[3].set_ylabel("モータ電流 [A]"); ax[3].set_xlabel("時間 [s]"); ax[3].legend(fontsize=8)
    for a in ax: a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig1_mission.png"), dpi=130)

    # 図2: 坂での外乱推定 (起動直後の過渡は切る)
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    _r = runs["④ KF + 前置補償 + ガバナ"]
    _m = _r["t"] > 1.0
    lg = {k: v[_m] for k, v in _r.items() if isinstance(v, np.ndarray)}
    ax[0].plot(lg["s"], np.rad2deg(lg["alpha"]), label="路面の勾配(真値)")
    p = p_nom
    ax[0].plot(lg["s"], np.rad2deg(np.arcsin(np.clip(-lg["taud_hat"] / (p.m * G * p.r), -1, 1))),
               label="KFが推定した勾配 (τ̂d から換算)")
    ax[0].set_ylabel("勾配 [deg]"); ax[0].legend(fontsize=9)
    ax[1].plot(lg["s"], np.rad2deg(lg["th"]), label="実際の傾き")
    ax[1].plot(lg["s"], np.rad2deg(lg["th_eq"]), label="前置補償が指示する平衡姿勢 θ_eq")
    ax[1].set_ylabel("傾き [deg]"); ax[1].legend(fontsize=9)
    ax[2].plot(lg["s"], lg["v"], label="実速度")
    ax[2].plot(lg["s"], lg["v_cmd"], label="ガバナ後の速度指令")
    ax[2].set_ylabel("速度 [m/s]"); ax[2].set_xlabel("進行距離 s [m]"); ax[2].legend(fontsize=9)
    for a in ax: a.grid(alpha=.3)
    fig.suptitle("坂・シーソーを「勾配の推定」で乗り切る (構成④)")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "fig2_slope.png"), dpi=130)
    print("\n図を out/fig1_mission.png, out/fig2_slope.png に保存しました")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    if what in ("all", "t1"): table1()
    if what in ("all", "t2"): table2()
    if what in ("all", "t3"): table3()
    if what in ("all", "fig"): figures()
