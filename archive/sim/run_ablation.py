"""推定器と制御則の寄与を切り分ける比較実験 (アブレーション)."""
import sys, numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from params import Params
from estimators import KalmanIP, StockHPF
from controllers import ModernCtl
import course as CRS, sim

CONFIGS = {
    "A_HPF+LQI":      dict(est="hpf", ff=False, gov=False),
    "B_KF+LQI":       dict(est="kf",  ff=False, gov=False),
    "C_KF+LQI+FF":    dict(est="kf",  ff=True,  gov=False),
    "D_full(+gov)":   dict(est="kf",  ff=True,  gov=True),
}


def build(cfg, p):
    est = StockHPF(p) if cfg["est"] == "hpf" else KalmanIP(p)
    ctl = ModernCtl(p, use_ff=cfg["ff"], use_governor=cfg["gov"])
    return ctl, est


def perturb(rng, base=None):
    """機体パラメータの誤差 (同定しても残る不確かさ) を振る"""
    p = Params() if base is None else base
    f = lambda s: float(np.exp(rng.normal(0, s)))
    return Params(m_b=p.m_b * f(.05), J_b=p.J_b * f(.25), l=p.l * f(.03),
                  Kt=p.Kt * f(.20), Ke=p.Ke * f(.20), Rm=p.Rm * f(.15),
                  eta=min(0.95, p.eta * f(.15)), J_rotor=p.J_rotor * f(.35),
                  c_m=p.c_m * f(.5), tau_c=p.tau_c * f(.5), c_r=p.c_r * f(.5),
                  V_bat=p.V_bat * f(.06), J_psi=p.J_psi * f(.3))


def main(n_mc=12):
    p_nom = Params()
    rows = []
    for scen_name, scen in CRS.SCENARIOS.items():
        for cname, cfg in CONFIGS.items():
            ok, dist, tilt, eerr = 0, [], [], []
            for k in range(n_mc):
                rng = np.random.default_rng(1000 + k)
                p_true = perturb(rng)              # 実機 (真のパラメータ)
                ctl, est = build(cfg, p_nom)       # 制御器は公称値で設計
                lg = sim.run(p_true, ctl, est, scen, seed=k)
                sc = sim.score(lg)
                ok += (not sc["fell"])
                dist.append(sc["dist"]); tilt.append(sc["th_rms"]); eerr.append(sc["th_err_rms"])
            rows.append((scen_name, cname, 100 * ok / n_mc, np.mean(dist),
                         np.mean(tilt), np.mean(eerr)))
    print(f"{'シナリオ':<12}{'構成':<16}{'成功率%':>8}{'平均距離m':>11}{'傾きRMS deg':>13}{'角度推定誤差RMS deg':>20}")
    print("-" * 82)
    last = None
    for r in rows:
        if last and last != r[0]:
            print("-" * 82)
        print(f"{r[0]:<12}{r[1]:<16}{r[2]:>8.0f}{r[3]:>11.2f}{r[4]:>13.2f}{r[5]:>20.2f}")
        last = r[0]


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 12)
