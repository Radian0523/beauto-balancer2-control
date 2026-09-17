"""KF のプロセス雑音 (q_td, q_bias, q_th) をシミュレーションで最適化する."""
import sys, itertools, numpy as np
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from params import Params, G
from estimators import KalmanIP
from controllers import ModernCtl
import course as CRS, sim


def evaluate(p, kw, scens=("E_slope12", "E_seesaw", "B_straight"), seeds=(0, 1, 2)):
    eth, etd = [], []
    for sn in scens:
        for sd in seeds:
            est = KalmanIP(p, **kw)
            lg = sim.run(p, ModernCtl(p, use_ff=False, use_governor=False), est,
                         CRS.SCENARIOS[sn], seed=sd)
            m = lg["t"] > 1.0
            eth.append(np.rad2deg(np.sqrt(np.mean((lg["th"][m] - lg["th_hat"][m]) ** 2))))
            td_true = -p.m * G * p.r * np.sin(lg["alpha"][m])
            etd.append(np.sqrt(np.mean((td_true - lg["taud_hat"][m]) ** 2)) / (p.m_b * G * p.l))
    return np.mean(eth), np.rad2deg(np.mean(etd))   # [deg], [deg相当]


if __name__ == "__main__":
    p = Params()
    print(f"{'q_td':>8}{'q_bias':>9}{'q_th':>8} | {'θ誤差RMS[deg]':>14}{'τd誤差[deg相当]':>16}")
    best = None
    for q_td in [3e-4, 1e-3, 3e-3, 1e-2]:
        for q_bias in [1e-4, 3e-4, 1e-3]:
            for q_th in [3e-3, 2e-2, 1e-1]:
                e1, e2 = evaluate(p, dict(q_td=q_td, q_bias=q_bias, q_th=q_th))
                J = e1 + 0.5 * e2
                if best is None or J < best[0]:
                    best = (J, q_td, q_bias, q_th, e1, e2)
                print(f"{q_td:>8.0e}{q_bias:>9.0e}{q_th:>8.0e} | {e1:>14.3f}{e2:>16.3f}")
    print("\n最良: q_td=%.0e q_bias=%.0e q_th=%.0e  θ誤差 %.3f deg, τd誤差 %.3f deg相当"
          % best[1:])
