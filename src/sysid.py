"""
当日10分でやるパラメータ同定。
バランサー2プログラマのログ機能 (500Hz, CSV) で取ったデータを食わせて、
物理パラメータを最小二乗で当てて gains.h を吐き直すところまで自動化する。

実験1 「振り子落とし」 : モータOFFで少し傾けて手を離し、ジャイロを記録
        -> omega_n^2 = m_b*g*l / (J_b + m_b*l^2)   … 倒立の速さそのもの
実験2 「車輪空転」     : 機体を持ち上げ(タイヤを浮かせ)、電流ステップを入れて
        車輪角速度を記録 -> 駆動ゲイン Kg と 車輪等価慣性 Jw_eq と粘性 c_m
実験3 「定常電流」     : 一定電流で回したときの定常速度 -> 逆起電力/摩擦
"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from params import Params, G


def fit_pendulum(t, gyro, mask=None):
    """
    実験1: モータOFFの自由倒れ。theta(t) ≈ theta0*cosh(wn*t) なので
    log|dtheta| の傾きから wn を読む (倒れ始めの線形域だけ使う)。
    戻り値: wn [rad/s],  および l/(J_b/m_b + l^2) の組み合わせ
    """
    g = np.abs(np.asarray(gyro))
    m = (g > np.max(g) * 0.05) & (g < np.max(g) * 0.7) if mask is None else mask
    A = np.vstack([t[m], np.ones(m.sum())]).T
    coef, *_ = np.linalg.lstsq(A, np.log(g[m]), rcond=None)
    return float(coef[0])


def unstable_pole(p: Params):
    """モデルの不安定極 (= 倒れる速さ)"""
    from params import linear_model
    A, _, _ = linear_model(p)
    return float(max(np.linalg.eigvals(A).real))


def fit_Jb_from_pole(lam, p: Params, lo=2e-5, hi=5e-3):
    """
    実測の不安定極 lam に一致する J_b を二分法で求める。
    ※ 車輪が自由に転がるぶん、単純な倒立振子の式 wn=sqrt(mgl/(J+ml^2)) では
      ずれる (車輪が逃げるので実際はもう少し速く倒れる)。必ず結合系で解くこと。
    """
    f = lambda J: unstable_pole(Params(**{**p.__dict__, "J_b": J})) - lam
    if f(lo) * f(hi) > 0:
        return None
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if f(lo) * f(mid) <= 0: hi = mid
        else: lo = mid
    return 0.5 * (lo + hi)


class WheelRig:
    """実験2用: 本体を手で固定してタイヤを空転させたときの1自由度モデル"""
    def __init__(self, p: Params):
        self.p, self.w = p, 0.0

    def step(self, i_cmd, dt):
        p = self.p
        emf = p.Ke * p.n * self.w
        i = np.clip(i_cmd, (-p.V_bat - emf) / p.Rm, (p.V_bat - emf) / p.Rm)
        i = np.clip(i, -p.i_max, p.i_max)
        tau = 2 * p.Kg * i - 2 * p.c_m * self.w
        tau -= 2 * p.tau_c * np.sign(self.w) if abs(self.w) > 1e-3 else np.clip(tau, -2 * p.tau_c, 2 * p.tau_c)
        self.w += tau / p.Jw_eq * dt
        return self.w, i


def fit_motor(t, i_cmd, wheel_spd):
    """
    実験2: タイヤを浮かせて電流ステップ i0 を入れる。
      Jw_eq * dw/dt = Kg2*i - c*w - tau_c*sign(w)
    立ち上がりの傾きから Kg2/Jw_eq、定常速度から (Kg2*i - tau_c)/c を得る。
    """
    i0 = np.median(i_cmd[i_cmd != 0])
    w = np.asarray(wheel_spd)
    w_ss = np.median(w[int(len(w) * 0.8):])
    k = max(int(len(w) * 0.02), 3)
    slope = np.polyfit(t[:k], w[:k], 1)[0]
    return dict(i0=i0, w_ss=w_ss, dwdt0=slope,
                Kg2_over_J=slope / i0, c_over_Kg2=i0 / max(w_ss, 1e-6))


def update_params(p: Params, wn=None, Kg2=None, Jw_eq=None, c_m=None, tau_c=None):
    """同定結果で Params を上書きした新しい Params を返す"""
    d = dict(p.__dict__)
    if wn is not None:
        Jb = fit_Jb_from_pole(wn, p)
        if Jb: d["J_b"] = Jb
    if Kg2 is not None:
        d["Kt"] = Kg2 / 2.0 / (p.n * p.eta)
        d["Ke"] = d["Kt"]
    if Jw_eq is not None:
        d["J_rotor"] = max((Jw_eq - p.J_w) / (2 * p.n ** 2), 1e-9)
    if c_m is not None:
        d["c_m"] = c_m
    if tau_c is not None:
        d["tau_c"] = tau_c
    return Params(**d)


def report(p: Params):
    from design import design_lqr_pos, design_lqi_vel, design_kf
    A, B, _ = __import__("params").linear_model(p)
    ev = np.linalg.eigvals(A)
    print("倒立の不安定極 %.2f rad/s (時定数 %.0f ms)" % (max(ev.real), 1000 / max(ev.real)))
    print("電流1A(両輪)あたりの車輪トルク %.4f Nm" % (2 * p.Kg))
    K, _, _ = design_lqr_pos(p)
    print("位置保持LQR K =", np.round(K, 5))
    K, _, _ = design_lqi_vel(p)
    print("速度サーボLQI K =", np.round(K, 5))


if __name__ == "__main__":
    # デモ: シミュレータで実験1・実験2を実行し、同定が効くか確認する
    from model import Plant
    p_true = Params(J_b=3.0e-4, Kt=1.1e-3, Ke=1.1e-3)   # 「実機」は公称値とズレている
    print("--- 実験1: 振り子落とし ---")
    pl = Plant(p_true); pl.x[1] = np.deg2rad(1.0)
    ts, gs = [], []
    for k in range(int(0.35 / p_true.Ts)):
        pl.step(np.zeros(2), p_true.Ts)
        ts.append(k * p_true.Ts); gs.append(pl.x[3])
    lam = fit_pendulum(np.array(ts), np.array(gs))
    p_nom = Params()
    Jb = fit_Jb_from_pole(lam, p_nom)
    print("  実測の不安定極 %.2f rad/s (真値 %.2f)" % (lam, unstable_pole(p_true)))
    print("  -> J_b = %.3e (真値 %.3e, 公称 %.3e)" % (Jb, p_true.J_b, p_nom.J_b))

    print("--- 実験2: 車輪空転 (本体を手で固定して 500mA ステップ) ---")
    rig = WheelRig(p_true)
    ts, ws = [], []
    for k in range(int(1.0 / p_true.Ts)):
        w, _ = rig.step(0.5, p_true.Ts)
        ts.append(k * p_true.Ts); ws.append(w)
    r = fit_motor(np.array(ts), np.full(len(ts), 0.5), np.array(ws))
    print("  立上り勾配から Kg2/Jw_eq = %.1f (真値 %.1f)"
          % (r["Kg2_over_J"], 2 * p_true.Kg / p_true.Jw_eq))
    print("  無負荷定常速度 %.1f rad/s -> Ke*n = %.5f (真値 %.5f)"
          % (r["w_ss"], p_true.V_bat / max(r["w_ss"], 1e-6) / p_true.n * p_true.n,
             p_true.Ke * p_true.n))
    print("\n--- 同定後のモデルで再設計 ---")
    p_id = update_params(p_nom, wn=lam)
    report(p_id)
