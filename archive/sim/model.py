"""非線形プラントモデル (ピッチ + ヨー + モータ電気系 + 摩擦 + 飽和).

速度重視のためスカラー演算は math で書く (numpy スカラーは遅い)。
"""
from math import sin, cos, copysign, sqrt
import numpy as np
from params import Params, G


class Plant:
    """
    状態 x = [phi, theta, dphi, dtheta, psi, dpsi]
      phi   : 車輪の絶対回転角(左右平均) [rad], 進行距離 s = r*phi
      theta : 本体傾き(鉛直基準, 前傾+) [rad]
      psi   : ヨー角 [rad]
    入力 i_cmd = (i_L, i_R) 電流指令 [A]
    環境 alpha_fn(s) -> 路面勾配 [rad] (登りが正)
    """

    def __init__(self, p: Params, alpha_fn=None):
        self.p = p
        self.alpha_fn = alpha_fn or (lambda s: 0.0)
        self.x = np.zeros(6)
        # 定数を展開しておく
        self.M11 = p.m * p.r ** 2 + p.Jw_eq
        self.mrl = p.m_b * p.r * p.l
        self.M22 = p.m_b * p.l ** 2 + p.J_b
        self.c_psi = 2.0 * p.c_m * (p.W / (2 * p.r)) ** 2
        self.half_wr = p.W / (2 * p.r)

    def _one_motor(self, ic, dph, dth):
        p = self.p
        w_m = p.n * (dph - dth)
        emf = p.Ke * w_m
        i_hi = min((p.V_bat - emf) / p.Rm, p.i_max)
        i_lo = max((-p.V_bat - emf) / p.Rm, -p.i_max)
        i = ic if i_lo < ic < i_hi else (i_hi if ic >= i_hi else i_lo)
        t = p.Kg * i
        v_rel = dph - dth
        if v_rel > 1e-3 or v_rel < -1e-3:
            t -= copysign(p.tau_c, v_rel)
        else:
            t -= max(-p.tau_c, min(p.tau_c, t))
        return t, i

    def deriv(self, x, iL, iR):
        p = self.p
        phi, th, dphi, dth, psi, dpsi = x
        al = self.alpha_fn(p.r * phi)
        tL, iaL = self._one_motor(iL, dphi - self.half_wr * dpsi, dth)
        tR, iaR = self._one_motor(iR, dphi + self.half_wr * dpsi, dth)
        u = tL + tR
        u_diff = tR - tL
        # --- トラクション限界: 接地摩擦を超える駆動力は出せない (空転) ---
        #     |F| <= mu * m * g * cos(alpha)   ->  |u| <= mu*m*g*cos(al)*r
        u_slip = p.mu * p.m * G * cos(al) * p.r
        if u > u_slip:
            u = u_slip
        elif u < -u_slip:
            u = -u_slip

        s_ta, c_ta = sin(th + al), cos(th + al)
        M12 = self.mrl * c_ta
        rhs1 = (u - p.c_m * (dphi - dth) - p.c_r * dphi
                + self.mrl * s_ta * dth * dth - p.m * G * p.r * sin(al))
        rhs2 = -u + p.c_m * (dphi - dth) + p.m_b * G * p.l * sin(th)
        det = self.M11 * self.M22 - M12 * M12
        ddphi = (self.M22 * rhs1 - M12 * rhs2) / det
        ddth = (self.M11 * rhs2 - M12 * rhs1) / det
        ddpsi = (self.half_wr * u_diff - self.c_psi * dpsi) / p.J_psi
        return (dphi, dth, ddphi, ddth, dpsi, ddpsi), (iaL, iaR), al

    def step(self, i_cmd, dt, substeps=1):
        iL, iR = float(i_cmd[0]), float(i_cmd[1])
        h = dt / substeps
        x = self.x
        for _ in range(substeps):
            k1, i_act, al = self.deriv(x, iL, iR)
            x2 = [x[j] + 0.5 * h * k1[j] for j in range(6)]
            k2, _, _ = self.deriv(x2, iL, iR)
            x3 = [x[j] + 0.5 * h * k2[j] for j in range(6)]
            k3, _, _ = self.deriv(x3, iL, iR)
            x4 = [x[j] + h * k3[j] for j in range(6)]
            k4, _, _ = self.deriv(x4, iL, iR)
            x = np.array([x[j] + (h / 6.0) * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j])
                          for j in range(6)])
        self.x = x
        return np.array(i_act), al

    @property
    def fallen(self):
        return abs(self.x[1]) > 0.7854   # 45deg
