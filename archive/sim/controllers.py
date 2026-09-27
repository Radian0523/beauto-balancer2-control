"""制御器: 純正相当の状態FB / LQI速度サーボ + 外乱前置補償 + 参照ガバナ"""
import numpy as np
from params import Params, G
from design import design_lqr_pos, design_lqi_vel, design_yaw, equilibrium


class StockFB:
    """純正ファーム相当の 4状態フィードバック (= 手調整されたLQR)。積分なし。"""
    def __init__(self, p: Params, K=None):
        self.p = p
        # 純正既定値を「トルク[Nm]」系に換算 (符号は本コードの座標系に合わせた)
        to_Nm = 2 * p.Kg * 1e-3
        self.K = K if K is not None else np.array([1320., 36400., 396., 5460.]) * to_Nm * \
            np.array([-1, 1, -1, 1]) * 0.5
        self.yawK = design_yaw(p)

    def __call__(self, xh, ref, sat):
        u = -(self.K[0] * xh[0] + self.K[1] * xh[1] + self.K[2] * xh[2] + self.K[3] * xh[3])
        return u, 0.0, {}


class ModernCtl:
    """
    速度サーボ LQI + 外乱トルク前置補償 + 参照ガバナ + 転倒回避
      * u_ff   : KF が推定した tau_d から平衡姿勢/平衡トルクを作る (坂・シーソー対策)
      * ガバナ : 飽和/傾き/勾配に応じて指令速度を自動で落とす (突っ込んで転ぶのを防ぐ)
      * AW     : 飽和中は積分を止め、逆算 (back-calculation) で巻き戻す
    """
    def __init__(self, p: Params, v_slew=1.2, aw=2.0, th_guard=np.deg2rad(22),
                 use_ff=True, use_governor=True, use_recover=True,
                 k_ff=0.6, th_eq_max=np.deg2rad(6.0), tau_ff=0.20):
        self.p = p
        self.use_ff = use_ff
        self.use_governor = use_governor
        self.use_recover = use_recover
        # --- 外乱前置補償の安全弁 ------------------------------------------
        # τ̂d を θ_eq に換算して姿勢目標をずらすと、入力ゲイン誤差(Kt/ギヤ効率の
        # 同定誤差)に対して正帰還ループが立つ:
        #     ∂u/∂τ̂d = -1 + K_th/(m_b*g*l) ≒ +6.4
        #   ループゲイン ≒ 6.4 x (入力ゲイン誤差) なので、誤差16%超で発振する。
        # k_ff で余裕を持たせ、θ_eq は物理的にありえる範囲(±6deg≒坂20deg相当)で
        # 飽和させ、τ̂d は tau_ff[s] で低域通過させてから使う。
        self.k_ff = k_ff
        self.th_eq_max = th_eq_max
        self.tau_ff = tau_ff
        self.taud_f = 0.0
        self.K, _, _ = design_lqi_vel(p)
        self.Kpos, _, _ = design_lqr_pos(p)
        self.yawK = design_yaw(p)
        self.xi = 0.0
        self.v_cmd = 0.0          # ガバナ通過後の実効指令速度 [m/s]
        self.v_slew = v_slew      # 指令速度のスルーレート [m/s^2]
        self.aw = aw
        self.th_guard = th_guard
        self.u_prev = 0.0
        self.recover = 0
        self.sat_f = 0.0      # 飽和量の1次フィルタ
        self.err_f = 0.0      # 姿勢追従誤差の1次フィルタ

    def reset_integrator(self):
        self.xi = 0.0

    def __call__(self, xh, ref, sat):
        """
        xh  = [phi, theta, dphi, dtheta, b, tau_d]  (KF推定)
        ref = dict(v=目標速度[m/s], w=目標ヨー角速度[rad/s], hold=位置保持か)
        sat = 前回の飽和量 (u_cmd - u_applied) [Nm]  ← アンチワインドアップ用
        """
        p = self.p
        phi, th, dphi, dth, b, tau_d = xh

        # --- 参照ガバナ -------------------------------------------------
        # 「トルクが足りない/姿勢が崩れている」ときだけ速度指令を落とす。
        # 坂そのものでは減速しない (坂は前置補償で吸収するので減速は不要)。
        v_tgt = ref.get("v", 0.0)
        a = p.Ts / 0.15
        self.sat_f += a * (abs(sat) - self.sat_f)
        v_lim = 1.0
        if self.use_governor:
            v_lim -= min(self.sat_f / 0.02, 1.0) * 0.7      # 電流飽和が続いたら減速
            v_lim -= min(self.err_f / np.deg2rad(6), 1.0) * 0.5  # 姿勢が崩れたら減速
        v_tgt = np.clip(v_tgt * max(v_lim, 0.15), -1.5, 1.5)
        dv = np.clip(v_tgt - self.v_cmd, -self.v_slew * p.Ts, self.v_slew * p.Ts)
        self.v_cmd += dv
        dphi_ref = self.v_cmd / p.r

        # --- 外乱(坂)の前置補償: 平衡姿勢と平衡トルク
        self.taud_f += (p.Ts / self.tau_ff) * (tau_d - self.taud_f)
        if self.use_ff:
            th_eq, u_eq = equilibrium(p, self.taud_f, dphi_ref)
            th_eq = np.clip(self.k_ff * th_eq, -self.th_eq_max, self.th_eq_max)
        else:
            th_eq, u_eq = 0.0, 0.0

        # --- 転倒回避モード: ガードを超えたら全力で足を出しに行く
        self.err_f += a * (abs(th - th_eq) - self.err_f)
        if self.use_recover and abs(th - th_eq) > self.th_guard:
            self.recover = 1
            u = np.sign(th - th_eq) * 0.25          # 最大トルクで倒れる方向へ足を出す
            self.xi *= 0.9
            return u, self._yaw(xh, ref), dict(mode="recover", th_eq=th_eq, v_cmd=self.v_cmd)
        self.recover = 0

        if ref.get("hold", False):
            # 位置保持 (ゾーンA): phi も戻す
            K = self.Kpos
            e = np.array([phi - ref.get("phi_ref", 0.0), th - th_eq, dphi, dth])
            u = u_eq - K @ e
        else:
            # 速度追従
            self.xi += (dphi - dphi_ref) * p.Ts - self.aw * sat * p.Ts
            self.xi = np.clip(self.xi, -30.0, 30.0)
            z = np.array([th - th_eq, dphi - dphi_ref, dth, self.xi])
            u = u_eq - self.K @ z
        return u, self._yaw(xh, ref), dict(mode="run", th_eq=th_eq, v_cmd=self.v_cmd,
                                           u_eq=u_eq, xi=self.xi)

    def _yaw(self, xh, ref):
        psi = ref.get("psi_meas", 0.0)
        dpsi = ref.get("dpsi_meas", 0.0)
        return -(self.yawK[0] * (psi - ref.get("psi_ref", 0.0))
                 + self.yawK[1] * (dpsi - ref.get("w", 0.0)))
