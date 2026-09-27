"""状態推定器: (a) 純正相当の HPF  (b) 拡張状態カルマンフィルタ"""
import numpy as np
from params import Params
from design import design_kf


class StockHPF:
    """
    純正ファーム相当:
        angle(t) = G*angle(t-1) + 0.5*(w(t)+w(t-1))*dt      (G = 0.99950)
    起動時に静止させてジャイロオフセットを平均で校正する。
    角速度は生のジャイロ、車輪角/角速度はエンコーダ差分。
    """
    def __init__(self, p: Params, G=0.99950):
        self.p, self.G = p, G
        self.th = 0.0
        self.w_prev = 0.0
        self.bias = 0.0
        self.calib_sum, self.calib_n = 0.0, 0
        self.enc_prev = 0.0
        self.dphi = 0.0

    def calibrate(self, gyro):
        self.calib_sum += gyro; self.calib_n += 1
        self.bias = self.calib_sum / self.calib_n

    def update(self, y, u):
        p = self.p
        w = y["gyro"] - self.bias
        self.th = self.G * self.th + 0.5 * (w + self.w_prev) * p.Ts
        self.w_prev = w
        enc = y["enc_mean"]
        self.dphi = (enc - self.enc_prev) / p.Ts + w   # 相対角速度 + 本体角速度
        self.enc_prev = enc
        return np.array([enc + self.th, self.th, self.dphi, w, 0.0, 0.0])


class KalmanIP:
    """
    拡張状態カルマンフィルタ (定常ゲイン)
      xh = [phi, theta, dphi, dtheta, b_gyro, tau_d]
      y  = [enc(=phi-theta), gyro(=dtheta+b_gyro)]
      u  = 左右合計の車輪トルク [Nm] (指令値ではなく、飽和後の推定実効値を入れる)
    """
    def __init__(self, p: Params, **kw):
        d = design_kf(p, **kw)
        self.Ad, self.Bd, self.C, self.L = d["Ad"], d["Bd"], d["C"], d["L"]
        self.x = np.zeros(6)
        self.p = p

    def calibrate(self, gyro):
        # 静止中はバイアス=ジャイロ生値。移動平均で初期値だけ入れておく
        self.x[4] += 0.02 * (gyro - self.x[4])

    def update(self, y, u):
        ym = np.array([y["enc_mean"], y["gyro"]])
        xp = self.Ad @ self.x + self.Bd.ravel() * u        # 予測
        self.x = xp + self.L @ (ym - self.C @ xp)          # 更新
        return self.x.copy()
