"""センサモデル: ジャイロ(量子化+雑音+ドリフト), エンコーダ(量子化, 本体相対)."""
import numpy as np
from params import Params


class Sensors:
    def __init__(self, p: Params, rng=None, bias0=None):
        self.p = p
        self.rng = rng or np.random.default_rng(0)
        # ENC-03 は温度で数 deg/s 平気でずれる. 起動時オフセットは校正で除くが残差が出る
        self.bias = self.rng.normal(0, np.deg2rad(0.5)) if bias0 is None else bias0
        self.enc_prev = np.zeros(2)
        self.cnt_prev = np.zeros(2)

    def read(self, x, dt):
        p, rng = self.p, self.rng
        phi, th, dphi, dth, psi, dpsi = x
        # --- ジャイロ: 本体角速度 + バイアス + 白色雑音, 1.2deg/s で量子化 ---
        self.bias += rng.normal(0, p.gyro_drift * np.sqrt(dt))
        g_raw = dth + self.bias + rng.normal(0, p.gyro_noise)
        gyro = np.round(g_raw / p.gyro_lsb) * p.gyro_lsb
        gyro = np.clip(gyro, -np.deg2rad(300), np.deg2rad(300))   # +-300deg/s でサチる
        # --- エンコーダ: 本体に対する相対角. 左右で分ける ---
        phi_L = phi - (p.W / (2 * p.r)) * psi
        phi_R = phi + (p.W / (2 * p.r)) * psi
        cnt = np.floor(np.array([phi_L - th, phi_R - th]) / p.enc_lsb)
        enc = cnt * p.enc_lsb
        self.cnt_prev = cnt
        return dict(gyro=float(gyro), enc=enc, enc_mean=float(enc.mean()))
