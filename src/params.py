"""
ビュートバランサー2 (Vstone) の物理パラメータと線形モデル.

出典:
  - 製品仕様 https://www.vstone.co.jp/products/beauto_balancer_2/spec.html
      寸法 177(H)x90(W)x42(D) mm / 重量 約150g / モータ FA-130-2270 x2
  - 取扱説明書 11章
      ジャイロ   : 村田 ENC-03RC/D, +-300deg/s, 分解能 1.2025 deg/s, 取得周期 2ms
      エンコーダ : 減速比 21:1, タイヤ径 42mm, 速度分解能 0.01246664 rad/s
      電流制御   : 分解能 1.61133mA, 制御周期 1ms (PI制御, 指令値は mA)
      寸法図     : 重心位置 車輪中心軸から Y = 74.5mm
  - モータ FA-130 のカタログ値 (要現地同定)

!! 印のパラメータは事前に確定できないので、当日 sysid.py で同定して上書きする。
"""
from dataclasses import dataclass, replace
import numpy as np

G = 9.80665


@dataclass
class Params:
    # ---- 機体 ----
    m_b: float = 0.136      # !! 本体質量 [kg]  (全体150g - 車輪系14g)
    m_w: float = 0.014      # !! 車輪2個の質量 [kg]
    l:   float = 0.0745     # 重心高さ(車軸から) [m]   ← 寸法図より確定値
    r:   float = 0.021      # タイヤ半径 [m]           ← φ42mm より確定値
    W:   float = 0.090      # 左右車輪間隔 [m]         (本体幅90mmから概算)
    J_b: float = 2.0e-4     # !! 本体の重心まわり慣性 [kg m^2]
    J_w: float = 3.1e-6     # !! 車輪2個の車軸まわり慣性 [kg m^2]
    J_psi: float = 1.5e-4   # !! ヨー慣性 [kg m^2]

    # ---- 駆動系 ----
    n:   float = 21.0       # 減速比                    ← 確定値
    Kt:  float = 1.43e-3    # !! モータトルク定数 [Nm/A] (FA-130 カタログ推定)
    Ke:  float = 1.43e-3    # !! 逆起電力定数 [V/(rad/s)]
    Rm:  float = 0.68       # !! 電機子抵抗 [ohm]
    eta: float = 0.70       # !! ギヤ効率
    J_rotor: float = 1.0e-7 # !! ロータ慣性 [kg m^2] (n^2倍で効くので影響大)
    V_bat: float = 3.0      # 電池電圧 [V] (単3 x2). 消耗すると下がる
    i_max: float = 3.0      # 電流指令の上限 [A]

    # ---- 摩擦 ----
    c_m: float = 2.0e-5     # !! 粘性摩擦 (車輪-本体間) [Nm/(rad/s)]
    tau_c: float = 1.2e-4   # !! クーロン摩擦 (ギヤ) [Nm] 車輪軸換算/1モータ
    c_r: float = 1.0e-5     # !! 転がり抵抗 [Nm/(rad/s)]
    mu: float = 0.8         # !! タイヤと床の摩擦係数 (これを超えるトルクは出ない)

    # ---- センサ ----
    Ts: float = 0.002               # 制御周期 [s] (= ジャイロ取得周期)
    gyro_lsb: float = np.deg2rad(1.202483675)   # ジャイロ分解能 [rad/s]
    gyro_noise: float = np.deg2rad(0.6)         # !! ジャイロ白色雑音 rms [rad/s]
    gyro_drift: float = np.deg2rad(0.8)         # !! バイアスドリフト [rad/s per sqrt(s)] 相当
    enc_lsb: float = 0.01246664 * 0.002         # 車輪角度分解能 [rad] (速度分解能xTs)
    i_lsb: float = 1.61133e-3                   # 電流分解能 [A]

    # ---- 導出量 ----
    @property
    def m(self):        # 全質量
        return self.m_b + self.m_w

    @property
    def Jw_eq(self):    # 車輪側の等価慣性 (ロータ慣性を減速比^2で換算して加算)
        return self.J_w + 2.0 * self.J_rotor * self.n ** 2

    @property
    def Kg(self):       # 電流[A] -> 車輪トルク[Nm] (1モータあたり)
        return self.n * self.eta * self.Kt


def linear_model(p: Params):
    """
    ピッチ平面の線形化モデル.
      状態 x = [phi, theta, dphi, dtheta]
        phi   : 車輪の絶対回転角 [rad] (進行距離 s = r*phi)
        theta : 本体の鉛直からの傾き [rad] (前傾が正)
      入力 u = 左右合計の車輪トルク [Nm]
      外乱 d = 車輪軸に働く外乱トルク [Nm]  (坂: d = -m*g*r*sin(alpha))

    非線形式 (導出は docs/02_model.md):
      M*ddphi + m_b*r*l*cos(th+al)*ddth - m_b*r*l*sin(th+al)*dth^2 + m*g*r*sin(al)
            = u - c_m*(dphi-dth) - c_r*dphi
      (m_b*l^2+J_b)*ddth + m_b*r*l*cos(th+al)*ddphi - m_b*g*l*sin(th)
            = -u + c_m*(dphi-dth)
    """
    M11 = p.m * p.r ** 2 + p.Jw_eq        # ddphi の係数 (phi式)
    M12 = p.m_b * p.r * p.l
    M22 = p.m_b * p.l ** 2 + p.J_b
    Mm = np.array([[M11, M12], [M12, M22]])

    # 重力・摩擦の線形項:  Mm*[ddphi,ddth] = Kx*[phi,th] + Cx*[dphi,dth] + Bu*u + Bd*d
    Kx = np.array([[0.0, 0.0],
                   [0.0, p.m_b * G * p.l]])
    Cx = np.array([[-(p.c_m + p.c_r), p.c_m],
                   [p.c_m, -p.c_m]])
    Bu = np.array([[1.0], [-1.0]])
    Bd = np.array([[1.0], [0.0]])

    Minv = np.linalg.inv(Mm)
    A = np.zeros((4, 4))
    A[0, 2] = 1.0
    A[1, 3] = 1.0
    A[2:, 0:2] = Minv @ Kx
    A[2:, 2:] = Minv @ Cx
    B = np.vstack([np.zeros((2, 1)), Minv @ Bu])
    Bd4 = np.vstack([np.zeros((2, 1)), Minv @ Bd])
    return A, B, Bd4


def yaw_model(p: Params):
    """ヨー(旋回)の1次モデル:  J_psi*ddpsi = (W/(2r))*(tau_R - tau_L) - c_psi*dpsi"""
    c_psi = 2.0 * p.c_m * (p.W / (2 * p.r)) ** 2
    A = np.array([[0.0, 1.0], [0.0, -c_psi / p.J_psi]])
    B = np.array([[0.0], [(p.W / (2 * p.r)) / p.J_psi]])
    return A, B


if __name__ == "__main__":
    p = Params()
    A, B, Bd = linear_model(p)
    np.set_printoptions(precision=4, suppress=False, linewidth=120)
    print("A =\n", A)
    print("B =\n", B.ravel())
    print("Bd=\n", Bd.ravel())
    print("\n開ループ極:", np.linalg.eigvals(A))
    print("倒れる時定数 [s]:", 1.0 / max(np.linalg.eigvals(A).real))
    print("Jw_eq = %.3e (うちロータ換算 %.3e)" % (p.Jw_eq, 2 * p.J_rotor * p.n ** 2))
    print("電流1A -> 車輪トルク %.4f Nm (2モータ)" % (2 * p.Kg))
    # 可制御性
    C = np.hstack([B, A @ B, A @ A @ B, A @ A @ A @ B])
    print("可制御性ランク:", np.linalg.matrix_rank(C))
