"""
本番コース (2025年度ルールを再現):
  A 直立15秒 → B 直進 → C S字 → D ゆるい坂 → E きつい坂 + シーソー
距離 s [m] でゾーンを切り替え、ゾーンごとに目標速度とヨー目標を与える。
"""
import numpy as np

ZONES = [
    ("A_stand",  0.00, 0.00),
    ("B_straight", 0.00, 1.50),
    ("C_scurve", 1.50, 3.00),
    ("D_slope",  3.00, 4.20),
    ("E_steep",  4.20, 5.30),
    ("E_seesaw", 5.30, 6.20),
]
GOAL = 6.20
STAND_TIME = 15.0
LANE_HALF = 0.22          # 走路の半幅 [m] (これを超えたらコースアウト)
S_AMP = 0.20              # S字の振れ幅 [m]


def path_y(x):
    """コース中心線 y(x) [m]  — S字ゾーンだけ横に振れる"""
    if 1.50 <= x < 3.00:
        return S_AMP * np.sin(2 * np.pi * (x - 1.50) / 1.50)
    return 0.0


def path_heading(x):
    """中心線の接線方向 [rad]"""
    if 1.50 <= x < 3.00:
        return np.arctan(S_AMP * (2 * np.pi / 1.50) * np.cos(2 * np.pi * (x - 1.50) / 1.50))
    return 0.0


def alpha(s):
    """路面勾配 [rad]"""
    def ramp(x, a, b):       # a->b で 0->1
        return min(max((x - a) / (b - a), 0.0), 1.0)
    # D: ゆるい坂 6deg (3.2-3.4 登り / 3.4-3.8 平ら / 3.8-4.0 下り)
    if 3.20 <= s < 4.05:
        a = np.deg2rad(6.0)
        return a * (ramp(s, 3.20, 3.40) - ramp(s, 3.80, 4.00))
    # E: きつい坂 15deg (エッジが急)
    if 4.40 <= s < 5.25:
        a = np.deg2rad(15.0)
        return a * (ramp(s, 4.40, 4.50) - ramp(s, 5.05, 5.15))
    # E: シーソー 8deg (支点通過で符号が反転)
    if 5.45 <= s < 6.05:
        return np.deg2rad(8.0) if s < 5.75 else -np.deg2rad(8.0)
    return 0.0


def zone_of(s):
    for name, s0, s1 in ZONES[1:]:
        if s0 <= s < s1:
            return name
    return "GOAL" if s >= GOAL else "A_stand"


def reference(t, s, v_nom=0.35):
    """時刻 t, 距離 s における指令"""
    if t < STAND_TIME:
        return dict(v=0.0, hold=True, phi_ref=0.0, psi_ref=0.0, w=0.0)
    z = zone_of(s)
    ref = dict(v=v_nom, hold=False, psi_ref=0.0, w=0.0)
    if z == "C_scurve":                       # S字: 中心線の接線方向を目標ヨー角に
        ref["psi_ref"] = path_heading(s)
        ref["w"] = 0.0
        ref["v"] = v_nom * 0.85
    elif z in ("D_slope", "E_steep"):
        ref["v"] = v_nom * 0.9
    elif z == "E_seesaw":
        ref["v"] = v_nom * 0.7                # シーソーはゆっくり渡る
    elif z == "GOAL":
        ref["v"] = 0.0
        ref["hold"] = True
        ref["phi_ref"] = None                 # その場停止
    return ref


def zone_index(name):
    names = [z[0] for z in ZONES] + ["GOAL"]
    return names.index(name) if name in names else 0
