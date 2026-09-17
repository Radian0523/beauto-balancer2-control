"""コース定義 (2025年度ルール: A 直立15秒 / B 直進 / C S字 / D ゆるい坂 / E きつい坂・シーソー)

alpha(s): 進行距離 s [m] における路面勾配 [rad] (登りが正)
"""
import numpy as np


def flat(s):
    return 0.0


def make_slope(s0, s1, deg, s2=None, s3=None):
    """s0->s1 で登り、s2->s3 で下る台形の坂"""
    a = np.deg2rad(deg)
    s2 = s0 + 0.6 if s2 is None else s2
    s3 = s1 + 0.6 if s3 is None else s3

    def f(s):
        if s0 <= s < s1:
            return a * (s - s0) / max(s1 - s0, 1e-6)
        if s1 <= s < s2:
            return a
        if s2 <= s < s3:
            return a * (1 - (s - s2) / max(s3 - s2, 1e-6))
        return 0.0
    return f


def make_seesaw(s_start=0.6, length=0.9, deg=7.0):
    """シーソー: 乗った瞬間は登り、支点を越えた瞬間に符号が反転する(不連続)"""
    a = np.deg2rad(deg)
    pivot = s_start + length / 2

    def f(s):
        if s < s_start or s > s_start + length:
            return 0.0
        return a if s < pivot else -a
    return f


def make_step(s0=0.8, deg=6.0, width=0.05):
    """段差(短い急勾配) — バンプ通過の外乱試験用"""
    a = np.deg2rad(deg)

    def f(s):
        if s0 <= s < s0 + width:
            return a
        if s0 + width <= s < s0 + 2 * width:
            return -a
        return 0.0
    return f


def yaw_ref_S(t, t0=0.0, amp=np.deg2rad(60), period=3.0):
    """S字: ヨー角の目標を正弦で振る"""
    if t < t0:
        return 0.0, 0.0
    ph = 2 * np.pi * (t - t0) / period
    return amp * np.sin(ph), amp * (2 * np.pi / period) * np.cos(ph)


SCENARIOS = {
    "A_stand":   dict(desc="ゾーンA 直立15秒 (その場)", alpha=flat, T=15.0, v=0.0, hold=True),
    "B_straight": dict(desc="ゾーンB 直進 2m",           alpha=flat, T=12.0, v=0.35, hold=False),
    "C_scurve":  dict(desc="ゾーンC S字",                alpha=flat, T=12.0, v=0.30, hold=False, yaw="S"),
    "D_slope5":  dict(desc="ゾーンD ゆるい坂 5deg",      alpha=make_slope(0.5, 0.8, 5.0, 1.6, 1.9), T=14.0, v=0.30, hold=False),
    "E_slope12": dict(desc="ゾーンE きつい坂 12deg",     alpha=make_slope(0.5, 0.8, 12.0, 1.6, 1.9), T=14.0, v=0.30, hold=False),
    "E_seesaw":  dict(desc="ゾーンE シーソー 7deg",      alpha=make_seesaw(0.6, 0.9, 7.0), T=14.0, v=0.25, hold=False),
    "F_bump":    dict(desc="段差 6deg x 5cm",            alpha=make_step(0.8, 6.0), T=12.0, v=0.30, hold=False),
}
