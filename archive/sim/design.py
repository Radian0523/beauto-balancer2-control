"""
制御器の設計 (すべてオフラインで計算し、結果だけを機体に焼く)

  1) LQR  : 位置保持 (ゾーンA 直立15秒)
  2) LQI  : 速度追従サーボ (走行)
  3) 定常カルマンフィルタ: 傾き角 + ジャイロバイアス + 外乱トルク の同時推定
  4) ヨー LQR

重みは Bryson の規則 (Q_ii = 1/x_i,max^2, R = 1/u_max^2) で物理量から決める。
「ゲインを手で回す」のではなく「許容する振れ幅を決める」設計にしておくと、
当日パラメータを同定し直しても 5 秒で再設計できる。
"""
import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are, expm
from params import Params, linear_model, yaw_model, G


def _care(A, B, Q, R):
    P = solve_continuous_are(A, B, Q, np.atleast_2d(R))
    return np.linalg.solve(np.atleast_2d(R), B.T @ P).ravel()


# --------------------------------------------------------- 1) 位置保持 LQR
def design_lqr_pos(p: Params, phi_max=8.0, th_max=0.12, dphi_max=25.0,
                   dth_max=2.5, u_max=0.06, rho=1.0):
    """状態 x=[phi,theta,dphi,dtheta], 入力 u=左右合計の車輪トルク[Nm], u=-Kx"""
    A, B, _ = linear_model(p)
    Q = np.diag([1 / phi_max**2, 1 / th_max**2, 1 / dphi_max**2, 1 / dth_max**2])
    R = rho / u_max**2
    K = _care(A, B, Q, R)
    return K, A, B


# --------------------------------------------------------- 2) 速度サーボ LQI
def design_lqi_vel(p: Params, th_max=0.12, dphi_max=25.0, dth_max=2.5,
                   xi_max=4.0, u_max=0.06, rho=1.0):
    """
    状態 z=[theta, dphi, dtheta, xi],  xi = ∫(dphi - dphi_ref) dt
    位置 phi は制御しない (コースを走るので位置を戻す必要がない)。
    xi_max を小さくするほど積分が強い = 坂でも指令速度を守るが、飽和に弱くなる。
    """
    A4, B4, _ = linear_model(p)
    A = A4[1:, 1:]                       # A は phi に依存しないので厳密に縮約できる
    B = B4[1:]
    Aa = np.zeros((4, 4)); Aa[:3, :3] = A; Aa[3, 1] = 1.0
    Ba = np.vstack([B, [[0.0]]])
    Q = np.diag([1 / th_max**2, 1 / dphi_max**2, 1 / dth_max**2, 1 / xi_max**2])
    K = _care(Aa, Ba, Q, rho / u_max**2)
    return K, Aa, Ba


# ------------------------------------------------ 3) 定常カルマンフィルタ
def design_kf(p: Params, q_td=3e-3, q_bias=3e-3, q_th=1e-3, sigma_gyro=None,
              sigma_enc=None):
    """
    推定状態 xh = [phi, theta, dphi, dtheta, b_gyro, tau_d]
      b_gyro : ジャイロバイアス [rad/s]     (ENC-03 の温度ドリフトを吸収)
      tau_d  : 車輪軸の外乱トルク [Nm]      (坂/段差/積載/転がり抵抗をまとめて吸収)
               坂角 alpha に対して tau_d = -m*g*r*sin(alpha)
    観測 y = [ enc = phi - theta , gyro = dtheta + b_gyro ]

    ★ここが本機最大のポイント★
      ビュートバランサー2 には加速度センサが無い。純正ファームはジャイロ積分に
      HPF を掛けて誤差を捨てているが、HPF は「傾いたままの定常状態」を必ず 0 に
      引き戻すので、坂・シーソーでは原理的に角度を誤る (時定数 ~2s)。
      一方この KF は「モデル + エンコーダ + 既知トルク」から絶対角を復元するので、
      定常的な傾きも坂の勾配も推定できる。可観測性ランク 6 で確認済み。
    """
    A4, B4, Bd4 = linear_model(p)
    A = np.zeros((6, 6)); A[:4, :4] = A4; A[:4, 5:6] = Bd4
    B = np.vstack([B4, [[0.0]], [[0.0]]])
    C = np.array([[1.0, -1.0, 0, 0, 0, 0],
                  [0.0, 0.0, 0, 1.0, 1.0, 0]])

    Ts, n = p.Ts, 6
    Mbig = np.zeros((n + 1, n + 1)); Mbig[:n, :n] = A; Mbig[:n, n:] = B
    Ed = expm(Mbig * Ts)
    Ad, Bd = Ed[:n, :n], Ed[:n, n:]

    sg = sigma_gyro or np.sqrt(p.gyro_noise**2 + p.gyro_lsb**2 / 12)
    se = sigma_enc or (p.enc_lsb / np.sqrt(12) + 1e-5)
    Rn = np.diag([se**2, sg**2])
    Qc = np.zeros((6, 6))
    Qc[3, 3] = q_th**2      # 本体角加速度のモデル誤差
    Qc[4, 4] = q_bias**2    # バイアスのランダムウォーク
    Qc[5, 5] = q_td**2      # 外乱トルクのランダムウォーク (大きいほど坂に素早く追従)
    Qd = Qc * Ts

    P = solve_discrete_are(Ad.T, C.T, Qd, Rn)
    L = P @ C.T @ np.linalg.inv(C @ P @ C.T + Rn)     # 予測形 (current estimator)
    Acl = Ad - L @ C @ Ad
    return dict(Ad=Ad, Bd=Bd, C=C, L=L, poles=np.linalg.eigvals(Acl))


def observability_check(p: Params):
    kf = design_kf(p)
    Ad, C = kf["Ad"], kf["C"]
    O = np.vstack([C @ np.linalg.matrix_power(Ad, k) for k in range(6)])
    return np.linalg.matrix_rank(O, tol=1e-9), np.linalg.svd(O, compute_uv=False)


# ------------------------------------------------------------------ 4) ヨー
def design_yaw(p: Params, psi_max=0.3, dpsi_max=3.0, ud_max=0.03):
    A, B = yaw_model(p)
    K = _care(A, B, np.diag([1 / psi_max**2, 1 / dpsi_max**2]), 1 / ud_max**2)
    return K


# ------------------------------------------- 平衡点(坂での姿勢/トルク)の計算
def equilibrium(p: Params, tau_d, dphi_ref=0.0):
    """
    外乱トルク tau_d (坂) のもとで、指令速度 dphi_ref を保つ平衡点。
      u_eq     = -tau_d + (c_m + c_r)*dphi_ref
      theta_eq = u_eq / (m_b*g*l)       ← 坂では「坂の分だけ前傾する」のが正解
    KF が tau_d を推定できるので、これを前置補償(フィードフォワード)に使える。
    積分器が効いてくるのを待たずに坂に入った瞬間から姿勢を切り替えられる。
    """
    u_eq = -tau_d + (p.c_m + p.c_r) * dphi_ref
    th_eq = u_eq / (p.m_b * G * p.l)
    return th_eq, u_eq


if __name__ == "__main__":
    p = Params()
    np.set_printoptions(precision=5, suppress=True, linewidth=140)
    to_mA = 1e3 / (2 * p.Kg)

    Kp, A, B = design_lqr_pos(p)
    print("[1] 位置保持LQR  u=-Kx, x=[phi,theta,dphi,dtheta]")
    print("    K       =", Kp)
    print("    mA換算  =", Kp * to_mA)
    print("    純正既定 : [1320, -36400, 396, -5460]  (純正の符号系・要現地確認)")
    print("    閉ループ極:", np.sort_complex(np.linalg.eigvals(A - B @ Kp.reshape(1, -1))))

    Kv, Aa, Ba = design_lqi_vel(p)
    print("\n[2] 速度サーボLQI z=[theta,dphi,dtheta,xi]")
    print("    K       =", Kv, "\n    mA換算  =", Kv * to_mA)
    print("    閉ループ極:", np.sort_complex(np.linalg.eigvals(Aa - Ba @ Kv.reshape(1, -1))))

    rank, sv = observability_check(p)
    kf = design_kf(p)
    print("\n[3] KF 可観測性ランク =", rank, " 最小特異値 = %.2e" % sv[-1])
    print("    推定器の離散極 |z| =", np.sort(np.abs(kf["poles"])))
    print("    -> 連続換算の時定数 [s]:", np.sort(-p.Ts / np.log(np.clip(np.abs(kf["poles"]), 1e-9, 0.999999))))
    print("    L =\n", kf["L"])

    print("\n[4] ヨー LQR K =", design_yaw(p))
    th_eq, u_eq = equilibrium(p, -p.m * G * p.r * np.sin(np.deg2rad(10)))
    print("\n[5] 10deg の坂での平衡: 前傾 %.2f deg, 必要トルク %.4f Nm (= %.0f mA)"
          % (np.rad2deg(th_eq), u_eq, u_eq * to_mA))


# ---------------------------------------------------------- C ヘッダの自動生成
def emit_gains_h(p: Params, path, ctl_kw=None, kf_kw=None):
    """設計結果を firmware/gains.h に書き出す。同定し直したら再実行するだけでよい。"""
    import datetime
    ctl_kw = ctl_kw or {}
    kf_kw = kf_kw or {}
    Kpos, _, _ = design_lqr_pos(p, **ctl_kw.get("pos", {}))
    Kv, _, _ = design_lqi_vel(p, **ctl_kw.get("vel", {}))
    Ky = design_yaw(p, **ctl_kw.get("yaw", {}))
    kf = design_kf(p, **kf_kw)

    def arr(name, M, fmt="%.8ef"):
        M = np.atleast_2d(M)
        if M.shape[0] == 1:
            body = ", ".join(fmt % v for v in M.ravel())
            return "static const float %s[%d] = {%s};\n" % (name, M.shape[1], body)
        rows = ",\n  ".join("{" + ", ".join(fmt % v for v in r) + "}" for r in M)
        return "static const float %s[%d][%d] = {\n  %s\n};\n" % (
            name, M.shape[0], M.shape[1], rows)

    s = ["/* 自動生成 (src/design.py) %s  —— 手で編集しないこと */\n"
         % datetime.date.today(), "#ifndef GAINS_H\n#define GAINS_H\n\n"]
    s.append("#define CTL_TS        %.8ef   /* 制御周期 [s] (MAIN_CYCLE=500Hz) */\n" % p.Ts)
    s.append("#define WHEEL_R       %.8ef   /* タイヤ半径 [m] */\n" % p.r)
    s.append("#define TREAD_W       %.8ef   /* 左右輪間隔 [m] */\n" % p.W)
    s.append("#define KG2           %.8ef   /* 電流1A(左右計2A)あたりの車輪トルク [Nm] */\n" % (2 * p.Kg))
    s.append("#define MGL           %.8ef   /* m_b*g*l [Nm/rad] */\n" % (p.m_b * G * p.l))
    s.append("#define I_CMD_MAX_MA  %.1ff   /* 片側モータの電流指令上限 [mA] */\n\n" % 2000.0)
    s.append(arr("K_POS", Kpos))
    s.append(arr("K_VEL", Kv))
    s.append(arr("K_YAW", Ky))
    s.append(arr("KF_AD", kf["Ad"]))
    s.append(arr("KF_BD", kf["Bd"].ravel()))
    s.append(arr("KF_L", kf["L"]))
    s.append("\n#endif /* GAINS_H */\n")
    with open(path, "w") as f:
        f.write("".join(s))
    return path
