"""シミュレーション本体: プラント + センサ + 推定器 + 制御器 を 500Hz で回す"""
import numpy as np
from params import Params
from model import Plant
from sensors import Sensors
import course as CRS


def allocate(p, u_sum, u_diff, i_cmd_max):
    """
    制御配分 (control allocation):
      倒立(前後トルクの和)を最優先、旋回(差トルク)は余った分だけ使う。
      |tau_L|,|tau_R| <= tau_max  <=>  |S| + |D| <= 2*tau_max
    こうしないと、S字で旋回指令が電流を食い潰して倒立側が飽和し、転倒する。
    """
    tau_max = p.Kg * i_cmd_max
    S = float(np.clip(u_sum, -2 * tau_max, 2 * tau_max))
    d_max = max(2 * tau_max - abs(S), 0.0)
    D = float(np.clip(u_diff, -d_max, d_max))
    tau = np.array([0.5 * (S - D), 0.5 * (S + D)])
    i_q = np.round(tau / p.Kg / p.i_lsb) * p.i_lsb
    i_q = np.clip(i_q, -i_cmd_max, i_cmd_max)
    u_applied = p.Kg * i_q.sum()
    return i_q, u_applied, u_sum - S        # 飽和量は倒立チャンネルのみで評価

def run(p: Params, ctl, est, scen, seed=0, calib_T=2.0, disturb=None,
        i_cmd_max=2.0, fric_ff=0.0, log_every=1):
    """
    ctl : controllers.* のインスタンス   (u[Nm], u_diff[Nm], info を返す)
    est : estimators.* のインスタンス
    scen: course.SCENARIOS の1要素
    disturb: t -> 外力トルク [Nm] (押し外乱試験用)
    """
    rng = np.random.default_rng(seed)
    plant = Plant(p, alpha_fn=scen["alpha"])
    sens = Sensors(p, rng=rng)
    plant.x[1] = rng.normal(0, np.deg2rad(0.5))     # 初期傾き誤差

    # --- 静止キャリブレーション (実機と同じく手で支えている想定) ---
    n_cal = int(calib_T / p.Ts)
    for _ in range(n_cal):
        y = sens.read(np.zeros(6), p.Ts)
        est.calibrate(y["gyro"])

    n = int(scen["T"] / p.Ts)
    log = {k: np.zeros(n) for k in
           ["t", "phi", "th", "dphi", "dth", "psi", "s", "v", "alpha",
            "th_hat", "dphi_hat", "taud_hat", "b_hat", "u", "i_L", "i_R",
            "th_eq", "v_cmd", "sat"]}
    sat = 0.0
    fell_at = None
    for k in range(n):
        t = k * p.Ts
        y = sens.read(plant.x, p.Ts)
        xh = est.update(y, np.clip(getattr(est, "_u_last", 0.0), -1, 1))

        ref = dict(v=scen.get("v", 0.0), hold=scen.get("hold", False),
                   psi_meas=plant.x[4], dpsi_meas=plant.x[5])
        if scen.get("yaw") == "S":
            pr, wr = CRS.yaw_ref_S(t, t0=1.0)
            ref.update(psi_ref=pr, w=wr)
        u, u_diff, info = ctl(xh, ref, sat)

        # 摩擦の前置補償 (オプション)
        if fric_ff:
            v_rel = xh[2] - xh[3]
            u += fric_ff * 2 * p.tau_c * np.tanh(v_rel / 0.5)

        # トルク -> 左右電流指令 -> 量子化 & 飽和
        i_q, u_applied, sat = allocate(p, u, u_diff, i_cmd_max)
        est._u_last = u_applied                  # 次回の予測に使う実効トルク

        if disturb is not None:
            pass
        i_act, al = plant.step(i_q, p.Ts)

        s = p.r * plant.x[0]
        for key, val in zip(["t", "phi", "th", "dphi", "dth", "psi", "s", "v", "alpha",
                             "th_hat", "dphi_hat", "taud_hat", "b_hat", "u", "i_L", "i_R",
                             "th_eq", "v_cmd", "sat"],
                            [t, plant.x[0], plant.x[1], plant.x[2], plant.x[3], plant.x[4],
                             s, p.r * plant.x[2], al, xh[1], xh[2], xh[5], xh[4],
                             u, i_q[0], i_q[1], info.get("th_eq", 0.0),
                             info.get("v_cmd", 0.0), sat]):
            log[key][k] = val

        if plant.fallen:
            fell_at = t
            for key in log:
                log[key] = log[key][:k + 1]
            break

    log["fell_at"] = fell_at
    log["dist"] = log["s"][-1] if len(log["s"]) else 0.0
    return log


def run_mission(p, ctl, est, seed=0, T=70.0, v_nom=0.35, calib_T=2.0,
                i_cmd_max=2.0, box=0.30):
    """本番コース(mission.py)を走らせる。戻り値に到達ゾーン/タイムを含む。"""
    import mission as MIS
    rng = np.random.default_rng(seed)
    plant = Plant(p, alpha_fn=MIS.alpha)
    sens = Sensors(p, rng=rng)
    plant.x[1] = rng.normal(0, np.deg2rad(0.5))
    for _ in range(int(calib_T / p.Ts)):
        est.calibrate(sens.read(np.zeros(6), p.Ts)["gyro"])

    n = int(T / p.Ts)
    keys = ["t", "s", "th", "th_hat", "taud_hat", "alpha", "v", "v_cmd", "psi",
            "psi_ref", "i_L", "i_R", "th_eq", "u", "b_hat", "sat", "x", "y", "lat_err"]
    log = {k: np.zeros(n) for k in keys}
    sat, u_applied = 0.0, 0.0
    x_pos = y_pos = 0.0
    fail, best_zone, t_goal = None, 0, None
    for k in range(n):
        t = k * p.Ts
        s = p.r * plant.x[0]
        y = sens.read(plant.x, p.Ts)
        xh = est.update(y, u_applied)
        ref = MIS.reference(t, s, v_nom)
        ref.update(psi_meas=plant.x[4], dpsi_meas=plant.x[5])
        if ref.get("phi_ref", 0.0) is None:
            ref["phi_ref"] = plant.x[0]
        u, u_diff, info = ctl(xh, ref, sat)
        i_q, u_applied, sat = allocate(p, u, u_diff, i_cmd_max)
        plant.step(i_q, p.Ts)
        v_now = p.r * plant.x[2]
        x_pos += v_now * np.cos(plant.x[4]) * p.Ts
        y_pos += v_now * np.sin(plant.x[4]) * p.Ts
        lat = y_pos - MIS.path_y(x_pos)

        for key, val in zip(keys, [t, s, plant.x[1], xh[1], xh[5], MIS.alpha(s),
                                   p.r * plant.x[2], info.get("v_cmd", 0.0), plant.x[4],
                                   ref.get("psi_ref", 0.0), i_q[0], i_q[1],
                                   info.get("th_eq", 0.0), u, xh[4], sat,
                                   x_pos, y_pos, lat]):
            log[key][k] = val
        best_zone = max(best_zone, MIS.zone_index(MIS.zone_of(s)))
        if plant.fallen:
            fail = "転倒"; break
        if t < MIS.STAND_TIME and abs(s) > box:
            fail = "ゾーンA枠外"; break
        if abs(lat) > MIS.LANE_HALF:
            fail = "コースアウト"; break
        if s >= MIS.GOAL:
            t_goal = t; best_zone = MIS.zone_index("GOAL"); break
    for key in keys:
        log[key] = log[key][:k + 1]
    log.update(fail=fail, zone=best_zone, t_goal=t_goal, dist=log["s"][-1],
               zone_name=(["A_stand", "B_straight", "C_scurve", "D_slope",
                           "E_steep", "E_seesaw", "GOAL"][best_zone]))
    return log


def score(log, target=2.0):
    """コンテスト的な評価: 転倒=そこまでの距離、完走=時間"""
    return dict(dist=log["dist"], fell=log["fell_at"] is not None,
                t_end=log["t"][-1] if len(log["t"]) else 0.0,
                th_rms=np.rad2deg(np.sqrt(np.mean(log["th"] ** 2))) if len(log["th"]) else 0,
                th_err_rms=np.rad2deg(np.sqrt(np.mean((log["th"] - log["th_hat"]) ** 2)))
                if len(log["th"]) else 0)
