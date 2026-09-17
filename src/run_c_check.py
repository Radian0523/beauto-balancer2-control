"""機体に焼く C コードを、そのまま Python のシミュレータで回して検証する。
   (設計 → C 実装 の写し間違いを本番前に潰すための回帰テスト)"""
import sys, os, ctypes as ct, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from params import Params
from model import Plant
from sensors import Sensors
import mission as MIS, sim
from run_ablation import perturb

LIB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "out", "libbb2.dylib")


class Meas(ct.Structure):
    _fields_ = [("enc_l", ct.c_float), ("enc_r", ct.c_float), ("gyro", ct.c_float)]


class Out(ct.Structure):
    _fields_ = [("i_l_ma", ct.c_float), ("i_r_ma", ct.c_float),
                ("th_hat", ct.c_float), ("dth_hat", ct.c_float), ("v_hat", ct.c_float),
                ("taud_hat", ct.c_float), ("bias_hat", ct.c_float), ("th_eq", ct.c_float),
                ("v_cmd", ct.c_float), ("s", ct.c_float), ("yaw", ct.c_float),
                ("mode_f", ct.c_float)]


def build_if_needed():
    """gains.h / pace.h / bb2_control.c のどれかが新しければビルドし直す"""
    import subprocess, glob
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    srcs = glob.glob(os.path.join(root, "firmware", "*.c")) + \
           glob.glob(os.path.join(root, "firmware", "*.h"))
    need = (not os.path.exists(LIB)) or \
           any(os.path.getmtime(f) > os.path.getmtime(LIB) for f in srcs)
    if need:
        os.makedirs(os.path.join(root, "out"), exist_ok=True)
        cmd = ["cc", "-O2", "-Wall", "-std=c99", "-I", os.path.join(root, "firmware"),
               "-shared", "-fPIC", os.path.join(root, "firmware", "bb2_control.c"),
               "-o", LIB, "-lm"]
        subprocess.run(cmd, check=True)
        print("(C をビルドしました)")


def load():
    build_if_needed()
    lib = ct.CDLL(LIB)
    lib.bb2_step.restype = Out
    lib.bb2_step.argtypes = [Meas]
    lib.bb2_reset.argtypes = []
    lib.bb2_set_mode.argtypes = [ct.c_int]
    lib.bb2_zero_position.argtypes = [ct.c_float, ct.c_float]
    return lib


def run_c(p, seed=0, T=70.0, calib_T=2.0, box=0.5, v_nom=None):
    lib = load()
    if v_nom is not None:
        ct.c_float.in_dll(lib, "BB2_V_NOM").value = v_nom
    lib.bb2_reset()
    rng = np.random.default_rng(seed)
    plant = Plant(p, alpha_fn=MIS.alpha)
    sens = Sensors(p, rng=rng)
    plant.x[1] = rng.normal(0, np.deg2rad(0.5))

    # キャリブレーション (実機のライブラリ相当: 静止させてジャイロ平均を取る)
    acc = []
    for _ in range(int(calib_T / p.Ts)):
        acc.append(sens.read(np.zeros(6), p.Ts)["gyro"])
    bias0 = float(np.mean(acc))

    n = int(T / p.Ts)
    keys = ["t", "s", "th", "th_hat", "taud_hat", "v", "v_cmd", "i_L", "i_R", "lat_err", "alpha"]
    log = {k: np.zeros(n) for k in keys}
    x_pos = y_pos = 0.0
    lib.bb2_zero_position(0.0, 0.0)
    lib.bb2_set_mode(1)          # HOLD
    fail, t_goal, zone = None, None, 0
    for k in range(n):
        t = k * p.Ts
        y = sens.read(plant.x, p.Ts)
        m = Meas(float(y["enc"][0]), float(y["enc"][1]), float(y["gyro"] - bias0))
        if t >= MIS.STAND_TIME and lib.bb2_get_mode() != 2:
            lib.bb2_set_mode(2)  # RUN
        o = lib.bb2_step(m)
        i_q = np.clip(np.round(np.array([o.i_l_ma, o.i_r_ma]) * 1e-3 / p.i_lsb) * p.i_lsb, -2.0, 2.0)
        plant.step(i_q, p.Ts)
        v_now = p.r * plant.x[2]
        x_pos += v_now * np.cos(plant.x[4]) * p.Ts
        y_pos += v_now * np.sin(plant.x[4]) * p.Ts
        s = p.r * plant.x[0]
        for key, val in zip(keys, [t, s, plant.x[1], o.th_hat, o.taud_hat, v_now,
                                   o.v_cmd, i_q[0], i_q[1], y_pos - MIS.path_y(x_pos),
                                   MIS.alpha(s)]):
            log[key][k] = val
        zone = max(zone, MIS.zone_index(MIS.zone_of(s)))
        if plant.fallen: fail = "転倒"; break
        if t < MIS.STAND_TIME and abs(s) > box: fail = "ゾーンA枠外"; break
        if abs(y_pos - MIS.path_y(x_pos)) > MIS.LANE_HALF: fail = "コースアウト"; break
        if s >= MIS.GOAL: t_goal = t; zone = MIS.zone_index("GOAL"); break
    for key in keys: log[key] = log[key][:k + 1]
    log.update(fail=fail, t_goal=t_goal, dist=log["s"][-1], zone=zone,
               zone_name=["A_stand","B_straight","C_scurve","D_slope","E_steep","E_seesaw","GOAL"][zone])
    return log


if __name__ == "__main__":
    p_nom = Params()
    print("=== 焼く予定の C 実装をシミュレータで走らせる (機体パラメータは毎回ランダムにずらす) ===")
    ok = 0
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    for k in range(N):
        p_true = perturb(np.random.default_rng(2000 + k))
        lg = run_c(p_true, seed=k)
        ok += lg["t_goal"] is not None
        err = np.rad2deg(np.sqrt(np.mean((lg["th"] - lg["th_hat"]) ** 2)))
        print(" %2d: %-10s %5.2f m  t=%s  %-8s 角度推定誤差 %.2f deg 横ずれ %.3f m"
              % (k, lg["zone_name"], lg["dist"],
                 ("%.1f" % lg["t_goal"]) if lg["t_goal"] else " -  ",
                 lg["fail"] or "", err, np.abs(lg["lat_err"]).max()))
    print("完走率 %d/%d" % (ok, N))
