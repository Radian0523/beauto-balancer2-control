#include "bb2_control.h"
#include "gains.h"
#include <math.h>

/* ------------------------------------------------------------------ 調整値 */
float BB2_V_NOM     = 0.35f;
float BB2_K_FF      = 0.6f;
float BB2_TH_EQ_MAX = 0.10472f;   /* 6 deg */
float BB2_TH_GUARD  = 0.38397f;   /* 22 deg */
int   BB2_USE_GOV   = 1;

/* --------------------------------------------------------- ペースノート
 * コースは事前に分かっている。中心線の通過点(ウェイポイント)と各区間の目標速度を
 * 表にしておき、エンコーダのデッドレコニングで追従する (Stanley 法)。
 * pace.h は src/gen_pace.py がコース定義から自動生成する。
 * 当日は練習走行でコースを測って gen_pace.py の数値を直すだけでよい。
 */
#include "pace.h"

/* ------------------------------------------------------------ 内部状態 */
static float xh[6];          /* [phi, theta, dphi, dtheta, bias, tau_d] */
static float xi;             /* 速度偏差の積分 */
static float u_prev;         /* 前回の実効トルク [Nm] */
static float v_cmd;          /* ガバナ通過後の速度指令 [m/s] */
static float taud_f, sat_f, err_f;
static float yaw_ref, phi_hold;
static float enc_off_l, enc_off_r;
static float od_x, od_y, od_psi;   /* デッドレコニングの自己位置 */
static int   seg = 0;              /* 追従中のウェイポイント区間 */
float BB2_K_STANLEY = 1.6f;        /* 横ずれを戻す強さ */
static bb2_mode_t mode = BB2_MODE_IDLE;

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

void bb2_reset(void)
{
    int i;
    for (i = 0; i < 6; i++) xh[i] = 0.0f;
    xi = u_prev = v_cmd = taud_f = sat_f = err_f = 0.0f;
    od_x = od_y = od_psi = 0.0f; seg = 0;
    yaw_ref = phi_hold = 0.0f;
    enc_off_l = enc_off_r = 0.0f;
    mode = BB2_MODE_IDLE;
}

void bb2_set_mode(bb2_mode_t m) { mode = m; if (m == BB2_MODE_RUN) xi = 0.0f; }
bb2_mode_t bb2_get_mode(void) { return mode; }

/* -------------------------------------------------- カルマンフィルタ 1周期 */
static void kf_update(float enc, float gyro, float u)
{
    float xp[6];
    float e0, e1;
    int i, j;

    for (i = 0; i < 6; i++) {
        float s = KF_BD[i] * u;
        for (j = 0; j < 6; j++) s += KF_AD[i][j] * xh[j];
        xp[i] = s;
    }
    e0 = enc  - (xp[0] - xp[1]);        /* エンコーダは本体基準 = phi - theta */
    e1 = gyro - (xp[3] + xp[4]);        /* ジャイロ = 本体角速度 + バイアス     */
    for (i = 0; i < 6; i++)
        xh[i] = xp[i] + KF_L[i][0] * e0 + KF_L[i][1] * e1;
}

/* ---------------------------------------- デッドレコニング + パス追従(Stanley) */
static void path_follow(float v_meas, float *v_ref, float *yaw_ref_out)
{
    float x0, y0, x1, y1, dx, dy, len, t_proj, e_lat, psi_path;

    while (seg < N_PATH - 2) {           /* 進んだ区間は捨てる */
        x0 = PATH[seg].x; y0 = PATH[seg].y;
        x1 = PATH[seg+1].x; y1 = PATH[seg+1].y;
        dx = x1 - x0; dy = y1 - y0;
        len = sqrtf(dx*dx + dy*dy) + 1e-6f;
        t_proj = ((od_x - x0) * dx + (od_y - y0) * dy) / (len * len);
        if (t_proj > 1.0f) seg++; else break;
    }
    x0 = PATH[seg].x; y0 = PATH[seg].y;
    x1 = PATH[seg+1].x; y1 = PATH[seg+1].y;
    dx = x1 - x0; dy = y1 - y0;
    len = sqrtf(dx*dx + dy*dy) + 1e-6f;
    e_lat    = ((od_y - y0) * dx - (od_x - x0) * dy) / len;   /* 左が正 */
    psi_path = atan2f(dy, dx);
    /* Stanley: 進路のずれを方位目標に織り込む (速いほど舵を弱く) */
    *yaw_ref_out = psi_path - atan2f(BB2_K_STANLEY * e_lat, fabsf(v_meas) + 0.15f);
    *v_ref = PATH[seg].v;
}

/* ------------------------------------------------------------------ 本体 */
bb2_out_t bb2_step(bb2_meas_t meas)
{
    bb2_out_t out;
    static float sat_prev = 0.0f, yaw_prev = 0.0f;
    float enc_l = meas.enc_l - enc_off_l;
    float enc_r = meas.enc_r - enc_off_r;
    float enc   = 0.5f * (enc_l + enc_r);
    float yaw, dyaw, v_ref, th_eq, u_eq, u, ud, v_lim, a;
    float S, D, d_max, tau_max, tau_l, tau_r, il, ir, s_travel, sat;
    int recover = 0;

    /* --- 推定 (前周期に実際に出したトルクを使う) ---------------------- */
    kf_update(enc, meas.gyro, u_prev);
    s_travel = WHEEL_R * xh[0];
    yaw  = WHEEL_R * (enc_r - enc_l) / TREAD_W;
    dyaw = (yaw - yaw_prev) / CTL_TS;
    yaw_prev = yaw;

    out.th_hat = xh[1]; out.dth_hat = xh[3]; out.v_hat = WHEEL_R * xh[2];
    out.taud_hat = xh[5]; out.bias_hat = xh[4];
    out.s = s_travel; out.yaw = yaw; out.mode_f = (float)mode;

    if (mode == BB2_MODE_IDLE) {
        u_prev = 0.0f; sat_prev = 0.0f;
        out.i_l_ma = out.i_r_ma = 0.0f;
        out.th_eq = 0.0f; out.v_cmd = v_cmd = 0.0f;
        return out;
    }

    /* --- 参照 -------------------------------------------------------- */
    /* デッドレコニング (エンコーダのみで自己位置を推定) */
    od_psi = yaw;
    od_x += WHEEL_R * xh[2] * cosf(od_psi) * CTL_TS;
    od_y += WHEEL_R * xh[2] * sinf(od_psi) * CTL_TS;
    if (mode == BB2_MODE_HOLD) { v_ref = 0.0f; yaw_ref = 0.0f; }
    else                       { path_follow(WHEEL_R * xh[2], &v_ref, &yaw_ref); }

    /* --- 参照ガバナ: トルク不足・姿勢崩れのときだけ減速 --------------- */
    a = CTL_TS / 0.15f;
    sat_f += a * (fabsf(sat_prev) - sat_f);
    v_lim = 1.0f;
    if (BB2_USE_GOV) {
        v_lim -= 0.7f * clampf(sat_f / 0.02f, 0.0f, 1.0f);
        v_lim -= 0.5f * clampf(err_f / 0.10472f, 0.0f, 1.0f);
        if (v_lim < 0.15f) v_lim = 0.15f;
    }
    {   /* 速度指令のスルーレート制限 (1.2 m/s^2) */
        float v_tgt = v_ref * v_lim;
        v_cmd += clampf(v_tgt - v_cmd, -1.2f * CTL_TS, 1.2f * CTL_TS);
    }

    /* --- 外乱(坂)の前置補償 ------------------------------------------
     * θ_eq = k_ff * (-τd) / (m_b g l) : 坂では「坂の分だけ前傾」が正解。
     * k_ff を 1 にすると入力ゲイン誤差に対して正帰還になるので 0.6 に落とす。 */
    taud_f += (CTL_TS / 0.20f) * (xh[5] - taud_f);
    u_eq  = -taud_f;
    th_eq = clampf(BB2_K_FF * (u_eq / MGL), -BB2_TH_EQ_MAX, BB2_TH_EQ_MAX);
    err_f += a * (fabsf(xh[1] - th_eq) - err_f);
    out.th_eq = th_eq;

    /* --- 姿勢フィードバック ------------------------------------------ */
    tau_max = KG2 * (I_CMD_MAX_MA * 1e-3f);          /* 片輪の上限 [Nm] */
    if (fabsf(xh[1] - th_eq) > BB2_TH_GUARD) {
        u = ((xh[1] > th_eq) ? 1.0f : -1.0f) * 2.0f * tau_max;  /* 転倒復帰 */
        xi *= 0.9f;
        recover = 1;
    } else if (mode == BB2_MODE_HOLD) {
        u = u_eq - (K_POS[0] * (xh[0] - phi_hold) + K_POS[1] * (xh[1] - th_eq)
                    + K_POS[2] * xh[2] + K_POS[3] * xh[3]);
    } else {
        float dphi_ref = v_cmd / WHEEL_R;
        xi += (xh[2] - dphi_ref) * CTL_TS - 2.0f * sat_prev * CTL_TS;  /* AW */
        xi = clampf(xi, -30.0f, 30.0f);
        u = u_eq - (K_VEL[0] * (xh[1] - th_eq) + K_VEL[1] * (xh[2] - dphi_ref)
                    + K_VEL[2] * xh[3] + K_VEL[3] * xi);
    }

    /* --- ヨー -------------------------------------------------------- */
    ud = recover ? 0.0f : -(K_YAW[0] * (yaw - yaw_ref) + K_YAW[1] * dyaw);

    /* --- 制御配分: 倒立を最優先、旋回は余ったトルクだけ --------------- */
    S = clampf(u, -2.0f * tau_max, 2.0f * tau_max);
    d_max = 2.0f * tau_max - fabsf(S);
    if (d_max < 0.0f) d_max = 0.0f;
    D = clampf(ud, -d_max, d_max);
    sat = u - S;                 /* 倒立チャンネルの飽和量 */
    sat_prev = sat;

    /* --- 電流指令 [mA]  (片側モータ: i = tau / Kg, Kg = KG2/2) -------- */
    tau_l = 0.5f * (S - D);
    tau_r = 0.5f * (S + D);
    il = clampf(2000.0f * tau_l / KG2, -I_CMD_MAX_MA, I_CMD_MAX_MA);
    ir = clampf(2000.0f * tau_r / KG2, -I_CMD_MAX_MA, I_CMD_MAX_MA);
    out.i_l_ma = il;
    out.i_r_ma = ir;
    u_prev = 0.5f * KG2 * (il + ir) * 1e-3f;   /* 実際に出したトルク */
    out.v_cmd = v_cmd;
    return out;
}

/* 直立開始時に呼ぶ: いまの車輪角を原点にする */
void bb2_zero_position(float enc_l, float enc_r)
{
    enc_off_l = enc_l; enc_off_r = enc_r;
    xh[0] = 0.0f; phi_hold = 0.0f; yaw_ref = 0.0f;
    od_x = od_y = od_psi = 0.0f; seg = 0;
}
