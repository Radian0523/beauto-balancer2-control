/*
 * Beauto Balancer 2 倒立制御ファーム v5 (コントローラ操縦)
 *
 * 純正サンプルの倒立制御 (vendor/control.c の Control()) を土台にしている。
 * フィードバックのゲインは純正のまま触らず、目標の与え方と推定を足した。
 *
 * 1 周期 (500Hz) の流れ (main の走行ループ)
 *   CycleProc()       [純正ライブラリ] センサー読み取り・傾き推定・周期待ち
 *   pad_to_speed()    ① スティック → 滑らかな目標軌道 + 目標前傾 (フィードフォワード)
 *   wheel_observer()  ② 車輪角 → なめらかな車輪角速度 (オブザーバ)
 *   control()         ③ 状態フィードバック → 電流指令 (純正 Control() に目標前傾・積分・ずれ上限を足したもの)
 *   log_sample()      走行ログを RAM に記録
 *   電流指令は [純正ライブラリ] の電流制御 (1kHz) がモータに流す。
 *
 * 純正のまま使っているもの
 *   ライブラリ (libBalancer2_lib.a): センサー読み取り、傾き推定 (ジャイロ積分 + HPF)、電流制御
 *   vendor/control.c: ClearVariables()、calibrarion()、isWhleeOverSpeed()、ゲインの値
 *
 * 操作: 純正と同じ。ボタン → 校正 (5 秒、機体を動かさない) → 立ててボタン →
 *       左スティック上下で前後、右スティック左右で旋回。もう一度ボタンで停止。
 *
 * 調整値は make の引数で変える (例: ../bb2 flash v5 VMAX=0.4)。下の既定値は Makefile と同じ。
 */
#include "Balancer2.h"
#include "control.h"

/* ============================================================ 調整値 (make の引数) */

#ifndef V_MAX
#define V_MAX     (0.5)     /* VMAX: スティックを倒しきったときの速度 [m/s]。0.6 は不安定だった */
#endif
#ifndef V_BACK
#define V_BACK    (0.3)     /* VBACK: 後退の最高速度 [m/s]。後退 0.47 m/s からの減速で倒れたので前進より低くする */
#endif
#ifndef ACC_MAX
#define ACC_MAX   (0.5)     /* ACC: 加速度の上限 [m/s^2] */
#endif
#ifndef FF_K
#define FF_K      (1)       /* FFK: 前傾フィードフォワードの係数 (物理的には 1、0 で無効) */
#endif
#ifndef TI_S
#define TI_S      (2.0)     /* TI: 積分の時定数 [s]。小さいほど強い。0 で無効 */
#endif
#ifndef I_CLIP
#define I_CLIP    (8.0)     /* ICLIP: 積分の上限 (車輪角 [rad] 換算)。傾き推定のずれ約 15° まで打ち消せる */
#endif
#ifndef E_MAX
#define E_MAX     (4.0)     /* EMAX: 走行中の車輪角のずれの上限 [rad] (1 rad = 2.1 cm) */
#endif
#ifndef VBW
#define VBW       (20.0)    /* VBW: 車輪速度オブザーバの帯域 [Hz] */
#endif
#ifndef V_BOOST
#define V_BOOST   (0)       /* BOOST: ボタンでダッシュするときの速度 [m/s]。0 で無効 */
#endif
#ifndef ACC_BOOST
#define ACC_BOOST (0.6)     /* BACC: ダッシュの加速度上限 [m/s^2] */
#endif
#ifndef CAL_RANGE
#define CAL_RANGE (30)      /* CAL_RANGE: 校正中のジャイロの揺れ (最大 - 最小) がこれを超えたらやり直す [LSB] */
#endif
#ifndef REC_DIV
#define REC_DIV   (20)      /* LOGDIV: ログの間引き。500Hz / 20 = 25Hz で約 10 秒ぶん */
#endif
#ifndef DIAG
#define DIAG      (0)       /* DIAG=1: ジャイロ診断用の記録に切り替える (read_log.py が自動で判別) */
#endif
#ifndef DIAG_DELAY
#define DIAG_DELAY (0)      /* DIAG_DELAY: >0 ならスタート後この秒数はモータを止めて記録だけ取る */
#endif

/* ============================================================ 固定値 */

#define JERK      (3.0)             /* 加速度の変化率の上限 [m/s^3] */
#define K_V       (4.0)             /* 速度の差 → 欲しい加速度 [1/s] */
#define BOOST_T   (0.5)             /* ダッシュの長さ [s] */
#define GRAV      (9.81)
#define WHEEL_R   (0.021)           /* タイヤ半径 [m] */
#define TURN_MAX  (9.42477796)      /* 旋回時の左右差 [rad/s] (純正と同じ) */
#define DEADZONE  (10)              /* スティックの中立の遊び (スタート時の値からのずれで判断する) */
#define NEUTRAL_MAX (30)            /* スタート時のスティックの値がこれより大きければ中立とはみなさない (触っていた) */

void LedBlink(unsigned char light, unsigned char blink, int msec, int callcycle);

static double clamp(double x, double lo, double hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ============================================================ ① 目標軌道 + 前傾フィードフォワード
 *
 * スティックの値をそのまま目標にせず、速度・加速度・ジャークの順に上限をかけた滑らかな軌道にする。
 * さらに、加速度 a で進むときに釣り合う前傾角 a/g を本体の目標傾き th_ref として ③ に渡す (2 自由度制御)。
 * 純正は目標がいつも「直立」なので、加速しようと前傾するとフィードバックが起こそうと逆らい、
 * 加速直後に倒れていた。この機体では前傾は BODY_ANGLE の負の向き (実機で確認)。
 */
static double v_cmd, w_cmd;         /* 目標の前進速度 [m/s]・旋回 [rad/s] */
static double a_cmd;                /* 目標の加速度 [m/s^2] */
static double th_ref, dth_ref;      /* 本体の目標傾き [rad]・その変化の速さ [rad/s] */
static int ramping;                 /* 加速/減速中なら 1 (③ の積分を止める) */
static int boost_left;              /* ダッシュの残り [周期] */
static unsigned short btn_prev, btn_idle;   /* ボタンを押していないときの値を基準にする */
static int ly0, rx0;                /* スタート時のスティックの値 = 中立。スティックは手を離してもぴったり 0 に戻らない */

static int neutral(char raw)
{
    int v = (signed char)raw;
    return (v > -NEUTRAL_MAX && v < NEUTRAL_MAX) ? v : 0;
}

static double stick(char raw, int zero)
{
    int v = (signed char)raw - zero;
    if (v > -DEADZONE && v < DEADZONE) return 0.0;
    return v / 128.0;
}

static void pad_to_speed(void)
{
    double v_tgt = -stick(memmap.values.PAD_AN_LY, ly0) * V_MAX;    /* 前に倒すと負 → 前進 */
    double w_tgt =  stick(memmap.values.PAD_AN_RX, rx0) * TURN_MAX;
    double amax = ACC_MAX;
    double dw = 4.0 * TURN_MAX / MAIN_CYCLE;                   /* 旋回は 0.25 秒で最大まで */
    double a_des, a_new;
    unsigned short btn = memmap.values.PAD_BTN;

    /* ダッシュ: どれかのボタンを押した瞬間から BOOST_T 秒だけ V_BOOST で前進 */
    if (V_BOOST > 0 && btn != btn_idle && btn_prev == btn_idle) boost_left = (int)(BOOST_T * MAIN_CYCLE);
    btn_prev = btn;
    if (boost_left > 0) {
        boost_left--;
        v_tgt = V_BOOST;
        amax = ACC_BOOST;
    }

    if (v_tgt < -V_BACK) v_tgt = -V_BACK;                        /* 後退だけ遅くする */

    a_des = clamp(K_V * (v_tgt - v_cmd), -amax, amax);                          /* 速度の差 → 欲しい加速度 */
    a_new = a_cmd + clamp(a_des - a_cmd, -JERK / MAIN_CYCLE, JERK / MAIN_CYCLE); /* 加速度は急に変えない */
    dth_ref = -FF_K * (a_new - a_cmd) * MAIN_CYCLE / GRAV;
    a_cmd = a_new;
    v_cmd += a_cmd / MAIN_CYCLE;
    th_ref = -FF_K * a_cmd / GRAV;                                              /* 目標前傾 = -a/g */
    ramping = (a_cmd > 0.01 || a_cmd < -0.01);

    w_cmd += clamp(w_tgt - w_cmd, -dw, dw);
    memmap.values.T_SPD_L = v_cmd / WHEEL_R + w_cmd;           /* 車輪の目標角速度 [rad/s] */
    memmap.values.T_SPD_R = v_cmd / WHEEL_R - w_cmd;
}

/* ============================================================ ② 車輪速度オブザーバ
 *
 * 純正はエンコーダの差分をそのまま速度にしていて、1 カウント = 6.2 rad/s 刻みと粗い。
 * 速度のゲイン (396) がこのガタつきを ±2500 程度の電流の揺れにしてしまう。
 * 車輪角を追いかける 2 次の追従オブザーバ (帯域 VBW Hz, 臨界減衰) で滑らかな角速度を得る。
 *   x̂' = v̂ + 2ω e,   v̂' = ω² e   (e = 測定した車輪角 - 推定した車輪角)
 */
static double ob_p[2], ob_v[2];     /* 推定した車輪角・車輪角速度 (左, 右) */
static int ob_init;
static double spd_l, spd_r;         /* ③ で使う車輪角速度 [rad/s] */

static void wheel_observer(void)
{
    const double w = 2.0 * 3.14159265 * VBW, dt = 1.0 / MAIN_CYCLE;
    double m[2];
    int i;
    m[0] = memmap.values.WHEEL_ANGLE_L;
    m[1] = memmap.values.WHEEL_ANGLE_R;
    if (!ob_init) {
        for (i = 0; i < 2; i++) { ob_p[i] = m[i]; ob_v[i] = 0.0; }
        ob_init = 1;
    }
    for (i = 0; i < 2; i++) {
        double e = m[i] - ob_p[i];
        ob_p[i] += (ob_v[i] + 2.0 * w * e) * dt;
        ob_v[i] += w * w * e * dt;
    }
    spd_l = ob_v[0];
    spd_r = ob_v[1];
}

/* ============================================================ ③ 状態フィードバック
 *
 * 純正 Control() と同じ形: 本体の傾き・傾く速さ・車輪角のずれ・車輪角速度のずれ に純正ゲインを掛けて足す。
 * 純正から変えたところ ([v5] の行)
 *   - 本体の傾きは直立ではなく ① の目標前傾 th_ref からのずれにする
 *   - 走行中だけ車輪角のずれ (左右の平均) に上限 E_MAX をかける (段差対策)
 *   - 車輪角速度は ② のオブザーバの値を使い、純正の単位の間違い (T_SPD / MAIN_CYCLE) を直す
 *   - 車輪角のずれの積分を足す (坂対策)
 */
static double ref_l, ref_r;         /* 目標車輪角 [rad] */
static double int_l, int_r;         /* 車輪角のずれの積分 [rad*s] */
static double last_el, last_er;     /* 直近の車輪角のずれ (ログ用) */

static void control(void)
{
    double el, er, sl, sr, corr;
    double outL, outR;
    double kw  = memmap.values.GAIN_WHEEL;
    double kwa = memmap.values.GAIN_WHEEL_ANGLE;
    /* 本体の項 [v5] 目標前傾からのずれ */
    double body = memmap.values.GAIN_BODY * (memmap.values.GAIN_BODY_ANGLE * (memmap.values.BODY_ANGLE - th_ref)
                + memmap.values.GAIN_BODY_ANGULAR_SPD * (memmap.values.BODY_ANGULAR_SPD - dth_ref));

    /* 目標車輪角 = 目標角速度の積分 (純正と同じ) */
    ref_l += memmap.values.T_SPD_L / MAIN_CYCLE;
    ref_r += memmap.values.T_SPD_R / MAIN_CYCLE;
    el = memmap.values.WHEEL_ANGLE_L - ref_l;
    er = memmap.values.WHEEL_ANGLE_R - ref_r;

    /* [v5] 段差対策: 段差で車輪が詰まっている間も目標だけ進むと、乗り越えた瞬間に溜まったずれの分だけ
     * 飛び出して倒れる。ずれが E_MAX を超えたら目標のほうを引き戻す。
     * 引き戻しは左右の平均に対して、左右同じだけ行う。左右別々に引き戻すと、片方の車輪だけ
     * 上限に当たったときに左右の目標の差 (= 向くべき方向) が書き換わり、曲がったまま戻らなくなる。
     * 止まっているときは引き戻さない。一定の力で押され続けたとき (傾き推定のずれなど) に
     * 目標が機体と一緒に動いてしまい、元の位置に戻らず一方向に流れ続けるため。 */
    if (v_cmd > 0.02 || v_cmd < -0.02) {
        double em = (el + er) * 0.5, d = 0.0;
        if (em >  E_MAX) d = em - E_MAX;
        if (em < -E_MAX) d = em + E_MAX;
        ref_l += d; ref_r += d;
        el -= d; er -= d;
    }

    /* [v5] 車輪角速度のずれ。純正は T_SPD / MAIN_CYCLE を引いていて、走行中は速度の項がブレーキになっていた */
    sl = spd_l - memmap.values.T_SPD_L;
    sr = spd_r - memmap.values.T_SPD_R;

    outL = body + kw * (kwa * el + memmap.values.GAIN_WHEEL_ANGULAR_SPD * sl);
    outR = body + kw * (kwa * er + memmap.values.GAIN_WHEEL_ANGULAR_SPD * sr);

#if TI_S > 0
    /* [v5] 坂対策: 坂では重力で押し戻され続け、純正は「ずれた分だけ押し返す」だけなので下がりっぱなしになる。
     * ずれを積分して足すと、坂の力を覚えて打ち消し、元の位置に戻る。
     * 加速中の遅れは溜めない (溜めると加速後に余計な力として一気に出る)。 */
    if (!ramping) {
        int_l = clamp(int_l + el / MAIN_CYCLE, -I_CLIP * TI_S, I_CLIP * TI_S);
        int_r = clamp(int_r + er / MAIN_CYCLE, -I_CLIP * TI_S, I_CLIP * TI_S);
    }
    outL += kw * kwa * int_l / TI_S;
    outR += kw * kwa * int_r / TI_S;
#endif

    /* 左右差の項 (純正と同じ): 左右の車輪をそろえてまっすぐ走る */
    corr = memmap.values.GAIN_WHEEL_CORRELATION * (memmap.values.GAIN_WHEEL_CORRELATION_ANGLE * (el - er)
         + memmap.values.GAIN_WHEEL_CORRELATION_ANGULAR_SPD * (sl - sr));
    outL -= corr;
    outR += corr;

    last_el = el;
    last_er = er;
    memmap.values.T_CURRENT_L = (short)clamp(outL, -32767.0, 32767.0);
    memmap.values.T_CURRENT_R = (short)clamp(outR, -32767.0, 32767.0);
}

/* ============================================================ 走行ログ
 *
 * 走行中の値を機体の RAM (純正ライブラリのログ用バッファ srcbuf, 4KB) にリングバッファで記録する。
 * USB を繋がずに走らせ、止めたあと電源を切らずに USB を挿して ../bb2 log で吸い出す。
 * srcbuf は USB の 'R' コマンドで 0x1000 番地から読める (純正のログ機能は止めているので衝突しない)。
 *
 * 形式: 先頭 8 バイト = 'L', 'G'(通常)/'D'(診断), 変数の数, 間引き, 次に書く位置(2), 有効サンプル数(2)
 *       その後に 1 サンプル = int16 × 11 (リトルエンディアン)
 */
#define LOG_NVAR  (11)
#define LOG_HDR   (8)
#define LOG_CAP   ((4096 - LOG_HDR) / (LOG_NVAR * 2))   /* 185 サンプル (25Hz で約 7.4 秒) */
extern uint8_t srcbuf[4096];
static unsigned int log_idx, log_cnt, log_div;

/* 診断 (DIAG=1) 用: 純正の傾き推定がずれる原因を調べるため、生のジャイロを別に集計する */
#define GYRO_K (0.041849 * 0.125)   /* 純正 GyroProc と同じ換算: (GYRO_DATA - OFFSET) * GYRO_K = rad/s */
static double dg_sum, dg_theta;     /* 生ジャイロの区間合計、生ジャイロを単純積分した傾き */
static int dg_upd;                  /* 区間内で GYRO_DATA が変わった回数 (ジャイロの更新頻度) */
static short dg_prev;

static short s16(double x)
{
    return (short)clamp(x, -32767.0, 32767.0);
}

static void log_reset(void)
{
    log_idx = log_cnt = log_div = 0;
    dg_sum = dg_theta = 0.0; dg_upd = 0; dg_prev = memmap.values.GYRO_DATA;
    srcbuf[0] = 'L'; srcbuf[1] = DIAG ? 'D' : 'G'; srcbuf[2] = LOG_NVAR; srcbuf[3] = REC_DIV;
    srcbuf[4] = srcbuf[5] = srcbuf[6] = srcbuf[7] = 0;
}

static void log_sample(void)
{
    short v[LOG_NVAR];
    unsigned int i;
    uint8_t *p;
#if DIAG
    {
        double g = memmap.values.GYRO_DATA - memmap.values.BODY_ANGULAR_SPD_OFFSET;
        dg_sum += g;
        dg_theta += g * GYRO_K / MAIN_CYCLE;
        if (memmap.values.GYRO_DATA != dg_prev) dg_upd++;
        dg_prev = memmap.values.GYRO_DATA;
    }
#endif
    if (++log_div < REC_DIV) return;
    log_div = 0;
    v[0] = s16(memmap.values.BODY_ANGLE * 1e4);                                 /* 本体角 [0.1 mrad] */
    v[1] = s16(th_ref * 1e4);                                                   /* 目標前傾 [0.1 mrad] */
    v[2] = s16(memmap.values.BODY_ANGULAR_SPD * 1e3);                           /* 本体角速度 [mrad/s] */
    v[3] = s16((spd_l + spd_r) * 50.0);                                         /* 車輪角速度 (オブザーバ) 平均 [0.01 rad/s] */
    v[4] = s16((memmap.values.T_SPD_L + memmap.values.T_SPD_R) * 50.0);         /* 目標車輪角速度 平均 [0.01 rad/s] */
    v[5] = s16((memmap.values.T_CURRENT_L + memmap.values.T_CURRENT_R) * 0.5);  /* 電流指令 平均 */
    v[6] = s16((last_el + last_er) * 500.0);                                    /* 車輪角のずれ 平均 [mrad] */
    v[7] = s16((int_l + int_r) * 500.0);                                        /* ずれの積分 平均 [mrad*s] */
    v[8] = s16(((memmap.values.M_CURRENT_L - memmap.values.CURRENT_OFFSET_L)
              + (memmap.values.M_CURRENT_R - memmap.values.CURRENT_OFFSET_R)) * 0.5);  /* 実際の電流 平均 (生値) */
    v[9] = s16((last_el - last_er) * 1e3);                                      /* 向きのずれ = 左右の車輪角のずれの差 [mrad] */
    v[10] = s16(w_cmd * 100.0);                                                 /* 旋回の指令 [0.01 rad/s] */
#if DIAG
    v[1] = s16(dg_sum / REC_DIV * 100.0);    /* 生ジャイロ (0 点を引いた値) の区間平均 [0.01 LSB] */
    v[4] = s16(dg_theta * 1e4);              /* 生ジャイロを単純積分した傾き [0.1 mrad] (0 に戻す処理なし) */
    v[6] = (short)dg_upd;                    /* 区間内でジャイロの値が変わった回数 */
    dg_sum = 0.0; dg_upd = 0;
#endif
    p = srcbuf + LOG_HDR + log_idx * LOG_NVAR * 2;
    for (i = 0; i < LOG_NVAR; i++) {
        p[2 * i] = (uint8_t)(v[i] & 0xFF);
        p[2 * i + 1] = (uint8_t)((v[i] >> 8) & 0xFF);
    }
    log_idx = (log_idx + 1) % LOG_CAP;
    if (log_cnt < LOG_CAP) log_cnt++;
    srcbuf[4] = (uint8_t)log_idx; srcbuf[5] = (uint8_t)(log_idx >> 8);
    srcbuf[6] = (uint8_t)log_cnt; srcbuf[7] = (uint8_t)(log_cnt >> 8);
}

/* ============================================================ 校正・初期化 */

/* ジャイロ校正 (純正 calibrarion) を、途中で機体が動かされていないか確かめながら行う。
 * 校正は 5 秒間のジャイロ平均を 0 点にするので、その間に機体を動かすと回した分が 0 点に混ざり、
 * 走行中に傾き推定がずれていく (実測で -22 LSB = -6.5 deg/s ずれた回があった)。
 * 静止中のジャイロの揺れは数 LSB、動かすと数十以上になるので、
 * 最大と最小の差が CAL_RANGE を超えたら両方の LED を 1 秒速く点滅させてやり直す。 */
static void calibrate_checked(void)
{
    while (1) {
        short gmin = 32767, gmax = -32768;
        int k;
        memmap.values.CALIBRATISON_CNT = 0;
        while (calibrarion() == 0) {
            short g;
            CycleProc();
            g = memmap.values.GYRO_DATA;
            if (g < gmin) gmin = g;
            if (g > gmax) gmax = g;
            LedBlink(0, 2, 150, MAIN_CYCLE);
        }
        if (gmax - gmin <= CAL_RANGE) return;
        for (k = 0; k < MAIN_CYCLE; k++) {        /* 失敗の合図 */
            CycleProc();
            LedBlink(0, 3, 100, MAIN_CYCLE);
        }
    }
}

/* スタート・停止のたびに、純正の変数と ①〜③ の状態をすべて 0 に戻す */
static void reset_state(void)
{
    ClearVariables();
    ref_l = ref_r = int_l = int_r = 0.0;
    v_cmd = w_cmd = 0.0;
    a_cmd = th_ref = dth_ref = 0.0;
    boost_left = 0;
    ob_init = 0;
    btn_idle = btn_prev = memmap.values.PAD_BTN;
    ly0 = neutral(memmap.values.PAD_AN_LY);
    rx0 = neutral(memmap.values.PAD_AN_RX);
    memmap.values.T_SPD_L = memmap.values.T_SPD_R = 0.0;
}

/* ============================================================ main */

int main(void)
{
    Balancer2Init();
    UARTInit(CBR_115200, non, 1);
    CurrentControlDisable();
    setLED(3);

    /* ボタンを押すまで待ってから校正 */
    while (getSW() == 0) CycleProc();
    while (getSW() == 1) __NOP();
    setLED(2);
    Wait(1000);

    calibrate_checked();
    reset_state();

    while (1) {
        /* 待機: 機体を立てて、ボタンでスタート */
        while (getSW() == 0) {
            CycleProc();
            LedBlink(2, 1, 1000, MAIN_CYCLE);
        }
        while (getSW() == 1) __NOP();
        setLED(2);
        reset_state();
        memmap.values.LOG_MODE = 0;      /* 純正のログ機能は止める (srcbuf を自前のログに使う) */
        log_reset();
#if DIAG_DELAY > 0
        /* 診断: 最初の DIAG_DELAY 秒はモータを止めたまま記録だけ取る (手で支えて静止させておく)。
         * モータが止まっているときと動いているときでジャイロの 0 点が変わるかを見る。 */
        {
            int k;
            for (k = 0; k < (int)(DIAG_DELAY * MAIN_CYCLE); k++) {
                CycleProc();
                log_sample();
                LedBlink(0, 3, 100, MAIN_CYCLE);
            }
            setLED(2);
            reset_state();          /* この時点の姿勢を 0° にしてから制御を始める (記録は消さない) */
        }
#endif
        CurrentControlEnable();

        /* 走行 (500Hz) */
        while (1) {
            uint8_t slow = CycleProc();
            pad_to_speed();
            wheel_observer();
            control();
            log_sample();
            if (slow == 1) {
                if (isWhleeOverSpeed() != 0) break;     /* 転倒 (車輪の回りすぎ) で停止 */
                if (getSW() == 1) {                     /* ボタンで停止 */
                    while (getSW() == 1) __NOP();
                    break;
                }
            }
        }
        CurrentControlDisable();
        reset_state();
        Wait(1000);      /* 止めた直後のボタンのチャタリングで再スタートして記録を消さないように */
    }
}

/* LED の点滅。light は点灯しっぱなし、blink は msec 周期で点滅させる LED (callcycle 回/秒 呼ぶ前提) */
void LedBlink(unsigned char light, unsigned char blink, int msec, int callcycle)
{
    static int cnt = 0;
    volatile int c = (msec * callcycle / 1000);
    cnt++;
    setLED(cnt < c / 2 ? (light | blink) : light);
    if (cnt > c) cnt = 0;
}
