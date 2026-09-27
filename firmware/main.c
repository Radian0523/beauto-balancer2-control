/*
 * v3: コントローラ操縦用。純正の倒立制御 (4状態フィードバック + 純正ゲイン) を土台に、
 *     段差と坂に強くするための小さな改良を足したもの。どれも make の引数で切れる。
 *
 *  改良点
 *   FIX=1  速度目標の単位ずれを直す。純正は WHEEL_ANGULAR_SPD から T_SPD/MAIN_CYCLE を
 *          引いていて、走行中は速度項がブレーキとして働く (遅れて付いてくる)。
 *   TI=2   車輪角ずれの積分 (時定数 TI 秒)。坂では重力で押し戻され、純正はずれた分だけ
 *          踏ん張るので下がりっぱなしになる。積分でずれを 0 に戻す = 坂の力を覚えて打ち消す。
 *          TI=0 で無効。
 *   EMAX=4 目標車輪角と実際の差の上限 [rad] (4 rad ≒ 8cm)。段差で詰まっている間も
 *          目標だけ進むと、乗り越えた瞬間に溜まった分だけ飛び出して倒れる。上限で目標を引き戻す。
 *   ACC=0.5 スティック操作に対する加速度の上限 [m/s^2]。急な操作で倒れないように。
 *   VMAX=0.2 スティックを倒しきったときの速度 [m/s] (純正は約 0.2)。
 *   BOOST=0.25 コントローラのボタン(どれでも)を押すと BOOST_T 秒だけこの速度で前進する。
 *          段差はゆっくり当たるとモータの反動で本体が後ろへ倒れるので、勢いで乗り越える。
 *   BACC=0.6 ダッシュ時の加速度上限 [m/s^2]。
 *
 *   FF=1   (v4) 目標軌道のフィードフォワード (2自由度制御)。加速度 a を指令するときは
 *          本体の目標傾きも a/g だけ前に倒し、加速度の変化 (ジャーク) も JERK で制限する。
 *          フィードバックゲインは純正のまま、ずれを直す役だけになる。
 *          FF_DIR=-1 で傾きの向きを反転 (加速で余計に倒れるならこれ)。
 *   VOBS=1 (v5) 車輪角速度をオブザーバで推定する。純正はエンコーダ差分 (1 カウント = 6.2 rad/s 刻み)
 *          をそのまま使うため速度項 (ゲイン 396) が ±2500 程度の電流のガタつきを生み、常に細かく振動していた。
 *          車輪角を追いかける 2 次の追従オブザーバ (帯域 VBW Hz, 臨界減衰) で滑らかな角速度を得る。
 *   NOTCH=11 電流指令に f0 [Hz] のノッチフィルタ (Q=NQ) を掛ける。250Hz ログで 11Hz の
 *          自励振動 (本体角速度が矩形波 = ギアのガタ) が見えたため、その周波数だけループゲインを下げる。
 *          0 で無効。
 *
 *  操作: 純正と同じ。ボタン → 校正 → 立ててボタン → 左スティック上下で前後、右スティック左右で旋回。
 */
#include "Balancer2.h"
#include "control.h"

#ifndef FIX_SPD_REF
#define FIX_SPD_REF 1
#endif
#ifndef TI_S
#define TI_S    (2.0)
#endif
#ifndef E_MAX
#define E_MAX   (4.0)
#endif
#ifndef ACC_MAX
#define ACC_MAX (0.5)
#endif
#ifndef V_MAX
#define V_MAX   (0.2)
#endif

#ifndef V_BOOST
#define V_BOOST (0.25)
#endif
#ifndef ACC_BOOST
#define ACC_BOOST (0.6)
#endif
#define BOOST_T    (0.5)                   /* ダッシュの長さ [s] */

#ifndef FF
#define FF      (0)                        /* 前傾フィードフォワードの係数 (物理的には 1) */
#endif
#ifndef FF_DIR
#define FF_DIR  (1.0)                      /* 前傾が BODY_ANGLE 負。純正ゲインの符号から推定 */
#endif
#ifndef JERK
#define JERK    (3.0)                      /* 加速度の変化率の上限 [m/s^3] */
#endif
#ifndef VOBS
#define VOBS    (0)
#endif
#ifndef VBW
#define VBW     (20.0)                     /* 速度オブザーバの帯域 [Hz] */
#endif
#ifndef NOTCH
#define NOTCH   (0)                        /* ノッチ周波数 [Hz]、0 で無効 */
#endif
#ifndef NQ
#define NQ      (2.0)
#endif
#define K_V        (4.0)                   /* 速度誤差 → 加速度 [1/s] */
#define GRAV       (9.81)

#define WHEEL_R    (0.021)                 /* タイヤ半径 [m] */
#define TURN_MAX   (9.42477796)            /* 旋回時の左右差 [rad/s] (純正と同じ) */
#define DEADZONE   (10)                    /* スティックの中立の遊び */
#ifndef I_CLIP
#define I_CLIP     (8.0)                   /* 積分の寄与を車輪角 8 rad ぶんまでに制限 (傾き推定のずれ約 15° まで打ち消せる) */
#endif

void LedBlink(unsigned char light, unsigned char blink, int msec, int callcycle);

static double ref_l, ref_r;       /* 目標車輪角 [rad] */
static double int_l, int_r;       /* 車輪角ずれの積分 [rad*s] */
static double last_el, last_er;   /* 直近の車輪角ずれ (ログ用) */
static double ob_p[2], ob_v[2];   /* 速度オブザーバ: 推定車輪角・推定角速度 (左,右) */
static int ob_init;
static double spd_l, spd_r;       /* 制御に使う車輪角速度 */

/* 車輪角 → 角速度の追従オブザーバ。 x̂' = v̂ + 2ωe,  v̂' = ω²e  (e = 測定 - 推定) */
static void wheel_observer(void)
{
#if VOBS
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
#else
    spd_l = memmap.values.WHEEL_ANGULAR_SPD_L;
    spd_r = memmap.values.WHEEL_ANGULAR_SPD_R;
#endif
}
static double v_cmd, w_cmd;       /* 加速度制限後の前進速度 [m/s]、旋回 [rad/s] */
static double a_cmd;              /* 指令加速度 [m/s^2] (FF 時) */
static double th_ref, dth_ref;     /* 本体の目標傾き [rad]・目標角速度 [rad/s] */
static int ramping;               /* 加速/減速中なら 1 (積分を止める) */
static int boost_left;            /* ダッシュの残り [周期] */
static unsigned short btn_prev, btn_idle;   /* ボタン無しのときの値を基準にする */

static double stick(char raw)
{
    int v = (signed char)raw;
    if (v > -DEADZONE && v < DEADZONE) return 0.0;
    return v / 128.0;
}

static double clamp(double x, double lo, double hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* スティック → 目標車輪角速度 T_SPD [rad/s]。500Hz で呼ぶ。 */
static void pad_to_speed(void)
{
    double v_tgt = -stick(memmap.values.PAD_AN_LY) * V_MAX;    /* 前に倒すと負 → 前進 */
    double w_tgt =  stick(memmap.values.PAD_AN_RX) * TURN_MAX;
    double dv = ACC_MAX / MAIN_CYCLE;
    double dw = 4.0 * TURN_MAX / MAIN_CYCLE;                   /* 旋回は 0.25 秒で最大まで */
    unsigned short btn = memmap.values.PAD_BTN;

    if (V_BOOST > 0 && btn != btn_idle && btn_prev == btn_idle) boost_left = (int)(BOOST_T * MAIN_CYCLE);  /* 押した瞬間 */
    btn_prev = btn;
    if (boost_left > 0) {
        boost_left--;
        v_tgt = V_BOOST;
        dv = ACC_BOOST / MAIN_CYCLE;
    }

#if FF
    {
        /* 速度 → 加速度 → ジャークの順に制限した滑らかな軌道を作り、
         * その加速度に見合った前傾 a/g を本体の目標にする */
        double amax = dv * MAIN_CYCLE;
        double a_des = clamp(K_V * (v_tgt - v_cmd), -amax, amax);
        double a_new = a_cmd + clamp(a_des - a_cmd, -JERK / MAIN_CYCLE, JERK / MAIN_CYCLE);
        dth_ref = -FF_DIR * FF * (a_new - a_cmd) * MAIN_CYCLE / GRAV;
        a_cmd = a_new;
        v_cmd += a_cmd / MAIN_CYCLE;
        th_ref = -FF_DIR * FF * a_cmd / GRAV;
        ramping = (a_cmd > 0.01 || a_cmd < -0.01);
    }
#else
    ramping = (v_tgt - v_cmd > 1e-4 || v_tgt - v_cmd < -1e-4);
    v_cmd += clamp(v_tgt - v_cmd, -dv, dv);
#endif
    w_cmd += clamp(w_tgt - w_cmd, -dw, dw);
    memmap.values.T_SPD_L = v_cmd / WHEEL_R + w_cmd;
    memmap.values.T_SPD_R = v_cmd / WHEEL_R - w_cmd;
}

#if NOTCH > 0
/* 双一次変換 (周波数プリワープ) のノッチ。左右で状態を分ける */
typedef struct { double b0, b1, b2, a1, a2, x1, x2, y1, y2; } biquad_t;
static biquad_t nf[2];
static void notch_init(void)
{
    double w0 = 2.0 * 3.14159265 * NOTCH / MAIN_CYCLE;
    /* w0 は 0.14 rad 程度なのでテイラー展開で十分 (libm の sin/cos は 3KB 近くあり入らない) */
    double w2 = w0 * w0;
    double c = 1.0 - w2 / 2.0 * (1.0 - w2 / 12.0 * (1.0 - w2 / 30.0));
    double sn = w0 * (1.0 - w2 / 6.0 * (1.0 - w2 / 20.0 * (1.0 - w2 / 42.0)));
    double al = sn / (2.0 * NQ), a0 = 1.0 + al;
    int i;
    for (i = 0; i < 2; i++) {
        nf[i].b0 = 1.0 / a0; nf[i].b1 = -2.0 * c / a0; nf[i].b2 = 1.0 / a0;
        nf[i].a1 = -2.0 * c / a0; nf[i].a2 = (1.0 - al) / a0;
        nf[i].x1 = nf[i].x2 = nf[i].y1 = nf[i].y2 = 0.0;
    }
}
static double notch(biquad_t *f, double x)
{
    double y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1; f->x1 = x; f->y2 = f->y1; f->y1 = y;
    return y;
}
#endif

static void control_v3(void)
{
    double el, er, sl, sr, corr;
    double outL, outR;
    double body = memmap.values.GAIN_BODY * (memmap.values.GAIN_BODY_ANGLE * (memmap.values.BODY_ANGLE - th_ref)
                + memmap.values.GAIN_BODY_ANGULAR_SPD * (memmap.values.BODY_ANGULAR_SPD - dth_ref));
    double kw  = memmap.values.GAIN_WHEEL;
    double kwa = memmap.values.GAIN_WHEEL_ANGLE;

    ref_l += memmap.values.T_SPD_L / MAIN_CYCLE;
    ref_r += memmap.values.T_SPD_R / MAIN_CYCLE;

    /* 段差対策: ずれが E_MAX を超えたら目標のほうを引き戻す (押す力は E_MAX ぶんで頭打ち) */
    el = memmap.values.WHEEL_ANGLE_L - ref_l;
    er = memmap.values.WHEEL_ANGLE_R - ref_r;
    /* 止まっているときは引き戻さない。引き戻すと、一定の力で押され続けたとき
     * (傾き推定のずれなど) 目標が機体と一緒に動いてしまい、元の位置に戻らず流れ続ける。 */
    if (v_cmd > 0.02 || v_cmd < -0.02) {
        if (el >  E_MAX) { ref_l = memmap.values.WHEEL_ANGLE_L - E_MAX; el =  E_MAX; }
        if (el < -E_MAX) { ref_l = memmap.values.WHEEL_ANGLE_L + E_MAX; el = -E_MAX; }
        if (er >  E_MAX) { ref_r = memmap.values.WHEEL_ANGLE_R - E_MAX; er =  E_MAX; }
        if (er < -E_MAX) { ref_r = memmap.values.WHEEL_ANGLE_R + E_MAX; er = -E_MAX; }
    }

#if FIX_SPD_REF
    sl = spd_l - memmap.values.T_SPD_L;
    sr = spd_r - memmap.values.T_SPD_R;
#else
    sl = spd_l - memmap.values.T_SPD_L / MAIN_CYCLE;
    sr = spd_r - memmap.values.T_SPD_R / MAIN_CYCLE;
#endif

    outL = body + kw * (kwa * el + memmap.values.GAIN_WHEEL_ANGULAR_SPD * sl);
    outR = body + kw * (kwa * er + memmap.values.GAIN_WHEEL_ANGULAR_SPD * sr);

#if TI_S > 0
    /* 坂対策: ずれの積分。寄与は車輪角 I_CLIP rad ぶんまで */
    /* 加速中の遅れは溜めない (溜めると加速後に余計な力として一気に出る) */
    if (!ramping) {
        int_l = clamp(int_l + el / MAIN_CYCLE, -I_CLIP * TI_S, I_CLIP * TI_S);
        int_r = clamp(int_r + er / MAIN_CYCLE, -I_CLIP * TI_S, I_CLIP * TI_S);
    }
    outL += kw * kwa * int_l / TI_S;
    outR += kw * kwa * int_r / TI_S;
#endif

    corr = memmap.values.GAIN_WHEEL_CORRELATION * (memmap.values.GAIN_WHEEL_CORRELATION_ANGLE * (el - er)
         + memmap.values.GAIN_WHEEL_CORRELATION_ANGULAR_SPD * (sl - sr));
    outL -= corr;
    outR += corr;

    last_el = el;
    last_er = er;
#if NOTCH > 0
    outL = notch(&nf[0], outL);
    outR = notch(&nf[1], outR);
#endif
    memmap.values.T_CURRENT_L = (short)clamp(outL, -32767.0, 32767.0);
    memmap.values.T_CURRENT_R = (short)clamp(outR, -32767.0, 32767.0);
}


/* ------------------------------------------------------------ 走行ログ
 * 走行中の値を機体の RAM (純正ライブラリのログ用バッファ srcbuf, 4KB) に
 * リングバッファで記録する。USB を繋がずに走らせ、止めたあと繋いで tools/read_log.py で吸い出す。
 * 電源を切ると消えるので、止めたら電源はそのままで USB を挿す。
 * srcbuf は USB の 'R' コマンドで 0x1000 番地から読める (純正のログ機能は使っていないので衝突しない)。
 */
#ifndef REC_DIV
#define REC_DIV   (20)                       /* 500Hz / 20 = 25Hz で記録 */
#endif
#define LOG_NVAR  (8)
#define LOG_HDR   (8)
#define LOG_CAP   ((4096 - LOG_HDR) / (LOG_NVAR * 2))   /* 255 サンプル = 約 10 秒 */
extern uint8_t srcbuf[4096];
static unsigned int log_idx, log_cnt, log_div;

static short s16(double x)
{
    return (short)clamp(x, -32767.0, 32767.0);
}

static void log_reset(void)
{
    log_idx = log_cnt = log_div = 0;
    srcbuf[0] = 'L'; srcbuf[1] = 'G'; srcbuf[2] = LOG_NVAR; srcbuf[3] = REC_DIV;
    srcbuf[4] = srcbuf[5] = srcbuf[6] = srcbuf[7] = 0;
}

static void log_sample(void)
{
    short v[LOG_NVAR];
    unsigned int i;
    uint8_t *p;
    if (++log_div < REC_DIV) return;
    log_div = 0;
    v[0] = s16(memmap.values.BODY_ANGLE * 1e4);                    /* 本体角 [0.1 mrad] */
    v[1] = s16(th_ref * 1e4);                                       /* 目標角 [0.1 mrad] */
    v[2] = s16(memmap.values.BODY_ANGULAR_SPD * 1e3);              /* 本体角速度 [mrad/s] */
    v[3] = s16((spd_l + spd_r) * 50.0);                            /* 制御に使った車輪角速度 平均 [0.01 rad/s] */
    v[4] = s16((memmap.values.T_SPD_L + memmap.values.T_SPD_R) * 50.0);                        /* 目標車輪角速度 [0.01 rad/s] */
    v[5] = s16((memmap.values.T_CURRENT_L + memmap.values.T_CURRENT_R) * 0.5);                  /* 電流指令 平均 */
    v[6] = s16((last_el + last_er) * 500.0);                        /* 車輪角のずれ 平均 [mrad] */
    v[7] = s16((int_l + int_r) * 500.0);                            /* ずれの積分 平均 [mrad*s] */
    p = srcbuf + LOG_HDR + log_idx * LOG_NVAR * 2;
    for (i = 0; i < LOG_NVAR; i++) {
        p[2 * i] = (uint8_t)(v[i] & 0xFF);
        p[2 * i + 1] = (uint8_t)((v[i] >> 8) & 0xFF);
    }
    log_idx = (log_idx + 1) % LOG_CAP;
    if (log_cnt < LOG_CAP) log_cnt++;
    srcbuf[4] = (uint8_t)log_idx; srcbuf[5] = (uint8_t)(log_idx >> 8);   /* 次に書く位置 */
    srcbuf[6] = (uint8_t)log_cnt; srcbuf[7] = (uint8_t)(log_cnt >> 8);   /* 有効サンプル数 */
}

static void reset_state(void)
{
    ClearVariables();
    ref_l = ref_r = int_l = int_r = 0.0;
    v_cmd = w_cmd = 0.0;
    a_cmd = th_ref = dth_ref = 0.0;
    boost_left = 0;
    ob_init = 0;
#if NOTCH > 0
    notch_init();
#endif
    btn_idle = btn_prev = memmap.values.PAD_BTN;
    memmap.values.T_SPD_L = memmap.values.T_SPD_R = 0.0;
}

int main(void)
{
    Balancer2Init();
    UARTInit(CBR_115200, non, 1);
    CurrentControlDisable();
    setLED(3);

    while (getSW() == 0) CycleProc();
    while (getSW() == 1) __NOP();
    setLED(2);
    Wait(1000);

    while (calibrarion() == 0) {
        CycleProc();
        LedBlink(0, 2, 150, MAIN_CYCLE);
    }
    reset_state();

    while (1) {
        while (getSW() == 0) {
            CycleProc();
            LedBlink(2, 1, 1000, MAIN_CYCLE);
        }
        while (getSW() == 1) __NOP();
        setLED(2);
        reset_state();
        memmap.values.LOG_MODE = 0;      /* 純正のログ機能は止めておく (srcbuf を自前で使う) */
        log_reset();
        CurrentControlEnable();

        while (1) {
            uint8_t slow = CycleProc();
            pad_to_speed();
            wheel_observer();
            control_v3();
            log_sample();
            if (slow == 1) {
                if (isWhleeOverSpeed() != 0) break;
                if (getSW() == 1) {
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

void LedBlink(unsigned char light, unsigned char blink, int msec, int callcycle)
{
    static int cnt = 0;
    volatile int c = (msec * callcycle / 1000);
    cnt++;
    setLED(cnt < c / 2 ? (light | blink) : light);
    if (cnt > c) cnt = 0;
}
