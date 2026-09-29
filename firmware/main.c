/*
 * Beauto Balancer 2 倒立制御ファーム v5 (コントローラ操縦)
 *
 * 純正サンプルの倒立制御 (vendor/control.c の Control()) を土台にしている。
 * フィードバックのゲインは純正のまま触らず、目標の与え方と推定を足した。
 *
 * 1 周期 (500Hz) の流れ (main の走行ループ)
 *   CycleProc()       [純正ライブラリ] センサー読み取り・傾き推定・周期待ち
 *   gyro_trim()       ジャイロの 0 点を走行中も少しずつ合わせ込む (校正のずれ対策)
 *   pad_to_speed()    ① スティック → 滑らかな目標軌道 + 目標前傾 (フィードフォワード)
 *   wheel_observer()  ② 車輪角 → なめらかな車輪角速度 (オブザーバ)
 *   balance_zero()    釣り合う角度を覚え直す (スタート時の持ち方に左右されないように)
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
#define V_MAX     (0.3)     /* VMAX: スティックを倒しきったときの速度 [m/s]。0.5 だと坂の入り口で力が足りなくなる (逆起電力) */
#endif
#ifndef V_BACK
#define V_BACK    (0.3)     /* VBACK: 後退の最高速度 [m/s]。後退 0.47 m/s からの減速で倒れたので前進より低くする */
#endif
#ifndef V_SLOW
#define V_SLOW    (0.08)    /* VSLOW: ボタンを押している間に自動で進む速さ [m/s] (ゆっくりモード。シーソーを渡るとき)。0 で無効 */
#endif
#ifndef V_FAST
#define V_FAST    (0.5)     /* VFAST: R2 を押している間に自動で進む速さ [m/s] (いそいそモード。段差を勢いで越えるとき)。0 で無効 */
#endif
#ifndef BTN_SLOW
#define BTN_SLOW  (0x0008)  /* BTNSLOW: ゆっくりモードのボタン (R1) の PAD_BTN のビット。./bb2 pad の「押されたビット」で確かめた値 */
#endif
#ifndef BTN_FAST
#define BTN_FAST  (0x0002)  /* BTNFAST: いそいそモードのボタン (R2) の PAD_BTN のビット */
#endif
#ifndef BTN_UP
#define BTN_UP    (0x0020)  /* BTNUP: いそいそモードの速さを上げるボタン (◯) のビット */
#endif
#ifndef BTN_DOWN
#define BTN_DOWN  (0x0040)  /* BTNDOWN: 下げるボタン (✖︎) のビット */
#endif
#define V_FAST_STEP (0.05)          /* ◯/✖︎ 1 回で変える速さ [m/s] */
#define V_FAST_MIN  (0.1)
#define V_FAST_MAX  (0.7)           /* 0.9 m/s あたりが電圧の限界 (実験 B) なので余裕を残す */
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
#define E_MAX     (1.0)     /* EMAX: 走行中の車輪角のずれの上限 [rad] (1 rad = 2.1 cm)。4 だと大きく飛び出したあと 8 cm 取り戻そうとして
                               * 「進みすぎてキュッと戻る」を繰り返し、坂やシーソーで倒れた */
#endif
#ifndef K_CP
#define K_CP      (4.0)     /* KCP: 純正ライブラリの電流制御の P ゲインの倍率 (純正 = 1)。4 で電流の立ち上がりが 93ms → 5ms、走りがはっきり安定した */
#endif
#ifndef K_CI
#define K_CI      (1.0)     /* KCI: 同じく I ゲインの倍率 */
#endif
#ifndef K_BA
#define K_BA      (1.0)     /* KBA: 本体の傾きのゲイン (K1) の倍率 (純正 = 1)。1.3 にすると坂を 0.3 m/s で登るとき行きすぎと戻りを繰り返して倒れた */
#endif
#ifndef K_WA
#define K_WA      (1.0)     /* KWA: 車輪の位置のゲイン (K3、積分にも効く) の倍率 (純正 = 1) */
#endif
#ifndef K_BR
#define K_BR      (1.0)     /* KBR: 本体角速度のゲインに掛ける倍率 (純正 = 1)。細かい振動を減らす試し用 */
#endif
#ifndef K_WR
#define K_WR      (1.0)     /* KWR: 車輪角速度のゲイン (K4) の倍率 (純正 = 1) */
#endif
#ifndef K_BA_S
#define K_BA_S    (1.0)     /* KBAS: ゆっくりモード (R1) の間だけ使う KBA。1.3 は EMAX=4 のときシーソー用に使っていたが、渡る途中が揺れる */
#endif
#ifndef K_WR_S
#define K_WR_S    (1.0)     /* KWRS: ゆっくりモードの間だけ使う KWR。1.25 だと渡る途中の揺れが 5 → 60°/s。EMAX=1 なら 1 でシーソーを完走 */
#endif
/* シーソー逆向き用のゆっくりモード (L1)。逆向きは揺れ方がきついので、R1 とは別に速さ・ゲイン・ずれの上限を変えられる。
 * 既定は R1 と同じ。make の引数で変えて試す */
#ifndef BTN_SLOW2
#define BTN_SLOW2 (0x0004)  /* BTNSLOW2: 逆向きのゆっくりモードのボタン (L1) のビット */
#endif
#ifndef V_SLOW2
#define V_SLOW2   (0.08)    /* VSLOW2: L1 を押している間に自動で進む速さ [m/s]。0 で無効 */
#endif
#ifndef K_BA_S2
#define K_BA_S2   (1.0)     /* KBAS2: L1 の間だけ使う KBA */
#endif
#ifndef K_WR_S2
#define K_WR_S2   (1.0)     /* KWRS2: L1 の間だけ使う KWR */
#endif
#ifndef E_MAX_S2
#define E_MAX_S2  (1.0)     /* EMAX2: L1 の間だけ使う、走行中の車輪角のずれの上限 [rad] */
#endif
#define SLOW_GAIN_T (0.3)   /* ゲインをゆっくりモード用に切り替えるのにかける時間 [s]。一度に変えると蹴られる */
#ifndef S_MAX
#define S_MAX     (0.4)     /* SMAX: 車輪の速さのずれ (前後の平均) の上限 [m/s]。0 で無効。
                             * シーソーが傾いた瞬間などに車輪が走り出したとき、速さの項が上限を超える力を要求しないように */
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
#ifndef GYRO_TRIM_T
#define GYRO_TRIM_T (5.0)   /* GTRIM: 走行中のジャイロ 0 点補正の時定数 [s]。0 で無効 */
#endif
#ifndef BL_I
#define BL_I      (0)       /* BL: ギアのガタの補償の強さ [電流指令]。0 で無効 */
#endif
#ifndef BL_W
#define BL_W      (500.0)   /* BLW: 補償が効き始める幅 [電流指令]。小さいほど向きの切り替わりが急 */
#endif
#ifndef BZ_T
#define BZ_T      (2.0)     /* BZT: 釣り合う角度を覚え直す時定数 [s] (スタート直後 BZ_T0 秒は BZ_T_FAST)。0 で無効 */
#endif
#ifndef CAL_RANGE
#define CAL_RANGE (80)      /* CAL_RANGE: 校正中のジャイロの揺れ (最大 - 最小) がこれを超えたらやり直す [LSB]。走行中の 0 点補正があるので緩め (30 だと温まる途中で終わらない) */
#endif
#ifndef REC_DIV
#define REC_DIV   (20)      /* LOGDIV: ログの間引き。500Hz / 20 = 25Hz で約 10 秒ぶん */
#endif
#ifndef DIAG
#define DIAG      (0)       /* DIAG=1: ジャイロ診断用の記録に切り替える (read_log.py が自動で判別) */
#endif
#ifndef TEST
#define TEST      (0)       /* TEST: 実験モード (制御しない)。1 = モータを切ったまま記録 (倒す実験)、2 = 決まった電流を前後交互に流す (車輪を浮かせて空回し) */
#endif
#ifndef TEST_AMP
#define TEST_AMP  (3000)    /* TAMP: TEST=2 で流す電流指令 */
#endif
#ifndef TEST_HZ
#define TEST_HZ   (2)       /* THZ: TEST=2 で前後を切り替える回数 [往復/s] */
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
static double v_fast = V_FAST;      /* いそいそモードの速さ。◯/✖︎ で変える (電源を切るまで覚えている) */
static unsigned short btn_last;     /* ◯/✖︎ の押した瞬間を見るための前回の値 */
static int slow_on;                 /* ゆっくりモード: 0 = 押していない、1 = R1 (順向き)、2 = L1 (逆向き) */
static double kba_now, kwr_now, emax_now;   /* 今使っている KBA・KWR・ずれの上限 (モードの値へ SLOW_GAIN_T かけて近づける) */
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
    /* ゆっくりモード: R1 を押している間は、スティックに触らなくても V_SLOW で前に進み続ける。
     * スティックを後ろに引いたときだけ減速・後退する (後ろに倒しきると -V_SLOW)。旋回は半分の速さ。
     * シーソーは止まった状態で傾くと揺れが育って倒れる。傾きそうになると思わずスティックを戻してしまうので
     * (13:38 に失敗した回)、ボタンを押し続けるだけで一定の速さで渡りきれるようにする */
    /* いそいそモード: R2 を押している間は、同じように V_FAST で前に進み続ける (段差を勢いで越えるとき)。
     * 押しているかどうかは、スタート時のボタンの値からどのビットが変わったかで見る */
    {
        unsigned short pressed = (unsigned short)(btn ^ btn_idle);
        double s = -stick(memmap.values.PAD_AN_LY, ly0);        /* 前 +1 〜 後ろ -1 */
        /* ◯ を押した瞬間に速く、✖︎ を押した瞬間に遅く */
        unsigned short down = (unsigned short)(pressed & ~(btn_last ^ btn_idle));   /* 今回新しく押されたビット */
        if (down & BTN_UP)   v_fast = clamp(v_fast + V_FAST_STEP, V_FAST_MIN, V_FAST_MAX);
        if (down & BTN_DOWN) v_fast = clamp(v_fast - V_FAST_STEP, V_FAST_MIN, V_FAST_MAX);
        btn_last = btn;
        memmap.values.USER_ARIA4 = v_fast;
        /* 確認用: USB で ./bb2 pad から見えるように memmap の空き欄に書き出す */
        memmap.values.USER_ARIA1 = btn_idle;
        memmap.values.USER_ARIA2 = pressed;
        slow_on = 0;
        if (V_SLOW > 0 && V_BOOST <= 0 && (pressed & BTN_SLOW)) slow_on = 1;
        else if (V_SLOW2 > 0 && V_BOOST <= 0 && (pressed & BTN_SLOW2)) slow_on = 2;
        if (slow_on) {
            double vs = (slow_on == 1) ? V_SLOW : V_SLOW2;
            v_tgt = (s < 0) ? vs * (1.0 + 2.0 * s) : vs;            /* 引かなければ +vs、引ききると -vs */
            w_tgt *= 0.5;
        } else if (V_FAST > 0 && V_BOOST <= 0 && (pressed & BTN_FAST)) {
            v_tgt = (s < 0) ? v_fast * (1.0 + 2.0 * s) : v_fast;
            if (v_tgt < -V_BACK) v_tgt = -V_BACK;
        }
    }

    a_des = clamp(K_V * (v_tgt - v_cmd), -amax, amax);                          /* 速度の差 → 欲しい加速度 */
    a_new = a_cmd + clamp(a_des - a_cmd, -JERK / MAIN_CYCLE, JERK / MAIN_CYCLE); /* 加速度は急に変えない */
    dth_ref = -FF_K * (a_new - a_cmd) * MAIN_CYCLE / GRAV;
    a_cmd = a_new;
    v_cmd += a_cmd / MAIN_CYCLE;
    th_ref = -FF_K * a_cmd / GRAV;                                              /* 目標前傾 = -a/g */
    ramping = (a_cmd > 0.01 || a_cmd < -0.01);
    memmap.values.USER_ARIA3 = v_tgt;                            /* 確認用: 目標速度 (ボタンのモードを反映したあと) */

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

/* ============================================================ 釣り合う角度の推定
 *
 * 純正の傾き推定は、スタートボタンを押した瞬間の姿勢を 0° としてジャイロを積分したもの。
 * 本当の鉛直はわからない (加速度センサが無い) ので、押したときの持ち方がそのまま傾きのずれとして
 * 最後まで残る (本番コースで -17° を実測。昨日の ±2〜20° のばらつきも主にこれ)。
 * 機体が立っていて加速していない間は、本当の傾きの平均は必ず「釣り合う角度」になる (物理的に決まる)。
 * そこで、加速していない間の (傾き推定 - 目標前傾) の平均を釣り合う角度 th0 として覚え、
 * ③ では th0 からのずれを使う。
 *   - スタート直後 BZ_T0 秒は速く (BZ_T_FAST)、その後は BZ_T でゆっくり覚え直す
 *   - 1 回に効かせるずれは ±BZ_STEP まで (倒れる途中や段差の衝撃に引っ張られないように)
 *   - th0 は ±BZ_MAX まで
 *   - 加速・減速中は覚えない (前傾フィードフォワードとの差を釣り合う角度と取り違えないように)
 */
#define BZ_T0     (3.0)                 /* [s] */
#define BZ_T_FAST (0.7)                 /* [s] */
#define BZ_STEP   (0.1)                 /* [rad] 約 6° */
#define BZ_MAX    (0.44)                /* [rad] 約 25° */
static double th0;                      /* 釣り合う角度 (純正の傾き推定の目盛りで) [rad] */
static int bz_tick;                     /* スタートからの周期数 */

static void balance_zero(void)
{
#if BZ_T > 0
    double tau = (bz_tick < (int)(BZ_T0 * MAIN_CYCLE)) ? BZ_T_FAST : BZ_T;
    if (bz_tick < (int)(BZ_T0 * MAIN_CYCLE)) bz_tick++;
    if (!ramping) {
        double e = clamp(memmap.values.BODY_ANGLE - th_ref - th0, -BZ_STEP, BZ_STEP);
        th0 = clamp(th0 + e / (tau * MAIN_CYCLE), -BZ_MAX, BZ_MAX);
    }
#endif
}

/* ============================================================ ギアのガタの補償
 *
 * 平地では、バランスを取る力の平均がほぼ 0 なので、モータの力の向きが頻繁に入れ替わる
 * (電流指令の符号が 79% の周期で入れ替わる)。そのたびにギアがガタ (歯のすき間) の中を空回りして歯がぶつかり、
 * これが 7〜11Hz の細かい振動 (キュッキュッ) になっている。坂を登るときは力がずっと同じ向きなので
 * ガタを行き来せず、振動が 1/10 になった (本体角速度の揺れ 56 → 6°/s)。
 * そこで、力の向きに合わせて BL_I だけ電流を足し、向きが入れ替わったときにすき間を素早く詰める。
 * 符号をそのまま使うと 0 付近で激しく切り替わるので、u / (|u| + BL_W) でなめらかにする。
 */
static double backlash(double u)
{
    double a = u < 0 ? -u : u;
    return u + BL_I * u / (a + BL_W);
}

/* ============================================================ ③ 状態フィードバック
 *
 * 純正 Control() と同じ形: 本体の傾き・傾く速さ・車輪角のずれ・車輪角速度のずれ に純正ゲインを掛けて足す。
 * 純正から変えたところ ([v5] の行)
 *   - 本体の傾きは直立ではなく、釣り合う角度 th0 + ① の目標前傾 th_ref からのずれにする
 *   - 走行中だけ車輪角のずれ (左右の平均) に上限 E_MAX をかける (段差対策)
 *   - 車輪角速度は ② のオブザーバの値を使い、純正の単位の間違い (T_SPD / MAIN_CYCLE) を直す
 *   - 車輪角のずれ (左右の平均) の積分を左右に同じだけ足す (坂対策)
 *   - 最後にギアのガタの補償をかける (BL_I > 0 のとき)
 */
static double ref_l, ref_r;         /* 目標車輪角 [rad] */
static double int_m;                /* 車輪角のずれ (左右の平均) の積分 [rad*s] */
static double last_el, last_er;     /* 直近の車輪角のずれ (ログ用) */

static void control(void)
{
    double el, er, sl, sr, corr;
    double outL, outR;
    double kw  = memmap.values.GAIN_WHEEL;
    double kwa = memmap.values.GAIN_WHEEL_ANGLE * K_WA;
    double kba, kwr, emax, body;
    /* ゆっくりモード (R1 / L1) の間は、そのモード用のゲインとずれの上限を使う (シーソー用)。
     * 一度に変えると蹴られるので、1 の変化に SLOW_GAIN_T 秒かける速さで近づける */
    {
        const double step = 1.0 / (SLOW_GAIN_T * MAIN_CYCLE);
        double tba = (slow_on == 1) ? K_BA_S : (slow_on == 2) ? K_BA_S2 : K_BA;
        double twr = (slow_on == 1) ? K_WR_S : (slow_on == 2) ? K_WR_S2 : K_WR;
        double tem = (slow_on == 2) ? E_MAX_S2 : E_MAX;
        kba_now += clamp(tba - kba_now, -step, step);
        kwr_now += clamp(twr - kwr_now, -step, step);
        emax_now += clamp(tem - emax_now, -step, step);
    }
    kba = kba_now;
    kwr = kwr_now;
    emax = emax_now;
    /* 本体の項 [v5] 目標前傾からのずれ */
    body = memmap.values.GAIN_BODY * (memmap.values.GAIN_BODY_ANGLE * kba * (memmap.values.BODY_ANGLE - th0 - th_ref)
                + memmap.values.GAIN_BODY_ANGULAR_SPD * K_BR * (memmap.values.BODY_ANGULAR_SPD - dth_ref));

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
        if (em >  emax) d = em - emax;
        if (em < -emax) d = em + emax;
        ref_l += d; ref_r += d;
        el -= d; er -= d;
    }

    /* [v5] 車輪角速度のずれ。純正は T_SPD / MAIN_CYCLE を引いていて、走行中は速度の項がブレーキになっていた */
    sl = spd_l - memmap.values.T_SPD_L;
    sr = spd_r - memmap.values.T_SPD_R;
    /* 速さのずれの上限は前後の平均だけにかけ、左右の差 (向き) はそのまま */
    if (S_MAX > 0) {
        double sm = (sl + sr) * 0.5, sd = (sl - sr) * 0.5;
        sm = clamp(sm, -S_MAX / WHEEL_R, S_MAX / WHEEL_R);
        sl = sm + sd;
        sr = sm - sd;
    }

    outL = body + kw * (kwa * el + memmap.values.GAIN_WHEEL_ANGULAR_SPD * kwr * sl);
    outR = body + kw * (kwa * er + memmap.values.GAIN_WHEEL_ANGULAR_SPD * kwr * sr);

#if TI_S > 0
    /* [v5] 坂対策: 坂では重力で押し戻され続け、純正は「ずれた分だけ押し返す」だけなので下がりっぱなしになる。
     * ずれを積分して足すと、坂の力を覚えて打ち消し、元の位置に戻る。
     * 加速中の遅れは溜めない (溜めると加速後に余計な力として一気に出る)。
     * 積分は左右の平均 (前後の位置のずれ) だけにかけ、左右に同じだけ足す。左右別々に積分すると、
     * 向きのずれが左右逆向きに溜まって向きのずれを広げる向きに働き、左右が上限 (±I_CLIP) に逆向きに張り付いて
     * 左右のモータが ±1 万の力で押し合ったまま固まっていた (向きのずれ 約 13cm、前後に使える力が残らない)。 */
    if (!ramping)
        int_m = clamp(int_m + (el + er) * 0.5 / MAIN_CYCLE, -I_CLIP * TI_S, I_CLIP * TI_S);
    outL += kw * kwa * int_m / TI_S;
    outR += kw * kwa * int_m / TI_S;
#endif

    /* 左右差の項 (純正と同じ): 左右の車輪をそろえてまっすぐ走る */
    corr = memmap.values.GAIN_WHEEL_CORRELATION * (memmap.values.GAIN_WHEEL_CORRELATION_ANGLE * (el - er)
         + memmap.values.GAIN_WHEEL_CORRELATION_ANGULAR_SPD * (sl - sr));
    outL -= corr;
    outR += corr;

    outL = backlash(outL);
    outR = backlash(outR);

    last_el = el;
    last_er = er;
    memmap.values.T_CURRENT_L = (short)clamp(outL, -32767.0, 32767.0);
    memmap.values.T_CURRENT_R = (short)clamp(outR, -32767.0, 32767.0);
}

/* ============================================================ ジャイロの 0 点の補正
 *
 * 校正 (5 秒間の平均) の 0 点は回によって数〜数十 LSB ずれ、そのぶん純正の傾き推定が
 * 一定の角度だけずれる (1 LSB で約 0.2°。-2〜-17° を実測)。機体は「傾いている」と思い込んで押し続け、
 * 止まっていても前に出る・車輪のずれが上限に張り付く・積分が上限に溜まる、の原因になっていた。
 * 立っている限り、傾く速さを長い時間で平均すると 0 になる (傾きは行ったり来たりするだけ) ので、
 * ジャイロの生の値を時定数 GYRO_TRIM_T 秒で平均したものを 0 点として使い続ける。
 *   - 1 回に効かせる値は ±TRIM_CLIP に制限 (11Hz の振動や倒れる瞬間の大きな値に引っ張られないように)
 *   - 校正の値から ±TRIM_MAX より外には動かさない
 *   - 倒れて止まったときは、0.5〜1 秒前の値に戻す (倒れる途中の値を次の走行に持ち越さない)
 */
#define TRIM_CLIP (1000.0)          /* [LSB] 約 300°/s。立っているときの振動 (±100°/s 程度) は切らない */
#define TRIM_MAX  (200.0)           /* [LSB] 走行中は校正時より 60 LSB 以上ずれることがあった */
static double cal_offset;           /* 校正で決まった 0 点 [LSB] */
static double trim_hist[2];         /* 0.5 秒ごとの 0 点の履歴 ([0] が新しい) */
static int trim_tick;

static void gyro_trim_reset(void)
{
    trim_hist[0] = trim_hist[1] = memmap.values.BODY_ANGULAR_SPD_OFFSET;
    trim_tick = 0;
}

static void gyro_trim(void)
{
#if GYRO_TRIM_T > 0
    double g = clamp(memmap.values.GYRO_DATA - memmap.values.BODY_ANGULAR_SPD_OFFSET, -TRIM_CLIP, TRIM_CLIP);
    double off = memmap.values.BODY_ANGULAR_SPD_OFFSET + g / (GYRO_TRIM_T * MAIN_CYCLE);
    memmap.values.BODY_ANGULAR_SPD_OFFSET = clamp(off, cal_offset - TRIM_MAX, cal_offset + TRIM_MAX);
    if (++trim_tick >= MAIN_CYCLE / 2) {
        trim_tick = 0;
        trim_hist[1] = trim_hist[0];
        trim_hist[0] = memmap.values.BODY_ANGULAR_SPD_OFFSET;
    }
#endif
}

/* ============================================================ 走行ログ
 *
 * 走行中の値を機体の RAM (純正ライブラリのログ用バッファ srcbuf, 4KB) にリングバッファで記録する。
 * USB を繋がずに走らせ、止めたあと電源を切らずに USB を挿して ../bb2 log で吸い出す。
 * srcbuf は USB の 'R' コマンドで 0x1000 番地から読める (純正のログ機能は止めているので衝突しない)。
 *
 * 形式: 先頭 8 バイト = 'L', 'G'(通常)/'D'(診断), 変数の数, 間引き, 次に書く位置(2), 有効サンプル数(2)
 *       その後に 1 サンプル = int16 × 13 (リトルエンディアン)
 */
#ifndef LOG_SET
#define LOG_SET   (1)       /* LOGSET: 1 = ふつうの記録 (13 変数)、2 = 小さい記録 (6 変数、約 2.2 倍の長さ) */
#endif
#if LOG_SET == 2 && DIAG
#error "DIAG=1 は LOGSET=1 で使う"
#endif
#if LOG_SET == 2
#define LOG_NVAR  (6)       /* 本体角・本体角速度・車輪角速度・電流指令・実際の電流・車輪角のずれ */
#else
#define LOG_NVAR  (13)
#endif
#define LOG_HDR   (8)
#define LOG_CAP   ((4096 - LOG_HDR) / (LOG_NVAR * 2))   /* 13 変数: 157 サンプル (25Hz で 6.3 秒)、6 変数: 340 サンプル (13.6 秒) */
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
#if LOG_SET == 2
    v[0] = s16(memmap.values.BODY_ANGLE * 1e4);                                 /* 本体角 [0.1 mrad] */
    v[1] = s16(memmap.values.BODY_ANGULAR_SPD * 1e3);                           /* 本体角速度 [mrad/s] */
    v[2] = s16((spd_l + spd_r) * 50.0);                                         /* 車輪角速度 (オブザーバ) 平均 [0.01 rad/s] */
    v[3] = s16((memmap.values.T_CURRENT_L + memmap.values.T_CURRENT_R) * 0.5);  /* 電流指令 平均 */
    v[4] = s16(((memmap.values.M_CURRENT_L - memmap.values.CURRENT_OFFSET_L)
              + (memmap.values.M_CURRENT_R - memmap.values.CURRENT_OFFSET_R)) * 0.5);  /* 実際の電流 平均 (生値) */
    v[5] = (short)(long)((memmap.values.WHEEL_ANGLE_L + memmap.values.WHEEL_ANGLE_R) * 500.0);  /* 車輪角 平均 [mrad]。±32 rad を超えると折り返す (読むときにつなぐ) */
#else
    v[0] = s16(memmap.values.BODY_ANGLE * 1e4);                                 /* 本体角 [0.1 mrad] */
    v[1] = s16(th_ref * 1e4);                                                   /* 目標前傾 [0.1 mrad] */
    v[2] = s16(memmap.values.BODY_ANGULAR_SPD * 1e3);                           /* 本体角速度 [mrad/s] */
    v[3] = s16((spd_l + spd_r) * 50.0);                                         /* 車輪角速度 (オブザーバ) 平均 [0.01 rad/s] */
    v[4] = s16((memmap.values.T_SPD_L + memmap.values.T_SPD_R) * 50.0);         /* 目標車輪角速度 平均 [0.01 rad/s] */
    v[5] = s16((memmap.values.T_CURRENT_L + memmap.values.T_CURRENT_R) * 0.5);  /* 電流指令 平均 */
    v[6] = s16((last_el + last_er) * 500.0);                                    /* 車輪角のずれ 平均 [mrad] */
    v[7] = s16(int_m * 1000.0);                                                 /* ずれの積分 (左右の平均) [mrad*s] */
    v[8] = s16(((memmap.values.M_CURRENT_L - memmap.values.CURRENT_OFFSET_L)
              + (memmap.values.M_CURRENT_R - memmap.values.CURRENT_OFFSET_R)) * 0.5);  /* 実際の電流 平均 (生値) */
    v[9] = s16((last_el - last_er) * 1e3);                                      /* 向きのずれ = 左右の車輪角のずれの差 [mrad] */
    v[10] = s16(w_cmd * 100.0);                                                 /* 旋回の指令 [0.01 rad/s] */
    v[11] = s16((memmap.values.BODY_ANGULAR_SPD_OFFSET - cal_offset) * 100.0);  /* ジャイロ 0 点の補正量 [0.01 LSB] */
    v[12] = s16(th0 * 1e4);                                                     /* 釣り合う角度 (推定) [0.1 mrad] */
#endif
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
    ref_l = ref_r = int_m = 0.0;
    v_cmd = w_cmd = 0.0;
    a_cmd = th_ref = dth_ref = 0.0;
    th0 = 0.0;
    bz_tick = 0;
    boost_left = 0;
    slow_on = 0;
    kba_now = K_BA;
    kwr_now = K_WR;
    emax_now = E_MAX;
    ob_init = 0;
    btn_idle = btn_prev = btn_last = memmap.values.PAD_BTN;
    ly0 = neutral(memmap.values.PAD_AN_LY);
    rx0 = neutral(memmap.values.PAD_AN_RX);
    memmap.values.T_SPD_L = memmap.values.T_SPD_R = 0.0;
}

/* ============================================================ main */

#define FALL_ANGLE (0.785)          /* 転倒とみなす傾き [rad] (45°) */
#define FALL_T     (0.1)            /* その傾きが続いたら止める時間 [s] */
static int fall_cnt;

int main(void)
{
    Balancer2Init();
    UARTInit(CBR_115200, non, 1);
    /* 純正の電流制御は遅い (指令 1500 の段で実電流が上がりきるまで約 100ms、実験 B)。
     * ゲインは memmap にあるので、起動時に倍率を掛ける (フラッシュには保存しない) */
    memmap.values.GAIN_CURRENT_P *= K_CP;
    memmap.values.GAIN_CURRENT_I *= K_CI;
    CurrentControlDisable();
    setLED(3);

    /* ボタンを押すまで待ってから校正 */
    while (getSW() == 0) CycleProc();
    while (getSW() == 1) __NOP();
    setLED(2);
    Wait(1000);

    calibrate_checked();
    cal_offset = memmap.values.BODY_ANGULAR_SPD_OFFSET;
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
#if TEST > 0
        /* 実験モード: 制御はせず記録だけ取る。ボタンで終了して待機に戻る (記録は残る)。
         *   TEST=1: モータを切ったまま。機体を立てて手を離し、倒れる様子を記録する (重力で倒れる速さを測る)
         *   TEST=2: 左右同じ電流 ±TEST_AMP を TEST_HZ 往復/s で流す。機体を持ち上げて車輪を浮かせておく
         *           (車輪とモータの重さ・摩擦、電流が力になるまでの遅れ、ギアのガタを測る) */
        {
            int k = 0;
            if (TEST == 2) CurrentControlEnable();
            while (1) {
                uint8_t slow = CycleProc();
                if (TEST == 2) {
                    short u = ((k / (MAIN_CYCLE / (2 * TEST_HZ))) % 2) ? TEST_AMP : -TEST_AMP;
                    memmap.values.T_CURRENT_L = memmap.values.T_CURRENT_R = u;
                }
                wheel_observer();
                log_sample();
                k++;
                LedBlink(0, 3, 100, MAIN_CYCLE);
                if (slow == 1 && getSW() == 1) {
                    while (getSW() == 1) __NOP();
                    break;
                }
            }
            CurrentControlDisable();
            memmap.values.T_CURRENT_L = memmap.values.T_CURRENT_R = 0;
            reset_state();
            Wait(1000);
            continue;
        }
#endif
        CurrentControlEnable();
        gyro_trim_reset();

        /* 走行 (500Hz) */
        fall_cnt = 0;
        while (1) {
            uint8_t slow = CycleProc();
            gyro_trim();
            pad_to_speed();
            wheel_observer();
            balance_zero();
            control();
            log_sample();
            /* 転倒の検出: 釣り合う角度から FALL_ANGLE 以上傾いた状態が FALL_T 秒続いたら止める。
             * 純正の isWhleeOverSpeed() は車輪 62.8 rad/s 超えで止めるが、この機体は全力でも 44〜50 rad/s しか
             * 回らない (電圧の限界) ので一度も働かず、倒れたあとも全力で空回りし続けていた。 */
            {
                double tilt = memmap.values.BODY_ANGLE - th0;
                fall_cnt = (tilt > FALL_ANGLE || tilt < -FALL_ANGLE) ? fall_cnt + 1 : 0;
            }
            if (fall_cnt > (int)(FALL_T * MAIN_CYCLE)) {
                memmap.values.BODY_ANGULAR_SPD_OFFSET = trim_hist[1];   /* 倒れる前の 0 点に戻す */
                break;
            }
            if (slow == 1) {
                if (isWhleeOverSpeed() != 0) {          /* 転倒 (車輪の回りすぎ) で停止 */
                    memmap.values.BODY_ANGULAR_SPD_OFFSET = trim_hist[1];   /* 倒れる前の 0 点に戻す */
                    break;
                }
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
