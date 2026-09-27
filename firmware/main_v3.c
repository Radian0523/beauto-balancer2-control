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

#define WHEEL_R    (0.021)                 /* タイヤ半径 [m] */
#define TURN_MAX   (9.42477796)            /* 旋回時の左右差 [rad/s] (純正と同じ) */
#define DEADZONE   (10)                    /* スティックの中立の遊び */
#define I_CLIP     (3.0)                   /* 積分の寄与を車輪角 3 rad ぶんまでに制限 */

void LedBlink(unsigned char light, unsigned char blink, int msec, int callcycle);

static double ref_l, ref_r;       /* 目標車輪角 [rad] */
static double int_l, int_r;       /* 車輪角ずれの積分 [rad*s] */
static double v_cmd, w_cmd;       /* 加速度制限後の前進速度 [m/s]、旋回 [rad/s] */
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

    if (btn != btn_idle && btn_prev == btn_idle) boost_left = (int)(BOOST_T * MAIN_CYCLE);  /* 押した瞬間 */
    btn_prev = btn;
    if (boost_left > 0) {
        boost_left--;
        v_tgt = V_BOOST;
        dv = ACC_BOOST / MAIN_CYCLE;
    }

    ramping = (v_tgt - v_cmd > 1e-4 || v_tgt - v_cmd < -1e-4);
    v_cmd += clamp(v_tgt - v_cmd, -dv, dv);
    w_cmd += clamp(w_tgt - w_cmd, -dw, dw);
    memmap.values.T_SPD_L = v_cmd / WHEEL_R + w_cmd;
    memmap.values.T_SPD_R = v_cmd / WHEEL_R - w_cmd;
}

static void control_v3(void)
{
    double el, er, sl, sr, corr;
    double outL, outR;
    double body = memmap.values.GAIN_BODY * (memmap.values.GAIN_BODY_ANGLE * memmap.values.BODY_ANGLE
                + memmap.values.GAIN_BODY_ANGULAR_SPD * memmap.values.BODY_ANGULAR_SPD);
    double kw  = memmap.values.GAIN_WHEEL;
    double kwa = memmap.values.GAIN_WHEEL_ANGLE;

    ref_l += memmap.values.T_SPD_L / MAIN_CYCLE;
    ref_r += memmap.values.T_SPD_R / MAIN_CYCLE;

    /* 段差対策: ずれが E_MAX を超えたら目標のほうを引き戻す (押す力は E_MAX ぶんで頭打ち) */
    el = memmap.values.WHEEL_ANGLE_L - ref_l;
    er = memmap.values.WHEEL_ANGLE_R - ref_r;
    if (el >  E_MAX) { ref_l = memmap.values.WHEEL_ANGLE_L - E_MAX; el =  E_MAX; }
    if (el < -E_MAX) { ref_l = memmap.values.WHEEL_ANGLE_L + E_MAX; el = -E_MAX; }
    if (er >  E_MAX) { ref_r = memmap.values.WHEEL_ANGLE_R - E_MAX; er =  E_MAX; }
    if (er < -E_MAX) { ref_r = memmap.values.WHEEL_ANGLE_R + E_MAX; er = -E_MAX; }

#if FIX_SPD_REF
    sl = memmap.values.WHEEL_ANGULAR_SPD_L - memmap.values.T_SPD_L;
    sr = memmap.values.WHEEL_ANGULAR_SPD_R - memmap.values.T_SPD_R;
#else
    sl = memmap.values.WHEEL_ANGULAR_SPD_L - memmap.values.T_SPD_L / MAIN_CYCLE;
    sr = memmap.values.WHEEL_ANGULAR_SPD_R - memmap.values.T_SPD_R / MAIN_CYCLE;
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

    memmap.values.T_CURRENT_L = (short)clamp(outL, -32767.0, 32767.0);
    memmap.values.T_CURRENT_R = (short)clamp(outR, -32767.0, 32767.0);
}

static void reset_state(void)
{
    ClearVariables();
    ref_l = ref_r = int_l = int_r = 0.0;
    v_cmd = w_cmd = 0.0;
    boost_left = 0;
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
        CurrentControlEnable();

        while (1) {
            uint8_t slow = CycleProc();
            pad_to_speed();
            control_v3();
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
