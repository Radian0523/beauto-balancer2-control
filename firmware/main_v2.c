/*
 * v2: 純正サンプル (Balancer2_firm_Default) の倒立制御をそのまま土台にした版。
 *
 *  自作の bb2_control (カルマン+LQI) はゲインが実機に対して弱すぎて立たなかったため、
 *  実機で立つことが確認できている純正の 4 状態フィードバックとゲイン (memmap の初期値) を使い、
 *  走行は純正の VS-C3 操縦と同じ仕組み (目標車輪角速度 T_SPD を積分して車輪角の目標にする) で行う。
 *
 *  動作:
 *    電源ON → ボタン → 校正(約5秒、寝かせて静止) → 手で立ててボタン
 *    → HOLD_S 秒その場で直立 → V_RUN [m/s] まで 1 秒かけて加速し直進 → ボタンで停止
 *
 *  make v2 SPEED=0.15 HOLD=15 のように速度と直立時間を変えられる。
 */
#include "Balancer2.h"
#include "control.h"

#ifndef V_RUN
#define V_RUN   (0.15)          /* 直進速度 [m/s] */
#endif
#ifndef HOLD_S
#define HOLD_S  (15.0)          /* 直立する時間 [s] (ゾーンA) */
#endif
#ifndef FIX_SPD_REF
#define FIX_SPD_REF 0           /* 1: 純正の角速度目標の単位ずれ(T_SPD/MAIN_CYCLE)を直す */
#endif

#ifndef RUN_DIR
#define RUN_DIR (1.0)           /* 前進の向き (+1 / -1) */
#endif

#define WHEEL_R     (0.021)     /* タイヤ半径 [m] (φ42mm) */
#define RAMP_S      (1.0)       /* 加速にかける時間 [s] */

void LedBlink(unsigned char light, unsigned char blink, int msec, int callcycle);

static double tspd_l_i = 0.0, tspd_r_i = 0.0;   /* 目標車輪角 (目標角速度の積分) */

/* 純正 Control() と同じ計算。違いは目標角の積分変数をここで持つことと、
 * FIX_SPD_REF で角速度目標の扱いを選べることだけ。 */
static void control_v2(void)
{
    double wl_a, wr_a, wl_s, wr_s, corr;
    double outL = 0.0, outR = 0.0;
    double body = memmap.values.GAIN_BODY * (memmap.values.GAIN_BODY_ANGLE * memmap.values.BODY_ANGLE
                + memmap.values.GAIN_BODY_ANGULAR_SPD * memmap.values.BODY_ANGULAR_SPD);
    outL += body;
    outR += body;

    tspd_l_i += memmap.values.T_SPD_L / MAIN_CYCLE;
    tspd_r_i += memmap.values.T_SPD_R / MAIN_CYCLE;

    wl_a = memmap.values.WHEEL_ANGLE_L - tspd_l_i;
    wr_a = memmap.values.WHEEL_ANGLE_R - tspd_r_i;
#if FIX_SPD_REF
    wl_s = memmap.values.WHEEL_ANGULAR_SPD_L - memmap.values.T_SPD_L;
    wr_s = memmap.values.WHEEL_ANGULAR_SPD_R - memmap.values.T_SPD_R;
#else
    wl_s = memmap.values.WHEEL_ANGULAR_SPD_L - (memmap.values.T_SPD_L / MAIN_CYCLE);
    wr_s = memmap.values.WHEEL_ANGULAR_SPD_R - (memmap.values.T_SPD_R / MAIN_CYCLE);
#endif

    outL += memmap.values.GAIN_WHEEL * (memmap.values.GAIN_WHEEL_ANGLE * wl_a
            + memmap.values.GAIN_WHEEL_ANGULAR_SPD * wl_s);
    outR += memmap.values.GAIN_WHEEL * (memmap.values.GAIN_WHEEL_ANGLE * wr_a
            + memmap.values.GAIN_WHEEL_ANGULAR_SPD * wr_s);

    corr = memmap.values.GAIN_WHEEL_CORRELATION * (memmap.values.GAIN_WHEEL_CORRELATION_ANGLE * (wl_a - wr_a)
         + memmap.values.GAIN_WHEEL_CORRELATION_ANGULAR_SPD * (wl_s - wr_s));
    outL -= corr;
    outR += corr;

    if (outL >  32767.0) outL =  32767.0;
    if (outL < -32767.0) outL = -32767.0;
    if (outR >  32767.0) outR =  32767.0;
    if (outR < -32767.0) outR = -32767.0;
    memmap.values.T_CURRENT_L = (short)outL;
    memmap.values.T_CURRENT_R = (short)outR;
}

/* 走行計画: 時刻から目標車輪角速度 [rad/s] を決める。
 * 純正 VS_C3Control では T_SPD = -(スティックY)。スティックは前に倒すと負なので前進は T_SPD 正のはず。
 * 逆に走ったら make v2 DIR=-1 で作り直す。 */
static void plan(unsigned long tick)
{
    double t = (double)tick / MAIN_CYCLE;
    double v = 0.0;
    if (t > HOLD_S) {
        double k = (t - HOLD_S) / RAMP_S;
        v = V_RUN * (k < 1.0 ? k : 1.0);
    }
    memmap.values.T_SPD_L = RUN_DIR * v / WHEEL_R;
    memmap.values.T_SPD_R = RUN_DIR * v / WHEEL_R;
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
    ClearVariables();

    while (1) {
        unsigned long tick = 0;
        while (getSW() == 0) {                       /* 立てて待つ */
            CycleProc();
            LedBlink(2, 1, 1000, MAIN_CYCLE);
        }
        while (getSW() == 1) __NOP();
        setLED(2);
        ClearVariables();
        tspd_l_i = tspd_r_i = 0.0;
        memmap.values.T_SPD_L = memmap.values.T_SPD_R = 0.0;
        CurrentControlEnable();

        while (1) {
            uint8_t slow = CycleProc();
            plan(tick++);
            control_v2();
            if (slow == 1) {
                if ((double)tick / MAIN_CYCLE > HOLD_S) setLED(3);   /* 走行中は両方点灯 */
                if (isWhleeOverSpeed() != 0) break;                  /* 転倒 */
                if (getSW() == 1) {
                    while (getSW() == 1) __NOP();
                    break;
                }
            }
        }
        CurrentControlDisable();
        memmap.values.T_SPD_L = memmap.values.T_SPD_R = 0.0;
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
