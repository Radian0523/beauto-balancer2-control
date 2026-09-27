/*
 * ビュートバランサー2 本体側 main (LPCXpresso プロジェクトの src/ に置く)
 *
 *  純正サンプル Balancer2_firm_Simple の main.c を下敷きにして、
 *  Control() の中身を bb2_control.c (カルマンフィルタ + LQIサーボ) に差し替えたもの。
 *
 *  使い方:
 *    1. 電源ON → LED1,2点灯。ボタンを押すとキャリブレーション(静止させておく)
 *    2. LED2点滅が終わったら機体を手で立てる → ボタンでスタート
 *    3. ゾーンA(15秒直立) → 自動で走行モードへ
 *    4. 走行中にボタンを押すと停止
 *
 *  ★最初に必ず確認すること★ (符号の向き。ここを間違えると一発で倒れる)
 *    - 機体を前に傾けたとき memmap.values.BODY_ANGULAR_SPD が正か負か
 *    - T_CURRENT_L に +300 を書いたとき車輪が前に回るか
 *    以下の SIGN_* を実機に合わせて +1 / -1 にする。
 */
#include "Balancer2.h"
#include "bb2_control.h"

/* ソースを直さずに make GYRO=-1 WHEEL=-1 CUR=-1 のように切り替えられる */
#ifndef SIGN_GYRO
#define SIGN_GYRO   (+1.0f)   /* 前傾方向を正にする */
#endif
#ifndef SIGN_WHEEL
#define SIGN_WHEEL  (+1.0f)   /* 前進方向を正にする */
#endif
#ifndef SIGN_CUR
#define SIGN_CUR    (+1.0f)   /* 正の電流で前進する */
#endif

#define FALL_ANGLE  (0.785f)  /* 45 deg を超えたら転倒とみなしてモータを止める */

#define STAND_TICKS (15 * MAIN_CYCLE)   /* ゾーンA: 15秒 */

unsigned char calibrarion(void);        /* 純正サンプルと同じもの */
unsigned char isWhleeOverSpeed(void);
void ClearVariables(void);

static int fallen = 0;

static void user_control(void)
{
    static int tick = 0;
    bb2_meas_t m;
    bb2_out_t o;

    m.enc_l = (float)(SIGN_WHEEL * memmap.values.WHEEL_ANGLE_L);
    m.enc_r = (float)(SIGN_WHEEL * memmap.values.WHEEL_ANGLE_R);
    m.gyro  = (float)(SIGN_GYRO  * memmap.values.BODY_ANGULAR_SPD);

    if (tick == 0) {
        bb2_zero_position(m.enc_l, m.enc_r);
        bb2_set_mode(BB2_MODE_HOLD);
    }
    if (tick == STAND_TICKS) bb2_set_mode(BB2_MODE_RUN);
    tick++;

    o = bb2_step(m);
    if (o.th_hat > FALL_ANGLE || o.th_hat < -FALL_ANGLE) fallen = 1;
    if (fallen) { o.i_l_ma = 0.0f; o.i_r_ma = 0.0f; }

    memmap.values.T_CURRENT_L = (short)(SIGN_CUR * o.i_l_ma);
    memmap.values.T_CURRENT_R = (short)(SIGN_CUR * o.i_r_ma);

    /* バランサー2プログラマでグラフ表示・ログ取得するための出力 */
    memmap.values.USER_ARIA1 = o.th_hat;     /* 推定した本体傾き [rad] */
    memmap.values.USER_ARIA2 = o.taud_hat;   /* 推定した外乱トルク [Nm] = 坂 */
    memmap.values.USER_ARIA3 = o.bias_hat;   /* 推定したジャイロバイアス [rad/s] */
    memmap.values.USER_ARIA4 = o.th_eq;      /* 前置補償が指示する平衡姿勢 [rad] */
    memmap.values.USER_ARIA5 = o.v_hat;      /* 推定速度 [m/s] */
    memmap.values.USER_ARIA6 = o.v_cmd;      /* ガバナ後の速度指令 [m/s] */
    memmap.values.USER_ARIA7 = o.s;          /* 進行距離 [m] */
    memmap.values.USER_ARIA8 = o.yaw;        /* 方位 [rad] */
}

int main(void)
{
    Balancer2Init();
    UARTInit(CBR_115200, non, 1);
    CurrentControlDisable();
    setLED(3);

    while (getSW() == 0) CycleProc();       /* ボタン待ち */
    while (getSW() == 1) __NOP();
    setLED(2);
    Wait(1000);

    while (calibrarion() == 0) CycleProc(); /* ジャイロ静止校正 */
    ClearVariables();
    bb2_reset();

    while (getSW() == 0) CycleProc();       /* 機体を立ててからボタン */
    while (getSW() == 1) __NOP();
    setLED(2);
    ClearVariables();
    bb2_reset();
    CurrentControlEnable();

    while (1) {
        unsigned char slow = CycleProc();   /* 500Hz で同期。100Hz ごとに 1 */
        user_control();
        if (fallen) break;
        if (slow && isWhleeOverSpeed()) break;   /* 純正と同じ 100Hz で判定 */
        if (getSW() == 1) {
            while (getSW() == 1) __NOP();
            break;
        }
    }
    CurrentControlDisable();
    bb2_set_mode(BB2_MODE_IDLE);
    while (1) CycleProc();
}
