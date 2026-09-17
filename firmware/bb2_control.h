/*
 * ビュートバランサー2 用 現代制御コントローラ (ハード非依存の中核)
 *
 *   カルマンフィルタ(6状態: 角度/角速度/車輪/ジャイロバイアス/外乱トルク)
 *   + LQI速度サーボ + 外乱前置補償 + 参照ガバナ + 制御配分
 *
 * PC 上でも同じソースをコンパイルしてシミュレータで検証できるように、
 * ハードウェア依存 (memmap, CycleProc など) は main_bb2.c 側に隔離してある。
 */
#ifndef BB2_CONTROL_H
#define BB2_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float enc_l;   /* 左車輪角 [rad] (本体基準・エンコーダそのまま) */
    float enc_r;   /* 右車輪角 [rad] */
    float gyro;    /* 本体角速度 [rad/s] (キャリブレーション済み) */
} bb2_meas_t;

typedef struct {
    float i_l_ma;  /* 左モータ電流指令 [mA] */
    float i_r_ma;  /* 右モータ電流指令 [mA] */
    /* 以下はログ/デバッグ用 */
    float th_hat, dth_hat, v_hat, taud_hat, bias_hat;
    float th_eq, v_cmd, s, yaw, mode_f;
} bb2_out_t;

typedef enum {
    BB2_MODE_IDLE = 0,   /* 出力 0 */
    BB2_MODE_HOLD = 1,   /* その場で直立 (ゾーンA) */
    BB2_MODE_RUN  = 2    /* ペースノートに従って走行 */
} bb2_mode_t;

void bb2_reset(void);
void bb2_set_mode(bb2_mode_t m);
bb2_mode_t bb2_get_mode(void);
/* 500Hz (MAIN_CYCLE) で呼ぶ */
bb2_out_t bb2_step(bb2_meas_t meas);
void bb2_zero_position(float enc_l, float enc_r);

/* --- 現地で触る調整つまみ ------------------------------------------------- */
extern float BB2_V_NOM;       /* 基準速度 [m/s] */
extern float BB2_K_FF;        /* 外乱前置補償の強さ 0..1 (ゲイン誤差が大きいなら下げる) */
extern float BB2_TH_EQ_MAX;   /* 平衡姿勢の上限 [rad] */
extern float BB2_TH_GUARD;    /* 転倒復帰に入るしきい値 [rad] */
extern int   BB2_USE_GOV;     /* 参照ガバナ 1=有効 */
extern float BB2_K_STANLEY;   /* パス追従(横ずれ復帰)の強さ */

#ifdef __cplusplus
}
#endif
#endif
