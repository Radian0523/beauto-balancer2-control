# 走行ログ

2026-09-27〜29 に実機で取ったログです。ファイル名の日時が走らせた時刻です。

| ファイル | 中身 |
|---|---|
| `run_*.csv` / `run_*.png` | 走行ログ。止めたあと `./bb2 log` で機体の RAM から吸い出したもの。PNG は自動で作ったグラフ |
| `live_*.csv` | `./bb2 live` で USB をつないだまま取り続けたログ (時間の制限なし) |
| `diag_*.csv` / `diag_*.png` | 診断用ログ。ジャイロの生の値と、傾きの推定の中身を記録したもの |
| `battery_history.csv` | `./bb2 battery` で車輪を浮かせて空回しした最高の速さ。電池の減り具合の目安 |

## 列の意味 (run / live)

時刻 `t_s` は、run では止めた瞬間を 0、live では記録を始めた瞬間を 0 とした秒数です。古いログほど列が少ないです。

| 列 | 意味 |
|---|---|
| `body_angle_rad` | 本体の傾き (純正の推定) |
| `theta_ref_rad` | 目標の傾き (釣り合う角度 + 前傾フィードフォワード) |
| `body_rate_rad_s` | 傾く速さ |
| `wheel_spd_rad_s` / `target_spd_rad_s` | 車輪の速さ (オブザーバの推定) と目標の速さ |
| `current_cmd` | 電流指令 (左右の平均) |
| `wheel_err_rad` / `wheel_err_int_rad_s` | 目標の車輪角とのずれと、その積分 |
| `m_current` | 実際に流れた電流 (左右の平均) |
| `yaw_err_rad` / `turn_cmd_rad_s` | 向きのずれと旋回の指令 |
| `gyro_trim_lsb` | ジャイロ 0 点の補正量 |
| `balance_angle_rad` | 推定した釣り合う角度 |

ログを比べるときは `./bb2 stats logs/run_A.csv logs/run_B.csv` を使います。
うまくいった回のログは [snapshots/](../snapshots/) にも設定と一緒に置いています。
