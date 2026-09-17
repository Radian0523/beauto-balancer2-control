# src/ の使い方

| ファイル | 役割 |
|---|---|
| `params.py` | 物理パラメータ（`!!` は要同定）と線形化モデル |
| `model.py` | 非線形プラント（坂・摩擦・逆起電力・トラクション限界・ヨー） |
| `sensors.py` | ジャイロ（量子化1.2deg/s＋雑音＋ドリフト）とエンコーダのモデル |
| `design.py` | LQR / LQI / 定常カルマン / ヨー LQR の設計、`gains.h` 生成 |
| `estimators.py` | `StockHPF`（純正相当）と `KalmanIP`（6状態） |
| `controllers.py` | `ModernCtl`（LQI＋前置補償＋ガバナ＋復帰） |
| `mission.py` | 本番コース（A〜E）の勾配・中心線・ゾーン定義 |
| `sim.py` | 500Hz のシミュレーションループ、制御配分 `allocate()` |
| `gen_pace.py` | コース定義 → `firmware/pace.h`（ウェイポイント表）生成 |
| `sysid.py` | 当日の同定（振り子落とし／車輪空転）と再設計 |
| `run_ablation.py` / `run_experiments.py` / `run_c_check.py` / `tune_kf.py` | 実験スクリプト |

当日の再設計は次の 4 コマンドだけ:

```bash
# 1) params.py を同定結果で書き換えてから
python3 -c "import sys;sys.path.insert(0,'src');from params import Params;from design import emit_gains_h;emit_gains_h(Params(),'firmware/gains.h')"
python3 src/gen_pace.py 0.35     # 目標速度を引数で
python3 src/run_c_check.py       # 焼く前に必ず通す
```
