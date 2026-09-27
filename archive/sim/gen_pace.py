"""コース定義 (mission.py) から firmware/pace.h を自動生成する。
当日は mission.py の寸法を実測値に直して python3 src/gen_pace.py を叩くだけ。"""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mission as MIS

def speed_of(x, v_nom=0.35):
    z = MIS.zone_of(x)
    return dict(B_straight=v_nom, C_scurve=v_nom * 0.85, D_slope=v_nom * 0.9,
                E_steep=v_nom * 0.9, E_seesaw=v_nom * 0.65).get(z, 0.0)

def main(step=0.10, v_nom=0.35, path=None):
    path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                                "kf_firmware", "pace.h")
    xs = np.arange(0.0, MIS.GOAL + 0.6, step)
    rows = [(x, MIS.path_y(x), speed_of(x, v_nom)) for x in xs]
    with open(path, "w") as f:
        f.write("/* 自動生成 (src/gen_pace.py) — コース中心線のウェイポイント */\n")
        f.write("#ifndef PACE_H\n#define PACE_H\n\n")
        f.write("typedef struct { float x, y, v; } bb2_wp_t;\n\n")
        f.write("static const bb2_wp_t PATH[] = {\n")
        for x, y, v in rows:
            f.write("    {%7.4ff, %7.4ff, %5.3ff},\n" % (x, y, v))
        f.write("};\n#define N_PATH ((int)(sizeof(PATH)/sizeof(PATH[0])))\n\n#endif\n")
    print(path, "に", len(rows), "点を書き出しました")

if __name__ == "__main__":
    main(v_nom=float(sys.argv[1]) if len(sys.argv) > 1 else 0.35)
