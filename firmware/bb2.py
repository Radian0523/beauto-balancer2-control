"""Beauto Balancer 2 の開発をコマンド 1 つにまとめたもの。リポジトリ直下の ./bb2 から呼ぶ。

  ./bb2 build [v5]           ファームをビルド (make)。make の引数も渡せる: ./bb2 build v5 VBW=30
  ./bb2 flash [v5]           ビルドして書き込む。書き込みモードの機体が繋がるまで待つ (sudo のパスワードを聞かれる)
  ./bb2 log                  走行ログを吸い出してグラフ (logs/)
  ./bb2 get [名前...]        パラメータ・memmap の値を表示 (名前なしで調整用パラメータ一覧)
  ./bb2 set 名前=値 ...      値を書き換える (次にボタンで走らせたときから効く)
  ./bb2 save                 今の値を機体のフラッシュに保存 (電源を切っても残る)
  ./bb2 watch [名前...]      値を 0.2 秒ごとに表示 (USB を繋いだまま。Ctrl+C で終了)

パラメータの名前は別名でも書ける: ff ti vmax acc boost bacc emax jerk
  例) ./bb2 set vmax=0.25 boost=0.3        ./bb2 set GAIN_BODY=1.1
"""
import os
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
VID, PID = 0x1962, 0x2080

# memmap のレイアウト (Balancer2.h の struct MAP、ARM EABI の整列で並べたもの)
FIELDS = [
    ("Product_ID", "H"), ("Version", "B"), ("MODE", "B"),
    ("M_CURRENT_L", "h"), ("M_CURRENT_R", "h"), ("T_CURRENT_L", "h"), ("T_CURRENT_R", "h"),
    ("CURRENT_OFFSET_L", "h"), ("CURRENT_OFFSET_R", "h"),
    ("BODY_ANGULAR_SPD", "d"), ("BODY_ANGLE", "d"), ("BODY_ANGULAR_SPD_OFFSET", "d"),
    ("WHEEL_ANGULAR_SPD_L", "d"), ("WHEEL_ANGULAR_SPD_R", "d"),
    ("WHEEL_ANGLE_L", "d"), ("WHEEL_ANGLE_R", "d"), ("T_SPD_L", "d"), ("T_SPD_R", "d"),
    ("ENC_L", "q"), ("ENC_R", "q"), ("GYRO_DATA", "h"),
    ("ADC_C_LA", "H"), ("ADC_C_LB", "H"), ("ADC_C_RA", "H"), ("ADC_C_RB", "H"),
    ("PAD_BTN", "H"), ("PAD_AN_RX", "b"), ("PAD_AN_RY", "b"), ("PAD_AN_LX", "b"), ("PAD_AN_LY", "b"),
    ("USER_ARIA1", "d"), ("USER_ARIA2", "d"), ("USER_ARIA3", "d"), ("USER_ARIA4", "d"),
    ("USER_ARIA5", "d"), ("USER_ARIA6", "d"), ("USER_ARIA7", "d"), ("USER_ARIA8", "d"),
    ("CALIBRATISON_CNT", "H"), ("CALIBRATISON_TIME", "B"), ("LOG_MODE", "B"),
    ("LOG_DIV", "H"), ("LOG_CNT", "H"),
    ("LOG_FLG_1", "Q"), ("LOG_FLG_2", "Q"), ("LOG_FLG_3", "Q"),
    ("GAIN_BODY", "d"), ("GAIN_WHEEL", "d"), ("GAIN_WHEEL_CORRELATION", "d"),
    ("GAIN_BODY_ANGLE", "d"), ("GAIN_BODY_ANGULAR_SPD", "d"),
    ("GAIN_WHEEL_ANGLE", "d"), ("GAIN_WHEEL_ANGULAR_SPD", "d"),
    ("GAIN_WHEEL_CORRELATION_ANGLE", "d"), ("GAIN_WHEEL_CORRELATION_ANGULAR_SPD", "d"),
    ("GAIN_CURRENT_P", "d"), ("GAIN_CURRENT_I", "d"), ("GAIN_GYRO_HPF_COF", "d"),
    ("GAIN_OPTION1", "d"), ("GAIN_OPTION2", "d"), ("GAIN_OPTION3", "d"),
    ("GAIN_OPTION4", "d"), ("GAIN_OPTION5", "d"), ("GAIN_OPTION6", "d"),
]
LAYOUT = {}
_off = 0
for _n, _t in FIELDS:
    _sz = struct.calcsize(_t)
    _off = (_off + _sz - 1) // _sz * _sz
    LAYOUT[_n] = (_off, _t)
    _off += _sz
# 番地は arm-none-eabi-gcc の offsetof と照合済み。構造体は 360 バイトだが、
# 純正ライブラリの USB 読み書きとフラッシュ保存は先頭 352 バイトまで (GAIN_OPTION6 は範囲外)
MEMMAP_USB = 352
assert LAYOUT["CALIBRATISON_TIME"][0] == 186 and LAYOUT["GAIN_BODY"][0] == 216 and _off == 360
LAYOUT = {n: v for n, v in LAYOUT.items() if v[0] + struct.calcsize(v[1]) <= MEMMAP_USB}

# v5 の調整用パラメータ (main_v3.c の params_init と対応)
ALIAS = {
    "ff": ("GAIN_OPTION1", "前傾フィードフォワードの係数 (物理的には 1)"),
    "ti": ("GAIN_OPTION2", "積分の時定数 [s] (大きいほど弱い、0 で無効)"),
    "vmax": ("GAIN_OPTION3", "スティック最大の速度 [m/s]"),
    "acc": ("GAIN_OPTION4", "加速度の上限 [m/s^2]"),
    "boost": ("GAIN_OPTION5", "ダッシュの速度 [m/s] (0 で無効)"),
    "bacc": ("USER_ARIA1", "ダッシュの加速度 [m/s^2]"),
    "emax": ("USER_ARIA2", "走行中の目標とのずれの上限 [rad] (1 rad = 2.1 cm)"),
    "jerk": ("USER_ARIA3", "加速度の変化率の上限 [m/s^3]"),
    "body": ("GAIN_BODY", "純正: 本体角フィードバックの倍率"),
    "wheel": ("GAIN_WHEEL", "純正: 車輪フィードバックの倍率"),
}


def resolve(name):
    if name.lower() in ALIAS:
        return ALIAS[name.lower()][0]
    if name.upper() in LAYOUT:
        return name.upper()
    sys.exit(f"不明な名前: {name}\n使える別名: {', '.join(ALIAS)}  / memmap の名前: {', '.join(LAYOUT)}")


class Robot:
    def __init__(self, wait=True):
        import hid
        self.dev = None
        t0 = time.time()
        told = False
        while self.dev is None:
            try:
                d = hid.device()
                d.open(VID, PID)
                self.dev = d
            except OSError:
                if not wait or time.time() - t0 > 30:
                    sys.exit("機体が見つからない。電源を入れて USB を挿す (ボタンは押さずに = 通常起動)。")
                if not told:
                    print("機体の接続を待っています (30 秒)...")
                    told = True
                time.sleep(0.5)

    def _xfer(self, payload, tag):
        out = bytearray(65)
        out[1:1 + len(payload)] = payload
        self.dev.write(out)
        for _ in range(10):
            r = self.dev.read(64, 500)
            if r and r[0] == tag:
                return bytes(r)
        raise RuntimeError("機体から応答がない")

    def read(self, addr, n):
        r = self._xfer(bytes([ord("R"), addr & 0xFF, addr >> 8, n]), ord("R"))
        return r[2:2 + r[1]]

    def read_block(self, addr, n):
        buf = b""
        while len(buf) < n:
            buf += self.read(addr + len(buf), min(60, n - len(buf)))
        return buf

    def write(self, addr, data):
        self._xfer(bytes([ord("W"), addr & 0xFF, addr >> 8, len(data)]) + data, ord("W"))

    def get(self, name):
        off, t = LAYOUT[name]
        return struct.unpack("<" + t, self.read(off, struct.calcsize(t)))[0]

    def set(self, name, value):
        off, t = LAYOUT[name]
        v = float(value) if t in "df" else int(float(value))
        self.write(off, struct.pack("<" + t, v))

    def save(self):
        r = self._xfer(b"FM", ord("f"))
        return r[:3] == b"fme"


def fmt(name, v):
    return f"{v:.6g}" if isinstance(v, float) else str(v)


def cmd_get(args):
    rb = Robot()
    if not args:
        print(f"{'名前':6s} {'memmap':14s} {'値':>10s}  説明")
        for a, (n, desc) in ALIAS.items():
            print(f"{a:6s} {n:14s} {fmt(n, rb.get(n)):>10s}  {desc}")
        return
    for a in args:
        n = resolve(a)
        print(f"{n} = {fmt(n, rb.get(n))}")


def cmd_set(args):
    if not args:
        sys.exit("例: ./bb2 set vmax=0.25 boost=0.3")
    rb = Robot()
    for kv in args:
        if "=" not in kv:
            sys.exit(f"名前=値 の形で書く: {kv}")
        k, v = kv.split("=", 1)
        n = resolve(k)
        before = rb.get(n)
        rb.set(n, v)
        print(f"{n}: {fmt(n, before)} -> {fmt(n, rb.get(n))}")
    print("次にボタンで走らせたときから効く。電源を切っても残すなら ./bb2 save")


def cmd_save(_):
    rb = Robot()
    print("保存しました" if rb.save() else "保存の応答が想定と違う (値は RAM には入っている)")


def cmd_watch(args):
    names = [resolve(a) for a in args] or ["BODY_ANGLE", "BODY_ANGULAR_SPD", "WHEEL_ANGLE_L", "T_CURRENT_L", "PAD_BTN"]
    rb = Robot()
    try:
        while True:
            print("  ".join(f"{n}={fmt(n, rb.get(n))}" for n in names))
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass


def make(args):
    target = args[0] if args and "=" not in args[0] else "v5"
    extra = [a for a in args if "=" in a]
    r = subprocess.run(["make", "-C", HERE, target] + extra, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(r.stdout[-3000:] + r.stderr[-3000:])
    size = [l for l in r.stdout.splitlines() if "v3.axf" in l or l.strip().startswith("text")]
    print("\n".join(size))
    path = os.path.join(HERE, "build", f"{target}.bin")
    print(f"ビルド OK: {os.path.relpath(path, ROOT)}")
    return path


def cmd_flash(args):
    if args and args[0].endswith(".bin"):
        path = args[0]
    else:
        path = make(args)
    print("書き込みモードにする: ボタンを押しながら USB を挿して、挿したら離す")
    py = os.path.join(ROOT, ".venv", "bin", "python")
    sys.exit(subprocess.call(["sudo", py, os.path.join(HERE, "flash_mac.py"), "--wait", path]))


def cmd_log(_):
    sys.exit(subprocess.call([os.path.join(ROOT, ".venv", "bin", "python"), os.path.join(HERE, "read_log.py")], cwd=ROOT))


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help", "help"):
        print(__doc__)
        return
    cmd, args = sys.argv[1], sys.argv[2:]
    table = {"build": make, "flash": cmd_flash, "log": cmd_log, "get": cmd_get,
             "set": cmd_set, "save": cmd_save, "watch": cmd_watch}
    if cmd not in table:
        sys.exit(f"不明なコマンド: {cmd}\n{__doc__}")
    table[cmd](args)


if __name__ == "__main__":
    main()
