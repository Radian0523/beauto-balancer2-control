"""Beauto Balancer 2 の開発をコマンド 1 つにまとめたもの。リポジトリ直下の ./bb2 から呼ぶ。

  ./bb2 build [v5]           ファームをビルド (make)。make の引数も渡せる: ./bb2 build v5 VBW=30
  ./bb2 flash [v5]           ビルドして書き込む。書き込みモードの機体が繋がるまで待つ (sudo のパスワードを聞かれる)
  ./bb2 log                  走行ログを吸い出してグラフ (logs/)
  パラメータは今まで通り make の引数で変える: ./bb2 flash v5 VMAX=0.25 BOOST=0.3
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
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
    table = {"build": make, "flash": cmd_flash, "log": cmd_log}
    if cmd not in table:
        sys.exit(f"不明なコマンド: {cmd}\n{__doc__}")
    table[cmd](args)


if __name__ == "__main__":
    main()
