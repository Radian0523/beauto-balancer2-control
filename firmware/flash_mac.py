"""Mac で CRP DISABLD ドライブがマウントされないときの書き込みツール。

LPC1343 の ROM ブートローダ(USB ISP)は USB マスストレージとして振る舞い、
ディスク上の firmware.bin のデータ領域に書いた内容がそのままフラッシュに書かれる。
macOS のドライバがこのディスクを開けないので、libusb で直接 SCSI コマンドを送り、
firmware.bin のデータ領域(FAT を読んで場所を求める)へ書き込む。
Linux でよく使われる `dd ... of=firmware.bin conv=notrunc` と同じことをしている。

使い方 (機体はボタンを押しながら USB を差して書き込みモードにしておく):
    sudo .venv/bin/python firmware/flash_mac.py                        # 読むだけ(確認)
    sudo .venv/bin/python firmware/flash_mac.py firmware/build/bb2.bin  # 書き込み
sudo が要るのは macOS のドライバから USB を奪うため。
"""
import struct
import sys
import time

import usb.backend.libusb1
import usb.core
import usb.util

VID, PID = 0x04CC, 0x0003          # "NXP LPC13XX IFLASH"
SECTOR = 512

backend = usb.backend.libusb1.get_backend(
    find_library=lambda _: "/opt/homebrew/lib/libusb-1.0.dylib")


class Bot:
    """USB Mass Storage Bulk-Only Transport の最小実装。"""

    def __init__(self):
        self.dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
        if self.dev is None:
            sys.exit("書き込みモードの機体が見つからない。ボタンを押しながら USB を差し直す。")
        try:
            if self.dev.is_kernel_driver_active(0):
                self.dev.detach_kernel_driver(0)
        except usb.core.USBError as e:
            sys.exit(f"macOS のドライバから切り離せない ({e})。sudo を付けて実行する。")
        try:
            self.dev.set_configuration()
        except usb.core.USBError:
            pass                                     # 既に構成済みなら無視
        usb.util.claim_interface(self.dev, 0)
        intf = self.dev.get_active_configuration()[(0, 0)]
        is_in = lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        self.ep_in = usb.util.find_descriptor(intf, custom_match=is_in)
        self.ep_out = usb.util.find_descriptor(intf, custom_match=lambda e: not is_in(e))
        self.tag = 1
        self.reset_recovery()

    def reset_recovery(self):
        """macOS のドライバが初期化に失敗したあとは端点が STALL したまま残るので、
        Bulk-Only Mass Storage Reset と Clear Feature(HALT) で初期状態に戻す。"""
        try:
            self.dev.ctrl_transfer(0x21, 0xFF, 0, 0, None, timeout=2000)
        except usb.core.USBError:
            pass
        for ep in (self.ep_in, self.ep_out):
            try:
                self.dev.clear_halt(ep.bEndpointAddress)
            except usb.core.USBError:
                pass

    def cmd(self, cb, data_len=0, data_out=None, retry=2):
        try:
            return self._cmd(cb, data_len, data_out)
        except usb.core.USBError:
            if retry == 0:
                raise
            self.reset_recovery()
            return self.cmd(cb, data_len, data_out, retry - 1)

    def _cmd(self, cb, data_len=0, data_out=None):
        flags = 0x80 if data_out is None else 0x00
        n = len(data_out) if data_out is not None else data_len
        cbw = struct.pack("<4sIIBBB", b"USBC", self.tag, n, flags, 0, len(cb)) + cb.ljust(16, b"\0")
        self.ep_out.write(cbw, timeout=5000)
        data = b""
        if data_out is not None:
            self.ep_out.write(data_out, timeout=10000)
        elif data_len:
            try:
                data = bytes(self.ep_in.read(data_len, timeout=5000))
            except usb.core.USBError as e:
                if e.errno != 32:
                    raise
                self.dev.clear_halt(self.ep_in.bEndpointAddress)   # データ無しで STALL
        
        csw = bytes(self.ep_in.read(13, timeout=10000))
        sig, tag, residue, status = struct.unpack("<4sIIB", csw)
        if sig != b"USBS" or tag != self.tag:
            raise RuntimeError(f"CSW 異常: {csw.hex()}")
        self.tag += 1
        if status != 0:
            raise RuntimeError(f"SCSI コマンド {cb[0]:#04x} が失敗 (status={status})")
        return data

    def read(self, lba, count=1):
        cb = struct.pack(">BBIBHB", 0x28, 0, lba, 0, count, 0)
        return self.cmd(cb, count * SECTOR)

    def write(self, lba, data):
        cb = struct.pack(">BBIBHB", 0x2A, 0, lba, 0, len(data) // SECTOR, 0)
        self.cmd(cb, data_out=data)


def locate_firmware(bot):
    """FAT のブートセクタとルートディレクトリから firmware.bin の位置を求める。"""
    bs = bot.read(0)
    bps, spc, reserved, nfats, nroot, _, _, spf = struct.unpack_from("<HBHBHHBH", bs, 11)
    assert bps == SECTOR, f"セクタ長 {bps} は想定外"
    root_lba = reserved + nfats * spf
    root_secs = (nroot * 32 + SECTOR - 1) // SECTOR
    data_lba = root_lba + root_secs
    root = bot.read(root_lba, root_secs)
    for i in range(0, len(root), 32):
        e = root[i:i + 32]
        if e[0] in (0, 0xE5) or e[11] & 0x08:      # 空き・削除済み・ボリューム名
            continue
        name = (e[0:8].decode(errors="replace").strip() + "." + e[8:11].decode(errors="replace").strip())
        if name.upper() == "FIRMWARE.BIN":
            cluster = struct.unpack_from("<H", e, 26)[0]
            size = struct.unpack_from("<I", e, 28)[0]
            lba = data_lba + (cluster - 2) * spc
            return lba, size
    raise RuntimeError("ディスク上に firmware.bin が見つからない")


def main():
    bot = Bot()
    inq = bot.cmd(bytes([0x12, 0, 0, 0, 36, 0]), 36)
    print("デバイス:", inq[8:36].decode(errors="replace").strip())
    for _ in range(10):                              # 準備できるまで待つ
        try:
            bot.cmd(bytes(6))
            break
        except RuntimeError:
            time.sleep(0.3)
    lba, size = locate_firmware(bot)
    print(f"firmware.bin: LBA {lba}, {size} バイト (= フラッシュ容量)")

    if len(sys.argv) < 2:
        cur = bot.read(lba, 1)
        print("先頭 32 バイト:", cur[:32].hex())
        print("確認のみで終了。書き込むには .bin を引数に付ける。")
        return

    img = open(sys.argv[1], "rb").read()
    if len(img) > size:
        sys.exit(f"{sys.argv[1]} ({len(img)} B) がフラッシュ ({size} B) より大きい")
    w = struct.unpack_from("<8I", img)
    if sum(w) & 0xFFFFFFFF:
        sys.exit("ベクタ表のチェックサムが 0 でない。make で作った .bin を使う。")
    img += b"\xff" * (-len(img) % SECTOR)
    n = len(img) // SECTOR
    for i in range(n):                               # 先頭から順に 1 セクタずつ
        bot.write(lba + i, img[i * SECTOR:(i + 1) * SECTOR])
        print(f"\r書き込み {i + 1}/{n}", end="", flush=True)
    print()

    back = bot.read(lba, n) if n <= 64 else b"".join(bot.read(lba + i) for i in range(n))
    if back[:len(img)] == img:
        print("照合 OK。USB を抜いて電源を入れ直すと新しいファームで起動する。")
    else:
        bad = next(i for i in range(len(img)) if back[i] != img[i])
        print(f"照合 NG (オフセット {bad:#x} から不一致)。もう一度書き込みモードに入れて再実行する。")


if __name__ == "__main__":
    main()
