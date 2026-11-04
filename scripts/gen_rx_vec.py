#!/usr/bin/env python3
"""gen_rx_vec.py — T7 接收侧单元对拍向量生成（afdx_ref 金标准 → tb 输入文件）

产出（写入指定目录）：
  ic_vec.txt    IC 穷举：psn(256) × seen(2) × sn(256) = 131072 行
                每行 "<psn> <seen> <sn> <verdict>"，verdict: 0=drop 1=accept 2=accept_lost
  rm_vec.txt    RM 穷举：rsn(256) × sn(256) × 3 组配置 = 196608 行 + 初态 256 行
                每行 "<rsn> <sn> <elapsed> <skew> <verdict>"，verdict: 0=drop 1=accept
                组1 elapsed=0/skew=5000（窗内判决）；组2 elapsed=5001/skew=5000（超窗全收）
                组3 elapsed=0/skew=0（同 µs 到达，窗口判决）；末 256 行 valid=0 全 accept
  f<k>.hdr/.exp 解析向量：帧前 42B（hex 每行一字节）+ 期望
                "<ok_mac> <ok_etype> <ok_ip> <ok_udp> <vlid> <iptot> <ipid>
                 <mf> <off> <sport> <dport> <ulen>"（异常帧期望按 RTL 口径独立重算）
  manifest.txt  解析向量帧数清单
  asm_vec.txt   重组场景：SCEN 行 + "F <off8> <len> <mf> <udplen>" + len 行
                hex 字节 + "E done|abort|timeout"（done 后跟 total 行期望 hex）

用法: python3 gen_rx_vec.py <输出目录>
"""
import pathlib
import struct
import sys
import zlib

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from afdx_ref import (ic_step, rm_step, parse_frame, build_udp_ip, build_frame,
                      ETH_TYPE_IPV4, PROTO_UDP)

UID = 0x0A17
SRC_IP = "10.0.2.5"
SPORT, DPORT = 5001, 5002
IF_ID_A_BYTE = 0x20


def gen_ic(out):
    lines = []
    for psn in range(256):
        for seen in (0, 1):
            for sn in range(256):
                v, _, _ = ic_step(psn, seen, sn)
                verd = {"drop": 0, "accept": 1, "accept_lost": 2}[v]
                lines.append("%d %d %d %d\n" % (psn, seen, sn, verd))
    (out / "ic_vec.txt").write_text("".join(lines))
    return len(lines)


def gen_rm(out):
    lines = []
    for elapsed, skew in ((0, 5000), (5001, 5000), (0, 0)):
        for rsn in range(256):
            for sn in range(256):
                v, _, _ = rm_step(1, rsn, sn, elapsed, skew)
                verd = 1 if v == "accept" else 0
                lines.append("%d %d %d %d %d\n" % (rsn, sn, elapsed, skew, verd))
    for sn in range(256):                        # 初态 valid=0：全 accept
        v, _, _ = rm_step(0, 0, sn, 0, 5000)
        assert v == "accept"
        lines.append("%d %d %d %d %d\n" % (0, sn, 0, 5000, 1))
    (out / "rm_vec.txt").write_text("".join(lines))
    return len(lines)


def _frame_from_ip(ip, vlid, sn):
    return build_frame(ip, vlid, UID, IF_ID_A_BYTE, sn)


def gen_parser(out):
    """正常帧 + 各异常变体（etype/proto/csum/ver/ulen/dst MAC），覆盖 ok 标志组合。
    期望值按 RTL 检查口径独立重算（ok_mac/ok_etype/ok_ip/ok_udp + 提取字段）。"""
    def fix_fcs(f):
        f = bytearray(f)
        fcs = zlib.crc32(bytes(f[:-4])) & 0xFFFFFFFF
        f[-4:] = fcs.to_bytes(4, "little")
        return bytes(f)

    def manual_exp(f):
        ok_mac = f[0:4] == bytes([0x03, 0x00, 0x00, 0x00])
        ok_et = ((f[12] << 8) | f[13]) == ETH_TYPE_IPV4
        ipb = f[14:-5]                           # 去 MAC/etype 头与 SN/FCS 尾
        iptot = (ipb[2] << 8) | ipb[3]
        ver_ok = ipb[0] == 0x45
        proto_ok = ipb[9] == PROTO_UDP
        z = bytearray(ipb[:20]); z[10] = 0; z[11] = 0
        ssum = sum(struct.unpack("!10H", bytes(z)))
        while ssum >> 16:
            ssum = (ssum & 0xFFFF) + (ssum >> 16)
        csum_f = (ipb[10] << 8) | ipb[11]
        ok_ip = ver_ok and proto_ok and ((~ssum) & 0xFFFF) == csum_f
        ulen = (ipb[24] << 8) | ipb[25]          # UDP Length（ipb[20..27]=UDP 头）
        fo = (ipb[6] << 8) | ipb[7]
        off = fo & 0x1FFF
        mf = bool(fo & 0x2000)
        # 与 RTL 同口径：单帧 iptot==20+ulen；首片 ulen 填整条消息长（ISS-06）
        if off == 0:
            ok_udp = (ulen >= 9) if mf else (ulen >= 8 and iptot == 20 + ulen)
        else:
            ok_udp = 0                           # 中/尾片 34..41 为数据，调用方忽略
        return "%d %d %d %d %d %d %d %d %d %d %d %d" % (
            int(ok_mac), int(ok_et), int(ok_ip), int(ok_udp),
            (f[4] << 8) | f[5], iptot, (ipb[4] << 8) | ipb[5],
            int(mf), off,
            (ipb[20] << 8) | ipb[21], (ipb[22] << 8) | ipb[23], ulen)

    vlid = 0x002A
    dip = "224.224.%d.%d" % (vlid >> 8, vlid & 0xFF)
    msg = bytes(range(50))
    cases = []
    # 1 正常单帧 / 2 最小帧（1B+pad16）/ 3 首片（mf=1）/ 4 中片 / 5 尾片
    ip = build_udp_ip(msg, UID, dip, SRC_IP, SPORT, DPORT, 1)
    cases.append(_frame_from_ip(ip, vlid, 7))
    ip = build_udp_ip(b"\x55", UID, dip, SRC_IP, SPORT, DPORT, 2)
    cases.append(_frame_from_ip(ip, vlid, 8))
    ip = build_udp_ip(bytes(64), UID, dip, SRC_IP, SPORT, DPORT, 3,
                      0x2000, udp_len_field=8 + 3000)
    cases.append(_frame_from_ip(ip, vlid, 9))
    ip = build_udp_ip(bytes(128), UID, dip, SRC_IP, SPORT, DPORT, 3, 0x2000 | 182)
    cases.append(_frame_from_ip(ip, vlid, 10))
    ip = build_udp_ip(bytes(80), UID, dip, SRC_IP, SPORT, DPORT, 3, 365)
    cases.append(_frame_from_ip(ip, vlid, 11))
    # 6 etype 错（ARP）/ 7 IP csum 错（TOS 翻转）/ 8 proto 错（TCP）
    # 9 ver/ihl 错（0x46）/ 10 ulen 与 ip_total 不一致 / 11 dst MAC 常量错
    f = bytearray(cases[0]); f[12], f[13] = 0x08, 0x06
    cases.append(fix_fcs(f))
    f = bytearray(cases[0]); f[15] ^= 0x01
    cases.append(fix_fcs(f))
    f = bytearray(cases[0]); f[23] = 6
    cases.append(fix_fcs(f))
    f = bytearray(cases[0]); f[14] = 0x46
    cases.append(fix_fcs(f))
    f = bytearray(cases[0]); f[40] ^= 0x01
    cases.append(fix_fcs(f))
    f = bytearray(cases[0]); f[2] = 0x01
    cases.append(fix_fcs(f))
    # 12 不同 VLID 正常帧
    ip = build_udp_ip(msg, UID, "224.224.01.02", SRC_IP, SPORT, DPORT, 9)
    cases.append(_frame_from_ip(ip, 0x0102, 12))

    man = []
    for k, f in enumerate(cases):
        (out / ("f%d.hdr" % k)).write_text("".join("%02x\n" % b for b in f[:42]))
        (out / ("f%d.exp" % k)).write_text(manual_exp(f) + "\n")
        man.append("f%d" % k)
    (out / "manifest.txt").write_text("\n".join(man) + "\n")
    return len(cases)


def gen_asm(out):
    """重组场景：在序 2/3/6 片、乱序、缺片（abort）、重叠（abort）、缺首片（abort）。
    文件格式（tb 逐行 $fscanf）：SCEN <名> / F <off8> <len> <mf> <udplen> +
    len 行 hex 字节 / E done + len 行期望 hex 字节 | E abort | E timeout"""
    import random
    rnd = random.Random(7)
    lines = []
    ncase = 0

    def add(frags, expect, whole=b""):
        nonlocal ncase
        lines.append("SCEN c%d\n" % ncase)
        for pl, off8, mf, ipid, ul in frags:
            lines.append("F %d %d %d %d\n" % (off8, len(pl), int(mf), ul))
            lines.append("".join("%02x\n" % b for b in pl))
        if expect == "done":
            lines.append("E done\n")
            lines.append("".join("%02x\n" % b for b in whole))
        else:
            lines.append("E %s\n" % expect)
        ncase += 1

    def mkfrags(total, ipid=5):
        first = 1456 if total > 1471 else total
        chunks = [bytes(rnd.randrange(256) for _ in range(min(first, total)))]
        off = first
        while off < total:
            n = min(1464, total - off)
            chunks.append(bytes(rnd.randrange(256) for _ in range(n)))
            off += n
        out = []
        pos = 0
        for k, ch in enumerate(chunks):
            mf = k < len(chunks) - 1
            off8 = pos // 8
            ul = (8 + total) if off8 == 0 else 0
            out.append((ch, off8, mf, ipid, ul))
            pos += len(ch)
        return out, b"".join(chunks)

    for total in (2000, 3000, 8192):                 # 在序 2/3/6 片（TC-FRG-01/02/03）
        frags, whole = mkfrags(total)
        add(frags, "done", whole)
    frags, whole = mkfrags(3000)                     # 乱序（TC-FRG-04）
    add([frags[1], frags[2], frags[0]], "done", whole)
    frags, _ = mkfrags(3000)                         # 缺中片 → 定时器回收（TC-FRG-05）
    add([frags[0], frags[2]], "timeout")
    frags, _ = mkfrags(3000)                         # 缺尾片 → 定时器回收
    add([frags[0], frags[1]], "timeout")
    frags, _ = mkfrags(3000)                         # 重叠（片1 重发）→ abort
    add([frags[0], frags[1], frags[0]], "abort")
    frags, _ = mkfrags(3000)                         # 缺首片 → 定时器回收
    add([frags[1], frags[2]], "timeout")
    # Luna 原例（VERDICT-010 §1-02）：total=24（udplen=32），片序 0/3/2——
    # off8=3 尾片终点 32 越 total、off8=2 中片露 8B 空洞 → abort 且零交付
    add([(b"\x11" * 8, 0, True, 5, 32),
         (b"\x22" * 8, 3, False, 5, 0),
         (b"\x33" * 8, 2, True, 5, 0)], "abort")
    # 越界回溯（FIX3-3）：越界尾片先到（total 未知）→ 首片到达回溯 maxend>total → abort
    add([(b"\x44" * 8, 3, False, 5, 0),
         (b"\x55" * 8, 0, True, 5, 32)], "abort")
    # 尾片终点不符（FIX3-3）：total=24 但尾片终点 16 ≠ 24 → abort
    add([(b"\x66" * 8, 0, True, 5, 32),
         (b"\x77" * 8, 1, False, 5, 0)], "abort")
    (out / "asm_vec.txt").write_text("".join(lines))
    return ncase


def main():
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "sim/tb_out/rxvec")
    out.mkdir(parents=True, exist_ok=True)
    n_ic = gen_ic(out)
    n_rm = gen_rm(out)
    n_p = gen_parser(out)
    n_a = gen_asm(out)
    print("gen_rx_vec: ic=%d rm=%d parser=%d asm=%d -> %s"
          % (n_ic, n_rm, n_p, n_a, out))


if __name__ == "__main__":
    main()
