#!/usr/bin/env python3
"""check_output.py — DUT 发送输出全帧判定（ISS-09 硬性验收项）

判定内容（不依赖单一来源，FCS/IP 校验和独立重算）：
  1. 帧数与金标准一致（A/B 两网）。
  2. 全帧逐字节比对：按 VL 分组后与 golden 同 VL 同序号帧逐一比对
     （golden 按 VLID 分组，跨 VL 的线上交错次序不参与比对）。
  3. FCS 独立核算：对每个 DUT 帧重算 zlib.crc32(帧体) 并与尾部 FCS 比对。
  4. IP 头校验和独立核算：对每个 DUT 帧重算反码和并与字段比对。
  5. 结构检查：帧长 ∈ [64,1518]、Padding 区全 0、SN 序列（每 VL 从 0 起、
     逐帧 +1、255→1 回绕）、A/B 同帧序号 SN 一致。
  6. 时序检查（frame_times.txt）：同 VL 相邻帧起始间隔 ≥ BAG；
     分片几何独立核对（8B 偏移/MF/IP ID/末片长度，FIX2 增补）；
     A/B 同帧起始差 ≤ 0.5ms（P7-24 冗余副本时限）。

用法: python3 check_output.py <case目录>   # exit 0 = PASS
"""
import json
import pathlib
import struct
import sys
import zlib

CYCLES_PER_US = 125          # 125 MHz


def load_hex(p):
    return bytes(int(x, 16) for x in p.read_text().split())


def load_lens(p):
    return [int(x) for x in p.read_text().split()]


def split_frames(data, lens):
    out, off = [], 0
    for n in lens:
        out.append(data[off:off + n])
        off += n
    assert off == len(data), "byte count mismatch in frame file"
    return out


def ip_csum(hdr):
    s = sum(struct.unpack("!10H", hdr))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def frame_vlid(f):
    return (f[4] << 8) | f[5]


def frame_sn(f):
    return f[-5]          # 帧体末 4B 是 FCS，其前 1B 为 SN


def check_frame_struct(f, tag, errs):
    if not (64 <= len(f) <= 1518):
        errs.append("%s: 帧长 %d 越界 [64,1518]" % (tag, len(f)))
        return
    body, fcs = f[:-4], struct.unpack("<I", f[-4:])[0]
    # FCS 独立核算
    if zlib.crc32(body) & 0xFFFFFFFF != fcs:
        errs.append("%s: FCS 不符（算 %08X 存 %08X）"
                    % (tag, zlib.crc32(body) & 0xFFFFFFFF, fcs))
    # IP 头校验和独立核算
    ip = body[14:]
    if ip[0] != 0x45:
        errs.append("%s: IP 版本/IHL 非 0x45" % tag)
        return
    ip_total = (ip[2] << 8) | ip[3]
    if ip_csum(ip[:10] + b"\x00\x00" + ip[12:20]) != (ip[10] << 8) | ip[11]:
        errs.append("%s: IP 头校验和不符" % tag)
    # Padding 区（IP 包尾到 SN 之前）必须全 0
    pad_lo, pad_hi = 14 + ip_total, len(body) - 1
    if pad_hi > pad_lo and any(b != 0 for b in body[pad_lo:pad_hi]):
        errs.append("%s: Padding 区非零（偏移 %d..%d）" % (tag, pad_lo, pad_hi))


def main():
    d = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "sim/tb_out/tx1")
    case = json.loads((d / "case.json").read_text())
    errs = []

    gold = {}
    dut = {}
    times = {}
    for net in ("A", "B"):
        g = split_frames(load_hex(d / ("golden_%s.hex" % net)),
                         load_lens(d / ("frame_%s.lens" % net)))
        o = split_frames(load_hex(d / ("frame_out_%s.hex" % net)),
                         load_lens(d / ("frame_out_%s.lens" % net)))
        # 按 VL 分组（保持各自次序）
        gold[net] = {}
        for f in g:
            gold[net].setdefault(frame_vlid(f), []).append(f)
        dut[net] = {}
        dut_times = {}
        for f in o:
            dut[net].setdefault(frame_vlid(f), []).append(f)
        # times 文件：每行 "start_cycle"，与 frame_out 同序
        cyc = [int(x) for x in (d / ("frame_times_%s.txt" % net)).read_text().split()]
        if len(cyc) != len(o):
            errs.append("网%s: times 行数 %d ≠ 帧数 %d" % (net, len(cyc), len(o)))
        idx = 0
        for f in o:
            dut_times.setdefault(frame_vlid(f), []).append(cyc[idx])
            idx += 1
        times[net] = dut_times

    # 1/2. 帧数与逐字节比对（按 VL）
    n_cmp = 0
    for net in ("A", "B"):
        for vl in sorted(gold[net]):
            gv, dv = gold[net][vl], dut[net].get(vl, [])
            if len(gv) != len(dv):
                errs.append("网%s VL%04X: 帧数 %d ≠ 期望 %d"
                            % (net, vl, len(dv), len(gv)))
                continue
            for i, (a, b) in enumerate(zip(gv, dv)):
                if a != b:
                    diff = next(k for k in range(min(len(a), len(b)))
                                if a[k] != b[k]) if len(a) == len(b) else -1
                    errs.append("网%s VL%04X 帧%d: 第 %s 字节不符"
                                % (net, vl, i, ("偏移 %d" % diff) if diff >= 0
                                   else "长度 %d≠%d" % (len(b), len(a))))
                n_cmp += 1
        extra = set(dut[net]) - set(gold[net])
        for vl in extra:
            errs.append("网%s: 多出未配置 VL%04X 的 %d 帧"
                        % (net, vl, len(dut[net][vl])))

    # 3~5. DUT 帧独立核算（FCS / IP csum / 结构 / SN 序列）
    for net in ("A", "B"):
        for vl, fs in sorted(dut[net].items()):
            for i, f in enumerate(fs):
                check_frame_struct(f, "网%s VL%04X 帧%d" % (net, vl, i), errs)
            sns = [frame_sn(f) for f in fs]
            exp, sn = [], 0
            for _ in sns:
                exp.append(sn)
                sn = 1 if sn >= 255 else sn + 1
            if sns != exp:
                errs.append("网%s VL%04X: SN 序列 %s ≠ 期望 %s"
                            % (net, vl, sns, exp))
            # 与 golden 的 SN 也应一致（逐字节比对已覆盖，此处冗余明示）
            if vl in gold[net]:
                gsns = [frame_sn(f) for f in gold[net][vl]]
                if len(gsns) == len(sns) and gsns != sns:
                    errs.append("网%s VL%04X: SN 与 golden 不一致" % (net, vl))

    # A/B 同帧 SN 一致
    for vl in sorted(set(dut["A"]) & set(dut["B"])):
        sa = [frame_sn(f) for f in dut["A"][vl]]
        sb = [frame_sn(f) for f in dut["B"][vl]]
        if sa != sb:
            errs.append("VL%04X: A/B 网 SN 序列不一致" % vl)

    # 7. 分片几何独立核对（FIX2/REV-002-08：8B 对齐偏移、MF、IP ID 一致、末片长度。
    #    偏移口径与 afdx_ref 冻结一致：off8 = (1456 + (k-1)*1464)/8，改动须两处同步）
    FRAG_FIRST, FRAG_MID = 1456, 1464           # 首片/中片载荷（= afdx_ref 常量）
    msgs_spec = [(m[0], m[1], (m[2] if len(m) > 2 else "ok"))
                 for m in case.get("msgs", [])]
    if msgs_spec and case.get("vls"):
        def nfrags(n):
            return 1 if n <= 1471 else 1 + -(-(n - FRAG_FIRST) // FRAG_MID)
        spec_by_vl = {}
        for vlid, ln, flag in msgs_spec:
            spec_by_vl.setdefault(vlid, []).append((ln, flag))
        for net in ("A", "B"):
            for vl, frames in sorted(dut[net].items()):
                spec = spec_by_vl.get(vl)
                if spec is None:
                    continue
                fi = 0
                for mi, (ln, flag) in enumerate(spec):
                    if flag == "reject":
                        continue                 # 拒收消息不产生帧
                    nf = nfrags(ln)
                    fs = frames[fi:fi + nf]
                    fi += nf
                    if len(fs) != nf:
                        break                    # 帧数不足已由逐字节比对报错
                    ipids = []
                    for k, f in enumerate(fs):
                        ip = f[14:-5]            # 去帧头/FCS/SN 后的 IP 区
                        total = (ip[2] << 8) | ip[3]
                        ipid = (ip[4] << 8) | ip[5]
                        fo = (ip[6] << 8) | ip[7]
                        off8, mf = fo & 0x1FFF, bool(fo & 0x2000)
                        if k == 0:
                            exp_off, exp_mf = 0, nf > 1
                        else:
                            exp_off = (FRAG_FIRST + (k - 1) * FRAG_MID) // 8
                            exp_mf = k < nf - 1
                        if k == 0:
                            # 单帧消息（nf==1）载荷为整条 ln；分片消息首片截 1456
                            exp_total = 20 + 8 + (min(ln, FRAG_FIRST) if nf > 1 else ln)
                        else:
                            exp_total = 20 + min(FRAG_MID, ln - FRAG_FIRST
                                                 - (k - 1) * FRAG_MID)
                        ipids.append(ipid)
                        if off8 != exp_off or mf != exp_mf or total != exp_total:
                            errs.append("网%s VL%04X 消息%d 片%d: off=%d(exp %d) "
                                        "mf=%d(exp %d) ip_total=%d(exp %d)"
                                        % (net, vl, mi, k, off8, exp_off,
                                           int(mf), int(exp_mf), total, exp_total))
                    if len(set(ipids)) > 1:
                        errs.append("网%s VL%04X 消息%d: 分片 IP ID 不一致 %s"
                                    % (net, vl, mi, ipids))
                if fi != len(frames):
                    errs.append("网%s VL%04X: 帧数 %d 与消息序列预期 %d 不匹配"
                                % (net, vl, len(frames), fi))

    # 6. 时序：同 VL 相邻帧间隔 ≥ BAG；A/B 同帧起始差 ≤ 0.5ms
    vls_cfg = case.get("vls", {})
    half_ms = 500 * CYCLES_PER_US
    for net in ("A", "B"):
        for vl, ts in sorted(times[net].items()):
            bag_cyc = vls_cfg.get(str(vl), {}).get("bag_ms", 0) * 1000 * CYCLES_PER_US
            for a, b in zip(ts, ts[1:]):
                if b - a < bag_cyc:
                    errs.append("网%s VL%04X: 帧间隔 %d < BAG %d 拍"
                                % (net, vl, b - a, bag_cyc))
    for vl in sorted(set(times["A"]) & set(times["B"])):
        ta, tb = times["A"][vl], times["B"][vl]
        for i, (a, b) in enumerate(zip(ta, tb)):
            if abs(a - b) > half_ms:
                errs.append("VL%04X 帧%d: A/B 起始差 %d 拍 > 0.5ms"
                            % (vl, i, abs(a - b)))

    # 汇总
    total = sum(len(v) for v in dut["A"].values()) + sum(len(v) for v in dut["B"].values())
    print("[CHECK] 比对帧数: %d（DUT 两网合计），逐字节比对 %d 帧" % (total, n_cmp))
    if errs:
        print("[CHECK] FAIL: %d 项" % len(errs))
        for e in errs[:20]:
            print("  - " + e)
        sys.exit(1)
    print("[CHECK] PASS: 逐字节比对 / FCS / IP校验和 / Padding / SN序列 / "
          "分片几何 / BAG间隔 / A-B 0.5ms 全部通过")


if __name__ == "__main__":
    main()
