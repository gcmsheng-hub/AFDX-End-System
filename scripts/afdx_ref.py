#!/usr/bin/env python3
"""afdx_ref.py — AFDX (ARINC 664 Part 7) 帧参考模型（验证平台的金标准）

帧格式依据（doc/05 + 执行单裁决 7 + doc/01 v1，已按 P7 原文逐字段核对）：
  [前导+SFD 8B 不含于文件] 目的MAC 6B | 源MAC 6B | 类型 2B | 数据 46~1500B | FCS 4B
  目的MAC = DST_MAC_CONST(32b,默认 03:00:00:00) + VLID[15:8]:VLID[7:0]
  源  MAC = 02:00:00:UID[15:8]:UID[7:0]:{IF_ID[2:0],5'b0}
            IF_ID: A网 3'b001 → 字节 0x20；B网 3'b010 → 字节 0x40
  类型   = 0x0800 (IPv4)；数据区 = IP包 + Padding(0~16B,补至46) + SN(1B)
  约束：UDP 载荷 1~1471B（IP 包 ≤1499）；SN 0~255：0=复位后首帧，之后 1..255 循环、255→1
  分片：单条消息 >1471B 走 IPv4 分片（首片含 UDP 头，UDP Length=8+整条消息长；后续片纯 IP）
"""
import struct
import zlib

IF_ID_A = 0x20          # 3'b001 << 5
IF_ID_B = 0x40          # 3'b010 << 5
ETH_TYPE_IPV4 = 0x0800
PROTO_UDP = 17
MAX_PAYLOAD = 1471      # UDP 载荷上限
MIN_DATA = 46           # 数据区下限（IP包 + Padding + SN）
MAX_DATA = 1500         # 数据区上限（IP ≤1499 + SN 1B）
SN_MIN, SN_MAX = 1, 255
SN_FIRST = 0            # 复位后第一帧 SN=0（P7-23 §3.2.6.1），之后 0→1→…→255→1

# ---------------------------------------------------------------- 工具
def next_sn(sn):
    """SN 递增（v2 §5.2）：0→1，1..254→+1，255→1（0 只在复位后第一帧出现）。"""
    return SN_MIN if sn >= SN_MAX else sn + 1

def ip_csum(data):
    """IPv4 反码校验和（对校验和字段清零后的头计算）。"""
    if len(data) % 2:
        data += b"\x00"
    s = sum(struct.unpack("!%dH" % (len(data) // 2), data))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF

def mac_src(uid, if_id):
    return bytes([0x02, 0x00, 0x00, (uid >> 8) & 0xFF, uid & 0xFF, if_id & 0xFF])

def mac_dst(vlid):
    return bytes([0x03, 0x00, 0x00, 0x00, (vlid >> 8) & 0xFF, vlid & 0xFF])

def dst_ip_for_vl(vlid):
    """v2 §5.1：目的 IP = 组播 224.224.VL[15:8].VL[7:0]"""
    return "224.224.%d.%d" % ((vlid >> 8) & 0xFF, vlid & 0xFF)

# ---------------------------------------------------------------- IPv4/UDP
def build_udp_ip(payload, uid, dst_ip, src_ip, sport, dport, ip_id,
                 frag_off_mf=0, udp_len_field=None):
    """构造一个 IP 包（可能为分片）。
    frag_off_mf: bit13=MF，低13位=偏移(8B单位)。
    udp_len_field: UDP Length 字段值——分片首片必须传 8+整条消息总长（ISS-06/VERDICT-004-7）；
                   非分片缺省 8+len(payload)。"""
    is_frag = (frag_off_mf & 0x1FFF) != 0
    if not is_frag:
        ulen = udp_len_field if udp_len_field is not None else 8 + len(payload)
        udp = struct.pack("!HHHH", sport, dport, ulen, 0) + payload
        total = 20 + len(udp)
    else:
        total = 20 + len(payload)                      # 非首片：纯 IP 分片
    iph = struct.pack("!BBHHHBBH4s4s",
                      0x45, 0, total, ip_id, frag_off_mf & 0xFFFF,
                      64, PROTO_UDP, 0,
                      bytes(int(x) for x in src_ip.split(".")),
                      bytes(int(x) for x in dst_ip.split(".")))
    iph = iph[:10] + struct.pack("!H", ip_csum(iph)) + iph[12:]
    return iph + (udp if not is_frag else payload)

# ---------------------------------------------------------------- 组帧
def build_frame(ip_pkt, vlid, uid, if_id, sn):
    """IP 包 + Padding(0~16B,补至数据区46) + SN → 完整以太网帧（含 FCS）。
    Padding 位于 IP 包之后、SN 之前（P7-24 图3-19；VERDICT-004 新增项 8）。"""
    pad = max(0, MIN_DATA - 1 - len(ip_pkt))
    data = ip_pkt + b"\x00" * pad + bytes([sn & 0xFF])
    if len(data) > MAX_DATA:
        raise ValueError("data field %d > 1500（应先分片）" % len(data))
    body = mac_dst(vlid) + mac_src(uid, if_id) + struct.pack("!H", ETH_TYPE_IPV4) + data
    fcs = zlib.crc32(body) & 0xFFFFFFFF
    return body + struct.pack("<I", fcs)              # FCS 小端

def message_to_frames(vlid, msg, uid, if_id, sn0, dst_ip=None, src_ip="10.0.2.5",
                      sport=5001, dport=5002, ip_id0=1):
    """一条应用消息 → [完整帧...]（>1471B 自动 IPv4 分片；SN 逐帧递增）。
    dst_ip 缺省按 P7-44 图3-38 组播规则 224.224.VL[15:8].VL[7:0]。"""
    if dst_ip is None:
        dst_ip = dst_ip_for_vl(vlid)
    frames, sn, ip_id = [], sn0, ip_id0
    if len(msg) <= MAX_PAYLOAD:
        ip = build_udp_ip(msg, uid, dst_ip, src_ip, sport, dport, ip_id)
        frames.append(build_frame(ip, vlid, uid, if_id, sn))
        return frames, next_sn(sn), ip_id + 1
    # 分片：片偏移必须 8B 对齐（IPv4 偏移单位 8B）
    #   首片载荷 = (1471-8) 向下取整到 8 的倍数 = 1456；后续片 = 1464
    first_cap = ((MAX_PAYLOAD - 8) // 8) * 8
    frag_size = MAX_PAYLOAD & ~7                       # 1464
    chunks = [msg[:first_cap]] + [msg[i:i + frag_size]
                                  for i in range(first_cap, len(msg), frag_size)]
    for k, ch in enumerate(chunks):
        off = sum(first_cap if j == 0 else frag_size for j in range(k))
        off8 = off // 8
        assert off % 8 == 0, "fragment offset not 8B aligned"
        mf = 0x2000 if k < len(chunks) - 1 else 0     # 最后一片 MF=0
        # ISS-06：首片 UDP Length = 8 + 整条消息总长（重组侧按此字段还原）
        ip = build_udp_ip(ch, uid, dst_ip, src_ip, sport, dport, ip_id,
                          mf | off8, udp_len_field=8 + len(msg) if k == 0 else None)
        frames.append(build_frame(ip, vlid, uid, if_id, sn))
        sn = next_sn(sn)
    return frames, sn, ip_id + 1

# ---------------------------------------------------------------- 接收侧参考（T7 对拍金标准）
class ReassemblyError(ValueError):
    """重组失败（缺片/重叠/首片缺失/无末片/IP ID 不一致）。"""

def reassemble(frags):
    """IP 分片重组金标准（doc/02 §9 开放项：按片偏移排序、缺片报错、超时留接口）。

    frags: [(payload, off8, mf, ip_id)] —— off8 为 8B 单位片偏移，mf=最后片标志。
    乱序到达允许；返回重组后完整消息 bytes；任何不合法抛 ReassemblyError。
    超时语义由调用侧（RTL 重组定时器 / tb）驱动，本函数只做纯拼装判定。
    """
    if not frags:
        raise ReassemblyError("no fragments")
    ip_ids = {f[3] for f in frags}
    if len(ip_ids) > 1:
        raise ReassemblyError("ip_id mismatch: %s" % sorted(ip_ids))
    last = [f for f in frags if not f[2]]
    if len(last) != 1:
        raise ReassemblyError("exactly one final (MF=0) fragment required, got %d" % len(last))
    pieces = sorted(frags, key=lambda f: f[1] * 8)          # 按字节偏移排序
    if pieces[0][1] != 0:
        raise ReassemblyError("first fragment (off=0) missing")
    if pieces[-1][2]:
        raise ReassemblyError("final (MF=0) fragment is not the tail piece")
    out = bytearray()
    pos = 0
    for payload, off8, mf, _ipid in pieces:
        start = off8 * 8
        if start < pos:
            raise ReassemblyError("overlap at offset %d" % start)
        if start > pos:
            raise ReassemblyError("gap: %d bytes missing before offset %d" % (start - pos, start))
        out += payload
        pos += len(payload)
    return bytes(out)

def ic_step(psn, seen, sn):
    """IC 完整性检查单步（v2 §5.3 / doc/01 §4.3；PSN 不论接受与否都更新）。
    返回 (verdict, new_psn, new_seen)；verdict ∈ accept/accept_lost/drop。
    accept_lost = 窗内第二格 [PSN+2]（中间丢一帧，计数器 +1）。"""
    def nxt(x):
        return 1 if (x == 255 or x == 0) else x + 1
    if not seen or sn == 0:
        verdict = "accept"                                   # 初态 / 发送端复位标志
    elif sn == nxt(psn):
        verdict = "accept"
    elif sn == nxt(nxt(psn)):
        verdict = "accept_lost"
    else:
        verdict = "drop"
    return verdict, sn, True

def rm_step(valid, rsn, sn, elapsed_us, skew_us, rm_win=127):
    """RM 冗余管理单步（v2 §5.4 / doc/07 §4.4，MODE=1 环形距离窗，First-Win）。
    elapsed_us = (now - t_last) mod 2^24（µs）。返回 (verdict, new_valid, new_rsn)；
    verdict ∈ accept/dup/old。距离公式按 doc/07 §4.4 冻结 RTL 式：
      d = (sn >= rsn) ? (sn - rsn) : (sn - rsn + 255)（8b 无符号，0 特例走旁路）。
    SN=0 复位旁路保留，但按 VERDICT-009 §1 增补 SkewMax 窗内副本去重：
      RSN==0（上一接受帧即 SN=0，t_last 即其交付时刻）且 elapsed <= skew
      → dup；否则视为发送端再次复位 → accept。（正常帧 SN 恒非 0，RSN==0
      只可能来自 SN=0 旁路，与 RTL 复用 {valid,rsn,t_last} 状态字的实现等价。）
    (sn,rsn)=(255,0)：RSN=0 定为复位同步态、在模 255 公式定义域之外，按 RTL
    现行分类记 old（VERDICT-010 §1-04 口径澄清，仅统计分类、不改交付行为）。"""
    if not valid:
        verdict = "accept"                                   # 初态首帧
    elif sn == 0:
        if rsn == 0 and elapsed_us <= skew_us:
            verdict = "dup"                                  # 窗内 SN=0 副本
        else:
            verdict = "accept"                               # 复位旁路 / 再次复位
    elif elapsed_us > skew_us:
        verdict = "accept"                                   # SkewMax 超窗无条件接受
    else:
        d = (sn - rsn) if sn >= rsn else (sn - rsn + 255)
        if d == 0:
            verdict = "dup"                                  # 重复副本
        elif 1 <= d <= rm_win:
            verdict = "accept"                               # 递增新帧
        else:
            verdict = "old"                                  # 旧帧 / 严重乱序
    if verdict == "accept":
        return verdict, True, sn
    return verdict, valid, rsn

# ---------------------------------------------------------------- 解析
def parse_frame(raw):
    """解析并校验完整帧，返回字段 dict；任何不合法抛 ValueError。"""
    if len(raw) < 64 or len(raw) > 1518:
        raise ValueError("frame len %d out of [64,1518]" % len(raw))
    body, fcs = raw[:-4], struct.unpack("<I", raw[-4:])[0]
    if zlib.crc32(body) & 0xFFFFFFFF != fcs:
        raise ValueError("FCS mismatch")
    dst, src = body[0:6], body[6:12]
    etype, = struct.unpack("!H", body[12:14])
    if etype != ETH_TYPE_IPV4:
        raise ValueError("etype 0x%04x" % etype)
    data = body[14:]
    sn = data[-1]                     # SN=0 合法（复位首帧）；是否合法由 IC 层按
                                      # 复位/窗口状态判定，解析层原样返回（VERDICT-002-2）
    raw_ip = data[:-1]
    ver_ihl, tos, total, ipid, fo, ttl, proto, csum, sip, dip = struct.unpack(
        "!BBHHHBBH4s4s", raw_ip[:20])
    if ver_ihl != 0x45 or proto != PROTO_UDP or not (20 <= total <= len(raw_ip)):
        raise ValueError("IP header invalid")
    ip = raw_ip[:total]               # 按 IP 总长截取，跳过 Padding（0~16B，VERDICT-004-8）
    hdr = ip[:20]                                     # 校验和仅覆盖 IP 头（IHL=5 → 20B）
    if ip_csum(hdr[:10] + b"\x00\x00" + hdr[12:]) != csum:
        raise ValueError("IP csum mismatch")
    frag_off, mf = fo & 0x1FFF, bool(fo & 0x2000)
    udp = ip[20:28] if frag_off == 0 else None
    payload = ip[28:total] if udp else ip[20:total]
    return dict(dst_mac=dst, src_mac=src, vlid=(dst[4] << 8) | dst[5],
                uid=(src[3] << 8) | src[4], if_id=src[5] & 0xE0,
                sn=sn, ip_id=ipid, frag_off=frag_off, mf=mf,
                udp=(struct.unpack("!HHHH", udp) if udp else None),
                payload=payload, ip_len=total)

# ---------------------------------------------------------------- 自测
def _selftest():
    uid, src_ip = 0x0A17, "10.0.2.5"
    # 1) 单帧回环：build→parse 字段一致（A/B 网 FCS 不同；目的 IP=v2 组播规则）
    msg = bytes(range(100))
    for if_id in (IF_ID_A, IF_ID_B):
        f, sn, _ = message_to_frames(0x002A, msg, uid, if_id, 1, src_ip=src_ip)
        assert len(f) == 1
        p = parse_frame(f[0])
        assert p["payload"] == msg and p["sn"] == 1 and p["vlid"] == 0x002A
        assert p["uid"] == uid and p["if_id"] == if_id and not p["mf"]
        assert p["udp"][0] == 5001 and p["udp"][1] == 5002
        assert f[0][30:34] == bytes([224, 224, 0x00, 0x2A]), "dst IP 组播规则"   # IP头[16:20]
    fa, _, _ = message_to_frames(0x002A, msg, uid, IF_ID_A, 1, src_ip=src_ip)
    fb, _, _ = message_to_frames(0x002A, msg, uid, IF_ID_B, 1, src_ip=src_ip)
    assert fa[0][11] == IF_ID_A and fb[0][11] == IF_ID_B and fa[0] != fb[0]
    # 2) SN 序列：0→1（复位首帧策略），255→1 回绕
    assert next_sn(0) == 1 and next_sn(254) == 255 and next_sn(255) == 1
    # 3) 边界帧长：1471B 载荷（总帧长 1518）
    f, _, _ = message_to_frames(1, bytes(1471), uid, IF_ID_A, 5, src_ip=src_ip)
    assert len(f[0]) == 1518 and parse_frame(f[0])["payload"] == bytes(1471)
    # 4) 最小帧：1B 载荷 → Padding 16B（IP包后、SN前）→ 64B 帧；解析载荷恰为 1B，UDP Length=9
    f, _, _ = message_to_frames(1, b"\x55", uid, IF_ID_A, 6, src_ip=src_ip)
    assert len(f[0]) == 64
    p = parse_frame(f[0])
    assert p["payload"] == b"\x55" and p["sn"] == 6
    assert p["udp"][2] == 9, "UDP Length = 8 + 1"
    # 5) 分片：3000B 消息 → 3 片（1456/1464/80），重组 == 原文，偏移 8B 对齐，SN 连续
    big = bytes((i * 13 + 7) & 0xFF for i in range(3000))
    frs, sn_end, _ = message_to_frames(2, big, uid, IF_ID_A, 10, src_ip=src_ip)
    assert len(frs) == 3 and sn_end == next_sn(next_sn(next_sn(10)))
    offs = [parse_frame(f)["frag_off"] for f in frs]
    assert offs == [0, 182, 365], offs                    # 0/1456*8? 1456/8=182, 2920/8=365
    assert parse_frame(frs[0])["mf"] and parse_frame(frs[1])["mf"] and not parse_frame(frs[2])["mf"]
    reasm = b"".join(parse_frame(f)["payload"] for f in frs)
    assert reasm == big, "reassembly mismatch"
    # 6) CRC 错误检出：翻转一比特必须抛异常
    bad = bytearray(frs[0]); bad[30] ^= 0x01
    try:
        parse_frame(bytes(bad)); raise SystemExit("FAIL: corrupted frame accepted")
    except ValueError:
        pass
    # 7) SN=0（复位首帧）：组帧→解析→载荷原样（合法性归 IC 层，VERDICT-002-2）
    f0, sn_after, _ = message_to_frames(3, b"\x01\x02\x03", uid, IF_ID_A, 0, src_ip=src_ip)
    p0 = parse_frame(f0[0])
    assert p0["sn"] == 0 and p0["payload"] == b"\x01\x02\x03"
    assert sn_after == next_sn(0) == 1
    # 8) reassemble 金标准（T7 接收重组对拍）：在序/乱序/缺片/重叠/IP ID 不一致
    frs, _, _ = message_to_frames(2, big, uid, IF_ID_A, 10, src_ip=src_ip)
    frs6, _, _ = message_to_frames(4, bytes((i * 3 + 5) & 0xFF for i in range(8192)),
                                   uid, IF_ID_A, 20, src_ip=src_ip)
    assert len(frs6) == 6
    def _frags(frames):
        ps = [parse_frame(f) for f in frames]
        return [(p["payload"], p["frag_off"], p["mf"], p["ip_id"]) for p in ps]
    assert reassemble(_frags(frs)) == big                          # 3 片在序
    assert reassemble(_frags(frs6)) == bytes((i * 3 + 5) & 0xFF for i in range(8192))
    sh = _frags(frs); sh = [sh[1], sh[2], sh[0]]                   # 乱序（TC-FRG-04 口径）
    assert reassemble(sh) == big
    for bad in ([sh[1], sh[2]],                                    # 缺首片
                [sh[0], sh[1]],                                    # 缺末片
                [sh[0], sh[0], sh[2]],                             # 重叠/重复
                sh[:2] + [(b"\x00" * 8, 183, False, 1)],           # 中间空洞
                sh[:2] + [(sh[2][0], sh[2][1], False, 999)]):      # IP ID 不一致
        try:
            reassemble(bad); raise SystemExit("FAIL: bad reassembly accepted")
        except ReassemblyError:
            pass
    # 9) IC/RM 单步参考：状态转移全矩阵穷举（RTL 单元 tb 逐向量对拍同一生成口径）
    for psn in range(256):
        w1 = 1 if psn in (0, 255) else psn + 1             # nxt(psn)
        w2 = 1 if w1 in (0, 255) else w1 + 1               # nxt(nxt(psn))
        for seen in (0, 1):
            for sn in range(256):
                v, npsn, nseen = ic_step(psn, seen, sn)
                assert npsn == sn and nseen == 1
                if not seen or sn == 0 or sn == w1 or sn == w2:
                    assert v in ("accept", "accept_lost"), (psn, seen, sn, v)
                else:
                    assert v == "drop", (psn, seen, sn, v)
    for rsn in range(256):
        for sn in range(256):
            for skew in (0, 5000):
                for el in (0, skew, skew + 1):
                    for valid in (0, 1):
                        v, nv, nrsn = rm_step(valid, rsn, sn, el, skew)
                        assert nv in (0, 1)
                        if v == "accept":
                            assert nv == 1 and nrsn == sn
                        else:
                            assert nv == valid and nrsn == rsn and v in ("dup", "old")
    print("afdx_ref selftest: PASS (round-trip, SN 0-first+wrap+parse, 1518B/64B bounds, "
          "8B-aligned 3-fragment reassembly, multicast dst IP, FCS error detection, "
          "reassemble in/out-of-order+5 error kinds, IC/RM step exhaustive 256x256)")

if __name__ == "__main__":
    _selftest()
