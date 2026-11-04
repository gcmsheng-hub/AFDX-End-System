#!/usr/bin/env python3
"""gen_rx_cases.py — T7 接收通路流水级用例生成（GMII 激励 + 期望交付/计数）

产出目录 sim/tb_out/<case>/：
  rx_gmii_A.hex / rx_gmii_B.hex   GMII 行流（tb 每网每拍消费一行，独立回放）
      行格式（新文件格式，RPT-T7-001 报备；冻结的 msg/golden 格式不动）：
        "B xx"   数据拍（rx_dv=1，字节 xx）
        "I xx"   空闲拍（rx_dv=0）
        "W n"    空闲 n 拍（SkewMax 等 µs 级间隔）
      文件读完该网保持空闲。帧按绝对周期放置（place_at）。
  rx_cfg.txt    接收 VL 表配置："idx vlid ic_en rm_en mask smax skew_us"
  rx_expect.hex / rx_expect.lens   期望应用口交付消息流（VLID 2B+载荷，交付序）
  rx_stats.txt  期望事件计数："ev名 值"
  case.json     用例回显

用例（对应 doc/06 接收类）：
  rx1 基本交付+双网去重+分片混流（BND/FRG-01/02/03/06/07、SNRM-06）
  rx2 IC 序列（SNRM-02/04/05/12、255 回绕）
  rx3 RM/单网/互补/故障恢复（SNRM-06/07/08/09、NET-01/02/03/04；NET-03 为
     REV-F-03 补测真恢复场景：A 断 100ms/B 至 SN=50/恢复点 A,B 同周期 SN=51）
  rx4 错误注入（ERR-01~06）
  rx5 SN=0 副本去重（VERDICT-009 §1 / FIX3-1：双网一次交付/超窗再复位/分片首帧 SN=0）
  rx6 分片健壮性流水级（VERDICT-010 §1-01/02/03 / FIX3-2/3/4：乱序/Luna 原例/
     MF 对齐；⑥d/⑥e 为 REV-F-01 负向：hp_off=2048/4096 越界拒收）

用法: python3 gen_rx_cases.py <case名> <输出目录>
"""
import json
import pathlib
import sys
import zlib

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from afdx_ref import message_to_frames, build_udp_ip, build_frame, IF_ID_A, IF_ID_B

UID = 0x0A17
SRC_IP = "10.0.2.5"
SPORT, DPORT = 5001, 5002
CYC_PER_US = 125                       # 125 MHz


def frames_of(vlid, msg, if_id, sn0, ip_id0=1, dport=DPORT):
    return message_to_frames(vlid, msg, UID, if_id, sn0,
                             src_ip=SRC_IP, sport=SPORT, dport=dport,
                             ip_id0=ip_id0)[0]


class Net:
    """单网 GMII 行流：帧按绝对周期放置，缺口用 W 行填充。"""
    def __init__(self):
        self.lines = []
        self.cyc = 0

    def frame_at(self, cyc, f):
        fl = ["B 55"] * 7 + ["B d5"] + ["B %02x" % b for b in f]
        if cyc > self.cyc:
            self.lines.append("W %d" % (cyc - self.cyc))
            self.cyc = cyc
        elif cyc < self.cyc:
            raise ValueError("frame_at %d < current %d" % (cyc, self.cyc))
        self.lines += fl
        self.cyc += len(fl)
        return self.cyc                # 帧结束周期

    def idle_to(self, cyc):
        if cyc > self.cyc:
            self.lines.append("W %d" % (cyc - self.cyc))
            self.cyc = cyc


def us2cyc(us):          # µs → 拍（125 MHz）
    return us * CYC_PER_US


# ---------------------------------------------------------------- 用例
def case_rx1():
    """基本交付：双网同发去重（First-Win）+ 单帧边界 + 2/3/6 片分片 + 双 VL 混流。"""
    cfg = [(42, 0x002A, 1, 1, 3, 1518, 5000),
           (43, 0x002B, 1, 1, 3, 1518, 5000)]
    msgs = [
        (0x002A, bytes([0x55])),                                  # 最小帧 1B
        (0x002A, bytes(range(17))),                               # 零填充临界
        (0x002A, bytes((i * 7) & 0xFF for i in range(1471))),     # 最大单帧
        (0x002A, bytes((i * 13 + 7) & 0xFF for i in range(2000))),  # 2 片
        (0x002B, bytes((i * 11 + 3) & 0xFF for i in range(100))),   # 混流单帧
        (0x002A, bytes((i * 5 + 1) & 0xFF for i in range(3000))),   # 3 片
        (0x002B, bytes((i * 3 + 9) & 0xFF for i in range(500))),
        (0x002A, bytes((i * 29 + 9) & 0xFF for i in range(8192))),  # 6 片
    ]
    na, nb = Net(), Net()
    expect = []
    cyc = 100
    nframes = 0
    sn_seq, ip_seq = {}, {}
    for vlid, m in msgs:
        sn0 = sn_seq.get(vlid, 1)              # 从 SN=1 起（SN=0 旁路语义见 rx2）
        ip0 = ip_seq.get(vlid, 1)              # IP ID 按消息独立递增（重组键）
        fa, sn_end, ip_end = message_to_frames(vlid, m, UID, IF_ID_A, sn0, ip_id0=ip0)
        fb = frames_of(vlid, m, IF_ID_B, sn0, ip_id0=ip0)
        sn_seq[vlid] = sn_end
        ip_seq[vlid] = ip_end
        # 分片消息片间隔 2ms（真实 AFDX 受 BAG 门控；串行引擎重组读出期间
        # 环缓冲只吸收 2 帧——背靠背分片会 OVF，TC-PERF-06 的 BAG 间隔模型）
        gap = 40 if len(fa) == 1 else us2cyc(2000)   # 分片间隔 2ms
        acyc = bcyc = cyc
        for f in fa:
            acyc = na.frame_at(acyc, f) + gap
        for f in fb:
            bcyc = nb.frame_at(bcyc, f) + gap
        nframes += len(fa)
        expect.append((vlid, m))
        cyc = max(acyc, bcyc) + 60
    return dict(cfg=cfg, na=na, nb=nb, expect=expect,
                stats=dict(rxframe=nframes, fcs=0, len=0, mac=0, vlunk=0,
                           net=0, ip=0, ic=0, rmdup=nframes, rmold=0, fragto=0,
                           ovf=0, ctx=0, msg=len(msgs)),
                note="双网去重 + 单帧边界 + 2/3/6 片 + 混流")


def case_rx2():
    """IC 序列（单网 A，双 VL 分工）：
    VL 0x002C：窗口容错/超窗丢弃/PSN 强更新/SN=0 重同步；
    VL 0x0030：连续序列 252..255→1→2（IC 与 RM 双层 255→1 回绕，
    RM 距离窗要求无中间跳号——254/255/1 与前段 drop 交错会被 RM 判 old）。"""
    cfg = [(44, 0x002C, 1, 1, 1, 1518, 5000),
           (48, 0x0030, 1, 1, 1, 1518, 5000)]
    na, nb = Net(), Net()
    expect = []
    n_acc = 0
    cyc = 100
    # VL 0x002C：SN=5(acc) 7(lost-acc) 8(acc) 20(drop) 21(acc) 0(acc) 1(acc)
    vlid = 0x002C
    seq = [5, 7, 8, 20, 21, 0, 1]
    verdict = ["acc", "lost", "acc", "drop", "acc", "acc", "acc"]
    for k, (sn, v) in enumerate(zip(seq, verdict)):
        m = bytes([(k * 7 + 1) & 0xFF] * 30)
        f = frames_of(vlid, m, IF_ID_A, sn)[0]
        end = na.frame_at(cyc, f)
        if v in ("acc", "lost"):
            expect.append((vlid, m))
            n_acc += 1
        cyc = end + 60
    # VL 0x0030：252..255→1→2 连续（双层回绕）
    vlid = 0x0030
    for k, sn in enumerate([252, 253, 254, 255, 1, 2]):
        m = bytes([(k * 11 + 5) & 0xFF] * 30)
        f = frames_of(vlid, m, IF_ID_A, sn)[0]
        end = na.frame_at(cyc, f)
        expect.append((vlid, m))
        n_acc += 1
        cyc = end + 60
    return dict(cfg=cfg, na=na, nb=nb, expect=expect,
                stats=dict(rxframe=13, fcs=0, len=0, mac=0, vlunk=0,
                           net=0, ip=0, ic=1, rmdup=0, rmold=0, fragto=0,
                           ovf=0, ctx=0, msg=n_acc),
                note="IC 窗口 [PSN+1,PSN+2] + SN=0 旁路 + 双层 255 回绕")


def case_rx3():
    """RM/单网/互补/故障恢复（TC-SNRM-06/07/08/09、NET-01/02/03/04）。SkewMax=5ms。
    RM 专项 VL(0x002D) ic_en=0——RM 判决与 IC 解耦（IC 由 rx2 与穷举单元覆盖；
    doc/06 TC-SNRM-09 的旧号帧在严格 IC 窗口下会被 IC 先丢，RM 超窗分支须以
    ic_en=0 或 IC 兼容序列隔离验证，见 RPT-T7-001 报备）。另设 VL(0x002F)
    ic_en=1 验证"IC 容错 + RM 超窗"叠加路径（SN=10 → 6ms 后 SN=12 双层接受）。"""
    cfg = [(45, 0x002D, 0, 1, 3, 1518, 5000),
           (47, 0x002F, 1, 1, 1, 1518, 5000),
           (51, 0x0033, 0, 1, 3, 1518, 5000)]
    vlid = 0x002D
    na, nb = Net(), Net()
    expect = []
    st = dict(rxframe=0, rmdup=0, rmold=0, msg=0)

    def onef(sn, net, cyc, tag, v=vlid):
        m = bytes([tag] * 30)
        f = frames_of(v, m, IF_ID_A if net == "A" else IF_ID_B, sn)[0]
        end = (na if net == "A" else nb).frame_at(cyc, f)
        st["rxframe"] += 1
        return end, m

    # 1) 同网 dup（A 两份 SN=10）
    e, m = onef(10, "A", 100, 0)
    onef(10, "A", e + 100, 0)
    st["rmdup"] += 1; st["msg"] += 1; expect.append((vlid, m))
    # 2) A 先 B 后（SN=11）
    e, m = onef(11, "A", 400, 1)
    onef(11, "B", 600, 1)
    st["rmdup"] += 1; st["msg"] += 1; expect.append((vlid, m))
    # 3) B 先 A 后（SN=12，对称）
    e, m = onef(12, "B", 900, 2)
    onef(12, "A", 1100, 2)
    st["rmdup"] += 1; st["msg"] += 1; expect.append((vlid, m))
    # 4) SkewMax 窗内：A SN=30 @2ms，B @3ms（elapsed≈1ms < 5ms → 窗内 d=0 dup）
    e, m = onef(30, "A", us2cyc(2000), 3)
    onef(30, "B", us2cyc(3000), 3)
    st["rmdup"] += 1; st["msg"] += 1; expect.append((vlid, m))
    # 5) SkewMax 超窗：最后接受 ~3ms，@8.1ms 注入旧号 SN=25 → 无条件接受
    #    （elapsed≈5.1ms > SkewMax=5ms）（TC-SNRM-09，ic_en=0 口径）
    e, m = onef(25, "A", us2cyc(8100), 4)
    st["msg"] += 1; expect.append((vlid, m))
    # 6) 单网 A-only（NET-01）：SN=40..44
    cyc = e + 200
    for sn in range(40, 45):
        e, m = onef(sn, "A", cyc, 5)
        cyc = e + 200
        st["msg"] += 1; expect.append((vlid, m))
    # 7) 单网 B-only（NET-02）：SN=50..52
    for sn in range(50, 53):
        e, m = onef(sn, "B", cyc, 6)
        cyc = e + 200
        st["msg"] += 1; expect.append((vlid, m))
    # 8) 双网互补（NET-04）：奇 A 偶 B，SN=60..69
    for sn in range(60, 70):
        e, m = onef(sn, "A" if sn & 1 else "B", cyc, 7)
        cyc = e + 200
        st["msg"] += 1; expect.append((vlid, m))
    # 9) IC 容错 + RM 超窗叠加（VL 0x002F，ic_en=1）：SN=10（RM 初态接受）→
    #    6ms 后 SN=12（IC：12 ∈ [PSN+1,PSN+2] 接受；RM：elapsed 6ms > 5ms 强制接受）
    e, m = onef(10, "A", us2cyc(8300), 8, v=0x002F)
    st["msg"] += 1; expect.append((0x002F, m))
    e, m = onef(12, "A", e + us2cyc(6000), 8, v=0x002F)
    st["msg"] += 1; expect.append((0x002F, m))
    # 10) NET-03 真恢复（REV-F-03 / doc/06 TC-NET-03，专设 VL 0x0033 隔离，
    #     ic_en=0 纯 RM 口径）：A 网中断 100ms 期间 B 网以 2ms 间隔推进
    #     SN=1..50（单网降级逐帧交付）；恢复点（t0+100ms）A/B 同周期同步注入
    #     SN=51 → First-Win 交付其一（rmdup+1，仅一份）；随后 SN=52(A)/53(B)
    #     连续交付，双网均存活、无乱序无重复。
    vlid3 = 0x0033
    t0 = e + 200
    for sn in range(1, 51):                      # B 网独活段：SN=1..50 @2ms
        e, m = onef(sn, "B", t0 + (sn - 1) * us2cyc(2000), sn, v=vlid3)
        st["msg"] += 1; expect.append((vlid3, m))
    t51 = t0 + us2cyc(100000)                    # A 网恢复点（中断满 100ms）
    e, m = onef(51, "A", t51, 51, v=vlid3)       # A/B 同周期同步注入 SN=51
    onef(51, "B", t51, 51, v=vlid3)
    st["rmdup"] += 1; st["msg"] += 1; expect.append((vlid3, m))
    e, m = onef(52, "A", e + 400, 52, v=vlid3)   # 后续连续：A 网复活后首帧
    st["msg"] += 1; expect.append((vlid3, m))
    e, m = onef(53, "B", e + 400, 53, v=vlid3)
    st["msg"] += 1; expect.append((vlid3, m))
    return dict(cfg=cfg, na=na, nb=nb, expect=expect,
                stats=dict(rxframe=st["rxframe"], fcs=0, len=0, mac=0,
                           vlunk=0, net=0, ip=0, ic=0, rmdup=st["rmdup"],
                           rmold=st["rmold"], fragto=0, ovf=0, ctx=0,
                           msg=st["msg"]),
                note="RM First-Win/去重/SkewMax 窗内超窗/单网/互补 + IC/RM 叠加"
                     " + NET-03 真恢复（A 断 100ms/B 推进至 SN=50/恢复点 A,B 同周期"
                     " SN=51 仅交付一份后连续，REV-F-03）")


def case_rx4():
    """错误注入：FCS 坏/runt/未配置 VL/IP csum 错/etype 错/proto 错 全丢弃。"""
    cfg = [(46, 0x002E, 1, 1, 1, 1518, 5000)]
    vlid = 0x002E
    na, nb = Net(), Net()
    expect = []
    msg = bytes(range(40))

    def ref(sn):
        return frames_of(vlid, msg, IF_ID_A, sn)[0]

    def refcs(f):
        f = bytearray(f)
        f[-4:] = (zlib.crc32(bytes(f[:-4])) & 0xFFFFFFFF).to_bytes(4, "little")
        return bytes(f)

    na.frame_at(100, ref(1))                       # 1 好帧（对照，交付）
    expect.append((vlid, msg))
    fbad = bytearray(ref(2)); fbad[30] ^= 0x01     # 2 FCS 坏（FCS 域不重算）
    na.frame_at(300, bytes(fbad))
    na.frame_at(500, bytes(ref(3)[:63]))           # 3 runt 63B
    ip = build_udp_ip(msg, UID, "224.224.00.66", SRC_IP, SPORT, DPORT, 9)
    na.frame_at(700, build_frame(ip, 0x0066, UID, IF_ID_A, 4))   # 4 未配置 VL
    f5 = bytearray(ref(5)); f5[15] ^= 0x01         # 5 IP csum 错（重算 FCS）
    na.frame_at(900, refcs(f5))
    f6 = bytearray(ref(6)); f6[12], f6[13] = 0x08, 0x06   # 6 etype 错
    na.frame_at(1100, refcs(f6))
    f7 = bytearray(ref(7)); f7[23] = 6             # 7 proto 错（TCP）
    na.frame_at(1300, refcs(f7))
    return dict(cfg=cfg, na=na, nb=nb, expect=expect,
                stats=dict(rxframe=5, fcs=1, len=1, mac=0, vlunk=1, net=0,
                           ip=3, ic=0, rmdup=0, rmold=0, fragto=0, ovf=0,
                           ctx=0, msg=1),
                note="ERR-01~06：MAC 层好帧 5（good/vlunk/3×ip 帧体完好），"
                     "坏帧分类丢弃：fcs 1 + runt 1，应用口仅交付 1 条")


def case_rx5():
    """SN=0 副本去重（VERDICT-009 §1 / FIX3-1 三条验收用例，SkewMax=5ms）：
    ① 双网 SN=0 副本（间隔 100µs）只交付一次——第二副本窗内判 dup；
    ② 间隔 > SkewMax（+6ms）的第二次 SN=0 视为再次复位正常交付，随后 SN=1 交付；
    ③ 首条分片消息（首帧 SN=0）双副本：B 的 SN=0 首片与 SN=1 尾片均被 RM 去重，
       重组只发生一次、不触发位图重叠 abort（旧实现 B 首片进重组必重叠 abort）。"""
    cfg = [(49, 0x0031, 1, 1, 3, 1518, 5000)]
    vlid = 0x0031
    na, nb = Net(), Net()
    expect = []
    st = dict(rxframe=0, rmdup=0, msg=0)
    # ① 双网 SN=0 副本 → 只交付一次
    m1 = bytes([0xA1] * 30)
    e = na.frame_at(100, frames_of(vlid, m1, IF_ID_A, 0)[0])
    st["rxframe"] += 1
    e = nb.frame_at(100 + us2cyc(100), frames_of(vlid, m1, IF_ID_B, 0)[0])
    st["rxframe"] += 1
    st["rmdup"] += 1; st["msg"] += 1; expect.append((vlid, m1))
    # ② +6ms（> SkewMax）第二次 SN=0 → 再次复位交付；SN=1 跟进交付
    m2 = bytes([0xB2] * 30)
    e = na.frame_at(e + us2cyc(6000), frames_of(vlid, m2, IF_ID_A, 0)[0])
    st["rxframe"] += 1; st["msg"] += 1; expect.append((vlid, m2))
    m3 = bytes([0xC3] * 30)
    e = na.frame_at(e + 200, frames_of(vlid, m3, IF_ID_A, 1)[0])
    st["rxframe"] += 1; st["msg"] += 1; expect.append((vlid, m3))
    # ③ 分片消息（2000B=2 片）首帧 SN=0，双网副本各发一遍
    m4 = bytes((i * 17 + 3) & 0xFF for i in range(2000))
    fa, _, _ = message_to_frames(vlid, m4, UID, IF_ID_A, 0, ip_id0=7)
    fb, _, _ = message_to_frames(vlid, m4, UID, IF_ID_B, 0, ip_id0=7)
    t0 = e + 200
    ea = na.frame_at(t0, fa[0]); st["rxframe"] += 1                 # A 首片 SN=0
    eb = nb.frame_at(t0 + us2cyc(100), fb[0]); st["rxframe"] += 1   # B 首片（dup）
    st["rmdup"] += 1
    ea = na.frame_at(ea + us2cyc(300), fa[1]); st["rxframe"] += 1   # A 尾片 SN=1
    eb = nb.frame_at(max(ea, eb) + us2cyc(100), fb[1]); st["rxframe"] += 1  # B 尾片（dup）
    st["rmdup"] += 1
    st["msg"] += 1; expect.append((vlid, m4))
    return dict(cfg=cfg, na=na, nb=nb, expect=expect,
                stats=dict(rxframe=st["rxframe"], fcs=0, len=0, mac=0, vlunk=0,
                           net=0, ip=0, ic=0, rmdup=st["rmdup"], rmold=0,
                           fragto=0, ovf=0, ctx=0, msg=st["msg"]),
                note="SN=0 窗内副本去重：双网只交付一次/超窗再复位交付/"
                     "分片首帧 SN=0 双副本只重组一次无重叠 abort")


def case_rx6():
    """分片健壮性流水级（VERDICT-010 §1-01/02/03 / FIX3-2/3/4）：
    ⑥a 流水级乱序（TC-FRG-04 rxpipe 口径）：尾片（SN=1）先到建上下文，首片
        （SN=3，IC 容错跳 1）后到补 total → 重组完成交付（旧实现尾片无座
        abort、消息丢失）；
    ⑥b Luna 原例缩放（total=41，片 0/5/3 序：off8=5 尾片终点 49 越 total 41、
        off8=3 中片露 8B 空洞）：尾片终点不符 → abort 零交付（旧实现
        got==total 误判齐备、带空洞交付）；
    ⑥c MF=1 载荷 12B 非 8B 对齐（RFC 791 §3.2）：E_LUT len_fail 拒绝、不进
        IC/RM/重组（旧实现放行入重组、残座超时）；
    ⑥d/⑥e REV-F-01 负向：中片 hp_off=2048/4096 字节偏移越界，17 位全宽
        frag_end>8192 拒收计 lenerr（旧 14 位乘加回绕别名放行、残座超时）。"""
    cfg = [(50, 0x0032, 1, 1, 1, 1518, 5000)]
    vlid = 0x0032
    na, nb = Net(), Net()
    expect = []
    st = dict(rxframe=0, len=0, ctx=0, fragto=0, msg=0)
    dip = "224.224.%d.%d" % (vlid >> 8, vlid & 0xFF)

    def frag(ipid, off8, mf, pl, ulen=None, sn=1):
        fo = (0x2000 if mf else 0) | off8
        ip = build_udp_ip(pl, UID, dip, SRC_IP, SPORT, DPORT, ipid, fo,
                          udp_len_field=ulen)
        return build_frame(ip, vlid, UID, IF_ID_A, sn)

    # ⑥a 乱序：尾片先到（off8=182, 544B, MF=0）→ 首片后到（SN=3 容错跳 1）
    m1 = bytes((i * 23 + 5) & 0xFF for i in range(2000))
    e = na.frame_at(100, frag(100, 182, 0, m1[1456:], sn=1))
    st["rxframe"] += 1
    e = na.frame_at(e + 400, frag(100, 0, 1, m1[:1456], ulen=8 + 2000, sn=3))
    st["rxframe"] += 1
    st["msg"] += 1; expect.append((vlid, m1))
    # ⑥b Luna 原例缩放：首片 16B（total=41）→ 越界尾片 9B@off8=5 → 中片 16B@off8=3
    # （⑥a 引擎串行处理 + 2000B 重组读出占环缓冲槽至 ~6140 拍，⑥b 从 7000 拍起布点）
    e = na.frame_at(7000, frag(101, 0, 1, bytes([0x5A] * 16), ulen=49, sn=4))
    st["rxframe"] += 1
    e = na.frame_at(e + 400, frag(101, 5, 0, bytes([0x5B] * 9), sn=5))
    st["rxframe"] += 1
    st["ctx"] += 1                       # 尾片终点 49 != total 41 → abort 释放
    e = na.frame_at(e + 400, frag(101, 3, 1, bytes([0x5C] * 16), sn=6))
    st["rxframe"] += 1
    st["fragto"] += 1                    # 残座（首片已亡）由重组定时器回收
    # ⑥c MF=1 载荷 12B 非 8B 对齐：E_LUT 拒绝计 lenerr
    e = na.frame_at(e + 400, frag(102, 0, 1, bytes([0x6D] * 12), ulen=108, sn=7))
    st["rxframe"] += 1; st["len"] += 1
    # ⑥d/⑥e REV-F-01 负向（冻结终审）：中片 hp_off=2048/4096（8B 单位）→ 字节
    # 偏移 16384/32768 越界。旧实现 14 位乘加（{1'b0,hp_off}*8+plen_c 截 14 位）
    # 回绕到 0+48，别名 f_off8=0 绕过 8192 边界入重组（残座超时）；新实现 17 位
    # 全宽 frag_end>8192 拒收计 lenerr，无交付不入重组。
    e = na.frame_at(e + 400, frag(103, 2048, 1, bytes([0x6E] * 48), sn=8))
    st["rxframe"] += 1; st["len"] += 1
    e = na.frame_at(e + 400, frag(104, 4096, 1, bytes([0x6F] * 48), sn=9))
    st["rxframe"] += 1; st["len"] += 1
    return dict(cfg=cfg, na=na, nb=nb, expect=expect,
                stats=dict(rxframe=st["rxframe"], fcs=0, len=st["len"], mac=0,
                           vlunk=0, net=0, ip=0, ic=0, rmdup=0, rmold=0,
                           fragto=st["fragto"], ovf=0, ctx=st["ctx"],
                           msg=st["msg"]),
                note="流水级乱序重组交付 + Luna 原例缩放 abort 零交付 + "
                     "MF=1 非 8B 对齐拒绝 + hp_off=2048/4096 越界分片 17 位"
                     "全宽拒收（REV-F-01 负向）")


CASES = dict(rx1=case_rx1, rx2=case_rx2, rx3=case_rx3, rx4=case_rx4,
             rx5=case_rx5, rx6=case_rx6)


def main():
    case = sys.argv[1] if len(sys.argv) > 1 else "rx1"
    out = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else "sim/tb_out/" + case)
    if case not in CASES:
        raise SystemExit("unknown rx case: %s" % case)
    c = CASES[case]()
    out.mkdir(parents=True, exist_ok=True)
    (out / "rx_gmii_A.hex").write_text("\n".join(c["na"].lines) + "\n")
    (out / "rx_gmii_B.hex").write_text("\n".join(c["nb"].lines) + "\n")
    (out / "rx_cfg.txt").write_text("".join(
        "%d %d %d %d %d %d %d\n" % t for t in c["cfg"]))
    ex, exl = [], []
    for vlid, m in c["expect"]:
        ex += [(vlid >> 8) & 0xFF, vlid & 0xFF] + list(m)
        exl.append(2 + len(m))
    (out / "rx_expect.hex").write_text("".join("%02x\n" % b for b in ex))
    (out / "rx_expect.lens").write_text("".join("%d\n" % n for n in exl))
    (out / "rx_stats.txt").write_text("".join(
        "%s %d\n" % kv for kv in sorted(c["stats"].items())))
    (out / "case.json").write_text(json.dumps(
        dict(case=case, note=c["note"], n_expect=len(c["expect"]),
             stats=c["stats"]), indent=1))
    print("gen_rx_cases %s: cfg=%d expect=%d gmiiA=%d lines -> %s"
          % (case, len(c["cfg"]), len(c["expect"]), len(c["na"].lines), out))


if __name__ == "__main__":
    main()
