#!/usr/bin/env python3
"""gen_stim.py — 生成 tb 激励与金标准期望文件（文件式协同仿真的桥）。

文件格式（冻结，见 T2/doc/02 v1）：
  msg_tx.hex        每行一字节：逐条消息 [VLID_hi, VLID_lo, payload...] 连续排列
  msg_lens.txt      每行一个整数：每条消息的字节数（含 2B VLID 前缀）
  golden_{A,B}.hex  每行一字节：金标准完整以太网帧（A/B 网）连续排列
  frame_{A,B}.lens  每行一个整数：每条帧的字节数
  case.json         用例配置回显

用法: python3 gen_stim.py <case名> <输出目录>
"""
import json, pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from afdx_ref import message_to_frames, IF_ID_A, IF_ID_B, next_sn

UID = 0x0A17
SRC_IP = "10.0.2.5"                     # 目的 IP 由 afdx_ref 按组播规则 224.224.VL 生成
SPORT, DPORT = 5001, 5002

CASES = {
    # smoke：最小帧 + 最大帧 + 中间帧，多 VL 并发（覆盖官方帧长集合要求）
    # sn0=0：每 VL 首帧为复位标记帧（P7-23 §3.2.6.1；VERDICT-004 ISS-01 增量）
    "smoke": dict(sn0=0, msgs=[
        (0x002A, bytes(range(100))),                    # 中间帧（100B 载荷）
        (0x002B, bytes((i * 7) & 0xFF for i in range(1471))),  # 最大载荷 1471B
        (0x002C, b"\x55"),                              # 最小帧
    ]),

    # tx1（T6-005）：发送通路全帧对拍用例——帧长边界/分片/跨 slot/多消息排队
    # vls：VL 表索引 → 配置（tb 写 cfg；Smax 为应用载荷层级，见 doc/02 §8.2）
    "tx1": dict(sn0=0,
        vls={
            10: dict(vlid=0x000A, bag_ms=1, smax=1518, mask=3),
            11: dict(vlid=0x000B, bag_ms=2, smax=1518, mask=3),
            12: dict(vlid=0x000C, bag_ms=1, smax=64,   mask=3),
            13: dict(vlid=0x000D, bag_ms=1, smax=1518, mask=3),
        },
        msgs=[
            (10, bytes(range(100))),                          # 中间帧
            (10, bytes([0x55])),                              # 最小帧（1B，pad16）
            (10, bytes((i * 7) & 0xFF for i in range(1471))), # 最大帧 1518B
            (11, bytes((i * 13 + 7) & 0xFF for i in range(3000))),  # 3 片（2ms BAG）
            (12, bytes([0xAA])),                              # smax=64 下最小帧
            (12, bytes(range(17))),                           # 零填充临界 17B
            (13, bytes((i * 11 + 3) & 0xFF for i in range(1472))),  # 2 片，末片 16B（pad）
            (13, bytes((i * 5 + 1) & 0xFF for i in range(2049))),   # 跨 slot（2 片）
        ]),

    # tx2（FIX2/REV-002-02 回归，tb 模式 TX_EARLY）：tx_en 开启状态下交错注入——
    # 同 VL 多消息排队与发送并行；含 A-only(22)/B-only(23) 单网路由。
    # 注：表索引与 VLID 数值保持一致（check_output 的 BAG 检查按 VLID 查 vls 配置）
    "tx2": dict(sn0=0, tb="TX_EARLY",
        vls={
            20: dict(vlid=0x0014, bag_ms=1, smax=1518, mask=3),
            21: dict(vlid=0x0015, bag_ms=1, smax=1518, mask=3),
            22: dict(vlid=0x0016, bag_ms=1, smax=1518, mask=1),   # A-only
            23: dict(vlid=0x0017, bag_ms=2, smax=1518, mask=2),   # B-only
            24: dict(vlid=0x0018, bag_ms=1, smax=64,   mask=3),
        },
        msgs=[
            (20, bytes(range(100))),
            (21, bytes((i * 3 + 1) & 0xFF for i in range(200))),
            (20, bytes((i * 5 + 2) & 0xFF for i in range(50))),
            (22, bytes((i * 7 + 3) & 0xFF for i in range(300))),   # A-only
            (24, bytes(range(17))),                                # pad 临界
            (21, bytes((i * 11 + 4) & 0xFF for i in range(200))),
            (23, bytes((i * 13 + 5) & 0xFF for i in range(400))),  # B-only
            (20, bytes((i * 17 + 6) & 0xFF for i in range(1471))), # 最大帧
            (24, bytes((i * 19 + 7) & 0xFF for i in range(64))),   # smax 边界
            (21, bytes((i * 23 + 8) & 0xFF for i in range(1500))), # 2 片
        ]),

    # txbig（FIX2/REV-002-08）：8192B 最大消息端到端向量——6 片（1456/1464×4/880），
    # 逐片核对 8B 对齐偏移、MF、SN、IP ID、末片长度（check_output 分片几何核对）
    "txbig": dict(sn0=0,
        vls={
            30: dict(vlid=0x001E, bag_ms=1, smax=1518, mask=3),
            31: dict(vlid=0x001F, bag_ms=1, smax=1518, mask=3),
        },
        msgs=[
            (30, bytes((i * 29 + 9) & 0xFF for i in range(8192))),  # 最大消息（6 片）
            (30, bytes(range(100))),                                # 4 槽链释放后续发
            (31, bytes((i * 31 + 10) & 0xFF for i in range(2049))), # 跨 slot（2 片）
        ]),

    # txsmax（FIX2/REV-002-09）：Smax 拒收（单帧超限 + 分片消息超限）与资源回收。
    # msg 三元组 (vlid, payload, "reject") = 注入但期望拒收（不计入 golden、不耗 SN/IP ID）；
    # tb 模式 LENERR_EXP=3 断言 stats g_lenerr 计数
    "txsmax": dict(sn0=0, tb="LENERR_EXP=3",
        vls={
            32: dict(vlid=0x0020, bag_ms=1, smax=64,   mask=3),
            33: dict(vlid=0x0021, bag_ms=1, smax=1518, mask=3),
        },
        msgs=[
            (32, bytes((i * 3) & 0xFF for i in range(100)),  "reject"),  # 单帧 100>64
            (32, bytes((i * 5) & 0xFF for i in range(2000)), "reject"),  # 分片 1464>64
            (32, bytes(range(17))),                                      # 接受
            (32, bytes((i * 7) & 0xFF for i in range(64))),              # ==Smax 接受
            (32, bytes((i * 11) & 0xFF for i in range(65)),  "reject"),  # 65>64 拒收
            (33, bytes((i * 13) & 0xFF for i in range(500))),            # 拒收后恢复正常
        ]),

    # txsn（FIX2/REV-002-09，tb 模式 SN_WRAP）：RTL SN 255→1 回绕长序列（260 帧）
    # + 复位后首帧重发（tb 内比对 golden 帧 0：SN=0 复位标记帧）
    "txsn": dict(sn0=0, tb="SN_WRAP",
        vls={
            40: dict(vlid=0x0028, bag_ms=1, smax=1518, mask=3),
        },
        msgs=[(40, bytes([k & 0xFF])) for k in range(260)]),
}

def hexdump(path, data):
    path.write_text("".join("%02x\n" % b for b in data))

def lensdump(path, lens):
    path.write_text("".join("%d\n" % n for n in lens))

def main():
    case = sys.argv[1] if len(sys.argv) > 1 else "smoke"
    out = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else "sim/tb_out/" + case)
    if case not in CASES:
        raise SystemExit("unknown case: %s" % case)
    cfg = CASES[case]
    out.mkdir(parents=True, exist_ok=True)

    tx, tx_lens, goldA, goldB, flA, flB = [], [], [], [], [], []
    # VERDICT-004-6：SN 与 IP ID 均按 VL 独立维护；A/B 副本同一起点分别生成（同 SN）
    sn_by_vl, ipid_by_vl = {}, {}
    idx_by_vlid = {v["vlid"]: i for i, v in (cfg.get("vls") or {}).items()}
    n_reject = 0
    case_msgs = []                                   # case.json 回显（含拒收标记）
    for entry in cfg["msgs"]:
        vlid, msg = entry[0], entry[1]
        reject = len(entry) > 2 and entry[2] == "reject"
        case_msgs.append([vlid, len(msg)] + (["reject"] if reject else []))
        tx += [(vlid >> 8) & 0xFF, vlid & 0xFF] + list(msg)
        tx_lens.append(2 + len(msg))
        if reject:
            # Smax 拒收（RTL 在 eop 整条丢弃）：不产生帧、不耗 SN/IP ID，
            # 仅注入激励供 tb 驱动（DUT 拒收由 ev_lenerr/无帧体现）
            n_reject += 1
            continue
        sn0 = sn_by_vl.get(vlid, cfg["sn0"])          # 每 VL 首帧从复位 SN=0 起
        ip0 = ipid_by_vl.get(vlid, 1)                 # IP ID 每 VL 独立递增
        fa, sn_a, ip_a = message_to_frames(vlid, msg, UID, IF_ID_A, sn0,
                                           src_ip=SRC_IP, sport=SPORT, dport=DPORT, ip_id0=ip0)
        fb, sn_b, ip_b = message_to_frames(vlid, msg, UID, IF_ID_B, sn0,
                                           src_ip=SRC_IP, sport=SPORT, dport=DPORT, ip_id0=ip0)
        assert sn_a == sn_b and ip_a == ip_b, "A/B 副本 SN/IP ID 必须一致"
        sn_by_vl[vlid] = sn_a
        ipid_by_vl[vlid] = ip_a
        assert len(fa) == len(fb) and all(len(a) == len(b) for a, b in zip(fa, fb))
        for a, b in zip(fa, fb):
            goldA += a; flA.append(len(a))
            goldB += b; flB.append(len(b))

    hexdump(out / "msg_tx.hex", tx);          lensdump(out / "msg_lens.txt", tx_lens)
    hexdump(out / "golden_A.hex", goldA);     lensdump(out / "frame_A.lens", flA)
    hexdump(out / "golden_B.hex", goldB);     lensdump(out / "frame_B.lens", flB)
    # VL 配置（tb 写 cfg_bus；列：idx en bag_ms smax vlid mask phase_us sport dport）
    if cfg.get("vls"):
        lines = []
        for i, v in sorted(cfg["vls"].items()):
            lines.append("%d %d %d %d %d %d %d %d %d" % (
                i, 1, v["bag_ms"], v["smax"], v["vlid"], v["mask"], 0, SPORT, DPORT))
        (out / "vl_cfg.txt").write_text(chr(10).join(lines) + chr(10))
    tbspec = cfg.get("tb", "")
    tb_mode = "-"
    tb_lenerr = "-"
    for tok in tbspec.split():
        if tok == "TX_EARLY" or tok == "SN_WRAP":
            tb_mode = tok
        elif tok.startswith("LENERR_EXP="):
            tb_lenerr = tok.split("=", 1)[1]
    (out / "tb_mode.txt").write_text("%s\n%s\n" % (tb_mode, tb_lenerr))
    (out / "case.json").write_text(json.dumps(
        dict(case=case, uid=UID, src_ip=SRC_IP, dst_ip="224.224.VL[15:8].VL[7:0]",
             sport=SPORT, dport=DPORT, sn0="per-VL (first frame SN=0 reset marker)",
             vls=cfg.get("vls") or {},
             msgs=case_msgs,
             n_msgs=len(cfg["msgs"]), n_reject=n_reject, n_frames=len(flA)), indent=1))
    print("gen_stim %s: %d msgs, %d frames(A), %d tx bytes -> %s"
          % (case, len(cfg["msgs"]), len(flA), len(tx), out))

if __name__ == "__main__":
    main()
