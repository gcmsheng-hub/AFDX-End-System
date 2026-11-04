#!/usr/bin/env python3
"""gen_perf_cases.py — T8 性能/边界专项用例生成（GEM-003 §二 11 条待测清单）

逐条对应 doc/06 用例与 VERDICT-009 §二/SN=0 去重验收口径：
  bnd04   TC-BND-04  跨 64B 最小帧非对齐（18B 载荷 → 65B 帧，Smax=128B）
  bnd06   TC-BND-06  中长帧无分片（1000B 载荷 → 1047B 帧，Smax=1100B）
  vl03    TC-VL-03   BAG=4ms 门控整形（3 条 500B 背靠背注入）
  vl04    TC-VL-04   BAG=8ms（3 条 1000B）
  vl05    TC-VL-05   BAG=16ms（2 条 1471B）
  vl06    TC-VL-06   BAG=32ms（2 条 100B）
  vl07    TC-VL-07   BAG=64ms（2 条 100B）
  vl08    TC-VL-08   BAG=128ms（2 条 100B）
  vl10    TC-VL-10   8 BAG 异构并发 64ms（帧数精确 = 64ms/BAG_i）
  perf01  TC-PERF-01 100 Mbps 纯载荷吞吐（16 VL 混合帧长 10ms，1B/10 拍口径）
  perf05  TC-PERF-05 满载抖动（128 VL 全开 + 基准 VL 50 帧时延极差 <100µs）

产出 sim/tb_out/perf_<case>/：perf_cfg.txt / perf_inject.txt / case.json
（tb_perf.v 消费；判定 scripts/check_perf.py <case> <log>）。

用法: python3 gen_perf_cases.py <case名> <输出目录>
"""
import json
import pathlib
import sys

SPORT, DPORT = 5001, 5002
CYC_PER_MS = 125000


def cfg_tx(idx, bag_ms, smax, vlid, mask=3, phase=0):
    return "T %d %d %d %d %d %d\n" % (idx, bag_ms, smax, vlid, mask, phase)


def cfg_rx(idx, vlid, ic=1, rm=1, mask=3, smax=1518, skew=5000):
    return "R %d %d %d %d %d %d %d\n" % (idx, vlid, ic, rm, mask, smax, skew)


def inj(pace, max_ms, items):
    s = "P %d\nM %d\n" % (pace, max_ms)
    for vlid, ln, cnt in items:
        s += "I %d %d %d\n" % (vlid, ln, cnt)
    return s


# ---------------------------------------------------------------- 用例
def case_bnd04():
    # 18B 载荷 → 帧 65B（IP 46 + SN 1 + 无 pad，14+47+4）
    return dict(cfg=cfg_tx(4, 1, 128, 0x0004) + cfg_rx(4, 0x0004, smax=128),
                inj=inj(1, 20, [(0x0004, 20, 1)]), max_ms=20,
                note="TC-BND-04：18B 载荷 65B 帧跨越最小帧界限（Smax=128B）")


def case_bnd06():
    # 1000B 载荷 → 1047B 帧（14+1033+... IP=20+8+1000=1028, data=1029, frame=1047）
    return dict(cfg=cfg_tx(6, 2, 1100, 0x0006) + cfg_rx(6, 0x0006, smax=1100),
                inj=inj(1, 20, [(0x0006, 1002, 1)]), max_ms=20,
                note="TC-BND-06：1000B 中长帧无分片")


def _bag_case(name, idx, vlid, bag_ms, smax, payload, n):
    env_len = 2 + payload
    return dict(cfg=cfg_tx(idx, bag_ms, smax, vlid) + cfg_rx(idx, vlid, smax=smax),
                inj=inj(1, int(bag_ms * (n + 2) + 10),
                        [(vlid, env_len, n)]),
                max_ms=int(bag_ms * (n + 2) + 10),
                note="%s：BAG=%dms 门控，%d 条 %dB 背靠背注入（doc/06：帧间隔 ≥ BAG）"
                     % (name, bag_ms, n, payload))


def case_vl03():
    return _bag_case("TC-VL-03", 0x30, 0x0030, 4, 600, 500, 3)


def case_vl04():
    return _bag_case("TC-VL-04", 0x40, 0x0040, 8, 1100, 1000, 3)


def case_vl05():
    return _bag_case("TC-VL-05", 0x50, 0x0050, 16, 1518, 1471, 2)


def case_vl06():
    return _bag_case("TC-VL-06", 0x60, 0x0060, 32, 200, 100, 2)


def case_vl07():
    return _bag_case("TC-VL-07", 0x70, 0x0070, 64, 200, 100, 2)


def case_vl08():
    return _bag_case("TC-VL-08", 0x80, 0x0080, 128, 200, 100, 2)


def case_vl10():
    # 8 VL：BAG 1/2/4/8/16/32/64/128ms，64ms 窗；注入速率=各自最高理论频率
    #（1 msg/BAG_i）→ 帧数精确 = 64ms/BAG_i（BAG=128ms 记首帧 1 条，见报告）
    bags = [1, 2, 4, 8, 16, 32, 64, 128]
    counts = [64 // b if 64 >= b else 1 for b in bags]
    # 表索引 = 信封 VLID 低 7 位（tx_vl_table / rx_vl_lookup 均按 VLID 寻址）
    cfg = "".join(cfg_tx(0x0031 + k, bags[k], 400, 0x0031 + k) for k in range(8))
    cfg += "".join(cfg_rx(0x0031 + k, 0x0031 + k, smax=400) for k in range(8))
    # 注入调度（S 行绝对时刻）：每 VL 按"自身最高理论频率"（1 msg/BAG_i）注入，
    # 相位按 VL 序错开 40µs；帧数精确 = 64ms/BAG_i
    ev = []
    for k in range(8):
        b = bags[k]
        for j in range(counts[k]):
            cyc = 12500 + k * 5000 + j * b * CYC_PER_MS
            ev.append((cyc, 0x0031 + k, 202))
    ev.sort()                                               # 按时刻排序（注入器顺序执行）
    sched = "".join("S %d %d 202\n" % (cyc, vl) for cyc, vl, _ln in ev)
    return dict(cfg=cfg, inj=sched, max_ms=80,
                note="TC-VL-10：8 BAG 异构并发 64ms，各 VL 按自身 BAG 节奏注入，"
                     "帧数=64ms/BAG_i（%s）" % dict(zip(bags, counts)))


def case_perf01():
    # 16 VL（0x0001~0x0010）：BAG=1ms×12 + 2/4/8/16ms 各 1（BAG 覆盖口径）。
    # 注入调度（S 行绝对时刻）：全部单帧（帧长取 doc/06 混合 {1,100,500,1471}，
    # 不涉分片 → 不占重组上下文）；尺寸精确凑 125000B/10ms = 100.0 Mbps：
    # 84×1471 + 1×500 + 9×100 + 36×1 = 125000B，共 130 条。
    # 注入节奏：BAG=1ms VL 每条间隔 ≥1ms（VL11 发 11 条跨 10ms 窗）。
    # 说明：doc/06 "加权平均 500B" 与 100Mbps/该 BAG 分布在 1 帧/BAG 服务下
    # 不相容（129 条×500B 仅 51.6Mbps）——取帧长混合上限口径，差异报调度。
    bags = [1] * 12 + [2, 4, 8, 16]
    counts = [10] * 11 + [11, 5, 2, 1, 1]                 # 130 条
    sizes = [1] * 36 + [100] * 9 + [500] + [1471] * 84    # 125000B；1B 前置与 1471B 簇错开
    assert sum(sizes) == 125000 and len(sizes) == sum(counts)
    cfg = ""
    for k in range(16):
        cfg += cfg_tx(0x0001 + k, bags[k], 1518, 0x0001 + k)
        cfg += cfg_rx(0x0001 + k, 0x0001 + k, smax=1518)
    # 尺寸分配：1471 尽量给 BAG=1ms VL（前 121 槽），余量给低频 VL
    slots = []
    si = 0
    for k in range(16):
        for j in range(counts[k]):
            cyc = 12500 + k * 6000 + j * bags[k] * CYC_PER_MS
            slots.append((cyc, 0x0001 + k, sizes[si] + 2))  # S 行=信封长（载荷+2）
            si += 1
    slots.sort()
    sched = "".join("S %d %d %d\n" % (c, v, l) for c, v, l in slots)
    return dict(cfg=cfg, inj=sched, max_ms=40, total=125000,
                note="TC-PERF-01：16 VL 全单帧混合帧长 100.0 Mbps 注入 10ms"
                     "（125000B；BAG=1ms×12+2/4/8/16 各 1；无分片）")


def case_perf05():
    # 128 VL 全开（BAG=1ms）；基准 VL 0x0001 200B；其余 127 条 97B。
    # W4 phase 按 VL 错 1µs（离线调度错开各 VL 的 BAG 到期，doc/02 §8.2-8），
    # 注入调度（S 行）与相位对齐：每 VL 50 条、节奏 1ms、首条 k×1µs 处。
    # W4 phase=(1000−k)µs：W0 后装载 → VL k 在 cfg 后 (1+k)µs 到期（升序错开
    # 1µs、铺满 1ms BAG）；注入与到期对齐（125125+k×125 拍 + j×1ms）
    # VLID = 0x0000~0x007F（128 条恰满 7 位 tx 表索引空间；VLID 0x0080 的低 7 位
    # =0 会覆写 0 号表项——实测抓出）。基准 VL = 0x0000（200B）。
    cfg = cfg_tx(0, 1, 400, 0x0000, phase=1000) + cfg_rx(0, 0x0000, smax=400)
    for k in range(1, 128):
        cfg += cfg_tx(k, 1, 200, k, phase=1000 - k)
        cfg += cfg_rx(k, k, smax=200)
    ev = []
    for k in range(128):
        vl = k
        for j in range(50):
            cyc = 125125 + k * 125 + j * 125250   # 1.002ms 节拍（慢于服务 1.001ms，T6-006 补偿）
            ev.append((cyc, vl, 202 if k == 0 else 99))
    ev.sort()
    sched = "".join("S %d %d %d\n" % e for e in ev)
    return dict(cfg=cfg, inj=sched, max_ms=90,
                note="TC-PERF-05：128 VL 满载（各 1ms 节奏），基准 VL 0x0001 "
                     "50 帧时延极差 <100µs")


CASES = dict(bnd04=case_bnd04, bnd06=case_bnd06,
             vl03=case_vl03, vl04=case_vl04, vl05=case_vl05,
             vl06=case_vl06, vl07=case_vl07, vl08=case_vl08,
             vl10=case_vl10, perf01=case_perf01, perf05=case_perf05)


def main():
    case = sys.argv[1] if len(sys.argv) > 1 else "bnd04"
    out = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else "sim/tb_out/perf_" + case)
    if case not in CASES:
        raise SystemExit("unknown perf case: %s" % case)
    c = CASES[case]()
    out.mkdir(parents=True, exist_ok=True)
    (out / "perf_cfg.txt").write_text(c["cfg"])
    (out / "perf_inject.txt").write_text(c["inj"])
    (out / "case.json").write_text(json.dumps(
        dict(case=case, note=c["note"], max_ms=c["max_ms"],
             total=c.get("total")), indent=1))
    print("gen_perf_cases %s -> %s" % (case, out))


if __name__ == "__main__":
    main()
