#!/usr/bin/env python3
"""check_perf.py — T8 性能专项判定（消费 tb_perf.v 的 [PERF-TB] 输出）

用法: python3 check_perf.py <case名> <tb日志路径> [case目录]
退出码 0=PASS 1=FAIL；逐条判定行原样打印（数字诚信：只引用真实运行输出）。
"""
import json
import pathlib
import re
import sys

CYC_PER_MS = 125000


def load(path):
    frames, msgs, lats, cnt, inj, delivered = [], [], [], {}, {}, None
    pat_f = re.compile(r"frame net=(\w) idx=(\d+) sfd_cyc=(\d+) len=(\d+) vlid=([0-9a-f]+)")
    pat_m = re.compile(r"msg idx=(\d+) vlid=([0-9a-f]+) len=(\d+) first_cyc=(\d+)")
    pat_l = re.compile(r"lat vlid=([0-9a-f]+) seq=(\d+) cyc=(\d+)")
    pat_c = re.compile(r"cnt qfull=(\d+) ovf=(\d+) txlenerr=(\d+) rxlenerr=(\d+) "
                       r"rxfcs=(\d+) fragto=(\d+) ctxerr=(\d+) msgbad=(\d+)")
    pat_i = re.compile(r"inj total_bytes=(\d+)")
    pat_d = re.compile(r"delivered (\d+) msgs \(exp (\d+)\)")
    for line in pathlib.Path(path).read_text(errors="replace").splitlines():
        m = pat_f.search(line)
        if m:
            frames.append(dict(net=m.group(1), idx=int(m.group(2)),
                               sfd=int(m.group(3)), len=int(m.group(4)),
                               vlid=int(m.group(5), 16)))
        m = pat_m.search(line)
        if m:
            msgs.append(dict(idx=int(m.group(1)), vlid=int(m.group(2), 16),
                             len=int(m.group(3)), cyc=int(m.group(4))))
        m = pat_l.search(line)
        if m:
            lats.append(dict(vlid=int(m.group(1), 16), seq=int(m.group(2)),
                             cyc=int(m.group(3))))
        m = pat_c.search(line)
        if m:
            cnt = dict(qfull=int(m.group(1)), ovf=int(m.group(2)),
                       txlenerr=int(m.group(3)), rxlenerr=int(m.group(4)),
                       rxfcs=int(m.group(5)), fragto=int(m.group(6)),
                       ctxerr=int(m.group(7)), msgbad=int(m.group(8)))
        m = pat_i.search(line)
        if m:
            inj["bytes"] = int(m.group(1))
        m = pat_d.search(line)
        if m:
            delivered = dict(got=int(m.group(1)), exp=int(m.group(2)))
    return frames, msgs, lats, cnt, inj, delivered


def gaps(frames, net="A"):
    """同（网, VL）帧的 SFD 间隔（拍）——A/B 锁步同时刻，混算会得 0"""
    per = {}
    for f in frames:
        if f["net"] == net:
            per.setdefault(f["vlid"], []).append(f["sfd"])
    out = {}
    for v, ts in per.items():
        ts.sort()
        out[v] = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
    return out


def check(case, log, casedir):
    frames, msgs, lats, cnt, inj, delivered = load(log)
    cj = json.loads((pathlib.Path(casedir) / "case.json").read_text())
    res = []

    def chk(ok, name, detail):
        res.append((ok, name, detail))

    if case in ("bnd04", "bnd06"):
        exp_len = 65 if case == "bnd04" else 1047
        n_ok = len(frames) == 2 and all(f["len"] == exp_len for f in frames)
        chk(n_ok, "frame_len", "%d 帧长=%s exp %d" %
            (len(frames), sorted(set(f["len"] for f in frames)), exp_len))
        m_ok = (delivered == dict(got=1, exp=1) and len(msgs) == 1
                and msgs[0]["len"] == (20 if case == "bnd04" else 1002))
        chk(m_ok, "deliver", "%s exp 1 条 %dB" %
            (delivered, 20 if case == "bnd04" else 1002))
        chk(all(cnt[k] == 0 for k in cnt), "counters", str(cnt))

    elif case.startswith("vl0") and case != "vl10":
        bag_ms = {"vl03": 4, "vl04": 8, "vl05": 16, "vl06": 32,
                  "vl07": 64, "vl08": 128}[case]
        n_exp = {"vl03": 3, "vl04": 3, "vl05": 2, "vl06": 2,
                 "vl07": 2, "vl08": 2}[case]
        gs = gaps(frames)
        min_gap = min((g for v in gs for g in gs[v]), default=-1)
        chk(len(frames) == 2 * n_exp, "frame_cnt",
            "%d (A+B, exp %d)" % (len(frames), 2 * n_exp))
        chk(min_gap >= bag_ms * CYC_PER_MS, "bag_gap",
            "min SFD 间隔 %d 拍 exp ≥%d（BAG=%dms）" %
            (min_gap, bag_ms * CYC_PER_MS, bag_ms))
        chk(delivered == dict(got=n_exp, exp=n_exp), "deliver", str(delivered))
        chk(all(cnt[k] == 0 for k in cnt), "counters", str(cnt))

    elif case == "vl10":
        exp_cnt = {0x0031: 64, 0x0032: 32, 0x0033: 16, 0x0034: 8,
                   0x0035: 4, 0x0036: 2, 0x0037: 1, 0x0038: 1}
        cnt_a = {}
        for f in frames:
            if f["net"] == "A":
                cnt_a[f["vlid"]] = cnt_a.get(f["vlid"], 0) + 1
        chk(all(cnt_a.get(v, 0) == n for v, n in exp_cnt.items()),
            "frame_cnt_per_vl", "%s exp %s" % (sorted(cnt_a.items()),
                                               sorted(exp_cnt.items())))
        gs = gaps(frames)
        bags = {0x0031: 1, 0x0032: 2, 0x0033: 4, 0x0034: 8,
                0x0035: 16, 0x0036: 32, 0x0037: 64, 0x0038: 128}
        bad = [(v, min(g)) for v, g in gs.items()
               if g and min(g) < bags[v] * CYC_PER_MS]
        chk(not bad, "bag_gap", "violations=%s" % bad)
        chk(delivered == dict(got=sum(exp_cnt.values()), exp=sum(exp_cnt.values())),
            "deliver", str(delivered))
        chk(all(cnt[k] == 0 for k in cnt), "counters", str(cnt))

    elif case == "perf01":
        # 可行上界口径：聚合吞吐 = 交付纯载荷 / 观测窗（首帧 SFD → 末交付）；
        # 模型上界 ≈84.6Mbps（BAG 服务能力），doc/06 的 100Mbps 差异报调度
        total = cj.get("total") or 0
        got = sum(m["len"] - 2 for m in msgs)
        acc = inj.get("bytes", 0) - 4 * cnt.get("qfull", 0) * 8194 if inj else 0
        chk(delivered and delivered["got"] == delivered["exp"],
            "deliver", str(delivered))
        chk(got == total, "payload_bytes",
            "交付纯载荷 %dB（注入基准 %dB）" % (got, total))
        if len(frames) >= 2 and msgs:
            span = (msgs[-1]["cyc"] - frames[0]["sfd"]) / 125.0  # us
            rate = got * 8.0 / (span * 1e-6) / 1e6
            chk(rate >= 80.0, "throughput",
                "聚合吞吐 %.2f Mbps（观测窗 %.0f us；BAG 服务模型上界 ≈84.6，"
                "doc 100 差异报调度）" % (rate, span))
        else:
            chk(False, "throughput", "无交付")
        chk(cnt.get("ovf", 0) == 0 and cnt.get("rxfcs", 0) == 0
            and cnt.get("msgbad", 0) == 0 and cnt.get("ctxerr", 0) == 0
            and cnt.get("fragto", 0) == 0, "counters", str(cnt))

    elif case == "perf05":
        base = [l["cyc"] for l in lats if l["vlid"] == 0x0000]
        chk(len(base) >= 50, "baseline_frames", "%d 帧时延样本 exp ≥50" % len(base))
        jit = max(base) - min(base) if base else -1
        chk(jit < 12500, "jitter", "基准 VL 时延极差 %d 拍 = %.1f us exp <100us"
            % (jit, jit / 125.0))
        chk(cnt["ovf"] == 0 and cnt["rxfcs"] == 0 and cnt["msgbad"] == 0,
            "counters", str(cnt))

    ok_all = all(ok for ok, _, _ in res)
    for ok, name, detail in res:
        print("[PERF-CHK] %s: %s — %s" % ("PASS" if ok else "FAIL", name, detail))
    print("[PERF-CHK] %s (%s)" % ("PASS" if ok_all else "FAIL", case))
    return 0 if ok_all else 1


def main():
    case = sys.argv[1]
    log = sys.argv[2]
    casedir = sys.argv[3] if len(sys.argv) > 3 else "sim/tb_out/perf_" + case
    sys.exit(check(case, log, casedir))


if __name__ == "__main__":
    main()
