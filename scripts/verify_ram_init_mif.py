#!/usr/bin/env python3
"""T3 证据固化：验证 eLinx 综合生成的 MIF 与 $readmemh 初始化数据逐条一致。

证据链：ram_init.hex ($readmemh) → eLinx 综合 → altsyncram 原语 (ram_block1a0~1a7)
        → vqm defparam .init_file → hello_afdx.ram0_ram_init_test_*.hdl.mif → 位流生成器消费

用法: python3 verify_ram_init_mif.py <mif文件> [hex文件]
"""
import sys, pathlib, re

def parse_mif(path):
    """解析 Quartus 风格 MIF，返回 [(地址, int值)]。容忍降序排列与注释。"""
    entries = {}
    for line in open(path, encoding="utf-8", errors="ignore"):
        s = line.split("--")[0].strip().rstrip(";").strip()
        if not s or re.match(r"(?i)^(WIDTH|DEPTH|ADDRESS_RADIX|DATA_RADIX|CONTENT)\b", s):
            continue
        if ":" in s:
            a, v = s.split(":", 1)
            entries[int(a.strip())] = int(v.strip(), 2)  # DATA_RADIX=BIN
    return entries

def main():
    mif_path = pathlib.Path(sys.argv[1])
    hex_path = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else \
        mif_path.parents[2] / "k1k2_bram" / "ram_init.hex"
    mif = parse_mif(mif_path)
    ref = {i: (i * 7 + 11) & 0xFF for i in range(4096)}  # gen_init_hex.py 同一公式
    if hex_path.exists():
        ref = {i: int(l, 16) for i, l in enumerate(open(hex_path)) if l.strip()}
    if len(mif) != len(ref):
        print(f"FAIL: MIF 条目数 {len(mif)} != 期望 {len(ref)}"); return 1
    bad = [(a, mif[a], ref[a]) for a in ref if mif.get(a) != ref[a]]
    if bad:
        print(f"FAIL: {len(bad)} 条不一致，前 3 条: {bad[:3]}"); return 1
    print(f"PASS: MIF {len(mif)} 条与 $readmemh 数据按地址全量一致")
    print(f"证据: {mif_path}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
