#!/usr/bin/env python3
"""vqm_simfix2.py -- v2 simfix: v1 (ena/inverta) + arithmetic 进位链 cin_used 修正

背景（2026-11-04 微探针定案）：
  v1 补 .ena(vcc)/.inverta(gnd) 让寄存器能装载，但 arithmetic 模式的 sum 单元
  在 elinx_atoms.v 里受 `cin_used` 参数控制；缺省 "false" 时模型用 inverta 当
  进位、**忽略 .cin 链**（stratix_asynch_lcell 算术分支），导致所有计数器/加法器
  的进位丢失。网表从不写 cin_used（变体 0 条 / 全量 0 条），于是 ram_init 的
  cnt[1] 不翻转 -> init 卡在 S_TBL -> init_done 永不置位、ri_we 恒 1 吞掉 cfg 写。

v2 = v1 全部行为不变 + 对每个 arithmetic 且接有 .cin 的 stratix_lcell 实例
     追加 `defparam <inst> .cin_used = "true";`（插在实例块之后、原 defparam 之前；
     同一实例不同参数的 defparam 顺序无关）。

usage: python vqm_simfix2.py <in.vqm> <out.vqm>
"""
import re
import sys

PORT_RE = re.compile(r"^\s*\.(\w+)\s*\(")
ARITH_RE = re.compile(r'^defparam (\S+) \.operation_mode = "arithmetic";')


def lut_depends(mask_hex, bit):
    m = int(mask_hex, 16)
    for idx in range(16):
        if ((m >> idx) & 1) != ((m >> (idx ^ (1 << bit))) & 1):
            return True
    return False


def scan_arith_cin(src):
    """第一遍：找出 arithmetic 且 .cin 已连接的实例名集合。"""
    names = set()
    name = None
    ports = None
    in_block = False
    with open(src, "r", encoding="latin-1", newline="") as f:
        for line in f:
            if not in_block:
                if line.startswith("stratix_lcell "):
                    name = line.split()[1]
                    ports = set()
                    in_block = True
                    continue
                m = ARITH_RE.match(line)
                if m and name is not None and m.group(1) == name and "cin" in ports:
                    names.add(name)
                continue
            pm = PORT_RE.match(line)
            if pm:
                ports.add(pm.group(1))
            if line.rstrip().endswith(");"):
                in_block = False
    return names


def main(src, dst):
    arith_cin = scan_arith_cin(src)

    stats = dict(lcell=0, add_ena=0, add_ena_clocked=0, add_inv=0,
                 add_cin_used=0, lut_dep_unconn=0)
    dep_examples = []
    pending = {}
    out = open(dst, "w", encoding="latin-1", newline="")
    block = None
    name = None
    with open(src, "r", encoding="latin-1", newline="") as f:
        for line in f:
            if block is None:
                if line.startswith("stratix_lcell "):
                    block = [line]
                    name = line.split()[1]
                    continue
                ma = re.match(r"^defparam (\S+) \.(operation_mode = \"arithmetic\"|sum_lutc_input = \"(cin|qfbk)\");", line)
                if ma and ma.group(1) in pending:
                    del pending[ma.group(1)]
                m = re.match(r'^defparam (\S+) \.lut_mask = "([0-9A-Fa-f]{4})";', line)
                if m and m.group(1) in pending:
                    ports, mode = pending[m.group(1)]
                    for bit, p in enumerate(("dataa", "datab", "datac", "datad")):
                        if p not in ports and lut_depends(m.group(2), bit):
                            stats["lut_dep_unconn"] += 1
                            if len(dep_examples) < 10:
                                dep_examples.append((m.group(1), p, m.group(2)))
                    del pending[m.group(1)]
                out.write(line)
                continue
            block.append(line)
            if line.rstrip().endswith(");"):
                ports = set()
                for l in block[1:]:
                    pm = PORT_RE.match(l)
                    if pm:
                        ports.add(pm.group(1))
                stats["lcell"] += 1
                extra = []
                if "ena" not in ports:
                    extra.append("\t.ena(vcc),\n")
                    stats["add_ena"] += 1
                    if "clk" in ports:
                        stats["add_ena_clocked"] += 1
                if "inverta" not in ports:
                    extra.append("\t.inverta(gnd),\n")
                    stats["add_inv"] += 1
                out.write(block[0])
                out.writelines(extra)
                out.writelines(block[1:])
                if name in arith_cin:
                    out.write('defparam %s .cin_used = "true";\n' % name)
                    stats["add_cin_used"] += 1
                pending[name] = (ports, None)
                if len(pending) > 64:
                    pending.pop(next(iter(pending)))
                block = None
    out.close()
    for k, v in stats.items():
        print(f"{k:>18}: {v}")
    for e in dep_examples:
        print("  LUT depends on unconnected input:", e)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
