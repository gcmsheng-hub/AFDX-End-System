#!/usr/bin/env python3
"""vqm_simfix3.py -- FLAGSHIP_Q2_VERDICT §5 Pass 3 + Pass 4 增量补丁（只做这两段）

输入必须是 simfix2 件（v1 ena/inverta + cin_used 已在位）。本脚本**只**改 10 处：
  Pass 3（2 处 ROM，pause_tx_mac，macA/macB 各一）：
    * init_file_layout "Port_A" -> "port_a"（elinx_atoms.v 只在严格小写时装载 mem_init）
    * 补 defparam .mem_init0 = 256'h...（内容取自 MIF：地址 1..72 为 1，其余 0）
  Pass 4（8 个 mdio 快链单元）：
    * .ena(link~2) -> .ena(vcc)
    * lut_mask AA00->AAAA (link) / F000->FF00 (speed[1]) /
               CC00->0000 (speed[0]) / F000->FF00 (mac_duplex_mode)
其余任何行一律原样输出；ena/inverta/cin_used 不做别的改动。

usage: python vqm_simfix3.py <simfix2.vqm> <simfix3.vqm>
"""
import re
import sys

MAC_LV = ("Tri_Mode_Ethernet_Mac_0:u_ip|Tri_Mode_Ethernet_Mac_Module:u_Tri_Mode_Ethernet_Mac_Module")

CELL_MASKS = {}
for mac in ("u_macA", "u_macB"):
    base = ("\\afdx_es_core:u_core|mac_ip_top:%s|%s|mdio_top_mac:u_mdio_top_mac|" % (mac, MAC_LV))
    CELL_MASKS[base + "link~I"] = "AAAA"
    CELL_MASKS[base + "speed[1]~I"] = "FF00"
    CELL_MASKS[base + "speed[0]~I"] = "0000"
    CELL_MASKS[base + "mac_duplex_mode~I"] = "FF00"

ROMS = []
for mac in ("u_macA", "u_macB"):
    ROMS.append("\\afdx_es_core:u_core|mac_ip_top:%s|%s|mac_tx_top_mac:u_mac_tx_top_mac|"
                "mac_tx_ctrl_mac:u_mac_tx_ctrl_mac|pause_tx_mac:u_pause_tx_mac|"
                "altsyncram:WideOr0_rtl_0|altsyncram_9c21:auto_generated|ram_block1a0~I"
                % (mac, MAC_LV))

# MIF: addr0=0, addr1..72=1, addr73..255=0 -> 值为 2^73-2
MEM_INIT0 = 256
mem_val = (1 << 73) - 2
assert mem_val.bit_length() <= MEM_INIT0
MEM_HEX = "%064X" % mem_val
MEM_LIT = "256'h" + MEM_HEX


def main(src, dst):
    st = dict(mdio_ena=0, mdio_mask=0, rom_layout=0, rom_meminit=0)
    cur = None
    out = open(dst, "w", encoding="latin-1", newline="")
    with open(src, "r", encoding="latin-1", newline="") as f:
        for line in f:
            m = re.match(r"^stratix_lcell (\S+) \(", line)
            if m:
                cur = m.group(1)
                out.write(line)
                continue
            if cur in CELL_MASKS and re.match(r"^\t\.ena\(", line):
                out.write("\t.ena(vcc),\n")
                st["mdio_ena"] += 1
                continue
            m = re.match(r'^defparam (\S+) \.lut_mask = "([0-9A-Fa-f]{4})";', line)
            if m and m.group(1) in CELL_MASKS:
                name, old = m.group(1), m.group(2).upper()
                new = CELL_MASKS[name]
                expect_old = {"AAAA": "AA00", "FF00": "F000", "0000": "CC00"}[new]
                if old != expect_old:
                    sys.stderr.write("ANOMALY: %s lut_mask=%s expect %s\n" % (name, old, expect_old))
                    sys.exit(1)
                out.write('defparam %s .lut_mask = "%s";\n' % (name, new))
                st["mdio_mask"] += 1
                continue
            m = re.match(r'^defparam (\S+) \.init_file_layout = "Port_A";', line)
            if m and m.group(1) in ROMS:
                name = m.group(1)
                out.write('defparam %s .init_file_layout = "port_a";\n' % name)
                out.write('defparam %s .mem_init0 = %s;\n' % (name, MEM_LIT))
                st["rom_layout"] += 1
                st["rom_meminit"] += 1
                continue
            out.write(line)
    out.close()

    print("   +mdio_fastlink: %d  (ena %d / mask %d)" %
          (st["mdio_ena"], st["mdio_ena"], st["mdio_mask"]))
    print("        +rom_init: %d  (layout %d / mem_init0 %d)" %
          (st["rom_layout"], st["rom_layout"], st["rom_meminit"]))
    ok = (st["mdio_ena"] == 8 and st["mdio_mask"] == 8 and
          st["rom_layout"] == 2 and st["rom_meminit"] == 2)
    if not ok:
        sys.stderr.write("SELF-CHECK FAIL: expected mdio 8/8, rom 2/2\n")
        sys.exit(2)
    print("SELF-CHECK OK: mdio 8/8, rom 2/2")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
