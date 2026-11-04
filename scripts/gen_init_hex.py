#!/usr/bin/env python3
"""生成 ram_init.hex：mem[i] = (i*7+11) & 0xFF，共 4096 行、每行 2 位十六进制。
tb_ram_init_test.v 的 expect_val 公式与此保持一致。"""
import sys, pathlib

def main():
    out = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(__file__).parent.parent / "sim" / "k1k2_bram" / "ram_init.hex"
    lines = [f"{(i * 7 + 11) & 0xFF:02x}" for i in range(4096)]
    out.write_text("\n".join(lines) + "\n")
    print(f"written {out} ({len(lines)} lines)")

if __name__ == "__main__":
    main()
