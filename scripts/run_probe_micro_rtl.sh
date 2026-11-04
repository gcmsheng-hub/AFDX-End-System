#!/usr/bin/env bash
# run_probe_micro_rtl.sh — tb_probe_micro.v 的 RTL 基准跑（2026-11-04 令）
# 与 run_delivery_e2e.sh 的唯一差别：TB 换 tb/tb_probe_micro.v；日志目录 probe_micro；
# 判定标签换 [MPROBE]（探针无 PASS 语义，出 SUMMARY/DELIVERED 即算跑完）。
# RTL 文件清单与 run_delivery_e2e.sh 完全一致（AFDX_MAC_IP=1 + LOOPBACK=1 口径）。
set -u
cd "$(dirname "$0")/.."
OUT="sim/tb_out"
LOGS="$OUT/logs/probe_micro"
IVERILOG="${IVERILOG:-iverilog}"; VVP="${VVP:-vvp}"
if ! command -v "$IVERILOG" >/dev/null 2>&1; then
    IVERILOG="/d/Agent_Project/Agent_tool/apps/iverilog/bin/iverilog.exe"
    VVP="/d/Agent_Project/Agent_tool/apps/iverilog/bin/vvp.exe"
fi
mkdir -p "$LOGS"

"$IVERILOG" -g2001 -Wall -I tb -o "$LOGS/tb_probe_micro.vvp" \
    tb/tb_probe_micro.v rtl/AFDX_top.v rtl/core/afdx_es_core.v \
    rtl/core/cfg_bus.v rtl/core/ram_init.v rtl/core/timebase.v \
    rtl/core/host_arb.v rtl/core/stats.v \
    rtl/lib/buf_mgr.v rtl/lib/sdp_ram.v rtl/lib/sync_fifo.v \
    rtl/lib/rr_arbiter.v rtl/lib/tick_gen.v rtl/lib/crc32_d8.v rtl/lib/csum16.v \
    rtl/tx/tx_vl_table.v rtl/tx/tx_regulator.v rtl/tx/tx_scheduler.v \
    rtl/tx/tx_sn_gen.v rtl/tx/tx_frame_builder.v \
    rtl/mac/mac_ip_top.v rtl/mac/mac_ip_behav.v rtl/mac/mdio_phy_sim.v \
    rtl/mac/mac_rx.v \
    rtl/rx/rx_path.v rtl/rx/rx_parser.v rtl/rx/rx_vl_lookup.v \
    rtl/rx/rx_integrity.v rtl/rx/rx_redundancy.v rtl/rx/rx_reassembly.v \
    > "$LOGS/compile.log" 2>&1
if [ $? -ne 0 ]; then
    echo "probe_micro_rtl: COMPILE FAIL —— $LOGS/compile.log 尾部："
    tail -12 "$LOGS/compile.log"
    exit 1
fi

"$VVP" "$LOGS/tb_probe_micro.vvp" > "$LOGS/sim.log" 2>&1
rc=$?
grep -aE "\[MPROBE\]" "$LOGS/sim.log"
if [ $rc -eq 0 ] && grep -aq "\[MPROBE\] SUMMARY" "$LOGS/sim.log"; then
    echo "probe_micro_rtl: DONE (SUMMARY 已出)"
    exit 0
else
    echo "probe_micro_rtl: FAIL (rc=$rc)"
    tail -12 "$LOGS/sim.log"
    exit 1
fi
