#!/usr/bin/env bash
# run_delivery_e2e.sh — VERDICT-015 验收②「交付形态 e2e」运行器
# 交付口径（与 sim/afdx_elinx 综合同源宏）：AFDX_MAC_IP=1 + AFDX_MAC_LOOPBACK=1
# （主仓 rtl/core/afdx_es_core.v 顶部硬编码）；IP 链行为替身：
# mac_ip_behav.v（Tri_Mode_Ethernet_Mac + Tri_Mode_Ethernet_Mac_0 包装）+
# mdio_phy_sim.v（虚拟 PHY）。
# tb：tb/tb_delivery_e2e.v（零层级引用，纯 AFDX_top 顶层引脚驱动）。
set -u
cd "$(dirname "$0")/.."
OUT="sim/tb_out"
LOGS="$OUT/logs/delivery_e2e"
IVERILOG="${IVERILOG:-iverilog}"; VVP="${VVP:-vvp}"
if ! command -v "$IVERILOG" >/dev/null 2>&1; then
    IVERILOG="/d/Agent_Project/Agent_tool/apps/iverilog/bin/iverilog.exe"
    VVP="/d/Agent_Project/Agent_tool/apps/iverilog/bin/vvp.exe"
fi
mkdir -p "$LOGS"

"$IVERILOG" -g2001 -Wall -I tb -o "$LOGS/tb_delivery.vvp" \
    tb/tb_delivery_e2e.v rtl/AFDX_top.v rtl/core/afdx_es_core.v \
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
    > "$LOGS/delivery_compile.log" 2>&1
if [ $? -ne 0 ]; then
    echo "delivery_e2e: COMPILE FAIL —— $LOGS/delivery_compile.log 尾部："
    tail -8 "$LOGS/delivery_compile.log"
    exit 1
fi

"$VVP" "$LOGS/tb_delivery.vvp" > "$LOGS/delivery_sim.log" 2>&1
rc=$?
grep -aE "\[DELIV-TB\]" "$LOGS/delivery_sim.log" | tail -8
if [ $rc -eq 0 ] && grep -aq "\[DELIV-TB\] PASS" "$LOGS/delivery_sim.log"; then
    echo "delivery_e2e: PASS"
    exit 0
else
    echo "delivery_e2e: FAIL (rc=$rc)"
    exit 1
fi
