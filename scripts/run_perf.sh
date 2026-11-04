#!/usr/bin/env bash
# run_perf.sh — T8 性能/边界 11 条批量执行（GEM-003 §二清单）
# 用法: bash scripts/run_perf.sh [case ...]（缺省全 11 条；逐案 gen→sim→chk）
set -u
cd "$(dirname "$0")/.."
OUT="sim/tb_out"
LOGS="$OUT/logs/perf"
mkdir -p "$LOGS"

IVERILOG="${IVERILOG:-iverilog}"; VVP="${VVP:-vvp}"
if ! command -v "$IVERILOG" >/dev/null 2>&1; then
    IVERILOG="/d/Agent_Project/Agent_tool/apps/iverilog/bin/iverilog.exe"
    VVP="/d/Agent_Project/Agent_tool/apps/iverilog/bin/vvp.exe"
fi

CASES="${*:-bnd04 bnd06 vl03 vl04 vl05 vl06 vl07 vl08 vl10 perf01 perf05}"

# tb 一次性编译（T7 RTL mac 路径——性能清单不依赖 MAC IP）
SRCS="tb/tb_perf.v rtl/AFDX_top.v rtl/core/afdx_es_core.v rtl/lib/buf_mgr.v \
rtl/lib/sdp_ram.v rtl/lib/sync_fifo.v rtl/lib/rr_arbiter.v rtl/lib/tick_gen.v \
rtl/lib/crc32_d8.v rtl/lib/csum16.v rtl/core/cfg_bus.v rtl/core/ram_init.v \
rtl/core/timebase.v rtl/core/host_arb.v rtl/core/stats.v rtl/tx/tx_vl_table.v \
rtl/tx/tx_regulator.v rtl/tx/tx_scheduler.v rtl/tx/tx_sn_gen.v \
rtl/tx/tx_frame_builder.v rtl/mac/mac_tx.v rtl/mac/mac_port_if.v rtl/mac/mac_rx.v \
rtl/rx/rx_path.v rtl/rx/rx_parser.v rtl/rx/rx_vl_lookup.v rtl/rx/rx_integrity.v \
rtl/rx/rx_redundancy.v rtl/rx/rx_reassembly.v"
echo "---- [perf_compile]"
if ! "$IVERILOG" -g2001 -Wall -I tb -o "$LOGS/tb_perf.vvp" $SRCS \
        > "$LOGS/compile.log" 2>&1; then
    echo "     FAIL — compile"; tail -5 "$LOGS/compile.log"; exit 1
fi
echo "     PASS"

PASS=0; FAIL=0
for rc in $CASES; do
    echo "---- [perf_$rc]"
    python3 -I scripts/gen_perf_cases.py "$rc" "$OUT/perf_$rc" > "$LOGS/gen_$rc.log" 2>&1
    "$VVP" "$LOGS/tb_perf.vvp" +CASE_DIR="$OUT/perf_$rc" > "$LOGS/perf_$rc.log" 2>&1
    if grep -q "TB-PASS" "$LOGS/perf_$rc.log" \
       && PYTHONIOENCODING=utf-8 python3 -I scripts/check_perf.py "$rc" \
            "$LOGS/perf_$rc.log" "$OUT/perf_$rc" > "$LOGS/chk_$rc.log" 2>&1; then
        grep -aE "\[PERF-CHK\]" "$LOGS/chk_$rc.log" | sed 's/^/       /'
        PASS=$((PASS+1)); echo "     PASS ($rc)"
    else
        FAIL=$((FAIL+1)); echo "     FAIL ($rc) — tb/check 日志："
        tail -4 "$LOGS/perf_$rc.log" | sed 's/^/         /'
        grep -aE "\[PERF-CHK\]" "$LOGS/chk_$rc.log" 2>/dev/null | sed 's/^/         /'
    fi
done
echo "===="
echo "perf: $PASS passed, $FAIL failed"
exit "$FAIL"
