#!/usr/bin/env bash
# run_psynth_iverilog.sh — VERDICT-016 补令路线 B：iverilog 编译同一 vqm 门级网表
# + 原语库（elinx_atoms/220model/elinx_mf）跑缩范围探针集。编译通即跑；
# 长跑跨窗由调用方分离进程保证，日志全落盘。
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WD="$ROOT/sim/afdx_elinx/out/psynth_work"
SYNTH1="$ROOT/sim/afdx_elinx/afdx_es.runs/synth_1"
LIBS="D:/elinx/eLinx3.0/Simulation/verilog"
LOGS="$WD/logs"
IVERILOG="/d/Agent_Project/Agent_tool/apps/iverilog/bin/iverilog.exe"
VVP="/d/Agent_Project/Agent_tool/apps/iverilog/bin/vvp.exe"
cd "$WD" || exit 1

echo "==== [routeB] iverilog 编译 vqm+libs+tb $(date '+%H:%M:%S') ===="
"$IVERILOG" -g2001 -o "$WD/psynth_iv.vvp" \
    "$LIBS/elinx_atoms.v" "$LIBS/220model.v" \
    "$SYNTH1/afdx_es.vqm" \
    "$ROOT/tb/tb_delivery_short.v" \
    > "$LOGS/iv_compile.log" 2>&1
rc=$?
if [ $rc -ne 0 ]; then
    echo "routeB COMPILE FAIL rc=$rc"
    tail -12 "$LOGS/iv_compile.log"
    exit 1
fi
echo "routeB compile OK $(date '+%H:%M:%S')"

echo "==== [routeB] vvp 短探针集 $(date '+%H:%M:%S') ===="
"$VVP" "$WD/psynth_iv.vvp" > "$LOGS/iv_run.log" 2>&1
rc=$?
echo "[$?] vvp rc=$rc $(date '+%H:%M:%S')"
grep -aE "\[SHORT-TB\]" "$LOGS/iv_run.log" | tail -12
if [ $rc -eq 0 ] && grep -aq "\[SHORT-TB\] PASS" "$LOGS/iv_run.log"; then
    echo "routeB short: PASS"
    exit 0
fi
echo "routeB short: FAIL/未完 (rc=$rc)"
exit 1
