#!/usr/bin/env bash
# run_psynth_short.sh — VERDICT-016 补令路线 A：Questa 门级缩范围探针集仿真
# 装载：-voptargs="-suppress 14408"（vopt-14408 已实证可抑制装载通过；
#       vsim-16154 同类非致命提示不再阻断）。
# 范围：tb_delivery_short（smoke 1B + 100B + 2000B 各一，里程碑 $time 戳）。
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WD="$ROOT/sim/afdx_elinx/out/psynth_work"
QBIN="/d/Questa/25.1std/questa_fse/win64"
LOGS="$WD/logs"
cd "$WD" || exit 1
export PATH="$QBIN:$PATH"

echo "==== [routeA] vlog short tb $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet -work work "$ROOT/tb/tb_delivery_short.v" \
    > "$LOGS/vlog_short.log" 2>&1 \
    || { echo "vlog short FAIL"; tail -8 "$LOGS/vlog_short.log"; exit 1; }
echo "vlog short OK"

echo "==== [routeA] vsim short $(date '+%H:%M:%S') ===="
"$QBIN/vsim.exe" -c -voptargs="-suppress 14408" -do "run -all; quit -f" \
    tb_delivery_short > "$LOGS/vsim_run_short.log" 2>&1
rc=$?
echo "[routeA] vsim rc=$rc $(date +%H:%M:%S)"
grep -aE "\[SHORT-TB\]|Error loading|End time" "$LOGS/vsim_run_short.log" | tail -12
if [ $rc -eq 0 ] && grep -aq "\[SHORT-TB\] PASS" "$LOGS/vsim_run_short.log"; then
    echo "routeA short: PASS"
    exit 0
fi
echo "routeA short: FAIL/未完 (rc=$rc)"
exit 1
