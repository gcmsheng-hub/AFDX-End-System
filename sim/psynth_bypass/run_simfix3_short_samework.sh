#!/usr/bin/env bash
# run_simfix3_short_samework.sh — 在 simfix3 探针的既建 work 库上直接续跑 tb_delivery_short
# （不重编网表/原语库；仅 vlog TB 后 vsim；判据 PASS/FAIL）
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WD="$ROOT/sim/afdx_elinx/out/psynth_fullsim3_mp"
QBIN="/d/modeltech64_2020.4/win64"
cd "$WD" || exit 1
export PATH="$QBIN:$PATH"
echo "==== [simfix3-short] vlog tb $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$ROOT/tb/tb_delivery_short.v" > logs/vlog_tb_short.log 2>&1 \
    || { echo "vlog tb FAIL"; tail -12 logs/vlog_tb_short.log; exit 1; }
echo "vlog tb OK"
echo "==== [simfix3-short] vsim $(date '+%H:%M:%S') ===="
"$QBIN/vsim.exe" -c -suppress 2718,2685 -do "run -all; quit -f" tb_delivery_short > logs/vsim_short.log 2>&1
rc=$?
grep -a "\[SHORT-TB\]" logs/vsim_short.log
if [ $rc -eq 0 ] && grep -aq "\[SHORT-TB\] PASS" logs/vsim_short.log; then
    echo "simfix3-short: PASS"; exit 0
else
    echo "simfix3-short: FAIL/INCOMPLETE (rc=$rc)"; tail -12 logs/vsim_short.log; exit 1
fi
