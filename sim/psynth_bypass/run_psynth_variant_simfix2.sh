#!/usr/bin/env bash
# run_psynth_variant_mprobe.sh — run_psynth_variant_simfix2.sh — v2 网表 + tb_probe_micro（2026-11-04 令）
# 与 run_psynth_variant_simfix.sh 的唯一差别：TB 换 tb/tb_probe_micro.v（TAG=MPROBE），
# 收尾判定换「[MPROBE] SUMMARY 已出」（探针无 PASS 语义，由回报人判读读数）。
# - 网表换 v2 副本（v1 ena/inverta + cin_used 修正）、-suppress 2718,2685、工作目录/库/工具路径全同。
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SYNTH1="$ROOT/sim/afdx_elinx_variant/afdx_es.runs/synth_1"
WD="$ROOT/sim/afdx_elinx_variant/out/psynth_variant_ms"
LIBS="D:/elinx/eLinx3.0/Simulation/verilog"
QBIN="/d/modeltech64_2020.4/win64"
LOGS="$WD/logs"
TB_V="${TB_V:-tb_probe_micro}"
case "$TB_V" in
    tb_delivery_short) TAG="SHORT-TB" ;;
    tb_probe_micro)    TAG="MPROBE" ;;
    *)                 TAG="DELIV-TB" ;;
esac
mkdir -p "$WD" "$LOGS"

cd "$WD" || exit 1
cp -f "$SYNTH1/Tri_Mode_Ethernet_Mac_0.Tri_Mode_Ethernet_Mac_00.rtl.mif" . 2>/dev/null

export PATH="$QBIN:$PATH"

echo "==== [psynth-ms] vlib $(date '+%H:%M:%S') ===="
"$QBIN/vlib.exe" work > "$LOGS/vlib.log" 2>&1 || { cat "$LOGS/vlib.log"; exit 1; }

echo "==== [psynth-ms] vlog 原语库 $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$LIBS/220model.v" "$LIBS/elinx_mf.v" "$LIBS/elinx_atoms.v" \
    > "$LOGS/vlog_libs.log" 2>&1 \
    || { echo "vlog libs FAIL"; tail -12 "$LOGS/vlog_libs.log"; exit 1; }
echo "vlog libs OK"

echo "==== [psynth-ms] vlog 网表 simfix vqm（耐心） $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$WD/afdx_es_variant.simfix2.vqm" > "$LOGS/vlog_vqm.log" 2>&1 \
    || { echo "vlog vqm FAIL"; tail -12 "$LOGS/vlog_vqm.log"; exit 1; }
echo "vlog vqm OK"

echo "==== [psynth-ms] vlog tb ($TB_V) $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$ROOT/tb/${TB_V}.v" > "$LOGS/vlog_tb.log" 2>&1 \
    || { echo "vlog tb FAIL"; tail -12 "$LOGS/vlog_tb.log"; exit 1; }
echo "vlog tb OK"

echo "==== [psynth-ms] vsim -c run all（SE-64；TB=$TB_V，suppress 2718,2685） $(date '+%H:%M:%S') ===="
"$QBIN/vsim.exe" -c -suppress 2718,2685 -do "run -all; quit -f" "$TB_V" \
    > "$LOGS/vsim_run.log" 2>&1
rc=$?
grep -aE "\[$TAG\]" "$LOGS/vsim_run.log"
if [ $rc -eq 0 ] && grep -aq "\[$TAG\] SUMMARY" "$LOGS/vsim_run.log"; then
    echo "psynth-ms $TB_V: DONE (SUMMARY 已出)"
    exit 0
else
    echo "psynth-ms $TB_V: INCOMPLETE (vsim rc=$rc)"
    tail -12 "$LOGS/vsim_run.log"
    exit 1
fi
