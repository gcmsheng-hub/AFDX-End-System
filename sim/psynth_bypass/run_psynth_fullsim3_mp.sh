#!/usr/bin/env bash
# run_psynth_fullsim3_mp.sh — 全量 simfix2 网表 + tb_probe_micro（2026-11-04 令）
# 参照 run_psynth_fullsimfix_short.sh（scripts/run_psynth_ms.sh 的全量 simfix 派生），
# 仅两处功能改动：
#   ① 网表换全量 simfix2 件（v1 + cin_used + Pass3/Pass4（mdio 快链 + pause ROM））：
#      $NETLIST = sim/afdx_elinx/out/psynth_fullsimfix_ms/afdx_es_full.simfix3.vqm
#   ② 工作目录独立：out/psynth_fullsim3_mp（与 v1 全量目录 psynth_fullsimfix_ms 隔离）
# .mif 拷入、vsim -suppress 2718,2685、判定口径均与 run_psynth_fullsimfix_short.sh 同。
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SYNTH1="$ROOT/sim/afdx_elinx/afdx_es.runs/synth_1"
NETLIST="$ROOT/sim/afdx_elinx/out/psynth_fullsimfix_ms/afdx_es_full.simfix3.vqm"
WD="$ROOT/sim/afdx_elinx/out/psynth_fullsim3_mp"
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
[ -f "$NETLIST" ] || { echo "NETLIST MISSING: $NETLIST"; exit 1; }

cd "$WD" || exit 1
# INIT_FILE 相对路径解析：mif 就位工作目录
cp -f "$SYNTH1/Tri_Mode_Ethernet_Mac_0.Tri_Mode_Ethernet_Mac_00.rtl.mif" . 2>/dev/null

export PATH="$QBIN:$PATH"

echo "==== [psynth-ms] vlib $(date '+%H:%M:%S') ===="
"$QBIN/vlib.exe" work > "$LOGS/vlib.log" 2>&1 || { cat "$LOGS/vlib.log"; exit 1; }

echo "==== [psynth-ms] vlog 原语库 $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$LIBS/220model.v" "$LIBS/elinx_mf.v" "$LIBS/elinx_atoms.v" \
    > "$LOGS/vlog_libs.log" 2>&1 \
    || { echo "vlog libs FAIL"; tail -12 "$LOGS/vlog_libs.log"; exit 1; }
echo "vlog libs OK"

echo "==== [psynth-ms] vlog 网表 simfix2 vqm（68MB，耐心） $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$NETLIST" > "$LOGS/vlog_vqm.log" 2>&1 \
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
    echo "psynth-ms $TB_V: FAIL (vsim rc=$rc)"
    tail -12 "$LOGS/vsim_run.log"
    exit 1
fi
