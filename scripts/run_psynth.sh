#!/usr/bin/env bash
# run_psynth.sh — VERDICT-015 验收④：Questa 综合后仿真（e2e 主集）
# 口径（RUNBOOK §三 + VERDICT-015 按 .vqm 产物改）：
#   网表：sim/afdx_elinx/afdx_es.runs/synth_1/afdx_es.vqm（stratix_lcell/
#         stratix_ram_block WYSIWYG，TOP=AFDX_top，原语模型 elinx_atoms.v；
#         220model.v/elinx_mf.v 一并编入作 mf 兜底）
#   tb：tb/tb_delivery_e2e.v（零层级引用、纯 AFDX_top 顶层引脚；cfg 引脚配置；
#       片内回环在网表内）——与验收②同一 tb，验证交付口径功能在门级网表成立
#   工作目录：sim/afdx_elinx/out/psynth_work/（out/ 不入库）
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SYNTH1="$ROOT/sim/afdx_elinx/afdx_es.runs/synth_1"
WD="$ROOT/sim/afdx_elinx/out/psynth_work"
LIBS="D:/elinx/eLinx3.0/Simulation/verilog"
QBIN="/d/Questa/25.1std/questa_fse/win64"
LOGS="$WD/logs"
mkdir -p "$WD" "$LOGS"

cd "$WD" || exit 1
# INIT_FILE 相对路径解析：mif 就位工作目录
cp -f "$SYNTH1/Tri_Mode_Ethernet_Mac_0.Tri_Mode_Ethernet_Mac_00.rtl.mif" . 2>/dev/null

export PATH="$QBIN:$PATH"

echo "==== [psynth] vlib $(date '+%H:%M:%S') ===="
"$QBIN/vlib.exe" work > "$LOGS/vlib.log" 2>&1 || { cat "$LOGS/vlib.log"; exit 1; }

echo "==== [psynth] vlog 原语库 $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$LIBS/220model.v" "$LIBS/elinx_mf.v" "$LIBS/elinx_atoms.v" \
    > "$LOGS/vlog_libs.log" 2>&1 \
    || { echo "vlog libs FAIL"; tail -12 "$LOGS/vlog_libs.log"; exit 1; }
echo "vlog libs OK"

echo "==== [psynth] vlog 网表 vqm（68MB，耐心） $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$SYNTH1/afdx_es.vqm" > "$LOGS/vlog_vqm.log" 2>&1 \
    || { echo "vlog vqm FAIL"; tail -12 "$LOGS/vlog_vqm.log"; exit 1; }
echo "vlog vqm OK"

echo "==== [psynth] vlog tb $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$ROOT/tb/tb_delivery_e2e.v" > "$LOGS/vlog_tb.log" 2>&1 \
    || { echo "vlog tb FAIL"; tail -12 "$LOGS/vlog_tb.log"; exit 1; }
echo "vlog tb OK"

echo "==== [psynth] vsim -c run all $(date '+%H:%M:%S') ===="
"$QBIN/vsim.exe" -c -do "run -all; quit -f" tb_delivery_e2e \
    > "$LOGS/vsim_run.log" 2>&1
rc=$?
grep -aE "\[DELIV-TB\]" "$LOGS/vsim_run.log" | tail -8
if [ $rc -eq 0 ] && grep -aq "\[DELIV-TB\] PASS" "$LOGS/vsim_run.log"; then
    echo "psynth e2e: PASS"
    exit 0
else
    echo "psynth e2e: FAIL (vsim rc=$rc)"
    tail -12 "$LOGS/vsim_run.log"
    exit 1
fi
