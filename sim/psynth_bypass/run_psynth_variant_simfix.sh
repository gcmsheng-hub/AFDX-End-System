#!/usr/bin/env bash
# run_psynth_variant_simfix.sh — run_psynth_variant.sh 的 simfix 派生（E1′ 重跑主动脉）
# 与 run_psynth_variant.sh 相比**仅两处功能改动**：
#   ① vlog 网表：$SYNTH1/afdx_es.vqm（原网表，只读） -> $WD/afdx_es_variant.simfix.vqm
#      （vqm_simfix.py 对变体原网表的仿真副本：缺 .ena 的 lcell 补 .ena(vcc)、
#        缺 .inverta 的补 .inverta(gnd)，逻辑零改动；已与 scratch 现成件 SHA256 一致确证）
#   ② vsim 命令加 -suppress 2718,2685（2718=Missing connection for port，simfix 后已无实质意义）
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SYNTH1="$ROOT/sim/afdx_elinx_variant/afdx_es.runs/synth_1"
WD="$ROOT/sim/afdx_elinx_variant/out/psynth_variant_ms"
LIBS="D:/elinx/eLinx3.0/Simulation/verilog"
QBIN="/d/modeltech64_2020.4/win64"
LOGS="$WD/logs"
# VERDICT-019 阶段 1.5：TB 可选参数化（默认 tb_delivery_e2e 不变，全量后仿口径
# 不动；短集探针跑法 TB_V=tb_delivery_short，判定标签随 TB 映射）
TB_V="${TB_V:-tb_delivery_e2e}"
case "$TB_V" in
    tb_delivery_short) TAG="SHORT-TB" ;;
    *)                 TAG="DELIV-TB" ;;
esac
mkdir -p "$WD" "$LOGS"

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

echo "==== [psynth-ms] vlog 网表 simfix vqm（耐心） $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$WD/afdx_es_variant.simfix.vqm" > "$LOGS/vlog_vqm.log" 2>&1 \
    || { echo "vlog vqm FAIL"; tail -12 "$LOGS/vlog_vqm.log"; exit 1; }
echo "vlog vqm OK"

echo "==== [psynth-ms] vlog tb ($TB_V) $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$ROOT/tb/${TB_V}.v" > "$LOGS/vlog_tb.log" 2>&1 \
    || { echo "vlog tb FAIL"; tail -12 "$LOGS/vlog_tb.log"; exit 1; }
echo "vlog tb OK"

echo "==== [psynth-ms] vsim -c run all（SE-64 无容量限制，直跑；TB=$TB_V，suppress 2718,2685） $(date '+%H:%M:%S') ===="
"$QBIN/vsim.exe" -c -suppress 2718,2685 -do "run -all; quit -f" "$TB_V" \
    > "$LOGS/vsim_run.log" 2>&1
rc=$?
grep -aE "\[$TAG\]" "$LOGS/vsim_run.log" | tail -8
if [ $rc -eq 0 ] && grep -aq "\[$TAG\] PASS" "$LOGS/vsim_run.log"; then
    echo "psynth-ms $TB_V: PASS"
    exit 0
else
    echo "psynth-ms $TB_V: FAIL (vsim rc=$rc)"
    tail -12 "$LOGS/vsim_run.log"
    exit 1
fi
