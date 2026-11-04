#!/usr/bin/env bash
# run_psynth_ms.sh — ModelSim SE-64 2020.4 版综合后仿真配方（小额余额任务单②）
# 与 scripts/run_psynth.sh（Questa 25.1std 版）同配方：编译对象与顺序完全一致，
# 仅两处不同——
#   ① 工具路径：/d/modeltech64_2020.4/win64（SE-64 全功能版，无 Starter
#      5000 实例容量限制——RPT-T8-010 §四遗留资产即插即用）
#   ② 工作目录：out/psynth_work_ms/（与 Questa 25.1 的 psynth_work/work 库
#      隔离，两代 QDB 库格式不互认；运维隔离，非编译对象改动）
# 冒烟已过（2026-11-02 22:0x）：
#   版本行 "Model Technology ModelSim SE-64 vsim 2020.4 Simulator 2020.10 Oct 13 2020"
#   启动零报错；LM_LICENSE_FILE=D:\modeltech64_2020.4\LICENSE.TXT，LICENSE.TXT 在位
# 本脚本按任务单口径**留待 22:00 后窗口执行**（重编三库+网表+全量跑）。
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SYNTH1="$ROOT/sim/afdx_elinx/afdx_es.runs/synth_1"
WD="$ROOT/sim/afdx_elinx/out/psynth_work_ms"
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

echo "==== [psynth-ms] vlog 网表 vqm（68MB，耐心） $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$SYNTH1/afdx_es.vqm" > "$LOGS/vlog_vqm.log" 2>&1 \
    || { echo "vlog vqm FAIL"; tail -12 "$LOGS/vlog_vqm.log"; exit 1; }
echo "vlog vqm OK"

echo "==== [psynth-ms] vlog tb ($TB_V) $(date '+%H:%M:%S') ===="
"$QBIN/vlog.exe" -quiet "$ROOT/tb/${TB_V}.v" > "$LOGS/vlog_tb.log" 2>&1 \
    || { echo "vlog tb FAIL"; tail -12 "$LOGS/vlog_tb.log"; exit 1; }
echo "vlog tb OK"

echo "==== [psynth-ms] vsim -c run all（SE-64 无容量限制，直跑；TB=$TB_V） $(date '+%H:%M:%S') ===="
"$QBIN/vsim.exe" -c -do "run -all; quit -f" "$TB_V" \
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
