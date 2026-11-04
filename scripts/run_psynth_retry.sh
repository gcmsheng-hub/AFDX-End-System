#!/usr/bin/env bash
# run_psynth_retry.sh — VERDICT-016 验收④授权重试
# 法一：vsim -voptargs="-suppress 14408"（vopt-14408 为 suppressible 类，工具自证）
# 法二：仍拒则 vsim -novopt（跳过 vopt 优化装载）
# 每法各试一次；vopt/装载阶段由外部会话监控 60 分钟上限与内存；
# 仿真范围=交付 e2e 主集（tb_delivery_e2e，6 信封）。
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WD="$ROOT/sim/afdx_elinx/out/psynth_work"
QBIN="/d/Questa/25.1std/questa_fse/win64"
LOGS="$WD/logs"
cd "$WD" || exit 1
export PATH="$QBIN:$PATH"

run_method() {
  local name="$1"; shift
  local t0=$(date +%s)
  "$QBIN/vsim.exe" -c "$@" -do "run -all; quit -f" tb_delivery_e2e \
      > "$LOGS/vsim_run_$name.log" 2>&1
  local rc=$?
  local t1=$(date +%s)
  echo "[$name] vsim rc=$rc elapsed=$((t1-t0))s $(date '+%H:%M:%S')"
  grep -aE "^\# \*\* Error|\[DELIV-TB\]|Error loading|End time" "$LOGS/vsim_run_$name.log" | head -8
  if [ $rc -eq 0 ] && grep -aq "\[DELIV-TB\] PASS" "$LOGS/vsim_run_$name.log"; then
    return 0
  fi
  return 1
}

echo "==== [retry] 法一 -suppress 14408 start $(date '+%H:%M:%S') ===="
if run_method m1 -voptargs="-suppress 14408"; then
  echo "==== [retry] 法一 PASS，结束 ===="
  exit 0
fi
echo "==== [retry] 法一未通过，法二 -novopt start $(date '+%H:%M:%S') ===="
if run_method m2 -novopt; then
  echo "==== [retry] 法二 PASS，结束 ===="
  exit 0
fi
echo "==== [retry] 两法皆未通过（详见 logs/vsim_run_m1.log / vsim_run_m2.log） ===="
exit 1
