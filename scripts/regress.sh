#!/usr/bin/env bash
# regress.sh — 一键回归：参考模型自测 + 激励生成 + 单元/端到端仿真（iverilog）
# 用法: bash scripts/regress.sh [case名，默认 smoke]
# 日志写入 sim/tb_out/logs/<case>/（多 case 连跑互不覆盖）
set -u
cd "$(dirname "$0")/.."
CASE="${1:-smoke}"
OUT="sim/tb_out"
LOGS="sim/tb_out/logs/$CASE"
PASS=0; FAIL=0
mkdir -p "$OUT" "$LOGS"

# 仿真器解析：优先 PATH，兜底统一工具目录（Agent_tool/apps/iverilog）
IVERILOG="${IVERILOG:-iverilog}"; VVP="${VVP:-vvp}"
if ! command -v "$IVERILOG" >/dev/null 2>&1; then
    IVERILOG="/d/Agent_Project/Agent_tool/apps/iverilog/bin/iverilog.exe"
    VVP="/d/Agent_Project/Agent_tool/apps/iverilog/bin/vvp.exe"
fi

step() {  # step <名称> <命令...>
    local name="$1"; shift
    echo "---- [$name] $*"
    if "$@" > "$LOGS/$name.log" 2>&1; then
        PASS=$((PASS+1)); echo "     PASS ($(tail -1 "$LOGS/$name.log"))"
    else
        FAIL=$((FAIL+1)); echo "     FAIL —— $LOGS/$name.log 尾部："
        tail -5 "$LOGS/$name.log" | sed 's/^/       /'
    fi
}

step ref_selftest python3 -I scripts/afdx_ref.py
step gen_stim     python3 -I scripts/gen_stim.py "$CASE" "$OUT/$CASE"

echo "---- [lib_tb] rtl/lib 基础件单元测试"
if "$IVERILOG" -g2001 -Wall -o "$LOGS/tb_lib.vvp" \
       tb/tb_lib.v rtl/lib/sdp_ram.v rtl/lib/sync_fifo.v rtl/lib/crc32_d8.v \
       rtl/lib/csum16.v rtl/lib/rr_arbiter.v rtl/lib/tick_gen.v \
       > "$LOGS/lib_compile.log" 2>&1 \
   && "$VVP" "$LOGS/tb_lib.vvp" > "$LOGS/lib_tb.log" 2>&1 \
   && grep -q "\[LIB-TB\] PASS" "$LOGS/lib_tb.log"; then
    PASS=$((PASS+1)); grep -E "\[LIB-TB\]" "$LOGS/lib_tb.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -8 "$LOGS/lib_compile.log" "$LOGS/lib_tb.log" 2>/dev/null | sed 's/^/       /'
fi

echo "---- [bufmgr_tb] buf_mgr 单元测试"
if "$IVERILOG" -g2001 -Wall -o "$LOGS/tb_bufmgr.vvp" \
       tb/tb_bufmgr.v rtl/lib/buf_mgr.v rtl/lib/sdp_ram.v \
       > "$LOGS/bufmgr_compile.log" 2>&1 \
   && "$VVP" "$LOGS/tb_bufmgr.vvp" > "$LOGS/bufmgr_tb.log" 2>&1 \
   && grep -q "\[BM-TB\] PASS" "$LOGS/bufmgr_tb.log"; then
    PASS=$((PASS+1)); grep -E "\[BM-TB\]" "$LOGS/bufmgr_tb.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -8 "$LOGS/bufmgr_compile.log" "$LOGS/bufmgr_tb.log" 2>/dev/null | sed 's/^/       /'
fi

echo "---- [cfg_tb] cfg/表/初始化单元测试"
if "$IVERILOG" -g2001 -Wall -o "$LOGS/tb_cfg.vvp" \
       tb/tb_cfg.v rtl/core/cfg_bus.v rtl/core/ram_init.v rtl/core/timebase.v \
       rtl/tx/tx_vl_table.v rtl/lib/sdp_ram.v \
       > "$LOGS/cfg_compile.log" 2>&1 \
   && "$VVP" "$LOGS/tb_cfg.vvp" > "$LOGS/cfg_tb.log" 2>&1 \
   && grep -q "\[CFG-TB\] PASS" "$LOGS/cfg_tb.log"; then
    PASS=$((PASS+1)); grep -E "\[CFG-TB\]" "$LOGS/cfg_tb.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -8 "$LOGS/cfg_compile.log" "$LOGS/cfg_tb.log" 2>/dev/null | sed 's/^/       /'
fi

echo "---- [sched_tb] 调度/整形/统计单元测试"
SCH_SRCS="tb/tb_sched.v rtl/core/cfg_bus.v rtl/core/ram_init.v rtl/core/timebase.v rtl/core/host_arb.v rtl/core/stats.v rtl/tx/tx_vl_table.v rtl/tx/tx_regulator.v rtl/tx/tx_scheduler.v rtl/tx/tx_sn_gen.v rtl/lib/buf_mgr.v rtl/lib/sdp_ram.v rtl/lib/rr_arbiter.v rtl/lib/tick_gen.v"
if "$IVERILOG" -g2001 -Wall -o "$LOGS/tb_sched.vvp" $SCH_SRCS \
       > "$LOGS/sched_compile.log" 2>&1 \
   && "$VVP" "$LOGS/tb_sched.vvp" > "$LOGS/sched_tb.log" 2>&1 \
   && grep -q "\[SCH-TB\] PASS" "$LOGS/sched_tb.log"; then
    PASS=$((PASS+1)); grep -E "\[SCH-TB\]" "$LOGS/sched_tb.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -8 "$LOGS/sched_compile.log" "$LOGS/sched_tb.log" 2>/dev/null | sed 's/^/       /'
fi


echo "---- [rxvec] 接收单元向量生成（afdx_ref 金标准）"
step rxvec_gen   python3 -I scripts/gen_rx_vec.py "$OUT/rxvec"

echo "---- [rxicrm_tb] IC/RM 穷举对拍（afdx_ref.ic_step/rm_step 全矩阵）"
if "$IVERILOG" -g2001 -Wall -o "$LOGS/tb_rxicrm.vvp" tb/tb_rxicrm.v rtl/rx/rx_integrity.v        rtl/rx/rx_redundancy.v rtl/lib/sdp_ram.v        > "$LOGS/rxicrm_compile.log" 2>&1    && "$VVP" "$LOGS/tb_rxicrm.vvp" +VEC_DIR="$OUT/rxvec" > "$LOGS/rxicrm_tb.log" 2>&1    && grep -q "\[RXICRM-TB\] PASS" "$LOGS/rxicrm_tb.log"; then
    PASS=$((PASS+1)); grep -aE "\[RXICRM-TB\]" "$LOGS/rxicrm_tb.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -6 "$LOGS/rxicrm_compile.log" "$LOGS/rxicrm_tb.log" 2>/dev/null | sed 's/^/       /'
fi

echo "---- [rxparser_tb] rx_parser 单元对拍（12 帧含 6 类异常）"
if "$IVERILOG" -g2001 -Wall -o "$LOGS/tb_rxparser.vvp" tb/tb_rxparser.v rtl/rx/rx_parser.v        rtl/lib/csum16.v        > "$LOGS/rxparser_compile.log" 2>&1    && "$VVP" "$LOGS/tb_rxparser.vvp" +VEC_DIR="$OUT/rxvec" > "$LOGS/rxparser_tb.log" 2>&1    && grep -q "\[RXP-TB\] PASS" "$LOGS/rxparser_tb.log"; then
    PASS=$((PASS+1)); grep -aE "\[RXP-TB\] PASS" "$LOGS/rxparser_tb.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -6 "$LOGS/rxparser_compile.log" "$LOGS/rxparser_tb.log" 2>/dev/null | sed 's/^/       /'
fi

echo "---- [rxasm_tb] rx_reassembly 单元对拍（8 场景：在序/乱序/缺片/超时/重叠）"
if "$IVERILOG" -g2001 -Wall -o "$LOGS/tb_rxasm.vvp" tb/tb_rxasm.v rtl/rx/rx_reassembly.v        rtl/lib/sdp_ram.v        > "$LOGS/rxasm_compile.log" 2>&1    && "$VVP" "$LOGS/tb_rxasm.vvp" +VEC_DIR="$OUT/rxvec" > "$LOGS/rxasm_tb.log" 2>&1    && grep -q "\[RXASM-TB\] PASS" "$LOGS/rxasm_tb.log"; then
    PASS=$((PASS+1)); grep -aE "\[RXASM-TB\] PASS" "$LOGS/rxasm_tb.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -6 "$LOGS/rxasm_compile.log" "$LOGS/rxasm_tb.log" 2>/dev/null | sed 's/^/       /'
fi

echo "---- [rxlookup_tb] rx_vl_lookup 4096 全表配置/读回/查表扫"
if "$IVERILOG" -g2001 -Wall -o "$LOGS/tb_rxlookup.vvp" tb/tb_rxlookup.v rtl/rx/rx_vl_lookup.v        rtl/lib/sdp_ram.v        > "$LOGS/rxlookup_compile.log" 2>&1    && "$VVP" "$LOGS/tb_rxlookup.vvp" > "$LOGS/rxlookup_tb.log" 2>&1    && grep -q "\[RXLUT-TB\] PASS" "$LOGS/rxlookup_tb.log"; then
    PASS=$((PASS+1)); grep -aE "\[RXLUT-TB\] (cfg|readback|lookup|PASS)" "$LOGS/rxlookup_tb.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -6 "$LOGS/rxlookup_compile.log" "$LOGS/rxlookup_tb.log" 2>/dev/null | sed 's/^/       /'
fi

echo "---- [fix2_tb] REV-002-04 定向：FIFO 满与 ready 同拍恢复（K 扫描）"
# FIX-FREEZE 顺手修复（HEAD 既有破损，非本批引入）：T8 MAC IP 换装（1226399）
# 后 core 固定例化 mac_ip_top，下列含 core 四段（fix2/smoke/e2e/e2erx）的
# iverilog 清单漏跟——此处按已验收配方 run_delivery_e2e.sh（VERDICT-015 验收②）
# 换装：mac_ip_top+行为替身 mac_ip_behav+虚拟 PHY mdio_phy_sim 顶替
# mac_tx+mac_port_if；txframe 段不含 core、直编 mac_tx 自洽，不动。
FIX2_SRCS="tb/tb_fix2.v rtl/AFDX_top.v rtl/core/afdx_es_core.v rtl/lib/buf_mgr.v rtl/lib/sdp_ram.v rtl/lib/sync_fifo.v rtl/lib/rr_arbiter.v rtl/lib/tick_gen.v rtl/lib/crc32_d8.v rtl/lib/csum16.v rtl/core/cfg_bus.v rtl/core/ram_init.v rtl/core/timebase.v rtl/core/host_arb.v rtl/core/stats.v rtl/tx/tx_vl_table.v rtl/tx/tx_regulator.v rtl/tx/tx_scheduler.v rtl/tx/tx_sn_gen.v rtl/tx/tx_frame_builder.v rtl/mac/mac_ip_top.v rtl/mac/mac_ip_behav.v rtl/mac/mdio_phy_sim.v rtl/rx/rx_path.v rtl/rx/rx_parser.v rtl/rx/rx_vl_lookup.v rtl/rx/rx_integrity.v rtl/rx/rx_redundancy.v rtl/rx/rx_reassembly.v rtl/mac/mac_rx.v"
if "$IVERILOG" -g2001 -Wall -I tb -o "$LOGS/tb_fix2.vvp" $FIX2_SRCS \
       > "$LOGS/fix2_compile.log" 2>&1 \
   && "$VVP" "$LOGS/tb_fix2.vvp" > "$LOGS/fix2_tb.log" 2>&1 \
   && grep -q "\[FIX2-TB\] PASS" "$LOGS/fix2_tb.log"; then
    PASS=$((PASS+1)); grep -E "\[FIX2-TB\] (PASS|race|FAIL)" "$LOGS/fix2_tb.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -8 "$LOGS/fix2_compile.log" "$LOGS/fix2_tb.log" 2>/dev/null | sed 's/^/       /'
fi

echo "---- [smoke_sim] iverilog + vvp"
SMOKESRCS="tb/tb_afdx_smoke.v rtl/AFDX_top.v rtl/core/afdx_es_core.v rtl/lib/buf_mgr.v rtl/lib/sdp_ram.v rtl/lib/sync_fifo.v rtl/lib/rr_arbiter.v rtl/lib/tick_gen.v rtl/lib/crc32_d8.v rtl/lib/csum16.v rtl/core/cfg_bus.v rtl/core/ram_init.v rtl/core/timebase.v rtl/core/host_arb.v rtl/core/stats.v rtl/tx/tx_vl_table.v rtl/tx/tx_regulator.v rtl/tx/tx_scheduler.v rtl/tx/tx_sn_gen.v rtl/tx/tx_frame_builder.v rtl/mac/mac_ip_top.v rtl/mac/mac_ip_behav.v rtl/mac/mdio_phy_sim.v rtl/rx/rx_path.v rtl/rx/rx_parser.v rtl/rx/rx_vl_lookup.v rtl/rx/rx_integrity.v rtl/rx/rx_redundancy.v rtl/rx/rx_reassembly.v rtl/mac/mac_rx.v"
if "$IVERILOG" -g2001 -Wall -I tb -o "$LOGS/tb_smoke.vvp" $SMOKESRCS \
       > "$LOGS/smoke_compile.log" 2>&1 \
   && "$VVP" "$LOGS/tb_smoke.vvp" +CASE_DIR="$OUT/$CASE" > "$LOGS/smoke_sim.log" 2>&1 \
   && grep -q "\[TB\] PASS" "$LOGS/smoke_sim.log"; then
    PASS=$((PASS+1)); grep -E "\[TB\]" "$LOGS/smoke_sim.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -8 "$LOGS/smoke_compile.log" "$LOGS/smoke_sim.log" 2>/dev/null | sed 's/^/       /'
fi

echo "---- [tx1_gen] tx1 案例数据生成（txframe 消费；docfix 第 10 项：此步前 regress 不自生 tx1，新克隆 txframe 必挂）"
step tx1_gen python3 -I scripts/gen_stim.py tx1 "$OUT/tx1"

echo "---- [txframe_sim] 发送通路全帧仿真 (tx1)"
TX_SRCS="tb/tb_txframe.v rtl/lib/buf_mgr.v rtl/lib/sdp_ram.v rtl/lib/rr_arbiter.v rtl/lib/tick_gen.v rtl/lib/crc32_d8.v rtl/lib/sync_fifo.v rtl/lib/csum16.v rtl/core/cfg_bus.v rtl/core/ram_init.v rtl/core/timebase.v rtl/core/host_arb.v rtl/core/stats.v rtl/tx/tx_vl_table.v rtl/tx/tx_regulator.v rtl/tx/tx_scheduler.v rtl/tx/tx_sn_gen.v rtl/tx/tx_frame_builder.v rtl/mac/mac_tx.v rtl/mac/mac_port_if.v"
if "$IVERILOG" -g2001 -Wall -I tb -o "$LOGS/tb_txframe.vvp" $TX_SRCS \
       > "$LOGS/txframe_compile.log" 2>&1 \
   && "$VVP" "$LOGS/tb_txframe.vvp" +CASE_DIR="$OUT/tx1" > "$LOGS/txframe_sim.log" 2>&1 \
   && grep -q "\[TX-TB\] PASS" "$LOGS/txframe_sim.log" \
   && PYTHONIOENCODING=utf-8 python3 -I scripts/check_output.py "$OUT/tx1" > "$LOGS/check_output.log" 2>&1; then
    PASS=$((PASS+1)); grep -aE "\[TX-TB\]|\[CHECK\]" "$LOGS/txframe_sim.log" "$LOGS/check_output.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -6 "$LOGS/txframe_compile.log" "$LOGS/txframe_sim.log" "$LOGS/check_output.log" 2>/dev/null | sed 's/^/       /'
fi

echo "---- [e2e_sim] 端到端仿真 (AFDX_top 应用口, tx1: a 案 CASE_DIR 固定 tx1)"
E2ESRCS="tb/tb_afdx_tx.v rtl/AFDX_top.v rtl/core/afdx_es_core.v rtl/lib/buf_mgr.v rtl/lib/sdp_ram.v rtl/lib/sync_fifo.v rtl/lib/rr_arbiter.v rtl/lib/tick_gen.v rtl/lib/crc32_d8.v rtl/lib/csum16.v rtl/core/cfg_bus.v rtl/core/ram_init.v rtl/core/timebase.v rtl/core/host_arb.v rtl/core/stats.v rtl/tx/tx_vl_table.v rtl/tx/tx_regulator.v rtl/tx/tx_scheduler.v rtl/tx/tx_sn_gen.v rtl/tx/tx_frame_builder.v rtl/mac/mac_ip_top.v rtl/mac/mac_ip_behav.v rtl/mac/mdio_phy_sim.v rtl/rx/rx_path.v rtl/rx/rx_parser.v rtl/rx/rx_vl_lookup.v rtl/rx/rx_integrity.v rtl/rx/rx_redundancy.v rtl/rx/rx_reassembly.v rtl/mac/mac_rx.v"
if "$IVERILOG" -g2001 -Wall -I tb -o "$LOGS/tb_afdx_tx.vvp" $E2ESRCS \
       > "$LOGS/e2e_compile.log" 2>&1 \
   && "$VVP" "$LOGS/tb_afdx_tx.vvp" +CASE_DIR="$OUT/tx1" > "$LOGS/e2e_sim.log" 2>&1 \
   && grep -q "\[E2E-TB\] PASS" "$LOGS/e2e_sim.log" \
   && PYTHONIOENCODING=utf-8 python3 -I scripts/check_output.py "$OUT/tx1" > "$LOGS/check_output.log" 2>&1; then
    PASS=$((PASS+1)); grep -aE "\[E2E-TB\]|\[CHECK\]" "$LOGS/e2e_sim.log" "$LOGS/check_output.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -6 "$LOGS/e2e_compile.log" "$LOGS/e2e_sim.log" "$LOGS/check_output.log" 2>/dev/null | sed 's/^/       /'
fi


echo "---- [rxpipe_tb] 接收通路流水级用例（rx1~rx6：去重/IC/RM/SkewMax/单网/错误注入/分片健壮性）"
RXSRCS="tb/tb_rxpipe.v rtl/rx/rx_path.v rtl/rx/rx_parser.v rtl/rx/rx_vl_lookup.v rtl/rx/rx_integrity.v rtl/rx/rx_redundancy.v rtl/rx/rx_reassembly.v rtl/mac/mac_rx.v rtl/lib/sdp_ram.v rtl/lib/sync_fifo.v rtl/lib/tick_gen.v rtl/lib/crc32_d8.v rtl/lib/csum16.v"
# REV-F-02（冻结终审）：运行前清旧 vvp 与旧 case 目录——编译/激励生成失败时
# 不得复用旧产物继续跑并计入 PASS。
rm -f "$LOGS/tb_rxpipe.vvp"
rm -rf "$OUT/rx1" "$OUT/rx2" "$OUT/rx3" "$OUT/rx4" "$OUT/rx5" "$OUT/rx6"
if "$IVERILOG" -g2001 -Wall -I tb -o "$LOGS/tb_rxpipe.vvp" $RXSRCS \
       > "$LOGS/rxpipe_compile.log" 2>&1; then
    PASS=$((PASS+1)); echo "     PASS (compile)"
    for rc in rx1 rx2 rx3 rx4 rx5 rx6; do
        if python3 -I scripts/gen_rx_cases.py "$rc" "$OUT/$rc" > "$LOGS/rxgen_$rc.log" 2>&1 \
           && "$VVP" "$LOGS/tb_rxpipe.vvp" +CASE_DIR="$OUT/$rc" > "$LOGS/rxpipe_$rc.log" 2>&1 \
           && grep -q "\[RXPIPE-TB\] PASS" "$LOGS/rxpipe_$rc.log"; then
            PASS=$((PASS+1)); grep -aE "\[RXPIPE-TB\] (delivered|latency|PASS)" "$LOGS/rxpipe_$rc.log" | sed 's/^/       /'
            echo "     PASS ($rc)"
        else
            FAIL=$((FAIL+1)); echo "     FAIL ($rc) —— $LOGS/rxgen_$rc.log / $LOGS/rxpipe_$rc.log 尾部："
            tail -5 "$LOGS/rxgen_$rc.log" "$LOGS/rxpipe_$rc.log" 2>/dev/null | sed 's/^/       /'
        fi
    done
else
    FAIL=$((FAIL+1)); echo "     FAIL (compile) —— $LOGS/rxpipe_compile.log 尾部："
    tail -8 "$LOGS/rxpipe_compile.log" | sed 's/^/       /'
fi

echo "---- [e2e_loopback] 端到端回环（TX→GMII 回灌→RX→应用口对称 + 时延实测）"
# P3 修复（PROBE-001）：tb_afdx_rx 的回环驱动块以 `ifndef AFDX_MAC_LOOPBACK 保护，
# 但该宏在 rtl/core/afdx_es_core.v 中定义、编译序在 tb 之后，tb 侧看不见 →
# tb 与核内片内回环同时驱动 gmii_rx_*，RX 收到撕裂流，delivered 0（实测
# e2erx_sim.log：delivered 0 envelopes / FAIL 1 errors @99.6ms）。编译期显式
# -DAFDX_MAC_LOOPBACK = 生产口径（回环归核内 mac_ip_top），与 tb 头注一致。
E2ESRCS="tb/tb_afdx_rx.v rtl/AFDX_top.v rtl/core/afdx_es_core.v rtl/lib/buf_mgr.v rtl/lib/sdp_ram.v rtl/lib/sync_fifo.v rtl/lib/rr_arbiter.v rtl/lib/tick_gen.v rtl/lib/crc32_d8.v rtl/lib/csum16.v rtl/core/cfg_bus.v rtl/core/ram_init.v rtl/core/timebase.v rtl/core/host_arb.v rtl/core/stats.v rtl/tx/tx_vl_table.v rtl/tx/tx_regulator.v rtl/tx/tx_scheduler.v rtl/tx/tx_sn_gen.v rtl/tx/tx_frame_builder.v rtl/mac/mac_ip_top.v rtl/mac/mac_ip_behav.v rtl/mac/mdio_phy_sim.v rtl/mac/mac_rx.v rtl/rx/rx_path.v rtl/rx/rx_parser.v rtl/rx/rx_vl_lookup.v rtl/rx/rx_integrity.v rtl/rx/rx_redundancy.v rtl/rx/rx_reassembly.v"
if "$IVERILOG" -g2001 -Wall -I tb -DAFDX_MAC_LOOPBACK -o "$LOGS/tb_afdx_rx.vvp" $E2ESRCS        > "$LOGS/e2erx_compile.log" 2>&1    && "$VVP" "$LOGS/tb_afdx_rx.vvp" > "$LOGS/e2erx_sim.log" 2>&1    && grep -q "\[E2E-RX-TB\] PASS" "$LOGS/e2erx_sim.log"; then
    PASS=$((PASS+1)); grep -aE "\[E2E-RX-TB\] (delivered|latency|e2e|PASS)" "$LOGS/e2erx_sim.log" | sed 's/^/       /'
    echo "     PASS"
else
    FAIL=$((FAIL+1)); echo "     FAIL —— 日志尾部："
    tail -6 "$LOGS/e2erx_compile.log" "$LOGS/e2erx_sim.log" 2>/dev/null | sed 's/^/       /'
fi

echo "===="
echo "regress: $PASS passed, $FAIL failed"
exit "$FAIL"
