//=============================================================================
// tb_afdx_smoke.v — T5 冒烟用例
// 目的：验证文件式协同仿真基础设施——激励/金标准文件加载、应用口驱动任务、
//       金标准帧头逐字段检查，并建立 Python↔Verilog 文件格式契约。
// 说明：当前 rtl/AFDX_top.v 为外壳桩（rx 侧无输出），端到端数据检查待 T6/T7
//       核心落地后在同一 tb 框架上扩展（tb 结构已按其预留）。
// 用例内容（scripts/gen_stim.py smoke）：3 条消息（100B 中间帧 / 1471B 最大帧 /
//       1B 最小帧），多 VL 并发，A/B 双网金标准帧各 3 条。
//=============================================================================

`timescale 1ns/1ps

module tb_afdx_smoke;

    //-------------------------------------------------------------------------
    // DUT 与时钟
    //-------------------------------------------------------------------------
    reg         clk;
    reg         reset;
    reg  [7:0]  tx_data;
    wire [7:0]  rx_data;
    reg         tx_valid;
    wire        rx_valid;
    wire        tx_ready;          // DUT 输出（恒 1，VERDICT-002-1 ready 回正）
    reg         rx_ready;          // DUT 输入（应用侧就绪，恒 1；可不使用）

    AFDX_top u_dut (
        .clk      (clk),
        .reset    (reset),
        .tx_data  (tx_data),
        .rx_data  (rx_data),
        .tx_valid (tx_valid),
        .rx_valid (rx_valid),
        .tx_ready (tx_ready),
        .rx_ready (rx_ready),
        // cfg 主机配置接口 tie-off（本 tb 不配置；VERDICT-015 端口化后防 X 泄入）
        .cfg_addr (20'h0),
        .cfg_wdata(32'h0),
        .cfg_we   (1'b0),
        .cfg_re   (1'b0),
        .cfg_rdata(),
        .cfg_rvalid()
    );

    parameter CLK_HALF = 4;            // 125 MHz（GMII 域，doc/02 v1 时钟方案）
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    //-------------------------------------------------------------------------
    // tb 公共任务与存储（afdx_if.v）
    //-------------------------------------------------------------------------
    integer    tx_n, gold_n, ml_n, fl_n;
    integer    i, base_tx, base_gold;
    reg [1023:0] casedir;
    reg [2047:0] path;

    `include "afdx_if.v"

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    initial begin : main
        errors  = 0;
        tx_data = 0; tx_valid = 0; rx_ready = 1'b1;

        // 用例目录由 plusargs 传入（默认 smoke）
        if (!$value$plusargs("CASE_DIR=%s", casedir))
            casedir = "sim/tb_out/smoke";

        // 1) 加载激励与金标准（Python gen_stim.py 产物）
        path = {casedir, "/msg_tx.hex"};
        load_msg_file(path, tx_n);
        path = {casedir, "/msg_lens.txt"};
        load_msg_lens(path, ml_n);
        path = {casedir, "/golden_A.hex"};
        load_gold_file(path, gold_n);
        path = {casedir, "/frame_A.lens"};
        load_gold_lens(path, fl_n);

        $display("[TB] loaded: tx_bytes=%0d msgs=%0d gold_bytes=%0d frames=%0d",
                 tx_n, ml_n, gold_n, fl_n);
        if (ml_n == 0 || fl_n == 0 || tx_n == 0 || gold_n == 0) begin
            $display("[TB] FAIL: 激励/金标准文件为空或缺失");
            errors = errors + 1;
        end

        // 2) 金标准帧头逐字段检查（A 网：IF_ID=0x20）
        base_gold = 0;
        for (i = 0; i < fl_n; i = i + 1) begin
            check_frame_header(base_gold, {gold_mem[base_gold+4], gold_mem[base_gold+5]},
                               8'h0A, 8'h17, 8'h20, gold_len[i]);
            base_gold = base_gold + gold_len[i];
        end
        $display("[TB] golden header check done: %0d frames, errors=%0d", fl_n, errors);

        // 3) 复位后向应用口驱动全部消息（当前外壳不响应；核心落地后接检查器）
        reset = 1'b1;
        repeat (4) @(posedge clk);
        reset = 1'b0;

        base_tx = 0;
        for (i = 0; i < ml_n; i = i + 1) begin
            drive_app_msg(base_tx, msg_len[i]);
            base_tx = base_tx + msg_len[i];
        end
        $display("[TB] drove %0d application messages", ml_n);

        // 4) 结论
        if (errors == 0)
            $display("[TB] PASS");
        else
            $display("[TB] FAIL: %0d errors", errors);
        $finish;
    end

    //-------------------------------------------------------------------------
    // 超时看门狗
    //-------------------------------------------------------------------------
    initial begin
        #10_000_000;                       // 10 ms 仿真时间上限
        $display("[TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
