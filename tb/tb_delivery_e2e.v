//=============================================================================
// tb_delivery_e2e.v — VERDICT-015 验收②「交付形态 e2e」
// 纯顶层引脚驱动：AFDX_top 全部端口（应用口 + cfg 主机配置接口），零层级
// 引用（读/写均无）。链路：cfg 引脚配表 → 应用口信封注入 → 核内 TX 引擎 →
// mac_ip_top（iverilog 链行为替身 mac_ip_behav 的 Tri_Mode_Ethernet_Mac_0
// 包装）→ 片内 1 拍回环 → mac_rx ×2 → parser/LUT/IC/RM/rx_reassembly →
// 应用口交付（rx_data/rx_valid）→ 信封对称校验。
// 覆盖：VL10 100B（单帧）/ VL11 2000B（分片→重组）/ VL12 1B（最短），各 2 条。
// 判定：交付 6/6，逐消息 VLID/长度/逐字节 pattern（(k*7+vlid)&0xFF）全等。
// 编译（交付口径，与 sim/afdx_elinx 综合同源宏）：见 scripts/run_delivery_e2e.sh
//=============================================================================
`timescale 1ns/1ps

module tb_delivery_e2e;

    parameter CLK_HALF = 4;                 // 125 MHz（doc/02 v1）
    // ram_init 上电装载等待（确定性拍数，无层级探测；旧 tb guard<50000 口径
    // 的 2 倍裕量）
    parameter integer INIT_WAIT_CYC = 100000;
    // 注入完毕后交付看门狗（仿真时间）
    parameter integer WD_NS = 60_000_000;   // 60ms

    reg         clk;
    reg         reset;
    reg  [7:0]  tx_data;
    wire [7:0]  rx_data;
    reg         tx_valid;
    wire        rx_valid;
    wire        tx_ready;
    reg         rx_ready;
    reg  [19:0] cfg_addr;
    reg  [31:0] cfg_wdata;
    reg         cfg_we, cfg_re;
    wire [31:0] cfg_rdata;
    wire        cfg_rvalid;

    integer errors;
    initial errors = 0;

    AFDX_top u_dut (
        .clk      (clk),
        .reset    (reset),
        .tx_data  (tx_data),
        .rx_data  (rx_data),
        .tx_valid (tx_valid),
        .rx_valid (rx_valid),
        .tx_ready (tx_ready),
        .rx_ready (rx_ready),
        .cfg_addr (cfg_addr),
        .cfg_wdata(cfg_wdata),
        .cfg_we   (cfg_we),
        .cfg_re   (cfg_re),
        .cfg_rdata(cfg_rdata),
        .cfg_rvalid(cfg_rvalid)
    );

    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    //-------------------------------------------------------------------------
    // cfg 写任务（顶层引脚，cfg_bus 同步采样口径同各 tb）
    //-------------------------------------------------------------------------
    task cfg_write;
        input [19:0] a;
        input [31:0] d;
        begin
            @(negedge clk);
            cfg_addr  = a;
            cfg_wdata = d;
            cfg_we    = 1'b1;
            @(negedge clk);
            cfg_we    = 1'b0;
        end
    endtask

    //-------------------------------------------------------------------------
    // 配置：3 VL（idx 10/11/12 = vlid 0x000A/B/C），TX 表 + RX 表 + 全局
    // （序列与 tb_afdx_rx.do_config 同口径：W0 最后写=配置即到期）
    //-------------------------------------------------------------------------
    task do_config;
        integer i;
        reg [15:0] vlid_q;
        begin
            for (i = 0; i < 3; i = i + 1) begin
                vlid_q = 16'h000A + i;
                cfg_write(20'h01000 + (10 + i) * 8 + 4, 32'h0);          // W4 相位
                cfg_write(20'h01000 + (10 + i) * 8 + 1, 32'h000005EE);   // W1 smax=1518
                cfg_write(20'h01000 + (10 + i) * 8 + 2, 32'h0000000A + i); // W2 vlid
                cfg_write(20'h01000 + (10 + i) * 8 + 3, 32'h3);          // W3 mask=AB
                cfg_write(20'h01000 + (10 + i) * 8 + 5, 32'h00001389);   // W5 sport=5001
                cfg_write(20'h01000 + (10 + i) * 8 + 6, 32'h0000138A);   // W6 dport=5002
                cfg_write(20'h01000 + (10 + i) * 8 + 0,
                          (i == 2) ? 32'h8000_0002 : 32'h8000_0001);     // W0 en+BAG 1/1/2ms
                cfg_write(20'h10000 + (10 + i) * 2,
                          {11'b0, 2'b11, 1'b1, 1'b1, 1'b1, vlid_q});     // RX W0
                cfg_write(20'h10000 + (10 + i) * 2 + 1,
                          {5'b0, 11'd1518, 16'd5000});                   // RX W1
            end
            cfg_write(20'h00001, 32'h00000A17);   // UID
            cfg_write(20'h00002, 32'h0A000205);   // SRC_IP
            cfg_write(20'h00003, 32'h1);          // CTRL.tx_en 开闸
        end
    endtask

    //-------------------------------------------------------------------------
    // 注入：应用口信封（VLID 2B + pattern 载荷）
    //-------------------------------------------------------------------------
    task drive_envelope;
        input [15:0] vlid;
        input integer plen;
        integer k;
        begin
            @(negedge clk);
            tx_valid = 1'b1; tx_data = vlid[15:8];
            @(negedge clk);
            tx_data = vlid[7:0];
            for (k = 0; k < plen; k = k + 1) begin
                @(negedge clk);
                tx_data = (k * 7 + vlid[7:0]) & 8'hFF;
            end
            @(negedge clk);
            tx_valid = 1'b0;
            @(negedge clk);                    // 信封间 ≥1 拍间隙
        end
    endtask

    //-------------------------------------------------------------------------
    // 交付期望表（按 VLID 查找，每 VL 2 条）
    //-------------------------------------------------------------------------
    reg [15:0] exp_vlid  [0:2];
    integer    exp_plen  [0:2];
    integer    got_cnt   [0:2];
    integer    rx_msgs;
    reg        rx_in_msg;
    integer    rx_k;
    reg [15:0] rx_v;
    reg        pat_bad;

    task note_msg;
        input [15:0] vlid;
        input integer plen;
        input integer bad;
        integer vi;
        begin
            vi = vlid - 16'h000A;
            rx_msgs = rx_msgs + 1;
            if (vi < 0 || vi > 2) begin
                $display("[DELIV-TB] FAIL: 未知 VLID 交付 %0h", vlid);
                errors = errors + 1;
            end else if (bad) begin
                errors = errors + 1;
            end else if (plen !== exp_plen[vi]) begin
                $display("[DELIV-TB] FAIL: vlid=%0h len=%0d exp=%0d",
                         vlid, plen, exp_plen[vi]);
                errors = errors + 1;
            end else begin
                got_cnt[vi] = got_cnt[vi] + 1;
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 接收进程：rx_valid 字节流 → 信封边界（valid 落=一条）→ VLID/长度/逐字节校验
    //（纯阻塞赋值监控进程；字节比较显式括号——Verilog 等号优先级高于 &）
    //-------------------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            rx_in_msg = 1'b0; rx_k = 0; rx_v = 16'h0; pat_bad = 1'b0;
        end else begin
            if (rx_valid) begin
                if (!rx_in_msg) begin
                    rx_in_msg = 1'b1;
                    rx_k      = 0;
                    rx_v      = {rx_data, 8'h0};        // VLID 高字节
                    pat_bad   = 1'b0;
                end else if (rx_k == 0) begin
                    rx_v[7:0] = rx_data;                // VLID 低字节
                    rx_k      = 1;
                end else begin
                    if ((((((rx_k - 1) * 7) + rx_v[7:0]) & 8'hFF)) !== rx_data) begin
                        if (!pat_bad)
                            $display("[DELIV-TB] FAIL: vlid=%0h 字节 %0d 不符 got=%02h exp=%02h",
                                     rx_v, rx_k - 1, rx_data,
                                     ((((rx_k - 1) * 7) + rx_v[7:0]) & 8'hFF));
                        pat_bad = 1'b1;
                    end
                    rx_k = rx_k + 1;
                end
            end else if (rx_in_msg) begin
                rx_in_msg = 1'b0;
                note_msg(rx_v, rx_k - 1, pat_bad);       // rx_k-1=载荷字节数
            end
        end
    end

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    integer i, j, wait_cyc;
    initial begin : main
        reset = 1'b1; tx_valid = 1'b0; tx_data = 8'h0; rx_ready = 1'b1;
        cfg_addr = 20'h0; cfg_wdata = 32'h0; cfg_we = 1'b0; cfg_re = 1'b0;
        exp_vlid[0] = 16'h000A; exp_plen[0] = 100;
        exp_vlid[1] = 16'h000B; exp_plen[1] = 2000;
        exp_vlid[2] = 16'h000C; exp_plen[2] = 1;
        got_cnt[0] = 0; got_cnt[1] = 0; got_cnt[2] = 0;
        rx_msgs = 0; rx_in_msg = 0; rx_k = 0; rx_v = 16'h0;
        pat_bad = 0;

        // 复位 100 拍 → 释放
        repeat (100) @(posedge clk);
        @(negedge clk); reset = 1'b0;

        // ram_init 上电装载确定性等待（无层级探测）
        for (wait_cyc = 0; wait_cyc < INIT_WAIT_CYC; wait_cyc = wait_cyc + 1)
            @(posedge clk);

        do_config;
        $display("[DELIV-TB] cfg done (3VL + globals + tx_en), t=%0t", $time);

        // 两轮注入：每轮 3 VL 各 1 条（100B / 2000B / 1B）
        for (j = 0; j < 2; j = j + 1) begin
            drive_envelope(16'h000A, 100);
            drive_envelope(16'h000B, 2000);
            drive_envelope(16'h000C, 1);
        end
        $display("[DELIV-TB] injected 6 envelopes (3VL x2: 100B/2000B/1B), t=%0t", $time);

        // 交付等待（上限 WD_NS）
        wait_cyc = 0;
        while (rx_msgs < 6 && wait_cyc < WD_NS / (2 * CLK_HALF * 2))
            @(posedge clk);

        #1000;
        if (rx_msgs == 6 && got_cnt[0] == 2 && got_cnt[1] == 2 && got_cnt[2] == 2
            && errors == 0)
            $display("[DELIV-TB] PASS (delivered 6/6: VL10 x%0d VL11 x%0d VL12 x%0d, payload byte-exact)",
                     got_cnt[0], got_cnt[1], got_cnt[2]);
        else begin
            $display("[DELIV-TB] FAIL: delivered=%0d/6 (VL10 x%0d VL11 x%0d VL12 x%0d) errors=%0d",
                     rx_msgs, got_cnt[0], got_cnt[1], got_cnt[2], errors);
        end
        $finish;
    end

    // 看门狗
    initial begin
        #WD_NS;
        $display("[DELIV-TB] FAIL: watchdog timeout (delivered=%0d/6)", rx_msgs);
        $finish;
    end

endmodule
