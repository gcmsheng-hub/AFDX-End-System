//=============================================================================
// tb_delivery_short.v — VERDICT-016 补令：门级仿真缩范围用例（探针集）
// 范围（调度裁定：官方「主要业务」口径）：smoke(1B) + 100B(单帧) + 2000B
// （分片→重组）各 1 信封。纯 AFDX_top 顶层引脚驱动，零层级引用。
// 里程碑 $display 带 $time（cfg done / inject / 每条交付），供墙钟/模拟时间比
// 外推记录（RPT-T8-010）。
//=============================================================================
`timescale 1ns/1ps

module tb_delivery_short;

    parameter CLK_HALF = 4;
    parameter integer INIT_WAIT_CYC = 100000;
    parameter integer WD_NS = 120_000_000;   // 120ms 看门狗（缩范围跨窗长跑）

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

    task do_config;
        integer i;
        reg [15:0] vlid_q;
        begin
            for (i = 0; i < 3; i = i + 1) begin
                vlid_q = 16'h000A + i;
                cfg_write(20'h01000 + (10 + i) * 8 + 4, 32'h0);
                cfg_write(20'h01000 + (10 + i) * 8 + 1, 32'h000005EE);
                cfg_write(20'h01000 + (10 + i) * 8 + 2, 32'h0000000A + i);
                cfg_write(20'h01000 + (10 + i) * 8 + 3, 32'h3);
                cfg_write(20'h01000 + (10 + i) * 8 + 5, 32'h00001389);
                cfg_write(20'h01000 + (10 + i) * 8 + 6, 32'h0000138A);
                cfg_write(20'h01000 + (10 + i) * 8 + 0,
                          (i == 2) ? 32'h8000_0002 : 32'h8000_0001);
                cfg_write(20'h10000 + (10 + i) * 2,
                          {11'b0, 2'b11, 1'b1, 1'b1, 1'b1, vlid_q});
                cfg_write(20'h10000 + (10 + i) * 2 + 1,
                          {5'b0, 11'd1518, 16'd5000});
            end
            cfg_write(20'h00001, 32'h00000A17);
            cfg_write(20'h00002, 32'h0A000205);
            cfg_write(20'h00003, 32'h1);
        end
    endtask

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
            @(negedge clk);
        end
    endtask

    reg [15:0] exp_vlid [0:2];
    integer    exp_plen [0:2];
    integer    got_cnt  [0:2];
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
            $display("[SHORT-TB] delivered #%0d vlid=%0h len=%0d bad=%0d t=%0t",
                     rx_msgs, vlid, plen, bad, $time);
            if (vi < 0 || vi > 2) begin
                $display("[SHORT-TB] FAIL: 未知 VLID 交付 %0h", vlid);
                errors = errors + 1;
            end else if (bad) begin
                errors = errors + 1;
            end else if (plen !== exp_plen[vi]) begin
                $display("[SHORT-TB] FAIL: vlid=%0h len=%0d exp=%0d",
                         vlid, plen, exp_plen[vi]);
                errors = errors + 1;
            end else begin
                got_cnt[vi] = got_cnt[vi] + 1;
            end
        end
    endtask

    always @(posedge clk) begin
        if (reset) begin
            rx_in_msg = 1'b0; rx_k = 0; rx_v = 16'h0; pat_bad = 1'b0;
        end else begin
            if (rx_valid) begin
                if (!rx_in_msg) begin
                    rx_in_msg = 1'b1;
                    rx_k      = 0;
                    rx_v      = {rx_data, 8'h0};
                    pat_bad   = 1'b0;
                end else if (rx_k == 0) begin
                    rx_v[7:0] = rx_data;
                    rx_k      = 1;
                end else begin
                    if ((((((rx_k - 1) * 7) + rx_v[7:0]) & 8'hFF)) !== rx_data) begin
                        if (!pat_bad)
                            $display("[SHORT-TB] FAIL: vlid=%0h 字节 %0d 不符 got=%02h exp=%02h",
                                     rx_v, rx_k - 1, rx_data,
                                     ((((rx_k - 1) * 7) + rx_v[7:0]) & 8'hFF));
                        pat_bad = 1'b1;
                    end
                    rx_k = rx_k + 1;
                end
            end else if (rx_in_msg) begin
                rx_in_msg = 1'b0;
                note_msg(rx_v, rx_k - 1, pat_bad);
            end
        end
    end

    integer i, wait_cyc;
    initial begin : main
        reset = 1'b1; tx_valid = 1'b0; tx_data = 8'h0; rx_ready = 1'b1;
        cfg_addr = 20'h0; cfg_wdata = 32'h0; cfg_we = 1'b0; cfg_re = 1'b0;
        exp_vlid[0] = 16'h000A; exp_plen[0] = 100;
        exp_vlid[1] = 16'h000B; exp_plen[1] = 2000;
        exp_vlid[2] = 16'h000C; exp_plen[2] = 1;
        got_cnt[0] = 0; got_cnt[1] = 0; got_cnt[2] = 0;
        rx_msgs = 0; rx_in_msg = 0; rx_k = 0; rx_v = 16'h0; pat_bad = 0;

        repeat (100) @(posedge clk);
        @(negedge clk); reset = 1'b0;
        for (wait_cyc = 0; wait_cyc < INIT_WAIT_CYC; wait_cyc = wait_cyc + 1)
            @(posedge clk);

        do_config;
        $display("[SHORT-TB] MILESTONE cfg-done 3VL+globals+tx_en t=%0t", $time);

        // 探针集：smoke(1B) → 100B → 2000B（分片），各 1 信封
        drive_envelope(16'h000C, 1);
        $display("[SHORT-TB] MILESTONE injected smoke(1B,VL12) t=%0t", $time);
        drive_envelope(16'h000A, 100);
        $display("[SHORT-TB] MILESTONE injected 100B(VL10) t=%0t", $time);
        drive_envelope(16'h000B, 2000);
        $display("[SHORT-TB] MILESTONE injected 2000B(VL11,fragmented) t=%0t", $time);

        wait_cyc = 0;
        while (rx_msgs < 3 && wait_cyc < WD_NS / (2 * CLK_HALF * 2))
            @(posedge clk);

        #1000;
        if (rx_msgs == 3 && got_cnt[0] == 1 && got_cnt[1] == 1 && got_cnt[2] == 1
            && errors == 0)
            $display("[SHORT-TB] PASS (delivered 3/3: smoke(1B) x1, 100B x1, 2000B(fragmented) x1, payload byte-exact) t=%0t",
                     $time);
        else
            $display("[SHORT-TB] FAIL: delivered=%0d/3 (100B x%0d, 2000B x%0d, 1B x%0d) errors=%0d t=%0t",
                     rx_msgs, got_cnt[0], got_cnt[1], got_cnt[2], errors, $time);
        $finish;
    end

    initial begin
        #WD_NS;
        $display("[SHORT-TB] FAIL: watchdog timeout (delivered=%0d/3) t=%0t",
                 rx_msgs, $time);
        $finish;
    end

endmodule
