//=============================================================================
// tb_probe_micro.v — 微探针 TB（2026-11-04 令）：门级 simfix 修复的第一层验证
// 骨架取自 tb_delivery_short.v（引脚/时钟/cfg_write/do_config 原样），三处改动：
//   ① INIT_WAIT_CYC = 30000（作 init 轮询上限：240 µs，覆盖预期 ≈142.4 µs）
//   ② 新增 cfg_read 任务：negedge 驱动 cfg_re 单拍，等 cfg_rvalid 捕获 cfg_rdata，
//      超时 32 拍打印 TIMEOUT（cfg_bus 契约：re 拍锁存地址，下一拍 rvalid+rdata）
//   ③ 主流程改为寄存器读数探针（删除原 SHORT-TB 逐字节 rx 监视，前缀统一 [MPROBE]）
//
// 探针序列（每步打印实测值 + 期望值）：
//   P1 复位释放即读 STATUS@0x00004       期望 0（init 未完成；=1 即异常，停报）
//   P2 每 2000 拍轮询 STATUS             init_done 置位时刻期望 ≈143 µs
//   P3 do_config 后回读 TX 表            W2@0x01052 期望 0xA / W0@0x01050 期望
//                                        0x80000001 / 未配置 idx20 W0@0x010A0 期望 0
//   P4 只注入 VL10 100B，注入毕即读 g_msgs@0x203C2   期望 ≥1
//   P5 每 250 µs 读 g_tx_frames@0x203C0 / g_rx_frames@0x203C5 /
//      g_rx_msgs@0x203D0，最多 6 次；g_rx_msgs≥1 打印 DELIVERED 收尾
//   看门狗 2.5 ms（6 次轮询共 1.5 ms + init/配置/注入 ≈0.15 ms，留余量）
//
// 地址依据（rtl/core/cfg_bus.v、rtl/core/stats.v 已核）：
//   0x00004 = STATUS.bit0 init_done；0x01000+VL*8+W = TX VL 表；
//   0x203C0+n（n=addr[5:0]，addr[9:6]==0xF 全局区）：0x00 g_frames、0x02 g_msgs、
//   0x05 g_rx_frames、0x10 g_rx_msgs。
//=============================================================================
`timescale 1ns/1ps

module tb_probe_micro;

    parameter CLK_HALF = 4;
    parameter integer INIT_WAIT_CYC = 30000;   // init 轮询上限（240 µs @8ns 拍）
    parameter integer WD_NS = 2_500_000;       // 2.5 ms 看门狗

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

    // ---- 新增：单拍 cfg 读（地址 negedge 驱动，rvalid 下一拍；超时 32 拍）----
    task cfg_read;
        input  [19:0] a;
        output [31:0] d;
        integer tmo;
        begin
            @(negedge clk);
            cfg_addr = a;
            cfg_re   = 1'b1;
            @(negedge clk);
            cfg_re   = 1'b0;
            tmo = 0;
            while (!cfg_rvalid && tmo < 32) begin
                @(posedge clk);
                tmo = tmo + 1;
            end
            if (cfg_rvalid)
                d = cfg_rdata;
            else begin
                d = 32'hDEAD_BEEF;
                $display("[MPROBE] TIMEOUT cfg_read addr=0x%05h (>32 cyc)", a);
            end
            @(negedge clk);              // 收口：rvalid 泄放
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

    localparam integer CYC_250US = 31250;      // 250 µs @ 8 ns 拍

    integer    poll_cyc, k, mism;
    reg [31:0] rd, tx_f, rx_f, rx_m;
    reg        init_seen, delivered;

    initial begin : main
        reset = 1'b1; tx_valid = 1'b0; tx_data = 8'h0; rx_ready = 1'b1;
        cfg_addr = 20'h0; cfg_wdata = 32'h0; cfg_we = 1'b0; cfg_re = 1'b0;
        mism = 0;
        tx_f = 0; rx_f = 0; rx_m = 0;
        delivered = 1'b0;

        repeat (100) @(posedge clk);
        @(negedge clk); reset = 1'b0;

        // ---- P1：复位释放即读 STATUS（期望 0）----
        cfg_read(20'h00004, rd);
        $display("[MPROBE] P1 STATUS@0x00004 after reset release = 0x%08h (expect 0) t=%0t (%0d.%03d us)",
                 rd, $time, $time/1000, $time%1000);
        if (rd[0] !== 1'b0) begin
            $display("[MPROBE] ABORT: init_done already 1 at reset release (abnormal) — gate-level NOT run");
            $finish;
        end

        // ---- P2：每 2000 拍轮询 STATUS，打印 init_done 置位时刻（期望 ≈143 µs）----
        init_seen = 1'b0;
        poll_cyc  = 0;
        while (!init_seen && poll_cyc < INIT_WAIT_CYC) begin
            cfg_read(20'h00004, rd);
            if (rd[0] === 1'b1) begin
                init_seen = 1'b1;
                $display("[MPROBE] P2 INIT-DONE init_done=1 t=%0t (%0d.%03d us) expect~143us poll_cyc=%0d",
                         $time, $time/1000, $time%1000, poll_cyc);
            end else begin
                repeat (2000) @(posedge clk);
                poll_cyc = poll_cyc + 2000;
            end
        end
        if (!init_seen) begin
            $display("[MPROBE] ABORT: init_done not set within INIT_WAIT_CYC=%0d cycles (t=%0t) — gate-level NOT run",
                     INIT_WAIT_CYC, $time);
            $finish;
        end

        // ---- P3：配置 + 回读 ----
        do_config;
        $display("[MPROBE] P3 cfg-done t=%0t (%0d.%03d us)", $time, $time/1000, $time%1000);

        cfg_read(20'h01052, rd);
        $display("[MPROBE] P3 TX W2@0x01052 = 0x%08h (expect 0x0000000A)", rd);
        if (rd !== 32'h0000_000A) mism = mism + 1;

        cfg_read(20'h01050, rd);
        $display("[MPROBE] P3 TX W0@0x01050 = 0x%08h (expect 0x80000001)", rd);
        if (rd !== 32'h8000_0001) mism = mism + 1;

        cfg_read(20'h010A0, rd);
        $display("[MPROBE] P3 TX W0@0x010A0 idx20-unconfig = 0x%08h (expect 0x00000000)", rd);
        if (rd !== 32'h0000_0000) mism = mism + 1;

        // ---- P4：只注入 VL10 100B，注入毕立即读 g_msgs（期望 ≥1）----
        drive_envelope(16'h000A, 100);
        $display("[MPROBE] P4 injected VL10 100B t=%0t (%0d.%03d us)", $time, $time/1000, $time%1000);

        cfg_read(20'h203C2, rd);
        $display("[MPROBE] P4 g_msgs@0x203C2 = %0d (expect >=1) t=%0t", rd, $time);
        if (rd < 32'd1) mism = mism + 1;

        // ---- P5：每 250 µs 读三个计数，最多 6 次；g_rx_msgs≥1 即 DELIVERED ----
        for (k = 0; k < 6 && !delivered; k = k + 1) begin
            repeat (CYC_250US) @(posedge clk);
            cfg_read(20'h203C0, tx_f);
            cfg_read(20'h203C5, rx_f);
            cfg_read(20'h203D0, rx_m);
            $display("[MPROBE] P5 poll#%0d tx_frames=%0d rx_frames=%0d rx_msgs=%0d t=%0t",
                     k + 1, tx_f, rx_f, rx_m, $time);
            if (rx_m >= 32'd1) begin
                delivered = 1'b1;
                $display("[MPROBE] DELIVERED rx_msgs=%0d tx_frames=%0d rx_frames=%0d t=%0t (%0d.%03d us)",
                         rx_m, tx_f, rx_f, $time, $time/1000, $time%1000);
            end
        end

        $display("[MPROBE] SUMMARY mismatches=%0d delivered=%0d tx_frames=%0d rx_frames=%0d rx_msgs=%0d t=%0t",
                 mism, delivered, tx_f, rx_f, rx_m, $time);
        $finish;
    end

    initial begin
        #WD_NS;
        $display("[MPROBE] FAIL: watchdog timeout (2.5ms) t=%0t — last tx_frames=%0d rx_frames=%0d rx_msgs=%0d",
                 $time, tx_f, rx_f, rx_m);
        $finish;
    end

endmodule
