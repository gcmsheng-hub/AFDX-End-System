//=============================================================================
// tb_cfg.v — T6-003 单元测试：timebase / cfg_bus / tx_vl_table / ram_init
// 覆盖：
//   1) ram_init：复位释放即清表+stats（观测 busy/done 时序），done 前表读出 0
//   2) cfg 全局寄存器：VERSION/UID/SRC_IP/CTRL/STATUS 读写 + 内核直读线
//   3) cfg VL 表：写 8 字 → 读回逐字比对（含 VL=127 边界）；rvalid 1 拍语义
//   3b) VL 表写门禁（FIX2）：非法 BAG（0/3/129）与 net_mask=00 拒收（广播不发、
//       读回保持旧值、0x00005 计数/0x00006 原因置位）；合法 BAG 全集合与
//       mask 01/10/11 接受
//   4) cw 写广播脉冲内容（idx/wsel/wdata）
//   5) 内核读口（rd_idx/rd_wsel → rd_data 1 拍）
//   6) timebase：time_ns == 8×周期数
//   7) 未映射区读回 0
//=============================================================================

`timescale 1ns/1ps

module tb_cfg;

    parameter CLK_HALF = 4;
    integer errors;

    reg clk, reset;
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    task chk;
        input cond;
        input [511:0] name;
        begin
            if (cond !== 1'b1) begin
                $display("[CFG-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // DUT 集（cfg_bus + tx_vl_table + ram_init + timebase，核级连线方式同 afdx_es_core）
    //-------------------------------------------------------------------------
    reg  [19:0] cfg_addr;
    reg  [31:0] cfg_wdata;
    reg         cfg_we, cfg_re;
    wire [31:0] cfg_rdata;
    wire        cfg_rvalid;

    wire [15:0] g_uid;
    wire [31:0] g_src_ip;
    wire        g_tx_en;

    wire        cw_we;
    wire [6:0]  cw_idx;
    wire [2:0]  cw_wsel;
    wire [31:0] cw_wdata;

    wire        tbl_re;
    wire [9:0]  tbl_raddr;
    wire [31:0] tbl_rdata;
    wire        tbl_busy = 1'b0;           // 单测无组帧器，读口永远空闲

    reg  [6:0]  rd_idx;
    reg  [2:0]  rd_wsel;
    wire [31:0] rd_data;

    wire        init_busy, init_done;
    wire [2:0]  ri_sel;
    wire [15:0] ri_addr;
    wire        ri_we;

    wire [63:0] time_ns;

    cfg_bus u_cfg (
        .clk(clk), .reset(reset),
        .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata), .cfg_we(cfg_we), .cfg_re(cfg_re),
        .cfg_rdata(cfg_rdata), .cfg_rvalid(cfg_rvalid),
        .g_uid(g_uid), .g_src_ip(g_src_ip), .g_tx_en(g_tx_en),
        .init_done(init_done),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .tbl_re(tbl_re), .tbl_raddr(tbl_raddr), .tbl_rdata(tbl_rdata), .tbl_busy(tbl_busy),
        .stats_re(), .stats_raddr(), .stats_rdata(32'h0)
    );

    tx_vl_table u_tbl (
        .clk(clk),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .tbl_re(tbl_re), .tbl_raddr(tbl_raddr), .tbl_busy(tbl_busy), .tbl_rdata(tbl_rdata),
        .rd_idx(rd_idx), .rd_wsel(rd_wsel), .rd_data(rd_data),
        .ri_we(ri_we && (ri_sel == 3'd1)),
        .ri_addr(ri_addr[9:0])
    );

    ram_init u_ri (
        .clk(clk), .reset(reset),
        .init_busy(init_busy), .init_done(init_done),
        .ri_sel(ri_sel), .ri_addr(ri_addr), .ri_we(ri_we)
    );

    timebase u_tb (
        .clk(clk), .reset(reset), .time_ns(time_ns)
    );

    //-------------------------------------------------------------------------
    // cfg 读写任务（negedge 驱动；读在 posedge+1 采样 rvalid/rdata）
    //-------------------------------------------------------------------------
    integer guard;

    task cfg_write;
        input [19:0] a;
        input [31:0] d;
        begin
            @(negedge clk);
            cfg_addr = a; cfg_wdata = d; cfg_we = 1'b1;
            @(negedge clk);
            cfg_we = 1'b0;
        end
    endtask

    task cfg_read;                          // 结果存 g_rd/g_rv
        input [19:0] a;
        output [31:0] rd;
        output rv;
        begin
            @(negedge clk);
            cfg_addr = a; cfg_we = 1'b0; cfg_re = 1'b1;
            @(posedge clk); #1;             // 1 拍后 rvalid+rdata 同拍有效
            rd = cfg_rdata;
            rv = cfg_rvalid;
            @(negedge clk);
            cfg_re = 1'b0;
        end
    endtask

    reg [31:0] g_rd;
    reg        g_rv;
    reg [31:0] exp_w [0:7];
    integer w;

    // 监视 cw 广播
    reg        m_cw_we;
    reg [6:0]  m_cw_idx;
    reg [2:0]  m_cw_wsel;
    reg [31:0] m_cw_wdata;
    always @(posedge clk) begin
        m_cw_we <= cw_we; m_cw_idx <= cw_idx;
        m_cw_wsel <= cw_wsel; m_cw_wdata <= cw_wdata;
    end

    // 周期计数（timebase 对照）
    integer cycles;
    always @(posedge clk) begin
        if (reset) cycles <= 0;
        else cycles <= cycles + 1;
    end

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    initial begin : main
        errors = 0; reset = 1'b1;
        cfg_addr = 0; cfg_wdata = 0; cfg_we = 0; cfg_re = 0;
        rd_idx = 0; rd_wsel = 0;
        m_cw_we = 0; m_cw_idx = 0; m_cw_wsel = 0; m_cw_wdata = 0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;

        //---------------------------------------------------------------
        // 1) ram_init：约 1024 + 3×128 拍
        //---------------------------------------------------------------
        chk(init_busy === 1'b1, "init_busy high right after reset release");
        chk(init_done === 1'b0, "init_done low during init");
        guard = 0;
        while (init_done !== 1'b1 && guard < 50000) begin
            @(posedge clk); #1;
            guard = guard + 1;
        end
        chk(init_done === 1'b1, "init_done rises within 50000 cycles");
        // T7：init 依次清 发送表(1024)+stats(3×128)+IC(8192)+RM(4096)+接收表(4096)
        chk(guard >= 1024 + 3*128 + 8192 + 4096 + 4096 - 8
            && guard <= 1024 + 3*128 + 8192 + 4096 + 4096 + 8,
            "init duration ~= 1024+384+16384 cycles (T7 增 IC/RM/接收表)");
        repeat (2) @(posedge clk); #1;
        chk(ri_we === 1'b0, "ri_we low after done");

        // done 后表应全 0（抽查表头/表中/表尾）
        cfg_read(20'h01000, g_rd, g_rv);
        chk(g_rv === 1'b1 && g_rd === 32'h0, "table[0][0]==0 after init");
        cfg_read(20'h01000 + 10'h200, g_rd, g_rv);   // VL64 W0
        chk(g_rv === 1'b1 && g_rd === 32'h0, "table[64][0]==0 after init");
        cfg_read(20'h013FF, g_rd, g_rv);             // VL127 W7
        chk(g_rv === 1'b1 && g_rd === 32'h0, "table[127][7]==0 after init");

        //---------------------------------------------------------------
        // 2) 全局寄存器
        //---------------------------------------------------------------
        cfg_read(20'h00000, g_rd, g_rv);
        chk(g_rd === 32'h20261031, "VERSION readback");
        cfg_write(20'h00001, 32'h00000A17);
        cfg_write(20'h00002, 32'h0A000205);
        cfg_write(20'h00003, 32'h0000_0001);
        #1;
        chk(g_uid === 16'h0A17, "g_uid wire after write");
        chk(g_src_ip === 32'h0A000205, "g_src_ip wire after write");
        chk(g_tx_en === 1'b1, "g_tx_en wire after write");
        cfg_read(20'h00001, g_rd, g_rv);
        chk(g_rd === 32'h00000A17, "UID readback");
        cfg_read(20'h00002, g_rd, g_rv);
        chk(g_rd === 32'h0A000205, "SRC_IP readback");
        cfg_read(20'h00003, g_rd, g_rv);
        chk(g_rd === 32'h1, "CTRL readback");
        cfg_read(20'h00004, g_rd, g_rv);
        chk(g_rd === 32'h1, "STATUS.init_done readback==1");
        // rvalid 单拍（re 只保持一拍）
        @(negedge clk); cfg_addr = 20'h00000; cfg_re = 1'b1;
        @(posedge clk); #1; chk(cfg_rvalid === 1'b1, "rvalid cycle+1");
        @(negedge clk); cfg_re = 1'b0;
        @(posedge clk); #1; chk(cfg_rvalid === 1'b0, "rvalid single-cycle");

        //---------------------------------------------------------------
        // 3) VL 表写读回（VL=5 全 8 字 + VL=127 边界）
        //    （W0 用合法 BAG=8：cfg 写门禁（FIX2）会拒收非法 bag_ms）
        //---------------------------------------------------------------
        exp_w[0] = 32'hA5000008;
        for (w = 1; w < 8; w = w + 1)
            exp_w[w] = 32'hA5000000 + w;
        for (w = 0; w < 8; w = w + 1)
            cfg_write(20'h01000 + (5 << 3) + w, exp_w[w]);
        // cw 广播监视：最后一条写应为 idx=5 wsel=7
        chk(m_cw_we === 1'b1, "cw_we pulse on table write");
        chk(m_cw_idx === 7'd5 && m_cw_wsel === 3'd7 && m_cw_wdata === 32'hA5000007,
            "cw broadcast idx/wsel/wdata");
        for (w = 0; w < 8; w = w + 1) begin
            cfg_read(20'h01000 + (5 << 3) + w, g_rd, g_rv);
            if (g_rd !== exp_w[w] || g_rv !== 1'b1) begin
                $display("[CFG-TB] FAIL: VL5 W%0d readback %08x exp %08x",
                         w, g_rd, exp_w[w]);
                errors = errors + 1;
            end
        end
        // 边界 VL=127
        cfg_write(20'h01000 + (127 << 3) + 3, 32'h0000_0003);  // net_mask=A|B
        cfg_read(20'h01000 + (127 << 3) + 3, g_rd, g_rv);
        chk(g_rd === 32'h0000_0003, "VL127 W3 readback");

        //---------------------------------------------------------------
        // 3b) VL 表写门禁（FIX2 #5/#7）：非法 BAG / net_mask=00 拒收，
        //     广播不发、读回保持旧值、拒收计数与原因可读（0x00005/0x00006）
        //---------------------------------------------------------------
        cfg_write(20'h01000 + (5 << 3) + 0, 32'h8000_0003);    // bag=3 非法
        chk(m_cw_we === 1'b0, "3b: illegal bag write suppressed (no cw_we)");
        cfg_read(20'h01000 + (5 << 3) + 0, g_rd, g_rv);
        chk(g_rd === 32'hA5000008, "3b: W0 readback unchanged after reject");
        cfg_read(20'h00005, g_rd, g_rv);
        chk(g_rd === 32'd1, "3b: rej_cnt==1");
        cfg_read(20'h00006, g_rd, g_rv);
        chk(g_rd === 32'h1, "3b: rej_flag bit0 (bag illegal)");
        cfg_write(20'h01000 + (5 << 3) + 0, 32'h8000_0000);    // bag=0 非法
        cfg_write(20'h01000 + (5 << 3) + 0, 32'h8000_0081);    // bag=129 非法
        cfg_read(20'h00005, g_rd, g_rv);
        chk(g_rd === 32'd3, "3b: rej_cnt==3 (bag=0, bag=129)");
        // 合法 BAG 全集合 {1,2,4,8,16,32,64,128} 逐个接受
        for (w = 0; w < 8; w = w + 1) begin
            cfg_write(20'h01000 + (5 << 3) + 0, 32'h8000_0000 + (32'd1 << w));
            cfg_read(20'h01000 + (5 << 3) + 0, g_rd, g_rv);
            if (g_rd !== (32'h8000_0000 + (32'd1 << w))) begin
                $display("[CFG-TB] FAIL: legal bag %0d rejected", (1 << w));
                errors = errors + 1;
            end
        end
        cfg_read(20'h00005, g_rd, g_rv);
        chk(g_rd === 32'd3, "3b: rej_cnt unchanged by legal bags");
        // net_mask=00 拒收；01/10/11 接受
        cfg_write(20'h01000 + (5 << 3) + 3, 32'h0000_0000);    // mask=00 非法
        chk(m_cw_we === 1'b0, "3b: mask=00 write suppressed");
        cfg_read(20'h01000 + (5 << 3) + 3, g_rd, g_rv);
        chk(g_rd === 32'hA5000003, "3b: W3 readback unchanged after reject");
        cfg_read(20'h00005, g_rd, g_rv);
        chk(g_rd === 32'd4, "3b: rej_cnt==4 (mask=00)");
        cfg_read(20'h00006, g_rd, g_rv);
        chk(g_rd === 32'h3, "3b: rej_flag bit0+bit1");
        cfg_write(20'h01000 + (5 << 3) + 3, 32'h0000_0001);    // A-only
        cfg_read(20'h01000 + (5 << 3) + 3, g_rd, g_rv);
        chk(g_rd === 32'h0000_0001, "3b: mask=01 (A-only) accepted");
        cfg_write(20'h01000 + (5 << 3) + 3, 32'h0000_0002);    // B-only
        cfg_read(20'h01000 + (5 << 3) + 3, g_rd, g_rv);
        chk(g_rd === 32'h0000_0002, "3b: mask=10 (B-only) accepted");
        cfg_write(20'h01000 + (5 << 3) + 3, 32'h0000_0003);    // A+B
        cfg_read(20'h01000 + (5 << 3) + 3, g_rd, g_rv);
        chk(g_rd === 32'h0000_0003, "3b: mask=11 (A+B) accepted");

        //---------------------------------------------------------------
        // 5) 内核读口（1 拍延迟）
        //---------------------------------------------------------------
        @(negedge clk);
        rd_idx = 7'd5; rd_wsel = 3'd2;
        @(posedge clk); #1;
        chk(rd_data === 32'hA5000002, "core read port 1-cycle data (VL5 W2)");
        @(negedge clk);
        rd_idx = 7'd127; rd_wsel = 3'd3;
        @(posedge clk); #1;
        chk(rd_data === 32'h0000_0003, "core read port (VL127 W3)");

        //---------------------------------------------------------------
        // 6) timebase：time_ns == 8 × 周期数
        //---------------------------------------------------------------
        #1;
        chk(time_ns === cycles * 8, "time_ns == 8*cycles");

        //---------------------------------------------------------------
        // 7) 未映射区
        //---------------------------------------------------------------
        cfg_read(20'h30000, g_rd, g_rv);
        chk(g_rv === 1'b1 && g_rd === 32'h0, "unmapped region reads 0 with rvalid");

        //---------------------------------------------------------------
        // 结论
        //---------------------------------------------------------------
        if (errors == 0)
            $display("[CFG-TB] PASS");
        else
            $display("[CFG-TB] FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #1_000_000;
        $display("[CFG-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
