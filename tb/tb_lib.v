//=============================================================================
// tb_lib.v — rtl/lib 六件基础模块单元测试（自检式，无波形判定）
// golden 常量来源（生成命令见 RPT-T6-001）：
//   CRC32：python3 -c "import zlib; print(hex(zlib.crc32(b'...')))"
//          '123456789' → 0xCBF43926（反射 CRC32 标准校验值）
//          0x00×64 → 0x758D6336；0xFF×64 → 0x0F6187BA；
//          (i*13+7)&0xFF ×1471 → 0x340E5693
//   IP 头反码校验和：afdx_ref.ip_csum(样例头) = 0x8DC9
// 覆盖：crc32_d8（4 向量）、csum16（含 clr 复算）、tick_gen（125 拍周期）、
//       rr_arbiter（轮转次序/单请求）、sdp_ram（读写延迟）、sync_fifo（FWFT/
//       count/almost_full/full 行为）。
//=============================================================================

`timescale 1ns/1ps

module tb_lib;

    parameter CLK_HALF = 4;        // 125 MHz
    integer errors;

    reg clk, reset;
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    task chk;
        input cond;
        input [511:0] name;                // 加宽避免长名字截断
        begin
            if (cond !== 1'b1) begin
                $display("[LIB-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    //=========================================================================
    // crc32_d8
    //=========================================================================
    reg        crc_init, crc_valid;
    reg  [7:0] crc_data;
    wire [31:0] crc_crc, crc_fcs;
    integer n, cycles;

    crc32_d8 u_crc (
        .clk(clk), .reset(reset),
        .init(crc_init), .valid(crc_valid), .data(crc_data),
        .crc(crc_crc), .fcs(crc_fcs)
    );

    task crc_run;                       // 喂一段字节并等结果稳定
        input [1023:0] dummy;
        begin
            // 负沿→负沿脉冲：保证恰好覆盖一个 posedge，避免与 always 竞争
            @(negedge clk);
            crc_init = 1'b1;
            @(negedge clk);
            crc_init = 1'b0;
        end
    endtask

    task crc_feed;                      // 喂单个字节
        input [7:0] b;
        begin
            @(negedge clk);
            crc_valid = 1'b1; crc_data = b;
            @(negedge clk);
            crc_valid = 1'b0;
        end
    endtask

    //=========================================================================
    // csum16
    //=========================================================================
    reg        cs_clr, cs_valid;
    reg  [7:0] cs_data;
    wire [15:0] cs_out;

    csum16 u_csum (
        .clk(clk), .reset(reset),
        .clr(cs_clr), .valid(cs_valid), .data(cs_data), .csum(cs_out)
    );

    task csum_feed;
        input [7:0] b;
        begin
            @(negedge clk);
            cs_valid = 1'b1; cs_data = b;
            @(negedge clk);
            cs_valid = 1'b0;
        end
    endtask

    //=========================================================================
    // tick_gen
    //=========================================================================
    wire tk_tick;
    integer tk_cnt, tk_last, tk_gap1, tk_gap2;

    tick_gen #(.NS_PER_CLK(8), .TICK_NS(1000)) u_tick (
        .clk(clk), .reset(reset), .tick(tk_tick)
    );

    always @(posedge clk) begin
        if (reset) begin
            tk_cnt <= 0;
        end else begin
            tk_cnt <= tk_cnt + 1;
            if (tk_tick) begin
                tk_gap2 <= tk_gap1;
                tk_gap1 <= tk_cnt - tk_last;
                tk_last <= tk_cnt;
            end
        end
    end

    //=========================================================================
    // rr_arbiter (N=4)
    //=========================================================================
    reg  [3:0]  rr_req;
    reg         rr_upd;
    wire        rr_gnt;
    wire [1:0]  rr_idx;

    rr_arbiter #(.N(4), .IW(2)) u_rr (
        .clk(clk), .reset(reset),
        .req(rr_req), .update(rr_upd), .gnt(rr_gnt), .gidx(rr_idx)
    );

    task rr_step;                       // 采样一次 grant 并推进指针
        output [1:0] g;
        begin
            @(negedge clk);
            g      = rr_idx;
            rr_upd = 1'b1;
            @(negedge clk);
            rr_upd = 1'b0;
        end
    endtask

    //=========================================================================
    // sdp_ram
    //=========================================================================
    reg         ram_we;
    reg  [3:0]  ram_wa, ram_ra;
    reg  [15:0] ram_wd;
    wire [15:0] ram_rd;

    sdp_ram #(.WIDTH(16), .DEPTH(16), .AW(4)) u_ram (
        .clk(clk), .we(ram_we), .waddr(ram_wa), .wdata(ram_wd),
        .raddr(ram_ra), .rdata(ram_rd)
    );

    //=========================================================================
    // sync_fifo
    //=========================================================================
    reg         f_we, f_re;
    reg  [7:0]  f_wd;
    wire [7:0]  f_rd;
    wire        f_full, f_empty, f_af;
    wire [4:0]  f_cnt;

    sync_fifo #(.DW(8), .DEPTH(16), .AW(4), .AF_LEVEL(14)) u_fifo (
        .clk(clk), .reset(reset),
        .we(f_we), .wdata(f_wd), .re(f_re),
        .rdata(f_rd), .full(f_full), .empty(f_empty),
        .count(f_cnt), .almost_full(f_af)
    );

    //=========================================================================
    // 主流程
    //=========================================================================
    reg [1:0] g0;
    reg [71:0] s_pat;                  // "123456789"（V2001 字符串须落 reg 后才能位选）
    integer m;

    initial begin : main
        errors   = 0;
        reset    = 1'b1;
        crc_init = 0; crc_valid = 0; crc_data = 0;
        cs_clr = 0; cs_valid = 0; cs_data = 0;
        rr_req = 0; rr_upd = 0;
        ram_we = 0; ram_wa = 0; ram_ra = 0; ram_wd = 0;
        f_we = 0; f_re = 0; f_wd = 0;
        tk_cnt = 0; tk_last = 0; tk_gap1 = 0; tk_gap2 = 0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        repeat (2) @(posedge clk);

        //------------------------------------------------ crc32_d8
        s_pat = "123456789";
        crc_run(0);
        for (n = 0; n < 9; n = n + 1) crc_feed(s_pat[71 - n*8 -: 8]);  // 串右对齐，从高位取
        @(posedge clk); #1;                          // 等最后字节推进完成
        chk(crc_fcs === 32'hCBF43926, "crc32('123456789')==0xCBF43926");

        crc_run(0);
        for (n = 0; n < 64; n = n + 1) crc_feed(8'h00);
        @(posedge clk); #1;
        chk(crc_fcs === 32'h758D6336, "crc32(0x00 x64)==0x758D6336");

        crc_run(0);
        for (n = 0; n < 64; n = n + 1) crc_feed(8'hFF);
        @(posedge clk); #1;
        chk(crc_fcs === 32'h0F6187BA, "crc32(0xFF x64)==0x0F6187BA");

        crc_run(0);
        for (n = 0; n < 1471; n = n + 1) crc_feed((n*13+7) & 8'hFF);
        @(posedge clk); #1;
        chk(crc_fcs === 32'h340E5693, "crc32(pattern1471)==0x340E5693");

        //------------------------------------------------ csum16
        // 样例 IPv4 头（csum 字段为 0）：45 00 00 14 00 01 00 00 40 11 00 00
        //                              0A 00 02 05 E0 E0 00 2A → 0x8DC9
        @(negedge clk); cs_clr = 1'b1; @(negedge clk); cs_clr = 1'b0;
        csum_feed(8'h45); csum_feed(8'h00); csum_feed(8'h00); csum_feed(8'h14);
        csum_feed(8'h00); csum_feed(8'h01); csum_feed(8'h00); csum_feed(8'h00);
        csum_feed(8'h40); csum_feed(8'h11); csum_feed(8'h00); csum_feed(8'h00);
        csum_feed(8'h0A); csum_feed(8'h00); csum_feed(8'h02); csum_feed(8'h05);
        csum_feed(8'hE0); csum_feed(8'hE0); csum_feed(8'h00); csum_feed(8'h2A);
        @(posedge clk); #1;
        chk(cs_out === 16'h8DC9, "csum16(sample ip hdr)==0x8DC9");

        // clr 复算同向量（验证清零正确）
        @(negedge clk); cs_clr = 1'b1; @(negedge clk); cs_clr = 1'b0;
        csum_feed(8'h45); csum_feed(8'h00); csum_feed(8'h00); csum_feed(8'h14);
        csum_feed(8'h00); csum_feed(8'h01); csum_feed(8'h00); csum_feed(8'h00);
        csum_feed(8'h40); csum_feed(8'h11); csum_feed(8'h00); csum_feed(8'h00);
        csum_feed(8'h0A); csum_feed(8'h00); csum_feed(8'h02); csum_feed(8'h05);
        csum_feed(8'hE0); csum_feed(8'hE0); csum_feed(8'h00); csum_feed(8'h2A);
        @(posedge clk); #1;
        chk(cs_out === 16'h8DC9, "csum16(clr recompute)==0x8DC9");

        //------------------------------------------------ tick_gen
        // 等 3 个 tick 后检查相邻间隔均为 125 拍
        wait (tk_gap2 != 0);
        chk(tk_gap1 === 125 && tk_gap2 === 125, "tick_gen period==125 cycles");

        //------------------------------------------------ rr_arbiter
        rr_req = 4'b0101;                        // 置位 bit0/bit2；last=0 → 从 1 环找：2,0,2...
        rr_step(g0); chk(g0 === 2'd2, "rr(0101) first grant==2");
        rr_step(g0); chk(g0 === 2'd0, "rr(0101) second grant==0");
        rr_step(g0); chk(g0 === 2'd2, "rr(0101) third grant==2");

        rr_req = 4'b1111;                        // 全请求 → 严格轮转（接 last=2）：3,0,1,2,3
        rr_step(g0); chk(g0 === 2'd3, "rr(1111) grant==3");
        rr_step(g0); chk(g0 === 2'd0, "rr(1111) grant==0");
        rr_step(g0); chk(g0 === 2'd1, "rr(1111) grant==1");
        rr_step(g0); chk(g0 === 2'd2, "rr(1111) grant==2");
        rr_step(g0); chk(g0 === 2'd3, "rr(1111) grant==3");

        rr_req = 4'b0010;                        // 置位 bit1；last=3 → 环找命中 1，恒 1
        rr_step(g0); chk(g0 === 2'd1, "rr(0010) grant==1");
        rr_step(g0); chk(g0 === 2'd1, "rr(0010) grant==1 again");
        rr_req = 4'b0000;
        @(negedge clk);
        chk(rr_gnt === 1'b0, "rr(no req) gnt==0");

        //------------------------------------------------ sdp_ram
        for (m = 0; m < 4; m = m + 1) begin
            @(negedge clk);
            ram_we = 1; ram_wa = m[3:0]; ram_wd = 16'hB000 + m[15:0];
            @(negedge clk);
        end
        ram_we = 0;
        // 同步读延迟 1 拍：给出地址后下一拍数据有效
        ram_ra = 4'd2; @(posedge clk); #1;
        chk(ram_rd === 16'hB002, "sdp_ram readback[2]==0xB002 (1-cycle lat)");
        ram_ra = 4'd0; @(posedge clk); #1;
        chk(ram_rd === 16'hB000, "sdp_ram readback[0]==0xB000");

        //------------------------------------------------ sync_fifo
        chk(f_empty === 1'b1, "fifo starts empty");
        for (m = 0; m < 16; m = m + 1) begin      // 推满
            @(negedge clk); f_we = 1; f_wd = 8'h40 + m[7:0]; @(negedge clk); f_we = 0;
        end
        chk(f_full === 1'b1, "fifo full after 16 pushes");
        chk(f_cnt === 5'd16, "fifo count==16");
        chk(f_af === 1'b1, "fifo almost_full at 16");
        chk(f_rd === 8'h40, "fifo FWFT head==0x40");
        // 满时写被忽略：再写一个不同值，队首不变
        @(negedge clk); f_we = 1; f_wd = 8'hEE; @(negedge clk); f_we = 0;
        chk(f_rd === 8'h40, "fifo write-when-full ignored");
        // 弹 3 个
        for (m = 0; m < 3; m = m + 1) begin
            @(negedge clk); f_re = 1; @(negedge clk); f_re = 0;
        end
        chk(f_rd === 8'h43, "fifo head after 3 pops==0x43");
        chk(f_cnt === 5'd13, "fifo count==13");
        // almost_full 阈值=14：13 个时不置位
        chk(f_af === 1'b0, "fifo almost_full deasserts below 14");
        // 弹空
        for (m = 0; m < 13; m = m + 1) begin
            @(negedge clk); f_re = 1; @(negedge clk); f_re = 0;
        end
        chk(f_empty === 1'b1, "fifo empty after drain");
        chk(f_cnt === 5'd0, "fifo count==0 after drain");

        //------------------------------------------------ 结论
        if (errors == 0)
            $display("[LIB-TB] PASS");
        else
            $display("[LIB-TB] FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #200_000;
        $display("[LIB-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
