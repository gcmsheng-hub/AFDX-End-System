//=============================================================================
// tb_sched.v — T6-004 单元测试：tx_sn_gen / tx_regulator / tx_scheduler /
//              host_arb / stats（组帧器与 mac 用 tb 桩代替，全链集成在 T6-005）
// 覆盖：
//   S1 sn_gen：SN 0→1..255→1 回绕 + VL 间独立（257 次读序列）
//   S2 host_arb(N=2)：包级锁定不交错 + RR 轮转 + 字节完整
//   S3 regulator：W0 配置即到期；W4 相位延迟 = reload−phase；fs 后精确 BAG 再到期
//   S4 集成（4 VL × 2 轮）：RR 次序、SN 每 VL 从 0 独立、stats 每 VL/全局计数
//=============================================================================

`timescale 1ns/1ps

module tb_sched;

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
                $display("[SCH-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 公共：cfg/初始化/时钟件
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
    wire        tbl_busy = 1'b0;
    wire        stats_re;
    wire [9:0]  stats_raddr;
    wire [31:0] stats_rdata;
    wire        init_busy, init_done;
    wire [2:0]  ri_sel;
    wire [15:0] ri_addr;
    wire        ri_we;
    wire [63:0] time_ns;
    wire        tick_us;

    cfg_bus u_cfg (
        .clk(clk), .reset(reset),
        .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata), .cfg_we(cfg_we), .cfg_re(cfg_re),
        .cfg_rdata(cfg_rdata), .cfg_rvalid(cfg_rvalid),
        .g_uid(g_uid), .g_src_ip(g_src_ip), .g_tx_en(g_tx_en),
        .init_done(init_done),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .tbl_re(tbl_re), .tbl_raddr(tbl_raddr), .tbl_rdata(tbl_rdata), .tbl_busy(tbl_busy),
        .stats_re(stats_re), .stats_raddr(stats_raddr), .stats_rdata(stats_rdata)
    );

    tx_vl_table u_tbl (
        .clk(clk),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .tbl_re(tbl_re), .tbl_raddr(tbl_raddr), .tbl_busy(tbl_busy), .tbl_rdata(tbl_rdata),
        .rd_idx(7'd0), .rd_wsel(3'd0), .rd_data(),
        .ri_we(ri_we && (ri_sel == 3'd1)), .ri_addr(ri_addr[9:0])
    );

    ram_init u_ri (
        .clk(clk), .reset(reset),
        .init_busy(init_busy), .init_done(init_done),
        .ri_sel(ri_sel), .ri_addr(ri_addr), .ri_we(ri_we)
    );

    tick_gen u_tick (.clk(clk), .reset(reset), .tick(tick_us));
    timebase u_time (.clk(clk), .reset(reset), .time_ns(time_ns));

    //-------------------------------------------------------------------------
    // 发送子系统：host_arb(N=1) → buf_mgr；regulator/scheduler/stats/sn_gen
    //-------------------------------------------------------------------------
    reg          ah_valid, ah_sop, ah_eop;
    reg  [7:0]   ah_data;
    reg  [6:0]   ah_vl;
    wire         ah_ready;
    wire         htx_valid, htx_sop, htx_eop, htx_ready;
    wire [7:0]   htx_data;
    wire [6:0]   htx_vl;
    wire [127:0] q_nonempty;
    wire         desc_vld, ev_enq, ev_qfull, ev_lenerr, ev_frame;
    wire [5:0]   desc_slot;
    wire [13:0]  desc_len;
    wire [15:0]  desc_ipid;
    wire [3:0]   desc_fidx;
    reg          desc_req;
    reg  [6:0]   desc_vl;
    reg          frag_upd, frag_last;
    reg  [6:0]   frag_vl;
    reg  [5:0]   frag_slot0;
    reg  [13:0]  frag_len;
    reg  [16:0]  pre_addr;
    wire [7:0]   pre_data;
    reg  [5:0]   nx_ra;
    wire [5:0]   nx_rd;
    wire [127:0] elig_vec;
    reg          fs_man;                  // S3 直测手动脉冲
    reg  [6:0]   fs_vl_man;
    reg          fs_p_r;
    reg  [6:0]   fs_v_r;
    wire         fs_pulse = fs_p_r;
    wire [6:0]   fs_vl    = fs_v_r;

    host_arb #(.NUM_HOST(1), .IW(1)) u_harb (
        .clk(clk), .reset(reset),
        .h_valid(ah_valid), .h_sop(ah_sop), .h_eop(ah_eop),
        .h_data(ah_data), .h_vl(ah_vl), .h_ready(ah_ready),
        .o_valid(htx_valid), .o_sop(htx_sop), .o_eop(htx_eop),
        .o_data(htx_data), .o_vl(htx_vl), .o_ready(htx_ready)
    );

    buf_mgr u_bm (
        .clk(clk), .reset(reset),
        .htx_valid(htx_valid), .htx_ready(htx_ready),
        .htx_sop(htx_sop), .htx_eop(htx_eop), .htx_data(htx_data), .htx_vl(htx_vl),
        .htx_abort(1'b0),                   // 单测无核级满丢，恒 0（避免悬空 Z→X）
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .q_nonempty(q_nonempty),
        .desc_req(desc_req), .desc_vl(desc_vl),
        .desc_vld(desc_vld), .desc_slot(desc_slot), .desc_len(desc_len),
        .desc_ipid(desc_ipid), .desc_fidx(desc_fidx),
        .frag_upd(frag_upd), .frag_last(frag_last), .frag_vl(frag_vl),
        .frag_slot0(frag_slot0), .frag_len(frag_len),
        .pre_addr(pre_addr), .pre_data(pre_data),
        .nx_ra(nx_ra), .nx_rd(nx_rd),
        .ev_enq(ev_enq), .ev_qfull(ev_qfull), .ev_lenerr(ev_lenerr), .ev_frame(ev_frame)
    );

    tx_regulator u_reg (
        .clk(clk), .reset(reset), .tick_us(tick_us),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .fs_pulse(fs_pulse), .fs_vl(fs_vl),
        .eligible(elig_vec)
    );

    wire [127:0] req_vec = elig_vec & q_nonempty & {128{g_tx_en}};

    wire         task_start, task_done, st_frame;
    wire [6:0]   task_vl, st_vl;
    wire [13:0]  st_paylen;
    wire         macA_start, macB_start;

    reg          bld_busy;
    reg          fr_r;
    reg  [13:0]  fr_paylen;
    reg          macA_done, macB_done;

    // 核级连线（T6-006 同）：任务完成 → BAG 重启；S3 直测走手动通道
    always @(*) begin
        fs_p_r  = task_done || fs_man;
        fs_v_r  = task_done ? st_vl : fs_vl_man;
    end

    tx_scheduler u_sched (
        .clk(clk), .reset(reset),
        .req(req_vec),
        .task_start(task_start), .task_vl(task_vl), .bld_busy(bld_busy),
        .frame_ready(fr_r), .fr_mask(2'b11), .fr_paylen(fr_paylen),
        .macA_start(macA_start), .macA_done(macA_done),
        .macB_start(macB_start), .macB_done(macB_done),
        .task_done(task_done),
        .st_frame(st_frame), .st_vl(st_vl), .st_paylen(st_paylen)
    );

    reg         sn_re;
    reg  [6:0]  sn_vl;
    wire        sn_vld;
    wire [7:0]  sn_val;

    tx_sn_gen u_sn (
        .clk(clk), .reset(reset),
        .sn_re(sn_re), .sn_vl(sn_vl), .sn_vld(sn_vld), .sn_val(sn_val)
    );

    stats u_st (
        .clk(clk), .reset(reset), .tick_us(tick_us),
        .elig(req_vec),
        .st_frame(st_frame), .st_vl(st_vl), .st_paylen(st_paylen),
        .ev_enq(ev_enq), .ev_qfull(ev_qfull), .ev_lenerr(ev_lenerr),
        .stats_re(stats_re), .stats_raddr(stats_raddr), .stats_rdata(stats_rdata),
        .ri_we(ri_we && (ri_sel == 3'd2 || ri_sel == 3'd3 || ri_sel == 3'd4)),
        .ri_sel(ri_sel), .ri_addr(ri_addr)
    );

    //-------------------------------------------------------------------------
    // 组帧器/mac 桩：task_start → 取 SN → 12 拍后 frame_ready(paylen=100)；
    // macA/B_start → 6 拍后 done。SN 期望按每 VL 独立计数核对。
    //-------------------------------------------------------------------------
    reg [7:0] st_fr_cnt;
    reg [7:0] st_a_cnt, st_b_cnt;
    reg [7:0] sn_cnt4 [0:3];           // VL10..13 已见 SN 数（= 已发帧数）
    reg [7:0] task_cnt;                // 已见 task_start 数（桩侧计数）
    reg [7:0] vl_log [0:15];           // task_start 次序（桩侧记录）

    always @(posedge clk) begin
        if (reset) begin
            st_fr_cnt <= 0; fr_r <= 0; sn_re <= 0; sn_vl <= 0;
            st_a_cnt <= 0; st_b_cnt <= 0; macA_done <= 0; macB_done <= 0;
        end else begin
            fr_r <= 0; sn_re <= 0; macA_done <= 0; macB_done <= 0;
            if (task_start) begin
                st_fr_cnt <= 8'd1;
                sn_re     <= 1'b1;
                sn_vl     <= task_vl;
            end else if (st_fr_cnt != 0) begin
                st_fr_cnt <= st_fr_cnt + 8'd1;
                if (st_fr_cnt == 8'd11) begin
                    fr_r      <= 1'b1;
                    fr_paylen <= 14'd100;
                    st_fr_cnt <= 0;
                end
            end
            if (macA_start) st_a_cnt <= 8'd1;
            else if (st_a_cnt != 0) begin
                st_a_cnt <= st_a_cnt + 8'd1;
                if (st_a_cnt == 8'd5) begin
                    macA_done <= 1'b1;
                    st_a_cnt  <= 0;
                end
            end
            if (macB_start) st_b_cnt <= 8'd1;
            else if (st_b_cnt != 0) begin
                st_b_cnt <= st_b_cnt + 8'd1;
                if (st_b_cnt == 8'd5) begin
                    macB_done <= 1'b1;
                    st_b_cnt  <= 0;
                end
            end
            // task_start 可靠记录（主循环在 pop 窗口内会漏采单拍 task_start）
            if (task_start) begin
                vl_log[task_cnt] <= task_vl;
                task_cnt         <= task_cnt + 8'd1;
            end
            // SN 核对：每 VL 第 k 帧 SN=k（复位首帧 0，之后递增）
            if (sn_vld) begin
                if (sn_vl >= 7'd10 && sn_vl <= 7'd13) begin
                    if (sn_val !== sn_cnt4[sn_vl - 7'd10]) begin
                        $display("[SCH-TB] FAIL: SN VL%0d got %0d exp %0d",
                                 sn_vl, sn_val, sn_cnt4[sn_vl - 7'd10]);
                        errors = errors + 1;
                    end
                    sn_cnt4[sn_vl - 7'd10] <= sn_cnt4[sn_vl - 7'd10] + 8'd1;
                end
            end
        end
    end

    //-------------------------------------------------------------------------
    // 任务
    //-------------------------------------------------------------------------
    integer guard, m;
    reg [31:0] g_rd;

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

    task cfg_read;
        input [19:0] a;
        output [31:0] rd;
        begin
            @(negedge clk);
            cfg_addr = a; cfg_re = 1'b1;
            @(posedge clk); #1;
            rd = cfg_rdata;
            @(negedge clk);
            cfg_re = 1'b0;
        end
    endtask

    task send_htx;                      // 经 host_arb 到 buf_mgr
        input [6:0]  vl;
        input integer len;
        integer kb, wm;
        begin
            for (kb = 0; kb < len; kb = kb + 1) begin
                @(negedge clk);
                ah_valid = 1'b1;
                ah_sop   = (kb == 0);
                ah_eop   = (kb == len - 1);
                ah_vl    = vl;
                ah_data  = (kb * 7) & 8'hFF;
                if (kb == 0) begin
                    // 就绪判定须在传输发生前：#1 等组合稳定后读 ready。
                    // sop 传输后 enq_active 置位，此时再读 ready（sop 仍高）
                    // 会得到 0——那是"新 sop"的判定，不是本字节的。
                    #1;
                    wm = 0;
                    while (ah_ready !== 1'b1 && wm < 20) begin
                        @(negedge clk); #1;
                        wm = wm + 1;
                    end
                    chk(wm < 20, "htx sop accepted");
                    if (wm >= 20) begin             // 被拒：撤激励中止
                        @(negedge clk);
                        ah_valid = 0; ah_sop = 0; ah_eop = 0;
                        kb = 1 << 30;
                    end
                end
            end
            @(negedge clk);
            ah_valid = 0; ah_sop = 0; ah_eop = 0;
        end
    endtask

    task read_desc;
        input [6:0] vl;
        integer wm;
        begin
            @(negedge clk);
            desc_req = 1; desc_vl = vl;
            @(negedge clk); desc_req = 0;
            wm = 0;
            while (desc_vld !== 1'b1 && wm < 10) begin
                @(posedge clk); #1; wm = wm + 1;
            end
            chk(wm < 10, "desc_vld seen");
        end
    endtask

    task pop_msg;
        input [6:0] vl;
        input [5:0] s0;
        input [13:0] l;
        begin
            @(negedge clk);
            frag_upd = 1; frag_last = 1; frag_vl = vl; frag_slot0 = s0; frag_len = l;
            @(negedge clk);
            frag_upd = 0; frag_last = 0;
        end
    endtask

    //-------------------------------------------------------------------------
    // S2: host_arb N=2 独立例化（模块级连线 + 任务驱动）
    //-------------------------------------------------------------------------
    reg  [1:0]  a2_v, a2_s, a2_e;
    reg  [15:0] a2_d;                   // {客户1字节, 客户0字节}
    reg  [13:0] a2_vl;                  // {客户1 vl, 客户0 vl}
    reg         a2_rdy = 1'b1;
    wire        a2_o_v, a2_o_s, a2_o_e;
    wire [7:0]  a2_o_d;
    wire [6:0]  a2_o_l;
    wire [1:0]  a2_rdy_v;

    host_arb #(.NUM_HOST(2), .IW(1)) u_arb2 (
        .clk(clk), .reset(reset),
        .h_valid(a2_v), .h_sop(a2_s), .h_eop(a2_e),
        .h_data(a2_d), .h_vl(a2_vl), .h_ready(a2_rdy_v),
        .o_valid(a2_o_v), .o_sop(a2_o_s), .o_eop(a2_o_e),
        .o_data(a2_o_d), .o_vl(a2_o_l), .o_ready(a2_rdy)
    );

    // 每客户端独立驱动进程：sop/valid 保持到获准（htx 协议语义），获准拍推进。
    // 客户0 = 2 字节包(0x11,0x22,VL21)；客户1 = 3 字节包(0x33,0x44,0x55,VL22)。
    // RR 从 last=0 的下一索引起扫 → 首轮先授客户 1。输出按传输次序核对。
    reg [7:0] c0_b [0:1];
    reg [7:0] c1_b [0:2];
    integer   c0_i, c1_i, got2;
    reg [7:0] out_b [0:7];
    reg [7:0] out_sop [0:7];
    reg [6:0] out_vl [0:7];

    // 负沿驱动：本拍 v&&rdy 将在拍末传输 → 中拍推进索引并备好下一字节
    //（DUT 在拍末采到的仍是当前字节，无重复传输）
    always @(negedge clk) begin
        if (!reset) begin
            // 先驱动本拍字节，#1 等组合（cur/h_ready）稳定后再判"本拍将传输"
            //（置位当拍直接读 rdy 是 delta 竞态，会读到旧值而抢跑）
            if (c0_i < 2) begin
                a2_v[0] = 1'b1;
                a2_s[0] = (c0_i == 0);
                a2_e[0] = (c0_i == 1);
                a2_d[7:0]   = c0_b[c0_i];
                a2_vl[6:0]  = 7'd21;
            end else begin
                a2_v[0] = 1'b0; a2_s[0] = 1'b0; a2_e[0] = 1'b0;
            end
            if (c1_i < 3) begin
                a2_v[1] = 1'b1;
                a2_s[1] = (c1_i == 0);
                a2_e[1] = (c1_i == 2);
                a2_d[15:8]  = c1_b[c1_i];
                a2_vl[13:7] = 7'd22;
            end else begin
                a2_v[1] = 1'b0; a2_s[1] = 1'b0; a2_e[1] = 1'b0;
            end
            #1;
            if (c0_i < 2 && a2_v[0] && a2_rdy_v[0])
                c0_i = c0_i + 1;
            if (c1_i < 3 && a2_v[1] && a2_rdy_v[1])
                c1_i = c1_i + 1;
        end
    end

    // 传输监视（时钟块内取前值，无采样竞争）
    integer mon_i;
    always @(posedge clk) begin
        if (!reset && a2_o_v && a2_rdy && got2 < 8) begin
            out_b[got2]   <= a2_o_d;
            out_sop[got2] <= a2_o_s;
            out_vl[got2]  <= a2_o_l;
            got2          <= got2 + 1;
        end
    end

    task standalone_arb_test;
        integer wi;
        begin
            // 期望（RR 先客户 1）：33,44,55,11,22
            for (wi = 0; wi < 40 && got2 < 5; wi = wi + 1)
                @(posedge clk);
            #1;
            chk(got2 === 5, "S2: exactly 5 bytes out");
            chk(out_b[0] === 8'h33 && out_sop[0] === 1'b1 && out_vl[0] === 7'd22,
                "S2: pkt1 b0 (grant client1 first)");
            chk(out_b[1] === 8'h44, "S2: pkt1 b1");
            chk(out_b[2] === 8'h55 && out_sop[2] === 1'b0, "S2: pkt1 b2");
            chk(out_b[3] === 8'h11 && out_sop[3] === 1'b1 && out_vl[3] === 7'd21,
                "S2: pkt0 b0 (switch to client0)");
            chk(out_b[4] === 8'h22, "S2: pkt0 b1");
        end
    endtask

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    initial begin : main
        errors = 0; reset = 1'b1;
        cfg_addr = 0; cfg_wdata = 0; cfg_we = 0; cfg_re = 0;
        ah_valid = 0; ah_sop = 0; ah_eop = 0; ah_data = 0; ah_vl = 0;
        bld_busy = 0; fr_r = 0; fr_paylen = 0;
        macA_done = 0; macB_done = 0;
        fs_man = 0; fs_vl_man = 0;
        sn_re = 0; sn_vl = 0;
        desc_req = 0; desc_vl = 0;
        frag_upd = 0; frag_last = 0; frag_vl = 0; frag_slot0 = 0; frag_len = 0;
        pre_addr = 0; nx_ra = 0;
        a2_v = 0; a2_s = 0; a2_e = 0; a2_d = 0; a2_vl = 0;
        c0_b[0] = 8'h11; c0_b[1] = 8'h22;
        c1_b[0] = 8'h33; c1_b[1] = 8'h44; c1_b[2] = 8'h55;
        c0_i = 0; c1_i = 0; got2 = 0;
        sn_cnt4[0] = 0; sn_cnt4[1] = 0; sn_cnt4[2] = 0; sn_cnt4[3] = 0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        guard = 0;
        while (init_done !== 1'b1 && guard < 50000) begin
            @(posedge clk); #1; guard = guard + 1;
        end
        chk(init_done === 1'b1, "init done");

        //---------------------------------------------------------------
        // S1: sn_gen 序列（VL7：0,1..255,1）+ VL8 独立
        //---------------------------------------------------------------
        for (m = 0; m < 257; m = m + 1) begin
            @(negedge clk);
            sn_re = 1; sn_vl = 7'd7;
            @(posedge clk); #1;
            if (m == 0)          g_rd = 32'd0;
            else if (m <= 255)   g_rd = m;
            else                 g_rd = 32'd1;      // 255 后回绕到 1
            if (sn_val !== g_rd[7:0]) begin
                $display("[SCH-TB] FAIL: S1 SN step %0d got %0d exp %0d",
                         m, sn_val, g_rd[7:0]);
                errors = errors + 1;
            end
        end
        @(negedge clk); sn_re = 0;
        @(negedge clk); sn_re = 1; sn_vl = 7'd8;
        @(posedge clk); #1;
        chk(sn_val === 8'd0, "S1: VL8 independent (first SN=0)");
        @(negedge clk); sn_re = 0;

        //---------------------------------------------------------------
        // S2: host_arb N=2
        //---------------------------------------------------------------
        standalone_arb_test;

        //---------------------------------------------------------------
        // S3: regulator 直测（VL2）
        //---------------------------------------------------------------
        cfg_write(20'h01000 + (2 << 3) + 0, 32'h8000_0002);   // en=1 bag=2ms
        #1;
        chk(elig_vec[2] === 1'b1, "S3: eligible immediately after W0");
        cfg_write(20'h01000 + (2 << 3) + 4, 32'd700);         // phase=700us
        #1;
        chk(elig_vec[2] === 1'b0, "S3: phase reload defers eligibility");
        guard = 0;                                            // 2000-700=1300us=162500 拍
        while (elig_vec[2] !== 1'b1 && guard < 200000) begin
            @(posedge clk); guard = guard + 1;
        end
        chk(elig_vec[2] === 1'b1, "S3: eligible after phase delay");
        chk(guard >= 162500 - 200 && guard <= 162500 + 200,
            "S3: phase delay ~= reload-phase cycles");
        @(negedge clk); fs_man = 1; fs_vl_man = 7'd2;
        @(negedge clk); fs_man = 0;
        #1;
        chk(elig_vec[2] === 1'b0, "S3: fs drops eligible");
        guard = 0;                                            // 2ms = 250000 拍
        while (elig_vec[2] !== 1'b1 && guard < 260000) begin
            @(posedge clk); guard = guard + 1;
        end
        chk(guard >= 250000 - 200 && guard <= 250000 + 200,
            "S3: BAG gap ~= 2ms after fs");
        cfg_write(20'h01000 + (2 << 3) + 0, 32'h0000_0002);   // en=0
        #1;
        chk(elig_vec[2] === 1'b0, "S3: en=0 gates eligible");

        //---------------------------------------------------------------
        // S4: 集成——4 VL（10..13）各 1 条 100B 消息，BAG=1ms，跑 2 轮
        //---------------------------------------------------------------
        for (m = 0; m < 4; m = m + 1) begin
            cfg_write(20'h01000 + ((10 + m) << 3) + 0, 32'h8000_0001);
            cfg_write(20'h01000 + ((10 + m) << 3) + 1, 32'd1518);
            cfg_write(20'h01000 + ((10 + m) << 3) + 2, 32'd10 + m);
            cfg_write(20'h01000 + ((10 + m) << 3) + 3, 32'd3);
            cfg_write(20'h01000 + ((10 + m) << 3) + 5, 32'd5001);
            cfg_write(20'h01000 + ((10 + m) << 3) + 6, 32'd5002);
        end
        cfg_write(20'h00001, 32'h0A17);
        // 先入队后开 tx_en；每 VL 2 条消息 → 2 轮 × 4 VL = 8 任务
        for (m = 0; m < 4; m = m + 1) begin
            send_htx(10 + m, 100);
            send_htx(10 + m, 100);
        end
        // 调度监视（含弹出）必须先于开闸运行，否则前几个 task_done 无人弹出；
        // fork：50 拍后写 CTRL 开闸，主循环即刻开始收 task/pop
        guard = 0; task_cnt = 0;
        fork
            begin : open_gate
                repeat (50) @(posedge clk);
                cfg_write(20'h00003, 32'h1);
            end
            begin : sched_watch
                while (task_cnt < 8 && guard < 400000) begin
                    @(posedge clk); #1;
                    if (task_done && st_vl >= 7'd10 && st_vl <= 7'd13) begin
                        @(negedge clk);
                        read_desc(st_vl);        // 重读队首描述符（二轮消息 slot 不同）
                        pop_msg(st_vl, desc_slot, desc_len);
                    end
                    guard = guard + 1;
                end
            end
        join
        chk(task_cnt === 8'd8, "S4: 8 tasks in 2 rounds");
        repeat (200) @(posedge clk);   // 等末帧 task_done + stats RMW 完成再读数
        // 开闸时 4 VL 同时 eligible：RR 自 last+1 起扫 → 首轮 10,11,12,13；
        // 二轮按各 VL BAG 到期先后（轮内同序）→ 10,11,12,13（轮转无饥饿）
        chk(vl_log[0] === 7'd10 && vl_log[1] === 7'd11 &&
            vl_log[2] === 7'd12 && vl_log[3] === 7'd13,
            "S4: round1 RR order 10,11,12,13");
        chk(vl_log[4] === 7'd10 && vl_log[5] === 7'd11 &&
            vl_log[6] === 7'd12 && vl_log[7] === 7'd13,
            "S4: round2 RR order 10,11,12,13");
        cfg_read(20'h203C0, g_rd);                     // g_frames
        chk(g_rd === 32'd8, "S4: g_tx_frames==8");
        cfg_read(20'h203C2, g_rd);                     // g_msgs
        chk(g_rd === 32'd8, "S4: g_msgs==8");
        cfg_read(20'h203C3, g_rd);
        chk(g_rd === 32'd0, "S4: g_qfull==0");
        cfg_read(20'h203C4, g_rd);
        chk(g_rd === 32'd0, "S4: g_lenerr==0");
        for (m = 0; m < 4; m = m + 1) begin
            cfg_read(20'h20000 + ((10 + m) << 3), g_rd);
            chk(g_rd === 32'd2, "S4: per-VL frames==2");
            cfg_read(20'h20000 + ((10 + m) << 3) + 2, g_rd);
            chk(g_rd === 32'd200, "S4: per-VL bytes==200");
            cfg_read(20'h20000 + ((10 + m) << 3) + 4, g_rd);
            chk(g_rd <= 32'd5, "S4: jitter max <= 5us (stub latencies)");
        end

        //---------------------------------------------------------------
        // 结论
        //---------------------------------------------------------------
        if (errors == 0)
            $display("[SCH-TB] PASS");
        else
            $display("[SCH-TB] FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #100_000_000;
        $display("[SCH-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
