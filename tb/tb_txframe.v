//=============================================================================
// tb_txframe.v — T6-005 发送通路全帧对拍 tb（afdx_es_core 集成前形态）
// 链路：tb 驱动 htx（应用载荷，VLID 已剥离）→ host_arb → buf_mgr →
//       regulator/scheduler → sn_gen + tx_frame_builder → s_stream →
//       mac_tx×2（锁步，IF_ID 0x20/0x40 + 各自 FCS）→ mac_port_if → tb 抓帧。
// 判定：tb 自检（帧数/前导/SFD）+ 落盘 frame_out_{A,B}.hex/lens 与
//       frame_times_{A,B}.txt → scripts/check_output.py 与 afdx_ref 金标准
//       逐字节比对 + FCS/IP 校验和独立核算 + BAG/0.5ms 时序检查。
// 激励与配置来自 scripts/gen_stim.py tx1（msg_tx.hex / msg_lens.txt /
//       vl_cfg.txt / golden_{A,B}.hex / frame_{A,B}.lens / case.json）。
//=============================================================================

`timescale 1ns/1ps

module tb_txframe;

    parameter CLK_HALF = 4;
    // errors 由 `include "afdx_if.v" 声明（公共任务集作用域）

    reg clk, reset;
    initial clk = 1'b0;
    always #CLK_HALF clk = ~clk;

    //-------------------------------------------------------------------------
    // 模块间连线（先声明后使用；afdx_if 在声明后 include）
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    wire [6:0]  bld_rd_idx;
    wire [2:0]  bld_rd_wsel;
    wire [31:0] bld_rd_data;
    wire        bld_desc_req, bld_desc_vld;
    wire [6:0]  bld_desc_vl;
    wire [10:0] fr_body_len;
    wire [13:0] fr_paylen;
    wire [1:0]  fr_mask;
    wire        frame_ready, bld_busy;
    wire        task_start, task_done, st_frame;
    wire [6:0]  task_vl, st_vl;
    wire [13:0] st_paylen;
    wire        macA_start, macB_start, macA_done, macB_done;
    wire        macA_sready, macB_sready;
    wire        s_valid, s_sop, s_eop;
    wire [7:0]  s_data;
    wire        sn_re_w, sn_vld_w;
    wire [6:0]  sn_vl_w;
    wire [7:0]  sn_val_w;

    // afdx_if 的 drive_app_msg/check_frame_header 需要的宿主信号（本 tb 不用，占位）
    reg         tx_valid;
    reg  [7:0]  tx_data;
    integer     tx_n, gold_n, ml_n, fl_n;   // 加载任务输出（afdx_if 约定）
    integer     i0;
    reg [1023:0] casedir;
    reg [2047:0] path;

    `include "afdx_if.v"

    task chk;
        input cond;
        input [511:0] name;
        begin
            if (cond !== 1'b1) begin
                $display("[TX-TB] FAIL: %0s", name);
                errors = errors + 1;
            end
        end
    endtask


    //-------------------------------------------------------------------------
    // cfg/公共
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
    wire        stats_re;
    wire [9:0]  stats_raddr;
    wire [31:0] stats_rdata;
    wire        init_busy, init_done;
    wire [2:0]  ri_sel;
    wire [15:0] ri_addr;
    wire        ri_we;
    wire        tick_us;
    wire [63:0] time_ns;

    cfg_bus u_cfg (
        .clk(clk), .reset(reset),
        .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata), .cfg_we(cfg_we), .cfg_re(cfg_re),
        .cfg_rdata(cfg_rdata), .cfg_rvalid(cfg_rvalid),
        .g_uid(g_uid), .g_src_ip(g_src_ip), .g_tx_en(g_tx_en),
        .init_done(init_done),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .tbl_re(tbl_re), .tbl_raddr(tbl_raddr), .tbl_rdata(tbl_rdata), .tbl_busy(1'b0),
        .stats_re(stats_re), .stats_raddr(stats_raddr), .stats_rdata(stats_rdata)
    );

    tx_vl_table u_tbl (
        .clk(clk),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .tbl_re(tbl_re), .tbl_raddr(tbl_raddr), .tbl_busy(1'b0), .tbl_rdata(tbl_rdata),
        .rd_idx(bld_rd_idx), .rd_wsel(bld_rd_wsel), .rd_data(bld_rd_data),
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
    // 发送链
    //-------------------------------------------------------------------------
    reg          ah_valid, ah_sop, ah_eop;
    reg  [7:0]   ah_data;
    reg  [6:0]   ah_vl;
    wire         ah_ready;
    wire         htx_valid, htx_sop, htx_eop, htx_ready;
    wire [7:0]   htx_data;
    wire [6:0]   htx_vl;
    wire [127:0] q_nonempty;
    wire         ev_enq, ev_qfull, ev_lenerr, ev_frame;
    wire [5:0]   desc_slot;
    wire [13:0]  desc_len;
    wire [15:0]  desc_ipid;
    wire [3:0]   desc_fidx;
    wire [16:0]  bld_pre_addr;
    wire [7:0]   bld_pre_data;
    wire [5:0]   bld_nx_ra, bld_nx_rd;
    wire         frag_upd, frag_last;
    wire [6:0]   frag_vl;
    wire [5:0]   frag_slot0;
    wire [13:0]  frag_len;
    wire [127:0] elig_vec;
    wire         fs_pulse;
    wire [6:0]   fs_vl;
    wire [127:0] req_vec;

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
        .desc_req(bld_desc_req), .desc_vl(bld_desc_vl),
        .desc_vld(bld_desc_vld), .desc_slot(desc_slot), .desc_len(desc_len),
        .desc_ipid(desc_ipid), .desc_fidx(desc_fidx),
        .frag_upd(frag_upd), .frag_last(frag_last), .frag_vl(frag_vl),
        .frag_slot0(frag_slot0), .frag_len(frag_len),
        .pre_addr(bld_pre_addr), .pre_data(bld_pre_data),
        .nx_ra(bld_nx_ra), .nx_rd(bld_nx_rd),
        .ev_enq(ev_enq), .ev_qfull(ev_qfull), .ev_lenerr(ev_lenerr), .ev_frame(ev_frame)
    );

    tx_regulator u_reg (
        .clk(clk), .reset(reset), .tick_us(tick_us),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .fs_pulse(fs_pulse), .fs_vl(fs_vl),
        .eligible(elig_vec)
    );

    assign req_vec = elig_vec & q_nonempty & {128{g_tx_en}};

    tx_scheduler u_sched (
        .clk(clk), .reset(reset),
        .req(req_vec),
        .task_start(task_start), .task_vl(task_vl), .bld_busy(bld_busy),
        .frame_ready(frame_ready), .fr_mask(fr_mask), .fr_paylen(fr_paylen),
        .macA_start(macA_start), .macA_done(macA_done),
        .macB_start(macB_start), .macB_done(macB_done),
        .task_done(task_done),
        .st_frame(st_frame), .st_vl(st_vl), .st_paylen(st_paylen)
    );

    // 组帧器 s_ready：按当前帧 mask 取使能网的 BODY 就绪（未使能网不阻塞）
    wire        bld_s_ready = (fr_mask[0] ? macA_sready : 1'b1)
                            & (fr_mask[1] ? macB_sready : 1'b1);

    tx_sn_gen u_sn (
        .clk(clk), .reset(reset),
        .sn_re(sn_re_w), .sn_vl(sn_vl_w), .sn_vld(sn_vld_w), .sn_val(sn_val_w)
    );

    tx_frame_builder u_bld (
        .clk(clk), .reset(reset),
        .task_start(task_start), .task_vl(task_vl),
        .bld_busy(bld_busy), .frame_ready(frame_ready),
        .fr_body_len(fr_body_len), .fr_paylen(fr_paylen), .fr_mask(fr_mask),
        .rd_idx(bld_rd_idx), .rd_wsel(bld_rd_wsel), .rd_data(bld_rd_data),
        .desc_req(bld_desc_req), .desc_vl(bld_desc_vl),
        .desc_vld(bld_desc_vld), .desc_slot(desc_slot), .desc_len(desc_len),
        .desc_ipid(desc_ipid), .desc_fidx(desc_fidx),
        .pre_addr(bld_pre_addr), .pre_data(bld_pre_data),
        .nx_ra(bld_nx_ra), .nx_rd(bld_nx_rd),
        .frag_upd(frag_upd), .frag_last(frag_last), .frag_vl(frag_vl),
        .frag_slot0(frag_slot0), .frag_len(frag_len),
        .sn_re(sn_re_w), .sn_vl(sn_vl_w), .sn_vld(sn_vld_w), .sn_val(sn_val_w),
        .g_uid(g_uid), .g_src_ip(g_src_ip),
        .s_valid(s_valid), .s_sop(s_sop), .s_eop(s_eop), .s_data(s_data),
        .s_ready(bld_s_ready)
    );

    wire [7:0]  macA_dout, macB_dout;     // mac_tx 原始输出
    wire        macA_enout, macB_enout;
    wire [7:0]  gmii_a_d, gmii_b_d;       // mac_port_if 观测侧
    wire        gmii_a_en, gmii_b_en;
    wire        pif_a_s, pif_a_e, pif_b_s, pif_b_e;

    mac_tx #(.IF_ID(3'b001), .IF_ID_BYTE(8'h20)) u_macA (
        .clk(clk), .reset(reset),
        .start(macA_start), .body_len(fr_body_len),
        .s_valid(s_valid), .s_ready(macA_sready), .s_sop(s_sop), .s_eop(s_eop),
        .s_data(s_data),
        .gmii_d(macA_dout), .gmii_en(macA_enout), .done(macA_done)
    );
    mac_tx #(.IF_ID(3'b010), .IF_ID_BYTE(8'h40)) u_macB (
        .clk(clk), .reset(reset),
        .start(macB_start), .body_len(fr_body_len),
        .s_valid(s_valid), .s_ready(macB_sready), .s_sop(s_sop), .s_eop(s_eop),
        .s_data(s_data),
        .gmii_d(macB_dout), .gmii_en(macB_enout), .done(macB_done)
    );

    mac_port_if #(.NET_ID(2'b01)) u_pifA (
        .clk(clk), .reset(reset),
        .tx_d_in(macA_dout), .tx_en_in(macA_enout),
        .gmii_d(gmii_a_d), .gmii_en(gmii_a_en), .gmii_er(),
        .frame_start(pif_a_s), .frame_end(pif_a_e)
    );
    mac_port_if #(.NET_ID(2'b10)) u_pifB (
        .clk(clk), .reset(reset),
        .tx_d_in(macB_dout), .tx_en_in(macB_enout),
        .gmii_d(gmii_b_d), .gmii_en(gmii_b_en), .gmii_er(),
        .frame_start(pif_b_s), .frame_end(pif_b_e)
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

    // 任务完成 → BAG 重启（核级连线）
    assign fs_pulse = task_done;
    assign fs_vl    = st_vl;




    //-------------------------------------------------------------------------
    // GMII 抓帧（A/B 各自缓存；校验前导/SFD；记录起始周期）
    //-------------------------------------------------------------------------
    reg [7:0]  capA [0:65535];
    reg [7:0]  capB [0:65535];
    integer    capA_n, capB_n;
    integer    capA_frames, capB_frames;
    integer    capA_len [0:511];
    integer    capB_len [0:511];
    integer    capA_base, capB_base;    // 帧起始时的累计字节数（帧长=差值）
    reg [31:0] capA_cyc [0:511];
    reg [31:0] capB_cyc [0:511];
    integer    capA_idx, capB_idx;      // 帧内字节序号（含前导 8B）
    reg        capA_in, capB_in;
    reg [31:0] cyc_cnt;

    // 单网采样（g=0 A / 1 B）
    task cap_sample;
        input       g;
        input [7:0] d;
        input       en;
        begin
            if (g == 0) begin
                if (en && !capA_in) begin
                    capA_in  = 1;
                    capA_idx = 0;
                    capA_cyc[capA_frames] = cyc_cnt;
                end
                if (capA_in && en) begin   // 仅 TX_EN 有效拍采样（REV-002-01）
                    if (capA_idx < 8) begin
                        if (capA_idx < 7) begin
                            if (d !== 8'h55) begin
                                $display("[TX-TB] FAIL: A 前导@%0d = %02x",
                                         capA_idx, d);
                                errors = errors + 1;
                            end
                        end else if (d !== 8'hD5) begin
                            $display("[TX-TB] FAIL: A SFD = %02x", d);
                            errors = errors + 1;
                        end
                    end else begin
                        capA[capA_n] = d;
                        capA_n = capA_n + 1;
                    end
                    capA_idx = capA_idx + 1;
                end
                if (!en && capA_in) begin
                    capA_in = 0;
                    capA_len[capA_frames] = capA_n - capA_base;
                    capA_base = capA_n;
                    capA_frames = capA_frames + 1;
                end
            end else begin
                if (en && !capB_in) begin
                    capB_in  = 1;
                    capB_idx = 0;
                    capB_cyc[capB_frames] = cyc_cnt;
                end
                if (capB_in && en) begin   // 仅 TX_EN 有效拍采样（REV-002-01）
                    if (capB_idx < 8) begin
                        if (capB_idx < 7) begin
                            if (d !== 8'h55) begin
                                $display("[TX-TB] FAIL: B 前导@%0d = %02x",
                                         capB_idx, d);
                                errors = errors + 1;
                            end
                        end else if (d !== 8'hD5) begin
                            $display("[TX-TB] FAIL: B SFD = %02x", d);
                            errors = errors + 1;
                        end
                    end else begin
                        capB[capB_n] = d;
                        capB_n = capB_n + 1;
                    end
                    capB_idx = capB_idx + 1;
                end
                if (!en && capB_in) begin
                    capB_in = 0;
                    capB_len[capB_frames] = capB_n - capB_base;
                    capB_base = capB_n;
                    capB_frames = capB_frames + 1;
                end
            end
        end
    endtask

    always @(posedge clk) begin
        if (reset) begin
            cyc_cnt <= 0;
            capA_in <= 0; capB_in <= 0;
        end else begin
            cyc_cnt <= cyc_cnt + 1;
            cap_sample(0, gmii_a_d, gmii_a_en);
            cap_sample(1, gmii_b_d, gmii_b_en);
        end
    end

    //-------------------------------------------------------------------------
    // 配置/激励任务
    //-------------------------------------------------------------------------
    integer guard, m, n, vl_idx;
    reg [31:0] w_en, w_bag, w_smax, w_vlid, w_mask, w_phase, w_sport, w_dport;
    integer cfg_fd;

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

    task send_htx;
        input [6:0]  vl;
        input integer base;      // msg_mem 内载荷起点（跳过 2B VLID）
        input integer len;       // 载荷字节数 = msg_len-2
        integer kb, wm;
        begin
            for (kb = 0; kb < len; kb = kb + 1) begin
                @(negedge clk);
                ah_valid = 1'b1;
                ah_sop   = (kb == 0);
                ah_eop   = (kb == len - 1);
                ah_vl    = vl;
                ah_data  = msg_mem[base + kb];
                if (kb == 0) begin
                    #1;
                    wm = 0;
                    while (ah_ready !== 1'b1 && wm < 20) begin
                        @(negedge clk); #1;
                        wm = wm + 1;
                    end
                    chk(wm < 20, "htx sop accepted");
                    if (wm >= 20) begin
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

    //-------------------------------------------------------------------------
    // 落盘（hex 每行一字节；lens 每行一整数；times 每行一起始周期）
    //-------------------------------------------------------------------------
    task dump_net;
        input g;
        integer fd, i;
        reg [1023:0] f_hex, f_len, f_tim;
        begin
            if (g == 0) begin
                f_hex = {casedir, "/frame_out_A.hex"};
                f_len = {casedir, "/frame_out_A.lens"};
                f_tim = {casedir, "/frame_times_A.txt"};
                fd = $fopen(f_hex, "w");
                for (i = 0; i < capA_n; i = i + 1)
                    $fdisplay(fd, "%h", capA[i]);
                $fclose(fd);
                fd = $fopen(f_len, "w");
                for (i = 0; i < capA_frames; i = i + 1)
                    $fdisplay(fd, "%0d", capA_len[i]);
                $fclose(fd);
                fd = $fopen(f_tim, "w");
                for (i = 0; i < capA_frames; i = i + 1)
                    $fdisplay(fd, "%0d", capA_cyc[i]);
                $fclose(fd);
            end else begin
                f_hex = {casedir, "/frame_out_B.hex"};
                f_len = {casedir, "/frame_out_B.lens"};
                f_tim = {casedir, "/frame_times_B.txt"};
                fd = $fopen(f_hex, "w");
                for (i = 0; i < capB_n; i = i + 1)
                    $fdisplay(fd, "%h", capB[i]);
                $fclose(fd);
                fd = $fopen(f_len, "w");
                for (i = 0; i < capB_frames; i = i + 1)
                    $fdisplay(fd, "%0d", capB_len[i]);
                $fclose(fd);
                fd = $fopen(f_tim, "w");
                for (i = 0; i < capB_frames; i = i + 1)
                    $fdisplay(fd, "%0d", capB_cyc[i]);
                $fclose(fd);
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // 主流程
    //-------------------------------------------------------------------------
    integer exp_frames;
    integer done_wait;

    initial begin : main
        errors = 0; reset = 1'b1;
        cfg_addr = 0; cfg_wdata = 0; cfg_we = 0; cfg_re = 0;
        ah_valid = 0; ah_sop = 0; ah_eop = 0; ah_data = 0; ah_vl = 0;
        capA_n = 0; capB_n = 0; capA_frames = 0; capB_frames = 0;
        capA_idx = 0; capB_idx = 0; capA_in = 0; capB_in = 0;
        capA_base = 0; capB_base = 0;
        if (!$value$plusargs("CASE_DIR=%s", casedir))
            casedir = "sim/tb_out/tx1";

        repeat (4) @(posedge clk);
        @(negedge clk);
        reset = 1'b0;
        guard = 0;
        while (init_done !== 1'b1 && guard < 50000) begin
            @(posedge clk); #1; guard = guard + 1;
        end
        chk(init_done === 1'b1, "init done");

        // 1) 加载激励/金标准
        path = {casedir, "/msg_tx.hex"};
        load_msg_file(path, tx_n);
        path = {casedir, "/msg_lens.txt"};
        load_msg_lens(path, ml_n);
        path = {casedir, "/golden_A.hex"};
        load_gold_file(path, gold_n);
        path = {casedir, "/frame_A.lens"};
        load_gold_lens(path, fl_n);
        exp_frames = fl_n;
        $display("[TX-TB] loaded: msgs=%0d frames=%0d tx_bytes=%0d",
                 ml_n, exp_frames, tx_n);
        chk(ml_n > 0 && fl_n > 0, "case files non-empty");

        // 2) vl_cfg.txt → VL 表（列：idx en bag_ms smax vlid mask phase sport dport）
        path = {casedir, "/vl_cfg.txt"};
        cfg_fd = $fopen(path, "r");
        chk(cfg_fd != 0, "vl_cfg.txt opened");
        if (cfg_fd != 0) begin
            while ($fscanf(cfg_fd, "%d %d %d %d %d %d %d %d %d",
                           vl_idx, w_en, w_bag, w_smax, w_vlid, w_mask,
                           w_phase, w_sport, w_dport) == 9) begin
                // W4(相位) 先写，W0(en/bag) 最后写——W0 装载"配置即到期"
                cfg_write(20'h01000 + (vl_idx << 3) + 4, {16'h0, w_phase[15:0]});
                cfg_write(20'h01000 + (vl_idx << 3) + 1, {16'h0, w_smax[15:0]});
                cfg_write(20'h01000 + (vl_idx << 3) + 2, {16'h0, w_vlid[15:0]});
                cfg_write(20'h01000 + (vl_idx << 3) + 3, {30'h0, w_mask[1:0]});
                cfg_write(20'h01000 + (vl_idx << 3) + 5, {16'h0, w_sport[15:0]});
                cfg_write(20'h01000 + (vl_idx << 3) + 6, {16'h0, w_dport[15:0]});
                cfg_write(20'h01000 + (vl_idx << 3) + 0,
                          (w_en != 0 ? 32'h8000_0000 : 32'h0) | {24'h0, w_bag[7:0]});
            end
            $fclose(cfg_fd);
        end
        cfg_write(20'h00001, 32'h00000A17);      // UID（与 gen_stim 一致）
        cfg_write(20'h00002, 32'h0A000205);      // SRC_IP = 10.0.2.5
        // CTRL 暂不开：先入队后开闸

        // 3) 全部消息入队（msg = VLID 2B + 载荷；htx 只送载荷，vl = VLID[6:0]）
        n = 0;
        m = 0;
        while (m < ml_n) begin
            send_htx(msg_mem[n + 1][6:0], n + 2, msg_len[m] - 2);
            n = n + msg_len[m];
            m = m + 1;
        end
        chk(m == ml_n, "all messages injected");
        $display("[TX-TB] injected %0d messages", m);

        // 4) 开闸 + 等两网帧数均达期望
        fork
            begin
                repeat (50) @(posedge clk);
                cfg_write(20'h00003, 32'h1);
            end
            begin
                guard = 0;
                done_wait = exp_frames;   // 每网帧数（非两网合计）
                while ((capA_frames < done_wait || capB_frames < done_wait)
                       && guard < 2_500_000) begin
                    @(posedge clk);
                    guard = guard + 1;
                end
            end
        join
        chk(capA_frames >= done_wait && capB_frames >= done_wait,
            "all frames captured on both nets");
        $display("[TX-TB] captured A=%0d B=%0d (exp %0d/net)",
                 capA_frames, capB_frames, exp_frames);
        repeat (2000) @(posedge clk);            // 末帧释放/统计收尾

        // 5) 落盘（case 目录）
        dump_net(0);
        dump_net(1);
        $display("[TX-TB] outputs written to case dir");

        if (errors == 0)
            $display("[TX-TB] PASS");
        else
            $display("[TX-TB] FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #100_000_000;
        $display("[TX-TB] FAIL: watchdog timeout");
        $finish;
    end

endmodule
