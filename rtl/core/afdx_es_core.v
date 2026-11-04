//=============================================================================
// afdx_es_core.v — 协议核心顶层连线（v2 §4.2 core；doc/02 §2 发送链）
//
// 发送链：应用口信封（tx_valid 上升沿→下降沿 = 一条消息，前 2 字节 VLID 大端）
//   → 信封解析器（剥离 VLID → htx 流）→ host_arb(1) → buf_mgr →
//   regulator/scheduler → sn_gen + tx_frame_builder → s_stream →
//   mac_tx×2（锁步，IF_ID 0x20/0x40 各自 FCS）→ mac_port_if×2 → GMII 观测口。
// 接收链（T7 落地）：GMII 收流 reg（tb 层级驱动）→ mac_rx×2 → rx_parser →
//   rx_vl_lookup（cfg 0x10000 区）→ rx_integrity → rx_redundancy →
//   rx_reassembly → hrx → 应用口信封交付。管理平面（含 64 口 CAM 分发）：P1。
//
// 信封解析器要点：
//   - 1 拍流水：载荷字节 k 在 k+1 拍进吸收 FIFO；htx_eop 为组合量
//     （!tx_valid——信封在本拍已落 = 上一字节是末字节），随字节一起入队，
//     末字节携 eop，无误增字节；
//   - htx_vl = VLID[6:0]（发送 VL 表索引；帧头 16b VLID 取表 W2）；
//   - 应用口无反压（doc/05）：8B 吸收 FIFO 承接 htx_ready 瞬态（队列非满时
//     仅为释放行走 1-4 拍）；FIFO 将溢出（队列真满）→ 丢弃整条消息：清 FIFO、
//     向 buf_mgr 发 htx_abort（半截入队回收，无描述符落环），待信封结束恢复。
//     计数路径：丢弃经 buf_mgr ev_qfull 口径之外的 abort 事件（T6 计入
//     stats 前先旁路，T7 一并整理）。
//   - 纯 VLID 信封（0 载荷）不合 AFDX（载荷 ≥1B），静默丢弃。
//
// cfg 主侧四信号（cfg_addr/cfg_wdata/cfg_we/cfg_re）为 input wire：顶层主机
// 配置接口（VERDICT-015 交付口径，AFDX_top cfg_* 引脚直入）。T6/T7 时期为
// output reg + tb 层级驱动——纯综合下零驱动致 g_tx_en≡0、req_vec≡0，整机被
// eLinx 判 stuck 整族删除（网表空心，RPT-T8-007/008 定案）——故升为真实引脚。
//=============================================================================

`define AFDX_MAC_IP 1
`define AFDX_MAC_LOOPBACK 1

module afdx_es_core #(
    parameter integer TX_VL_NUM  = 128,
    parameter integer RX_VL_NUM  = 4096,
    parameter integer TX_SLOTS   = 32,
    parameter integer SLOT_BYTES = 2048,
    parameter integer Q_DEPTH    = 4,
    parameter integer MAX_MSG    = 8192
)(
    input  wire        clk,
    input  wire        reset,

    // 应用口（doc/05 信封语义；AFDX_top 直连）
    input  wire [7:0]  tx_data,
    input  wire        tx_valid,
    output wire [7:0]  rx_data,          // T7：rx_path 交付（VLID 2B + 载荷信封）
    output wire        rx_valid,
    output wire        tx_ready,         // 恒 1（doc/05：应用口无反压）
    input  wire        rx_ready,         // 未使用（doc/05：可不连线）

    // cfg 总线主机侧（VERDICT-015 交付口径：顶层主机配置接口，AFDX_top 引脚直入）
    input  wire [19:0] cfg_addr,
    input  wire [31:0] cfg_wdata,
    input  wire        cfg_we,
    input  wire        cfg_re,
    output wire [31:0] cfg_rdata,     // 读回数据（cfg_bus 出）
    output wire        cfg_rvalid,

    // GMII 观测口（mac_port_if 输出；T6 纯仿真抓帧；T8 对 eLinx MAC IP）
    output wire [7:0]  gmii_a_d,
    output wire        gmii_a_en,
    output wire [7:0]  gmii_b_d,
    output wire        gmii_b_en
);

    //-------------------------------------------------------------------------
    // GMII 接收侧 stimulus reg（T7 纯仿真：tb 层级引用驱动，未驱动前为 X；
    // T8 改接 eLinx MAC IP 的 GMII 客户端——与本文件 cfg 主侧 reg 同一模式）
    //-------------------------------------------------------------------------
    reg [7:0] gmii_rx_a_d;
    reg       gmii_rx_a_dv, gmii_rx_a_er;
    reg [7:0] gmii_rx_b_d;
    reg       gmii_rx_b_dv, gmii_rx_b_er;

    //-------------------------------------------------------------------------
    // 发送链连线（先声明后使用——host_arb/buf_mgr 等实例化均在其后）
    //-------------------------------------------------------------------------
    wire       htx_valid_w;             // host_arb 输出（解析器段实例）
    wire       htx_sop_w, htx_eop_w;
    wire [7:0] htx_data_w;
    wire [6:0] htx_vl_w;
    wire       htx_valid;               // 核内统一命名（buf_mgr 输入；丢弃态掐断，
                                        // assign 见解析器段——dropping 声明在其后）
    wire       htx_sop   = htx_sop_w;
    wire       htx_eop   = htx_eop_w;
    wire [7:0] htx_data  = htx_data_w;
    wire [6:0] htx_vl    = htx_vl_w;
    wire       htx_ready_w;
    reg        htx_abort;               // 单拍脉冲（解析器满丢时回收半截入队）
    wire [127:0] q_nonempty;
    wire       ev_enq, ev_qfull, ev_lenerr, ev_frame;
    wire [5:0] desc_slot;
    wire [13:0] desc_len;
    wire [15:0] desc_ipid;
    wire [3:0]  desc_fidx;
    wire [16:0] bld_pre_addr;
    wire [7:0]  bld_pre_data;
    wire [5:0]  bld_nx_ra, bld_nx_rd;
    wire       frag_upd, frag_last;
    wire [6:0] frag_vl;
    wire [5:0] frag_slot0;
    wire [13:0] frag_len;
    wire [127:0] elig_vec;
    wire       fs_pulse;
    wire [6:0] fs_vl;
    wire [127:0] req_vec;
    wire       task_start, task_done, st_frame;
    wire [6:0] task_vl, st_vl;
    wire [13:0] st_paylen;
    wire       macA_start, macB_start, macA_done, macB_done;
    wire       bld_busy, frame_ready;
    wire [10:0] fr_body_len;
    wire [13:0] fr_paylen;
    wire [1:0]  fr_mask;
    wire       sn_re_w, sn_vld_w;
    wire [6:0] sn_vl_w;
    wire [7:0] sn_val_w;
    wire       bld_desc_req, bld_desc_vld;
    wire [6:0] bld_desc_vl, bld_rd_idx;
    wire [2:0] bld_rd_wsel;
    wire [31:0] bld_rd_data;
    wire       s_valid, s_sop, s_eop;
    wire [7:0] s_data;
    wire       macA_sready, macB_sready;
    wire [7:0] macA_dout, macB_dout;
    wire       macA_enout, macB_enout;

    //-------------------------------------------------------------------------
    // 信封解析器：应用口 → htx
    //-------------------------------------------------------------------------
    reg  [7:0]  vlid_lo;                   // VLID 低字节（帧索引来源）
    reg  [1:0]  env_idx;                   // 信封内字节序号（0/1=VLID，≥2=载荷）
    reg         dropping;                  // 满丢模式（至信封结束）
    reg         presented;                 // 本信封已有字节下送 buf_mgr
    reg         abort_pend;                // 待发 htx_abort（回收半截入队）
    reg         dropping_r;                // dropping 打一拍（FIFO 清空用）

    // 1 拍流水：载荷字节 k 在 k+1 拍进 FIFO
    reg         p_valid, p_sop;
    reg  [7:0]  p_data;
    wire        p_eop = !tx_valid;         // 组合：信封本拍已落 = 上一字节是末字节
    wire [6:0]  p_vl  = vlid_lo[6:0];      // vlid_lo 在首载荷拍已稳定

    // 吸收 FIFO：{vl[6:0], sop, eop, data} 打包 17b
    wire [16:0] sk_wdata = {p_vl, p_sop, p_eop, p_data};
    wire        sk_we    = p_valid && !dropping;
    wire        sk_full, sk_empty;
    wire [16:0] sk_rdata;
    wire        sk_re;
    wire [3:0]  sk_cnt;
    wire        sk_flush;                  // 丢弃拍清 FIFO（指针归零）

    sync_fifo #(.DW(17), .DEPTH(8), .AW(3), .AF_LEVEL(6)) u_skid (
        .clk(clk), .reset(reset || sk_flush),
        .we(sk_we), .wdata(sk_wdata), .re(sk_re),
        .rdata(sk_rdata), .full(sk_full), .empty(sk_empty),
        .count(), .almost_full()
    );

    wire [6:0] sk_vl;
    wire       sk_sop, sk_eop;
    wire [7:0] sk_data;
    assign {sk_vl, sk_sop, sk_eop, sk_data} = sk_rdata;
    wire sk_vld = !sk_empty;

    // htx 呈现：FIFO 非空即有效；htx_ready（buf_mgr）反压
    host_arb #(.NUM_HOST(1), .IW(1)) u_harb (
        .clk(clk), .reset(reset),
        .h_valid(sk_vld), .h_sop(sk_sop), .h_eop(sk_eop),
        .h_data(sk_data), .h_vl(sk_vl), .h_ready(),
        .o_valid(htx_valid_w), .o_sop(htx_sop_w), .o_eop(htx_eop_w),
        .o_data(htx_data_w), .o_vl(htx_vl_w), .o_ready(htx_ready_w)
    );

    // 丢弃态下掐断向 buf_mgr 的呈现（REV-002-04 伴随项）：否则 host_arb 纯透传
    // 会让 buf_mgr 在 dropping 期间反复重收队首字节（幻影写入半截消息）。
    // 由此 buf_mgr 的 do_xfer 与 sk_re 同一化，abort 判定（下方）以 sk_re 为准即足够。
    assign htx_valid = htx_valid_w && !dropping;

    // FIFO 弹出：有字节且下游就绪且非丢弃态
    assign sk_re = sk_vld && htx_ready_w && !dropping;
    assign sk_flush = dropping_r;              // 进入丢弃的下一拍清 FIFO

    // 满丢判定与执行：载荷字节到来而 FIFO 满 → 丢整条
    always @(posedge clk) begin
        if (reset) begin
            vlid_lo <= 8'h00; env_idx <= 2'd0;
            dropping <= 1'b0; dropping_r <= 1'b0;
            presented <= 1'b0; abort_pend <= 1'b0; htx_abort <= 1'b0;
            p_valid <= 1'b0; p_sop <= 1'b0; p_data <= 8'h00;
        end else begin
            // VLID 捕获与序号（0=VLID 高，1=VLID 低，2=首个载荷，3=其余载荷；
            //  饱和到 3——若停在 2，p_sop 条件会使每个载荷字节都带 SOP）
            if (tx_valid) begin
                case (env_idx)
                    2'd1: vlid_lo <= tx_data;          // byte1 = VLID 低字节
                    default: ;
                endcase
                env_idx <= (env_idx == 2'd3) ? 2'd3 : env_idx + 2'd1;
            end
            if (!tx_valid) begin                       // 信封结束：复位
                env_idx   <= 2'd0;
                dropping  <= 1'b0;
                presented <= 1'b0;
            end

            // 流水寄存（p_eop/p_vl 为组合量：!tx_valid / vlid_lo）
            p_valid <= tx_valid && (env_idx >= 2'd2) && !dropping;
            p_sop   <= tx_valid && (env_idx == 2'd2);   // 仅首个载荷字节
            p_data  <= tx_data;

            // presented：本信封有字节已下送（sk_re 脉冲即置位，信封结束清）
            if (sk_re)
                presented <= 1'b1;

            // 满丢：载荷字节无处可去 → 丢整条（清 FIFO + abort 半截入队）
            if (tx_valid && (env_idx >= 2'd2) && sk_full && !dropping) begin
                dropping   <= 1'b1;
                // REV-002-04：回收判定以"本信封是否已有字节被 buf_mgr 接受"为准，
                // 含同拍首次下送（presented 是旧值，须并入当拍 sk_re）；
                // 当拍下送的是 eop 字节则消息恰在本拍完整入环，无需回收
                abort_pend <= presented || (sk_re && !sk_eop);
            end
            dropping_r <= dropping;
            if (abort_pend) begin
                abort_pend <= 1'b0;
                htx_abort  <= 1'b1;            // 单拍（buf_mgr 回收半截入队）
            end else
                htx_abort <= 1'b0;
        end
    end

    //-------------------------------------------------------------------------
    // cfg / 时基
    //-------------------------------------------------------------------------
    wire        init_busy, init_done;
    wire [2:0]  ri_sel;
    wire [15:0] ri_addr;
    wire        ri_we;
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
    wire        rxt_we;                      // 接收 VL 表写广播（T7，0x10000 区）
    wire [10:0] rxt_idx;
    wire        rxt_wsel;
    wire [31:0] rxt_wdata;
    wire        rxt_re;
    wire [10:0] rxt_ridx;
    wire        rxt_rsel;
    wire [31:0] rxt_rdata;
    wire        stats_re;
    wire [9:0]  stats_raddr;
    wire [31:0] stats_rdata;
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
        .rxt_we(rxt_we), .rxt_idx(rxt_idx), .rxt_wsel(rxt_wsel), .rxt_wdata(rxt_wdata),
        .rxt_re(rxt_re), .rxt_ridx(rxt_ridx), .rxt_rsel(rxt_rsel), .rxt_rdata(rxt_rdata),
        .stats_re(stats_re), .stats_raddr(stats_raddr), .stats_rdata(stats_rdata)
    );

    ram_init u_ri (
        .clk(clk), .reset(reset),
        .init_busy(init_busy), .init_done(init_done),
        .ri_sel(ri_sel), .ri_addr(ri_addr), .ri_we(ri_we)
    );

    tick_gen u_tick (.clk(clk), .reset(reset), .tick(tick_us));
    timebase u_time (.clk(clk), .reset(reset), .time_ns(time_ns));

    //-------------------------------------------------------------------------
    // 发送数据通路
    //-------------------------------------------------------------------------
    wire bld_s_ready = (fr_mask[0] ? macA_sready : 1'b1)
                     & (fr_mask[1] ? macB_sready : 1'b1);

    buf_mgr #(
        .TX_VL_NUM(TX_VL_NUM), .TX_SLOTS(TX_SLOTS), .SLOT_BYTES(SLOT_BYTES),
        .Q_DEPTH(Q_DEPTH), .MAX_MSG(MAX_MSG)
    ) u_bm (
        .clk(clk), .reset(reset),
        .htx_valid(htx_valid), .htx_ready(htx_ready_w),
        .htx_sop(htx_sop), .htx_eop(htx_eop), .htx_data(htx_data), .htx_vl(htx_vl),
        .htx_abort(htx_abort),
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

`ifdef AFDX_MAC_IP
    // T8：Tri_Mode_Ethernet_Mac IP 集成（每网 1 例化，doc/07 §2.1 第 3 层）。
    // 适配层/片内回环/虚拟 PHY 见 rtl/mac/mac_ip_top.v 头注；AFDX_MAC_IP_SIM
    // 时以行为替身编译（iverilog 功能回归），综合链接 eLinx 加密 IP。
    wire [7:0] macA_lb_d, macB_lb_d;
    wire       macA_lb_dv, macA_lb_er, macB_lb_dv, macB_lb_er;
    mac_ip_top #(.NET_ID(2'b01), .IF_ID_BYTE(8'h20)) u_macA (
        .clk(clk), .reset(reset), .tick_us(tick_us),
        .start(macA_start), .body_len(fr_body_len),
        .s_valid(s_valid), .s_ready(macA_sready), .s_sop(s_sop), .s_eop(s_eop),
        .s_data(s_data),
        .done(macA_done),
        .gmii_d(gmii_a_d), .gmii_en(gmii_a_en), .gmii_er(),
        .frame_start(), .frame_end(),
        .ev_frame(), .ev_fcserr(), .ev_lenerr(), .ev_overflow(),
        .lb_gmii_d(macA_lb_d), .lb_gmii_dv(macA_lb_dv), .lb_gmii_er(macA_lb_er)
    );
    mac_ip_top #(.NET_ID(2'b10), .IF_ID_BYTE(8'h40)) u_macB (
        .clk(clk), .reset(reset), .tick_us(tick_us),
        .start(macB_start), .body_len(fr_body_len),
        .s_valid(s_valid), .s_ready(macB_sready), .s_sop(s_sop), .s_eop(s_eop),
        .s_data(s_data),
        .done(macB_done),
        .gmii_d(gmii_b_d), .gmii_en(gmii_b_en), .gmii_er(),
        .frame_start(), .frame_end(),
        .ev_frame(), .ev_fcserr(), .ev_lenerr(), .ev_overflow(),
        .lb_gmii_d(macB_lb_d), .lb_gmii_dv(macB_lb_dv), .lb_gmii_er(macB_lb_er)
    );
    // 片内回环电缆（1 拍）：IP eth_txd → mac_ip_top 打 1 拍 → gmii_rx_* 寄存器
    // → rx_path 内 mac_rx（T7 tb 层级回环的核内版；AFDX_MAC_LOOPBACK 口径）
`ifdef AFDX_MAC_LOOPBACK
    always @(posedge clk) begin
        if (reset) begin
            gmii_rx_a_d <= 8'h0; gmii_rx_a_dv <= 1'b0; gmii_rx_a_er <= 1'b0;
            gmii_rx_b_d <= 8'h0; gmii_rx_b_dv <= 1'b0; gmii_rx_b_er <= 1'b0;
        end else begin
            gmii_rx_a_d <= macA_lb_d; gmii_rx_a_dv <= macA_lb_dv; gmii_rx_a_er <= macA_lb_er;
            gmii_rx_b_d <= macB_lb_d; gmii_rx_b_dv <= macB_lb_dv; gmii_rx_b_er <= macB_lb_er;
        end
    end
`endif
`else
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
        .frame_start(), .frame_end()
    );
    mac_port_if #(.NET_ID(2'b10)) u_pifB (
        .clk(clk), .reset(reset),
        .tx_d_in(macB_dout), .tx_en_in(macB_enout),
        .gmii_d(gmii_b_d), .gmii_en(gmii_b_en), .gmii_er(),
        .frame_start(), .frame_end()
    );
`endif

    // 接收侧事件（T7：rx_path → stats）
    wire        rx_ev_frame, rx_ev_fcs,   rx_ev_len,   rx_ev_mac,  rx_ev_vlunk;
    wire        rx_ev_net,   rx_ev_ip,    rx_ev_ic,    rx_ev_rmdup, rx_ev_rmold;
    wire        rx_ev_fragto, rx_ev_ovf,  rx_ev_ctx,   rx_ev_msg;
    wire [13:0] rx_ev_bytes;

    stats u_st (
        .clk(clk), .reset(reset), .tick_us(tick_us),
        .elig(req_vec),
        .st_frame(st_frame), .st_vl(st_vl), .st_paylen(st_paylen),
        .ev_enq(ev_enq), .ev_qfull(ev_qfull), .ev_lenerr(ev_lenerr),
        .ev_rxframe(rx_ev_frame), .ev_rxfcs(rx_ev_fcs), .ev_rxlen(rx_ev_len),
        .ev_rxmac(rx_ev_mac), .ev_rxvlunk(rx_ev_vlunk), .ev_rxnet(rx_ev_net),
        .ev_rxip(rx_ev_ip), .ev_rxic(rx_ev_ic), .ev_rmdup(rx_ev_rmdup),
        .ev_rmold(rx_ev_rmold), .ev_fragto(rx_ev_fragto), .ev_rxovf(rx_ev_ovf),
        .ev_rxctx(rx_ev_ctx), .ev_rxmsg(rx_ev_msg), .ev_rxbytes(rx_ev_bytes),
        .stats_re(stats_re), .stats_raddr(stats_raddr), .stats_rdata(stats_rdata),
        .ri_we(ri_we && (ri_sel == 3'd2 || ri_sel == 3'd3 || ri_sel == 3'd4)),
        .ri_sel(ri_sel), .ri_addr(ri_addr)
    );

    // 任务完成 → BAG 重启
    assign fs_pulse = task_done;
    assign fs_vl    = st_vl;

    tx_vl_table u_tbl (
        .clk(clk),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_wsel(cw_wsel), .cw_wdata(cw_wdata),
        .tbl_re(tbl_re), .tbl_raddr(tbl_raddr), .tbl_busy(1'b0), .tbl_rdata(tbl_rdata),
        .rd_idx(bld_rd_idx), .rd_wsel(bld_rd_wsel), .rd_data(bld_rd_data),
        .ri_we(ri_we && (ri_sel == 3'd1)), .ri_addr(ri_addr[9:0])
    );

    // 接收侧（T7）：rx_path 全链（mac_rx×2 → parser/lookup/IC/RM/重组 → 应用口）
    rx_path #(.RX_VL_NUM(RX_VL_NUM)) u_rx (
        .clk(clk), .reset(reset), .tick_us(tick_us), .time_ns(time_ns),
        .rxd_a(gmii_rx_a_d), .dv_a(gmii_rx_a_dv), .er_a(gmii_rx_a_er),
        .rxd_b(gmii_rx_b_d), .dv_b(gmii_rx_b_dv), .er_b(gmii_rx_b_er),
        .rxcfg_we(rxt_we), .rxcfg_idx(rxt_idx), .rxcfg_wsel(rxt_wsel),
        .rxcfg_wdata(rxt_wdata),
        .rxcfg_re(rxt_re), .rxcfg_ridx(rxt_ridx), .rxcfg_rsel(rxt_rsel),
        .rxcfg_rdata(rxt_rdata),
        .ri_we5(ri_we && (ri_sel == 3'd5)), .ri_addr5(ri_addr[12:0]),
        .ri_we6(ri_we && (ri_sel == 3'd6)), .ri_addr6(ri_addr[11:0]),
        .ri_we7(ri_we && (ri_sel == 3'd7)), .ri_addr7(ri_addr[11:0]),
        .ev_rxframe(rx_ev_frame), .ev_fcserr(rx_ev_fcs), .ev_lenerr(rx_ev_len),
        .ev_macerr(rx_ev_mac), .ev_vlunk(rx_ev_vlunk), .ev_neterr(rx_ev_net),
        .ev_iperr(rx_ev_ip), .ev_icerr(rx_ev_ic), .ev_rmdup(rx_ev_rmdup),
        .ev_rmold(rx_ev_rmold), .ev_fragto(rx_ev_fragto), .ev_ovf(rx_ev_ovf),
        .ev_ctxerr(rx_ev_ctx), .ev_rxmsg(rx_ev_msg), .ev_rxbytes(rx_ev_bytes),
        .rx_data(rx_data), .rx_valid(rx_valid)
    );

    assign tx_ready = 1'b1;   // 应用发送通道恒就绪（doc/05 ready 可不用）

endmodule
