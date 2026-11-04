//=============================================================================
// mac_ip_top_variant.v — 后仿规避变体（官方答疑 2026-10-31~11-02 授权）
//
// 用途：门级 e2e 在真实 Tri_Mode_Ethernet_Mac IP 上网表零交付（delivered=0/6），
//       为取得"综合后 AFDX 逻辑可交付"的证据，本文件提供一个**接口完全一致**的
//       mac_ip_top 变体：TX 侧内联简化 MAC（前导/SFD + 帧体直通 + FCS32 追加，
//       语义取自 rtl/mac/mac_ip_behav.v 的 GMII 回环口径），**不例化加密 IP**，
//       从而可被 eLinx 综合并做门级仿真。
//
// 硬约束（与调度令一致）：
//   - 不改 rtl/、不改第 11 轮 .vqm、不改 TB；
//   - AFDX 业务逻辑（组帧/调度/BAG/SN/分片/接收链）全部沿用原 RTL，本文件只替换
//     "IP 边界"这一层；
//   - 提交物仍为正常版本，本变体仅作诊断/规避证据，不入正常交付路径。
//
// 与 rtl/mac/mac_ip_top.v 的差异（仅三处）：
//   1) 删除 Tri_Mode_Ethernet_Mac_0 例化，改为内联 TX 状态机（见下）；
//   2) 删除 MDIO 25MHz 分频与虚拟 PHY 端口（变体不读 MDIO，link 恒有效）；
//   3) 其余（TX 适配/skid FIFO/复位链/lb 回环寄存/gmii 与 done 口径）逐字保留。
//
// 端口、参数与 rtl/mac/mac_ip_top.v 完全一致，可直接替换综合源列表中的原文件。
//=============================================================================

`define AFDX_MAC_LOOPBACK 1

module mac_ip_top #(
    parameter [1:0] NET_ID      = 2'b01,    // 01=A 网 / 10=B 网（观测标注）
    parameter [7:0] IF_ID_BYTE  = 8'h20     // {IF_ID,5'b0}：A=0x20 / B=0x40
)(
    input  wire        clk,            // 125MHz 核时钟（=gtx_clk 域）
    input  wire        reset,          // 同步高有效（核域）
    input  wire        tick_us,        // （保留，mac_tx 兼容）

    // ---- mac_tx 兼容口 ----
    input  wire        start,
    input  wire [10:0] body_len,
    input  wire        s_valid,
    output wire        s_ready,
    input  wire        s_sop,
    input  wire        s_eop,
    input  wire [7:0]  s_data,
    output reg         done,

    // ---- GMII 出 ----
    output wire [7:0]  gmii_d,
    output wire        gmii_en,
    output wire        gmii_er,
    output wire        frame_start,
    output wire        frame_end,

    // ---- RX 侧事件（恒 0）----
    output wire        ev_frame,
    output wire        ev_fcserr,
    output wire        ev_lenerr,
    output wire        ev_overflow,

    // ---- 回环 GMII 出 ----
    output wire [7:0]  lb_gmii_d,
    output wire        lb_gmii_dv,
    output wire        lb_gmii_er
);

    //-------------------------------------------------------------------------
    // TX 适配：s_stream → tx_axis（与原文件逐字一致）
    //   - 字节 11（源 MAC 第 6 字节）按网替换 IF_ID_BYTE
    //   - skid 16×9b 吸收 MAC tready 空档
    //-------------------------------------------------------------------------
    reg  [10:0] tcnt;
    reg  [3:0]  ifg_cnt;
    reg  [1:0]  dstate;
    reg         en_d;
    reg  [7:0]  lb_d;
    reg         lb_dv, lb_er;

    wire       ax_tready;
    wire       f_we;
    wire [8:0] f_rdata;
    wire       f_full, f_empty;
    wire       ax_tvalid = !f_empty && (dstate == 2'd1 || dstate == 2'd2);
    wire       ax_tlast  = f_rdata[8];
    wire [7:0] ax_tdata  = f_rdata[7:0];
    wire       f_pop     = ax_tvalid && ax_tready;

    sync_fifo #(.DW(9), .DEPTH(16), .AW(4), .AF_LEVEL(14)) u_skid (
        .clk(clk), .reset(reset),
        .we(f_we), .wdata({s_eop, (tcnt == 11'd11) ? IF_ID_BYTE : s_data}),
        .re(f_pop),
        .rdata(f_rdata), .full(f_full), .empty(f_empty),
        .count(), .almost_full());

    assign f_we    = (dstate == 2'd1) && s_valid && !f_full;
    assign s_ready = (dstate == 2'd1) && !f_full;

    always @(posedge clk) begin
        if (reset) begin
            tcnt <= 11'd0; ifg_cnt <= 4'd0; dstate <= 2'd0;
            done <= 1'b0; en_d <= 1'b0;
        end else begin
            done <= 1'b0;
            en_d <= gmii_en;
            case (dstate)
                2'd0: begin
                    if (start) begin
                        tcnt   <= 11'd0;
                        dstate <= 2'd1;
                    end
                end
                2'd1: begin
                    if (f_we)
                        tcnt <= tcnt + 11'd1;
                    if (!gmii_en && en_d) begin
                        ifg_cnt <= 4'd0;
                        dstate  <= 2'd2;
                    end
                end
                2'd2: begin
                    ifg_cnt <= ifg_cnt + 4'd1;
                    if (ifg_cnt == 4'd11) begin
                        done   <= 1'b1;
                        dstate <= 2'd0;
                    end
                end
                default: dstate <= 2'd0;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 低有效复位链（与原文件一致）
    //-------------------------------------------------------------------------
    wire      rst_n = ~reset;
    reg  [3:0] rst_sh;
    wire      tx_rst_n;
    always @(posedge clk) begin
        if (reset) rst_sh <= 4'b0000;
        else       rst_sh <= {rst_sh[2:0], 1'b1};
    end
    assign tx_rst_n = rst_sh[3];

    //-------------------------------------------------------------------------
    // 内联简化 MAC（TX only；语义 = rtl/mac/mac_ip_behav.v 的 TX 流式口径）
    //   IDLE 收首字节 → PRE(7×55+SFD D5) → BODY 直通 → FCS32(低字节先发) → IDLE
    //   RX 方向不实现：本设计 RX 走 rtl/mac/mac_rx（回环电缆直接喂 GMII）
    //-------------------------------------------------------------------------
    localparam [2:0] S_IDLE = 3'd0, S_PRE = 3'd1, S_BODY = 3'd2,
                     S_FCSW = 3'd3, S_FCS = 3'd4;
    reg  [2:0]  tstate;
    reg  [2:0]  pre_cnt;
    reg  [1:0]  fcs_cnt;
    reg  [31:0] fcs_r;
    wire [31:0] crc_w;

    // 吸收队列（4×9b：{eop,data}，PRE 期 8 拍输入积压）
    reg [8:0] q_mem [0:3];
    reg [1:0] q_wp, q_rp;
    reg [2:0] q_cnt;
    wire      q_we = ax_tvalid && ax_tready;
    wire      q_re = (tstate == S_BODY) && (q_cnt != 3'd0);
    wire [8:0] q_head = q_mem[q_rp[1:0]];

    always @(posedge clk) begin
        if (!rst_n || !tx_rst_n) begin
            q_wp <= 2'd0; q_rp <= 2'd0; q_cnt <= 3'd0;
        end else begin
            if (q_we) begin
                q_mem[q_wp[1:0]] <= {ax_tlast, ax_tdata};
                q_wp <= q_wp + 2'd1;
            end
            if (q_re)
                q_rp <= q_rp + 2'd1;
            case ({q_we, q_re})
                2'b10: q_cnt <= q_cnt + 3'd1;
                2'b01: q_cnt <= q_cnt - 3'd1;
                default: ;
            endcase
        end
    end

    assign ax_tready = (tstate == S_IDLE || tstate == S_BODY)
                     && (q_cnt < 3'd3) && rst_n && tx_rst_n;

    crc32_d8 u_crc (
        .clk(clk), .reset(!rst_n),
        .init(tstate == S_IDLE && q_we),
        .valid(tstate == S_BODY && q_re),
        .data(q_head[7:0]),
        .crc(), .fcs(crc_w));

    wire [7:0] eth_txd;
    wire       eth_tx_en;

    assign eth_tx_en = (tstate != S_IDLE);
    assign eth_txd   = (tstate == S_PRE)  ? ((pre_cnt == 3'd7) ? 8'hD5 : 8'h55)
                     : (tstate == S_BODY) ? q_head[7:0]
                     : (tstate == S_FCSW) ? crc_w[7:0]
                     : (tstate == S_FCS)  ?
                           (fcs_cnt == 2'd1) ? fcs_r[15:8] :
                           (fcs_cnt == 2'd2) ? fcs_r[23:16]
                                             : fcs_r[31:24]
                                          : 8'h00;

    always @(posedge clk) begin
        if (!rst_n || !tx_rst_n) begin
            tstate <= S_IDLE; pre_cnt <= 3'd0; fcs_cnt <= 2'd0; fcs_r <= 32'd0;
        end else begin
            case (tstate)
                S_IDLE: begin
                    if (q_we) begin
                        pre_cnt <= 3'd0;
                        tstate  <= S_PRE;
                    end
                end
                S_PRE: begin
                    pre_cnt <= pre_cnt + 3'd1;
                    if (pre_cnt == 3'd7) begin
                        pre_cnt <= 3'd0;
                        tstate  <= S_BODY;
                    end
                end
                S_BODY: begin
                    if (q_re && q_head[8])
                        tstate <= S_FCSW;
                end
                S_FCSW: begin
                    fcs_r   <= crc_w;
                    fcs_cnt <= 2'd1;
                    tstate  <= S_FCS;
                end
                S_FCS: begin
                    fcs_cnt <= fcs_cnt + 2'd1;
                    if (fcs_cnt == 2'd3)
                        tstate <= S_IDLE;
                end
                default: tstate <= S_IDLE;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // GMII 出 + 片内回环电缆（1 拍；与原文件一致）
    //-------------------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            lb_d <= 8'h0; lb_dv <= 1'b0; lb_er <= 1'b0;
        end else begin
            lb_d  <= eth_txd;
            lb_dv <= eth_tx_en;
            lb_er <= 1'b0;
        end
    end

    assign lb_gmii_d  = lb_d;
    assign lb_gmii_dv = lb_dv;
    assign lb_gmii_er = lb_er;

    assign gmii_d  = eth_txd;
    assign gmii_en = eth_tx_en;
    assign gmii_er = 1'b0;
    assign frame_start = gmii_en && !en_d;
    assign frame_end   = !gmii_en && en_d;

    // RX 侧事件恒 0（接收链事件在 rx_path 内部产生）
    assign ev_frame    = 1'b0;
    assign ev_fcserr   = 1'b0;
    assign ev_lenerr   = 1'b0;
    assign ev_overflow = 1'b0;

endmodule
