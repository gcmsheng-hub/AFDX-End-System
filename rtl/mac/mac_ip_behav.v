//=============================================================================
// mac_ip_behav.v — Tri_Mode_Ethernet_Mac 行为替身（iverilog 功能回归用）
// 端口与 eLinx 加密 IP 一致（用户指南表 2.2~2.7），行为口径（指南第三章）：
//   TX：流式（不存储转发）——IDLE 检到首字节先插前导 7×55+SFD D5（输入字节
//       经 4 深吸收队列不丢），随后 1B/拍直通帧体，tlast 后补 FCS32（CRC-32/
//       POLY 0x04C11DB7 INIT FFFFFFFF REFIN REFOUT XOROUT FFFFFFFF，低字节
//       先发），TX_EN 落；tready 仅在帧发送期拉低（同一时刻只发一帧）。
//   RX：rx_mac_rst_n=0 → rx_axis 恒无效（本设计 RX 走 T7 RTL mac_rx）。
//   MDIO：不复现主机时序（虚拟 PHY 在 mac_ip_top 内应答，替身不依赖）。
//   时序：gtx_clk 单域 125MHz（1000M 口径，§3.2）；eth_tx_clk 直通 gtx_clk。
// 仅用于 AFDX_MAC_IP_SIM 回归；综合/后仿以 eLinx 加密 IP 的网表为准。
//=============================================================================

module Tri_Mode_Ethernet_Mac (
    // 用户发送口（表 2.2）
    input  wire        gtx_clk,
    input  wire        tx_mac_rst_n,
    output wire        tx_mac_aclk,
    output wire        tx_axis_mac_tready,
    input  wire        tx_axis_mac_tuser,
    input  wire        tx_axis_mac_tlast,
    input  wire        tx_axis_mac_tvalid,
    input  wire [7:0]  tx_axis_mac_tdata,
    // 用户接收口（表 2.3）
    input  wire        rx_mac_rst_n,
    output wire        rx_mac_aclk,
    output wire        rx_axis_mac_tuser,
    output wire        rx_axis_mac_tlast,
    output wire        rx_axis_mac_tvalid,
    output wire [7:0]  rx_axis_mac_tdata,
    output wire [3:0]  rx_axis_filter_tuser,
    // PHY 接收口（表 2.4）
    input  wire        eth_rx_clk,
    input  wire        mii_tx_clk,
    input  wire        eth_rx_er,
    input  wire        eth_rx_dv,
    input  wire [7:0]  eth_rxd,
    input  wire        col,
    input  wire        crs,
    // PHY 发送口（表 2.5）
    output wire        eth_tx_clk,
    output wire        eth_tx_er,
    output wire        eth_tx_en,
    output wire [7:0]  eth_txd,
    // MDIO（表 2.6）
    input  wire        mdio_rst_n,
    input  wire        mdio_clk,
    output wire        mdc,
    inout  wire        mdio,
    // 用户控制口（表 2.7）
    input  wire        sys_rst_n,
    output wire        link,
    output wire [1:0]  speed,
    input  wire        user_duplex_mode,
    input  wire        user_port_mode,
    input  wire [4:0]  phy_addr,
    input  wire        user_wr_req,
    input  wire [4:0]  user_wr_addr,
    input  wire [15:0] user_wr_data,
    input  wire        user_rd_req,
    input  wire [4:0]  user_rd_addr,
    output wire        user_rd_dv,
    output wire [15:0] user_rd_data,
    input  wire        mac_filter_sw,
    input  wire        mac_filter_en,
    input  wire [3:0]  mac_filter_num,
    input  wire [47:0] mac_filter_addr,
    input  wire        pause_req,
    input  wire [15:0] pause_info,
    input  wire [47:0] sour_mac_addr
);

    assign tx_mac_aclk = gtx_clk;
    assign rx_mac_aclk = eth_rx_clk;
    assign eth_tx_clk  = gtx_clk;
    assign eth_tx_er   = 1'b0;
    assign mdc         = mdio_clk;
    assign rx_axis_mac_tvalid   = 1'b0;
    assign rx_axis_mac_tdata    = 8'h0;
    assign rx_axis_mac_tlast    = 1'b0;
    assign rx_axis_mac_tuser    = 1'b0;
    assign rx_axis_filter_tuser = 4'b0;
    assign user_rd_dv   = 1'b0;
    assign user_rd_data = 16'h0;
    assign link         = 1'b1;            // 替身：链路恒有效
    assign speed        = 2'b10;           // 替身：恒千兆

    //-------------------------------------------------------------------------
    // TX 流式：吸收队列(4×9b) → FSM：IDLE（收首字节）→ PRE(8) → BODY（队列
    // 弹出直通）→ FCSW → FCS(4) → IDLE。CRC 随帧体逐字节吸入。
    //-------------------------------------------------------------------------
    localparam [2:0] S_IDLE = 3'd0, S_PRE = 3'd1, S_BODY = 3'd2,
                     S_FCSW = 3'd3, S_FCS = 3'd4;
    reg  [2:0]  tstate;
    reg  [2:0]  pre_cnt;
    reg  [1:0]  fcs_cnt;
    reg  [31:0] fcs_r;
    wire [31:0] crc_w;

    // 吸收队列（4×9b：{eop, data}）——PRE 期 8 拍的输入积压
    reg [8:0] q_mem [0:3];
    reg [1:0] q_wp, q_rp;
    reg [2:0] q_cnt;
    wire      q_we  = tx_axis_mac_tvalid && tx_axis_mac_tready;
    wire      q_re  = (tstate == S_BODY) && (q_cnt != 3'd0);
    wire [8:0] q_head = q_mem[q_rp[1:0]];

    always @(posedge gtx_clk) begin
        if (!sys_rst_n || !tx_mac_rst_n) begin
            q_wp <= 2'd0; q_rp <= 2'd0; q_cnt <= 3'd0;
        end else begin
            if (q_we) begin
                q_mem[q_wp[1:0]] <= {tx_axis_mac_tlast, tx_axis_mac_tdata};
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

    assign tx_axis_mac_tready = (tstate == S_IDLE || tstate == S_BODY)
                              && (q_cnt < 3'd3) && sys_rst_n && tx_mac_rst_n;

    // CRC：首字节入队拍重置（前导不参与）；BODY 期随弹出逐字节吸入；
    // 末字节吸入次拍 fcs 口出最终 FCS（mac_tx 同口径）。
    crc32_d8 u_crc (
        .clk(gtx_clk), .reset(!sys_rst_n),
        .init(tstate == S_IDLE && q_we),
        .valid(tstate == S_BODY && q_re),
        .data(q_head[7:0]),
        .crc(), .fcs(crc_w));

    // 发送数据选择（EN 无效期线上 0）。S_FCSW 拍 FCS 首字节由 CRC 组合输出
    // 直出（末字节上一沿已吸入，fcs 口当拍有效）；S_FCS 发第 2~4 字节。
    assign eth_tx_en = (tstate != S_IDLE);
    assign eth_txd   = (tstate == S_PRE)  ? ((pre_cnt == 3'd7) ? 8'hD5 : 8'h55)
                     : (tstate == S_BODY) ? q_head[7:0]
                     : (tstate == S_FCSW) ? crc_w[7:0]
                     : (tstate == S_FCS)  ?
                           (fcs_cnt == 2'd1) ? fcs_r[15:8] :
                           (fcs_cnt == 2'd2) ? fcs_r[23:16]
                                             : fcs_r[31:24]
                                          : 8'h00;

    always @(posedge gtx_clk) begin
        if (!sys_rst_n || !tx_mac_rst_n) begin
            tstate <= S_IDLE; pre_cnt <= 3'd0; fcs_cnt <= 2'd0; fcs_r <= 32'd0;
        end else begin
            case (tstate)
                S_IDLE: begin
                    if (q_we) begin            // 首字节到：先发前导
                        pre_cnt <= 3'd0;
                        tstate  <= S_PRE;
                    end
                end
                S_PRE: begin                   // 7×55 + SFD
                    pre_cnt <= pre_cnt + 3'd1;
                    if (pre_cnt == 3'd7) begin
                        pre_cnt <= 3'd0;
                        tstate  <= S_BODY;
                    end
                end
                S_BODY: begin
                    if (q_re && q_head[8])     // 帧尾字节已出线
                        tstate <= S_FCSW;      // CRC 末字节本沿吸入
                end
                S_FCSW: begin                  // FCS[7:0] 本拍组合出线
                    fcs_r   <= crc_w;
                    fcs_cnt <= 2'd1;
                    tstate  <= S_FCS;
                end
                S_FCS: begin                   // FCS 4B 低字节先发
                    fcs_cnt <= fcs_cnt + 2'd1;
                    if (fcs_cnt == 2'd3)
                        tstate <= S_IDLE;      // TX_EN 落（IFG 为线上静默）
                end
                default: tstate <= S_IDLE;
            endcase
        end
    end

endmodule

//=============================================================================
// Tri_Mode_Ethernet_Mac_0 — 交付口径例化名包装（VERDICT-015 交付形态 e2e 用）
// mac_ip_top 例化 `Tri_Mode_Ethernet_Mac_0`（37 口，RPT-T8-002 §3.2），本替身
// 本体为指南 50 口 `Tri_Mode_Ethernet_Mac`。本包装按真实 IP 37 口出线，13 个
// 交付口径不存在的入口在本层按 E4 删除前语义拉死（逐一对应 RPT-T8-002 §3.2
// ③：mii_tx_clk=gtx_clk、col/crs=0、duplex=1、port_mode=0（GMII）、filter 组
// =0、pause=0、sour_mac_addr=0），零语义差（替身不读这些口）。仅仿真用，
// 不入 GUI 工程。
//=============================================================================
module Tri_Mode_Ethernet_Mac_0 (
    input  wire        gtx_clk,
    input  wire        tx_mac_rst_n,
    output wire        tx_mac_aclk,
    output wire        tx_axis_mac_tready,
    input  wire        tx_axis_mac_tuser,
    input  wire        tx_axis_mac_tlast,
    input  wire        tx_axis_mac_tvalid,
    input  wire [7:0]  tx_axis_mac_tdata,
    input  wire        rx_mac_rst_n,
    output wire        rx_mac_aclk,
    output wire        rx_axis_mac_tuser,
    output wire        rx_axis_mac_tlast,
    output wire        rx_axis_mac_tvalid,
    output wire [7:0]  rx_axis_mac_tdata,
    input  wire        eth_rx_clk,
    input  wire        eth_rx_er,
    input  wire        eth_rx_dv,
    input  wire [7:0]  eth_rxd,
    output wire        eth_tx_clk,
    output wire        eth_tx_er,
    output wire        eth_tx_en,
    output wire [7:0]  eth_txd,
    input  wire        mdio_rst_n,
    input  wire        mdio_clk,
    output wire        mdc,
    inout  wire        mdio,
    input  wire        sys_rst_n,
    output wire        link,
    output wire [1:0]  speed,
    input  wire [4:0]  phy_addr,
    input  wire        user_wr_req,
    input  wire [4:0]  user_wr_addr,
    input  wire [15:0] user_wr_data,
    input  wire        user_rd_req,
    input  wire [4:0]  user_rd_addr,
    output wire        user_rd_dv,
    output wire [15:0] user_rd_data
);

    Tri_Mode_Ethernet_Mac u_standin (
        .gtx_clk            (gtx_clk),
        .tx_mac_rst_n       (tx_mac_rst_n),
        .tx_mac_aclk        (tx_mac_aclk),
        .tx_axis_mac_tready (tx_axis_mac_tready),
        .tx_axis_mac_tuser  (tx_axis_mac_tuser),
        .tx_axis_mac_tlast  (tx_axis_mac_tlast),
        .tx_axis_mac_tvalid (tx_axis_mac_tvalid),
        .tx_axis_mac_tdata  (tx_axis_mac_tdata),
        .rx_mac_rst_n       (rx_mac_rst_n),
        .rx_mac_aclk        (rx_mac_aclk),
        .rx_axis_mac_tuser  (rx_axis_mac_tuser),
        .rx_axis_mac_tlast  (rx_axis_mac_tlast),
        .rx_axis_mac_tvalid (rx_axis_mac_tvalid),
        .rx_axis_mac_tdata  (rx_axis_mac_tdata),
        .rx_axis_filter_tuser(),
        .eth_rx_clk         (eth_rx_clk),
        .mii_tx_clk         (gtx_clk),      // E4 删前同款 tie
        .eth_rx_er          (eth_rx_er),
        .eth_rx_dv          (eth_rx_dv),
        .eth_rxd            (eth_rxd),
        .col                (1'b0),
        .crs                (1'b0),
        .eth_tx_clk         (eth_tx_clk),
        .eth_tx_er          (eth_tx_er),
        .eth_tx_en          (eth_tx_en),
        .eth_txd            (eth_txd),
        .mdio_rst_n         (mdio_rst_n),
        .mdio_clk           (mdio_clk),
        .mdc                (mdc),
        .mdio               (mdio),
        .sys_rst_n          (sys_rst_n),
        .link               (link),
        .speed              (speed),
        .user_duplex_mode   (1'b1),         // 全双工（E4 删前 tie）
        .user_port_mode     (1'b0),         // GMII（E4 删前 tie）
        .phy_addr           (phy_addr),
        .user_wr_req        (user_wr_req),
        .user_wr_addr       (user_wr_addr),
        .user_wr_data       (user_wr_data),
        .user_rd_req        (user_rd_req),
        .user_rd_addr       (user_rd_addr),
        .user_rd_dv         (user_rd_dv),
        .user_rd_data       (user_rd_data),
        .mac_filter_sw      (1'b0),
        .mac_filter_en      (1'b0),
        .mac_filter_num     (4'd0),
        .mac_filter_addr    (48'd0),
        .pause_req          (1'b0),
        .pause_info         (16'd0),
        .sour_mac_addr      (48'd0)
    );

endmodule
