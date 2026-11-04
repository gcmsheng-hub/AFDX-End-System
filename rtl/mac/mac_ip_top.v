//=============================================================================
// mac_ip_top.v — T8：Tri_Mode_Ethernet_Mac IP 集成适配层（每网 1 例化）
//
// 结构（doc/07 §2.1 第 3 层：A/B 双网各 1 套 MAC IP）：
//   s_stream（组帧器帧体：目的 MAC..SN，无 FCS）→ TX 适配（IF_ID 字节替换 +
//   skid 缓冲）→ IP tx_axis（tdata/tvalid/tlast，IP 自加前导/SFD/FCS）
//   → IP eth_txd/en → GMII 出（核 gmii_*_d/en 观测口）
//   → 片内回环电缆（1 拍，`ifdef AFDX_MAC_LOOPBACK`，对应 T7 tb 的回环口）
//   → RTL mac_rx（T7 接收 MAC，收标准 GMII：前导+SFD+帧+FCS）
//   → rx_path（不变）。
// RX 方向不用 IP 的 rx_axis（该口数据/时钟域 rx_mac_aclk 源自 PHY 读到的速
// 率，纯仿真无外接 PHY 时不可控；T7 RTL mac_rx 已全量验收，其 GMII 口径与
// IP eth_txd 输出兼容），rx_mac_rst_n 拉低抑制、rx_axis_* 悬空。
// MDIO：纯仿真无外接 PHY——mdio 线挂仿真用虚拟 PHY 从机（Clause 22 读应答：
// BMCR=0x0140 1000M/全双工、BMSR=0x0020 链路有效），保证 IP 内部读得
// speed=2'b10/link=1（`ifdef AFDX_MAC_IP_SIM` 时随替身编译；真实硬件接外
// PHY 时去除替身与虚拟 PHY）。
//
// start/done 语义与 mac_tx 对齐：done = eth_tx_en 下降沿后再计 12 拍（mac_tx
// 的 IFG 计费口径，BAG 重启时刻逐拍一致）。锁步：A/B 两 IP 共享同一 gtx_clk
// （核 125MHz），同拍收到同一 s_stream，副本同时发出（0.5ms 时限）。
//
// 语言：Verilog-2001。IP 例化名 Tri_Mode_Ethernet_Mac（eLinx 加密源，仅
// eLinx 综合可读；`ifdef AFDX_MAC_IP_SIM` 时以行为替身编译 iverilog 功能
// 回归——替身 rtl/mac/mac_ip_behav.v，同端口同行为口径）。
//=============================================================================

`define AFDX_MAC_LOOPBACK 1

module mac_ip_top #(
    parameter [1:0] NET_ID      = 2'b01,    // 01=A 网 / 10=B 网（观测标注）
    parameter [7:0] IF_ID_BYTE  = 8'h20     // {IF_ID,5'b0}：A=0x20 / B=0x40
)(
    input  wire        clk,            // 125MHz 核时钟（=gtx_clk 域）
    input  wire        reset,          // 同步高有效（核域）
    input  wire        tick_us,        // （保留，mac_tx 兼容）

    // ---- mac_tx 兼容口（afdx_es_core 原 u_macA/u_macB 接线不变）----
    input  wire        start,          // 单拍：调度器启动（与对网同拍）
    input  wire [10:0] body_len,       // 帧体字节数（流自定界，观测用）
    input  wire        s_valid,
    output wire        s_ready,
    input  wire        s_sop,
    input  wire        s_eop,
    input  wire [7:0]  s_data,
    output reg         done,           // 单拍：帧发出 + IFG 计费毕（同 mac_tx）

    // ---- GMII 出（核观测口 gmii_*_d/en 直连；T6 纯仿真抓帧口径不变）----
    output wire [7:0]  gmii_d,
    output wire        gmii_en,
    output wire        gmii_er,
    output wire        frame_start,    // 单拍：EN 上升沿（tb 抓帧定位）
    output wire        frame_end,      // 单拍：EN 下降沿

    // ---- RX 侧事件（接收链事件由 rx_path 内 mac_rx 实例产生，恒 0）----
    output wire        ev_frame,
    output wire        ev_fcserr,
    output wire        ev_lenerr,
    output wire        ev_overflow,

    // ---- 回环 GMII 出（1 拍电缆后；core 在 IP 模式下接 gmii_rx_* 寄存器
    //      → rx_path 内 mac_rx，T8 集成口径见头注）----
    output wire [7:0]  lb_gmii_d,
    output wire        lb_gmii_dv,
    output wire        lb_gmii_er
);

    //-------------------------------------------------------------------------
    // TX 适配：s_stream → tx_axis
    //   - 字节 11（源 MAC 第 6 字节）按网替换 IF_ID_BYTE（与 mac_tx 同规则，
    //     A/B FCS 随之不同——IP 逐网重算 FCS）
    //   - skid 16×9b：吸收 IP tx_axis_mac_tready 的换缓冲空档（手册 §3.4.11.1：
    //     tlast 后判定期间 tready 拉低），tready 空档由 s_ready 反压组帧器
    //   - tlast = s_eop；tuser 恒 0（预留口）
    //-------------------------------------------------------------------------
    reg  [10:0] tcnt;                     // 已收字节计数（替换判定）
    reg  [3:0]  ifg_cnt;                  // done 计费（EN 落后 12 拍，同 mac_tx）
    reg  [1:0]  dstate;                   // 0=idle 1=run 2=ifg
    reg         en_d;
    reg  [7:0]  lb_d;
    reg         lb_dv, lb_er;

    wire       ax_tready;
    wire       f_we;
    wire [8:0] f_rdata;                   // {eop, data}
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
                2'd0: begin                          // IDLE
                    if (start) begin
                        tcnt   <= 11'd0;
                        dstate <= 2'd1;
                    end
                end
                2'd1: begin                          // RUN（流式送 IP）
                    if (f_we)
                        tcnt <= tcnt + 11'd1;
                    if (!en_d && gmii_en) ;          // EN 起（帧上线，观测）
                    if (!gmii_en && en_d) begin      // EN 落：FCS 已完，计 IFG
                        ifg_cnt <= 4'd0;
                        dstate  <= 2'd2;
                    end
                end
                2'd2: begin                          // IFG 计费（同 mac_tx 口径）
                    ifg_cnt <= ifg_cnt + 4'd1;
                    if (ifg_cnt == 4'd11) begin      // EN 落后第 12 拍
                        done   <= 1'b1;
                        dstate <= 2'd0;
                    end
                end
                default: dstate <= 2'd0;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 低有效复位链：sys_rst_n = ~reset；tx_mac_rst_n 延 4 拍释放
    //（手册 §3.1.1 顺序：sys → mdio/rx/tx）
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
    // 25MHz MDIO 时钟（125 五分频占空 40/60——MDIO 从机以 mdc 沿采样，容忍）
    //-------------------------------------------------------------------------
    reg [2:0] div25;
    reg       clk25;
    always @(posedge clk) begin
        if (reset) begin
            div25 <= 3'd0; clk25 <= 1'b0;
        end else if (div25 == 3'd4) begin
            div25 <= 3'd0; clk25 <= ~clk25;
        end else begin
            div25 <= div25 + 3'd1;
        end
    end

    //-------------------------------------------------------------------------
    // Tri_Mode_Ethernet_Mac IP 例化（每网 1 套，doc/07 §2.1 第 3 层）
    //   AFDX_MAC_IP_SIM：行为替身（iverilog 功能回归，同端口同行为口径）；
    //   缺省：eLinx 加密 IP（仅 eLinx 综合可读，.vo 联调走综合后仿真）。
    //-------------------------------------------------------------------------
    wire [7:0] eth_txd;
    wire       eth_tx_en, eth_tx_er, eth_tx_clk;
    wire [7:0] gmii_rxd;
    wire       gmii_rdv, gmii_rer;
    wire       rx_mac_aclk, tx_mac_aclk;
    wire       rx_axis_tvalid, rx_axis_tlast, rx_axis_tuser;
    wire [3:0] rx_axis_ftuser;
    wire       mdc;
    wire       mdio_io;
    wire       gtx_clk = clk;             // 1000M：gtx_clk = 125MHz（手册 §3.2）

    // IP 例化：iverilog 回归链接行为替身（rtl/mac/mac_ip_behav.v，同模块名）；
    // eLinx 综合链接加密 RTL（bin/ip/RTL/Tri_Mode_Ethernet_Mac_RTL）。
    Tri_Mode_Ethernet_Mac_0 u_ip (
        // 用户发送口（表 2.2）
        .gtx_clk             (gtx_clk),
        .tx_mac_rst_n        (tx_rst_n),
        .tx_mac_aclk         (tx_mac_aclk),
        .tx_axis_mac_tready  (ax_tready),
        .tx_axis_mac_tuser   (1'b0),
        .tx_axis_mac_tlast   (ax_tlast),
        .tx_axis_mac_tvalid  (ax_tvalid),
        .tx_axis_mac_tdata   (ax_tdata),
        // 用户接收口（表 2.3；本设计 RX 走 T7 RTL mac_rx：复位抑制）
        .rx_mac_rst_n        (1'b0),
        .rx_mac_aclk         (rx_mac_aclk),
        .rx_axis_mac_tuser   (rx_axis_tuser),
        .rx_axis_mac_tlast   (rx_axis_tlast),
        .rx_axis_mac_tvalid  (rx_axis_tvalid),
        .rx_axis_mac_tdata   (),
        // rx_axis_filter_tuser 口在 v3.0 IP 中不存在（Frame Filter 未启用）；
        // PHY 接收口（表 2.4；片内回环电缆馈入）
        .eth_rx_clk          (gtx_clk),
        .eth_rx_er           (gmii_rer),
        .eth_rx_dv           (gmii_rdv),
        .eth_rxd             (gmii_rxd),
        // col/crs/mii_tx_clk 口在 v3.0 IP 中不存在（GMII 全双工配置固化）
        // PHY 发送口（表 2.5）
        .eth_tx_clk          (eth_tx_clk),
        .eth_tx_er           (eth_tx_er),
        .eth_tx_en           (eth_tx_en),
        .eth_txd             (eth_txd),
        // MDIO（表 2.6；仿真挂虚拟 PHY）
        .mdio_rst_n          (rst_n),
        .mdio_clk            (clk25),
        .mdc                 (mdc),
        .mdio                (mdio_io),
        // 用户控制口（表 2.7；全双工/GMII 由 Customize IP 配置固化，
        // user_duplex_mode/user_port_mode 及 mac_filter_*/pause_*/sour_mac_addr
        // 口在 v3.0 IP 中不存在，原 0 值拉死语义由配置承载）
        .sys_rst_n           (rst_n),
        .link                (),
        .speed               (),
        .phy_addr            (5'd0),
        .user_wr_req         (1'b0),
        .user_wr_addr        (5'd0),
        .user_wr_data        (16'd0),
        .user_rd_req         (1'b0),
        .user_rd_addr        (5'd0),
        .user_rd_dv          (),
        .user_rd_data        ()
    );

    //-------------------------------------------------------------------------
    // 虚拟 PHY（Clause 22 MDIO 从机，rtl/mac/mdio_phy_sim.v）：读应答使 IP 内部
    // speed/link 判千兆/有效。纯仿真设计常驻（真实硬件时替换为外接 PHY，
    // mdc/mdio 引出到管脚）。
    //-------------------------------------------------------------------------
    mdio_phy_sim #(.BMCR_VAL(16'h0140), .BMSR_VAL(16'h0024)) u_vphy (
        .mdc(mdc), .mdio(mdio_io));

    //-------------------------------------------------------------------------
    // GMII 出 + 片内回环电缆（1 拍；T7 tb 回环同口径）
    //-------------------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            lb_d <= 8'h0; lb_dv <= 1'b0; lb_er <= 1'b0;
        end else begin
            lb_d  <= eth_txd;
            lb_dv <= eth_tx_en;
            lb_er <= eth_tx_er;
        end
    end

`ifdef AFDX_MAC_LOOPBACK
    assign gmii_rxd = lb_d;
    assign gmii_rdv = lb_dv;
    assign gmii_rer = lb_er;
`else
    assign gmii_rxd = 8'h0;                // 真实硬件：接 PHY RX（T8 后扩展）
    assign gmii_rdv = 1'b0;
    assign gmii_rer = 1'b0;
`endif

    // 回环电缆引出（core 的 IP 模式接 gmii_rx_* → rx_path；未定义回环时为 0）
    assign lb_gmii_d  = lb_d;
    assign lb_gmii_dv = lb_dv;
    assign lb_gmii_er = lb_er;

    assign gmii_d  = eth_txd;
    assign gmii_en = eth_tx_en;
    assign gmii_er = eth_tx_er;
    assign frame_start = gmii_en && !en_d;
    assign frame_end   = !gmii_en && en_d;

    // RX 侧事件恒 0（接收链事件在 rx_path 内部产生）
    assign ev_frame   = 1'b0;
    assign ev_fcserr  = 1'b0;
    assign ev_lenerr  = 1'b0;
    assign ev_overflow= 1'b0;

endmodule
