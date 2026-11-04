//=============================================================================
// AFDX 端系统顶层（竞赛指定模板端口名，方向按官方答疑口径 doc/05）
//-----------------------------------------------------------------------------
// 语义（doc/05 官方答疑 10.19-10.30）：
//   本模块 = 端系统对外的【应用数据接口】。
//   发送：应用侧在 tx_data 上注入 "VLID(前2字节,大端) + 应用数据"，
//         tx_valid 上升沿→下降沿为一条消息（可超 1471B，内部做 IP 分片）。
//   接收：rx_data 输出处理完毕的应用数据（含 VLID 2 字节前缀），rx_valid 标记。
//   完整以太网帧只存在于内部（afdx_es_core 与 GMII 侧），不出现在顶层。
//   ready 方向（VERDICT-002-1）：保留指南模板方向——tx_ready 为 output（ES 恒 1，
//         表示可接收应用数据）；rx_ready 为 input（应用侧就绪，设计中可不使用）。
//         官方明确 ready 可不用、评测不查（doc/05 09.24-10.26）。
//   接收交付与输入信封对称：rx_data = VLID（前 2 字节）+ 处理完毕的应用载荷。
// 内部分层：AFDX_top（本外壳）→ afdx_es_core（协议核心）+ mgmt（管理平面，P1）
// 端口名/顺序与选题指南模板逐字一致；数据/valid 方向按 doc/05：tx_* 入、rx_* 出。
//   - 原始 payload 吞吐 100 Mbps（GMII 千兆链路，MAC IP ×2，A/B 双网）
// 系统配置（VL 表/全局寄存器）：VERDICT-015 交付口径经顶层 cfg_* 引脚完成
//   （cfg_bus 地址地图：全局 0x0xx / TX 表 0x01000 区 / RX 表 0x10000 区；
//   官方口径"能配置、配置方式不限"）。T6/T7 时期为 tb 层级引用驱动核心内
//   output reg——纯综合下零驱动致网表空心（RPT-T8-007/008 定案），已清除。
// GMII 观测口（T6 抓帧）仍为层级引用只读采样，不在顶层增口。
//=============================================================================

module AFDX_top (
    input  wire        clk,        // 时钟信号（频率与接口速率匹配，见 doc/02 v1 时钟方案）
    input  wire        reset,      // 复位信号（同步，高有效）
    input  wire [7:0]  tx_data,    // 应用发送数据（VLID 2B 前缀 + 载荷，字节流）
    output wire [7:0]  rx_data,    // 应用接收数据（VLID 2B 前缀 + 载荷，与输入信封对称）
    input  wire        tx_valid,   // 应用发送数据有效指示（上升沿→下降沿 = 一条消息）
    output wire        rx_valid,   // 应用接收数据有效指示（语义同 tx_valid）
    output wire        tx_ready,   // ES 发送通道就绪（恒 1；官方：取值不做要求）
    input  wire        rx_ready,   // 应用侧接收就绪（官方：可不使用、不连线亦可）
    // ---- cfg 主机配置接口（VERDICT-015 交付口径增补；官方"能配置、配置方式不限"）----
    input  wire [19:0] cfg_addr,   // 配置地址（0x0xx 全局 / 0x01000 区 TX 表 / 0x10000 区 RX 表）
    input  wire [31:0] cfg_wdata,  // 配置写数据
    input  wire        cfg_we,     // 配置写使能（cfg_bus 同步采样，1 拍一写）
    input  wire        cfg_re,     // 配置读使能
    output wire [31:0] cfg_rdata,  // 配置读回数据
    output wire        cfg_rvalid  // 配置读有效
);

    // 应用口直连核心（信封解析器在核心内）
    wire [7:0] core_tx_data = tx_data;
    wire       core_tx_valid = tx_valid;

    // cfg 总线主机侧：本层引脚直通核心（VERDICT-015 交付口径）
    wire [19:0] cfg_addr_w   = cfg_addr;
    wire [31:0] cfg_wdata_w  = cfg_wdata;
    wire        cfg_we_w     = cfg_we;
    wire        cfg_re_w     = cfg_re;
    wire [31:0] cfg_rdata_w;
    wire        cfg_rvalid_w;
    assign cfg_rdata  = cfg_rdata_w;
    assign cfg_rvalid = cfg_rvalid_w;

    // GMII 观测（核心内 mac_port_if 输出；T6 tb 层级引用采样）
    wire [7:0]  gmii_a_d, gmii_b_d;
    wire        gmii_a_en, gmii_b_en;

    afdx_es_core #(
        .TX_VL_NUM(128), .RX_VL_NUM(4096),
        .TX_SLOTS(32), .SLOT_BYTES(2048), .Q_DEPTH(4), .MAX_MSG(8192)
    ) u_core (
        .clk      (clk),
        .reset    (reset),
        .tx_data  (core_tx_data),
        .tx_valid (core_tx_valid),
        .rx_data  (rx_data),
        .rx_valid (rx_valid),
        .tx_ready (tx_ready),
        .rx_ready (rx_ready),
        .cfg_addr (cfg_addr_w),
        .cfg_wdata(cfg_wdata_w),
        .cfg_we   (cfg_we_w),
        .cfg_re   (cfg_re_w),
        .cfg_rdata(cfg_rdata_w),
        .cfg_rvalid(cfg_rvalid_w),
        .gmii_a_d (gmii_a_d),
        .gmii_a_en(gmii_a_en),
        .gmii_b_d (gmii_b_d),
        .gmii_b_en(gmii_b_en)
    );

    //-------------------------------------------------------------------------
    // 管理平面：SNMP / 615A / RTC（P1，最小可演示子集）
    //-------------------------------------------------------------------------
    // TODO(P1): mgmt 例化（届时由管理主机经本层 cfg_* 引脚完成同一配置面）

endmodule
