//=============================================================================
// rx_vl_lookup.v — 接收 VL 表（4096 项 × 2 字，cfg 0x10000~；v2 §4.3 ① / §4.4）
//
// 表项布局（T7 定案，RPT-T7-001 报备）：
//   W0 [15:0]  vlid 全 16 位 tag（4096 直接映射 + tag 全比对，v2 §4.2 rx_vl_lookup）
//      [16]    en        项使能（0 = 未配置，查表未命中）
//      [17]    ic_en     完整性检查使能
//      [18]    rm_en     冗余管理使能
//      [20:19] net_mask  bit0=A 网 bit1=B 网（与 mac_port_if NET_ID 01/10 对齐）
//      [31:21] 保留（写 0）
//   W1 [15:0]  skew_us   SkewMax（µs，RM 超窗判定 >skew 接受）
//      [26:16] smax[10:0] 应用载荷层级上限（0 = 不检查；口径同 doc/02 §8.2-3）
//      [31:27] 保留（写 0）
//
// 索引：vlid[11:0] 直接映射；tag = 完整 16 位 VLID，低 12 位相同高 4 位不同的
//       VLID 映射同一表项且互斥（后写覆盖前写），tag 不符按未命中处理——
//       4096 直接映射 + tag 的固有语义（v2 §4.2 rx_vl_lookup 原口径）。
//
// 写入：cfg_bus 0x10000 区广播（地址 = 0x10000 + idx*2 + w，bit0 = W0/W1 选择），
//       上电 ram_init sel=7 两块 RAM 并行清零（en=0 默认全部未配置）。
// 读回：cfg 读与内核查表共用 RAM 读口（内核 kr_re 优先——查表拍 cfg 读回值
//       无效，tb/管理面仅在无帧在途时读回，与 tx_vl_table tbl_busy 同口径）。
// 内核查表：kr_re 拍给完整 VLID（RAM 同沿采样），其后 2 拍 kr_vld=1 且各
//       字段有效（BRAM 同步读 1 拍 + 译码寄存 1 拍）。125 MHz 下关键路径 =
//       BRAM 读出 + 16b 比较 + 字段散出，单拍完成，裕量充足（tb_rxlookup 实测）。
//=============================================================================

module rx_vl_lookup #(
    parameter integer RX_VL_NUM = 4096
)(
    input  wire        clk,
    input  wire        reset,

    // ---- cfg 写广播（0x10000 区）----
    input  wire        rxcfg_we,
    input  wire [10:0] rxcfg_idx,         // 表索引 0..4095（cfg 地址 = 0x10000+idx*2+w）
    input  wire        rxcfg_wsel,        // W0/W1 字选（cfg 地址 bit0）
    input  wire [31:0] rxcfg_wdata,

    // ---- cfg 读回（与内核查表分时共用读口）----
    input  wire        rxcfg_re,
    input  wire [10:0] rxcfg_ridx,
    input  wire        rxcfg_rsel,
    output wire [31:0] rxcfg_rdata,

    // ---- 上电清零（ram_init sel=7：两块 RAM 并行写 0）----
    input  wire        ri_we,
    input  wire [11:0] ri_addr,

    // ---- 内核查表（kr_re 拍给 vlid[15:0]，下一拍结果有效）----
    input  wire        kr_re,
    input  wire [15:0] kr_vlid_in,       // 查表键：完整 16 位 VLID（索引取低 12 位）
    output reg         kr_vld,
    output reg         kr_hit,
    output reg         kr_ic_en,
    output reg         kr_rm_en,
    output reg  [1:0]  kr_mask,
    output reg  [10:0] kr_smax,
    output reg  [15:0] kr_skew
);

    //-------------------------------------------------------------------------
    // 双 RAM：W0/W1 各 4096×32b（同地址并行读出整项）
    //-------------------------------------------------------------------------
    wire [11:0] waddr  = rxcfg_we ? {1'b0, rxcfg_idx} : ri_addr;
    wire        we0    = rxcfg_we ? !rxcfg_wsel : ri_we;     // wsel=0 → W0
    wire        we1    = rxcfg_we ?  rxcfg_wsel : ri_we;     // wsel=1 → W1
    wire [31:0] wdata  = rxcfg_wdata;

    // 读口：内核优先（kr_re 拍 cfg 读让位——该拍读回值无效，调用方避让）
    wire [11:0] raddr  = kr_re ? kr_vlid_in[11:0] : {1'b0, rxcfg_ridx};
    wire [31:0] w0_r, w1_r;

    sdp_ram #(.WIDTH(32), .DEPTH(RX_VL_NUM), .AW(12)) u_w0 (
        .clk(clk), .we(we0), .waddr(waddr), .wdata(we0 ? wdata : 32'h0),
        .raddr(raddr), .rdata(w0_r));
    sdp_ram #(.WIDTH(32), .DEPTH(RX_VL_NUM), .AW(12)) u_w1 (
        .clk(clk), .we(we1), .waddr(waddr), .wdata(we1 ? wdata : 32'h0),
        .raddr(raddr), .rdata(w1_r));

    // cfg 读回多路（rsel 选字；内核查表拍数据无关）
    assign rxcfg_rdata = rxcfg_rsel ? w1_r : w0_r;

    //-------------------------------------------------------------------------
    // 内核译码（两级）：t0 kr_re 拍发地址（RAM 同沿采样）→ t1 RAM 数据到 +
    // 译码寄存 + kr_vld。查表延迟 = kr_re 后 2 拍出全部字段
    // （BRAM 同步读 1 拍是下限；引擎在帧头读期间重叠查表，无吞吐影响）。
    //-------------------------------------------------------------------------
    reg [15:0] vlid_q;                        // tag 比对用（锁存 kr_re 拍完整 VLID）
    reg        kr_re_d;
    always @(posedge clk) begin
        if (kr_re) vlid_q <= kr_vlid_in;
        kr_re_d <= kr_re;
    end

    always @(posedge clk) begin
        if (reset) begin
            kr_vld   <= 1'b0;
            kr_hit   <= 1'b0;
            kr_ic_en <= 1'b0;  kr_rm_en <= 1'b0;
            kr_mask  <= 2'b00; kr_smax  <= 11'h0; kr_skew <= 16'h0;
        end else begin
            kr_vld   <= kr_re_d;              // 本拍 w0_r/w1_r 即 kr_re 拍地址的数据
            kr_hit   <= w0_r[16] && (w0_r[15:0] == vlid_q);   // en + tag 全比对
            kr_ic_en <= w0_r[17];
            kr_rm_en <= w0_r[18];
            kr_mask  <= w0_r[20:19];
            kr_smax  <= w1_r[26:16];
            kr_skew  <= w1_r[15:0];
        end
    end

endmodule
