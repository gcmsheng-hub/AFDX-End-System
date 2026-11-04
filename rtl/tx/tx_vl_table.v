//=============================================================================
// tx_vl_table.v — 128 项发送 VL 配置表（v2 §4.2 tx；地址区 0x01000~0x013FF）
//
// 表项 8 字布局（T6 定案，见 doc/02 §8.2 增补；0x01000 + VL*8 + W）：
//   W0 {en, 保留[22:8], bag_ms[7:0]}   BAG 毫秒值 ∈ {1,2,4,8,16,32,64,128}
//   W1 Smax[15:0]                      应用载荷层级（每帧 UDP 载荷字节上限）
//   W2 VLID[15:0]                      完整 VLID（帧头取此字段；与信封一致性由配置者保证）
//   W3 {保留[31:2], net_mask[1:0]}     bit0=A 网 bit1=B 网（冗余/单网）
//   W4 phase_us[15:0]                  BAG 相位偏移（复位/使能时装入，µs）
//   W5 sport[15:0]                     UDP 源端口（端口按 VLID 所在表项索引）
//   W6 dport[15:0]                     UDP 目的端口
//   W7 保留
//
// RAM 口分配：sdp 口 A = cfg 写；唯一读口分时复用——默认给内核读（组帧器连读
// 8 字），cfg 读回（tbl_re 单拍）仅在 tbl_busy=0 时切入。tbl_busy=1 期间 cfg 读回
// 得到的是内核读地址的数据（读回自检让位于业务），内核读永不被干扰。
// 上电清零由 ram_init 经 ri_* 口写入（we/waddr/wdata 由核级多选）。
//=============================================================================

module tx_vl_table #(
    parameter integer TX_VL_NUM = 128
)(
    input  wire        clk,

    // ---- cfg 写广播（来自 cfg_bus，0x01000 区写）----
    input  wire        cw_we,
    input  wire [6:0]  cw_idx,
    input  wire [2:0]  cw_wsel,
    input  wire [31:0] cw_wdata,

    // ---- cfg 读回（单拍读地址切入）----
    input  wire        tbl_re,
    input  wire [9:0]  tbl_raddr,          // {vl[6:0], wsel[2:0]}
    input  wire        tbl_busy,           // 内核读占用读口
    output wire [31:0] tbl_rdata,          // 1 拍延迟（busy 期间值无意义）

    // ---- 内核读口（组帧器；tbl_busy=1 期间地址保持）----
    input  wire [6:0]  rd_idx,
    input  wire [2:0]  rd_wsel,
    output wire [31:0] rd_data,            // 1 拍延迟

    // ---- 上电清零（ram_init；we/waddr 数据恒 0 由核级多选接入）----
    input  wire        ri_we,
    input  wire [9:0]  ri_addr
);

    wire [9:0] ra = (tbl_re && !tbl_busy) ? tbl_raddr : {rd_idx, rd_wsel};

    sdp_ram #(
        .WIDTH(32), .DEPTH(TX_VL_NUM * 8), .AW(10)
    ) u_tbl (
        .clk   (clk),
        .we    (cw_we || ri_we),
        .waddr (ri_we ? ri_addr : {cw_idx, cw_wsel}),
        .wdata (ri_we ? 32'h0   : cw_wdata),
        .raddr (ra),
        .rdata (tbl_rdata)
    );

    assign rd_data = tbl_rdata;   // 内核读与读回共用读出寄存器（地址由 ra 分时）

endmodule
