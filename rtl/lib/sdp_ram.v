//=============================================================================
// sdp_ram.v — 简单双口 RAM 推断模板（v2 §4.2 lib 基础件）
// 端口 A：同步写；端口 B：同步读（读延迟 1 拍）。
// 写读同地址当拍：读出旧值（write-before-read 无冒险要求，调用方自行避开）。
// Verilog-2001；eLinx 综合按双口 BRAM 推断。
//=============================================================================

module sdp_ram #(
    parameter integer WIDTH     = 8,    // 数据位宽
    parameter integer DEPTH     = 1024, // 条数（必须 ≤ 2^AW）
    parameter integer AW        = 10,   // 地址位宽（实例化方按 DEPTH 给定）
    parameter         INIT_EN   = 0,    // 1 = 用 INIT_FILE 初始化（仿真用）
    parameter         INIT_FILE = ""    // $readmemh 文件
)(
    input  wire             clk,      // 单时钟域（doc/02 §5：125 MHz）
    input  wire             we,
    input  wire [AW-1:0]    waddr,
    input  wire [WIDTH-1:0] wdata,
    input  wire [AW-1:0]    raddr,
    output reg  [WIDTH-1:0] rdata     // 1 拍读延迟
);

    reg [WIDTH-1:0] mem [0:DEPTH-1];

    // 仿真初始化（综合侧改走 eLinx MIF 流程，见 doc/04）
    initial begin
        if (INIT_EN) $readmemh(INIT_FILE, mem);
    end

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end

endmodule
